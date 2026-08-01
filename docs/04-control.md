# 4. Control

Perception said where to go. Control decides what to do with the wheels and
the throttle. Two independent laws, supervised by a small state machine.

## Steering: a PD controller

### The error

First, express "how wrong are we pointing" as a single number in `[-1;1]`:

```c
error = target_angle / (FOV / 2)
```

The gap is at −15°, the half-FOV is 30°, so the error is −0.5. Dead ahead is
0. The far left edge of the cone is −1.

Normalising by the half field of view rather than using degrees directly means
the gains have no units and stay meaningful if the sensor changes. Swap in a
90° lidar and the tuning still holds.

### Proportional alone is not enough

The obvious law is: turn in proportion to the error.

```c
wheels = Kp * error;
```

Try it. The car aims at the gap, turns towards it, and — because it is still
turning when it gets there — sails past. Now the error is on the other side,
so it turns back, and overshoots again. Down a straight it weaves. In a corner
it can oscillate hard enough to lose the back end.

The problem is that `Kp * error` only knows where the car is pointing. It has
no idea how fast that is changing, so it keeps steering hard right up to the
moment the error hits zero — by which time the car has too much rotational
momentum to stop there.

### Adding the derivative

```c
derivative = error - last_error;
wheels = Kp * error + Kd * derivative;
```

The derivative term measures how fast the error is shrinking. When the car is
swinging quickly towards the target, `derivative` has the opposite sign to
`error`, and the term subtracts from the command — it eases off *before*
arriving. That is the whole trick, and it is what damps the weave.

Intuitively: **P is how far off you are, D is how fast you are fixing it.**

Note there is no division by `dt`. One cycle is the time unit, which is only
legitimate because the loop period is constant by construction — see
[architecture](02-architecture.md#the-cycle). If the loop rate varied, `dt`
would have to be measured, and a jittery `dt` in a derivative term produces
jittery steering.

### Why there is no I term

The textbook controller is PID. The integral term accumulates error over time
and exists to eliminate a constant bias — a steady crosswind, a wheel
alignment that pulls left.

We do not have one. The steady-state error here is not constant; it is
whatever the next corner brings. An integrator would accumulate through every
corner and dump that accumulation into the following straight. Worse, the
actuator is clamped and rate limited, so the integrator winds up while the
wheels are already at full lock, and the car takes several seconds to respond
to anything at all afterwards.

**PD, deliberately.** Adding I to a controller whose error is not biased makes
it worse, and the failure mode looks like a car that has stopped listening.

### The centring term

Follow-the-gap will happily shave a wall it is not aimed at. It optimises for
the opening, not for the corridor.

```c
centring = (right_mm - left_mm) / (right_mm + left_mm)
```

Positive when there is more room on the right, i.e. when the car is drifting
towards the left wall. Added with a small weight (`--balance`, 0.12 by
default), it keeps the car roughly centred without arguing with the gap
target. Turn it up and the car refuses to take the inside of a corner; turn it
to zero and it clips apexes it should not.

### Rate limiting

```c
wheels = rate_limit(wheels, last_wheels, cfg->steer_rate_limit);
```

The command may not change by more than `--steer-rate` (0.25) in one cycle, so
going from straight to full lock takes four cycles minimum.

Real steering racks have a slew rate. Simulated ones do not, and a controller
that snaps from −1 to +1 between two cycles unsettles the car enough for the
physics engine to spin it. This is the cheapest possible model of a physical
constraint, and it removes a whole class of spectacular failures.

### Clamping

`WHEELS_DIR` must be in `[-1;1]`. Out-of-range values are not saturated by the
API — they are **rejected**, and the actuator keeps its previous value.

This is worth dwelling on, because it inverts the usual intuition. An
unclamped controller with too much gain does not steer too hard. It stops
steering *at all*, silently, at exactly the moments when the error is largest
— which is to say, in corners. Everything that reaches the wire goes through a
clamp, twice: once in the control law, once in
[`commands.c`](../src/protocol/commands.c).

## Throttle

```c
power = map_range(forward_mm, dist_slow, dist_fast, throttle_min, throttle_max);
power *= 1.0 - turn_penalty * fabs(wheels);
if (forward_mm < brake_dist) power = 0.0;
```

Three ideas.

**Distance sets the base power.** A linear ramp: at `--dist-slow` (600 mm) the
car creeps at `--throttle-min`; at `--dist-fast` (2600 mm) and beyond it runs
at full power. Between the two it interpolates.

`map_range` clamps at both ends. This is not defensive tidiness — it is the
fix for a specific bug. The unclamped formula

```c
(x - a) / (b - a) * (d - c) + c
```

with a=10, b=50, c=0, d=0.1 and a genuine distance of 500 mm returns **1.225**.
That is outside `[0;1]`, so the API rejects it and the car does not accelerate,
with nothing anywhere saying why. There is a regression test on that exact
arithmetic ([`test_map_range_is_clamped`](../tests/test_util.c)).

**Steering costs power.** `turn_penalty` (0.55) means full lock gives up 55% of
the throttle. Braking and cornering compete for the same grip; asking for both
at once puts the car sideways. Trade one for the other explicitly.

**Below `--brake-dist`, nothing.** No amount of clear space on the sides
justifies driving into the wall in front.

### On tuning for speed

The evaluation rewards fast laps, and the temptation is to raise
`--throttle-max`. That is usually the wrong knob. The car is rarely limited by
its top speed — it is limited by how late it can brake and how well it holds a
line. In rough order of return:

1. `--dist-fast` **down** — reach full power sooner out of corners
2. `--turn-penalty` **down** — carry more speed through them
3. `--kd` **up** — a steadier line lets everything else be more aggressive
4. `--throttle-max` up — last, and only once the car is stable

See [tuning](06-tuning.md) for the method.

## The state machine

```
   RACING ──── no progress for blind_cycles ──────▶ ESCAPE
      ▲          or forward < reverse_dist            │
      └── end of a forward phase, gap found,          │
      │   forward > reverse_dist × 1.8 ───────────────┘
      │                                               │
      │                          escape_max_pairs and still stuck
      └── deadend_attempts recoveries ─▶ HALTING ◀────┘
                                            │
                                            ▼
                                        FINISHED
```

**`RACING`** runs the two laws above, under one governing rule: **the car is
never idle**. Every cycle it either commands forward motion or is recovering.

That rule came from a real failure. An earlier version cut the throttle below
`brake_dist` and started reversing below `reverse_dist`, which leaves a band
between the two where the car does neither. On a real track it parked itself
in that band, 300mm from a wall, and sat there:

```
[00021] RACING  fwd=300 near=283 gap= no  wheels=+0.005 thr=0.000 rev=0.000
[00022] RACING  fwd=297 near=283 gap= no  wheels=+0.005 thr=0.000 rev=0.000
...                                (twenty identical cycles)
[00040] RACING  fwd=300 near=283 gap= no  wheels=+0.006 thr=0.000 rev=0.000
```

Nothing in that telemetry says what is wrong, because nothing *is* wrong by
any single rule — it is the gap between two rules. So progress is now measured
rather than inferred: any cycle that does not command forward motion counts as
stuck, whatever the reason, and `blind_cycles` of them triggers a recovery.

Two behaviours follow from the same rule:

**Never drive without a gap.** If the gap search found nothing, the throttle
is zero — not `throttle_min`, not "creep and see". Driving into space the
perception layer failed to make sense of is how a recoverable situation
becomes a collision.

**Crawl rather than stop.** If a gap *is* visible but the wall is closer than
`brake_dist`, the car applies `crawl_power` instead of nothing. A stationary
car with its wheels turned goes nowhere: cutting the throttle completely near
a wall prevents the exact manoeuvre needed to get around it. This is the
difference between taking a tight corner and stopping in front of it.

### The escape manoeuvre

**`ESCAPE`** is the interesting one, and it exists because the obvious thing
does not work.

The obvious thing is to reverse. The previous version did, and steered while
reversing by aiming at the gap — except that when the car is stuck there
usually *is* no gap, `heading_error()` returns zero by construction, and the
steering command is zero. Measured across the real runs: **128 of 294
reversing cycles applied exactly zero steering.**

Reversing in a straight line is geometrically useless here. It translates the
car without rotating it, so when it drives forward again it faces the same
wall from slightly further away. The logs show precisely that: reverse two
cycles, drive four, reverse two, drive four, indefinitely. The car was not
escaping, it was pacing.

A car with Ackermann steering cannot turn on the spot. The only way to change
its heading is a three-point turn:

```
   phase A: back up, wheels turned one way    -> rear goes that way,
                                                 nose goes the other

   phase B: pull forward, wheels the other way -> nose continues the same way

   repeat
```

Both phases rotate the car in the *same* direction, which is why the steering
is opposite between them. Getting that sign backwards produces a car that
carefully undoes its own turn, and it looks like a perception bug.

Four rules make it work:

**Commit to a direction.** Chosen once at entry, from the gap if there is one
and otherwise from the more open side of the cone, then held for the whole
manoeuvre. Re-deciding every cycle reproduces exactly the dithering the
manoeuvre exists to cure.

**Full lock, both phases.** The rate limiter is bypassed here. It exists to
stop the car being unsettled at speed; during the manoeuvre it is moving at
crawl or reverse power, and spending four of six phase cycles ramping up to
lock would waste most of the rotation.

**The forward phase still watches the wall.** Crawling forward with the wheels
on full lock is still crawling forward. Before this check existed, five
forward cycles took the clearance from 140mm to 110mm — touching the very wall
the manoeuvre was escaping. It now cuts the phase short and goes back to
reversing.

**Leave only from a forward phase, with somewhere to go.** Both conditions.
Leaving mid-reverse abandons the rotation half done; leaving without a gap
drives straight back into the wall.

The honest result: the manoeuvre fixed the mechanism — **zero straight-line
reversing cycles across the runs since** — and it makes the car persist
properly instead of pacing. It did not, on its own, unlock any checkpoint the
old version could not reach. What it buys is that the car always terminates,
always tries a real escape, and never sits grinding against a wall.

**`HALTING`** ends the run in a dead end, and it is fiddlier than it sounds.
The requirement is to stop at least three feet from the wall without touching
it, so halting is not "set throttle to zero". Two real failures shaped it:

*Backing up to exactly the legal distance never settles.* The car reverses to
1000mm, stops reversing, coasts forward a few centimetres, drops below the
line, reverses again. Observed: eighty cycles of shuffling between 950mm and
1150mm. Fixed by backing up to a 25% margin, so ordinary coasting stays legal.

*And the margin has to be latched.* Reaching it once is enough; re-checking
the clearance every cycle re-creates the same loop one level up, because there
is no brake — `CAR_FORWARD:0` stops driving, it does not hold the car still,
so the car always rolls forward again. Measured on a real track: sixty cycles
oscillating between 900mm and 1275mm, ended only by the ceiling below. The
margin exists to absorb that roll; once earned, the car stops and settles.

*The margin is not always reachable.* There is no rear sensor. The car
reverses blind, and in a corner it wedges against a wall it cannot see, with
the forward distance stuck short of the margin forever:

```
[00403] HALTING  fwd=1125 near=1050 rev=0.350
[00404] HALTING  fwd=1125 near=1125 rev=0.350
[00405] HALTING  fwd=1125 near=1050 rev=0.350    (250 more of these)
```

So halting also watches whether reversing is still buying clearance. Once it
stops gaining ground, the car settles for what it has, provided that is legal.
Insisting on the ideal outcome here means never terminating at all.

**Recoveries, not cycles.** A dead end is declared after
`--deadend-attempts` reverse-and-retry cycles that never lead anywhere — not
after a fixed number of gapless scans. And a clean stretch of racing
(`--recovery-reset` cycles) clears the counter, because a track with several
tight corners should not accumulate its way into a false dead end.

**`FINISHED`** is also entered directly, from any state, when `Track Cleared`
arrives or the lap limit is reached — those are noticed by the race loop,
which owns the protocol, not by the controller.

Next: [5. Simulator setup](05-simulator-setup.md)
