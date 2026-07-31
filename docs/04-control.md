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
   RACING ──── forward < reverse_dist ────▶ REVERSING
      ▲                                        │
      └──── forward > reverse_dist × 1.8 ──────┘
      │
      └─ no gap for deadend_cycles ─▶ HALTING ─▶ FINISHED
```

**`RACING`** runs the two laws above.

**`REVERSING`** backs out of trouble. One counter-intuitive line:

```c
out->wheels = -heading_error(gap);
```

The steering is *inverted* while reversing. Backing up with the wheels turned
left swings the nose to the right, so to end up pointing at an opening on the
left, the wheels go right. Getting this sign wrong produces a car that
reverses itself deeper into the corner it was trying to escape — and it looks
like a perception bug, which is where you will waste the afternoon.

The exit threshold is 1.8× the entry threshold. Without that hysteresis the
car shuffles forwards and backwards on the boundary indefinitely.

**`HALTING`** ends the run in a dead end. The requirement is to stop at least
three feet from the wall without touching it, so halting is not just "set
throttle to zero" — the car keeps backing up until it has a metre of
clearance, then holds still for five cycles to confirm it has actually
stopped, and only then declares itself finished.

**Persistence before declaring a dead end.** One scan with no gap is noise —
a reflection, a glitch, a moment where the bubble covered everything.
`--deadend-cycles` (12) consecutive gapless cycles is a wall. Reacting to the
first one ends races in the middle of perfectly drivable corners.

**`FINISHED`** is also entered directly, from any state, when `Track Cleared`
arrives or the lap limit is reached — those are noticed by the race loop,
which owns the protocol, not by the controller.

Next: [5. Simulator setup](05-simulator-setup.md)
