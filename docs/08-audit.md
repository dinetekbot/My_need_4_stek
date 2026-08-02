# 8. Audit: why this car is not fast, and what would fix it

Written after seeing smoother, quicker runs of the same project elsewhere.
Everything below is measured on this machine, this simulator and these scenes.
No claim is made about anyone else's implementation — the numbers describe
*this* one, and they are enough to explain the gap.

## The algorithm, in one page

Each control cycle:

1. **Read** the lidar: 32 distances, 60° cone, millimetres, saturated at 3010.
2. **Clean**: interpolate isolated dropouts, collapse runs of them to "wall
   here", clamp, 3-ray moving average.
3. **Bubble**: find the closest obstacle, erase every ray within an angular
   radius of it. Radius = `atan2(bubble_mm, distance)`, capped at a quarter of
   the field of view.
4. **Gap**: longest run of rays reaching past `gap_threshold`; aim at the
   midpoint of the deepest plateau inside that run.
5. **Steer**: PD on the heading error, plus a small left/right centring term,
   clamped and rate limited.
6. **Throttle**: linear ramp from the forward distance, cut in proportion to
   steering effort; crawl below the braking distance.
7. **Supervise**: a state machine — race, three-point-turn escape, halt.

It is a reactive follow-the-gap controller. No map, no memory, no plan.

## The economics nobody mentions

Every command costs **simulated time**, and simulated time is distance driven
blind. Measured, 60 commands of each kind:

| Command | Simulated time each |
|---|---|
| `GET_INFO_LIDAR` | 0.155 s |
| `CAR_FORWARD` | 0.102 s |
| `WHEELS_DIR` | 0.102 s |
| `GET_CURRENT_SPEED` | 0.102 s |

Wall clock per command is ~0.135 s, and the simulator advances in real time
while it waits, so the cost is latency inside the API, not computation. It
does not matter how clever the code is: **the AI's budget is denominated in
commands.**

Cycle shapes, measured directly:

| Cycle | Simulated time per decision |
|---|---|
| lidar + wheels + forward + backwards (current) | **0.464 s** |
| lidar + wheels + forward | 0.360 s |
| lidar + forward | 0.262 s |
| lidar alone | 0.159 s |

At a typical racing speed the car covers roughly a metre in half a second.
So the current design drives about a metre between two decisions.

---

## Finding 1 — a third of every command budget is wasted

`main.c` sends all four commands every cycle, unconditionally. The stated
reason, in this repository's own documentation, was that a constant number of
round trips gives a constant loop period, which lets the derivative term of
the steering controller be a plain difference instead of a division by a
measured timestep.

**That trade was wrong, and it was wrong by a wide margin.** It buys tidiness
in one term of one controller and pays for it in the scarcest resource the
system has.

Measured over a real cleared run of 45 cycles:

| Command | Cycles where the value actually changed |
|---|---|
| `WHEELS_DIR` | 43 (96%) |
| `CAR_FORWARD` | 28 (62%) |
| `CAR_BACKWARDS` | 1 (2%) |

180 commands sent, 117 of them carrying new information: **35% waste**, almost
all of it `CAR_BACKWARDS:0.000` repeated into a car that is already not
reversing.

Fixing this looked like arithmetic: skip an actuator command when the value
has not moved by more than a small deadband. Expected gain, from the table
above, 0.464 → ~0.33 s per decision.

**It was tried, and it made the car strictly worse: 0 cleared runs out of 8,
against 8 out of 8 without it.** Same scene, same binary otherwise,
alternating runs.

The measurement of waste is still correct. The inference that removing it is
free was not. Every distance threshold, and the startup pace calibration
itself, is calibrated against a cycle that costs four commands; make the cycle
cheaper and the whole tuning is wrong at once, in the conservative direction
— the car brakes far too early, crawls, trips the stuck detector and spends
its life in the escape manoeuvre.

The lesson is not "do not spend fewer commands". It is that the command budget
and the distance tuning are one parameter, not two, and changing either alone
invalidates the other. A real attempt has to re-derive the thresholds from the
new cycle cost, and be measured over enough runs to see it.

## Finding 2 — the heading target is discontinuous, and that is the jerkiness

The sensor resolves 1.9° per ray. If the target moved smoothly you would
expect changes of that order between consecutive cycles. Measured over two
cleared runs:

| | run A | run B |
|---|---|---|
| mean change in target angle per cycle | **8.3°** | **9.8°** |
| largest change | 24.4° | 26.3° |
| cycles changing by more than 10° | 34% | 45% |
| **cycles where the steering reverses sign** | **53%** | **44%** |

The car saws at the wheel on roughly every other cycle. Jumps of 8–10° are
four to five times the sensor quantum, so this is not resolution — it is the
gap search itself switching its mind.

Three mechanisms produce it, and they compound:

**The target is an index, not an angle.** `plateau_centre` returns the middle
ray of the deepest plateau. As the car advances, which contiguous run is
"longest" changes discretely: a wall edge crosses a ray boundary and the whole
target teleports to a different opening.

**The bubble moves discontinuously too.** It is centred on the single closest
ray. When two obstacles are nearly equidistant, the closest index flips
between them and the erased sector jumps with it.

**Thresholding destroys gradient.** `gap_threshold` turns a smooth distance
field into a binary open/closed mask. Everything downstream sees a staircase.

A controller cannot be smooth when its setpoint is not. The PD gains and the
rate limiter are currently doing damage control on a signal that should not
have been jumpy in the first place, and the rate limiter in particular is
hiding the symptom while adding lag.

## Finding 3 — the throttle is open loop and has no idea what is coming

`throttle_command` is a function of the distance straight ahead, reduced by
how hard the wheels are turned *right now*. Two consequences:

- **No speed feedback.** `GET_CURRENT_SPEED` is read once at startup and never
  again. The car does not know how fast it is going, so "brake below 600mm"
  means something different at 10 and at 45.
- **No anticipation.** It slows for the wall in front, not for the corner it
  is about to enter. A human driver brakes *before* turning; this one brakes
  because it is already turning, which is exactly backwards.

## Finding 4 — the tuning is implicitly tied to a cycle period

Every distance in the configuration is a claim about how far the car travels
between decisions, and that depends on how many commands the cycle sends and
how fast the simulator answers. When the windowed simulator halved the
decision rate, every threshold became wrong at once, and the car drove into
the first corner.

The startup pace calibration added in response is a workaround, not a fix. The
principled version is to express the thresholds in *distance the car will
cover before it can react* — `speed x cycle_period` — and derive them, rather
than scale hand-written millimetres by a fudge factor.

## Finding 5 — 60° of vision and no memory

The car cannot see the wall it is scraping if that wall is beside it, and it
forgets everything between cycles. Follow-the-gap is the right algorithm for
that, and it is also why the car drives down corridor centres rather than
along a fast line. Measured on `criss_cross`, it passes within 60mm of walls
it never saw.

This is the ceiling on the whole approach, and it is the last thing to fix,
not the first.

---

## What to do, in order

Ranked by measured benefit against effort. The first two are where the
"fluid and quick" difference almost certainly lives.

### 1. Make the target angle continuous — a few hours

Replace "midpoint of the deepest plateau" with a quantity that varies
smoothly as the car moves:

```
target = Σ w(r_i) · θ_i  /  Σ w(r_i)
```

a weighted centroid of the free directions, with `w` a smooth function of
range (for instance `max(0, r − r_min)²`) rather than a hard threshold. Then
low-pass the result across cycles. Expected: the 8–10° jumps collapse towards
the sensor quantum, the sign reversals largely disappear, and the rate limiter
stops being load-bearing.

Keep the bubble for safety, but soften it: attenuate rays near the closest
obstacle instead of zeroing them, so the erased sector cannot teleport.

### 2. Spend the command budget only on information — and re-tune with it

Send `WHEELS_DIR`, `CAR_FORWARD`, `CAR_BACKWARDS` only when the value has
moved past a deadband. ~30% more decisions per metre.

**Not a free win, measured.** Done on its own it took the car from 8 cleared
runs out of 8 to 0 out of 8, because every distance threshold is implicitly
calibrated against the cost of a cycle. The braking distances have to be
re-derived from the new, cheaper cycle at the same time — and the result
measured over eight or more runs, not two.

### 3. Anticipate the corner instead of reacting to the wall — a day

Compute the curvature the car is about to commit to (from the target angle and
the depth of the gap), and set the speed from *that*, ahead of the turn.
Combine with a real speed measurement so braking distances mean something.
This is what turns "slow everywhere" into "fast on straights, slow in
corners", which is most of a lap time.

### 4. Give it a short memory — a day or two

Accumulate the last N scans in a car-fixed frame, using the commanded steering
and speed for dead reckoning. The car rotates as it drives, so recent scans
cover the sides the current one cannot. That synthesises a much wider
effective field of view and unlocks proper wall-following, which is what most
fast implementations of this exercise actually use on closed tracks.

### 5. Follow a line rather than a direction — larger

With a memory you can extract a centreline and run pure pursuit on it, which
is the standard way to get a smooth fast lap. This is a different project from
follow-the-gap and should only be started once 1–4 are done and measured.

---

## How to measure anything here

`scripts/benchmark.sh N scene` runs the same race N times and reports the
cleared rate and the spread of cycle counts.

Use it. Single runs mislead in both directions: during this audit a change was
judged a regression against a three-run baseline that had not been
re-verified, and the same code later measured 8 out of 8. Conversely a change
that looked harmless in three runs turned out to be 0 out of 8.

For reference, the current code on `bootstrap_track`:

```
cleared  8/8  (100%)
cycles   min 42  median 43  max 46
```

That spread is tight, so the setup is not inherently noisy — which means a
change that costs half the cleared runs is a real regression and not bad luck.
Two or three runs simply cannot see the difference.

## Experiments run, and what they returned

Everything below was measured with `scripts/benchmark.sh` over eight runs on
`bootstrap_track`, against a baseline of 8 cleared out of 8 at a median of
43 cycles.

| Change | Cleared | Median cycles | Verdict |
|---|---|---|---|
| **Continuous aim** (centroid over the plateau, softened bubble, gap hysteresis) | 8/8 | **40** | **kept** |
| Skip actuator commands that repeat a value | 0/8 | 43–238 | rejected |
| Loosen the steering rate limit to 0.60 | 8/8 | 51 | rejected, 27% slower |
| Lower the proportional gain to 1.0 | 4/7 | — | rejected |

The negative results are the informative ones.

**The sawing is not what it looked like.** The audit blamed a discontinuous
setpoint. Making the setpoint continuous reduced the target's mean jump from
8.3° to 7.1° and left the steering reversals at 70%, exactly where they were.
So the reversals are not caused by the setpoint.

The next suspect was the rate limiter: the commanded steering is clipped in
69% of cycles while the car never uses more than 0.43 of the available lock,
which looks like a saturated controller producing a triangle wave. Loosening
it made the car slower and no smoother — the limiter simply clipped at a
higher level, and the swings got bigger. Lowering the gain instead cost half
the cleared runs.

Three attempts to damp the oscillation each made the car worse. The honest
reading is that **the reversals are functional, not pathological**: a car with
60° of vision and one decision per metre, driving down the middle of a narrow
corridor, corrects constantly because that is what staying in the middle
requires. It looks like sawing on a telemetry trace. Damping it costs more
than it buys, because every bit of damping is lag, and lag here is metres.

That reframes the remaining work. The way to a smoother, faster car is not to
filter the controller. It is to give it something better to follow —
anticipation and memory, findings 3 and 5 — so that it *needs* fewer
corrections.

## What this audit does not claim

I have not seen the implementations being compared against. The findings above
are properties of this code, established by measurement, and they are
sufficient to explain jerky steering and a slow lap without needing to
speculate about anyone else's design.

Two of them — the discontinuous setpoint and the wasted command budget — are
straightforward to fix and independently testable. That is where to start.
