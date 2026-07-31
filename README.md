# lidar-racer

An autonomous car that drives a race track using nothing but a 32-ray lidar,
a text protocol, and about a thousand lines of C.

The car has no map, no odometry, no camera and no knowledge of its own
dimensions. Thirty-two distances in a 60° cone in front of it, updated every
cycle, are the entire input. Two numbers — steering angle and engine power —
are the entire output. Everything between the two is in this repository.

```
   lidar (32 rays, 60°)              decision                actuators
  ╲   ╲   │   ╱   ╱
   ╲   ╲  │  ╱   ╱          clean → bubble → gap        WHEELS_DIR  −1…1
    ╲   ╲ │ ╱   ╱      ──▶       ↓            ──▶       CAR_FORWARD  0…1
     ╲   ╲│╱   ╱                PD + throttle curve
      ┌───────┐
      │  car  │
      └───────┘
```

---

## Table of contents

| Document | What it covers |
|---|---|
| [docs/01-protocol.md](docs/01-protocol.md) | The wire protocol, and the three traps in it |
| [docs/02-architecture.md](docs/02-architecture.md) | How the code is laid out and why |
| [docs/03-perception.md](docs/03-perception.md) | From 32 numbers to "drive that way" |
| [docs/04-control.md](docs/04-control.md) | The steering and throttle laws, and the PD controller |
| [docs/05-simulator-setup.md](docs/05-simulator-setup.md) | Installing CoppeliaSim and the API, running a race |
| [docs/06-tuning.md](docs/06-tuning.md) | Every knob, what it does, how to find a good value |
| [docs/07-docker.md](docs/07-docker.md) | Running it in a container |

---

## Quick start

```bash
git clone <this repository>
cd lidar-racer
make                # builds ./ai
make test           # 948 assertions, no simulator required
```

`make` is not optional before a race: `scripts/run.sh` needs `./ai` to exist,
and `make fclean` removes it.

To actually drive, you also need the simulator and the API binaries — see
[docs/05-simulator-setup.md](docs/05-simulator-setup.md). Once they are in
place:

```bash
scripts/run.sh -- --verbose --laps=3
```

Real output from a cleared track:

```
[00030] RACING    fwd=  3010 near=   850 gap=yes@-004.7 wheels=-0.187 thr=0.493 rev=0.000 laps=0
[00031] RACING    fwd=  3007 near=   650 gap=yes@+006.6 wheels=+0.123 thr=0.513 rev=0.000 laps=0
[00032] RACING    fwd=  2923 near=   612 gap=yes@+008.4 wheels=+0.123 thr=0.501 rev=0.000 laps=0
        >>> checkpoint (laps=0)
[00039] RACING    fwd=   975 near=   875 gap=yes@+008.4 wheels=+0.292 thr=0.174 rev=0.000 laps=0
        >>> TRACK CLEARED (laps=0)
```

---

## How it drives

The method is **follow the gap**, the same family of algorithm used in
small-scale autonomous racing. Each cycle:

1. **Clean the scan.** Faulty readings collapse to "obstacle right here", not
   to "open road" — the failure mode has to be braking, never accelerating.
2. **Draw a bubble.** Erase every ray within a safety radius of the closest
   obstacle. Closer obstacles produce wider bubbles, so the car naturally
   swings wide around a corner it is about to clip.
3. **Find the widest gap.** The longest surviving run of rays that reach far
   enough to be drivable. Aim at the middle of its deepest plateau rather than
   at the single furthest ray, which would flicker between neighbours and make
   the car wag down the straights.
4. **Steer.** A PD controller on the heading error, plus a light corridor
   centring term, rate limited so the wheels never snap.
5. **Set the power.** A distance-to-throttle curve, reduced in proportion to
   how hard the car is turning.

A small state machine sits on top: `RACING` → `REVERSING` whenever the car
stops making progress, for any reason; → `HALTING` once backing up and
retrying has failed enough times to count as a genuine dead end. Halting backs
off until it has a metre of clearance before ending the simulation — stopping
with the bumper against the wall is a failed run even though the car did
stop — and settles for the clearance it can actually reach, since reversing is
blind and a corner can wedge the car short of the ideal.

---

## Design notes worth reading even if you never run it

Each of these is a specific decision with a specific reason, documented at the
place in the code where it applies.

**The response parser does not split on `:`.** `ADDITIONAL_INFO` is free text
that contains colons of its own:

```
1:OK:No errors so far:-25.1:First CP Cleared: [0][88s 26029102ns]
```

Splitting on `:` and reading fixed indices strands the timestamp in a separate
field, and code looking for the checkpoint message finds nothing — so the car
never learns it is making progress and never learns the race has ended. The
parser consumes numeric fields greedily and treats everything after them as
one opaque string. There is a regression test on that exact line.

**Nothing writes to stdout through stdio.** The processes form a ring, so a
buffered `printf` sits in memory while we block reading the answer to a
command that was never sent. Both sides wait forever, and the symptom is a
program that simply does nothing. Every byte goes out through `write(2)`.

**Every cycle costs exactly four round trips.** Always the same four, even
when one is redundant. A constant loop rate is what lets the derivative term
of the steering controller be a plain difference between consecutive errors
instead of a division by a jittery measured timestep. It is also the single
biggest speed-up available: re-reading the car's maximum and minimum speed
every cycle, as is tempting, spends half the loop waiting for two constants.

**One flag fixes both left/right conventions.** Whether lidar ray 0 points
left, and whether a positive steering value turns right, are both
undocumented. That looks like four cases but is really two — mirroring the
perception and mirroring the action cancel out, so only the combination is
observable. `--mirror` covers it, and it defaults to on because that is what
the simulator turned out to need: with it off, the car steers into the first
corner within three cycles, every time.

**Every threshold is a command line option.** Tuning a car by editing
constants and rebuilding turns a ten-minute job into an evening. The option
table also generates `--help` and the telemetry dump, so they cannot drift
apart.

**The car is never idle.** Every cycle it either commands forward motion or is
recovering. This sounds obvious and is the rule the first version broke: with
one threshold to cut the throttle and a lower one to start reversing, there is
a band between them where the car does neither. It parked itself in that band,
300mm from a wall, and waited twenty cycles for a timer — with nothing in the
telemetry explaining why, because no single rule was being violated.

**It never drives without a gap, and never stops when it has one.** Two halves
of the same idea. Creeping forward when the gap search found nothing is
driving blind. But stopping dead when a gap *is* visible is just as wrong: a
stationary car with its wheels turned goes nowhere, so cutting the throttle
near a wall prevents the exact manoeuvre needed to get round it. Below the
braking distance with a gap in sight, the car crawls.

**Speed is bounded by the loop rate, not the engine.** At full power the car
covered about 1.1m per control cycle, and the loop runs at roughly two cycles
per second against a live simulator — so it went from 3010mm of clear road to
396mm in six cycles, with the controller doing everything correctly the whole
way down. `--throttle-max` defaults to 0.55 for that reason.

---

## Results

Measured against CoppeliaSim 4.1 with the N4S API, ODE physics, default
settings, at roughly two control cycles per second:

| Scene | Result |
|---|---|
| `bootstrap_track` | **Track cleared**, 44 cycles, 22 s, zero recoveries |
| `criss_cross` | 2 of 3 checkpoints, then a clean dead-end stop |
| `track_3` | drives 1205 cycles with 3 recoveries and no dead ends, reaching 2 of 12 checkpoints — safe but slow |
| `track_4`, `diamond_bonus` | drive continuously for 380+ cycles, 0–7 recoveries, not run long enough to clear |
| `track_1` | the scene itself will not load |
| `track_2` | the scene's lidar returns nothing but zeros |

The two broken scenes are broken independently of this code: `track_1` fails
to load into CoppeliaSim at all, and `track_2` loads but its sensor never
reports. `track_1` is the scene the original project instructions tell you to
open first.

The honest reading of the long tracks: the car is **safe but slow** on them.
It never wedges itself and never falsely declares a dead end, but it takes a
lot of cycles per checkpoint. Follow-the-gap drives down the middle of the
corridor rather than along a racing line, and the throttle is deliberately
conservative because the control loop only runs at about two cycles per
second. Going faster is a tuning exercise, and
[docs/06-tuning.md](docs/06-tuning.md) says which knobs pay first.

Every default was set from a measurement, not a guess, and that document
records which measurement and why.

## Testing

The driving code is deliberately split so that the parts that can be tested
without a simulator, are:

```
$ make test
util
response
perception
control

948 checks, 0 failed
OK
```

Parsing, scan geometry, the safety bubble, gap selection, both control laws
and every state machine transition are covered by assertions on hand-written
inputs. A car that understeers in a 3D simulator tells you nothing about which
of five stages is wrong; a failing assertion tells you exactly.

Several of these tests are regressions written *after* a real run exposed
something the unit tests could not have caught on their own — the idle band
between the braking and reversing thresholds, driving forward with no gap, the
halting loop that never settles. Each one names the failure it prevents.

What the tests still do **not** cover is whether the resulting car is fast, or
whether it completes a given track. That needs the simulator. See
[Results](#results).

---

## Requirements

- A C99 compiler and `make` (that is the whole build dependency list)
- For driving: CoppeliaSim and the `n4s` API binaries — see
  [docs/05-simulator-setup.md](docs/05-simulator-setup.md)
- Optionally Docker — see [docs/07-docker.md](docs/07-docker.md)

## Licence

MIT. See [LICENSE](LICENSE).

The simulator, the scenes and the `n4s` API binaries are not part of this
repository and are not covered by that licence.
