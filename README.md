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
make test           # 900+ assertions, no simulator required
```

To actually drive, you need the simulator and the API binaries — see
[docs/05-simulator-setup.md](docs/05-simulator-setup.md). Once they are in
place:

```bash
scripts/run.sh -- --verbose --laps=3
```

```
[00071] RACING    fwd=  2410 near=  812 gap=yes@-04.7 wheels=-0.121 thr=0.836 rev=0.000 laps=0
[00072] RACING    fwd=  2290 near=  790 gap=yes@-06.6 wheels=-0.184 thr=0.771 rev=0.000 laps=0
[00073] RACING    fwd=  1980 near=  733 gap=yes@-10.3 wheels=-0.281 thr=0.652 rev=0.000 laps=0
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

A small state machine sits on top: `RACING` → `REVERSING` when a wall gets too
close, → `HALTING` when there has been no gap for long enough that it cannot
be sensor noise. Halting backs off until there is a metre of clearance before
ending the simulation, because stopping with the bumper against the wall is a
failed run even though the car did stop.

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
observable. `--mirror` covers it.

**Every threshold is a command line option.** Tuning a car by editing
constants and rebuilding turns a ten-minute job into an evening. The option
table also generates `--help` and the telemetry dump, so they cannot drift
apart.

---

## Testing

The driving code is deliberately split so that the parts that can be tested
without a simulator, are:

```
$ make test
util
response
perception
control

921 checks, 0 failed
OK
```

Parsing, scan geometry, the safety bubble, gap selection, both control laws
and every state machine transition are covered by assertions on hand-written
inputs. A car that understeers in a 3D simulator tells you nothing about which
of five stages is wrong; a failing assertion tells you exactly.

What the tests do **not** cover is whether the resulting car is fast, or
whether it completes a given track. That needs the simulator, and it is
judged by watching it.

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
