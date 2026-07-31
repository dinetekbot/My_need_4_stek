# 6. Tuning

Every threshold in this project is a command line option. Nothing is a magic
number buried in a source file, and nothing requires a rebuild to change.

```bash
./ai --help
scripts/run.sh -- --kd=0.5 --turn-penalty=0.4 --verbose
```

That is not a stylistic preference. Tuning a controller means trying twenty
values in twenty minutes; if each one costs an edit and a rebuild, you try
three and settle for the first that does not crash.

## The method

Change **one** parameter at a time, by a factor of roughly 1.5, and watch a
full lap. Controller parameters interact — if you move two at once you cannot
attribute the result to either, and you will end up with a configuration that
works on one track for reasons nobody understands.

Order matters. Get the car stable before making it fast:

1. **Stability first.** The car should complete a slow lap without touching
   anything. Tune `--kp`, `--kd`, `--balance`.
2. **Then safety margins.** `--bubble`, `--gap-threshold`, `--brake-dist`.
3. **Then speed.** `--dist-fast`, `--turn-penalty`, `--throttle-max`.

A fast car that crashes is worth less than a slow one that finishes, and it is
much harder to debug.

## The options

### Perception

| Option | Default | Effect |
|---|---|---|
| `--smooth` | 3 | Moving average width, in rays. Higher is steadier but blurs narrow openings. 1 disables. |
| `--bubble` | 260 | Safety radius in mm around the closest obstacle. **The single most impactful parameter.** |
| `--gap-threshold` | 1200 | How far a ray must reach to count as drivable, in mm. |
| `--lidar-floor` | 1 | Distance substituted for a faulty reading. Rarely worth changing. |

**`--bubble` too low** — the car clips walls on the inside of corners, and
eventually touches one.

**`--bubble` too high** — the car finds no gap in a corridor it fits through,
declares a dead end mid-hairpin, and stops. Note that this is capped
internally so it can never erase more than half the cone
([why](03-perception.md#why-it-is-capped)), which limits the damage but does
not remove it.

**`--gap-threshold` too high** — nothing qualifies as free space on a tight
track and the car sees dead ends everywhere. **Too low** — the gap search
picks openings the car does not fit through.

### Steering

| Option | Default | Effect |
|---|---|---|
| `--kp` | 0.85 | How hard the car turns for a given heading error. |
| `--kd` | 0.35 | Damping. Raise this when the car weaves. |
| `--balance` | 0.12 | Corridor centring weight. |
| `--steer-rate` | 0.25 | Maximum change of `WHEELS_DIR` per cycle. |

The classic diagnosis:

| What you see | What to do |
|---|---|
| Weaving down straights | `--kd` up, or `--kp` down |
| Understeers, hits the outside of corners | `--kp` up |
| Twitchy, oversteers, spins | `--kp` down and `--steer-rate` down |
| Hugs one wall consistently | `--balance` up |
| Refuses the inside line, slow through corners | `--balance` down |
| Ignores the gap and takes odd lines | `--balance` too high — it is overruling the gap |

If raising `--kd` stops the weave but the car now responds sluggishly, you
have gone too far: D also damps the response you want.

### Throttle

| Option | Default | Effect |
|---|---|---|
| `--throttle-min` | 0.15 | Slowest racing power. |
| `--throttle-max` | 1.0 | Fastest. |
| `--dist-slow` | 600 | Distance mapped to `throttle-min`, mm. |
| `--dist-fast` | 2600 | Distance mapped to `throttle-max`, mm. |
| `--turn-penalty` | 0.55 | Fraction of throttle given up at full lock. |
| `--brake-dist` | 320 | Below this, throttle is cut entirely, mm. |

Going faster is mostly not about `--throttle-max`. The car is rarely limited
by top speed; it is limited by how late it brakes and how well it holds a
line. In order of return:

1. **`--dist-fast` down** — reaches full power sooner leaving a corner. Often
   worth more than anything else on the list.
2. **`--turn-penalty` down** — carries more speed through corners. Lower it
   until the car starts sliding, then back off.
3. **`--kd` up** — a steadier line lets every other parameter be more
   aggressive.
4. **`--throttle-max` up** — last.

**`--brake-dist` too high** — the car brakes at shadows and crawls. **Too low**
— it does not stop in time. Note it interacts with `--reverse-dist`: braking
must start before reversing does, so keep `brake-dist > reverse-dist`.

### Recovery and race management

| Option | Default | Effect |
|---|---|---|
| `--reverse-dist` | 260 | Below this, back up, mm. |
| `--reverse-power` | 0.35 | Power used while reversing. |
| `--deadend-cycles` | 12 | Consecutive gapless cycles before declaring a dead end. |
| `--deadend-clearance` | 1000 | Clearance required before ending the simulation, mm. |
| `--laps` | 0 | Stop after N laps. 0 means run until told otherwise. |
| `--mirror` | off | Flip the left/right convention. |
| `--verbose` | off | One telemetry line per cycle on stderr. |

**`--deadend-cycles` too low** and the car gives up in corners it could have
taken; a single gapless scan is noise, not a wall. **Too high** and it spends
a long time nudging a dead end before accepting it.

**`--deadend-clearance` is a rule, not a preference.** The car must come to
rest at least three feet from the wall. Lowering it below 1000 mm risks
finishing a run in a state that does not count.

## Reading the telemetry

```
[00071] RACING    fwd=  2410 near=  812 gap=yes@-04.7 wheels=-0.121 thr=0.836 rev=0.000 laps=0
```

Things to look for:

- **`gap=no` appearing repeatedly while `fwd` is large.** The bubble or the
  gap threshold is too aggressive — the car is blinding itself in open space.
- **`wheels` oscillating in sign every cycle.** Weaving. `--kd` up.
- **`wheels` pinned at ±1.0 for long stretches.** The car is asking for more
  steering than exists. Either `--kp` is too high, or it genuinely cannot make
  the corner and is about to reverse.
- **`thr` stuck near `throttle-min` on an open track.** `--dist-fast` is too
  high — the car does not believe the road is clear.
- **`near` well below `--bubble` for many cycles.** It is scraping along a
  wall. Raise `--balance` or `--bubble`.

To capture a run for later study:

```bash
scripts/run.sh -- --verbose 2> run.log
grep REVERSING run.log | head          # where did it get stuck?
awk '{print $6}' run.log | sort | uniq -c   # how often was there no gap?
```

## Per-track configurations

Nothing needs to be recompiled to change track, so keep the configurations
that work:

```bash
# track_1 -- wide and gentle, room to be aggressive
scripts/run.sh -- --dist-fast=2200 --turn-penalty=0.45

# criss_cross -- tight, sharp corners, be conservative
scripts/run.sh -- --bubble=320 --dist-fast=3000 --turn-penalty=0.65 --kd=0.5
```

A single set of parameters that finishes every track is a better result than a
set of per-track configurations that are each slightly faster. Reactive
control degrades gracefully; over-fitted parameters do not.

Next: [7. Docker](07-docker.md)
