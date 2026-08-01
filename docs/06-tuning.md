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
| `--bubble` | 260 | Safety radius in mm around the closest obstacle. |
| `--gap-threshold` | 800 | How far a ray must reach to count as drivable, in mm. **The single most impactful parameter.** |
| `--lidar-floor` | 1 | Distance substituted for a faulty reading. Rarely worth changing. |
| `--mirror` | on | Left/right convention. See [perception](03-perception.md#which-end-is-the-left). |

**Start with `--gap-threshold`, and set it from the track, not from taste.**
It has to sit below the depth of the tightest corridor the car must drive
through, or the gap search flickers on and off from one cycle to the next and
the car crawls in fits.

The default was 1200 until a real run showed what that does on a track whose
corridors are about 1.1m deep:

```
[00250] RACING  fwd=1225 gap=yes@+000.9 wheels=+0.063 thr=0.188
[00251] RACING  fwd=1125 gap= no        wheels=+0.063 thr=0.000
[00252] RACING  fwd=1125 gap= no        wheels=+0.063 thr=0.000
[00253] RACING  fwd=1125 gap=yes@-014.1 wheels=-0.187 thr=0.157
[00254] RACING  fwd=1125 gap= no        wheels=-0.187 thr=0.000
```

The forward distance is sitting right on the threshold, so half the scans find
nothing. Dropping it to 800 turned that same track from *285 cycles, six
recoveries, dead end* into *44 cycles, no recoveries, track cleared*.

To pick a value: run with `--verbose`, watch the `fwd` column through the
tightest part of the track, and set the threshold comfortably below the
smallest value you see there.

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
| `--kp` | 1.5 | How hard the car turns for a given heading error. |
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
| `--throttle-max` | 0.55 | Fastest. |
| `--dist-slow` | 700 | Distance mapped to `throttle-min`, mm. |
| `--dist-fast` | 2600 | Distance mapped to `throttle-max`, mm. |
| `--turn-penalty` | 0.55 | Fraction of throttle given up at full lock. |
| `--brake-dist` | 600 | Below this, drop to a crawl, mm. |
| `--crawl-power` | 0.10 | Power used while crawling past an obstacle. |

**The thresholds must be ordered**, or whole bands of behaviour become
unreachable:

```
reverse-dist  <  brake-dist  <=  dist-slow  <  dist-fast
     400           600            700           2600
```

Put `dist-slow` below `brake-dist` and the entire `throttle-min` region sits
inside the braking region, so the slow end of the ramp never runs. There is an
assertion for this in the test suite, which is where that mistake was caught.

**`--throttle-max` is 0.55, not 1.0, and that is a measurement.** At full
power the car covered roughly 1.1m per control cycle, and the loop runs at
about two cycles per second against a live simulator. A car that moves more
than a metre between decisions cannot brake for anything: it went from 3010mm
of clear road to 396mm in six cycles, with the controller doing everything
right the whole way down. Speed is bounded by your loop rate before it is
bounded by the engine.

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
| `--reverse-dist` | 400 | Below this, back up, mm. |
| `--reverse-power` | 0.35 | Power used while reversing. |
| `--blind-cycles` | 4 | Cycles making no progress before starting the escape manoeuvre. |
| `--escape-reverse-cycles` | 6 | Length of the backing-up phase of the three-point turn. |
| `--escape-forward-cycles` | 6 | Length of the pulling-forward phase. |
| `--escape-max-pairs` | 4 | Back-and-forward pairs before admitting a dead end. |
| `--deadend-attempts` | 6 | Failed recoveries before declaring a dead end. |
| `--recovery-reset` | 20 | Clean racing cycles that clear the recovery counter. |
| `--deadend-clearance` | 1000 | Clearance required before ending the simulation, mm. |
| `--laps` | 0 | Stop after N laps. 0 means run until told otherwise. |
| `--verbose` | off | One telemetry line per cycle on stderr. |

**`--blind-cycles` must stay small.** It is not a "be sure it is really a
wall" timer — creeping forward while the gap search returns nothing is driving
blind, and the wall does not move away while you wait. Four cycles is enough
to reject noise.

**The escape phases scale with the track, like the distances.** Longer phases
rotate the car further per pair, which helps in an open dead end and hurts in
a corridor barely longer than the car -- there, a long reverse simply runs
into whatever is behind. Shorten both phases on tight tracks.

**`--deadend-attempts` too low** and the car gives up in corners it could have
taken. **Too high** and it spends a long time nudging a genuine dead end
before accepting it. Note it counts *recoveries*, not cycles, and
`--recovery-reset` clears it after a clean stretch — so a track with many
tight corners does not accumulate its way into a false dead end.

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
grep ESCAPE run.log | head             # where did it get stuck?
awk '{print $6}' run.log | sort | uniq -c   # how often was there no gap?
```

## Per-track configurations

Nothing needs to be recompiled to change track, so keep the configurations
that work:

```bash
# bootstrap_track -- short, three checkpoints, the one to tune against
scripts/run.sh -- --verbose

# tighter corridors: let the gap search see further into them
scripts/run.sh -- --gap-threshold=600 --brake-dist=500 --bubble=200

# more room: carry more speed
scripts/run.sh -- --dist-fast=2200 --turn-penalty=0.45

# 8_bonus -- very tight. Forward distance never exceeds ~870mm anywhere on
# the track, so the default thresholds leave the car crawling permanently and
# finding no gaps. Everything scales down together.
scripts/run.sh -- --gap-threshold=400 --brake-dist=300 --reverse-dist=200 \
                  --bubble=120 --dist-slow=350 --dist-fast=800
```

That last one is the general lesson: **the distance thresholds are a set, and
they scale with the track.** On `8_bonus` the defaults produced 215 cycles, 15
gapless ones and no checkpoint at all; scaled down, the same car made a
checkpoint with zero gapless cycles. Nothing about the controller changed.

To find the scale for an unfamiliar track, run once with `--verbose` and look
at the `fwd` column:

```bash
scripts/run.sh -- --verbose 2>&1 | grep -oP 'fwd=\s*\K[0-9]+' \
  | sort -n | awk 'NR==1{print "min", $1} END{print "max", $1}'
```

A single set of parameters that finishes every track is a better result than a
set of per-track configurations that are each slightly faster. Reactive
control degrades gracefully; over-fitted parameters do not.

Next: [7. Docker](07-docker.md)
