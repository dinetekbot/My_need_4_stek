# 5. Simulator setup

The AI is one of four processes. This chapter gets the other three running.

## What you need

| Piece | What it is | Where it comes from |
|---|---|---|
| `ai` | This repository | `make` |
| `n4s` | The API: text protocol ⇄ simulator | Not in this repository — see below |
| `n4s_buffer` | Unbuffered pipe forwarder | Ships with `n4s` |
| CoppeliaSim | The 3D physics simulator | coppeliarobotics.com |
| `*.ttt` scenes | Track and car | Ships with `n4s` |

**`n4s`, `n4s_buffer` and the scenes are not part of this repository.** They
are third-party binaries and assets that are not ours to redistribute. Put
them in `vendor/` (which is gitignored) or point `API_DIR` at wherever you
keep them.

## Installing CoppeliaSim

The scenes were authored for **CoppeliaSim Edu V4.1.0, Ubuntu 20.04 build**,
available from the "previous versions" page at coppeliarobotics.com. Download,
extract, and launch:

```bash
tar -xf CoppeliaSim_Edu_V4_1_0_Ubuntu20_04.tar.xz
cd CoppeliaSim_Edu_V4_1_0_Ubuntu20_04
./coppeliaSim.sh
```

Two notes before you spend an evening on this.

**It is a 2020 build.** On a modern distribution it may want Qt5 and X11
libraries that are no longer installed by default. The error you get is a
missing `.so` at startup; install the named library and try again. On Fedora
and Arch this is usually two or three packages.

**Never commit it.** It is a ~2 GB directory of binaries. An earlier
incarnation of this project committed the entire install, and removing it
later meant rewriting history. `.gitignore` already covers `CoppeliaSim*/` and
`*.ttt`.

## Configuring the scene

Two settings, both easy to miss, both worth getting right.

**1. Select the ODE physics engine.**

```
Simulation → Using the ODE physics engine
```

The car's suspension and tyre friction were built for ODE. Under the default
engine it handles noticeably worse, and you will find yourself tuning the
controller to compensate for the wrong physics.

**2. Load a track.**

```
File → Open scene…  →  scene/track_1.ttt
```

`track_1` is the gentle one — start there. `criss_cross` and the bonus scenes
are considerably harder and are the ones worth testing a dead-end handler on.

Leave CoppeliaSim running with the scene loaded. Do **not** press its own play
button: `START_SIMULATION`, sent by the AI, does that.

## Running a race

```bash
make
API_DIR=~/n4s_package scripts/run.sh -- --verbose
```

The script builds the pipeline and starts driving. `--verbose` prints one
telemetry line per cycle on stderr:

```
[00071] RACING    fwd=  2410 near=  812 gap=yes@-04.7 wheels=-0.121 thr=0.836 rev=0.000 laps=0
```

| Field | Meaning |
|---|---|
| `[00071]` | Cycle number |
| `RACING` | State machine state |
| `fwd` | Distance straight ahead, mm |
| `near` | Closest obstacle anywhere in the cone, mm |
| `gap=yes@-04.7` | A gap was found, at −4.7° (left of centre) |
| `wheels` | `WHEELS_DIR` sent |
| `thr` / `rev` | `CAR_FORWARD` / `CAR_BACKWARDS` sent |
| `laps` | Completed laps |

Ctrl-C stops the car and ends the simulation cleanly.

## Why the pipeline is a ring

```
    ai ──stdout──▶ buffer ──▶ n4s ──▶ buffer ──┐
     ▲                                          │
     └──────────────── FIFO ────────────────────┘
```

A shell pipeline can only express a line, and we need a loop: the AI's output
must reach the API, and the API's output must come back to the AI's input. The
return path is a named pipe (FIFO), created by
[`run.sh`](../scripts/run.sh) and removed when it exits.

The two `buffer` processes are there because the API writes through buffered
stdio. Between pipes that buffer is 4 KB, so its answer would sit in memory
while the AI waits for it and it waits for the next command — a deadlock with
no error message. `n4s_buffer` forwards byte by byte and breaks it. See
[protocol, trap 2](01-protocol.md#2-buffered-stdout-deadlocks-the-pipeline)
for why the AI never uses `printf` on stdout either.

`run.sh` creates its FIFO with `mktemp -u`, so two runs on one machine do not
fight over the same pipe.

## Troubleshooting

| Symptom | Cause |
|---|---|
| `6:KO:Network operation Failure` | CoppeliaSim is not running. The API connects to `127.0.0.1:19997` |
| `0:KO:Simulation has not been launched` | Scene loaded but the simulation was never started |
| `22:KO:Scene must contain at least 3 CPs` | Wrong scene, or an edited one that lost its checkpoints |
| `25:KO:Failed to load Map` | Scene not compatible with this build of CoppeliaSim |
| Nothing happens, no output, no error | Buffering deadlock — something is writing to stdout through stdio |
| Car steers into walls, consistently | Left/right convention — add `--mirror`, see [perception](03-perception.md#which-end-is-the-left) |
| Car crawls, or does not move at all | A command outside its valid range is being rejected. Run with `N4S_DEBUG=1` to see the API's replies |
| Car reaches a corner and stops | The safety bubble is erasing the whole cone. Lower `--bubble` |

To check the protocol without any of the above, talk to the API by hand:

```console
$ ./n4s
get_car_speed_max
1:OK:No errors so far:50.3:No further info
```

`GET_CAR_SPEED_MAX` and `GET_CAR_SPEED_MIN` answer even with no simulator
attached, so a reply here proves the API binary itself is fine and narrows the
problem to CoppeliaSim.

Next: [6. Tuning](06-tuning.md)
