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

The archive is about 150 MB and unpacks to 455 MB. On a current Fedora it
needed no extra packages at all — every library it wants (`libGL`, `libX11`,
the `xcb` family, `libxkbcommon-x11`, `fontconfig`, `nss`) was already
present. Check before you go hunting:

```bash
LD_LIBRARY_PATH=$PWD ldd coppeliaSim | grep "not found"
```

Silence means you are fine. Two plugins (`BlueZero`, `Image`) fail to load and
neither matters here.

**Never commit it.** An earlier incarnation of this project committed the
entire install and had to rewrite history to remove it. `.gitignore` already
covers `CoppeliaSim*/` and `*.ttt`.

### Startup dialogs will hang it

Out of the box, `system/usrset.txt` has `suppressStartupDialogs = false`.
CoppeliaSim then blocks on a modal dialog during startup — the remote API port
opens and accepts connections, but every request returns an error, and the
main thread sits in a futex forever. It looks like a crash and is not one.

Set these in `system/usrset.txt` before the first run:

```
suppressStartupDialogs = true
doNotShowUpdateCheckMessage = true
doNotShowCrashRecoveryMessage = true
```

## Loading a scene

The normal way, with the window open:

```bash
./coppeliaSim.sh ~/n4s-scenes/bootstrap_track.ttt
```

or launch it bare and use `File → Open scene…`. Either works.

Two things will waste your afternoon if you do not know them.

**The command line argument does not survive a path with spaces.**
`coppeliaSim.sh` does not quote it, so a scene under `~/my projects/` arrives
as two arguments and is silently ignored — you get an empty scene and the API
reports `5:KO:SimComponent Loading Failure`. Keep the scenes somewhere without
spaces.

**A corrupt scene hangs the whole application.** Handing it `track_1.ttt`
(see below) does not produce an error dialog. CoppeliaSim opens its remote API
port, accepts connections, and then blocks forever with its main thread parked
on a futex, answering every single request with an error. It looks exactly
like a broken installation, and it is not — it is one bad file. If the
application appears to start but nothing responds, try a different scene
before you start reinstalling libraries.

### Headless

`-h` runs without a window. It is noticeably quicker — the same track that
takes 88 control cycles and 55 seconds with the GUI takes 44 cycles and 22
seconds headless, because nothing is being rendered — which makes it the
better mode for tuning runs and the only sensible one for CI.

The catch: **headless ignores the scene argument entirely.** Load the scene
over the remote API instead. [`scripts/load_scene.py`](../scripts/load_scene.py)
does that, selects the ODE engine, and checks the scene actually contains the
objects the API drives:

```bash
export COPPELIASIM_ROOT=~/opt/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04
scripts/load_scene.py ~/n4s-scenes/bootstrap_track.ttt
```
```
scene       bootstrap_track.ttt
objects     258
checkpoints 3
engine      ode

Ready. Leave the simulation stopped; the AI starts it.
```

It refuses a path containing a space and reports a scene that will not load,
rather than leaving you to work out why the API says `SimComponent Loading
Failure`. Under the hood it is just the bindings CoppeliaSim ships in
`programming/remoteApiBindings/python/`:

```python
import sim
cid = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
sim.simxLoadScene(cid, '/home/you/n4s-scenes/bootstrap_track.ttt', 0,
                  sim.simx_opmode_blocking)
sim.simxSetIntegerParameter(cid, sim.sim_intparam_dynamic_engine, 1,
                            sim.simx_opmode_blocking)   # 1 = ODE
```

### Check the physics engine

```
Simulation → Using the ODE physics engine
```

The car's suspension and tyre friction were built for ODE; under another
engine you end up tuning the controller to compensate for the wrong physics.
The supplied scenes are already saved with ODE selected — worth confirming
rather than assuming.

### Not every supplied scene works

Of the eight scenes shipped with the API package, two are unusable:

| Scene | State |
|---|---|
| `bootstrap_track` | works — 3 checkpoints, the one to tune against |
| `criss_cross` | works — 3 checkpoints, tight |
| `diamond_bonus` | works — 3 checkpoints |
| `track_3`, `track_4` | work — 12 checkpoints each |
| `8_bonus` | works — 11 checkpoints |
| **`track_1`** | **will not load, and hangs the application if passed on the command line** |
| **`track_2`** | **loads, but its lidar returns nothing but zeros** |

`track_1` is the one the original instructions tell you to open first, which
is worth knowing before concluding that your AI is broken. Start with
`bootstrap_track`.

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
[00031] RACING    fwd=  3007 near=   650 gap=yes@+006.6 wheels=+0.123 thr=0.513 rev=0.000 laps=0
        >>> TRACK CLEARED (laps=0)
```

| Field | Meaning |
|---|---|
| `[00031]` | Cycle number |
| `RACING` | State machine state |
| `fwd` | Distance straight ahead, mm |
| `near` | Closest obstacle anywhere in the cone, mm |
| `gap=yes@+006.6` | A gap was found, at +6.6° (right of centre) |
| `wheels` | `WHEELS_DIR` sent |
| `thr` / `rev` | `CAR_FORWARD` / `CAR_BACKWARDS` sent |
| `laps` | Completed laps |
| `>>>` | A track progress event: checkpoint, lap, or track cleared |

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
| CoppeliaSim starts but every API call errors | A corrupt scene, or a startup dialog. Try another scene first |
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
