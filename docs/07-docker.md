# 7. Docker

## What is in the image, and what is not

The image contains the AI, its tests, and the script that wires it to the
simulator API. It does **not** contain CoppeliaSim or the `n4s` binaries.

That is a deliberate boundary, not an omission:

- CoppeliaSim is a ~2 GB GUI application that wants a GPU and a display.
  Containerising it is a project in its own right, and the result would take
  longer to pull than to compile the thing it is testing.
- `n4s` and `n4s_buffer` are third-party binaries, not ours to redistribute.

So the container runs the AI and reaches out to a CoppeliaSim running on the
host. The API binaries are bind-mounted rather than baked in.

```
   ┌─ container ──────────────────┐        ┌─ host ─────────────┐
   │  ai ─▶ buffer ─▶ n4s ─▶ ...  │──TCP──▶│  CoppeliaSim       │
   │            (bind-mounted)    │  19997 │  track_1.ttt       │
   └──────────────────────────────┘        └────────────────────┘
```

## Tests only — no simulator needed

This is the target to point CI at. It needs neither CoppeliaSim nor the API
binaries.

```bash
docker compose run --rm tests
```

Or without compose:

```bash
docker build --target build -t lidar-racer:test .
```

The build stage runs `make test` as its last step, so **a build that succeeds
is a build whose tests passed**. There is no way to produce an image from
code that does not pass.

## Running a race

```bash
# 1. Start CoppeliaSim on the host, load a scene, select ODE physics.
#    See docs/05-simulator-setup.md.

# 2. Point N4S_PACKAGE at the directory holding n4s and n4s_buffer.
export N4S_PACKAGE=~/n4s_package

# 3. Drive.
docker compose run --rm racer -- --verbose --laps=3
```

Without compose:

```bash
docker build -t lidar-racer .
docker run --rm --network=host \
    -v ~/n4s_package:/opt/n4s:ro \
    lidar-racer -- --verbose --laps=3
```

## Why `--network=host`

The `n4s` API opens a TCP connection to `127.0.0.1:19997`. Inside a container
on the default bridge network, `127.0.0.1` is the container's own loopback —
CoppeliaSim is not there, and the connection is refused. You would see:

```
6:KO:Network operation Failure:No further info
```

Sharing the host's network namespace is the smallest change that makes the
loopback address mean what the API expects. `host.docker.internal` would be an
alternative, but the API's target address is compiled in and cannot be
redirected.

Note that `--network=host` behaves differently on macOS and Windows, where
Docker runs inside a VM and the "host" is that VM rather than your desktop.
This setup is Linux-oriented, like the simulator it drives.

## The image

Multi-stage, for a reason that is worth stating:

| Stage | Contents | Measured size |
|---|---|---|
| `build` | gcc, libc-dev, make, sources | 267 MB, nearly all toolchain |
| `runtime` | bash, coreutils, the `ai` binary | 75 MB, nearly all base image |

The compiler does not ship. The AI is a single C binary that speaks a text
protocol over pipes — it needs a shell for the pipeline script and `mkfifo`
from coreutils, and nothing else.

The runtime stage runs as a non-root `racer` user. There is no reason for a
process whose entire job is reading and writing pipes to have root, and it
costs one line.

`init: true` in the compose file matters more than it looks: the entrypoint is
a shell script that spawns a four-process pipeline. Without an init process to
reap them, Ctrl-C leaves orphans behind.

## Environment variables

| Variable | Default | Purpose |
|---|---|---|
| `API_DIR` | `/opt/n4s` | Where `n4s` and `n4s_buffer` are mounted |
| `AI` | `/app/ai` | Path to the AI binary |
| `N4S` | `$API_DIR/n4s` | Override the API binary individually |
| `BUFFER` | `$API_DIR/n4s_buffer` | Override the buffer binary individually |
| `N4S_DEBUG` | `0` | Set to `1` to show the API's stderr |
| `N4S_PACKAGE` | `./vendor` | Host path bind-mounted to `/opt/n4s` (compose only) |

Everything after `--` on the command line goes to the AI unchanged, so every
option in [tuning](06-tuning.md) works identically inside and outside the
container.

## Reproducibility, honestly stated

What the container guarantees: the AI builds and its tests pass on a known
toolchain, on any machine with Docker, with no local setup.

What it does not guarantee: that the car completes a given track. That depends
on CoppeliaSim, its physics engine, and the scene — none of which are in the
image, and one of which is a 2020 build of a GUI application. Driving
behaviour is reproducible only in the sense that the same inputs produce the
same decisions; the simulator on the other end of the socket is the variable.

Back to the [README](../README.md).
