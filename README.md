# lidar-racer

This is a small autonomous race car. It sees with a 32-ray lidar, talks to the
simulator over a text protocol, and drives with two tables of numbers. That is
the whole trick.

I first built something much more ambitious: follow-the-gap, a safety bubble, a
PD steering controller, a recovery state machine. It looked clever on paper and
was catastrophic on the track. It drove in circles, got wedged in corners, and
found new ways to be stuck. A friend's version — a few thresholds on the
distance straight ahead, and a steering direction taken from the two edge rays
— cleared the same tracks almost immediately. This repository is that idea,
rewritten cleanly.

No map. No odometry. No camera. The car does not know how big it is. The lidar
and the two actuator values are the entire interface.

```
   lidar (32 rays, 60°)                 decision                actuators
  ╲   ╲   │   ╱   ╱
   ╲   ╲  │  ╱   ╱          forward ray -> power         WHEELS_DIR  −1…1
    ╲   ╲ │ ╱   ╱      ──▶  edge rays  -> direction  ──▶  CAR_FORWARD  0…1
     ╲   ╲│╱   ╱
      ┌───────┐
      │  car  │
      └───────┘
```

---

## Building and running

```bash
make                # builds ./ai
make test           # 111 assertions, no simulator required
```

Then start CoppeliaSim with a scene loaded, and run the same pipeline the N4S
launcher uses:

```bash
./pipe.sh -- --verbose
```

If you prefer the package's own launcher, copy `ai` next to it and run
`./pipes.sh` there, exactly as before:

```bash
cp ai /path/to/n4s_package/
cd /path/to/n4s_package
./pipes.sh
```

`pipe.sh` speaks the same protocol as any N4S binary. It needs `n4s` and
`n4s_buffer`; put them next to `ai`, or set `N4S` and `BUFFER`
current directory.

---

## How it drives

Every cycle it asks for one scan, picks a power, picks a steering lock, and
sends both back. Three round trips. No state is carried between cycles.

The distance straight ahead is `ranges[15]`. The two edge distances are
`ranges[1]` (left) and `ranges[31]` (right). Everything else is a lookup:

**Power**

| Clear road ahead | `CAR_FORWARD` |
|---|---|
| ≥ 2000 mm | 1.0 |
| ≥ 1500 mm | 0.8 |
| ≥ 1000 mm | 0.5 |
| ≥ 600 mm | 0.4 |
| ≥ 400 mm | 0.2 |
| otherwise | 0.1 |

**Steering**

| Clear road ahead | Lock |
|---|---|
| ≥ 1500 mm | 0.005 |
| ≥ 1000 mm | 0.05 |
| ≥ 600 mm | 0.1 |
| ≥ 400 mm | 0.2 |
| ≥ 200 mm | 0.3 |
| otherwise | 0.5 |

The lock direction is decided by the edge rays: if the left edge is closer
than the right, the lock is negative; otherwise it is positive. The size of
the lock comes from the forward distance — far away the car only needs a
feather of correction, close to a wall it needs a real turn.

That is genuinely all. No gap search, no bubble, no controller gains, no
recovery manoeuvre. On `bootstrap_track` it cleared the track in **8 to 11
cycles** across runs. The old version needed roughly forty, when it finished
at all.

---

## Keep the plumbing

The decision is simple; the plumbing around it is not, and I kept the parts
that earn their keep.

**The response parser does not split on `:`.** `ADDITIONAL_INFO` is free text
and contains colons of its own:

```
1:OK:No errors so far:-25.1:First CP Cleared: [0][88s 26029102ns]
```

Split it naively and the checkpoint message disappears into a field you are
not looking at, so the car never learns the race is over. The parser consumes
numeric fields greedily and treats the rest as one opaque string. There is a
regression test on that exact line.

**Nothing writes to stdout through stdio.** The processes form a ring. A
buffered `printf` sits in memory while the AI blocks waiting for an answer to
a command that was never sent. Both sides wait forever. Every byte goes out
through `write(2)`.

**Progress can arrive on any response.** The simulator attaches checkpoint
news to whatever reply happens to be next, so every response is inspected, not
just the one to a dedicated command.

**Start by stopping.** The AI sends `STOP_SIMULATION` before
`START_SIMULATION`. Without it, a run killed halfway leaves the car wherever
it died, and the next race starts nose-first against a wall. That looks
exactly like an AI driving at random.

---

## Testing

`make test` runs 111 checks with no simulator, network or fixtures:

```
$ make test
util
car
response

111 checks, 0 failed
OK
```

The interesting cases are the table boundaries: one millimetre below a
threshold must select a different tier than the threshold itself. The parser
is covered against the real response format, including the checkpoint line
above.

What the tests cannot tell you is whether the car completes a track. That
needs the simulator, a loaded scene, and `./pipe.sh`.

---

## Requirements

- A C99 compiler and `make`
- CoppeliaSim and the `n4s` API binaries to actually drive
- Optionally Docker — see [docs/07-docker.md](docs/07-docker.md)

## Documentation

| Document | What it covers |
|---|---|
| [docs/01-protocol.md](docs/01-protocol.md) | The wire protocol and the traps in it |
| [docs/02-architecture.md](docs/02-architecture.md) | The current code layout |
| [docs/05-simulator-setup.md](docs/05-simulator-setup.md) | Installing CoppeliaSim and the API |
| [docs/07-docker.md](docs/07-docker.md) | Running it in a container |

The other chapters (`03`, `04`, `06`, `08`) describe the controller I replaced.
I left them in place because the measurements and the mistakes are still worth
reading, but they are not what the code does any more.

## Licence

MIT. See [LICENSE](LICENSE).

The simulator, the scenes and the `n4s` API binaries are not part of this
repository and are not covered by that licence.
