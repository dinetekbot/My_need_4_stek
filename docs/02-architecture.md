# 2. Architecture

## The layers

```
                    ┌─────────────────────────────────────────┐
   src/main.c       │  race loop, signals, exit codes          │
                    └────────────────┬────────────────────────┘
                                     │
        ┌────────────────────────────┼────────────────────────────┐
        │                            │                            │
┌───────▼────────┐          ┌────────▼─────────┐                  │
│  src/protocol  │          │     src/car.c    │                  │
│                │          │                  │                  │
│ link.c    I/O  │          │ scan -> power    │                  │
│ response.c     │          │ scan -> steering │                  │
│ commands.c     │          │                  │                  │
└────────────────┘          └──────────────────┘                  │
        │                            │                            │
        └────────────────────────────┴────────────────────────────┘
                                     │
                          ┌──────────▼──────────┐
                          │       src/util       │
                          └─────────────────────┘
```

The dependency direction is strict: `car.c` knows nothing about the protocol.
It takes an array of doubles and returns two numbers, which is what makes it
testable without a simulator.

## The cycle

One iteration of the race loop, in full:

```c
n4s_get_lidar(raw);                    /* 1 round trip */
car_step(raw, &cmd);                   /* two table lookups */
n4s_set_throttle(cmd.throttle);        /* 1 round trip */
n4s_set_wheels(cmd.wheels);            /* 1 round trip */
```

Three round trips per cycle, no carried state, no planner. The previous
version spent four and carried a gap search, a safety bubble, a PD controller
and a recovery state machine. It was slower and it got stuck more often.

## The decision

`car_step()` reads three rays:

| Ray | Meaning |
|---|---|
| `15` | straight ahead |
| `1` | near the left wall |
| `31` | near the right wall |

The forward distance selects a power tier and a steering-lock tier. The edge
rays select the sign of the lock. There is nothing between the scan and the
actuators except those two tables.

## Files

| File | Responsibility |
|---|---|
| `src/main.c` | Race loop, signal handling, `--verbose`, exit codes |
| `src/car.c` | Power and steering tables, `car_step()` |
| `src/protocol/link.c` | Unbuffered `write`, one line in, one line out |
| `src/protocol/response.c` | Parsing; the colon problem |
| `src/protocol/commands.c` | Typed commands, argument clamping, lap counting |
| `src/util/util.c` | Clamping, numeric parsing, logging |
| `pipe.sh` | The AI/API ring, same shape as the N4S launcher |

Headers in `include/` carry the interface documentation; the `.c` files carry
the reasoning. If you want to know *what* something does, read the header; if
you want to know *why it is like that*, read the source.

## The two invariants worth preserving

**Never buffer stdout.** The AI sits in a ring of pipes. A `printf` that stays
in libc's buffer while the AI waits for a reply is a deadlock with no error
message. Everything on stdout goes through `write(2)`.

**Inspect every response for checkpoint news.** Progress is piggy-backed on
the `ADDITIONAL_INFO` field of whatever reply happens to be next. Checking only
a dedicated command means missing it, and missing "Track Cleared" means the
car keeps driving after the race is over.
