# 2. Architecture

## The layers

```
                    ┌─────────────────────────────────────────┐
   src/main.c       │  race loop, signals, exit codes          │
                    └────────────────┬────────────────────────┘
                                     │
        ┌────────────────────────────┼────────────────────────────┐
        │                            │                            │
┌───────▼────────┐          ┌────────▼─────────┐        ┌─────────▼────────┐
│  src/protocol  │          │ src/perception   │        │   src/control    │
│                │          │                  │        │                  │
│ link.c    I/O  │          │ perception.c     │        │ control.c        │
│ response.c     │          │                  │        │                  │
│ commands.c     │          │ scan → gap       │        │ gap → commands   │
└────────────────┘          └──────────────────┘        └──────────────────┘
        │                            │                            │
        └────────────────────────────┴────────────────────────────┘
                                     │
                          ┌──────────▼──────────┐
                          │ src/util, src/config │
                          └─────────────────────┘
```

The dependency direction is strict: perception and control know nothing about
the protocol. They take arrays of doubles and return numbers. That is what
makes them testable without a simulator, and it is the difference between a
project you can debug and one you can only stare at.

## The cycle

One iteration of the race loop, in full:

```c
n4s_get_lidar(raw);                              /* 1 round trip  */
scan_build(&scan, raw, &cfg);                    /* clean, smooth */
if (scan_is_faulty(&scan, &cfg)) { ... }         /* sensor guard  */
scan_apply_bubble(&scan, &cfg);                  /* safety        */
scan_find_gap(&scan, &cfg, &gap);                /* where to go   */
controller_step(&ctl, &scan, &gap, &cfg, &cmd);  /* what to do    */
apply_command(&cmd);                             /* 3 round trips */
```

Four round trips, every cycle, without exception. Even when the reverse
command is zero and redundant, it is sent.

That is a deliberate cost. It buys a **constant loop period**, and a constant
period is what lets the derivative term of the steering controller be written
as `error - last_error` rather than `(error - last_error) / measured_dt`. A
jittery `dt` in a derivative term produces jittery steering; measuring `dt`
accurately over a pipe is not worth the trouble when you can simply make the
loop regular instead.

It is also where the largest performance win lives. The temptation is to
refresh everything the car knows on every cycle:

```
GET_INFO_LIDAR, GET_CURRENT_SPEED, GET_CURRENT_WHEELS,
GET_CAR_SPEED_MAX, GET_CAR_SPEED_MIN            ← two constants, re-read forever
WHEELS_DIR, CAR_FORWARD
```

Seven round trips instead of four, two of them for numbers that never change.
The car reacts nearly half as often, which on a track shows up as
understeering into corners it could have taken. Constants are read once at
startup, in [`probe_car()`](../src/main.c).

## Files

| File | Responsibility |
|---|---|
| `src/main.c` | Race loop, signal handling, exit codes |
| `src/config.c` | Defaults and the option table |
| `src/protocol/link.c` | Unbuffered `write`, one line in, one line out |
| `src/protocol/response.c` | Parsing; the colon problem |
| `src/protocol/commands.c` | Typed commands, argument clamping, lap counting |
| `src/perception/perception.c` | Scan cleaning, safety bubble, gap search |
| `src/control/control.c` | Steering law, throttle law, state machine |
| `src/util/util.c` | Clamping, mapping, rate limiting, numeric parsing |

Headers in `include/` carry the interface documentation; the `.c` files carry
the reasoning. If you want to know *what* something does, read the header; if
you want to know *why it is like that*, read the source.

## The state machine

```
                    forward < reverse_dist
       ┌──────────────────────────────────────────┐
       │                                          ▼
  ┌────┴─────┐                            ┌───────────────┐
  │  RACING  │                            │   REVERSING   │
  └────┬─────┘◄───────────────────────────┴───────┬───────┘
       │        forward > reverse_dist × 1.8      │
       │                                          │
       │ no gap for deadend_cycles                │ no gap, and
       │                                          │ enough clearance
       ▼                                          │
  ┌───────────┐◄──────────────────────────────────┘
  │  HALTING  │  back off until 1m of clearance, then settle
  └────┬──────┘
       │
       ▼
  ┌────────────┐
  │  FINISHED  │  ← also entered directly on "Track Cleared" or lap limit
  └────────────┘
```

Two details that are easy to get wrong:

**Hysteresis on the reverse exit.** Leaving `REVERSING` at exactly the
distance that entered it makes the car shuffle forwards and backwards on the
threshold forever. The exit distance is 1.8× the entry distance.

**Clearance before halting.** The rules require the car to come to rest at
least three feet from the wall before ending the simulation. `HALTING` keeps
backing up until it has that clearance, then holds still for a few cycles to
be sure it has actually stopped, and only then declares itself finished. A car
that halts with its bumper against the wall has failed the run even though it
did stop.

On the cycle where a transition happens, the command sent is all zeros — the
car coasts for one cycle rather than acting on a decision made by the state it
is leaving.

## Failure handling

| Situation | Response |
|---|---|
| Unparseable response line | Treated as a failed request |
| Lidar reply with fewer than 32 values | Cycle skipped; driving on a partial scan is worse than not driving |
| Every ray reads as faulty | Cycle skipped, throttle zeroed — this is a broken sensor, not a car in a box |
| 100 consecutive bad reads | Give up, exit 84 |
| Pipe closed | `SIGPIPE` ignored, so it surfaces as a write error we can report |
| `SIGINT` / `SIGTERM` | Leave the loop, stop the car, end the simulation cleanly |

Exit code is `0` when the race ends cleanly and `84` on any error.

The important asymmetry: **a faulty reading collapses to "obstacle right
here", never to "open road"**. Every ambiguous case resolves towards braking.
A sensor glitch that reads as clear road at full throttle is how you put a car
into a wall.

Next: [3. Perception](03-perception.md)
