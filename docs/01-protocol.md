# 1. The wire protocol

The car is not driven through a library. It is driven by writing lines of text
on standard output and reading lines of text on standard input. That is the
whole interface.

```
    ai                          n4s (API)                  CoppeliaSim
     |  "GET_INFO_LIDAR\n"          |                           |
     | ---------------------------> |                           |
     |                              |  remote API call          |
     |                              | ------------------------> |
     |                              | <------------------------ |
     |  "1:OK:...:32 floats:..."    |                           |
     | <--------------------------- |                           |
```

The `n4s` binary is a translator. It reads our commands on its stdin, performs
the corresponding action in the simulator over a TCP connection to
`127.0.0.1:19997`, and writes the result on its stdout. It never initiates
anything: no command, no answer.

## Commands

Every command is one line terminated by `\n`. Commands are case insensitive.
Every command produces exactly one response line, including the ones that are
rejected.

| Command | Argument | Meaning |
|---|---|---|
| `START_SIMULATION` | — | Begin the run |
| `STOP_SIMULATION` | — | End the run |
| `CAR_FORWARD` | `float [0;1]` | Engine power forwards |
| `CAR_BACKWARDS` | `float [0;1]` | Engine power backwards |
| `WHEELS_DIR` | `float [-1;1]` | Steering angle |
| `GET_INFO_LIDAR` | — | 32 distances, millimetres |
| `GET_CURRENT_SPEED` | — | Current speed |
| `GET_CURRENT_WHEELS` | — | Current steering angle |
| `GET_CAR_SPEED_MAX` | — | Maximum speed (a constant) |
| `GET_CAR_SPEED_MIN` | — | Minimum speed, i.e. reverse (a constant) |
| `CYCLE_WAIT` | `int [0;INT_MAX]` | Let the physics run for N cycles |
| `GET_INFO_SIMTIME` | — | Time since `START_SIMULATION` |

`CAR_FORWARD` takes **engine power**, not a speed. `CAR_FORWARD:0.5` is half
throttle, and the speed that results from it depends on the slope, the
surface and how fast the car was already going.

## Responses

```
VALUE_ID : STATUS : CODE_STR [ : payload ] : ADDITIONAL_INFO
```

- `VALUE_ID` — numeric result code
- `STATUS` — `OK` or `KO`
- `CODE_STR` — the human-readable form of `VALUE_ID`
- payload — absent, one float, 32 floats, or a `[seconds,nanoseconds]` stamp
- `ADDITIONAL_INFO` — free text; `No further info` when there is nothing to say

Result codes worth branching on:

| ID | Meaning |
|---|---|
| 0 | Simulation has not been launched |
| 1 | No errors so far |
| 2 | Simulation running |
| 6 | Network operation failure — the API cannot reach CoppeliaSim |
| 9 | EOF reached |
| 10 | Simulation was correctly ended |
| 18 | Command not found |
| 19 | Simulation already up and running |
| 22 | Scene must contain at least 3 checkpoints |
| 25 | Failed to load map |

A `6` on `START_SIMULATION` almost always means CoppeliaSim is not running, or
is running without a scene loaded.

## Three traps

These are not hypothetical. Each one silently breaks a working-looking
implementation.

### 1. `ADDITIONAL_INFO` contains colons

Track progress is not reported by a dedicated command. It is piggy-backed on
the `ADDITIONAL_INFO` field of whatever response happens to come next:

```
1:OK:No errors so far:-25.1:First CP Cleared: [0][88s 26029102ns]
1:OK:No errors so far:[115s,559291238ns]:CP Cleared: [1][113s 756435254ns]
```

Two consequences.

First, the obvious parser — split the line on `:`, read field 3 as the value
and field 4 as the info — puts `First CP Cleared` in one field and
`[0][88s 26029102ns]` in another. Anything matching against the full message
finds nothing.

Second, and worse: that first line is the answer to `GET_CAR_SPEED_MIN`. The
news that the race has ended can arrive attached to a response you were not
paying attention to. `Track Cleared` missed this way means the car keeps
driving after the race is over.

This implementation parses positionally for the three mandatory fields,
consumes numeric fields greedily, and keeps everything after them as one
opaque string ([`response.c`](../src/protocol/response.c)). Every response
passes through `n4s_track_observe()` regardless of which command produced it.

The four progress messages:

| Message | Meaning |
|---|---|
| `First CP Cleared` | First checkpoint of the track |
| `CP Cleared` | Any intermediate checkpoint |
| `Lap Cleared` | One complete lap |
| `Track Cleared` | Race over — stop the car and end the simulation |

Note that `First CP Cleared` contains `CP Cleared` as a substring, so the
order in which a classifier tests them is load-bearing.

### 2. Buffered stdout deadlocks the pipeline

The processes are wired in a ring (see [`run.sh`](../scripts/run.sh)). C's
standard library buffers `stdout` in 4 KB blocks when it is a pipe rather than
a terminal. So:

- the AI calls `printf("GET_INFO_LIDAR\n")`; 15 bytes sit in a buffer
- the AI blocks reading the answer
- the API never received the command, so it sends nothing
- both wait forever

The program does not crash and prints no error. It just stops.

Two ways out: `write(2)`, or `setvbuf(stdout, NULL, _IONBF, 0)`. This project
uses `write(2)` throughout and never touches stdout through stdio, which makes
the rule impossible to break by accident. Note that stderr is not affected —
it is unbuffered by default, which is why telemetry goes there.

The `n4s_buffer` helper in the pipeline exists for the same reason on the
other side: it forwards the API's output byte by byte.

### 3. The API prints a prompt on stderr

`n4s` writes `$>` to stderr for every command it reads. Left visible it floods
the terminal and drowns out the AI's own telemetry, which shares that stream.
Redirecting the API's stderr to `/dev/null` — as `run.sh` does by default —
is the whole fix. Set `N4S_DEBUG=1` when you want to see it.

## Talking to it by hand

`n4s` runs standalone, which is the fastest way to check a protocol question:

```console
$ ./n4s
start_simulation
2:OK:Simulation running:No further info
get_info_lidar
1:OK:No errors so far:1203.4:1198.2:...:No further info
stop_simulation
10:OK:Simulation was correctly ended:No further info
```

`GET_CAR_SPEED_MAX` and `GET_CAR_SPEED_MIN` answer even when the simulation is
not running, which makes them a useful health check for the pipeline itself —
this is what [`probe_car()`](../src/main.c) uses them for at startup.

Next: [2. Architecture](02-architecture.md)
