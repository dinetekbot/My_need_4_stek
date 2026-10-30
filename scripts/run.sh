#!/usr/bin/env bash
#
# lidar-racer -- wire the AI to the simulator API and start driving.
#
# The processes form a ring, not a chain:
#
#     ai --stdout--> buffer --> n4s --> buffer --+
#      ^                                         |
#      +--------------- FIFO --------------------+
#
# A shell pipeline can only express a line, so the return path is a named
# pipe. The AI reads it on stdin and writes to stdout; the loop closes.
#
# The `buffer` helper exists because the simulator API writes its answers with
# a buffered stdio stream. Between two pipes that buffer is 4 kilobytes, so an
# answer would sit in memory until enough of them piled up -- while the AI
# waits for the first one, and the API waits for a command that the AI will
# not send until it is answered. Both sides block forever. The buffer process
# reads and forwards byte by byte, which breaks the deadlock.
#
set -euo pipefail

AI=${AI:-./ai}
N4S=${N4S:-./n4s}
BUFFER=${BUFFER:-./n4s_buffer}

usage() {
    cat >&2 <<'EOF'
usage: scripts/run.sh [-- <ai options>]

Environment:
  AI        path to the AI binary            (default: ./ai)
  N4S       path to the n4s binary           (default: ./n4s)
  BUFFER    path to the n4s_buffer binary    (default: ./n4s_buffer)
  N4S_DEBUG set to 1 to show the API's own stderr chatter

CoppeliaSim must already be running, with a track scene loaded and the ODE
physics engine selected. The API connects to it on 127.0.0.1:19997.

Examples:
  scripts/run.sh
  scripts/run.sh -- --verbose
  N4S=~/n4s_package/n4s BUFFER=~/n4s_package/n4s_buffer scripts/run.sh
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi
# Everything after `--` is forwarded to the AI untouched.
if [[ "${1:-}" == "--" ]]; then
    shift
fi

# Each missing piece gets the hint that actually applies to it. A single
# catch-all message sends you looking for the simulator API when all you
# needed was to run `make`.
missing=0

if [[ ! -x "$AI" ]]; then
    echo "error: the AI binary '$AI' is missing or not executable" >&2
    echo "       build it with:  make" >&2
    missing=1
fi

for binary in "$N4S" "$BUFFER"; do
    if [[ ! -x "$binary" ]]; then
        echo "error: the simulator API binary '$binary' is missing" >&2
        missing=1
    fi
done

if [[ ! -x "$N4S" || ! -x "$BUFFER" ]]; then
    echo "       n4s and n4s_buffer are not part of this repository." >&2
    echo "       Put them next to ./ai, or set N4S and BUFFER. See" >&2
    echo "       docs/05-simulator-setup.md." >&2
fi

if (( missing )); then
    exit 1
fi

# mktemp -u rather than a fixed name in the working directory: two runs on the
# same machine must not fight over the same pipe.
FIFO=$(mktemp -u /tmp/lidar-racer.XXXXXX.fifo)
mkfifo "$FIFO"
cleanup() {
    rm -f "$FIFO"
}
trap cleanup EXIT INT TERM

if [[ "${N4S_DEBUG:-0}" == "1" ]]; then
    API_STDERR=/dev/stderr
else
    # The API prints a "$>" prompt on stderr for every command it reads. Left
    # visible it drowns out the AI's own telemetry, which shares that stream.
    API_STDERR=/dev/null
fi

"$AI" "$@" < "$FIFO" \
    | "$BUFFER" \
    | "$N4S" 2> "$API_STDERR" \
    | "$BUFFER" > "$FIFO"
