#!/usr/bin/env bash
#
# lidar-racer -- wire the AI to the simulator API and start driving.
#
# This is the same ring the N4S launcher uses: the AI writes commands to the
# buffer, the buffer feeds the API, the API answers through another buffer,
# and a FIFO closes the loop back to the AI's stdin.
#
#     ai --stdout--> buffer --> n4s --> buffer --+
#      ^                                         |
#      +--------------- FIFO --------------------+
#
# Usage:
#   ./pipe.sh
#   ./pipe.sh -- --verbose
#
set -euo pipefail

AI=${AI:-./ai}
N4S=${N4S:-./n4s}
BUFFER=${BUFFER:-./n4s_buffer}

usage() {
    cat >&2 <<'EOF'
usage: ./pipe.sh [-- <ai options>]

Environment:
  AI        path to the AI binary              (default: ./ai)
  N4S       path to the n4s binary             (default: ./n4s)
  BUFFER    path to the n4s_buffer binary      (default: ./n4s_buffer)
  N4S_DEBUG set to 1 to keep the API's stderr

CoppeliaSim must already be running, with a track scene loaded and its
simulation stopped. The API connects to it on 127.0.0.1:19997.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi
# Everything after `--` goes to the AI untouched.
if [[ "${1:-}" == "--" ]]; then
    shift
fi

if [[ ! -x "$AI" ]]; then
    echo "error: the AI binary '$AI' is missing -- run: make" >&2
    exit 1
fi
if [[ ! -x "$N4S" || ! -x "$BUFFER" ]]; then
    echo "error: n4s/n4s_buffer not found in the current directory" >&2
    echo "       put them next to ./ai, or set N4S and BUFFER" >&2
    exit 1
fi

if [[ "${N4S_DEBUG:-0}" == "1" ]]; then
    API_STDERR=/dev/stderr
else
    # The API prints a "$>" prompt for every command; left visible it buries
    # the AI's own telemetry.
    API_STDERR=/dev/null
fi

FIFO=$(mktemp -u "${TMPDIR:-/tmp}/lidar-racer.XXXXXX.fifo")
mkfifo "$FIFO"
cleanup() {
    rm -f "$FIFO"
}
trap cleanup EXIT INT TERM

"$AI" "$@" < "$FIFO" \
    | "$BUFFER" \
    | "$N4S" 2> "$API_STDERR" \
    | "$BUFFER" > "$FIFO"
