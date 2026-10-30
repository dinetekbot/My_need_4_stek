#!/usr/bin/env bash
#
# lidar-racer -- one command: start the simulator on a scene and race it.
#
# Replaces a five-step ritual that is easy to get subtly wrong: leaving a
# second CoppeliaSim running so the two fight over port 19997, launching from
# the wrong directory, forgetting that headless ignores the scene argument, or
# racing against a scene whose lidar does not work.
#
#   scripts/simulate.sh bootstrap_track                 # headless (reliable)
#   scripts/simulate.sh --gui criss_cross               # windowed (watchable)
#   scripts/simulate.sh track_3 -- --gap-threshold=600  # options for the AI
#
set -euo pipefail

MODE=headless
SCENE=""
AI_ARGS=()

COPPELIASIM_ROOT=${COPPELIASIM_ROOT:-$HOME/opt/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04}
SCENE_DIR=${SCENE_DIR:-$HOME/opt/n4s-scenes}

usage() {
    cat >&2 <<'EOF'
usage: scripts/simulate.sh [--gui|--headless] <scene> [-- <ai options>]

  <scene>   a scene name (looked up in $SCENE_DIR) or a path to a .ttt file

Options:
  --gui         run CoppeliaSim with its window, so you can watch
  --headless    run without a window (default, and the dependable one)
  --keep        do not kill CoppeliaSim instances that are already running
  -h, --help    this message

Environment:
  COPPELIASIM_ROOT  CoppeliaSim install directory
  SCENE_DIR         where the .ttt scenes live   (default: ~/opt/n4s-scenes)
  N4S               path to the n4s binary       (default: ./n4s)
  BUFFER            path to the n4s_buffer binary (default: ./n4s_buffer)

Headless is the recommended mode. The windowed one is useful for watching the
car, but it has been observed to drive without making progress on a scene that
headless clears immediately afterwards -- see docs/05-simulator-setup.md.
EOF
}

KEEP=0
while (( $# )); do
    case "$1" in
        --gui)      MODE=gui; shift ;;
        --headless) MODE=headless; shift ;;
        --keep)     KEEP=1; shift ;;
        -h|--help)  usage; exit 0 ;;
        --)         shift; AI_ARGS=("$@"); break ;;
        -*)         echo "error: unknown option '$1'" >&2; usage; exit 1 ;;
        *)          if [[ -n "$SCENE" ]]; then
                        echo "error: more than one scene given" >&2; exit 1
                    fi
                    SCENE="$1"; shift ;;
    esac
done

if [[ -z "$SCENE" ]]; then
    usage
    exit 1
fi

# Accept either a bare name or a path.
if [[ -f "$SCENE" ]]; then
    SCENE_PATH=$(readlink -f "$SCENE")
elif [[ -f "$SCENE_DIR/$SCENE.ttt" ]]; then
    SCENE_PATH=$(readlink -f "$SCENE_DIR/$SCENE.ttt")
elif [[ -f "$SCENE_DIR/$SCENE" ]]; then
    SCENE_PATH=$(readlink -f "$SCENE_DIR/$SCENE")
else
    echo "error: no scene '$SCENE' here or in $SCENE_DIR" >&2
    echo "available:" >&2
    ls -1 "$SCENE_DIR"/*.ttt 2>/dev/null | xargs -rn1 basename >&2 || true
    exit 1
fi

# CoppeliaSim's own launcher does not quote the scene argument it forwards, so
# a path with a space arrives as two arguments and is silently ignored.
if [[ "$SCENE_PATH" == *" "* ]]; then
    echo "error: the scene path contains a space:" >&2
    echo "       $SCENE_PATH" >&2
    echo "       CoppeliaSim mishandles those. Copy it somewhere without." >&2
    exit 1
fi

if [[ ! -x ./ai ]]; then
    echo "error: ./ai is missing -- run: make" >&2
    exit 1
fi

port_open() {
    timeout 2 bash -c '</dev/tcp/127.0.0.1/19997' 2>/dev/null
}

# One simulator at a time. Two instances race for port 19997 and the API then
# talks to whichever won, which may not be the one holding your scene.
#
# And wait for the port to actually close before launching the replacement.
# A dying instance keeps its listening socket for a moment, so a naive "is the
# port up?" check answers yes, connects to the corpse, and the next check two
# seconds later fails -- which looks exactly like a simulator that refused to
# start.
if (( ! KEEP )) && pgrep -x coppeliaSim >/dev/null; then
    echo "stopping $(pgrep -cx coppeliaSim) CoppeliaSim instance(s) already running"
    pkill -9 -x coppeliaSim || true
    for _ in $(seq 1 30); do
        pgrep -x coppeliaSim >/dev/null || port_open || break
        sleep 1
    done
    sleep 2
fi

echo "starting CoppeliaSim ($MODE) on $(basename "$SCENE_PATH")"
if [[ "$MODE" == gui ]]; then
    ( cd "$COPPELIASIM_ROOT" && setsid ./coppeliaSim.sh "$SCENE_PATH" \
        > /tmp/lidar-racer-sim.log 2>&1 < /dev/null & )
else
    # Headless ignores the scene argument, so it is loaded over the API below.
    ( cd "$COPPELIASIM_ROOT" && setsid ./coppeliaSim.sh -h \
        > /tmp/lidar-racer-sim.log 2>&1 < /dev/null & )
fi

# Require the port to answer twice in a row: once is not proof that the
# instance we just started is the one answering.
printf 'waiting for the remote API on 127.0.0.1:19997'
ready=0
for _ in $(seq 1 60); do
    if port_open && sleep 1 && port_open; then
        ready=1
        break
    fi
    printf '.'
    sleep 2
done
echo
if (( ! ready )); then
    echo "error: CoppeliaSim never opened its port. See /tmp/lidar-racer-sim.log" >&2
    exit 1
fi
# Give the scene time to settle before touching it.
sleep 12

# Headless loads the scene; the windowed one already has it and refuses a
# remote load, so there we only verify what is open.
if [[ "$MODE" == gui ]]; then
    COPPELIASIM_ROOT="$COPPELIASIM_ROOT" "$(dirname "$0")/load_scene.py" --check
else
    COPPELIASIM_ROOT="$COPPELIASIM_ROOT" "$(dirname "$0")/load_scene.py" "$SCENE_PATH"
fi

echo
"$(dirname "$0")/run.sh" -- "${AI_ARGS[@]}"
