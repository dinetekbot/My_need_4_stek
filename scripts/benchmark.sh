#!/usr/bin/env bash
#
# lidar-racer -- run the same race N times and report the distribution.
#
# This exists because single runs lie. The simulator is not deterministic:
# the same binary, the same scene and the same calibration produced three
# cleared runs in a row and, an hour later, six failures in a row. Any A/B
# comparison on two or three runs is measuring noise.
#
#   scripts/benchmark.sh 10 bootstrap_track
#   scripts/benchmark.sh 10 bootstrap_track -- --command-deadband=0
#
# Compare two configurations by running this twice and looking at the
# cleared rate, not at any individual run.
#
set -euo pipefail

RUNS=${1:-10}
SCENE=${2:-bootstrap_track}
shift 2 2>/dev/null || true
[[ "${1:-}" == "--" ]] && shift || true
AI_ARGS="$*"

COPPELIASIM_ROOT=${COPPELIASIM_ROOT:-$HOME/opt/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04}
SCENE_DIR=${SCENE_DIR:-$HOME/opt/n4s-scenes}
TIMEOUT=${TIMEOUT:-170}

here=$(dirname "$0")
scene_path="$SCENE_DIR/$SCENE.ttt"
[[ -f "$scene_path" ]] || scene_path="$SCENE"
[[ -f "$scene_path" ]] || { echo "error: no scene $SCENE" >&2; exit 1; }
[[ -x ./ai ]] || { echo "error: ./ai missing -- run make" >&2; exit 1; }

if ! timeout 2 bash -c '</dev/tcp/127.0.0.1/19997' 2>/dev/null; then
    echo "error: CoppeliaSim is not listening on 19997." >&2
    echo "       start it first, e.g. scripts/simulate.sh --help" >&2
    exit 1
fi

log_dir=$(mktemp -d /tmp/lidar-racer-bench.XXXXXX)
cleared=0
declare -a cycles=()

echo "scene    $SCENE"
echo "options  ${AI_ARGS:-(defaults)}"
echo "runs     $RUNS"
echo

for ((run = 1; run <= RUNS; run++)); do
    # A failed load is not something to shrug at: the previous scene stays
    # loaded and every remaining run silently races the wrong track. That
    # happened, and produced two benchmarks of a scene that was never loaded.
    if ! COPPELIASIM_ROOT="$COPPELIASIM_ROOT" "$here/load_scene.py" \
            "$scene_path" >"$log_dir/load$run.txt" 2>&1; then
        echo "error: could not load $SCENE before run $run" >&2
        sed 's/^/       /' "$log_dir/load$run.txt" >&2
        exit 1
    fi
    sleep 2
    log="$log_dir/run$run.log"
    AI_OPTS="--verbose $AI_ARGS" \
        timeout "$TIMEOUT" "$here/run.sh" -- 2>"$log" || true
    n=$(grep -c '^\[0' "$log" || true)
    ok=$(grep -c 'TRACK CLEARED' "$log" || true)
    cycles+=("$n")
    (( ok > 0 )) && cleared=$((cleared + 1)) || true
    printf 'run %2d  cycles %-5s %s\n' "$run" "$n" \
        "$( ((ok > 0)) && echo cleared || echo '-')"
done

echo
printf 'cleared  %d/%d  (%.0f%%)\n' "$cleared" "$RUNS" \
    "$(echo "$cleared $RUNS" | awk '{print 100*$1/$2}')"
printf 'cycles   min %s  median %s  max %s\n' \
    "$(printf '%s\n' "${cycles[@]}" | sort -n | head -1)" \
    "$(printf '%s\n' "${cycles[@]}" | sort -n | awk '{a[NR]=$1} END{print a[int((NR+1)/2)]}')" \
    "$(printf '%s\n' "${cycles[@]}" | sort -n | tail -1)"
echo "logs     $log_dir"
echo
echo "A difference in cleared rate below roughly 3 out of 10 is not"
echo "distinguishable from noise at this sample size."
