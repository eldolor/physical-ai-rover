#!/usr/bin/env bash
# 3.195: N live runs of live.py in a row, each on a fresh maps volume and a
# fresh zone store (3.193's discarded run 1 reused both).
#   bash evaluations/nogo-3195/runs.sh IMAGE PREFIX FIRST LAST
# writes evaluations/nogo-3195/PREFIX-<i>.json and PREFIX-<i>.log
set -uo pipefail
IMAGE="$1"; PREFIX="$2"; FIRST="$3"; LAST="$4"
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
cd "$ROOT"
PY="${PICAR_PY:-python}"
eval "$("$PY" tools/plan_section.py env)"
for i in $(seq "$FIRST" "$LAST"); do
    vol="picar-maps-3195-$PREFIX-$i"
    store="$(mktemp -d)"
    docker volume rm -f "$vol" >/dev/null 2>&1
    PICAR_ROS_NETWORK=host ROS_IMAGE="$IMAGE" PICAR_MAPS_VOLUME="$vol" NOGO_DIR="$store" \
        "$PY" evaluations/nogo-3195/live.py "$HERE/$PREFIX-$i.json" > "$HERE/$PREFIX-$i.log" 2>&1
    echo "run $PREFIX-$i exit $?: $(tail -2 "$HERE/$PREFIX-$i.log" | head -1 | cut -c1-300)"
    rm -rf "$store"
    docker volume rm -f "$vol" >/dev/null 2>&1      # review: no map volumes left behind
done
