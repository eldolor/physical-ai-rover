#!/usr/bin/env bash
# 3.197: N live runs of live.py in a row, each on a fresh maps volume and a
# fresh zone store (3.193's discarded run 1 reused both).
#   bash evaluations/nogo-3197/runs.sh IMAGE PREFIX FIRST LAST [TURN_DEG]
# writes evaluations/nogo-3197/PREFIX-<i>.json and PREFIX-<i>.log
set -uo pipefail
IMAGE="$1"; PREFIX="$2"; FIRST="$3"; LAST="$4"; TURN="${5:-}"
# TURN: SIM_START_TURN_DEG for the rotated runs (unset: 3.195's runs as they were)
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
cd "$ROOT"
PY="${PICAR_PY:-python}"
eval "$("$PY" tools/plan_section.py env)"
unset SIM_START_TURN_DEG
[ -n "$TURN" ] && export SIM_START_TURN_DEG="$TURN"
for i in $(seq "$FIRST" "$LAST"); do
    vol="picar-maps-3197-$PREFIX-$i"
    store="$(mktemp -d)"
    docker volume rm -f "$vol" >/dev/null 2>&1
    PICAR_ROS_NETWORK=host ROS_IMAGE="$IMAGE" PICAR_MAPS_VOLUME="$vol" NOGO_DIR="$store" \
        "$PY" evaluations/nogo-3197/live.py "$HERE/$PREFIX-$i.json" > "$HERE/$PREFIX-$i.log" 2>&1
    echo "run $PREFIX-$i exit $?: $(tail -2 "$HERE/$PREFIX-$i.log" | head -1 | cut -c1-300)"
    rm -rf "$store"
    docker volume rm -f "$vol" >/dev/null 2>&1      # review: no map volumes left behind
done
