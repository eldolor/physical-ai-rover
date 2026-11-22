#!/usr/bin/env bash
# 3.213.B's bars from the checkout given (the branch, or a `dev` control with
# this directory copied in), into OUT, under the shared desktop lock.
# bash evaluations/false-sighting-3213/run_bars.sh CHECKOUT OUT
set -euo pipefail
SELF="$(realpath "$0")"; CHECKOUT="$(realpath "$1")"; mkdir -p "$2"; OUT="$(realpath "$2")"   # review
if [ -z "${PICAR_3213_LOCKED:-}" ]; then
  cd "$(dirname "$SELF")/../.."
  exec env PICAR_3213_LOCKED=1 python tools/desktop_lock.py shared --why "3.213.B bars" -- bash "$SELF" "$CHECKOUT" "$OUT"
fi
cd "$CHECKOUT"
export SWEEP_WORKERS="${SWEEP_WORKERS:-4}"
B=evaluations/false-sighting-3213/bars_find.py
python $B find "$OUT/find_rooms.json" --chooser rooms
python $B find "$OUT/find_rule.json"
python $B survey "$OUT/survey_rooms.json" --chooser rooms
python $B survey "$OUT/survey_rule.json"
python evaluations/false-sighting-3213/measure.py "$OUT/broad_rooms.json" --chooser rooms --arm dev
python evaluations/false-sighting-3213/measure.py "$OUT/broad_rule.json" --chooser rule --arm dev
W=$(mktemp -d); trap 'rm -rf "$W"' EXIT
python evaluations/find-last-seen-370/stale.py run "$OUT/rows-m3.jsonl" "$W/m3" "$SWEEP_WORKERS"
python evaluations/find-last-seen-370/sweep.py run "$OUT/rows-m2.jsonl" "$W/m2" "$SWEEP_WORKERS" fake
echo DONE
