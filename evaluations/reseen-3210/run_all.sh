#!/usr/bin/env bash
# 3.210.B's runs, from the checkout given (branch or dev control), into OUT.
# bash evaluations/reseen-3210/run_all.sh CHECKOUT OUT   (takes the shared desktop lock itself)
set -euo pipefail
if [ -z "${PICAR_3210_LOCKED:-}" ]; then   # review: the shared desktop lock, always
  cd "$(dirname "$0")/../.."
  exec env PICAR_3210_LOCKED=1 python tools/desktop_lock.py shared --why "3.210.B sweeps" -- bash "$0" "$@"
fi
cd "$1"; OUT="$2"; mkdir -p "$OUT"
export SWEEP_WORKERS="${SWEEP_WORKERS:-4}"
F=evaluations/nogo-3192/fix/sweep_c1.py
python $F find "$OUT/find_rooms.json" --chooser rooms
python $F find "$OUT/find_rule.json"
python evaluations/nogo-3192/sweep2.py survey "$OUT/survey_rooms.json" --chooser rooms
python evaluations/nogo-3192/sweep2.py survey "$OUT/survey_rule.json"
python evaluations/reseen-3210/broad.py "$OUT/broad_rooms.json" --chooser rooms
python evaluations/reseen-3210/broad.py "$OUT/broad_rule.json" --chooser rule
W=$(mktemp -d); trap 'rm -rf "$W"' EXIT
python evaluations/find-last-seen-370/stale.py run "$OUT/rows-m3.jsonl" "$W/m3" "$SWEEP_WORKERS"
python evaluations/find-last-seen-370/sweep.py run "$OUT/rows-m2.jsonl" "$W/m2" "$SWEEP_WORKERS" fake
echo DONE
