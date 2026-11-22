#!/usr/bin/env bash
# 3.213.A's four sweeps (both choosers x dev/oracle) into OUT, under the shared desktop lock.
# bash evaluations/false-sighting-3213/run_all.sh OUT
set -euo pipefail
cd "$(dirname "$0")/../.."
if [ -z "${PICAR_3213_LOCKED:-}" ]; then
  exec env PICAR_3213_LOCKED=1 python tools/desktop_lock.py shared --why "3.213.A sweeps" -- bash "$0" "$@"
fi
OUT="$1"; mkdir -p "$OUT"
export SWEEP_WORKERS="${SWEEP_WORKERS:-4}"
M=evaluations/false-sighting-3213/measure.py
for c in rooms rule; do for a in dev oracle; do
  python $M "$OUT/${c}_${a}.json" --chooser $c --arm $a
done; done
echo DONE
