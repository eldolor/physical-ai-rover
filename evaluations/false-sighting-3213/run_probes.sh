#!/usr/bin/env bash
# 3.213.B's probes (C1, C2, C1+C2; both choosers) into OUT, under the shared desktop lock.
# bash evaluations/false-sighting-3213/run_probes.sh OUT
set -euo pipefail
cd "$(dirname "$0")/../.."
if [ -z "${PICAR_3213_LOCKED:-}" ]; then
  exec env PICAR_3213_LOCKED=1 python tools/desktop_lock.py shared --why "3.213.B probes" -- bash "$0" "$@"
fi
OUT="$1"; mkdir -p "$OUT"
export SWEEP_WORKERS="${SWEEP_WORKERS:-4}"
M=evaluations/false-sighting-3213/measure.py
for a in c1 c2 c1c2; do for c in rooms rule; do
  python $M "$OUT/${c}_${a}.json" --chooser $c --arm $a
done; done
echo DONE
