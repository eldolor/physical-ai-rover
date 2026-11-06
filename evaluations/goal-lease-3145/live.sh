#!/usr/bin/env bash
# 3.145's desktop live checks, in order (criteria 2, 3, 4). Timed: run under
# the exclusive desktop lock --
#   python tools/desktop_lock.py exclusive --why "3.145 live" -- bash evaluations/goal-lease-3145/live.sh
cd "$(dirname "$0")/../.." || exit 1
PY=${PY:-/home/anshu/vision-picar/.venv/bin/python}
"$PY" evaluations/goal-lease-3145/run.py rows --runs 3; echo "ROWS_RC=$?"
"$PY" evaluations/goal-lease-3145/run.py plant; echo "PLANT_RC=$?"
"$PY" evaluations/goal-lease-3145/run.py explore --runs 3; echo "EXPLORE_RC=$?"
