#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
# evaluations/jetson-fmea-3135/queue.sh -- 3.135.B, on the DESKTOP: board rows one
# after another, each its own `run.py row` (its own board hold, released after it,
# so other sessions get the board between rows); a busy lock (exit 75) is retried.
#
#   bash evaluations/jetson-fmea-3135/queue.sh "F3f --runs 1" "F3f --runs 1 --plant vlm_absent" ...
set -u
cd "$(dirname "$0")/../.."
PY=../../../.venv/bin/python
[ -x "$PY" ] || PY=.venv/bin/python
for spec in "$@"; do
  echo "== $spec $(date +%T)"
  for try in $(seq 1 30); do
    # shellcheck disable=SC2086
    $PY evaluations/jetson-fmea-3135/run.py row $spec --wait 2>&1 | grep -v RuntimeWarning | tail -22
    rc=${PIPESTATUS[0]}
    [ "$rc" != 75 ] && break
    echo "board busy ($(date +%T)), retry in 120 s"; sleep 120
  done
done
echo "== queue done $(date +%T)"
