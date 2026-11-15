#!/bin/bash
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
#
# 3.183.A: bisect dev's first-parent history for the first commit whose 9
# in-process `spacing` surveys' label calls differ from PIN.
#
#   bash evaluations/small-fixes-3183/a_bisect.sh GOOD_SHA END_SHA 'PIN' LOG
#
# GOOD_SHA must give PIN (it is run first, and the search stops if it does
# not); END_SHA is the newest commit searched. Each commit runs in a worktree
# of its own (a_counts.py, from this directory). A run that gives no result
# (a crash, a timeout, a worktree that could not be made) stops the search:
# it is never counted as "changed". The two runs of 2026-11-15:
#   a_bisect.sh 528b0190 6b46ec32 '[6, 6, 6, 33, 29, 28, 42, 44, 37]' a_bisect1.log
#   a_bisect.sh 67b775d3 6b46ec32 '[6, 6, 6, 36, 31, 31, 42, 44, 37]' a_bisect2.log
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
GOOD=$1; END=$2; PIN=$3; LOG=$4
PY=${PY:-python}
TMP=$(mktemp -d)
mapfile -t C < <(git log --first-parent --format=%H "$GOOD..$END" | tac)
run() {
  local w="$TMP/wt-${1:0:8}"
  git worktree add -q --detach "$w" "$1" || { echo "worktree failed for $1" >&2; exit 2; }
  local out; out=$(cd "$w" && timeout 900 "$PY" "$HERE/a_counts.py" | tail -1)
  git worktree remove --force "$w"
  [[ "$out" == *label_calls* ]] || { echo "no result for $1 (crash or timeout)" >&2; exit 2; }
  echo "${1:0:8} $out" >> "$LOG"; echo "$out"
}
good() { [[ "$1" == *"\"label_calls\": $PIN"* ]]; }
r=$(run "$GOOD") || exit 2      # an exit inside $( ) only leaves the subshell
good "$r" || { echo "GOOD_SHA $GOOD does not give PIN" >&2; exit 2; }
lo=-1; hi=${#C[@]}
while (( hi - lo > 1 )); do
  mid=$(( (lo + hi) / 2 )); r=$(run "${C[$mid]}") || exit 2
  if good "$r"; then lo=$mid; else hi=$mid; fi
done
(( hi < ${#C[@]} )) && echo "FIRST BAD: ${C[$hi]}" >> "$LOG" || echo "NONE CHANGED" >> "$LOG"
rmdir "$TMP" 2>/dev/null
