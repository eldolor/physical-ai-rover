#!/bin/bash
# Binary search over dev's first-parent commits (oldest first) for the first whose
# 9 spacing surveys' label calls differ from the 3.120 pin. One worktree per commit.
S=$(dirname "$0"); PY=/home/user/physical-ai-rover/.venv/bin/python
PIN='[6, 6, 6, 33, 29, 28, 42, 44, 37]'
mapfile -t C < "$S/commits.txt"
run() {  # $1 = sha -> prints the json, logs it
  local w="$S/wt-${1:0:8}"
  git worktree add -q --detach "$w" "$1" 2>/dev/null
  local out; out=$(cd "$w" && timeout 900 "$PY" "$S/counts.py" 2>"$S/err-${1:0:8}.log" | tail -1)
  git worktree remove --force "$w"
  echo "${1:0:8} $out" >> "$S/bisect.log"; echo "$out"
}
good() { [[ "$1" == *"\"label_calls\": $PIN"* ]]; }
lo=-1; hi=${#C[@]}   # C[lo] good (528b0190's child side), C[hi] bad (head)
echo "base 528b0190 $(run 528b0190)" >> "$S/bisect.log"
while (( hi - lo > 1 )); do
  mid=$(( (lo + hi) / 2 )); r=$(run "${C[$mid]}")
  if good "$r"; then lo=$mid; else hi=$mid; fi
done
echo "FIRST BAD: ${C[$hi]}" >> "$S/bisect.log"
