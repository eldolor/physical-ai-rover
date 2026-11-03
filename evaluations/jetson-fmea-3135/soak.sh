#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
# evaluations/jetson-fmea-3135/soak.sh -- 3.135.C's memory soak (criterion 1)
# ON the board, from the bench checkout (~/bench-3135), under the board lock
# (run.py's `soak` runs it through 3.136's board_run with the twin's units
# stopped). The conditions are 3.136.B's steady window's (steady.sh): the
# desktop off (multi-user.target), the twin's units stopped, the board's own
# VLM unit left running as found (VE resident: never held or restarted here),
# 15 W recorded. soak.py does the missions; amendment 3 in the section says
# what each choice means for the bars.
#
#   bash evaluations/jetson-fmea-3135/soak.sh <out-dir> [soak.py args...]
set -uo pipefail
# 3.93: not under another session's hold; keeps the board while it runs
source tools/jetson/board_lock_check.sh
OUT="${1:?out dir}"; shift; mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"
oom() { sudo -n dmesg | grep -ciE "killed process" || true; }
teardown() {
  echo "$(oom)" > "$OUT/oom_end"
  sudo -n dmesg | grep -iE "killed process|oom-kill|invoked oom-killer" > "$OUT/oom.log"
  # Back to the target the board was found in (3.136's isolate assumes graphical).
  sudo -n systemctl isolate "$FOUND_TARGET"
  echo "== torn down $(date +%T); desktop: $(systemctl is-active gdm); vlm: $(systemctl is-active picar-local-vlm)" | tee -a "$OUT/run.log"
}
[ "$(systemctl is-active picar-local-vlm)" = active ] || { echo "STOP: picar-local-vlm is not active (VE must be resident as found)" | tee "$OUT/run.log"; exit 2; }
sudo -n true || { echo "STOP: no passwordless sudo on the board" | tee "$OUT/run.log"; exit 2; }
FOUND_TARGET=multi-user.target
[ "$(systemctl is-active graphical.target)" = active ] && FOUND_TARGET=graphical.target
echo "$FOUND_TARGET" > "$OUT/found_target"
trap teardown EXIT
# The isolate starts every enabled unit of multi-user (the twin's): stop them after it.
sudo -n systemctl isolate multi-user.target; sleep 5
sudo -n systemctl stop picar-stack picar-ngrok; sleep 3
{ echo "== soak $(date +%T) rev $(git rev-parse --short HEAD) image ${SOAK_IMAGE:-vision-picar-ros:latest}"
  echo "== $(sudo -n nvpmodel -q | head -1); boot $(uptime -s); uptime $(cut -d' ' -f1 /proc/uptime) s"
  echo "== desktop: $(systemctl is-active gdm); twin units: $(systemctl is-active picar-stack) $(systemctl is-active picar-ngrok); vlm: $(systemctl is-active picar-local-vlm)"
  echo "== MemAvailable before anything: $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB"; } | tee "$OUT/run.log"
sudo -n nvpmodel -q 2>&1 | grep -m1 'NV Power Mode' > "$OUT/nvpmodel.txt"
echo "$(oom)" > "$OUT/oom_start"
.venv/bin/python evaluations/jetson-fmea-3135/soak.py "$OUT" "$@" >> "$OUT/run.log" 2>&1
rc=$?
# The big logs gzipped before the fetch (the scorer reads either form).
gzip -f "$OUT/account.jsonl" "$OUT/perception.log" 2>/dev/null
echo "== soak.py exit $rc $(date +%T); MemAvailable $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB" | tee -a "$OUT/run.log"
exit $rc
