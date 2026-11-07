#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
# evaluations/jetson-perf-3136/lat.sh -- 3.136.C's latencies (P7, P6, P5) ON
# the board, from the bench checkout (~/bench-3136), under the board lock
# (run.py's `lat` runs it through board_run with the twin's units stopped).
# The conditions are 3.135.C's soak's (soak.sh): the desktop off
# (multi-user.target), the twin's units stopped, the board's own VLM unit
# left running as found (VE resident), 15 W recorded. lat.py does the rest.
#
#   bash evaluations/jetson-perf-3136/lat.sh <out-dir> [lat.py args...]
set -uo pipefail
source tools/jetson/board_lock_check.sh
OUT="${1:?out dir}"; shift; mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"
oom() { sudo -n dmesg | grep -ciE "killed process" || true; }
teardown() {
  echo "$(oom)" > "$OUT/oom_end"
  sudo -n dmesg | grep -iE "killed process|oom-kill|invoked oom-killer" > "$OUT/oom.log"
  sudo -n systemctl isolate "$FOUND_TARGET"
  echo "== torn down $(date +%T); desktop: $(systemctl is-active gdm); vlm: $(systemctl is-active picar-local-vlm)" | tee -a "$OUT/run.log"
}
[ "$(systemctl is-active picar-local-vlm)" = active ] || { echo "STOP: picar-local-vlm is not active (VE must be resident as found)" | tee "$OUT/run.log"; exit 2; }
sudo -n true || { echo "STOP: no passwordless sudo on the board" | tee "$OUT/run.log"; exit 2; }
FOUND_TARGET=multi-user.target
[ "$(systemctl is-active graphical.target)" = active ] && FOUND_TARGET=graphical.target
echo "$FOUND_TARGET" > "$OUT/found_target"
trap teardown EXIT
sudo -n systemctl isolate multi-user.target; sleep 5
sudo -n systemctl stop picar-stack picar-ngrok; sleep 3
{ echo "== lat $(date +%T) rev $(git rev-parse --short HEAD) image ${LAT_IMAGE:-vision-picar-ros:latest} ($(docker image inspect -f '{{.Id}}' "${LAT_IMAGE:-vision-picar-ros:latest}" | cut -c8-19))"
  echo "== $(sudo -n nvpmodel -q | head -1); boot $(uptime -s); uptime $(cut -d' ' -f1 /proc/uptime) s"
  echo "== desktop: $(systemctl is-active gdm); twin units: $(systemctl is-active picar-stack) $(systemctl is-active picar-ngrok); vlm: $(systemctl is-active picar-local-vlm)"
  echo "== MemAvailable before anything: $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB"; } | tee "$OUT/run.log"
sudo -n nvpmodel -q 2>&1 | grep -m1 'NV Power Mode' > "$OUT/nvpmodel.txt"
uptime -s > "$OUT/boot"; cut -d' ' -f1 /proc/uptime > "$OUT/uptime_start"
echo "$(oom)" > "$OUT/oom_start"
.venv/bin/python evaluations/jetson-perf-3136/lat.py "$OUT" "$@" >> "$OUT/run.log" 2>&1
rc=$?
for f in "$OUT"/s*/truth.jsonl "$OUT"/s*/lat_shim.jsonl "$OUT"/s*/pace.jsonl "$OUT/perception.log"; do
  [ -f "$f" ] && gzip -f "$f"
done
echo "== lat.py exit $rc $(date +%T); MemAvailable $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB" | tee -a "$OUT/run.log"
exit $rc
