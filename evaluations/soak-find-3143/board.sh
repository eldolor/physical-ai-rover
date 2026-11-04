#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
# evaluations/soak-find-3143/board.sh -- 3.143's board arms, ON the board,
# from the bench checkout (~/bench-3143), under the board lock (`board.py`
# runs it through 3.136's board_run, with the lease and the twin's units
# stopped). The conditions are 3.135.C's soak's (soak.sh): the desktop off
# (multi-user.target), the twin's units stopped, the board's own VLM unit
# left running as found (VE resident: never held or restarted here).
#
#   bash evaluations/soak-find-3143/board.sh <out-dir> a3   # criterion 1's A3 (V1)
#   bash evaluations/soak-find-3143/board.sh <out-dir> v3   # criterion 3's V3
set -uo pipefail
# 3.93: not under another session's hold; keeps the board while it runs
source tools/jetson/board_lock_check.sh
OUT="${1:?out dir}"; ARM="${2:?a3 or v3}"; mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"
case "$ARM" in a3|v3) ;; *) echo "STOP: arm must be a3 or v3" >&2; exit 2;; esac
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
{ echo "== 3.143 $ARM $(date +%T) rev $(git rev-parse --short HEAD) image ${SOAK_IMAGE:-vision-picar-ros:latest}"
  echo "== $(sudo -n nvpmodel -q | head -1); boot $(uptime -s)"
  echo "== desktop: $(systemctl is-active gdm); twin units: $(systemctl is-active picar-stack) $(systemctl is-active picar-ngrok); vlm: $(systemctl is-active picar-local-vlm)"
  echo "== MemAvailable before anything: $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB"; } | tee "$OUT/run.log"
sudo -n nvpmodel -q 2>&1 | grep -m1 'NV Power Mode' > "$OUT/nvpmodel.txt"
echo "$(oom)" > "$OUT/oom_start"
if [ "$ARM" = a3 ]; then
  # The ladder in process with the frames rendered and VE the local tier,
  # reached as the stack reaches it (soak.py's VE_URL and the unit's secret).
  vars="$(.venv/bin/python - <<'EOF'
import shlex, sys
sys.path.insert(0, "evaluations/jetson-fmea-3135")
import soak
print("export LOCAL_VLM_URL=" + shlex.quote(soak.VE_URL))
print("export LOCAL_VLM_SECRET=" + shlex.quote(soak.read_vlm_secret()))
EOF
)" || { echo "STOP: reading VE's address and secret failed (see stderr)" | tee -a "$OUT/run.log"; exit 2; }
  eval "$vars"
  [ -n "${LOCAL_VLM_SECRET:-}" ] || { echo "STOP: no APP_SHARED_SECRET in the VLM's env file" | tee -a "$OUT/run.log"; exit 2; }
  # VE ready before (as Soak.setup asks), and the same process after: a restart
  # mid-ladder means some pairs ran without it.
  ve() { systemctl show picar-local-vlm -p MainPID -p NRestarts --value | tr '\n' ' '; }
  curl -s "$LOCAL_VLM_URL/health" > "$OUT/ve_health_before.json"
  grep -q '"ready": *true' "$OUT/ve_health_before.json" || { echo "STOP: VE is not ready: $(head -c 200 "$OUT/ve_health_before.json")" | tee -a "$OUT/run.log"; exit 2; }
  VE_BEFORE="$(ve)"; echo "$VE_BEFORE" > "$OUT/ve_before"
  .venv/bin/python evaluations/soak-find-3143/ladder.py A3 "$OUT/ladder-A3.json" > "$OUT/ladder-A3.log" 2>&1
  rc=$?
  curl -s "$LOCAL_VLM_URL/health" > "$OUT/ve_health_after.json"
  VE_AFTER="$(ve)"; echo "$VE_AFTER" > "$OUT/ve_after"
  # VE must have answered: a refused secret or a VE never asked sends every
  # question elsewhere while 3.98's geometry still confirms (review).
  .venv/bin/python - "$OUT/ladder-A3.json" <<'EOF' | tee -a "$OUT/run.log" || { [ "$rc" = 0 ] && rc=4; }
import json, sys
rows = json.load(open(sys.argv[1]))["rows"]
scored = sum(1 for r in rows for v in r.get("arrivals") or [] if isinstance(v[2], (int, float)))
print(f"== arrivals with a VE score: {scored}")
sys.exit(0 if scored else 1)
EOF
  if [ "$VE_BEFORE" != "$VE_AFTER" ]; then
    echo "== VE changed during A3 (MainPID NRestarts: $VE_BEFORE -> $VE_AFTER): the arm is void" | tee -a "$OUT/run.log"
    [ "$rc" = 0 ] && rc=3
  fi
else
  .venv/bin/python evaluations/soak-find-3143/v3.py "$OUT" >> "$OUT/run.log" 2>&1
  rc=$?
  gzip -f "$OUT/account.jsonl" "$OUT/perception.log" 2>/dev/null
fi
echo "== $ARM exit $rc $(date +%T); MemAvailable $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB" | tee -a "$OUT/run.log"
exit $rc
