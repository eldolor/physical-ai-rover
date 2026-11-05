#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
# evaluations/local-vlm-3152/board.sh -- 3.152 criterion 10's arms, ON the
# board, from the bench checkout (~/bench-3152), under the board lock
# (`board.py` runs it through 3.136's board_run, with the lease and the twin's
# units stopped). One candidate restart delay per arm: crash_loop.py puts a
# temporary drop-in on picar-local-vlm and takes it out again. The board's
# VLM must be active as found; it is put back active (its start counter reset)
# whatever happens. vlm_hold.sh is not used: the hold flag stops every start
# of the unit, and this arm measures those starts.
#
#   bash evaluations/local-vlm-3152/board.sh <out-dir> d30|d60|d120
# crash_loop.py exits 0 (every bar met), 1 (valid, a bar missed) or 2 (not a
# crash loop: invalid, re-run once); the result is result.json either way.
set -uo pipefail
source tools/jetson/board_lock_check.sh
OUT="${1:?out dir}"; ARM="${2:?d30, d60 or d120}"; mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"
case "$ARM" in d30|d60|d120) ;; *) echo "STOP: arm must be d30, d60 or d120" >&2; exit 2;; esac
D="${ARM#d}"
UNIT=picar-local-vlm
# A runtime drop-in (amendment 2): a reboot clears it whatever happens here.
DROPIN="/run/systemd/system/$UNIT.service.d/zz-3152-crash-loop.conf"
teardown() {
  # crash_loop.py puts the unit back itself; this is the net under it.
  if [ -e "$DROPIN" ]; then
    echo "== teardown: the drop-in was still there; removing it" | tee -a "$OUT/run.log"
    sudo -n rm -f "$DROPIN"; sudo -n systemctl daemon-reload
  fi
  sudo -n systemctl reset-failed "$UNIT"
  [ "$(systemctl is-active "$UNIT")" = active ] || sudo -n systemctl start "$UNIT"
  sudo -n dmesg | grep -iE "killed process|oom-kill|invoked oom-killer" > "$OUT/oom.log"
  sudo -n systemctl isolate "$FOUND_TARGET"
  systemctl show "$UNIT" -p StartLimitIntervalUSec -p StartLimitBurst -p RestartUSec -p ActiveState > "$OUT/unit_end.txt"
  echo "== torn down $(date +%T); desktop: $(systemctl is-active gdm); vlm: $(systemctl is-active "$UNIT")" | tee -a "$OUT/run.log"
}
[ "$(systemctl is-active "$UNIT")" = active ] || { echo "STOP: $UNIT is not active as found" | tee "$OUT/run.log"; exit 2; }
sudo -n true || { echo "STOP: no passwordless sudo on the board" | tee "$OUT/run.log"; exit 2; }
[ -e "$DROPIN" ] && { echo "STOP: $DROPIN exists: an earlier run was not put back" | tee "$OUT/run.log"; exit 2; }
FOUND_TARGET=multi-user.target
[ "$(systemctl is-active graphical.target)" = active ] && FOUND_TARGET=graphical.target
echo "$FOUND_TARGET" > "$OUT/found_target"
trap teardown EXIT
sudo -n systemctl isolate multi-user.target; sleep 5
sudo -n systemctl stop picar-stack picar-ngrok; sleep 3
{ echo "== 3.152 $ARM $(date +%T) rev $(git rev-parse --short HEAD) image ${SOAK_IMAGE:-vision-picar-ros:latest}"
  echo "== $(sudo -n nvpmodel -q | head -1); boot $(uptime -s); systemd $(systemctl --version | head -1)"
  echo "== desktop: $(systemctl is-active gdm); twin units: $(systemctl is-active picar-stack) $(systemctl is-active picar-ngrok); vlm: $(systemctl is-active "$UNIT")"
  echo "== MemAvailable before anything: $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB"; } | tee "$OUT/run.log"
.venv/bin/python evaluations/local-vlm-3152/crash_loop.py "$OUT" "$D" >> "$OUT/run.log" 2>&1
rc=$?
gzip -f "$OUT/account.jsonl" "$OUT/perception.log" "$OUT/samples.jsonl" 2>/dev/null
echo "== $ARM exit $rc $(date +%T); MemAvailable $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB" | tee -a "$OUT/run.log"
exit $rc
