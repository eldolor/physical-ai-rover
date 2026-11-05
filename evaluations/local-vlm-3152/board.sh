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
# crash_loop.py exits 0 (every bar met), 1 (valid, a bar missed), 2 (invalid,
# re-run once) or 3 (a harness error); result.json has the scoring.
set -uo pipefail
source tools/jetson/board_lock_check.sh
OUT="${1:?out dir}"; ARM="${2:?d30, d60 or d120}"; mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"
case "$ARM" in d30|d60|d120) ;; *) echo "STOP: arm must be d30, d60 or d120" >&2; exit 2;; esac
D="${ARM#d}"
UNIT=picar-local-vlm
# A runtime drop-in (amendment 2): a reboot clears it whatever happens here.
DROPIN="/run/systemd/system/$UNIT.service.d/zz-3152-crash-loop.conf"
PY_PID=
teardown() {
  # Python first (review 2): it lets the hog go and puts the unit back itself;
  # this is the net under it, run only once it has ended.
  if [ -n "$PY_PID" ] && kill -0 "$PY_PID" 2>/dev/null; then
    kill -TERM "$PY_PID" 2>/dev/null
    for _ in $(seq 1 480); do kill -0 "$PY_PID" 2>/dev/null || break; sleep 1; done
    kill -KILL "$PY_PID" 2>/dev/null
  fi
  pkill -KILL -f "evaluations/local-vlm-3152/hog.py" 2>/dev/null   # never left holding
  # The soak's own processes, if python could not end them (reviews 3-4) --
  # only once python's setup began, so a run refused at its checks (say, a
  # container of that name already there) touches nothing it did not start.
  if [ -e "$OUT/setup_started" ]; then
    docker rm -f "${SOAK_CONTAINER:-picar-ros-3135}" >/dev/null 2>&1
    pkill -TERM -f "tools/jetson/perception_loop.py" 2>/dev/null
    pkill -TERM -f "standin_cloud.py [0-9]+ $OUT" 2>/dev/null
    sudo -n pkill -TERM -f "memory_account.py sample $OUT" 2>/dev/null
    # The mission stack's own servers (stack.py's PORTS), started in their own sessions.
    for port in "${PICAR_ROBOT_PORT:-10350}" "${PICAR_BRAIN_PORT:-10351}" "${PICAR_BODY_PORT:-10352}" \
                "${PICAR_SENSOR_PORT:-10353}" "${PICAR_SENSOR2_PORT:-10354}"; do
      sudo -n fuser -k -TERM "$port/tcp" >/dev/null 2>&1
    done
  fi
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
# In the background, so a TERM/HUP/INT to this shell reaches python (which
# releases the hog first) before the teardown runs.
.venv/bin/python evaluations/local-vlm-3152/crash_loop.py "$OUT" "$D" >> "$OUT/run.log" 2>&1 &
PY_PID=$!
trap 'kill -TERM "$PY_PID" 2>/dev/null' TERM HUP INT
wait "$PY_PID"; rc=$?
while kill -0 "$PY_PID" 2>/dev/null; do wait "$PY_PID"; rc=$?; done
PY_PID=
gzip -f "$OUT/account.jsonl" "$OUT/perception.log" "$OUT/samples.jsonl" 2>/dev/null
echo "== $ARM exit $rc $(date +%T); MemAvailable $(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo) MB" | tee -a "$OUT/run.log"
exit $rc
