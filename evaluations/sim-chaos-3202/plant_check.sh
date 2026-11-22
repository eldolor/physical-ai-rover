#!/usr/bin/env bash
# 3.202 criterion 7: each injector once against its planted build, and once
# shipped (to say whether it discriminates), each on its own stack; two at a
# time on two port blocks. Results: <out>/<kind>-<plant|shipped>/score.json.
#   bash evaluations/sim-chaos-3202/plant_check.sh <out>
set -u
OUT=${1:?out dir}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
run() {  # kind plant base
  local k=$1 p=$2 b=$3 d="$OUT/$1-${2:-shipped}"
  [ -e "$d" ] && mv "$d" "$d.old.$$"
  PICAR_ROBOT_PORT=$b PICAR_BRAIN_PORT=$((b+1)) PICAR_BODY_PORT=$((b+2)) \
  PICAR_SENSOR_PORT=$((b+3)) PICAR_SENSOR2_PORT=$((b+4)) \
    timeout 2400 python "$HERE/chaos.py" "$d" --faults 1 --kinds "$k" --gap 5,10 --stop-last \
      --house scaled_house ${p:+--plant $p} > "$d.out" 2>&1
  python "$HERE/score.py" "$d" > "$d.score.txt" 2>&1
  echo "$k ${p:-shipped}: $(head -1 "$d.score.txt")"
}
lane() {  # base, then kind:plant pairs
  local b=$1; shift
  for kp in "$@"; do run "${kp%%:*}" "${kp#*:}" "$b"; done
}
lane 11400 K1:watchdog K1: K6:watchdog K6: K11:watchdog K11: K10:cloud_deadline K10: &
lane 11410 K3:heartbeat K3: K4:lidar_fresh K4: K5:feedback_fresh K5: K2:watchdog K9: &
wait
