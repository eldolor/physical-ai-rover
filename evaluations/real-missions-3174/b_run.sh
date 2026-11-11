#!/usr/bin/env bash
# 3.174.B: run b_live.py's arm to its end, one batch (a fresh stack) a mission, 90 s apart.
# Run detached:  setsid nohup bash evaluations/real-missions-3174/b_run.sh <arm> OUT.jsonl > LOG 2>&1 < /dev/null &
cd "$(dirname "$0")/../.." || exit 1
eval "$(/home/anshu/vision-picar/.venv/bin/python tools/plan_section.py env 2>/dev/null)"
export ROS_IMAGE=vision-picar-ros:plan-3.156 PICAR_ROS_NETWORK=host
while true; do
  /home/anshu/vision-picar/.venv/bin/python evaluations/real-missions-3174/b_live.py "$2" --arm "$1" --batch-s 1
  rc=$?
  [ "$rc" = 3 ] || { echo "$1 exit=$rc $(date)"; exit "$rc"; }
  sleep 90
done
