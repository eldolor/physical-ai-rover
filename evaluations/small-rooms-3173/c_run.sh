#!/usr/bin/env bash
# 3.173.C: one survey a batch, 90 s apart (the lock is released between them).
# Run detached:  setsid nohup bash evaluations/small-rooms-3173/c_run.sh OUT.jsonl > LOG 2>&1 < /dev/null &
cd "$(dirname "$0")/../.." || exit 1
eval "$(/home/anshu/vision-picar/.venv/bin/python tools/plan_section.py env 2>/dev/null)"
export ROS_IMAGE=vision-picar-ros:plan-3.156 PICAR_ROS_NETWORK=host
while true; do
  /home/anshu/vision-picar/.venv/bin/python evaluations/small-rooms-3173/c_surveys.py "$1" --batch-s 1
  rc=$?
  [ "$rc" = 3 ] || { echo "surveys exit=$rc $(date)"; exit "$rc"; }
  sleep 90
done
