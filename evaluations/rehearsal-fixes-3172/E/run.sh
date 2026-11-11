#!/usr/bin/env bash
# 3.172.E: batches, one process each, so the exclusive desktop lock is released between them.
# Run detached:  setsid nohup bash evaluations/rehearsal-fixes-3172/E/run.sh OUT.jsonl > LOG 2>&1 < /dev/null &
cd "$(dirname "$0")/../../.." || exit 1
eval "$(/home/anshu/vision-picar/.venv/bin/python tools/plan_section.py env 2>/dev/null)"
export ROS_IMAGE=vision-picar-ros:plan-3.156 PICAR_ROS_NETWORK=host
out="$1"
while true; do
  /home/anshu/vision-picar/.venv/bin/python evaluations/rehearsal-fixes-3172/E/run.py "$out" --batch-s 4500
  rc=$?
  [ "$rc" = 3 ] || { echo "rehearsal exit=$rc $(date)"; exit "$rc"; }
  sleep 90    # a waiter (another session's live run) takes the lock here
done
