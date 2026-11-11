#!/usr/bin/env bash
# 3.172.E: batches, one process each, so the exclusive desktop lock is released between them.
# Run detached:  setsid nohup bash evaluations/rehearsal-fixes-3172/E/run.sh OUT.jsonl > LOG 2>&1 < /dev/null &
# Then (amendment 2) mission 9 once more into E/rerun9.jsonl, once the load is under 4.
cd "$(dirname "$0")/../../.." || exit 1
eval "$(/home/anshu/vision-picar/.venv/bin/python tools/plan_section.py env 2>/dev/null)"
export ROS_IMAGE=vision-picar-ros:plan-3.156 PICAR_ROS_NETWORK=host
out="$1"
while true; do
  /home/anshu/vision-picar/.venv/bin/python evaluations/rehearsal-fixes-3172/E/run.py "$out" --batch-s 4500
  rc=$?
  [ "$rc" = 3 ] || { echo "rehearsal exit=$rc $(date)"; break; }
  sleep 90    # a waiter (another session's live run) takes the lock here
done
[ "$rc" = 0 ] || exit "$rc"
extra="$(dirname "$out")/rerun9.jsonl"
[ -s "$extra" ] && exit 0
sleep 90
for _ in $(seq 60); do        # up to an hour for the load to fall under 4
  awk '{exit !($1 < 4)}' /proc/loadavg && break
  sleep 60
done
/home/anshu/vision-picar/.venv/bin/python evaluations/rehearsal-fixes-3172/E/run.py "$out" --extra 9 "$extra" --batch-s 4500
echo "rerun9 exit=$? $(date)"
