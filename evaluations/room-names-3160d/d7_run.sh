#!/usr/bin/env bash
# 3.160.D D7: one live find on the named saved map (d7_live.py), on this section's ports.
cd "$(dirname "$0")/../.." || exit 1
eval "$(/home/anshu/vision-picar/.venv/bin/python tools/plan_section.py env 2>/dev/null)"
export ROS_IMAGE=vision-picar-ros:plan-3.156 PICAR_ROS_NETWORK=host
/home/anshu/vision-picar/.venv/bin/python evaluations/room-names-3160d/d7_live.py "$1"
echo "d7 exit=$? $(date)"
