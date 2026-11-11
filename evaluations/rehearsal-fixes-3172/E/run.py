#!/usr/bin/env python3
"""3.172.E: 3.158.B's rehearsal again, unchanged but for a fresh map volume.

    eval "$(python tools/plan_section.py env)"; PICAR_ROS_NETWORK=host \\
    ROS_IMAGE=vision-picar-ros:plan-3.156 python evaluations/rehearsal-fixes-3172/E/run.py OUT.jsonl

Exit 0: the schedule is finished; 3: run another batch (`run.sh` loops).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "autonomy-3158" / "B"))

import rehearsal as R          # noqa: E402

R.VOLUME = "picar-maps-3172e"
sys.exit(R.main())
