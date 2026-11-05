# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/brain_floor_off.py

3.135.B F24 (amendment 19): the SHIPPED brain (`control.brain_server.create_app`)
on a copy of `config/robot.yaml` whose `brain.battery_floor_v` is 0 (off), as
3.137 criterion 1's arm R ran it. The brain's own floor (10.5 V, 3.70 E4)
ends a mission at Low, ~30 s before Reserve, so with it on no mission is left
running for the robot server's Reserve to stop. A setting, not a plant: no
code is patched, and every F24 run (shipped and planted) uses it.

    FMEA_DIR=<run dir> python evaluations/jetson-fmea-3135/brain_floor_off.py <port>
"""

import os
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
os.chdir(ROOT)


def floor_off_config(out: Path) -> Path:
    """The repo's config with the brain's battery floor off, written in `out`."""
    cfg = yaml.safe_load((ROOT / "config/robot.yaml").read_text())
    cfg["brain"]["battery_floor_v"] = 0
    path = Path(out) / "robot-floor-off.yaml"
    path.write_text(yaml.safe_dump(cfg, sort_keys=False))
    return path


if __name__ == "__main__":
    import uvicorn

    from control.brain_server import create_app

    app = create_app(config_path=str(floor_off_config(Path(os.environ["FMEA_DIR"]))))
    uvicorn.run(app, host="127.0.0.1", port=int(sys.argv[1]), log_level="warning")
