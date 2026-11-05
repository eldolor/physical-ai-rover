# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/brain_floor_off.py

3.135.B F24 (amendment 19): the shipped brain's code on a copy of
`config/robot.yaml` whose `brain.battery_floor_v` is 0 (off), as 3.137
criterion 1's arm R ran it. That one setting switches off BOTH of the brain's
own battery exits, its 10.5 V floor and its end at the ladder's Reserve or
Power-off (`MissionRunner._battery_spent()` is reached only with a floor). So
the mission drives on into Reserve, and F24 measures the robot server's Reserve
alone, the backstop behind the brain. No code is patched; every F24 run
(shipped and planted) uses it, and the run checks the brain reports the floor
off (`status.battery_floor_v` null).

    FMEA_DIR=<run dir> python evaluations/jetson-fmea-3135/brain_floor_off.py <port>
"""

import os
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(os.environ.get("FMEA_DIR", ".")).resolve()     # before the chdir below
sys.path.insert(0, str(ROOT))
os.chdir(ROOT)


def floor_off_config(out: Path) -> Path:
    """The repo's config with the brain's battery floor off, written in `out`,
    one file per brain process (a restart keeps the earlier copy)."""
    cfg = yaml.safe_load((ROOT / "config/robot.yaml").read_text())
    cfg["brain"]["battery_floor_v"] = 0
    path = Path(out) / f"robot-floor-off-{os.getpid()}.yaml"
    path.write_text(yaml.safe_dump(cfg, sort_keys=False))
    return path


if __name__ == "__main__":
    import uvicorn

    import control.brain_config as bc

    # Before control.brain_server is imported: its module-level `app` is then
    # the only app built, on this config (the run checks the floor it reports).
    _path = str(floor_off_config(OUT))
    _load = bc.load_brain_config
    bc.load_brain_config = lambda config_path=None: _load(config_path or _path)
    import control.brain_server as server  # noqa: E402

    app = server.app
    uvicorn.run(app, host="127.0.0.1", port=int(sys.argv[1]), log_level="warning")
