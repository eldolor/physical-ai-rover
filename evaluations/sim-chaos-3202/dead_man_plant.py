# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/sim-chaos-3202/dead_man_plant.py

3.202 criterion 7's plant for K11 (amendment 2): the shipped brain with B3.3's
dead-man pushed out past K11's bar -- `drill_tick_timeout_s` (the deadline a
`tick_hang` drill runs under; `drills.apply` uses it in place of
`tick_timeout_s`) raised to 600 s -- so K11's "the mission ends within
drill_tick_timeout_s + 30 s" must FAIL. The guard is still there, 600 s out:
the planted run ends "brain loop hung: tick exceeded 600.0s". The drill's
own hang is drill_tick_timeout_s x OVERSHOOT, so it grows with it (1,800 s);
the brain's abandoned tick thread outlives the mission. Started by
`chaos.py --plant dead_man` in place of the brain.

    python evaluations/sim-chaos-3202/dead_man_plant.py <port>
"""

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
os.chdir(ROOT)

from control import brain_config  # noqa: E402

_load = brain_config.load_brain_config


def _load_unguarded(*a, **k):
    cfg = _load(*a, **k)
    cfg["drill_tick_timeout_s"] = 600.0      # planted: the drill's dead-man past K11's 33 s bar
    return cfg


brain_config.load_brain_config = _load_unguarded     # brain_server takes it by name at import

import control.brain_server as server  # noqa: E402
import uvicorn  # noqa: E402

uvicorn.run(server.app, host="127.0.0.1", port=int(sys.argv[1]), log_level="warning")
