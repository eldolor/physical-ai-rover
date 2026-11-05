# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_sim_local_no_real.py

3.143 criterion 3, bar V2: real frames never take fix V's geometry. Whole
tiered missions in process (A2's tiers through the brain server's own
`_tiered_vision_fn`) on frames stamped as a real camera's are (`captured_at`,
`seq`, no `source`), with a local tier that answers yes, no and unable from a
hash of the frame. The tier's one worker thread is replaced by
`InPlacePool` (every dispatched job, local or cloud, runs at submit). Each
mission runs twice in this process: with the simulator's judge
(`sim_identity_fn`, 3.98's geometry, through which fix V and 3.91.D act)
and with none. On real frames every sim path must be a no-op,
so the two runs must agree on every arrival decision, every local-tier count
and the mission's outcome. Differential, so no frozen file to go stale
(review); the measurement against `dev` from before fix V is
`evaluations/soak-find-3143/v2_measure.py`'s.
"""

import hashlib
import json
import logging
import os
from concurrent.futures import Future
from unittest import mock

import pytest

from brain import tiered
from control import brain_server
from control.brain_config import load_brain_config
from control.mission_runner import MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav
from tests.test_sim_local_no import NoCloud

# A local no at the arrival (the box), a yes (the basketball), and frame nos
# about a detected candidate (asserted below). Red first: fix V planted onto
# real frames (its arrival overrule, or its frame-question flag) fails both.
PAIRS = (("procthor:val:0", "box"), ("procthor:val:2", "basketball"))
MAX_STEPS = 60                     # an arrival needs ~25-50 steps here; 30 reaches none
TICK_CAP = 1500
# What the brain's config and tier read from the environment: none of it may
# leak in from the shell running the suite (review).
KEEP_ENV = ("PATH", "HOME", "PYTHONPATH", "TMPDIR", "LANG")


class HashLocal:
    """`LocalVLMClient`'s surface. Its answer is a function of the frame
    alone: unable 1 in 8; else, with the target among the frame's
    detections, yes 2 in 8, otherwise yes 1 in 8; no for the rest."""

    def __init__(self, target):
        self.needle = target.lower()

    def ask(self, frame, box=False):
        h = int(hashlib.sha256(json.dumps(frame, sort_keys=True, default=str).encode())
                .hexdigest()[:8], 16) % 8
        if h == 0:
            return {"status": "unable", "score": None, "reason": "stub"}
        seen = any(self.needle in str(d.get("label", "")).lower()
                   for d in frame.get("detections") or [])
        yes = h <= 2 if seen else h == 1
        got = {"status": "answered", "score": 1.0 if yes else 0.0}
        if box and yes:
            got["box_frac"] = [0.45, 0.4, 0.55, 0.6]
        return got

    __call__ = ask

    def direction(self, frame):
        return {"status": "unsupported", "score": None, "reason": "stub"}

    def close(self):
        pass


class InPlacePool:
    """`ThreadPoolExecutor`'s surface for the tier's one worker: the job runs
    at `submit()`, so its answer has always landed when the next frame
    collects it. On a thread, whether it had depended on scheduling: a busy
    desktop collected some answers a frame later in one run than in the
    other, and the two runs differed with no sim path involved (3.143,
    2026-11-04, a `ready` beside a board job; a 0-30 ms jitter in the stub
    reproduces it). The cost: the branches that run while a call is still
    out are not reached here, and the runner's wall-clock guards (20 s, 8 s)
    remain. Red first again with it: both fix V plants fail both cases."""

    def __init__(self, *a, **k):
        pass

    def submit(self, fn, *args, **kwargs):
        fut = Future()
        try:
            fut.set_result(fn(*args, **kwargs))
        except Exception as e:  # noqa: BLE001 -- the future carries it
            fut.set_exception(e)
        return fut

    def shutdown(self, wait=True, cancel_futures=False):
        pass


class CameraRobot(MockRobot):
    """Unrendered frames stamped as a real camera stamps them (3.90 A):
    `captured_at` and `seq`, no `source`."""

    def __init__(self, grid):
        super().__init__(grid, render=False)
        self._seq = 0

    def get_camera_frame(self):
        frame = super().get_camera_frame()
        self._seq += 1
        frame["metadata"] = {"captured_at": self.world.sim_time, "seq": self._seq}
        return frame


def _mission(house, target, judge, monkeypatch):
    grid = build_world(house)
    robot = CameraRobot(grid)
    config = load_brain_config()
    config.update(tier_search="explore", sim_arrival_confirm="geometry",
                  tier_local_vlm_url="stub://hash")
    monkeypatch.setattr(brain_server, "_local_identity_fn", lambda tg, *a, **k: HashLocal(tg))
    monkeypatch.setattr(tiered, "ThreadPoolExecutor", InPlacePool)
    tier = brain_server._tiered_vision_fn(target, NoCloud(), config, simulated=True,
                                          confirm_vision_fn=NoCloud())
    assert tier.sim_identity_fn is not None          # the judge this test takes away
    if not judge:
        tier.sim_identity_fn = None
    runner = MissionRunner(robot, policy="tiered", vision_fn=tier, tier_search="explore",
                           explore_goal_policy="rooms", target_object=target,
                           navigator=FakeNav(robot), world=MockWorld(grid), max_steps=MAX_STEPS,
                           clock=lambda: grid.sim_time, idle=robot.pass_time,
                           vision_timeout_s=config["vision_timeout_s"],
                           arrival_confirm_timeout_s=config["arrival_confirm_timeout_s"],
                           max_vision_failures=config["max_vision_failures"])
    try:
        runner.start()
        ticks = 0
        while runner.tick() and ticks < TICK_CAP:
            ticks += 1
        s = runner.status()
        st = (s.get("tier") or {}).get("stats") or {}
        return {"outcome": s["outcome"], "step": s.get("step"), "ticks": ticks,
                "capped": ticks >= TICK_CAP,
                "arrivals": [[v.get("tier"), v.get("confirmed"), v.get("local_score"),
                              bool(v.get("local_overruled"))]
                             for v in s.get("arrival_verdicts") or []],
                **{k: st.get(k) for k in ("local", "local_frame", "local_triggers", "rejections",
                                          "cannot_tell", "escalations")}}
    finally:
        runner.stop()
        tier.close()


@pytest.fixture
def clean_env():
    env = {k: os.environ[k] for k in KEEP_ENV if k in os.environ}
    logging.disable(logging.WARNING)
    with mock.patch.dict(os.environ, env, clear=True):
        yield
    logging.disable(logging.NOTSET)


@pytest.mark.parametrize("house,target", PAIRS)
def test_real_frames_decide_the_same_without_the_sim_judge(house, target, clean_env, monkeypatch):
    with_judge = _mission(house, target, True, monkeypatch)
    without = _mission(house, target, False, monkeypatch)
    assert with_judge == without
    assert not with_judge["capped"]
    # Meaningful only if a local no was met, and none was overruled.
    assert with_judge["local"].get("no_overruled_sim", 0) == 0
    assert not any(v[3] for v in with_judge["arrivals"])
    assert (with_judge["local"].get("no", 0) + (with_judge["local_frame"] or {}).get("no", 0)) > 0
