# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_local_vlm_down.py

3.152 (`docs/plans/ros-alignment/3.152-local-vlm-down.md`), step A: a local
VLM found absent (a refused or dropped connection, or a 503) is held down --
the policy is then exactly the one with no local tier configured -- and its
free /health is probed until it says ready. A hang stays the unable rule's.

Whole tiered missions go through the brain server's own `_tiered_vision_fn`
(A2's tiers, explore, `rooms`), on unrendered frames, with the tier's worker
run in place (`InPlacePool`, 3.143's: the decisions must not depend on
thread scheduling). The policy's own unit tests use a fake pipeline.
"""

import hashlib
import json
import logging
import os
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from unittest import mock

import pytest

from brain import tiered
from brain.local_vlm import LocalVLMAbsent, LocalVLMClient
from brain.perceive import FrameReportedPipeline
from brain.tiered import HARD_RULES, TieredVision
from control import brain_server
from control.brain_config import load_brain_config
from control.mission_runner import MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav
from tests.test_sim_local_no import NoCloud
from tests.test_sim_local_no_real import InPlacePool

HOUSE, TARGET = "procthor:val:2", "basketball"
MAX_STEPS = 60
TICK_CAP = 1500
KEEP_ENV = ("PATH", "HOME", "PYTHONPATH", "TMPDIR", "LANG")

# dev's rules at 3.152's start (criterion 6): a held-down VLM changes none.
DEV_HARD_RULES = (
    "safety: robot/safety.py vetoes every move; only it overrides the tiers",
    "a person's Stop is the last word over autonomy (3.84)",
    "the tiers' order: perception < local VLM < cloud -- a lower tier never "
    "undoes a higher tier's answer about a candidate (3.91.C)",
    "an arrival the local VLM's box led to is confirmed only by the cloud (3.80.O)",
    "arrival is the lidar's rule, then the identity check",
    "B3: the watchdog, the runner's vision timeout and failure budget, the tick guard",
    "who to ask: the guards' order (call cap, one call at a time)",
)


class StubVE:
    """`LocalVLMClient`'s surface: `mode` is `down` (questions raise
    LocalVLMAbsent, /health not ready), `up` (answers from a hash of the
    frame, as V2's stub) or `slow` (every question times out, /health
    ready). Counts every question and every probe."""

    def __init__(self, target, mode="up"):
        self.needle = target.lower()
        self.mode = mode
        self.asked = 0
        self.probes = 0

    def ask(self, frame, box=False):
        self.asked += 1
        if self.mode == "down":
            raise LocalVLMAbsent("local VLM unreachable: stub refused")
        if self.mode == "slow":
            return {"status": "timeout", "score": None, "reason": "stub"}
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

    def healthy(self, timeout_s=2.0):
        self.probes += 1
        return self.mode != "down"

    def close(self):
        pass


class NoHealthVE(StubVE):
    """A client with no /health probe: nothing is held (dev's behaviour)."""
    healthy = None

    def __getattribute__(self, name):
        if name == "healthy":
            raise AttributeError(name)
        return super().__getattribute__(name)


class Camera(MockRobot):
    """Unrendered frames with no `source`, as V2's camera."""

    def __init__(self, grid):
        super().__init__(grid, render=False)


@pytest.fixture
def clean_env():
    env = {k: os.environ[k] for k in KEEP_ENV if k in os.environ}
    logging.disable(logging.WARNING)
    with mock.patch.dict(os.environ, env, clear=True):
        yield
    logging.disable(logging.NOTSET)


def _mission(monkeypatch, ve=None, pool=InPlacePool, cloud=None, **cfg):
    """One tiered mission; `ve` None is no local tier configured."""
    grid = build_world(HOUSE)
    robot = Camera(grid)
    config = load_brain_config()
    config.update(tier_search="explore", sim_arrival_confirm="geometry",
                  tier_local_vlm_url="stub://ve" if ve is not None else "", **cfg)
    monkeypatch.setattr(brain_server, "_local_identity_fn",
                        lambda tg, *a, **k: ve)
    monkeypatch.setattr(tiered, "ThreadPoolExecutor", pool)
    cloud = cloud or NoCloud()
    tier = brain_server._tiered_vision_fn(TARGET, cloud, config, simulated=True,
                                          confirm_vision_fn=cloud)
    runner = MissionRunner(robot, policy="tiered", vision_fn=tier, tier_search="explore",
                           explore_goal_policy="rooms", target_object=TARGET,
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
                "arrivals": [[v.get("tier"), v.get("confirmed")]
                             for v in s.get("arrival_verdicts") or []],
                "cloud_calls": st.get("cloud_calls"), "triggers": st.get("triggers"),
                "local_vlm": st.get("local_vlm"), "local_frame": st.get("local_frame"),
                "frame_escalations": st.get("frame_escalations"),
                "local_frame_ms": st.get("local_frame_ms"),
                "cloud": cloud.calls}, tier
    finally:
        runner.stop()
        tier.close()


# -- criterion 1: down is remembered, and down is "no local tier" ----------

def test_c1_a_down_ve_is_asked_once_then_never_again(clean_env, monkeypatch):
    ve = StubVE(TARGET, "down")
    got, _ = _mission(monkeypatch, ve)
    assert got["step"] >= 30
    assert ve.asked == 1
    assert got["local_vlm"]["state"] == "down"
    assert got["local_frame"].get("absent") == 1


def test_c1_down_decides_as_no_local_tier(clean_env, monkeypatch):
    down, _ = _mission(monkeypatch, StubVE(TARGET, "down"))
    none, _ = _mission(monkeypatch, None)
    keep = ("outcome", "step", "ticks", "arrivals", "cloud_calls", "triggers")
    assert {k: down[k] for k in keep} == {k: none[k] for k in keep}
    assert none["local_vlm"] == {"state": "off"}


def test_c1_without_a_probe_nothing_is_held(clean_env, monkeypatch):
    """A client with no /health: dev's behaviour, every question tried."""
    ve = NoHealthVE(TARGET, "down")
    got, _ = _mission(monkeypatch, ve)
    assert ve.asked > 1
    assert got["local_vlm"] == {"state": "unwatched"}


# -- criterion 2: no wait ---------------------------------------------------

def test_c2_the_down_window_has_no_local_wait(clean_env, monkeypatch):
    got, _ = _mission(monkeypatch, StubVE(TARGET, "down"))
    assert (got["local_frame_ms"] or {}).get("p90", 0) <= 200


def test_c2_a_refused_connection_is_quick_with_the_real_client():
    client = LocalVLMClient("http://127.0.0.1:9", TARGET, deadline_s=3.0)
    frame = {"image_base64": _tiny_jpeg_b64()}
    t0 = time.perf_counter()
    with pytest.raises(LocalVLMAbsent):
        client.ask(frame, box=True)
    assert time.perf_counter() - t0 <= 0.2
    assert client.healthy() is False
    client.close()


def _tiny_jpeg_b64():
    import base64
    import io
    from PIL import Image
    buf = io.BytesIO()
    Image.new("RGB", (8, 8)).save(buf, format="JPEG")
    return base64.b64encode(buf.getvalue()).decode()


# -- criterion 3: back on the first good probe ------------------------------

def _tier(ve, probe_s=0.2, **kw):
    return TieredVision(FrameReportedPipeline(TARGET), NoCloud(),
                        local_identity_fn=ve, local_sighting_fn=ve.ask,
                        local_health_fn=ve.healthy, local_probe_s=probe_s, **kw)


def _frame(n):
    return {"detections": [], "objects_visible": [], "pan_deg": 0.0,
            "metadata": {"captured_at": float(n), "seq": n}}


def _wait_for(pred, timeout_s):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.01)
    return pred()


def test_c3_up_within_one_probe_and_asked_first_again():
    ve = StubVE(TARGET, "down")
    tier = _tier(ve, probe_s=0.2)
    try:
        tier._note_local_error(LocalVLMAbsent("stub"), tier._local_gen)
        tier(_frame(1))
        assert tier.stats.local_vlm["state"] == "down"
        assert tier.local_sighting_fn is None and tier.local_identity_fn is None
        time.sleep(0.5)
        assert tier.stats.local_vlm["state"] == "down"      # not ready: held
        assert tier.stats.local_vlm["probes"] >= 1
        ve.mode = "up"
        turned = time.monotonic()
        assert _wait_for(lambda: tier._local_back_seen, 0.2 + 1.0)
        assert time.monotonic() - turned <= 0.2 + 1.0
        tier(_frame(2))
        assert tier.stats.local_vlm["state"] == "up"
        assert tier.local_sighting_fn is not None
        assert tier._local_probe is None or not tier._local_probe.is_alive() or \
            _wait_for(lambda: not tier._local_probe.is_alive(), 1.0)
    finally:
        tier.close()


@pytest.mark.parametrize("probe", [lambda: False, lambda: 1 / 0])
def test_c3_not_ready_or_a_raising_probe_keeps_it_down(probe):
    ve = StubVE(TARGET, "down")
    tier = TieredVision(FrameReportedPipeline(TARGET), NoCloud(), local_identity_fn=ve,
                        local_sighting_fn=ve.ask, local_health_fn=probe, local_probe_s=0.05)
    try:
        tier._note_local_error(LocalVLMAbsent("stub"), tier._local_gen)
        tier(_frame(1))
        time.sleep(0.3)
        tier(_frame(2))
        assert tier.stats.local_vlm["state"] == "down"
        assert tier.stats.local_vlm["probes"] >= 2
    finally:
        tier.close()


def test_c3_the_mission_comes_back_when_ve_does(clean_env, monkeypatch):
    """Down from the first question, ready a moment later: the mission's
    later questions go to VE again."""
    ve = StubVE(TARGET, "down")
    original = ve.ask

    def ask(frame, box=False):
        out = original(frame, box)       # raises while down
        return out

    def flip_after_first(frame, box=False):
        try:
            return ask(frame, box)
        finally:
            ve.mode = "up"               # ready by the first probe
    ve.ask = flip_after_first
    got, _ = _mission(monkeypatch, ve, tier_local_probe_s=0.05)
    assert got["local_vlm"]["state"] == "up"
    assert ve.asked > 1


def test_c3_the_probe_interval_is_validated(tmp_path):
    for bad in (0, -1, float("nan"), True, "5"):
        p = tmp_path / "robot.yaml"
        p.write_text(json.dumps({"brain": {"tier_local_probe_s": bad}})
                     if not isinstance(bad, float) else "brain:\n  tier_local_probe_s: .nan\n")
        with pytest.raises(ValueError, match="tier_local_probe_s"):
            load_brain_config(str(p))


# -- criterion 4: slow but alive is the unable rule -------------------------

def test_c4_a_slow_ve_stays_up_and_counts_as_dev_does(clean_env, monkeypatch):
    with_probe, _ = _mission(monkeypatch, StubVE(TARGET, "slow"))
    without, _ = _mission(monkeypatch, NoHealthVE(TARGET, "slow"))
    assert with_probe["local_vlm"]["state"] == "up"
    keep = ("outcome", "step", "ticks", "arrivals", "cloud_calls", "triggers",
            "local_frame", "frame_escalations")
    assert {k: with_probe[k] for k in keep} == {k: without[k] for k in keep}
    assert with_probe["local_frame"].get("unable_timeout", 0) > 0


# -- criterion 5: the cloud's guards hold -----------------------------------

class CountingCloud(NoCloud):
    def __init__(self):
        super().__init__()
        self._lock = threading.Lock()
        self.open = 0
        self.most_open = 0

    def __call__(self, frame):
        with self._lock:
            self.open += 1
            self.most_open = max(self.most_open, self.open)
        try:
            time.sleep(0.002)
            return super().__call__(frame)
        finally:
            with self._lock:
                self.open -= 1


def test_c5_capped_and_one_at_a_time_on_a_real_worker(clean_env, monkeypatch):
    cloud = CountingCloud()
    got, _ = _mission(monkeypatch, StubVE(TARGET, "down"), pool=ThreadPoolExecutor,
                      cloud=cloud, tier_max_calls=4)
    assert cloud.calls <= 4
    assert cloud.most_open == 1


# -- criterion 6: the rules are unchanged -----------------------------------

def test_c6_hard_rules_and_trees_unchanged_by_down():
    assert HARD_RULES == DEV_HARD_RULES
    ve = StubVE(TARGET, "down")
    tier = _tier(ve, probe_s=60)
    try:
        before = (tier.describe_who_tree(), tier.describe_when_tree(), tier.describe_tree())
        tier._note_local_error(LocalVLMAbsent("stub"), tier._local_gen)
        tier(_frame(1))
        assert tier.stats.local_vlm["state"] == "down"
        assert (tier.describe_who_tree(), tier.describe_when_tree(),
                tier.describe_tree()) == before
    finally:
        tier.close()


# -- criterion 7: probe and thread hygiene ----------------------------------

def test_c7_no_probes_while_up(clean_env, monkeypatch):
    ve = StubVE(TARGET, "up")
    got, _ = _mission(monkeypatch, ve)
    assert got["ticks"] >= 30
    assert ve.probes == 0
    assert got["local_vlm"]["state"] == "up"


def test_c7_no_probe_thread_after_close():
    ve = StubVE(TARGET, "down")
    tier = _tier(ve, probe_s=0.05)
    tier._note_local_error(LocalVLMAbsent("stub"), tier._local_gen)
    tier(_frame(1))
    probe = tier._local_probe
    assert probe is not None and probe.is_alive()
    tier.close()
    assert _wait_for(lambda: not probe.is_alive(), 1.0)


def test_c7_a_close_during_a_probe_pays_nothing_and_flips_nothing():
    cloud = NoCloud()
    started, release = threading.Event(), threading.Event()

    def probe():
        started.set()
        release.wait(2)
        return True
    ve = StubVE(TARGET, "down")
    tier = TieredVision(FrameReportedPipeline(TARGET), cloud, local_identity_fn=ve,
                        local_sighting_fn=ve.ask, local_health_fn=probe, local_probe_s=0.01)
    tier._note_local_error(LocalVLMAbsent("stub"), tier._local_gen)
    tier(_frame(1))
    assert started.wait(1)
    calls = cloud.calls
    tier.close()
    release.set()
    assert _wait_for(lambda: not tier._local_probe.is_alive(), 1.0)
    assert tier._local_back_seen is False
    assert cloud.calls == calls


def test_c7_other_client_faults_hold_nothing():
    ve = StubVE(TARGET, "up")
    tier = _tier(ve)
    try:
        tier._note_local_error(RuntimeError("a fault in the client"), tier._local_gen)
        tier(_frame(1))
        assert tier.stats.local_vlm["state"] == "up"
    finally:
        tier.close()


# -- criterion 8: the state is in the mission's status ----------------------

def test_c8_status_carries_the_state():
    ve = StubVE(TARGET, "down")
    tier = _tier(ve, probe_s=60)
    try:
        assert tier.stats.as_dict()["local_vlm"]["state"] == "up"
        tier._note_local_error(LocalVLMAbsent("stub"), tier._local_gen)
        tier(_frame(1))
        lv = tier.stats.as_dict()["local_vlm"]
        assert lv["state"] == "down" and isinstance(lv["since"], float)
    finally:
        tier.close()
    off = TieredVision(FrameReportedPipeline(TARGET), NoCloud())
    assert off.stats.as_dict()["local_vlm"] == {"state": "off"}
    off.close()


# -- the reviews' findings (2026-11-04), red first ---------------------------

def test_review_a_flip_after_close_starts_no_probe():
    """A frame that noted an absence races a Stop: close() first, then the
    frame's flip. No probe may start (it would poll a closed client for the
    process's life)."""
    ve = StubVE(TARGET, "down")
    tier = _tier(ve, probe_s=0.05)
    tier._note_local_error(LocalVLMAbsent("stub"), tier._local_gen)
    tier.close()
    tier._local_absent_seen = True          # as if noted in the race's window
    tier._apply_local_flip()
    assert tier._local_probe is None
    assert tier.stats.local_vlm["state"] == "up"


def test_review_a_stale_absence_after_the_return_holds_nothing():
    """A call taken before VE came back fails afterwards: it is about the
    old VE, so the tier stays up."""
    ve = StubVE(TARGET, "down")
    tier = _tier(ve, probe_s=0.05)
    try:
        before = tier._local_gen
        tier._note_local_error(LocalVLMAbsent("stub"), before)
        tier(_frame(1))
        assert tier.stats.local_vlm["state"] == "down"
        ve.mode = "up"
        assert _wait_for(lambda: tier._local_back_seen, 1.0)
        tier(_frame(2))
        assert tier.stats.local_vlm["state"] == "up"
        tier._note_local_error(LocalVLMAbsent("stale"), before)
        tier(_frame(3))
        assert tier.stats.local_vlm["state"] == "up"
        tier._note_local_error(LocalVLMAbsent("fresh"), tier._local_gen)
        tier(_frame(4))
        assert tier.stats.local_vlm["state"] == "down"
        assert tier.stats.local_vlm["probes"] == 0       # a new outage's own count
    finally:
        tier.close()


def test_review_a_ve_that_is_back_starts_with_no_unable_answers():
    ve = StubVE(TARGET, "down")
    tier = _tier(ve, probe_s=0.05)
    try:
        tier._note_local_error(LocalVLMAbsent("stub"), tier._local_gen)
        tier(_frame(1))
        tier._frame_unable = tier._local_unable = 2      # spent when it went
        ve.mode = "up"
        assert _wait_for(lambda: tier._local_back_seen, 1.0)
        tier._apply_local_flip()                          # the next frame's first step
        assert tier.stats.local_vlm["state"] == "up"
        assert tier._frame_unable == 0 and tier._local_unable == 0
        assert tier._local_first() is not None
        assert tier.next_confirm_is_local()
    finally:
        tier.close()


# -- the second review's findings ---------------------------------------------

def test_review2_the_probe_timeout_phases_sum_under_a_second():
    from brain.local_vlm import PROBE_TIMEOUT, PROBE_TIMEOUT_S
    phases = (PROBE_TIMEOUT.connect, PROBE_TIMEOUT.read, PROBE_TIMEOUT.write,
              PROBE_TIMEOUT.pool)
    assert sum(phases) == pytest.approx(PROBE_TIMEOUT_S) and PROBE_TIMEOUT_S < 1.0


def test_review2_a_probe_returning_after_close_records_nothing():
    started, release = threading.Event(), threading.Event()

    def probe():
        started.set()
        release.wait(2)
        return False
    ve = StubVE(TARGET, "down")
    tier = TieredVision(FrameReportedPipeline(TARGET), NoCloud(), local_identity_fn=ve,
                        local_sighting_fn=ve.ask, local_health_fn=probe, local_probe_s=0.01)
    tier._note_local_error(LocalVLMAbsent("stub"), tier._local_gen)
    tier(_frame(1))
    assert started.wait(1)
    tier.close()
    before = dict(tier.stats.local_vlm)
    release.set()
    assert _wait_for(lambda: not tier._local_probe.is_alive(), 1.0)
    assert tier.stats.local_vlm == before


def test_review2_a_read_across_a_flip_reads_nothing():
    ve = StubVE(TARGET, "up")
    tier = _tier(ve, probe_s=60)
    try:
        fn, gen = tier._local_read("local_identity_fn")
        assert fn is ve and gen == tier._local_gen

        # A read that lands inside a flip, as the writer orders it: the
        # generation odd, the state not yet changed (review 3).
        tier._local_gen += 1
        fn, _ = tier._local_read("local_identity_fn")
        assert fn is None
        tier._local_gen += 1

        class Flipping:                       # a whole flip lands mid-read
            def __get__(self, obj, owner):
                obj._local_gen += 2
                return ve
        with mock.patch.object(TieredVision, "local_identity_fn", Flipping()):
            fn, _ = tier._local_read("local_identity_fn")
        assert fn is None
    finally:
        tier.close()


def test_review3_a_down_tier_with_no_probe_alive_probes_again():
    ve = StubVE(TARGET, "down")
    tier = _tier(ve, probe_s=0.05)
    try:
        tier._note_local_error(LocalVLMAbsent("stub"), tier._local_gen)
        tier(_frame(1))
        tier._local_probe_stop.set()                 # the probe dies
        assert _wait_for(lambda: not tier._local_probe.is_alive(), 1.0)
        tier(_frame(2))
        assert tier._local_probe.is_alive()
        ve.mode = "up"
        assert _wait_for(lambda: tier._local_back_seen, 1.0)
        tier(_frame(3))
        assert tier.stats.local_vlm["state"] == "up"
    finally:
        tier.close()


def test_review3_generations_are_even_at_rest():
    ve = StubVE(TARGET, "down")
    tier = _tier(ve, probe_s=0.05)
    try:
        tier._note_local_error(LocalVLMAbsent("stub"), tier._local_gen)
        tier(_frame(1))
        assert tier._local_gen % 2 == 0 and tier._local_gen > 0
        ve.mode = "up"
        assert _wait_for(lambda: tier._local_back_seen, 1.0)
        tier(_frame(2))
        assert tier._local_gen % 2 == 0
    finally:
        tier.close()


def test_review4_the_flip_writes_odd_while_the_state_changes():
    """A seqlock's write order: the generation odd whenever `_local_down`
    is assigned (review 3)."""
    seen = []

    class Watched(TieredVision):
        def __setattr__(self, name, value):
            if name == "_local_down" and "_local_gen" in self.__dict__:
                seen.append(self._local_gen % 2)
            super().__setattr__(name, value)
    ve = StubVE(TARGET, "down")
    tier = Watched(FrameReportedPipeline(TARGET), NoCloud(), local_identity_fn=ve,
                   local_sighting_fn=ve.ask, local_health_fn=ve.healthy, local_probe_s=0.05)
    try:
        tier._note_local_error(LocalVLMAbsent("stub"), tier._local_gen)
        tier(_frame(1))
        ve.mode = "up"
        assert _wait_for(lambda: tier._local_back_seen, 1.0)
        tier(_frame(2))
        assert seen == [1, 1]
    finally:
        tier.close()


def test_review4_a_probe_that_cannot_start_is_retried_not_raised():
    ve = StubVE(TARGET, "down")
    tier = _tier(ve, probe_s=0.05)
    try:
        tier._note_local_error(LocalVLMAbsent("stub"), tier._local_gen)
        class NoStart(threading.Thread):
            def start(self):
                raise RuntimeError("can't start new thread")
        with mock.patch.object(TieredVision, "_probe_thread", NoStart):
            tier(_frame(1))                      # must not raise
        assert tier.stats.local_vlm["state"] == "down" and tier._local_probe is None
        tier(_frame(2))                          # retried
        assert tier._local_probe is not None and tier._local_probe.is_alive()
    finally:
        tier.close()


def test_review4_a_frame_overtaken_by_a_flip_asks_nothing():
    ve = StubVE(TARGET, "up")
    tier = _tier(ve, probe_s=60)
    try:
        fn = tier._local_first()
        assert fn is not None
        tier._note_local_error(LocalVLMAbsent("stub"), tier._local_gen)
        tier._apply_local_flip()                 # another frame flips it down
        asked = ve.asked
        from brain.perceive import Perception
        tier._ask_local_first(_frame(9), Perception(status="absent"), "cold_search",
                              tier._epoch, fn)
        assert ve.asked == asked
    finally:
        tier.close()
