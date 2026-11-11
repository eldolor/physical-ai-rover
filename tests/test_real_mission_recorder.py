# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.174.B: the real-mission recorder (evaluations/real-missions-3174/recorder.py),
in process against a scripted robot server and brain: its row, track and frames,
a link that drops (B8) and what a row may carry (B9)."""

import base64
import importlib.util
import io
import json
import subprocess
import sys
from pathlib import Path

import pytest
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location(
    "recorder_3174", ROOT / "evaluations" / "real-missions-3174" / "recorder.py")
rec = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rec)

MID = "20261111T200000.000000Z-red_backpack"


def _jpeg():
    buf = io.BytesIO()
    Image.new("RGB", (32, 24), (200, 30, 30)).save(buf, "JPEG")
    return base64.b64encode(buf.getvalue()).decode()


class _R:
    def __init__(self, code, body):
        self.status_code, self._body = code, body

    def json(self):
        return self._body


class _World:
    """A mission of `run_s` seconds driving +x at 0.1 m/s, then the report; the
    link down between `drop` seconds (both clients raise)."""

    def __init__(self, run_s=60.0, drop=None):
        self.t, self.run_s, self.drop = 0.0, run_s, drop
        self.img = _jpeg()

    def clock(self):
        return 1000.0 + self.t

    def sleep(self, s):
        self.t += max(s, 0.05)

    def down(self):
        return self.drop is not None and self.drop[0] <= self.t < self.drop[1]

    def running(self):
        return self.t < self.run_s


class _Robot:
    def __init__(self, w):
        self.w = w

    def get(self, path):
        w = self.w
        if w.down():
            raise ConnectionError("link down")
        if path == "/world/pose":
            return _R(200, {"usable": True, "map_id": "m1", "x_m": 0.1 * min(w.t, w.run_s),
                            "y_m": 1.0, "heading_deg": 90.0})
        if path == "/frame":
            return _R(200, {"image_base64": w.img, "media_type": "image/jpeg", "room": "kitchen"})
        if path == "/battery":
            return _R(200, {"v": 12.0 - w.t / 1000, "level": "ok"})
        if path == "/health":
            return _R(200, {"refusal_counts": {"too_close": int(w.t // 30)},
                            "wheel_loop": {"late_ticks": 1, "moving_ticks": int(w.t * 20)}})
        return _R(404, {})


class _Brain:
    def __init__(self, w):
        self.w = w

    def get(self, path):
        w = self.w
        if w.down():
            raise ConnectionError("link down")
        if path == "/mission/status":
            on = w.running()
            return _R(200, {"running": on, "mission_id": MID, "job": "find", "policy": "tiered",
                            "target_object": "red backpack", "outcome": None if on else "found",
                            "end_state": None if on else "found", "end_reason": "" if on else "seen",
                            "report": None if on else {"rooms": 3}, "budget": {"elapsed_s": w.t},
                            "return": {"state": "docked"}, "camera_failures": 0,
                            "last_action": "FORWARD"})
        if path == f"/mission/reports/{MID}":
            return _R(200, {"landmarks": [{"name": "a"}, {"name": "b"}]})
        return _R(404, {})


def _record(tmp_path, **kw):
    w = _World(**kw)
    r = rec.Recorder(_Robot(w), _Brain(w), tmp_path / "batch", interventions=None,
                     room_at=lambda x, y: "hall" if x < 3.0 else "kitchen",
                     clock=w.clock, sleep=w.sleep)
    first = r.wait_for_start(give_up_s=5)
    assert first is not None
    return r, r.record(first)


def test_a_mission_is_one_row_with_its_track_and_frames(tmp_path):
    r, row = _record(tmp_path)
    rows = [json.loads(x) for x in (r.dir / "rows.jsonl").read_text().splitlines()]
    assert rows == [row]
    assert row["outcome"] == "found" and row["end_state"] == "found" and row["job"] == "find"
    assert row["pose_coverage"] >= 0.95
    assert abs(row["distance_m"] - 6.0) < 0.1                  # 60 s at 0.1 m/s
    assert row["rooms_entered"] == ["hall", "kitchen"] and row["rooms_entered_n"] == 2
    assert row["frames_kept"] / row["seconds"] >= 0.45
    assert row["refusals"] == {"too_close": 2} and row["late_ticks"] == 0
    assert row["report"] == "kept" and row["report_landmarks"] == 2
    assert row["health"] == "kept" and row["battery"] == "kept" and row["gap_s"] == 0
    assert row["intervention_lines"] == "missing"
    for k in ("contacts", "min_gap_cm", "found_on_target", "wh"):
        assert k in row["unmeasurable"]


def test_b5_the_frames_are_a_walk_walk_eval_and_make_sets_read(tmp_path):
    r, row = _record(tmp_path)
    from control.walk_eval import compute_metrics
    from control.walk_store import LocalWalkStore
    store = LocalWalkStore(r.dir / "walks")
    assert store.list_walks() == row["walks"] == [f"{MID}-p1"]
    walk = r.dir / "walks" / row["walks"][0]
    entries = [json.loads(x) for x in (walk / "walk.jsonl").read_text().splitlines()]
    assert len(entries) == row["frames_kept"]
    assert compute_metrics(entries)["frames"] == row["frames_kept"]
    for e in entries:
        Image.open(walk / e["file"]).verify()                   # a decodable JPEG
    assert json.loads((walk / "meta.json").read_text())["frames"] == row["frames_kept"]
    # 3.80.U's set drawer, given the labels a person writes:
    labels = {"description": "red backpack",
              "target_visible_labels": {e["file"]: i % 2 == 0 for i, e in enumerate(entries)}}
    (walk / "labels.json").write_text(json.dumps(labels))
    out = tmp_path / "sets"
    p = subprocess.run([sys.executable, str(ROOT / "evaluations" / "tiered-380" / "U" / "make_sets.py"),
                        "--out", str(out), "--recordings", str(r.dir / "walks"),
                        "--walk", row["walks"][0]], capture_output=True, text=True, cwd=ROOT)
    assert p.returncode == 0, p.stderr
    drawn = json.loads(p.stdout)
    assert drawn["S1"] >= 1 and drawn["S2"] >= 1


def test_b5_a_long_mission_splits_its_frames_at_the_walk_cap(tmp_path, monkeypatch):
    monkeypatch.setattr(rec, "MAX_FRAMES_PER_WALK", 10)
    r, row = _record(tmp_path)
    assert len(row["walks"]) == -(-row["frames_kept"] // 10) >= 3
    from control.walk_store import LocalWalkStore
    assert LocalWalkStore(r.dir / "walks").list_walks() == sorted(row["walks"])


def test_b8_a_link_that_drops_for_10_s_is_a_counted_gap_not_a_lost_row(tmp_path):
    r, row = _record(tmp_path, drop=(20.0, 30.0))
    assert row["outcome"] == "found"
    assert 9.0 <= row["gap_s"] <= 11.0
    assert 0.80 <= row["pose_coverage"] < 0.95                   # the gap shows, it is not hidden
    assert (r.dir / "rows.jsonl").read_text().count("\n") == 1


def test_b8_a_brain_that_never_ends_still_gets_a_row_after_its_grace(tmp_path):
    w = _World()
    brain = _Brain(w)
    real = brain.get

    def no_report(path):
        out = real(path)
        if path == "/mission/status":
            out._body["report"] = None
        return out
    brain.get = no_report
    r = rec.Recorder(_Robot(w), brain, tmp_path / "b", clock=w.clock, sleep=w.sleep)
    row = r.record(r.wait_for_start(give_up_s=5))
    assert row["outcome"] == "found" and w.t <= 60 + rec.END_GRACE_S + 2


def test_b9_a_batch_inside_a_git_work_tree_is_refused(tmp_path):
    (tmp_path / "repo" / ".git").mkdir(parents=True)
    with pytest.raises(SystemExit, match="git work tree"):
        rec.refuse_in_repo(tmp_path / "repo" / "real")
    with pytest.raises(SystemExit):
        rec.refuse_in_repo(ROOT / "evaluations" / "real-missions-3174" / "b")


def test_b9_a_row_as_committed_carries_no_position_frame_or_map(tmp_path):
    _, row = _record(tmp_path)
    pub = rec.numbers_only(row)
    assert not set(pub) & rec.PRIVATE_KEYS
    text = json.dumps(pub)
    for word in ("x_m", "y_m", "image_base64", "kitchen", "hall", "map_id"):
        assert word not in text, word
    assert pub["pose_samples"] == row["pose_samples"] and pub["distance_m"] == row["distance_m"]


def test_distance_starts_a_new_leg_on_a_new_map():
    track = [{"x_m": 0, "y_m": 0, "map_id": "a"}, {"x_m": 1, "y_m": 0, "map_id": "a"},
             {"x_m": 50, "y_m": 0, "map_id": "b"}, {"x_m": 51, "y_m": 0, "map_id": "b"}]
    assert rec.distance_m(track) == 2.0


def test_the_frame_interval_is_a_setting_and_0_keeps_none(tmp_path):
    def run(every, name):
        w = _World()
        r = rec.Recorder(_Robot(w), _Brain(w), tmp_path / name, frame_every_s=every,
                         clock=w.clock, sleep=w.sleep)
        return r.record(r.wait_for_start(give_up_s=5))
    two, four, none = run(2.0, "a"), run(4.0, "b"), run(0, "c")
    assert abs(two["frames_kept"] - 2 * four["frames_kept"]) <= 1
    assert none["frames_kept"] == 0 and none["walks"] == []
    assert none["pose_coverage"] >= 0.95


def test_review_a_brain_unreachable_for_good_still_gets_a_row(tmp_path):
    """Review 1: the brain never answers again mid-mission; the row is written once
    no status has been read for STATUS_LOST_S, never waited on for ever."""
    w = _World(run_s=10_000.0)
    brain = _Brain(w)
    real = brain.get

    def dies(path):
        if w.t > 20:
            raise ConnectionError("brain gone")
        return real(path)
    brain.get = dies
    r = rec.Recorder(_Robot(w), brain, tmp_path / "b", clock=w.clock, sleep=w.sleep)
    row = r.record(r.wait_for_start(give_up_s=5))
    assert row["row_end"] == "status_lost" and row["outcome"] is None
    assert w.t <= 20 + rec.STATUS_LOST_S + 2


def test_review_a_failing_frame_write_or_a_bad_log_line_never_loses_the_row(tmp_path, monkeypatch):
    """Review 2: a frame write that raises (a full disk) and a cut intervention
    line are counted, and the row is still written."""
    def full(*a, **k):
        raise OSError(28, "No space left on device")
    monkeypatch.setattr(rec, "store_frame", full)
    log = tmp_path / "interventions.jsonl"
    w = _World()
    brain = _Brain(w)
    real = brain.get

    def logs_at_the_end(path):
        if w.t >= 50 and not log.exists():
            log.write_text('{"intervention": {"kind": "stop"}}\n{"interven')   # a cut last line
        return real(path)
    brain.get = logs_at_the_end
    r = rec.Recorder(_Robot(w), brain, tmp_path / "b", interventions=log,
                     clock=w.clock, sleep=w.sleep)
    row = r.record(r.wait_for_start(give_up_s=5))
    assert row["frames_kept"] == 0 and row["frame_errors"] > 0
    assert row["intervention_lines"] == 2 and row["interventions"] == ["stop", "unreadable"]
    assert row["row_end"] == "ended"


def test_review_distance_starts_a_new_leg_on_a_new_frame_in_one_map():
    track = [{"x_m": 0, "y_m": 0, "map_id": "s", "frame_id": None},
             {"x_m": 1, "y_m": 0, "map_id": "s", "frame_id": None},
             {"x_m": 40, "y_m": 0, "map_id": "s", "frame_id": "saved"},
             {"x_m": 41, "y_m": 0, "map_id": "s", "frame_id": "saved"}]
    assert rec.distance_m(track) == 2.0
