# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.162.B: what a mission saw, kept on the robot
(docs/plans/ros-alignment/3.162-what-i-saw.md: W3's storage bounds, W6 local
only, the found frame kept as judged; W1's every-found and W7's mission-loop
checks need the end-of-mission hook, 3.162.C)."""

import hashlib
import io
import json
import random
import subprocess
from pathlib import Path

import pytest
from PIL import Image

from brain import what_i_saw as wis

ROOT = Path(__file__).resolve().parents[1]


def _jpeg(w=640, h=480, seed=0, q=90) -> bytes:
    rng = random.Random(seed)
    img = Image.new("RGB", (w, h))
    img.putdata([(rng.randrange(256), rng.randrange(256), rng.randrange(256)) for _ in range(w * h)])
    buf = io.BytesIO()
    img.save(buf, "JPEG", quality=q)
    return buf.getvalue()


FRAME = _jpeg(320, 240, 1, 70)


def _looks(n, frame=FRAME):
    return [{"label": f"thing {i}", "x_m": 0.1 * i, "y_m": 1.0, "room": "kitchen",
             "belief": 1.0 - i / (n + 1), "looks": 2, "jpeg": frame,
             "box": [0.2, 0.2, 0.6, 0.7]} for i in range(n)]


def test_the_found_frame_is_kept_as_judged_with_its_hash(tmp_path):
    store = wis.ReportStore(tmp_path)
    rep = store.save("m-001", {"outcome": "found", "target": "red backpack", "job": "find"},
                     found={"jpeg": FRAME, "box": [0.4, 0.3, 0.6, 0.8], "confirmed": True},
                     looks=_looks(3))
    assert rep["found"]["sha256"] == hashlib.sha256(FRAME).hexdigest()
    assert store.image("m-001", "found.jpg") == FRAME
    got = store.get("m-001")
    assert got["found"]["confirmed"] is True and got["target"] == "red backpack"
    assert [m["thumb"] for m in got["landmarks"]] == ["thumbs/0.jpg", "thumbs/1.jpg", "thumbs/2.jpg"]
    t = Image.open(io.BytesIO(store.image("m-001", "thumbs/0.jpg")))
    assert max(t.size) <= wis.THUMB_PX


def test_an_oversized_frame_is_stored_smaller_but_keeps_the_sent_frames_hash(tmp_path):
    big = _jpeg(1280, 960, 2, 95)
    assert len(big) > wis.FOUND_MAX_BYTES
    rep = wis.ReportStore(tmp_path).save("m-big", {"outcome": "found"}, found={"jpeg": big})
    stored = (tmp_path / "m-big" / "found.jpg").read_bytes()
    assert len(stored) <= wis.FOUND_MAX_BYTES and rep["found"]["resized"] is True
    assert rep["found"]["sha256"] == hashlib.sha256(big).hexdigest()


def test_a_mission_is_bounded_in_thumbnails_and_bytes(tmp_path):
    noisy = _jpeg(640, 480, 3, 95)
    rep = wis.ReportStore(tmp_path).save("m-many", {"outcome": "exhausted"},
                                         found={"jpeg": _jpeg(1280, 960, 4, 95)}, looks=_looks(90, noisy))
    d = tmp_path / "m-many"
    thumbs = list((d / "thumbs").iterdir())
    assert len(thumbs) <= wis.MAX_THUMBS
    assert all(p.stat().st_size <= wis.THUMB_MAX_BYTES for p in thumbs)
    assert sum(p.stat().st_size for p in d.rglob("*") if p.is_file()) <= wis.MISSION_MAX_BYTES
    kept = [m for m in rep["landmarks"] if m["thumb"]]
    beliefs = [m["belief"] for m in rep["landmarks"]]
    assert beliefs == sorted(beliefs, reverse=True)          # the highest belief keeps its thumbnail
    assert kept == rep["landmarks"][:len(kept)]


def test_the_store_keeps_the_newest_and_is_bounded(tmp_path, monkeypatch):
    monkeypatch.setattr(wis, "KEEP", 3)
    store = wis.ReportStore(tmp_path)
    for i in range(5):
        store.save(f"m-{i}", {"outcome": "found"}, found={"jpeg": FRAME})
        (tmp_path / f"m-{i}" / "report.json").touch()
    assert [r["mission_id"] for r in store.list()][:3] and len(store.list()) == 3
    assert not (tmp_path / "m-0").exists() and (tmp_path / "m-4").exists()
    monkeypatch.setattr(wis, "STORE_MAX_BYTES", 1)          # over the byte bound: only the newest kept
    store.save("m-9", {"outcome": "found"}, found={"jpeg": FRAME})
    assert [p.name for p in tmp_path.iterdir() if not p.name.startswith(".")] == ["m-9"]


def test_a_delete_leaves_no_file_and_names_outside_a_report_are_refused(tmp_path):
    store = wis.ReportStore(tmp_path)
    store.save("m-1", {"outcome": "found"}, found={"jpeg": FRAME}, looks=_looks(2))
    assert store.delete("m-1") is True
    assert list(tmp_path.iterdir()) == []
    assert store.delete("m-1") is False
    store.save("m-2", {"outcome": "found"}, found={"jpeg": FRAME})
    for mid, name in (("m-2", "report.json"), ("m-2", "../m-2/found.jpg"), ("m-2", "thumbs/x.jpg"),
                      ("..", "found.jpg"), ("m-2/../..", "found.jpg"), (".hidden", "found.jpg")):
        with pytest.raises(wis.ReportRefused):
            store.image(mid, name)
    with pytest.raises(wis.ReportRefused):
        store.get("nope")


def test_the_list_is_newest_first_without_images(tmp_path):
    store = wis.ReportStore(tmp_path)
    store.save("m-a", {"outcome": "exhausted", "job": "survey"})
    import time
    time.sleep(1.1)
    store.save("m-b", {"outcome": "found", "target": "mug", "job": "find"}, found={"jpeg": FRAME})
    rows = store.list()
    assert [r["mission_id"] for r in rows] == ["m-b", "m-a"]
    assert rows[0]["found"] is True and rows[1]["found"] is False
    assert all("jpeg" not in json.dumps(r) for r in rows)


def test_the_writer_never_raises_and_never_blocks(tmp_path):
    blocker = tmp_path / "a-file"
    blocker.write_text("not a directory")
    w = wis.ReportWriter(wis.ReportStore(blocker))           # every save fails
    assert w.submit("m-1", {"outcome": "found"}, found={"jpeg": FRAME}) is True
    assert w.flush(5.0) and w.failed == 1 and w.saved == 0
    ok = wis.ReportWriter(wis.ReportStore(tmp_path / "ok"))
    ok.submit("m-2", {"outcome": "found"}, found={"jpeg": FRAME})
    assert ok.flush(5.0) and ok.saved == 1


def test_frames_stay_local_no_upload_path_reads_the_store():
    """W6: only this module, the brain's routes and hook (3.162.C) and their
    tests name the store; the inventory's S3 copy
    (control/inventory_store.py) carries no image."""
    out = subprocess.run(["git", "grep", "--untracked", "-l", "-e", "what_i_saw", "-e", "WHAT_I_SAW_DIR",
                          "--", "*.py"], cwd=ROOT, capture_output=True, text=True).stdout.split()
    assert sorted(out) == ["brain/what_i_saw.py", "control/brain_server.py", "control/mission_runner.py",
                           "tests/test_ui_what_i_saw.py", "tests/test_what_i_saw.py",
                           "tests/test_what_i_saw_live.py", "tests/test_what_i_saw_sweep.py"]
    assert "jpeg" not in (ROOT / "control" / "inventory_store.py").read_text().lower()


# ---------- review fixes (code-review + Thermos, round 1) ----------

def test_review_pruning_follows_the_stores_order_not_the_clock(tmp_path, monkeypatch):
    """A board with no RTC can set its clock back: the report just written is
    never the one pruned."""
    import os
    monkeypatch.setattr(wis, "KEEP", 2)
    store = wis.ReportStore(tmp_path)
    store.save("m-a", {"outcome": "found"})
    store.save("m-b", {"outcome": "found"})
    os.utime(tmp_path / "m-a" / "report.json", (4e9, 4e9))     # an older report, a clock in the future
    store.save("m-c", {"outcome": "found"})                    # written with the clock set back
    assert sorted(p.name for p in tmp_path.iterdir()) == ["m-b", "m-c"]
    assert [r["mission_id"] for r in store.list()] == ["m-c", "m-b"]


def test_review_orphans_are_swept_and_a_failed_person_delete_is_not_called_done(tmp_path, monkeypatch):
    import os
    store = wis.ReportStore(tmp_path)
    orphan = tmp_path / ".m-x.abc123"
    (orphan / "thumbs").mkdir(parents=True)
    (orphan / "found.jpg").write_bytes(FRAME)
    os.utime(orphan, (1, 1))
    store.save("m-1", {"outcome": "found"})
    assert not orphan.exists()
    monkeypatch.setattr(wis.shutil, "rmtree", lambda *a, **k: (_ for _ in ()).throw(OSError("busy")))
    with pytest.raises(OSError):
        store.delete("m-1")


def test_review_report_json_is_capped_and_clean(tmp_path):
    import numpy as np
    looks = [{"label": "x" * 40 + str(i), "x_m": np.float32(1.5), "y_m": float("nan"),
              "room": "kitchen", "belief": np.float64(0.5), "looks": np.int64(2)} for i in range(500)]
    store = wis.ReportStore(tmp_path)
    rep = store.save("m-1", {"outcome": "found", "step": np.int64(4)}, looks=looks)
    body = (tmp_path / "m-1" / "report.json").read_bytes()
    assert len(body) <= wis.REPORT_MAX_BYTES
    assert len(rep["landmarks"]) <= wis.MAX_ROWS
    got = json.loads(body)                                       # strict JSON: no NaN
    assert got["landmarks"][0]["y_m"] is None and got["landmarks"][0]["x_m"] == 1.5
    assert got["step"] == 4


def test_review_a_damaged_found_frame_loses_itself_only(tmp_path):
    rep = wis.ReportStore(tmp_path).save("m-1", {"outcome": "found"},
                                         found={"jpeg": b"\x00" * 200_000}, looks=_looks(2))
    assert rep["found"] is None and len(rep["landmarks"]) == 2


def test_review_a_pruning_error_does_not_fail_a_saved_report(tmp_path, monkeypatch):
    store = wis.ReportStore(tmp_path)
    monkeypatch.setattr(store, "_prune", lambda keep=None: (_ for _ in ()).throw(OSError("vanished")))
    store.save("m-1", {"outcome": "found"}, found={"jpeg": FRAME})
    assert store.get("m-1")["outcome"] == "found"


def test_review2_clean_keeps_big_ints_and_bools_and_a_bad_belief_saves(tmp_path):
    import numpy as np
    assert wis._clean(2 ** 60 + 1) == 2 ** 60 + 1
    assert wis._clean(np.int64(2 ** 60 + 1)) == 2 ** 60 + 1
    assert wis._clean(np.bool_(True)) is True
    assert wis._clean(10 ** 400) == 10 ** 400
    rep = wis.ReportStore(tmp_path).save("m-1", {"outcome": "found"},
                                         looks=[{"label": "a", "belief": "high"}, {"label": "b", "belief": 0.4}])
    assert [m["label"] for m in rep["landmarks"]] == ["b", "a"]
    (tmp_path / "junk").mkdir()
    (tmp_path / "junk" / "report.json").write_text("[1, 2]")       # valid JSON, not a report
    wis.ReportStore(tmp_path).save("m-2", {"outcome": "found"})
    assert [r["mission_id"] for r in wis.ReportStore(tmp_path).list()] == ["m-2", "m-1"]
