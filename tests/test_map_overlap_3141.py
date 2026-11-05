# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.141.F F0's scorer (`evaluations/wheel-loop-3141/map_overlap.py`) on a
made-up trace: a tick overlaps a map reply only when their spans meet, the
chance level widens each reply by the tick's length, replies are clipped to
the window, and the poller switch is read."""

import importlib.util
import json
from pathlib import Path

import pytest

_P = Path(__file__).resolve().parents[1] / "evaluations/wheel-loop-3141/map_overlap.py"
_spec = importlib.util.spec_from_file_location("map_overlap_3141", _P)
mo = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mo)


def _run(tmp_path, rows, on="1"):
    b = tmp_path / "board"
    b.mkdir()
    (b / "window_start").write_text("100.0")
    (b / "window_end").write_text("200.0")
    (b / "mappoll_on").write_text(on)
    (b / "trace3141.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    (tmp_path / "summary.json").write_text(json.dumps({"wheel": {"late": 1}, "missions": {"n": 2}}))
    return mo.score(tmp_path)


def test_overlap_chance_and_clipping(tmp_path):
    rows = [
        {"k": "req", "path": "/world/map", "t0": 110.0, "ms": 100.0},   # 110.0-110.1
        {"k": "req", "path": "/world/map", "t0": 99.95, "ms": 100.0},   # starts before the window
        {"k": "req", "path": "/scan", "t0": 150.0, "ms": 500.0},         # not a map reply
        {"k": "tick", "t": 110.15, "dt_ms": "105.0"},                    # 110.045-110.15: meets
        {"k": "tick", "t": 100.02, "dt_ms": "90.0"},                     # meets the early reply
        {"k": "tick", "t": 150.2, "dt_ms": "120.0"},                     # only the /scan: no
        {"k": "vet", "t": 110.05, "ms": 20.0},
        {"k": "vet", "t": 160.0, "ms": 20.0, "reason": "clamped"},       # clamped: not counted
    ]
    got = _run(tmp_path, rows)
    assert got["probe_ticks_over_80_ms"] == {"n": 3, "overlap_map": 2,
                                             "chance": round((0.05 + 0.18) / 100, 4)}
    assert got["probe_ticks_over_100_ms"]["n"] == 2 and got["probe_ticks_over_100_ms"]["overlap_map"] == 1
    assert got["map_in_flight_share"] == round(0.15 / 100, 4)            # clipped to the window
    assert got["map_replies_over_req_ms"]["n"] == 2
    assert got["unclamped_vets_over_15_ms"] == {"n": 1, "overlap_map": 1}
    assert got["mappoll_on"] is True and got["missions"] == {"n": 2} and got["wheel"] == {"late": 1}


def test_poller_off_and_no_trace(tmp_path):
    assert _run(tmp_path, [], on="0")["mappoll_on"] is False
    (tmp_path / "board" / "trace3141.jsonl").unlink()
    with pytest.raises(SystemExit):
        mo.score(tmp_path)
