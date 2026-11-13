# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.178.A: what the recorded walks already hold of the watch list, pinned.

The counts come from committed files only (3.177's pinned set, 3.46's
stored-object records), so they run in the suite with no frames or models.
"""

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
HERE = ROOT / "evaluations" / "watch-list-3178"


def _count():
    spec = importlib.util.spec_from_file_location("watch_list_count_3178", HERE / "count.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_no_list_item_is_a_labelled_target():
    """The human reference holds no list item: 21 walks, 2,261 frames, six
    targets, none of them keys, wallet, phone, glasses or remote."""
    ref = _count().count()["labelled"]
    assert (ref["walks"], ref["frames"]) == (21, 2261)
    assert ref["set_hash"] == "00d12dbc44f6495a927a2a58d53887fc58c9e468e5774d65a8d90288d364a2d6"
    assert sorted(ref["targets"]) == [   # folded by 3.177's names: burgundy is the red one
        "a white phone charger cable", "blue and yellow running shoes", "blue bottle",
        "grey backpack", "red backpack", "woven laundry basket"]
    assert all(c == {"walks": 0, "frames": 0, "visible": 0} for c in ref["per_item"].values())


def test_the_detector_named_a_list_item_once():
    """3.46's per-frame best box for the walk's own target: one frame named
    "phone" in each variant on the six search walks, none on the 15 others."""
    s = _count().count()["surfaced"]
    hits = {name: {k: v for k, v in d["exact"].items() if v} for name, d in s.items()}
    phone = ["blue-bottle-20260907-185007/frame-0003.jpg"]
    assert hits == {"memory-all.json": {"phone": phone},
                    "memory-objects.json": {"phone": phone},
                    "memory-heldout-all.json": {},
                    "memory-heldout-objects.json": {}}
    assert sum(d["frames"] for d in s.values()) == 2 * 2261


def test_exact_names_only():
    m = _count()
    assert m.item_of("Phone ") == "phone"
    assert m.item_of("TV remote") == "remote"
    assert m.item_of("phone charger cable") is None
    assert m.item_of("phone box") is None
    assert m.item_of("a white phone charger cable") is None
    assert m.item_of("telephone") is None    # a landline, not a lost phone
    assert m.item_of("purse") is None        # a handbag, not a wallet
    assert m.item_of("phones") == "phone"
    assert m._WORDS.search("headphones") and not m._WORDS.search("keyboard")


def test_unknown_arguments_are_refused():
    import pytest
    with pytest.raises(SystemExit):
        _count().main(["-w"])


def test_counts_file_is_current():
    m = _count()
    on_disk = json.loads((HERE / "counts.json").read_text())
    assert on_disk == json.loads(json.dumps(m.count(), sort_keys=True))
