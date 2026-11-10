"""3.173.B: tests/test_survey_sweep.py's label-call pins, both arms (the small-room
goal off and on), both labelling modes: calls per survey, rooms labelled at 95%.

    python evaluations/small-rooms-3173/b_label_cost.py
"""
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
import desktop_lock  # noqa: E402
import brain.explore as ex  # noqa: E402
from tests import survey_sweep as sw  # noqa: E402


def main():
    desktop_lock.hold_for_life("shared", f"3.173.B label cost ({__file__})")
    for mode in ("rooms", "spacing"):
        for on in (False, True):
            desktop_lock.step_aside()
            ex.SMALL_ROOM_CLEARANCE_M = 0.25 if on else 0.0
            rs = [sw.run(h, st, mode=mode) for h in sw.HOUSES for st in sw.starts(h)]
            calls = [r["label_calls"] for r in rs]
            good = sum(v >= 0.95 for r in rs for v in r["per_room"].values())
            print(mode, "on" if on else "off", "calls", calls, "median", statistics.median(calls),
                  "max", max(calls), "good", good, "outcomes", sorted({r["outcome"] for r in rs}), flush=True)


if __name__ == "__main__":
    main()
