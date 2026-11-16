# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.178.F, diagnosis only, TUNING seeds only: E's placed sweep with the
inventory's range rule swapped, to see whether ranging a sighting differently
recovers the passes lost to ranging (its section gives the numbers).

    python evaluations/watch-list-3178/range_rule_f.py closest --placed placements.json \
        --steps 150 --seed 3178 --workers 4 --out /tmp/x.json

`orig` is `inventory.range_at` (the nearest return within RANGE_GATE_DEG);
`closest` takes the return nearest in BEARING within the same gate; `median`
the median range in the gate. The swap is a monkeypatch in this process (and
its forked workers): nothing in `brain/` changes.
"""
import argparse
import importlib.util
import multiprocessing
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))

from brain import inventory as inv  # noqa: E402
from brain.goal_pose import _wrap  # noqa: E402


def load_sweep():
    """`sweep.py` under a unique module name (other evaluations have a
    `sweep.py` too), once per process."""
    name = "watch_sweep_3178"
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, HERE / "sweep.py")
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
    return sys.modules[name]

FRESH = ("5178", "5179", "6178", "6179")     # E's and F's fresh seeds: never here
TUNING = (3178, 4178)                        # B's and D's (search; present is seed + 1)
orig = inv.range_at


def closest(returns, bearing_deg, gate_deg=inv.RANGE_GATE_DEG):
    near = [(abs(_wrap(b - bearing_deg)), r) for b, r in returns
            if abs(_wrap(b - bearing_deg)) <= gate_deg]
    return min(near)[1] if near else None


def median(returns, bearing_deg, gate_deg=inv.RANGE_GATE_DEG):
    near = sorted(r for b, r in returns if abs(_wrap(b - bearing_deg)) <= gate_deg)
    return near[len(near) // 2] if near else None


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=("closest", "median", "orig"))
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--out", required=True)      # never sweep.py's default (a pinned file)
    args, _ = ap.parse_known_args(argv)
    if args.seed not in TUNING:
        raise SystemExit("tuning seeds only: 3178 or 4178 (present runs seed + 1)")
    out = Path(args.out).resolve()
    if out.parent == HERE:
        raise SystemExit("--out must be outside evaluations/watch-list-3178 (its files are pinned)")
    # fork: workers inherit the patched module; under spawn they would not.
    multiprocessing.set_start_method("fork", force=True)
    inv.range_at = {"closest": closest, "median": median, "orig": orig}[args.mode]
    load_sweep().main(argv[1:])


if __name__ == "__main__":
    main()
