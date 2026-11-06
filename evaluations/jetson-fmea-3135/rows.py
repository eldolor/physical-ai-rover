# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/rows.py

3.135.B, criterion 3, the BOARD side: one FMEA row run `--runs` times
(`fmea.py`, one fresh stack a run), then the row's verdict (amendment 4).

* A run is *scored* when it ran to a result and was valid (`error` absent,
  `verdict.valid` true: the board was driving forward at the fault). An
  errored or invalid run is recorded, not scored, and replaced by one more
  run, at most `EXTRA` replacements a row.
* The row is **met** when it has `--runs` scored runs and every one passed;
  **FAILED** when any scored run failed; **incomplete** when the
  replacements ran out first (not met, and not a guard's failure).

    python evaluations/jetson-fmea-3135/rows.py F9 --out DIR --runs 5 [--plant P]

Each run lands in `<out>/<row>-<i>/` (its `result.json`, stack logs and the
truth log gzipped); the verdict in `<out>/row.json`.
"""

from __future__ import annotations

import argparse
import gzip
import json
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

EXTRA = 3


def scored(rec: dict) -> bool:
    return rec.get("error") is None and (rec.get("verdict") or {}).get("valid") is True


def verdict(recs: list, runs: int) -> dict:
    """The row's verdict from its runs, in order (pure; the tests feed it)."""
    s = [r for r in recs if scored(r)][:runs]
    failed = [r for r in s if not r["verdict"]["pass"]]
    state = "FAILED" if failed else "met" if len(s) == runs else "incomplete"
    return {"state": state, "runs_wanted": runs, "runs_scored": len(s), "runs_total": len(recs),
            "runs_failed": len(failed),
            "not_scored": [{"i": r.get("i"), "error": r.get("error"),
                            "reasons": (r.get("verdict") or {}).get("reasons")}
                           for r in recs if not scored(r)],
            "reasons": sorted({why for r in failed for why in r["verdict"]["reasons"]})}


def gz(path: Path) -> None:
    if path.exists():
        with open(path, "rb") as src, gzip.open(f"{path}.gz", "wb") as dst:
            shutil.copyfileobj(src, dst)
        path.unlink()


def runner(row: str):
    """The module that runs a row: the stop rows (`fmea.py`) or the cloud
    rows (`cloud_rows.py`), the clock row (`sys_rows.py`) or the VE rows (`vlm_rows.py`)."""
    import cloud_rows
    import fmea
    import sys_rows
    import vlm_rows
    for mod in (fmea, cloud_rows, sys_rows, vlm_rows):
        if row in mod.ROWS:
            return mod
    raise SystemExit(f"no such row {row!r}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("row")
    ap.add_argument("--out", required=True)
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--plant", default="")
    ap.add_argument("--house", default="scaled_house")
    a = ap.parse_args(argv)
    runner(a.row)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    recs = []
    for i in range(1, a.runs + EXTRA + 1):
        if sum(scored(r) for r in recs) >= a.runs:
            break
        d = out / f"{a.row}-{i}"
        rec = runner(a.row).run_row(a.row, d, a.plant, a.house)
        rec["i"] = i
        recs.append(rec)
        gz(d / "truth.jsonl")
        print(json.dumps({"i": i, "verdict": rec["verdict"], "error": rec.get("error")}), flush=True)
    v = {"row": a.row, "plant": a.plant or None, "house": a.house, **verdict(recs, a.runs)}
    (out / "row.json").write_text(json.dumps(v, indent=1))
    print(json.dumps(v))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
