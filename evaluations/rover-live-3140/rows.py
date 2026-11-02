# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/rover-live-3140/rows.py

3.140.A: the rows, read from the section file (criterion A1). Nothing about
a row is copied into code: its level, check, source and bar come from the
L0-L8 tables of `docs/plans/ros-alignment/3.140-rover-live-checks.md`, and a
*reported* row's bar from amendment 1's table there.

* `load(text)` -> {row id: Row}, refusing a duplicate or an out-of-order ID.
* `coverage(text, index_text)` -> the problems criterion 1 counts (A2).
* `missing_bars(rows)` -> the *reported* rows with no amendment bar (A6).
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT / "docs/plans/ros-alignment/3.140-rover-live-checks.md"
INDEX = ROOT / "docs/plans/ros-alignment/README.md"
SELF = "3.140"
LEVEL_HEAD = re.compile(r"^### L(\d) -- ")
ROW_ID = re.compile(r"^L(\d)\.(\d+)$")
AMENDMENT_HEAD = re.compile(r"^## Amendment 1\b")


class RowsError(ValueError):
    pass


@dataclass(frozen=True)
class Row:
    id: str
    level: int
    check: str
    source: str
    bar: str                      # the table's own cell
    amended_bar: Optional[str]    # amendment 1's bar, for a *reported* row

    @property
    def reported(self) -> bool:
        return "*reported*" in self.bar

    @property
    def decision(self) -> bool:
        return self.level == 8    # L8: decisions handed to the user, never runs


def _cells(line: str) -> list:
    # A table row, split on every pipe: no cell may hold one (the loader's
    # cell count and the ID pattern catch a row that does).
    return [c.strip() for c in line.strip().strip("|").split("|")]


def load(text: str) -> dict:
    rows: dict = {}
    level: Optional[int] = None
    amended: dict = {}
    in_amendment = False
    for line in text.splitlines():
        if line.startswith("## "):
            in_amendment = bool(AMENDMENT_HEAD.match(line))
            level = None
            continue
        m = LEVEL_HEAD.match(line)
        if m:
            level = int(m.group(1))
            continue
        if not line.startswith("| L"):
            continue
        cells = _cells(line)
        if in_amendment:
            if ROW_ID.match(cells[0]):
                if cells[0] in amended:
                    raise RowsError(f"amendment 1 names {cells[0]} twice")
                amended[cells[0]] = cells[1]
            continue
        if level is None or not ROW_ID.match(cells[0]):
            continue
        rid = cells[0]
        lv, n = map(int, ROW_ID.match(rid).groups())
        if lv != level:
            raise RowsError(f"{rid} sits under L{level}'s table")
        if rid in rows:
            raise RowsError(f"{rid} appears twice")
        expected = 1 + sum(1 for r in rows.values() if r.level == level)
        if n != expected:
            raise RowsError(f"{rid} out of order (expected L{level}.{expected})")
        if len(cells) != 4:
            raise RowsError(f"{rid}: {len(cells)} cells, want 4")
        rows[rid] = Row(rid, level, cells[1], cells[2], cells[3], None)
    unknown = sorted(set(amended) - set(rows))
    if unknown:
        raise RowsError(f"amendment 1 names rows that do not exist: {unknown}")
    return {rid: Row(r.id, r.level, r.check, r.source, r.bar, amended.get(rid))
            for rid, r in rows.items()}


def load_plan(path: Path = PLAN) -> dict:
    return load(Path(path).read_text())


def missing_bars(rows: dict) -> list:
    return [r.id for r in rows.values() if r.reported and not r.amended_bar]


# ---------------------------------------------------------------- coverage

SECTION = re.compile(r"3\.(\d+)([a-z]?)")


def _index_sections(index_text: str) -> list:
    out = []
    for line in index_text.splitlines():
        m = re.match(r"^\| \[(3\.\d+[a-z]?)\]", line)
        if m:
            out.append(m.group(1))
    return out


def _named(cell: str, index: tuple = ()) -> set:
    """Sections a coverage cell names: '3.38-3.40 SLAM', '3.51 / 3.54',
    '3.110-3.111, 3.113-3.115'. A range covers every number between, and
    every lettered section of the index inside it (3.50a in 3.49-3.51)."""
    names = set()
    for m in re.finditer(r"3\.(\d+)([a-z]?)(?:\s*-\s*3\.(\d+))?", cell):
        a, suffix, b = int(m.group(1)), m.group(2), m.group(3)
        if b is None:
            names.add(f"3.{a}{suffix}")
        else:
            names.update(f"3.{i}" for i in range(a, int(b) + 1))
            for s in index:
                m2 = SECTION.fullmatch(s)
                if m2 and a <= int(m2.group(1)) <= int(b):
                    names.add(s)
    return names


def coverage(text: str, index_text: str, rows: Optional[dict] = None) -> list:
    """Criterion 1's problems, as lines; [] when every section in the index
    (but this one) has a row or an exclusion, and every row named exists."""
    rows = rows if rows is not None else load(text)
    body = text.split("## Coverage", 1)
    if len(body) < 2:
        return ["no ## Coverage section"]
    table = body[1].split("\n## ", 1)[0]
    index = tuple(_index_sections(index_text))
    covered: dict = {}
    for line in table.splitlines():
        if not line.startswith("| 3."):
            continue
        cells = _cells(line)
        # Two section/rows pairs a line, side by side, an empty cell between.
        for sec, what in ((cells[0], cells[1]), (cells[3], cells[4]) if len(cells) >= 5 else ("", "")):
            if not sec:
                continue
            for name in _named(sec, index):
                covered.setdefault(name, []).append(what)
    problems = []
    for name in index:
        if name == SELF:
            continue
        whats = covered.get(name)
        if not whats:
            problems.append(f"{name}: in the index, not in the coverage table")
            continue
        for what in whats:
            if what.startswith(("excluded", "see 3.")):
                continue
            ids = re.findall(r"L(\d)\.(\d+)(?:-L\d\.(\d+))?", what)
            if not ids:
                problems.append(f"{name}: '{what}' names no row and is no exclusion")
            for lv, a, b in ids:
                for i in range(int(a), int(b or a) + 1):
                    if f"L{lv}.{i}" not in rows:
                        problems.append(f"{name}: names L{lv}.{i}, which does not exist")
    for name, whats in covered.items():
        for what in whats:
            m = re.match(r"see (3\.\d+[a-z]?)", what)
            if m and m.group(1) not in covered:
                problems.append(f"{name}: 'see {m.group(1)}', which has no row")
    return problems
