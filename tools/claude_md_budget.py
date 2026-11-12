# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tools/claude_md_budget.py

CLAUDE.md is loaded into every Claude Code session's context, so every
byte of it is paid on every session start (2026-11-12, the user: a cleared
session already sat at 13%). This keeps it under a size budget.

Run:  python tools/claude_md_budget.py           sizes per section, and the verdict
      python tools/claude_md_budget.py --check   exit 1 when over SOFT (a trim is due)

tests/test_claude_md_budget.py fails the suite above HARD only, so a merge
that adds its line never breaks another session's push; the weekly trim
(a scheduled task on the desktop) brings it back under SOFT.

How to trim (the procedure the weekly task follows):

1. Work on a branch cut from origin/dev (`tools/claude-md-trim-<date>`),
   never on a plan/3.N branch; this is shared status, not a plan step.
2. Move, do not delete. In section 3b, a line for a merged or closed
   section goes to its area in `docs/guides/MERGED-STATE.md`; an entry
   longer than two lines is cut to one, its detail left in the section's
   own file (`docs/plans/ros-alignment/3.N-*.md`).
3. Elsewhere, text only some sessions need moves to the guide or spec that
   owns it, leaving a one-line pointer. Never drop a rule a test enforces,
   or a dated decision of the user's, without that pointer.
4. Before removing anything that does not move to MERGED-STATE.md, copy
   CLAUDE.md verbatim to `docs/archive/CLAUDE-<date>.md`, as earlier trims
   did, and name that copy at the top of CLAUDE.md.
5. Check: this tool under SOFT, `python tools/spec_lint.py`, then
   `python tools/plan_section.py ready` and push to dev.
"""

from __future__ import annotations

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH = os.path.join(ROOT, "CLAUDE.md")
SOFT = 38_000          # bytes: a trim is due above this
HARD = 46_000          # bytes: the suite fails above this

_HEAD = re.compile(r"^#{2,3} ")


def sections(text: str) -> list:
    """(heading, bytes) for every `## ` and `### ` section, in order."""
    out, head, n = [], "(preamble)", 0
    for line in text.splitlines(keepends=True):
        if _HEAD.match(line):
            out.append((head, n))
            head, n = line.strip(), 0
        n += len(line.encode())
    out.append((head, n))
    return out


def size(path: str = PATH) -> int:
    return os.path.getsize(path)


def report(path: str = PATH) -> str:
    text = open(path, encoding="utf-8").read()
    total = len(text.encode())
    rows = [f"{n:7d}  {h}" for h, n in sections(text)]
    verdict = ("OVER HARD: the suite fails; trim now" if total > HARD else
               "over SOFT: a trim is due" if total > SOFT else "under budget")
    return "\n".join(rows + [f"{total:7d}  total (SOFT {SOFT}, HARD {HARD}): {verdict}"])


def main(argv) -> int:
    print(report())
    return 1 if "--check" in argv and size() > SOFT else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
