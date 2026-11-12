# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""CLAUDE.md stays under its hard size budget (tools/claude_md_budget.py)."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

import claude_md_budget as cmb  # noqa: E402


def test_claude_md_is_under_its_hard_budget():
    assert cmb.size() <= cmb.HARD, cmb.report()


def test_sections_add_up_to_the_file():
    text = open(cmb.PATH, encoding="utf-8").read()
    assert sum(n for _, n in cmb.sections(text)) == len(text.encode())
    assert any(h.startswith("### 3b.") for h, _ in cmb.sections(text))


def test_merged_state_guide_exists():
    assert os.path.exists(os.path.join(cmb.ROOT, "docs", "guides", "MERGED-STATE.md"))
