# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""tools/plan_claims.py, 3.166: the claim tools' frictions of 2026-11-06/07
(docs/plans/ros-alignment/3.166-claim-tool-fixes.md, F1-F6), each red on
`dev` before the fix (criterion K1). Same scratch repos as 3.139's tests:
a local bare origin, real `sleep` processes as owners."""

import json
import os
import subprocess

import pytest

from tests.test_plan_claims import _git, owners, repo  # noqa: F401
from tests.test_plan_claims_origin import _edit_hook, _out
import plan_claims as pc  # noqa: E402


def _rewrite(repo, name, old, new):
    path = os.path.join(repo, pc.DIR, name)
    text = open(path).read()
    assert old in text
    open(path, "w").write(text.replace(old, new))


def _to_dev(repo, msg):
    _git(repo, "commit", "-qam", msg)
    _git(repo, "push", "-q", "origin", "dev")
    _git(repo, "fetch", "-q", "origin")


def _work(repo, path, msg="work"):
    full = os.path.join(repo, path)
    os.makedirs(os.path.dirname(full), exist_ok=True)
    with open(full, "a") as fh:
        fh.write("x = 1\n")
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", msg)


# ---------- F1: Touches are read from the plan, not frozen in the claim ----------

def test_f1_a_narrowed_row_frees_edits_at_once(repo, owners):
    _, a = owners()
    _, b = owners()
    pc.claim("3.90.A", cwd=repo, me=a)                    # stores `brain/a.py`, `sim/`
    _rewrite(repo, "3.90-test.md", "`brain/a.py`, `sim/` |", "`brain/a.py` |")
    _to_dev(repo, "3.90.A narrowed")
    # sim/x.py is in no step now (3.90.E's glob is sim/maps/*.py): free to B.
    assert _edit_hook(repo, "sim/x.py", b["pid"]) is None
    d = _edit_hook(repo, "brain/a.py", b["pid"])           # still A's
    assert d and d["permissionDecision"] == "deny" and "3.90.A" in d["permissionDecisionReason"]


# ---------- F2: an unclaimed step's top-level directory fences nothing ----------

def test_f2_an_unclaimed_top_level_directory_fences_nothing(repo, owners):
    _, a = owners()
    steps = pc.parse_steps(open(os.path.join(repo, pc.DIR, "3.90-test.md")).read(), "3.90")
    assert pc.tracked("sim/x.py", steps) == (set(), "")          # 3.90.A's `sim/`, unclaimed
    assert pc.tracked("brain/a.py", steps)[1] == "3.90.A"        # a file still fences
    assert pc.tracked("sim/maps/h.py", steps)[1] == "3.90.E"     # a glob still fences
    deep = [dict(steps[0], id="3.90.Q", touches=["evaluations/q/"])]
    assert pc.tracked("evaluations/q/run.py", deep)[1] == "3.90.Q"   # a deeper directory fences
    assert _edit_hook(repo, "sim/x.py", a["pid"]) is None
    # Held, it fences again: another session's claim covers its whole Touches.
    _, b = owners()
    pc.claim("3.90.A", cwd=repo, me=b)
    d = _edit_hook(repo, "sim/x.py", a["pid"])
    assert d and d["permissionDecision"] == "deny"


# ---------- F3: a lost push race, and a release before the push landed ----------

def test_f3_a_step_done_only_on_this_branch_can_be_claimed_again(repo, owners):
    _, a = owners()
    _git(repo, "checkout", "-q", "-b", "plan/3.90-test")
    _rewrite(repo, "3.90-test.md", "| 3.90.A | First | -- | -- | `brain/a.py`, `sim/` | open |",
             "| 3.90.A | First | -- | -- | `brain/a.py`, `sim/` | done |")
    _git(repo, "commit", "-qam", "3.90.A done (the push to dev lost its race)")
    c = pc.claim("3.90.A", cwd=repo, me=a)
    assert c["step"] == "3.90.A"


def test_f3_release_with_unpushed_work_is_refused_unless_asked(repo, owners):
    _, a = owners()
    _git(repo, "checkout", "-q", "-b", "plan/3.90-test")
    pc.claim("3.90.A", cwd=repo, me=a)
    _work(repo, "brain/a.py")                              # not on origin/dev
    with pytest.raises(SystemExit, match="--unpushed"):
        pc.release("3.90.A", cwd=repo, me=a)
    assert pc.release("3.90.A", cwd=repo, me=a, unpushed=True) == ["3.90.A"]


# ---------- F4: a section with no Steps table can be claimed whole ----------

def test_f4_a_section_without_a_table_is_claimable_whole(repo, owners):
    _, a = owners()
    open(os.path.join(repo, pc.DIR, "3.92-notes.md"), "w").write("# 3.92 Notes\n\nNo table.\n")
    _git(repo, "add", "-A")
    _to_dev(repo, "3.92 has no step table")
    assert pc.Board(repo, me=a, remote=False).whole("3.92") is not None
    assert pc.claim("3.92", cwd=repo, me=a)["step"] == "3.92"


# ---------- F5: refusals name the holding claim; allow from any shell ----------

def test_f5_the_push_gate_names_the_holding_claim_and_the_allow_command(repo, owners):
    _, a = owners()
    _, b = owners()
    b = dict(b, session="s-bbbbbbbb")
    pc.claim("3.90.A", cwd=repo, me=a)
    base = _out(repo, "rev-parse", "HEAD")
    _work(repo, "brain/a.py")
    out = pc.push_refusals(base, _out(repo, "rev-parse", "HEAD"), cwd=repo, me=b)
    text = " ".join(out)
    assert "brain/a.py" in text
    assert "allow 3.90.A --session s-bbbbbbbb" in text            # the holding claim, B's id
    assert "plan_section.py claim 3.90.A" not in text             # never "claim" a held step
    # The user's allow, run from a shell, lets the push through as it lets the edit.
    pc.allow("3.90.A", cwd=repo, session="s-bbbbbbbb")
    assert pc.push_refusals(base, _out(repo, "rev-parse", "HEAD"), cwd=repo, me=b) == []


def test_f5_allow_names_a_session_and_works_without_a_claude_process(repo, owners, monkeypatch):
    _, a = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    monkeypatch.setenv("PICAR_CLAIM_PID", "1")                     # no Claude process above
    c = pc.allow("3.90.A", cwd=repo, session="s-bbbbbbbb")
    assert any(x.get("session") == "s-bbbbbbbb" for x in c["allowed"])
    me_b = {"pid": 99999, "start": "x", "session": "s-bbbbbbbb", "host": pc.host()}
    procs = pc.processes()
    assert pc.foreign_claims_on("brain/a.py", pc.repo(repo)[1], procs, me_b) == []


# ---------- F6: --accept-overlap reaches an active, unclaimed same-section branch ----------

def test_f6_accept_overlap_reaches_an_active_unclaimed_branch_of_the_section(repo, owners, tmp_path):
    _, a = owners()
    _, b = owners()
    # A worked on 3.90 in its own worktree, released its claim, and left a commit unpushed.
    wt = str(tmp_path / "wt-a")
    _git(repo, "worktree", "add", "-q", "-b", "plan/3.90-test", wt, "dev")
    _work(wt, "brain/a.py", "unpushed")
    with pytest.raises(pc.ClaimRefused, match="active"):
        pc.claim("3.90.E", cwd=repo, me=b)
    c = pc.claim("3.90.E", cwd=repo, me=b, accept_overlap=True)
    assert any("plan/3.90-test" in x for x in c["accepted_overlap"])
    # Another session's CLAIM still refuses, flag or not.
    pc.release("3.90.E", cwd=repo, me=b)
    _, d = owners()
    pc.claim("3.90.A", cwd=repo, me=d, accept_overlap=True)
    _, e = owners()
    with pytest.raises(pc.ClaimRefused, match="held by|claimed"):
        pc.claim("3.90.E", cwd=repo, me=e, accept_overlap=True)


# ---------- K3: an old claim file is read as before ----------

def test_k3_a_claim_file_written_by_devs_code_is_read_fenced_and_released(repo, owners):
    _, a = owners()
    _, b = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    path = os.path.join(pc.claims_dir(pc.repo(repo)[1]), "3.90.A.json")
    old = json.load(open(path))
    for k in list(old):                                # only dev's fields, nothing new
        if k not in ("step", "section", "title", "touches", "pid", "start", "session", "host",
                     "worktree", "branch", "accepted_overlap", "claimed_at", "at", "sha"):
            old.pop(k)
    json.dump(old, open(path, "w"))
    d = _edit_hook(repo, "brain/a.py", b["pid"])
    assert d and d["permissionDecision"] == "deny"
    assert pc.release("3.90.A", cwd=repo, me=a) == ["3.90.A"]
