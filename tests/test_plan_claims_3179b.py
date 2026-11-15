# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""tools/plan_claims.py, 3.179.B: release where branch deletes are refused.

Claude Code's cloud proxy refuses a branch delete (HTTP 403, measured
2026-11-15) but allows a lease-guarded update, so a release there turns
`claim/<step>` into a "released" record that every reader takes as no
claim, and the next `claim` replaces it under a lease. An origin that
refuses deletes is simulated by a `pre-receive` hook on the local bare
repo; no test reaches the real origin."""

import os
import socket
import subprocess
import sys
import time

import pytest

from tests.test_plan_claims import TOOLS, _git, owners, repo  # noqa: F401
from tests.test_plan_claims_origin import fake_claim, local_files, other_machine, remote_refs
import plan_claims as pc  # noqa: E402

ZERO = "0" * 40
REFUSE_DELETES = f"""#!/bin/sh
while read old new ref; do
  if [ "$new" = "{ZERO}" ]; then
    echo "deletes are refused here (the cloud's proxy answers 403)" >&2
    exit 1
  fi
done
"""


def refuse_deletes(repo):
    """Make the bare origin refuse every ref deletion, as the cloud does."""
    bare = subprocess.run(["git", "remote", "get-url", "origin"], cwd=repo,
                          capture_output=True, text=True, check=True).stdout.strip()
    hook = os.path.join(bare, "hooks", "pre-receive")
    with open(hook, "w") as fh:
        fh.write(REFUSE_DELETES)
    os.chmod(hook, 0o755)


def message(repo, sha):
    return subprocess.run(["git", "log", "-1", "--format=%B", sha], cwd=repo,
                          capture_output=True, text=True, check=True).stdout


def released(repo, step):
    """The released record on claim/<step>, parsed, or None."""
    sha = remote_refs(repo).get(step)
    if not sha:
        return None
    msg = message(repo, sha)
    return pc.parse_released_message(msg) if msg.startswith("released ") else None


# ---------- criteria 1-3: release, read as free, claimed again ----------

def test_criterion_1_release_where_deletes_are_refused_leaves_a_released_record(repo, owners):
    _, a = owners()
    c = pc.claim("3.90.A", cwd=repo, me=a)
    refuse_deletes(repo)
    assert pc.release("3.90.A", cwd=repo, me=a) == ["3.90.A"]
    rec = released(repo, "3.90.A")
    assert rec is not None and rec["was"] == c["sha"] and rec["forced"] == "no"
    assert local_files(repo) == []


def test_criterion_2_a_released_record_reads_as_no_claim(repo, owners, monkeypatch):
    _, a = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    refuse_deletes(repo)
    pc.release("3.90.A", cwd=repo, me=a)
    remote, err = pc.remote_claims(repo)
    assert err is None and [x for x in remote if x["step"] == "3.90.A"] == []
    _, b = owners()
    board = pc.Board(repo, b)
    assert not [x for x in board.live if x["step"] == "3.90.A"]
    assert "free      3.90.A" in pc.cmd_board(b=board)
    monkeypatch.setenv("PICAR_CLAIM_PID", str(b["pid"]))
    assert pc.cmd_next(cwd=repo).startswith("3.90.A")


def test_criterion_3_another_session_claims_a_released_step(repo, owners):
    _, a = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    refuse_deletes(repo)
    pc.release("3.90.A", cwd=repo, me=a)
    _, b = owners()
    c = pc.claim("3.90.A", cwd=repo, me=b)
    assert remote_refs(repo)["3.90.A"] == c["sha"]
    assert pc.parse_claim_message(message(repo, c["sha"]))["pid"] == b["pid"]


# ---------- criterion 4: never over a live claim ----------

def test_criterion_4_a_stale_lease_never_overwrites(repo, owners):
    _, a = owners()
    first = pc.claim("3.90.A", cwd=repo, me=a)["sha"]
    refuse_deletes(repo)
    pc.release("3.90.A", cwd=repo, me=a)
    rec_sha = remote_refs(repo)["3.90.A"]
    _, b = owners()
    newer = pc.claim("3.90.A", cwd=repo, me=b)["sha"]
    # A releaser and a claimer that both read the ref before b's claim landed:
    ok, rejected, _ = pc._push_released(repo, "3.90.A", first, b, forced=False)
    assert not ok and rejected
    ok, rejected, _ = pc._push_replace(repo, "3.90.A", first, rec_sha)
    assert not ok and rejected
    assert remote_refs(repo)["3.90.A"] == newer


# ---------- criterion 5: no two holders ----------

RACER = """
import os, sys, time
sys.path.insert(0, {tools!r})
import plan_claims as pc
while not os.path.exists({go!r}):
    time.sleep(0.001)
try:
    pc.claim("3.90.A", cwd={repo!r})
    print("WIN")
except SystemExit as e:
    print("REFUSED", e)
"""


def test_criterion_5_racers_on_a_released_step_one_winner(repo, owners, tmp_path):
    _, a = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    refuse_deletes(repo)
    pc.release("3.90.A", cwd=repo, me=a)
    far = other_machine(repo, tmp_path)
    holders = [owners()[0] for _ in range(20)]
    go = str(tmp_path / "go")
    procs = []
    for i, h in enumerate(holders):
        where, name = (repo, "near") if i % 2 else (far, "far")
        procs.append(subprocess.Popen(
            [sys.executable, "-c", RACER.format(tools=TOOLS, go=go, repo=where)],
            stdout=subprocess.PIPE, text=True,
            env={**os.environ, "PICAR_CLAIM_PID": str(h.pid), "PICAR_CLAIM_HOST": name}))
    time.sleep(0.5)
    open(go, "w").close()
    outs = [p.communicate(timeout=120)[0] for p in procs]
    assert sum(o.startswith("WIN") for o in outs) == 1, outs
    assert all(o.startswith(("WIN", "REFUSED")) for o in outs), outs
    assert local_files(repo) + local_files(far) == ["3.90.A"]
    assert not message(repo, remote_refs(repo)["3.90.A"]).startswith("released ")


# ---------- criterion 6: the user's release --force ----------

def test_criterion_6_release_force_writes_a_forced_record(repo, owners):
    _, a = owners()
    theirs = fake_claim(repo, "3.90.A", host="far", session="s-far00001")
    refuse_deletes(repo)
    assert pc.release("3.90.A", cwd=repo, me=a, force=True) == ["3.90.A"]
    rec = released(repo, "3.90.A")
    assert rec is not None and rec["forced"] == "yes" and rec["was"] == theirs


def test_criterion_6_without_force_another_sessions_claim_is_untouched(repo, owners):
    _, a = owners()
    theirs = fake_claim(repo, "3.90.A", host="far", session="s-far00001")
    refuse_deletes(repo)
    with pytest.raises(SystemExit, match="refused"):
        pc.release("3.90.A", cwd=repo, me=a)
    assert remote_refs(repo)["3.90.A"] == theirs


# ---------- criterion 7: where deletes work, release deletes ----------

def test_criterion_7_where_deletes_work_the_ref_is_deleted(repo, owners):
    _, a = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    assert pc.release("3.90.A", cwd=repo, me=a) == ["3.90.A"]
    assert "3.90.A" not in remote_refs(repo)


def test_criterion_7_a_released_record_is_deleted_by_a_later_claimer_s_release(repo, owners):
    """Once the step is claimed again and released where deletes work, the
    ref goes: released records do not pile up on such a machine."""
    _, a = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    refuse_deletes(repo)
    pc.release("3.90.A", cwd=repo, me=a)
    os.remove(os.path.join(subprocess.run(["git", "remote", "get-url", "origin"], cwd=repo,
                                          capture_output=True, text=True).stdout.strip(),
                           "hooks", "pre-receive"))
    _, b = owners()
    pc.claim("3.90.A", cwd=repo, me=b)
    assert pc.release("3.90.A", cwd=repo, me=b) == ["3.90.A"]
    assert "3.90.A" not in remote_refs(repo)


# ---------- criterion 8: a legacy claim, released by its owner ----------

def test_criterion_8_a_legacy_claim_is_released_by_its_owner(repo, monkeypatch):
    monkeypatch.setenv("CLAUDE_CODE_REMOTE", "true")
    monkeypatch.setenv("CLAUDE_CODE_REMOTE_SESSION_ID", "cse_mine")
    try:
        ns = str(os.stat("/proc/self/ns/pid").st_ino)
    except OSError:
        ns = ""
    legacy = pc._digest("h-", f"{socket.gethostname()}|{ns}")
    theirs = fake_claim(repo, "3.90.A", host=legacy, session="s-mine0001")
    refuse_deletes(repo)
    me = {"pid": 4242424, "start": "x", "session": "s-mine0001", "host": pc.host()}
    assert pc.release("3.90.A", cwd=repo, me=me) == ["3.90.A"]
    assert released(repo, "3.90.A")["was"] == theirs


# ---------- criterion 9: public-safe ----------

def test_criterion_9_a_released_record_holds_digests_shas_and_a_time(repo, owners, monkeypatch):
    monkeypatch.setenv("CLAUDE_CODE_SESSION_ID", "raw-session-id-0001")
    _, a = owners()
    a = {**a, "session": pc.session_id()}
    pc.claim("3.90.A", cwd=repo, me=a)
    refuse_deletes(repo)
    pc.release("3.90.A", cwd=repo, me=a)
    msg = message(repo, remote_refs(repo)["3.90.A"])
    assert "raw-session-id" not in msg and socket.gethostname() not in msg.replace("vm", "")
    assert repo not in msg and "/" not in msg
    rec = pc.parse_released_message(msg)
    assert set(rec) == {"step", "by", "forced", "was", "at"}
    assert rec["by"].startswith("s-") and len(rec["was"]) == 40
