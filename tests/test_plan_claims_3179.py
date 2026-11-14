# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""tools/plan_claims.py, 3.179: a claim's host is unique per cloud
container. Every Claude Code cloud container has the same host name and
pid namespace, so before 3.179 every cloud claim carried one host and each
cloud session read the others' claims as its own machine's, with the
process gone: free, and taken over. A container is simulated here by
`CLAUDE_CODE_REMOTE_SESSION_ID`; origin is a local bare repo (`repo`)."""

import os
import re
import socket
import subprocess

import pytest

from tests.test_plan_claims import _git, owners, repo  # noqa: F401
from tests.test_plan_claims_origin import fake_claim, remote_refs
import plan_claims as pc  # noqa: E402

REMOTE = "CLAUDE_CODE_REMOTE_SESSION_ID"


def container(monkeypatch, remote_id):
    """Be the cloud container named `remote_id` (both variables Claude Code
    sets there)."""
    monkeypatch.setenv("CLAUDE_CODE_REMOTE", "true")
    monkeypatch.setenv(REMOTE, remote_id)


def _dev_host() -> str:
    """`host()` as `dev` computed it before 3.179, copied verbatim: the
    desktop's claims must keep matching it (criterion 3)."""
    try:
        ns = str(os.stat("/proc/self/ns/pid").st_ino)
    except OSError:
        ns = ""
    return pc._digest("h-", f"{socket.gethostname()}|{ns}")


def _claim_on_origin(repo, step):
    remote, err = pc.remote_claims(repo)
    assert err is None
    return next(c for c in remote if c["step"] == step)


def test_criterion_1_two_cloud_containers_are_two_machines(repo, owners, monkeypatch):
    container(monkeypatch, "cse_container_a")
    pa, a = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    sha = remote_refs(repo)["3.90.A"]
    pa.kill()                                  # A's process is not running in B
    pa.wait()

    container(monkeypatch, "cse_container_b")
    _, b = owners()
    theirs = _claim_on_origin(repo, "3.90.A")
    assert theirs["host"] != b["host"]
    assert pc.held(theirs, pc.processes(), b)
    board = pc.cmd_board(b=pc.Board(repo, b))
    assert re.search(r"^claimed\s+3\.90\.A", board, re.M), board
    assert not re.search(r"^free\s+3\.90\.A", board, re.M), board
    with pytest.raises(SystemExit, match="claimed"):
        pc.claim("3.90.A", cwd=repo, me=b)
    assert remote_refs(repo)["3.90.A"] == sha   # no takeover attempted


def test_criterion_1_next_does_not_offer_another_containers_claim(repo, owners, monkeypatch):
    container(monkeypatch, "cse_container_a")
    pa, a = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    assert "3.90.A" in remote_refs(repo)       # claimed on origin before B looks
    pa.kill()
    pa.wait()
    container(monkeypatch, "cse_container_b")
    _, b = owners()
    monkeypatch.setenv("PICAR_CLAIM_PID", str(b["pid"]))
    out = pc.cmd_next(cwd=repo)
    assert "3.90.A" not in out, out


def test_criterion_2_one_container_takes_over_its_own_dead_holder(repo, owners, monkeypatch):
    container(monkeypatch, "cse_container_a")
    pa, a = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    pa.kill()
    pa.wait()
    _, b = owners()
    c = pc.claim("3.90.A", cwd=repo, me=b)
    assert remote_refs(repo)["3.90.A"] == c["sha"]


@pytest.mark.parametrize("value", [None, ""])
def test_criterion_3_without_a_remote_id_the_digest_is_dev_s(monkeypatch, value):
    monkeypatch.delenv("PICAR_CLAIM_HOST", raising=False)
    if value is None:
        monkeypatch.delenv(REMOTE, raising=False)
    else:
        monkeypatch.setenv(REMOTE, value)
    assert pc.host() == _dev_host()


@pytest.mark.parametrize("remote", [None, "", "false", "1"])
def test_criterion_3_a_remote_id_outside_a_cloud_container_changes_nothing(monkeypatch, remote):
    """A desktop or Jetson session that inherits the id (Remote Control)
    keeps its machine's digest: only CLAUDE_CODE_REMOTE=true is the cloud."""
    monkeypatch.delenv("PICAR_CLAIM_HOST", raising=False)
    container(monkeypatch, "cse_inherited")
    if remote is None:
        monkeypatch.delenv("CLAUDE_CODE_REMOTE", raising=False)
    else:
        monkeypatch.setenv("CLAUDE_CODE_REMOTE", remote)
    assert pc.host() == _dev_host()


def test_criterion_3_the_cloud_digest_differs_and_is_stable(monkeypatch):
    monkeypatch.delenv("PICAR_CLAIM_HOST", raising=False)
    container(monkeypatch, "cse_one")
    one = pc.host()
    assert one != _dev_host() and pc.host() == one
    container(monkeypatch, "cse_two")
    assert pc.host() != one


def test_criterion_4_the_raw_id_never_reaches_origin(repo, owners, monkeypatch):
    secret = "cse_01SECRETVALUE"
    container(monkeypatch, secret)
    assert re.fullmatch(r"h-[0-9a-f]{12}", pc.host())
    assert secret not in pc.host()
    _, a = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    msg = subprocess.run(["git", "log", "-1", "--format=%B", remote_refs(repo)["3.90.A"]],
                         cwd=repo, capture_output=True, text=True).stdout
    assert secret not in msg and "SECRET" not in msg
    assert f"host: {pc.host()}" in msg


def test_criterion_5_a_claim_under_the_old_shared_digest_stays_its_sessions(repo, monkeypatch):
    container(monkeypatch, "cse_mine")
    legacy = _dev_host()
    fake_claim(repo, "3.90.A", host=legacy, session="s-mine0001", pid=1)
    me = {"pid": 4242424, "start": "x", "session": "s-mine0001", "host": pc.host()}
    mine = _claim_on_origin(repo, "3.90.A")
    assert mine["host"] != me["host"]
    assert pc.same_owner(mine, me)
    assert pc.release("3.90.A", cwd=repo, me=me) == ["3.90.A"]
    assert "3.90.A" not in remote_refs(repo)
