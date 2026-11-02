# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""tools/plan_claims.py, 3.139: a claim is a `claim/<step>` branch on
origin, pushed create-only before the local claim file is written; the
session owns it; the commit, edit and push hooks refuse work on a section
without one. Every origin here is a local bare repo (`repo` fixture); no
test reaches the real origin."""

import contextlib
import json
import os
import subprocess
import sys
import time

import pytest

from tests.test_plan_claims import TOOLS, _git, owners, repo  # noqa: F401
import plan_claims as pc  # noqa: E402
import plan_section as ps  # noqa: E402

ROOT = os.path.dirname(TOOLS)


def _out(cwd, *args):
    return subprocess.run(["git", *args], cwd=cwd, capture_output=True, text=True).stdout.strip()


def remote_refs(repo):
    """{step: sha} of the claim branches on the bare origin."""
    out = _out(repo, "ls-remote", "origin", "refs/heads/claim/*")
    return {line.split()[1][len("refs/heads/claim/"):]: line.split()[0]
            for line in out.splitlines() if line.strip()}


def local_files(repo):
    d = os.path.join(pc.repo(repo)[1], "plan-claims")
    return sorted(f[:-5] for f in os.listdir(d) if f.endswith(".json")) if os.path.isdir(d) else []


@contextlib.contextmanager
def as_host(name):
    old = os.environ.get("PICAR_CLAIM_HOST")
    os.environ["PICAR_CLAIM_HOST"] = name
    try:
        yield
    finally:
        if old is None:
            os.environ.pop("PICAR_CLAIM_HOST", None)
        else:
            os.environ["PICAR_CLAIM_HOST"] = old


def other_machine(repo, tmp_path, name="far"):
    """A second clone of the same bare origin: its own .git, so its own
    local claims and lock -- another machine as far as claims go."""
    bare = _out(repo, "remote", "get-url", "origin")
    path = str(tmp_path / f"clone-{name}")
    _git(tmp_path, "clone", "-q", "-b", "dev", bare, path)
    return path


def fake_claim(repo, step, host, pid=1, session="far-session", days_old=0.0, worktree="/far/wt"):
    """A claim pushed by some other holder, dated `days_old` days ago."""
    c = {"step": step, "section": pc.section_of(step), "title": "theirs", "session": session,
         "worktree": worktree, "branch": "plan/x", "host": host, "pid": pid,
         "start": "Thu Jan  1 00:00:00 1970", "at": "2026-01-01T00:00:00Z"}
    when = f"@{int(time.time() - days_old * 86400)} +0000"
    env = {**os.environ, "GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@t", "GIT_COMMITTER_NAME": "t",
           "GIT_COMMITTER_EMAIL": "t@t", "GIT_COMMITTER_DATE": when, "GIT_AUTHOR_DATE": when}
    tree = subprocess.run(["git", "hash-object", "-t", "tree", "-w", "--stdin"], cwd=repo, input="",
                          capture_output=True, text=True, check=True).stdout.strip()
    sha = subprocess.run(["git", "commit-tree", tree, "-m", pc.claim_message(c)], cwd=repo, env=env,
                         capture_output=True, text=True, check=True).stdout.strip()
    _git(repo, "push", "-q", "--force", "origin", f"{sha}:refs/heads/claim/{step}")
    return sha


def session_of(proc_owner, session, host=None):
    return {**proc_owner, "session": session, "host": host or pc.host()}


# ---------- criterion 1: one winner, across machines ----------

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


def test_criterion_1_twenty_racers_on_two_machines_one_winner(repo, owners, tmp_path):
    """Two clones (two .git, two local locks): only origin can pick one.
    PICAR_RACE_TRIALS=20 repeats the measurement."""
    far = other_machine(repo, tmp_path)
    trials = int(os.environ.get("PICAR_RACE_TRIALS", "1"))
    holders = [owners()[0] for _ in range(20)]
    for trial in range(trials):
        go = str(tmp_path / f"go-{trial}")
        ps_ = []
        for i, h in enumerate(holders):
            where, host = (repo, "near") if i % 2 else (far, "far")
            ps_.append(subprocess.Popen(
                [sys.executable, "-c", RACER.format(tools=TOOLS, go=go, repo=where)],
                stdout=subprocess.PIPE, text=True,
                env={**os.environ, "PICAR_CLAIM_PID": str(h.pid), "PICAR_CLAIM_HOST": host}))
        time.sleep(0.5)
        open(go, "w").close()
        outs = [p.communicate(timeout=120)[0] for p in ps_]
        assert sum(o.startswith("WIN") for o in outs) == 1, outs
        for o in outs:
            if not o.startswith("WIN"):
                assert o.startswith("REFUSED") and "claimed" in o, o
        files = local_files(repo) + local_files(far)
        assert files == ["3.90.A"], files               # the winner's only
        winner = holders[[o.startswith("WIN") for o in outs].index(True)]
        assert pc.release("3.90.A", cwd=repo if os.path.exists(os.path.join(
            pc.repo(repo)[1], "plan-claims", "3.90.A.json")) else far,
            me=pc.owner_of(winner.pid), force=True) == ["3.90.A"]
        assert remote_refs(repo) == {}


# ---------- criterion 2: published first ----------

def test_criterion_2_offline_claim_fails_and_writes_nothing(repo, owners, tmp_path):
    _, a = owners()
    _git(repo, "remote", "set-url", "origin", str(tmp_path / "nowhere.git"))
    with pytest.raises(SystemExit, match="origin"):
        pc.claim("3.90.A", cwd=repo, me=a)
    assert local_files(repo) == []


def test_criterion_2_a_rejected_push_writes_nothing_and_names_the_holder(repo, owners, monkeypatch):
    _, a = owners()
    real = pc.remote_claims
    calls = []

    def stale_view(top, timeout=None):                        # the board saw origin before the other push
        calls.append(1)
        return ([], None) if len(calls) == 1 else real(top)
    monkeypatch.setattr(pc, "remote_claims", stale_view)
    fake_claim(repo, "3.90.A", host="far", worktree="/far/their-wt")
    with pytest.raises(SystemExit, match=r"claimed on origin.*their-wt"):
        pc.claim("3.90.A", cwd=repo, me=a)
    assert local_files(repo) == []


def test_a_claim_is_a_branch_on_origin_named_for_its_holder(repo, owners):
    _, a = owners()
    a = session_of(a, "sess-a")
    c = pc.claim("3.90.A", cwd=repo, me=a)
    refs = remote_refs(repo)
    assert refs == {"3.90.A": c["sha"]}
    msg = _out(repo, "log", "-1", "--format=%B", c["sha"])
    held = pc.parse_claim_message(msg)
    assert held["session"] == "sess-a" and held["pid"] == a["pid"] and held["host"] == pc.host()
    assert held["step"] == "3.90.A" and held["worktree"] == os.path.basename(repo)   # public: no path
    assert json.load(open(os.path.join(pc.repo(repo)[1], "plan-claims", "3.90.A.json")))["sha"] == c["sha"]


# ---------- criterion 3: seen from another machine ----------

def test_criterion_3_another_machine_sees_and_refuses_the_claim(repo, owners, tmp_path):
    _, a = owners()
    _, b = owners()
    pc.claim("3.90.A", cwd=repo, me=session_of(a, "sess-a"))
    far = other_machine(repo, tmp_path)
    with as_host("far"):
        board = pc.cmd_board(cwd=far)
        line = next(l for l in board.splitlines() if "3.90.A" in l)
        assert line.startswith("claimed") and "(stale?)" not in line
        assert os.path.basename(repo) in board
        with pytest.raises(SystemExit, match="claimed"):
            pc.claim("3.90.A", cwd=far, me=session_of(b, "sess-b", "far"))
        with pytest.raises(SystemExit, match="claimed"):    # one session per section
            pc.claim("3.90.E", cwd=far, me=session_of(b, "sess-b", "far"))


# ---------- criterion 4: release ----------

def test_criterion_4_release_deletes_only_its_own_sha(repo, owners):
    _, a = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    theirs = fake_claim(repo, "3.90.A", host="far")       # replaced (the user freed it; someone took it)
    pc.release(cwd=repo, me=a)
    assert remote_refs(repo) == {"3.90.A": theirs}
    assert local_files(repo) == []


def test_criterion_4_release_gives_back_its_own_branch(repo, owners):
    _, a = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    assert pc.release(cwd=repo, me=a) == ["3.90.A"]
    assert remote_refs(repo) == {} and local_files(repo) == []


def test_criterion_4_two_sessions_in_one_process_release_only_their_own(repo, owners):
    """2026-11-01: a no-step release in one Desktop session dropped other
    sessions' claims (one owner per `claude` process)."""
    _, proc = owners()
    a, b = session_of(proc, "sess-a"), session_of(proc, "sess-b")
    pc.claim("3.90.A", cwd=repo, me=a)
    pc.claim("3.91.A", cwd=repo, me=b)
    assert pc.release(cwd=repo, me=b) == ["3.91.A"]
    assert local_files(repo) == ["3.90.A"] and list(remote_refs(repo)) == ["3.90.A"]


def test_criterion_4_force_release_frees_anyones(repo, owners):
    _, a = owners()
    _, b = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    with pytest.raises(SystemExit, match="refused"):
        pc.release("3.90.A", cwd=repo, me=b)
    assert pc.release("3.90.A", cwd=repo, me=b, force=True) == ["3.90.A"]
    assert remote_refs(repo) == {} and local_files(repo) == []


# ---------- criterion 5: stale claims ----------

def test_criterion_5_same_host_dead_holder_is_taken_over(repo, owners):
    pa, a = owners()
    _, b = owners()
    pc.claim("3.90.A", cwd=repo, me=a)
    pa.kill()
    pa.wait()
    c = pc.claim("3.90.A", cwd=repo, me=b)
    sha = remote_refs(repo)["3.90.A"]
    assert sha == c["sha"]
    assert pc.parse_claim_message(_out(repo, "log", "-1", "--format=%B", sha))["pid"] == b["pid"]


def test_criterion_5_another_hosts_claim_is_never_freed(repo, owners):
    _, a = owners()
    fake_claim(repo, "3.90.A", host="far", pid=1, days_old=3)     # pid 1 runs here: irrelevant
    with pytest.raises(SystemExit, match="claimed"):
        pc.claim("3.90.A", cwd=repo, me=a)
    board = pc.cmd_board(cwd=repo)
    assert "claimed (stale?) 3.90.A" in board and "only the user" in board
    fake_claim(repo, "3.91.A", host="far", days_old=0.1)
    assert "claimed   3.91.A" in pc.cmd_board(cwd=repo)


def test_criterion_5_a_live_plan_branch_keeps_an_old_claim_fresh(repo, owners):
    fake_claim(repo, "3.90.A", host="far", days_old=3)
    _git(repo, "checkout", "-q", "-b", "plan/3.90-test")
    open(os.path.join(repo, "x.txt"), "w").write("x\n")
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", "work")
    _git(repo, "push", "-q", "origin", "plan/3.90-test")
    _git(repo, "checkout", "-q", "dev")
    assert "(stale?)" not in pc.cmd_board(cwd=repo)


# ---------- criterion 6: the hooks ----------

def _install_pre_commit(repo):
    hooks = os.path.join(repo, ".hooks")
    os.makedirs(hooks, exist_ok=True)
    script = open(os.path.join(TOOLS, "hooks", "pre-commit")).read()
    open(os.path.join(hooks, "pre-commit"), "w").write(
        script.replace('"$root/tools/plan_claims.py"', repr(os.path.join(TOOLS, "plan_claims.py"))))
    os.chmod(os.path.join(hooks, "pre-commit"), 0o755)
    _git(repo, "config", "core.hooksPath", hooks)


def _commit(repo, owner_pid, msg, extra=None):
    env = {**os.environ, "PICAR_CLAIM_PID": str(owner_pid), "GIT_AUTHOR_NAME": "t",
           "GIT_AUTHOR_EMAIL": "t@t", "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@t",
           **(extra or {})}
    return subprocess.run(["git", "commit", "-q", "-m", msg], cwd=repo, capture_output=True,
                          text=True, env=env)


def test_criterion_6_commit_on_a_plan_branch_needs_a_claim_in_its_section(repo, owners):
    _, a = owners()
    _git(repo, "checkout", "-q", "-b", "plan/3.90-test")
    _install_pre_commit(repo)
    open(os.path.join(repo, "x.txt"), "w").write("x\n")
    _git(repo, "add", "-A")
    r = _commit(repo, a["pid"], "unclaimed")
    assert r.returncode != 0 and "plan_section.py claim" in r.stderr, r.stderr
    assert _commit(repo, a["pid"], "bypass", {"PICAR_CLAIM_BYPASS": "1"}).returncode == 0
    open(os.path.join(repo, "x.txt"), "a").write("y\n")
    _git(repo, "add", "-A")
    pc.claim("3.91.A", cwd=repo, me=a)                     # another section: not enough
    assert _commit(repo, a["pid"], "wrong section").returncode != 0
    pc.release(cwd=repo, me=a)
    pc.claim("3.90.A", cwd=repo, me=a)
    r = _commit(repo, a["pid"], "claimed")
    assert r.returncode == 0, r.stderr


def test_criterion_6_a_merge_from_dev_needs_no_claim(repo, owners):
    _, a = owners()
    _git(repo, "checkout", "-q", "-b", "plan/3.90-test")
    _git(repo, "checkout", "-q", "dev")
    open(os.path.join(repo, "dev.txt"), "w").write("dev\n")
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", "dev moves")
    _git(repo, "checkout", "-q", "plan/3.90-test")
    _install_pre_commit(repo)
    _git(repo, "merge", "-q", "--no-ff", "--no-commit", "dev")
    r = _commit(repo, a["pid"], "merge dev")
    assert r.returncode == 0, r.stderr


def _edit_hook(repo, path, owner_pid, mode="auto", extra=None):
    env = {**os.environ, "PICAR_CLAIM_PID": str(owner_pid), **(extra or {})}
    r = subprocess.run([sys.executable, os.path.join(TOOLS, "plan_claims.py"), "hook-edit"],
                       input=json.dumps({"tool_name": "Edit", "cwd": repo, "permission_mode": mode,
                                         "tool_input": {"file_path": os.path.join(repo, path)}}),
                       capture_output=True, text=True, env=env)
    assert r.returncode == 0
    return json.loads(r.stdout)["hookSpecificOutput"] if r.stdout.strip() else None


def test_criterion_6_the_edit_hook_needs_a_claim_for_tracked_paths(repo, owners):
    _, a = owners()
    for path in (f"{pc.DIR}/3.90-test.md", "brain/a.py", "sim/maps/h.py"):
        for mode in ("default", "auto"):
            d = _edit_hook(repo, path, a["pid"], mode)
            assert d and d["permissionDecision"] == "deny", (path, mode, d)
            assert "plan_section.py claim" in d["permissionDecisionReason"]
    assert _edit_hook(repo, "brain/d.py", a["pid"]) is None       # a done step's Touches
    assert _edit_hook(repo, "elsewhere/x.py", a["pid"]) is None   # in no step
    assert _edit_hook(repo, "brain/a.py", a["pid"], extra={"PICAR_CLAIM_BYPASS": "1"}) is None
    pc.claim("3.90.A", cwd=repo, me=a)
    for path in (f"{pc.DIR}/3.90-test.md", "brain/a.py", "brain/b.py"):
        assert _edit_hook(repo, path, a["pid"]) is None, path
    assert _edit_hook(repo, "control/x.py", a["pid"])["permissionDecision"] == "deny"   # 3.91's


def test_criterion_6_the_push_gate_needs_the_sections_claim(repo, owners, monkeypatch):
    _, a = owners()
    base = _out(repo, "rev-parse", "HEAD")
    p = os.path.join(repo, pc.DIR, "3.90-test.md")
    open(p, "a").write("\nmore\n")
    os.makedirs(os.path.join(repo, "brain"))
    open(os.path.join(repo, "brain", "a.py"), "w").write("x = 1\n")
    open(os.path.join(repo, "free.txt"), "w").write("x\n")
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", "work")
    sha = _out(repo, "rev-parse", "HEAD")
    out = pc.push_refusals(base, sha, cwd=repo, me=a)
    assert any("3.90-test.md" in x for x in out) and any("brain/a.py" in x for x in out)
    assert not any("free.txt" in x for x in out)
    monkeypatch.setenv("PICAR_CLAIM_BYPASS", "1")
    assert pc.push_refusals(base, sha, cwd=repo, me=a) == []
    monkeypatch.delenv("PICAR_CLAIM_BYPASS")
    # The gate refuses before any tier runs.
    monkeypatch.chdir(repo)
    monkeypatch.setenv("PICAR_CLAIM_PID", str(a["pid"]))
    monkeypatch.setattr(ps, "gate_dir", lambda: str(os.path.join(repo, ".git")))
    ran = []
    monkeypatch.setattr(ps, "run_tests", lambda root, full: ran.append(full) or True)
    with pytest.raises(SystemExit, match="plan_section.py claim"):
        ps.cmd_gate(base, sha)
    assert ran == []
    pc.claim("3.90.A", cwd=repo, me=a)
    assert pc.push_refusals(base, sha, cwd=repo, me=a) == []


def test_criterion_6_the_pre_push_hook_passes_claim_refs_and_still_gates_dev(tmp_path):
    r = tmp_path / "hookrepo"
    (r / "tools").mkdir(parents=True)
    marker = tmp_path / "gate-ran"
    (r / "tools" / "plan_section.py").write_text(
        f"import sys\nopen({str(marker)!r}, 'a').write(' '.join(sys.argv[1:]) + '\\n')\n")
    _git(r, "init", "-q", "-b", "dev")
    _git(r, "add", "-A")
    _git(r, "commit", "-q", "-m", "x")
    sha = _out(r, "rev-parse", "HEAD")
    zero = "0" * 40
    hook = os.path.join(TOOLS, "hooks", "pre-push")

    def push(line):
        env = {k: v for k, v in os.environ.items() if k != "PICAR_SKIP_GATE"}
        return subprocess.run(["sh", hook, "origin", "url"], cwd=r, input=line + "\n",
                              capture_output=True, text=True, env=env).returncode

    assert push(f"{sha} {sha} refs/heads/claim/3.90.A {zero}") == 0       # create
    assert push(f"(delete) {zero} refs/heads/claim/3.90.A {sha}") == 0    # release
    assert not marker.exists()
    assert push(f"refs/heads/dev {sha} refs/heads/dev {sha}") == 0
    assert marker.read_text().startswith("gate")


# ---------- `new` reserves its number on origin ----------

def test_numbers_held_by_a_claim_on_origin_are_used(repo):
    fake_claim(repo, "3.150", host="far")
    fake_claim(repo, "3.151.A", host="far")
    nums = ps.used_numbers(repo)
    assert {150, 151} <= nums


def test_board_shows_a_whole_section_claim(repo, owners):
    fake_claim(repo, "3.150", host="far", worktree="/far/plan-3.150")
    board = pc.cmd_board(cwd=repo)
    assert "3.150" in board and "plan-3.150" in board


def test_new_claims_its_section_on_origin_and_skips_a_number_held_there(tmp_path, monkeypatch):
    """`new` takes `claim/3.N` at once; a number another machine claimed on
    origin after this one listed the numbers in use is skipped."""
    from tests.test_review_376i import _with_origin
    repo, g = _with_origin(tmp_path, monkeypatch)
    monkeypatch.setattr(ps, "used_numbers", lambda root: set())     # the listing raced
    fake_claim(str(repo), "3.1", host="far")
    ps.cmd_new("foo", "Foo")
    refs = remote_refs(str(repo))
    assert set(refs) == {"3.1", "3.2"}
    assert pc.parse_claim_message(_out(str(repo), "log", "-1", "--format=%B", refs["3.2"]))["pid"] \
        == os.getpid()
    branches = g("branch", "--format=%(refname:short)").splitlines()
    assert "plan/3.2-foo" in branches and "plan/3.1-foo" not in branches


def test_shared_status_and_a_pushers_own_touches_are_not_fenced(repo, owners):
    """CLAUDE.md is edited by whoever merges (rule 3), whatever step lists
    it; a file another open step lists is the pusher's to change when a
    step of the section it holds declares it too."""
    _, a = owners()
    def _rewrite(name, old, new):
        path = os.path.join(repo, pc.DIR, name)
        text = open(path).read()
        assert old in text
        open(path, "w").write(text.replace(old, new))
    _rewrite("3.91-other.md", "`control/x.py`", "`control/x.py`, `CLAUDE.md`")
    _git(repo, "commit", "-qam", "3.91.A lists CLAUDE.md")
    _git(repo, "push", "-q", "origin", "dev")
    _git(repo, "fetch", "-q", "origin")
    assert _edit_hook(repo, "CLAUDE.md", a["pid"]) is None
    base = _out(repo, "rev-parse", "HEAD")
    pc.claim("3.90.A", cwd=repo, me=a)
    _rewrite("3.90-test.md", "`brain/b.py`", "`brain/b.py`, `control/x.py`")
    os.makedirs(os.path.join(repo, "control"))
    open(os.path.join(repo, "control", "x.py"), "w").write("x\n")
    open(os.path.join(repo, "CLAUDE.md"), "w").write("status\n")
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", "work")
    assert pc.push_refusals(base, _out(repo, "rev-parse", "HEAD"), cwd=repo, me=a) == []


# ---------- review fixes (code-review + Thermos, 2026-11-01) ----------

def test_review_a_sibling_session_in_one_process_cannot_take_a_claim(repo, owners):
    """Thermos 1: a claim by another session in this same, live process read
    as an earlier conversation's and was taken over or released."""
    _, proc = owners()
    a, b = session_of(proc, "sess-a"), session_of(proc, "sess-b")
    pc.claim("3.90.A", cwd=repo, me=a)
    with pytest.raises(SystemExit, match="claimed"):
        pc.claim("3.90.A", cwd=repo, me=b)
    with pytest.raises(SystemExit, match="refused"):
        pc.release("3.90.A", cwd=repo, me=b)
    assert list(remote_refs(repo)) == ["3.90.A"]


def test_review_a_rival_in_the_section_landing_meanwhile_withdraws_the_claim(repo, owners, monkeypatch):
    """code-review 1: claim/3.90 and claim/3.90.A are two refs; both pushes
    won from two machines with stale views."""
    _, a = owners()
    real = pc.remote_claims
    calls = []

    def stale_view(top, timeout=None):
        calls.append(1)
        return ([], None) if len(calls) == 1 else real(top)
    monkeypatch.setattr(pc, "remote_claims", stale_view)
    fake_claim(repo, "3.90", host="far")                    # the whole section, another machine
    with pytest.raises(SystemExit, match="meanwhile"):
        pc.claim("3.90.A", cwd=repo, me=a)
    assert list(remote_refs(repo)) == ["3.90"] and local_files(repo) == []


def test_review_a_session_survives_its_machine_renaming_itself(repo, owners):
    """code-review 6: macOS changes its host name with the network."""
    _, proc = owners()
    pc.claim("3.90.A", cwd=repo, me=session_of(proc, "sess-a", "old-name"))
    me = session_of(proc, "sess-a", "new-name")
    with as_host("new-name"):
        assert pc.claim("3.90.A", cwd=repo, me=me)["step"] == "3.90.A"     # still its own
        assert pc.release(cwd=repo, me=me) == ["3.90.A"]


def test_review_waiting_and_blocked_steps_fence_nothing(repo, owners):
    """Thermos 2: 3.90.B (needs A) and 3.90.C (waits on the user) cannot be
    claimed, so their Touches must not be fenced."""
    _, a = owners()
    assert _edit_hook(repo, "brain/b.py", a["pid"]) is None
    assert _edit_hook(repo, "brain/c.py", a["pid"]) is None


def test_review_a_claim_freed_on_origin_no_longer_passes_the_push_gate(repo, owners):
    """Both reviewers: the hooks trusted the local file after the user freed
    the claim with --force from another machine."""
    _, a = owners()
    base = _out(repo, "rev-parse", "HEAD")
    pc.claim("3.90.A", cwd=repo, me=a)
    open(os.path.join(repo, pc.DIR, "3.90-test.md"), "a").write("\nmore\n")
    _git(repo, "commit", "-qam", "work")
    sha = _out(repo, "rev-parse", "HEAD")
    assert pc.push_refusals(base, sha, cwd=repo, me=a) == []
    _git(repo, "push", "-q", "origin", ":refs/heads/claim/3.90.A")   # freed elsewhere
    assert pc.push_refusals(base, sha, cwd=repo, me=a)


def test_review_published_holder_carries_no_path_or_raw_ids(repo, owners, monkeypatch):
    """Origin is a public repository: the worktree's folder only, host and
    session as digests."""
    monkeypatch.setenv("CLAUDE_CODE_SESSION_ID", "66ecaeb8-raw-session")
    _, a = owners()
    me = {**a, "session": pc.session_id(), "host": pc.host()}
    c = pc.claim("3.90.A", cwd=repo, me=me)
    msg = _out(repo, "log", "-1", "--format=%B", c["sha"])
    assert "66ecaeb8-raw-session" not in msg and repo not in msg
    assert "session: s-" in msg


def test_review2_the_push_gate_fails_closed_and_names_an_unpublished_claim(repo, owners):
    """Round 2: a base origin moved to (absent here) passed everything; a
    claim made before 3.139 was dropped without a word."""
    _, a = owners()
    open(os.path.join(repo, pc.DIR, "3.90-test.md"), "a").write("\nmore\n")
    _git(repo, "commit", "-qam", "work")
    sha = _out(repo, "rev-parse", "HEAD")
    out = pc.push_refusals("1" * 40, sha, cwd=repo, me=a)
    assert out and "git fetch origin" in out[0]
    path = os.path.join(pc.repo(repo)[1], "plan-claims", "3.90.A.json")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    json.dump({"step": "3.90.A", "section": "3.90", "pid": a["pid"], "start": a["start"],
               "touches": []}, open(path, "w"))                          # a pre-3.139 claim
    out = pc.push_refusals(_out(repo, "rev-parse", "HEAD~1"), sha, cwd=repo, me=a)
    assert any("not on origin" in x and "claim 3.90.A" in x for x in out)
    c = pc.claim("3.90.A", cwd=repo, me=a)                               # re-claim publishes it
    assert remote_refs(repo) == {"3.90.A": c["sha"]}
    assert pc.push_refusals(_out(repo, "rev-parse", "HEAD~1"), sha, cwd=repo, me=a) == []


def test_review2_a_failed_recheck_keeps_the_claim_it_won(repo, owners, monkeypatch):
    _, a = owners()
    real = pc.remote_claims
    calls = []

    def flaky(top, timeout=None):
        calls.append(1)
        return real(top) if len(calls) == 1 else ([], "no answer from origin in 20 s")
    monkeypatch.setattr(pc, "remote_claims", flaky)
    c = pc.claim("3.90.A", cwd=repo, me=a)
    assert remote_refs(repo) == {"3.90.A": c["sha"]} and c["unchecked"]
