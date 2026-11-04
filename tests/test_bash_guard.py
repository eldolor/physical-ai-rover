# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""tools/bash_guard.py, 3.150: a PreToolUse hook on Bash refuses the
user's gate bypasses (`PICAR_SKIP_GATE=1`, `PICAR_CLAIM_BYPASS=1`), skipped
git hooks (`--no-verify`, `core.hooksPath`) and a `vision-picar-ros:latest`
built anywhere but a clean `origin/dev` checkout. Every row runs through the
command line `.claude/settings.json` itself gives Claude Code, so a hook
that is written but not wired fails here."""

import json
import os
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))


def settings_bash_hook() -> str:
    """The command `.claude/settings.json` runs before every Bash call
    ('' when none is wired)."""
    with open(os.path.join(ROOT, ".claude", "settings.json")) as fh:
        s = json.load(fh)
    for entry in s.get("hooks", {}).get("PreToolUse", []):
        if "Bash" in entry.get("matcher", "").split("|"):
            return entry["hooks"][0]["command"]
    return ""


def run_hook(command: str, cwd: str = ROOT, mode: str = "auto", raw: str = None):
    """The settings' hook on one Bash call: (decision or None, reason)."""
    hook = settings_bash_hook()
    assert hook, "no PreToolUse hook on Bash in .claude/settings.json"
    stdin = raw if raw is not None else json.dumps({
        "session_id": "t", "transcript_path": "/dev/null", "cwd": cwd,
        "permission_mode": mode, "hook_event_name": "PreToolUse",
        "tool_name": "Bash", "tool_input": {"command": command}})
    r = subprocess.run(["bash", "-c", hook], input=stdin, capture_output=True, text=True,
                       env={**os.environ, "CLAUDE_PROJECT_DIR": ROOT}, timeout=30)
    assert r.returncode == 0, r.stderr
    if not r.stdout.strip():
        return None, ""
    out = json.loads(r.stdout)["hookSpecificOutput"]
    return out["permissionDecision"], out["permissionDecisionReason"]


REFUSED = [
    "PICAR_SKIP_GATE=1 git push origin HEAD:dev",
    "cd /tmp && PICAR_SKIP_GATE=1 git push origin HEAD:dev",
    "git fetch origin dev; PICAR_SKIP_GATE=1 git push origin HEAD:dev",
    "env PICAR_SKIP_GATE=1 git push origin HEAD:dev",
    "env -u FOO PICAR_SKIP_GATE=1 git push",
    "export PICAR_SKIP_GATE=1",
    "export FOO=2 PICAR_SKIP_GATE=1; git push origin HEAD:dev",
    "PICAR_CLAIM_BYPASS=1 git commit -m x",
    "true && PICAR_CLAIM_BYPASS=1 python tools/plan_section.py claim 3.1.A",
    "(PICAR_CLAIM_BYPASS=1 git commit -qm x)",
    "sudo PICAR_SKIP_GATE=1 git push",
    "nohup env PICAR_SKIP_GATE=1 git push &",
    "PICAR_SKIP_GATE='1' git push",
    "git push --no-verify origin HEAD:dev",
    "git -C /tmp push origin HEAD:dev --no-verify",
    "git commit --no-verify -m x",
    "git commit -n -m x",
    "git commit -anm x",
    "git -c core.hooksPath=/dev/null push origin HEAD:dev",
    "git -c core.hookspath=/dev/null commit -m x",
    "git config core.hooksPath /dev/null",
    "git config --local core.hooksPath .nohooks",
    "docker build -t vision-picar-ros:latest service/slam",
    "docker build --tag vision-picar-ros:latest service/slam",
    "docker build --tag=vision-picar-ros:latest service/slam",
    "docker build -tvision-picar-ros:latest service/slam",
    "docker build -t vision-picar-ros service/slam",
    "docker buildx build -t vision-picar-ros:latest service/slam",
    "docker build -t mine:x -t vision-picar-ros:latest service/slam",
    "docker tag vision-picar-ros:plan-3.150 vision-picar-ros:latest",
    "docker image tag vision-picar-ros:plan-3.150 vision-picar-ros",
    "echo start\nPICAR_SKIP_GATE=1 git push origin HEAD:dev",
]

PASSED = [
    "git push origin HEAD:dev",
    "git push -u origin plan/3.150-bash-guard",
    "git push --dry-run origin HEAD:dev",
    "git push -n origin HEAD:dev",
    "git commit -qm 'fix: no-verify was a bad idea'",
    "git commit -am 'PICAR_SKIP_GATE=1 is the user'\"'\"'s'",
    "git config --get core.hooksPath",
    "git config core.hooksPath",
    "grep -n PICAR_SKIP_GATE tools/hooks/pre-push",
    "grep -rn 'PICAR_SKIP_GATE=1' docs/",
    "echo PICAR_SKIP_GATE=1",
    "PICAR_SKIP_GATE=0 git push origin HEAD:dev",
    "unset PICAR_SKIP_GATE",
    "export PICAR_ROBOT_PORT=10500",
    "cat > /tmp/x.md <<'EOF'\nRun PICAR_SKIP_GATE=1 git push only on the user's word.\ngit push --no-verify\nEOF",
    "cat <<-END > note\n\tdocker build -t vision-picar-ros:latest .\n\tEND\nls",
    "ls  # PICAR_SKIP_GATE=1 git push",
    "docker build -t vision-picar-ros:plan-3.150 service/slam",
    "docker build -t \"$ROS_IMAGE\" service/slam",
    "docker run --rm vision-picar-ros:latest ros2 --help",
    "docker images vision-picar-ros",
    "python -m pytest tests/test_bash_guard.py -q",
    "eval \"$(python tools/plan_section.py env)\"",
    "ls -la && git status",
    "git log --grep no-verify",
    "pytest -k 'hooksPath'",
]


@pytest.mark.parametrize("command", REFUSED)
def test_a_rule_breaking_command_is_refused(command):
    decision, reason = run_hook(command)
    assert decision == "deny", (command, decision)
    assert "the user's call" in reason


@pytest.mark.parametrize("command", PASSED)
def test_an_ordinary_command_passes(command):
    assert run_hook(command) == (None, ""), command


def test_the_tables_are_big_enough():
    assert len(REFUSED) >= 25 and len(PASSED) >= 25


# --- :latest from a checkout: only a clean origin/dev ----------------------

def _git(cwd, *args):
    subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True,
                   env={**os.environ, "GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@t",
                        "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@t"})


@pytest.fixture
def checkout(tmp_path):
    """A repo whose HEAD is origin/dev's commit, with a service/slam."""
    d = tmp_path / "co"
    (d / "service" / "slam").mkdir(parents=True)
    (d / "service" / "slam" / "Dockerfile").write_text("FROM scratch\n")
    _git(d, "init", "-q")
    _git(d, "add", "-A")
    _git(d, "commit", "-qm", "base")
    _git(d, "update-ref", "refs/remotes/origin/dev", "HEAD")
    return d


LATEST = "docker build -t vision-picar-ros:latest service/slam"


def test_latest_from_a_clean_origin_dev_checkout_passes(checkout):
    assert run_hook(LATEST, cwd=str(checkout)) == (None, "")
    _git(checkout, "checkout", "-q", "--detach")
    assert run_hook(LATEST, cwd=str(checkout)) == (None, "")


def test_latest_after_a_cd_into_a_clean_checkout_passes(checkout, tmp_path):
    assert run_hook(f"cd {checkout} && {LATEST}", cwd=str(tmp_path)) == (None, "")


def test_latest_from_a_branch_is_refused(checkout):
    (checkout / "x").write_text("x\n")
    _git(checkout, "add", "x")
    _git(checkout, "commit", "-qm", "branch work")
    decision, reason = run_hook(LATEST, cwd=str(checkout))
    assert decision == "deny" and "origin/dev" in reason


@pytest.mark.parametrize("change", ["tracked", "untracked"])
def test_latest_with_changes_under_service_slam_is_refused(checkout, change):
    if change == "tracked":
        (checkout / "service" / "slam" / "Dockerfile").write_text("FROM busybox\n")
    else:
        (checkout / "service" / "slam" / "new.py").write_text("x = 1\n")
    assert run_hook(LATEST, cwd=str(checkout))[0] == "deny"


def test_latest_after_a_cd_into_a_branch_is_refused(checkout, tmp_path):
    (checkout / "service" / "slam" / "new.py").write_text("x = 1\n")
    assert run_hook(f"cd {checkout} && {LATEST}", cwd=str(tmp_path))[0] == "deny"


def test_latest_outside_any_repo_is_refused(tmp_path):
    assert run_hook(LATEST, cwd=str(tmp_path))[0] == "deny"


def test_docker_tag_onto_latest_is_refused_even_from_a_clean_checkout(checkout):
    assert run_hook("docker tag vision-picar-ros:plan-3.150 vision-picar-ros:latest",
                    cwd=str(checkout))[0] == "deny"


# --- ask or deny ------------------------------------------------------------

@pytest.mark.parametrize("mode,want", [
    ("default", "ask"), ("plan", "ask"), ("acceptEdits", "ask"),
    ("auto", "deny"), ("bypassPermissions", "deny"), ("dontAsk", "deny"), ("", "deny")])
def test_a_refusal_asks_only_where_a_person_sees_it(mode, want):
    assert run_hook("PICAR_SKIP_GATE=1 git push", mode=mode)[0] == want


# --- failing closed ---------------------------------------------------------

def test_an_unparseable_command_naming_a_guarded_word_is_refused():
    decision, reason = run_hook("echo 'unclosed && PICAR_SKIP_GATE=1 git push")
    assert decision == "deny" and "could not read" in reason


def test_an_unparseable_command_naming_nothing_guarded_passes():
    assert run_hook("echo 'unclosed && ls") == (None, "")


def test_stdin_that_is_not_json_passes_unless_it_names_a_guarded_word():
    assert run_hook("", raw="not json at all") == (None, "")
    assert run_hook("", raw="not json PICAR_CLAIM_BYPASS=1")[0] == "deny"


def test_a_check_that_raises_fails_closed_only_for_guarded_commands(monkeypatch):
    import bash_guard as bg

    def boom(*a, **k):
        raise RuntimeError("injected")
    monkeypatch.setattr(bg, "check_simple", boom)
    payload = lambda c: json.dumps({"cwd": ROOT, "permission_mode": "auto",
                                    "tool_name": "Bash", "tool_input": {"command": c}})
    out = json.loads(bg.hook(payload("git push --no-verify")))["hookSpecificOutput"]
    assert out["permissionDecision"] == "deny" and "could not read" in out["permissionDecisionReason"]
    assert bg.hook(payload("ls -la")) == ""


def test_a_tool_other_than_bash_is_not_judged():
    import bash_guard as bg
    assert bg.hook(json.dumps({"tool_name": "Edit", "tool_input": {"command": "PICAR_SKIP_GATE=1"}})) == ""


def test_a_checkout_without_the_guard_is_not_blocked(tmp_path):
    """The settings line runs the script only where it exists, so a checkout
    from before 3.150 (or a moved project dir) never blocks every command."""
    hook = settings_bash_hook()
    r = subprocess.run(["bash", "-c", hook], input="{}", capture_output=True, text=True,
                       env={**os.environ, "CLAUDE_PROJECT_DIR": str(tmp_path)}, timeout=30)
    assert r.returncode == 0 and r.stdout.strip() == ""
