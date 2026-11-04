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
    # Review round 1 (/code-review and the Thermos pass): each slipped through.
    "# note\nPICAR_SKIP_GATE=1 git push",
    "ls # a comment\nPICAR_SKIP_GATE=1 git push",
    "cat <<< \"x\"\nPICAR_SKIP_GATE=1 git push",
    "git commit -m 'a <<EOF'\nPICAR_SKIP_GATE=1 git push",
    "python -c 'print(1 << n)'\nPICAR_SKIP_GATE=1 git push",
    "git push origin HEAD:dev \\\n  --no-verify",
    "timeout 60 git push --no-verify origin HEAD:dev",
    "nice -n 5 git push --no-verify origin HEAD:dev",
    "sudo -E -H PICAR_SKIP_GATE=1 git push",
    "env -u A -u B PICAR_SKIP_GATE=1 git push",
    "if true; then git push --no-verify origin HEAD:dev; fi",
    "bash -c 'PICAR_SKIP_GATE=1 git push'",
    "sh -lc \"git push --no-verify origin HEAD:dev\"",
    "eval 'PICAR_SKIP_GATE=1 git push'",
    "declare -x PICAR_SKIP_GATE=1",
    "git push --no-veri origin HEAD:dev",
    "git commit --no-veri -m x",    # git 2.53: `--no-ver` is ambiguous (--no-verbose)
    "git -ccore.hooksPath=/dev/null push origin HEAD:dev",
    "GIT_CONFIG_PARAMETERS=\"'core.hooksPath=/dev/null'\" git push",
    "GIT_CONFIG_COUNT=1 GIT_CONFIG_KEY_0=core.hooksPath GIT_CONFIG_VALUE_0=/dev/null git push",
    "git --config-env=core.hooksPath=X push origin HEAD:dev",
    "git config --unset core.hooksPath",
    "git config unset core.hooksPath",
    "git config set core.hooksPath /dev/null",
    "docker --context x build -t vision-picar-ros:latest service/slam",
    "docker -H unix:///x build -t vision-picar-ros:latest .",
    # Review round 2: each slipped through round 1's hook.
    'out="$(PICAR_SKIP_GATE=1 git push 2>&1)"',
    'echo "$(git push --no-verify origin HEAD:dev)"',
    "echo `git push --no-verify origin HEAD:dev`",
    "diff <(PICAR_SKIP_GATE=1 git push) x",
    "cat <<EOF\n$(git push --no-verify origin HEAD:dev)\nEOF",
    "bash <<'EOF'\ngit push --no-verify origin HEAD:dev\nEOF",
    "ssh jetson bash <<EOF\nPICAR_SKIP_GATE=1 git push\nEOF",
    'cat <<"EOF"x\nEOFx\nPICAR_SKIP_GATE=1 git push\nEOF',
    "git push origin dev 2>&1 --no-verify",
    "git push origin HEAD:dev &> log --no-verify",
    "python tools/desktop_lock.py exclusive --why b -- git push --no-verify origin HEAD:dev",
    "flock /tmp/l git push --no-verify origin HEAD:dev",
    "stdbuf -oL git push --no-verify origin HEAD:dev",
    "env -S 'PICAR_SKIP_GATE=1 git push'",
    "git config --remove-section core",
    "git config rename-section core x",
    "docker commit c vision-picar-ros:latest",
    # Round 3 (/code-review): each slipped through round 2's hook.
    "cat <<EOF | bash\nPICAR_SKIP_GATE=1 git push\nEOF",
    "bash <<< 'PICAR_SKIP_GATE=1 git push'",
    "ssh jetson 'PICAR_SKIP_GATE=1 git push'",
    "ssh -p 22 jetson 'cd ~/vp && git push --no-verify'",
    "git push --no-verify --repo origin dev",
    "docker build -qt vision-picar-ros:latest .",
    # Round 3 (the Thermos pass): each slipped through round 2's hook.
    "echo $(case x in a) PICAR_SKIP_GATE=1 ls;; esac)",
    "echo $(case x in a) git commit -n;; esac)",
    'echo > "$(PICAR_SKIP_GATE=1 ls)"',
    'cat <<< "$(PICAR_SKIP_GATE=1 ls)"',
    "echo > `PICAR_SKIP_GATE=1 ls`",
    "git push --no-verify origin 'refs/heads/*:refs/heads/*'",
    "git push --no-verify origin 'refs/heads/d*'",
    "git push --no-verify origin $BR",
    "for b in plan/x dev; do git push --no-verify origin $b:$b; done",
    "B=dev; git push --no-verify origin $B",
    # Round 4: still refused after the fixes above.
    "x=$(echo case); git push --no-verify origin HEAD:dev",
    "cd; git push --no-verify",
    "cat <<EOF | sudo bash -s\nPICAR_SKIP_GATE=1 git push\nEOF",
    "i=$(( i << 1 ))\ngit commit -n -m x\n1",
    # Round 3 (the transcript corpus): --no-verify only matters on a push
    # that can reach dev, the one ref the pre-push gate checks.
    "git push --no-verify origin --delete dev",
    "git push --no-verify --all origin",
    "git push --no-verify origin +HEAD:refs/heads/dev",
    "git push --no-verify origin feature:dev",
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
    # Review round 1: refused wrongly, or near a fixed hole.
    "git config core.hooksPath tools/hooks",
    "git commit -uno -m x",
    "git commit -m '-n foo'",
    "git commit -m -nope",
    "cat <<< 'hello'",
    "python -c 'print(1 << 3)'\nls",
    "# a comment only",
    "echo 'a # not a comment' && ls",
    "timeout 60 git push origin HEAD:dev",
    # Review round 2: refused wrongly by round 1's hook, or near a fixed hole.
    "git commit -m '--no-verify was dropped from the docs'",
    "git config core.hooksPath /home/x/vision-picar/tools/hooks",
    "git config core.hooksPath ./tools/hooks",
    "git -c core.hooksPath=tools/hooks status",
    "git commit -m \"$(cat <<'EOF'\nsay \" once and PICAR_SKIP_GATE here\nEOF\n)\"",
    "git commit -m \"$(cat <<'EOF'\nfix: PICAR_SKIP_GATE=1 is the user's\nEOF\n)\"",
    "cat <<'EOF' | python3 -\nprint('git push --no-verify')\nEOF",
    "cd service/slam 2>/dev/null && ls",
    "echo $((1 << 3)) && ls",
    "i=$(( i << 1 ))\nls\n1",
    "case $1 in a) echo a;; b) echo b;; esac",
    "for b in plan/3.96-x plan/3.50-y; do git push -q --no-verify origin $b:$b; done",
    "B=plan/3.150-bash-guard; git push --no-verify origin ${B}",
    # Round 4 (regressions of round 3, /code-review): each refused wrongly.
    "x=$(echo case); echo ok",
    'n=$(grep -c case f.txt); docker images "vision-picar-ros"',
    "ssh board 'echo it'\"'\"'s; docker ps | grep vision-picar-ros'",
    'X=$X; git push --no-verify origin "feat-$X"',
    "y=$(echo $((1+(2*3)))); ls",
    "((cd a && make) || true)\ngit status",
    "git push --no-verify origin HEAD:$USER-wip",
    "grep bash <<< 'it'\\''s vision-picar-ros'",
    "git commit -F - <<EOF || bash fallback.sh\nfix: git push --no-verify origin dev was wrong\nEOF",
    'x=$(case "$m" in auto) echo deny;; *) echo ask;; esac); echo $x',
    # Round 3: real commands from past sessions' transcripts; the pre-push
    # gate ignores every ref but dev, and tools/hooks has no stash hook.
    "git push -q --no-verify origin --delete plan/3.96-explore-goal-learning-revisited",
    "git push -q --no-verify picar-jetson:vision-picar HEAD:refs/heads/plan/3.80-e",
    "git push --no-verify origin plan/3.150-bash-guard",
    "git -c core.hooksPath=/dev/null stash push -q -m x -- service/",
    "git -c core.hooksPath=/dev/null worktree add -q -b scratch/x /tmp/x HEAD",
    # Round 3: refused wrongly, or the board's own rebuild (no checkout there).
    "git commit -am '-n is a flag'",
    "ssh picar-jetson 'cd ~/build-ros-dev && docker build -t vision-picar-ros:latest .'",
    "ssh picar-jetson \"docker images vision-picar-ros --format '{{.Tag}}'\"",
    "ssh picar-jetson 'cd ~/vp && git push --no-verify origin plan/3.80-e'",
    "cat <<'EOF' | bash -c 'cat > /tmp/x'\nPICAR_SKIP_GATE=1 git push\nEOF",
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


def test_git_dash_c_reads_that_repos_branch(checkout, tmp_path):
    """Round 3: `git -C <dir> push` pushes <dir>'s branch, not the cwd's."""
    _git(checkout, "checkout", "-q", "-b", "dev")
    assert run_hook(f"git -C {checkout} push --no-verify", cwd=str(tmp_path))[0] == "deny"


def test_nested_substitutions_take_linear_time():
    """Round 3: each level's body was scanned again by every level above it
    (5 s at 18 levels); a hook past its 10 s timeout fails open."""
    import time
    import bash_guard as bg
    def timed(cmd):
        t = time.perf_counter()
        out = bg.hook(json.dumps({"cwd": ROOT, "permission_mode": "auto", "tool_name": "Bash",
                                  "tool_input": {"command": cmd}}))
        return time.perf_counter() - t, out
    took, out = timed("echo " + "$(echo " * 40 + "x" + ")" * 40)      # passes: every level read
    assert took < 1.0 and out == ""
    took, out = timed("echo " + "$(echo " * 39 + "$(PICAR_SKIP_GATE=1 git push)" + ")" * 39)
    assert took < 1.0 and '"deny"' in out


def test_ask_modes_match_the_edit_hook():
    import bash_guard as bg
    import plan_claims as pc
    assert bg.ASK_MODES == pc.ASK_MODES


@pytest.mark.parametrize("command", ["git push --no-verify", "git push --no-verify origin",
                                     "git push --no-verify origin HEAD"])
def test_a_bare_no_verify_push_is_refused_only_from_dev(checkout, command):
    """With no destination, a push goes to the current branch: dev's is gated."""
    assert run_hook(command, cwd=str(checkout)) == (None, "")
    _git(checkout, "checkout", "-q", "-b", "dev")
    assert run_hook(command, cwd=str(checkout))[0] == "deny"


def test_latest_from_a_clean_origin_dev_checkout_passes(checkout):
    assert run_hook(LATEST, cwd=str(checkout)) == (None, "")
    _git(checkout, "checkout", "-q", "--detach")
    assert run_hook(LATEST, cwd=str(checkout)) == (None, "")


@pytest.mark.parametrize("tail", ["2>&1 | tail -5", "> build.log 2>&1", "--no-cache"])
def test_latest_from_a_clean_checkout_with_a_redirect_or_flag_after_passes(checkout, tail):
    """Review round 2: a redirection or a flag after the context read as it."""
    cmd = f"docker build -t vision-picar-ros:latest service/slam {tail}"
    assert run_hook(cmd, cwd=str(checkout)) == (None, "")


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


def test_latest_with_a_branch_context_from_a_clean_cwd_is_refused(checkout, tmp_path):
    """The build context, not the shell's cwd, is what goes into the image."""
    other = tmp_path / "other"
    (other / "service" / "slam").mkdir(parents=True)
    (other / "service" / "slam" / "Dockerfile").write_text("FROM busybox\n")
    _git(other, "init", "-q")
    _git(other, "add", "-A")
    _git(other, "commit", "-qm", "branch")
    cmd = f"docker build -t vision-picar-ros:latest {other}/service/slam"
    assert run_hook(cmd, cwd=str(checkout))[0] == "deny"
    cmd = f"docker build -t vision-picar-ros:latest -f {other}/service/slam/Dockerfile service/slam"
    assert run_hook(cmd, cwd=str(checkout))[0] == "deny"


def test_latest_from_inside_a_dirty_service_slam_is_refused(checkout):
    (checkout / "service" / "slam" / "new.py").write_text("x = 1\n")
    assert run_hook(LATEST.replace("service/slam", "."),
                    cwd=str(checkout / "service" / "slam"))[0] == "deny"


def test_latest_in_a_checkout_made_by_the_same_command_says_how(tmp_path):
    """Seen in past transcripts: `git worktree add $D origin/dev && cd $D &&
    docker build -t ...:latest`. The hook runs before the checkout exists, so
    it cannot check it; the reason says to build in a second call."""
    d = tmp_path / "dev-fresh"
    decision, reason = run_hook(f"git worktree add -q --detach {d} origin/dev && cd {d} && "
                                f"{LATEST}", cwd=str(tmp_path))
    assert decision == "deny" and "build in the next" in reason


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
