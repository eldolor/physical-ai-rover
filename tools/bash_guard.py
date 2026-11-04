# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tools/bash_guard.py -- Claude Code's PreToolUse hook on Bash (3.150).

    python tools/bash_guard.py hook     read the hook's JSON on stdin; print a
                                        refusal, or nothing to let it run

Refuses the three things a session never does on its own:

  * `PICAR_SKIP_GATE=1` / `PICAR_CLAIM_BYPASS=1` -- the user's escapes from
    the push gate and the claim hooks (CLAUDE.md 3c, PARALLEL-SESSIONS rule 12);
  * `--no-verify` on `git push`/`git commit`, or `core.hooksPath`, which skip
    those hooks altogether;
  * `vision-picar-ros:latest` built anywhere but a clean `origin/dev`
    checkout, or tagged onto by `docker tag` -- every session's live runs
    use it ("never rebuild :latest from a branch").

A refusal asks where Claude Code shows the user a question and denies
elsewhere, as `plan_claims.py hook-edit` does. If the command cannot be read,
or a check raises, a command naming a guarded word is refused and any other
passes (failing closed). A seatbelt for honest mistakes, not a sandbox: a
script run from a file is not read.
"""

import json
import os
import re
import shlex
import subprocess
import sys

BYPASS_ENV = ("PICAR_SKIP_GATE", "PICAR_CLAIM_BYPASS")
SHARED_IMAGE = "vision-picar-ros"
GUARDED_WORDS = BYPASS_ENV + ("no-verify", "hookspath", SHARED_IMAGE)
ASK_MODES = ("default", "plan", "acceptEdits")
SEPARATORS = {";", "&&", "||", "|", "&", "|&", "(", ")", "\n"}
# Words that run the command after them (their own NAME=VAL and flags skipped).
PREFIXES = {"env", "sudo", "nohup", "command", "time", "exec", "nice", "timeout"}
HEREDOC_RE = re.compile(r"<<(-?)\s*(['\"]?)([A-Za-z_][A-Za-z0-9_]*)\2")
ASSIGN_RE = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)=(.*)$", re.S)


class Refusal(Exception):
    pass


def strip_heredocs(text: str) -> str:
    """The command without heredoc bodies: their lines are data, not commands."""
    out, lines, i = [], text.split("\n"), 0
    while i < len(lines):
        line = lines[i]
        out.append(line)
        ends = [(m.group(1) == "-", m.group(3)) for m in HEREDOC_RE.finditer(line)]
        i += 1
        for dash, word in ends:
            while i < len(lines) and (lines[i].lstrip("\t") if dash else lines[i]) != word:
                i += 1
            i += 1
    return "\n".join(out)


def simple_commands(text: str) -> list:
    """The command split into simple commands, each a list of words."""
    lex = shlex.shlex(strip_heredocs(text).replace("\n", " ; "), posix=True,
                      punctuation_chars=";&|()")
    lex.whitespace_split = True
    cmds, cur = [], []
    for tok in lex:
        if tok in SEPARATORS or set(tok) <= set(";&|()"):
            if cur:
                cmds.append(cur)
            cur = []
        else:
            cur.append(tok)
    if cur:
        cmds.append(cur)
    return cmds


def _assignments(words: list) -> tuple:
    """(NAME=VAL pairs before the program, the words from the program on),
    looking through `env`, `sudo` and the like."""
    pairs, i = [], 0
    while i < len(words):
        m = ASSIGN_RE.match(words[i])
        if m:
            pairs.append((m.group(1), m.group(2)))
        elif words[i] in PREFIXES:
            pass
        elif words[i].startswith("-") and i > 0 and words[i - 1] in PREFIXES | {"-u"}:
            if words[i] in ("-u", "--unset") and i + 1 < len(words):
                i += 1                    # env -u NAME
        else:
            break
        i += 1
    return pairs, words[i:]


def _bypass(name: str, value: str):
    if name in BYPASS_ENV and value == "1":
        raise Refusal(f"`{name}=1` is the user's escape from "
                      + ("the push gate" if name == "PICAR_SKIP_GATE" else "the claim hooks")
                      + " (CLAUDE.md 3c); a session never sets it")


def check_git(args: list):
    sub, i = None, 0
    while i < len(args):
        a = args[i]
        if a == "-c" and i + 1 < len(args):
            if args[i + 1].lower().startswith("core.hookspath="):
                raise Refusal("`-c core.hooksPath=` skips the repo's git hooks")
            i += 2
            continue
        if a in ("-C", "--git-dir", "--work-tree", "--namespace") and i + 1 < len(args):
            i += 2
            continue
        if a.startswith("-"):
            i += 1
            continue
        sub = a
        break
    rest = args[i + 1:] if sub else []
    if sub in ("push", "commit") and "--no-verify" in rest:
        raise Refusal(f"`git {sub} --no-verify` skips the "
                      + ("push gate" if sub == "push" else "claim hook"))
    if sub == "commit" and any(re.match(r"^-[a-zA-Z]*n", a) and not a.startswith("--")
                               for a in rest):
        raise Refusal("`git commit -n` (no-verify) skips the claim hook")
    if sub == "config":
        keys = [a for a in rest if not a.startswith("-")]
        getting = any(a.startswith("--get") or a in ("-l", "--list") for a in rest)
        if keys and keys[0].lower() == "core.hookspath" and len(keys) > 1 and not getting:
            raise Refusal("setting `core.hooksPath` turns off the repo's git hooks for "
                          "every session")


def _is_shared(tag: str) -> bool:
    name, _, version = tag.rpartition(":") if ":" in tag.split("/")[-1] else (tag, "", "")
    return name.split("/")[-1] == SHARED_IMAGE and version in ("", "latest")


def clean_dev_checkout(cwd: str) -> str:
    """'' if `cwd` is a checkout at origin/dev with nothing changed under
    service/slam/, else why not."""
    def git(*a):
        return subprocess.run(["git", "-C", cwd, *a], capture_output=True, text=True,
                              timeout=10)
    head, dev = git("rev-parse", "HEAD"), git("rev-parse", "origin/dev")
    if head.returncode or dev.returncode:
        return f"{cwd} is not a checkout with an origin/dev"
    if head.stdout.strip() != dev.stdout.strip():
        return f"{cwd} is at {head.stdout.strip()[:9]}, not origin/dev"
    st = git("status", "--porcelain", "--untracked-files=all", "--", "service/slam")
    if st.returncode or st.stdout.strip():
        return f"{cwd} has changes under service/slam/"
    return ""


def check_docker(args: list, cwd: str):
    words = [a for a in args if not a.startswith("-")]
    if words[:1] == ["buildx"]:
        words = words[1:]
    if words[:1] == ["image"]:
        words = words[1:]
    verb = words[0] if words else ""
    if verb == "tag":
        if len(words) >= 3 and _is_shared(words[2]):
            raise Refusal(f"`docker tag` onto `{SHARED_IMAGE}:latest` gives every session "
                          "an image no one built from origin/dev")
        return
    if verb != "build":
        return
    tags, i = [], 0
    while i < len(args):
        a = args[i]
        if a in ("-t", "--tag") and i + 1 < len(args):
            tags.append(args[i + 1])
            i += 1
        elif a.startswith("--tag="):
            tags.append(a[len("--tag="):])
        elif a.startswith("-t") and len(a) > 2 and not a.startswith("--"):
            tags.append(a[2:])
        i += 1
    if any(_is_shared(t) for t in tags):
        why = clean_dev_checkout(cwd)
        if why:
            raise Refusal(f"`{SHARED_IMAGE}:latest` is built only from a clean origin/dev "
                          f"checkout, and {why}; build your own tag (`plan_section.py env`)")


def check_simple(words: list, state: dict):
    pairs, rest = _assignments(words)
    for name, value in pairs:
        _bypass(name, value)
    if not rest:
        return
    prog, args = os.path.basename(rest[0]), rest[1:]
    if prog == "export":
        for a in args:
            m = ASSIGN_RE.match(a)
            if m:
                _bypass(m.group(1), m.group(2))
    elif prog == "cd" and args:
        state["cwd"] = os.path.join(state["cwd"], os.path.expanduser(args[-1]))
    elif prog == "git":
        check_git(args)
    elif prog in ("docker", "podman"):
        check_docker(args, state["cwd"])


def _decision(decision: str, reason: str) -> str:
    return json.dumps({"hookSpecificOutput": {
        "hookEventName": "PreToolUse", "permissionDecision": decision,
        "permissionDecisionReason": reason}})


def _refuse(mode: str, why: str) -> str:
    return _decision("ask" if mode in ASK_MODES else "deny",
                     f"bash_guard (3.150): {why}. This is the user's call: ask them, "
                     "or they run it themselves.")


def _names_guarded(text: str) -> bool:
    low = text.lower()
    return any(w.lower() in low for w in GUARDED_WORDS)


def hook(stdin_text: str) -> str:
    """The hook's stdout for one Bash call: a refusal, or ''."""
    try:
        data = json.loads(stdin_text)
    except ValueError:
        return _refuse("", "could not read the hook's input, and it names a guarded word") \
            if _names_guarded(stdin_text) else ""
    if data.get("tool_name") != "Bash":
        return ""
    mode = data.get("permission_mode") or ""
    command = (data.get("tool_input") or {}).get("command") or ""
    try:
        state = {"cwd": data.get("cwd") or os.getcwd()}
        for words in simple_commands(command):
            check_simple(words, state)
    except Refusal as r:
        return _refuse(mode, str(r))
    except Exception as e:  # failing closed, for guarded commands only
        if _names_guarded(command):
            return _refuse(mode, f"could not read this command ({type(e).__name__}: {e}), "
                                 "and it names a guarded word")
    return ""


def main(argv: list):
    if argv != ["hook"]:
        raise SystemExit(__doc__)
    out = hook(sys.stdin.read())
    if out:
        print(out)


if __name__ == "__main__":
    main(sys.argv[1:])
