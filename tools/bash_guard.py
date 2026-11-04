# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tools/bash_guard.py -- Claude Code's PreToolUse hook on Bash (3.150).

    python tools/bash_guard.py hook     read the hook's JSON on stdin; print a
                                        refusal, or nothing to let it run

Refuses the three things a session never does on its own:

  * `PICAR_SKIP_GATE=1` / `PICAR_CLAIM_BYPASS=1` -- the user's escapes from
    the push gate and the claim hooks (CLAUDE.md 3c, PARALLEL-SESSIONS rule 12);
  * `--no-verify` on `git push`/`git commit`, or a `core.hooksPath` other than
    the repo's `tools/hooks`, which skip those hooks altogether;
  * `vision-picar-ros:latest` built from anything but a clean `origin/dev`
    checkout, or tagged onto by `docker tag` -- every session's live runs
    use it ("never rebuild :latest from a branch").

A refusal asks where Claude Code shows the user a question and denies
elsewhere, as `plan_claims.py hook-edit` does. If the input cannot be read,
or a check raises, a command naming a guarded word is refused and any other
passes (failing closed). It reads `bash -c` / `eval` strings, not scripts
run from files: a seatbelt for honest mistakes, not a sandbox.
"""

import json
import os
import re
import shlex
import subprocess
import sys

BYPASS_ENV = ("PICAR_SKIP_GATE", "PICAR_CLAIM_BYPASS")
SHARED_IMAGE = "vision-picar-ros"
REPO_HOOKS = "tools/hooks"
GUARDED_WORDS = BYPASS_ENV + ("no-ver", "hookspath", "git_config", "config-env", SHARED_IMAGE)
ASK_MODES = ("default", "plan", "acceptEdits")
PUNCT = set(";&|()")
# Words that run the command after them: {word: (options taking a value,
# positional words before the command)}. Shell keywords run what follows.
PREFIXES = {
    "env": ({"-u", "--unset", "-C", "--chdir", "-S", "--split-string"}, 0),
    "sudo": ({"-u", "--user", "-g", "--group", "-h", "--host", "-p", "--prompt",
              "-C", "--close-from", "-D", "--chdir", "-r", "--role", "-t", "--type",
              "-U", "--other-user", "-T", "--command-timeout"}, 0),
    "nice": ({"-n", "--adjustment"}, 0),
    "timeout": ({"-s", "--signal", "-k", "--kill-after"}, 1),
    "nohup": (set(), 0), "command": (set(), 0), "time": (set(), 0),
    "exec": ({"-a"}, 0), "xargs": ({"-I", "-n", "-P", "-d", "-L", "-s", "-E", "-a"}, 0),
}
KEYWORDS = {"if", "then", "else", "elif", "do", "while", "until", "!", "{", "}", "fi", "done"}
SHELLS = {"bash", "sh", "zsh", "dash"}
DECLARERS = {"export", "declare", "typeset", "readonly", "local"}
ASSIGN_RE = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)=(.*)$", re.S)
GIT_VALUE_OPTS = {"-C", "--git-dir", "--work-tree", "--namespace", "--exec-path",
                  "--super-prefix", "--config-env"}
DOCKER_GLOBAL_VALUE_OPTS = {"--context", "-c", "-H", "--host", "--config", "-l",
                            "--log-level", "--tlscacert", "--tlscert", "--tlskey"}
# git commit's short options that take a value: in a cluster, the rest of it
# (or the next word) is that value, never more flags (`-uno`, `-mnote`).
COMMIT_VALUE_SHORT = set("mFCctuS")


class Refusal(Exception):
    pass


def _delimiter(text: str, i: int) -> tuple:
    """A heredoc's (strip tabs, delimiter, index after it) at text[i:] just
    past `<<`, or None."""
    dash = text.startswith("-", i)
    j = i + dash
    while j < len(text) and text[j] in " \t":
        j += 1
    m = re.match(r"(['\"]?)([A-Za-z_][A-Za-z0-9_]*)\1", text[j:])
    return (dash, m.group(2), j + m.end()) if m else None


def preprocess(text: str) -> str:
    """The command as one line of shell for shlex: line continuations joined,
    comments and heredoc bodies dropped, each newline outside quotes a `;`.
    Quote state is tracked, so a `#` or `<<` inside a string is text."""
    out, i, quote, pending = [], 0, None, []
    while i < len(text):
        c = text[i]
        if quote == "'":
            out.append(c)
            quote = None if c == "'" else quote
            i += 1
            continue
        if c == "\\" and i + 1 < len(text):
            if text[i + 1] == "\n":
                i += 2                    # a line continuation
                continue
            out.append(text[i:i + 2])
            i += 2
            continue
        if quote == '"':
            out.append(c)
            quote = None if c == '"' else quote
            i += 1
            continue
        if c in "'\"":
            quote = c
            out.append(c)
            i += 1
            continue
        if c == "#" and (not out or out[-1][-1:] in " \t\n;&|()"):
            while i < len(text) and text[i] != "\n":
                i += 1                    # a comment, to the end of its line
            continue
        if text.startswith("<<<", i):
            out.append("<<<")
            i += 3
            continue
        if text.startswith("<<", i):
            d = _delimiter(text, i + 2)
            if d:
                pending.append(d[:2])
                out.append(" ")
                i = d[2]
                continue
        if c == "\n":
            out.append(" ; ")
            i += 1
            for dash, word in pending:    # skip each heredoc's body
                lines = text[i:].split("\n")
                for n, line in enumerate(lines):
                    if (line.lstrip("\t") if dash else line) == word:
                        i += sum(len(x) + 1 for x in lines[:n + 1])
                        break
                else:
                    pass                  # no closing line: keep the rest as commands
            pending = []
            continue
        out.append(c)
        i += 1
    return "".join(out)


def simple_commands(text: str) -> list:
    """The command split into simple commands, each a list of words."""
    lex = shlex.shlex(preprocess(text), posix=True, punctuation_chars=";&|()")
    lex.whitespace_split = True
    lex.commenters = ""
    cmds, cur = [], []
    for tok in lex:
        if tok and set(tok) <= PUNCT:
            if cur:
                cmds.append(cur)
            cur = []
        else:
            cur.append(tok)
    if cur:
        cmds.append(cur)
    return cmds


def _check_assignment(name: str, value: str):
    if name in BYPASS_ENV and value == "1":
        raise Refusal(f"`{name}=1` is the user's escape from "
                      + ("the push gate" if name == "PICAR_SKIP_GATE" else "the claim hooks")
                      + " (CLAUDE.md 3c); a session never sets it")
    if (name == "GIT_CONFIG_PARAMETERS" or name.startswith("GIT_CONFIG_KEY_")) \
            and "hookspath" in value.lower():
        raise Refusal(f"`{name}` setting core.hooksPath skips the repo's git hooks")


def _strip_prefixes(words: list) -> list:
    """The words from the program on; NAME=VAL before it (or after `env` /
    `sudo`) checked on the way, through keywords and wrappers."""
    i, n = 0, len(words)
    while i < n:
        w = words[i]
        m = ASSIGN_RE.match(w)
        if m:
            _check_assignment(m.group(1), m.group(2))
            i += 1
        elif w in KEYWORDS:
            i += 1
        elif w in PREFIXES:
            opts, positional = PREFIXES[w]
            i += 1
            while i < n:
                x = words[i]
                m = ASSIGN_RE.match(x)
                if m and w in ("env", "sudo"):
                    _check_assignment(m.group(1), m.group(2))
                    i += 1
                elif x == "--":
                    i += 1
                    break
                elif x.startswith("-") and x != "-":
                    i += 2 if x in opts else 1
                elif positional:
                    positional -= 1
                    i += 1
                else:
                    break
        else:
            break
    return words[i:]


def check_git(args: list):
    sub, i = None, 0
    while i < len(args):
        a = args[i]
        if a == "-c" and i + 1 < len(args):
            key = args[i + 1]
            i += 2
        elif a.startswith("-c") and not a.startswith("--"):
            key = a[2:]
            i += 1
        elif a.startswith("--config-env"):
            key = a.split("=", 1)[1] if "=" in a else (args[i + 1] if i + 1 < len(args) else "")
            i += 1 if "=" in a else 2
        elif a in GIT_VALUE_OPTS:
            i += 2
            continue
        elif a.startswith("-"):
            i += 1
            continue
        else:
            sub = a
            break
        if key.lower().startswith("core.hookspath"):
            raise Refusal("git's `-c core.hooksPath` skips the repo's git hooks")
    rest = args[i + 1:] if sub else []
    if sub in ("push", "commit") and any(a.startswith("--no-veri") for a in rest):
        raise Refusal(f"`git {sub} --no-verify` skips the "
                      + ("push gate" if sub == "push" else "claim hook"))
    if sub == "commit":
        skip = False
        for a in rest:
            if skip:
                skip = False
                continue
            if a == "--":
                break
            if not a.startswith("-") or a.startswith("--"):
                continue
            for k, ch in enumerate(a[1:]):
                if ch == "n":
                    raise Refusal("`git commit -n` (no-verify) skips the claim hook")
                if ch in COMMIT_VALUE_SHORT:
                    skip = k == len(a) - 2 and ch not in "uS"   # value is the next word
                    break
    if sub == "config":
        _check_git_config(rest)


def _check_git_config(rest: list):
    flags, pos, i = [], [], 0
    while i < len(rest):
        a = rest[i]
        if a in ("-f", "--file", "--blob", "--type", "--default", "--comment", "--value"):
            i += 2
            continue
        (flags if a.startswith("-") else pos).append(a)
        i += 1
    verb = None
    if pos and pos[0] in ("set", "unset", "get", "list", "edit", "rename-section",
                          "remove-section"):
        verb, pos = pos[0], pos[1:]
    elif any(f.startswith("--unset") for f in flags):
        verb = "unset"
    elif any(f.startswith("--get") or f in ("-l", "--list") for f in flags):
        verb = "get"
    else:
        verb = "set" if len(pos) >= 2 else "get"
    if not pos or pos[0].lower() != "core.hookspath":
        return
    if verb == "unset":
        raise Refusal("unsetting `core.hooksPath` turns off the repo's git hooks for "
                      "every session")
    if verb == "set" and pos[1:2] != [REPO_HOOKS] and pos[1:2] != [REPO_HOOKS + "/"]:
        raise Refusal(f"`core.hooksPath` other than `{REPO_HOOKS}` turns off the repo's "
                      "git hooks for every session")


def _is_shared(tag: str) -> bool:
    last = tag.split("/")[-1]
    name, _, version = last.partition(":")
    return name == SHARED_IMAGE and version in ("", "latest")


def clean_dev_checkout(path: str) -> str:
    """'' if `path` lies in a checkout at origin/dev with nothing changed
    under its service/slam/, else why not."""
    def git(*a):
        return subprocess.run(["git", "-C", path, *a], capture_output=True, text=True,
                              timeout=3)
    if not os.path.isdir(path):
        return f"{path} is not a directory here"
    head, dev = git("rev-parse", "HEAD"), git("rev-parse", "origin/dev")
    if head.returncode or dev.returncode:
        return f"{path} is not in a checkout with an origin/dev"
    if head.stdout.strip() != dev.stdout.strip():
        return f"{path} is at {head.stdout.strip()[:9]}, not origin/dev"
    st = git("status", "--porcelain", "--untracked-files=all", "--", ":/service/slam")
    if st.returncode or st.stdout.strip():
        return f"{path}'s checkout has changes under service/slam/"
    return ""


def check_docker(args: list, cwd: str):
    i = 0
    while i < len(args) and args[i].startswith("-"):
        i += 2 if args[i] in DOCKER_GLOBAL_VALUE_OPTS else 1
    words = args[i:]
    while words[:1] and words[0] in ("buildx", "image", "builder"):
        words = words[1:]
    verb, opts = (words[0], words[1:]) if words else ("", [])
    if verb == "tag":
        pos = [a for a in opts if not a.startswith("-")]
        if len(pos) >= 2 and _is_shared(pos[1]):
            raise Refusal(f"`docker tag` onto `{SHARED_IMAGE}:latest` gives every session "
                          "an image no one built from origin/dev")
        return
    if verb != "build":
        return
    tags, files, i = [], [], 0
    while i < len(opts):
        a = opts[i]
        if a in ("-t", "--tag", "-f", "--file") and i + 1 < len(opts):
            (tags if a in ("-t", "--tag") else files).append(opts[i + 1])
            i += 1
        elif a.startswith("--tag="):
            tags.append(a[len("--tag="):])
        elif a.startswith("--file="):
            files.append(a[len("--file="):])
        elif a.startswith("-t") and len(a) > 2 and not a.startswith("--"):
            tags.append(a[2:])
        elif a.startswith("-f") and len(a) > 2 and not a.startswith("--"):
            files.append(a[2:])
        i += 1
    if not any(_is_shared(t) for t in tags):
        return
    context = opts[-1] if opts and not opts[-1].startswith("-") else ""
    if not context:
        raise Refusal(f"`{SHARED_IMAGE}:latest` is built only from a clean origin/dev "
                      "checkout, and this build's context cannot be read")
    places = [os.path.join(cwd, os.path.expanduser(context))]
    places += [os.path.dirname(os.path.join(cwd, os.path.expanduser(f))) for f in files]
    for p in places:
        why = clean_dev_checkout(os.path.normpath(p))
        if why:
            raise Refusal(f"`{SHARED_IMAGE}:latest` is built only from a clean origin/dev "
                          f"checkout, and {why}; build your own tag (`plan_section.py env`)")


def check_simple(words: list, state: dict):
    rest = _strip_prefixes(words)
    if not rest:
        return
    prog, args = os.path.basename(rest[0]), rest[1:]
    if prog in DECLARERS:
        for a in args:
            m = ASSIGN_RE.match(a)
            if m:
                _check_assignment(m.group(1), m.group(2))
    elif prog == "cd" and args:
        state["cwd"] = os.path.join(state["cwd"], os.path.expanduser(args[-1]))
    elif prog == "git":
        check_git(args)
    elif prog in ("docker", "podman"):
        check_docker(args, state["cwd"])
    elif prog == "eval":
        check_text(" ".join(args), state)
    elif prog in SHELLS:
        for k, a in enumerate(args):
            if re.match(r"^-[a-zA-Z]*c[a-zA-Z]*$", a) and k + 1 < len(args):
                check_text(args[k + 1], dict(state))
                break


def check_text(text: str, state: dict):
    for words in simple_commands(text):
        check_simple(words, state)


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
        data = None
    if not isinstance(data, dict):
        return _refuse("", "could not read the hook's input, and it names a guarded word") \
            if _names_guarded(stdin_text) else ""
    if data.get("tool_name") != "Bash":
        return ""
    mode = str(data.get("permission_mode") or "")
    ti = data.get("tool_input")
    command = ti.get("command") if isinstance(ti, dict) else None
    if not isinstance(command, str):
        return _refuse(mode, "could not read the command, and it names a guarded word") \
            if _names_guarded(stdin_text) else ""
    try:
        check_text(command, {"cwd": str(data.get("cwd") or os.getcwd())})
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
    raw = sys.stdin.read()
    try:
        out = hook(raw)
    except Exception as e:  # never fail open on a guarded command
        out = _refuse("", f"the guard failed ({type(e).__name__})") if _names_guarded(raw) else ""
    if out:
        print(out)


if __name__ == "__main__":
    main(sys.argv[1:])
