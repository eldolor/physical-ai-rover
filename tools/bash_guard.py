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
    checkout, or put there by `docker tag`/`commit`/`import` -- every
    session's live runs use it ("never rebuild :latest from a branch").

A refusal asks where Claude Code shows the user a question and denies
elsewhere, as `plan_claims.py hook-edit` does. If the input cannot be read,
or a check raises, a command naming a guarded word is refused and any other
passes (failing closed). It reads command substitutions, `bash -c`/`eval`
strings, wrappers' `-- CMD` and heredocs fed to a shell or `ssh`; it does not
read scripts run from files: a seatbelt for honest mistakes, not a sandbox.
"""

import fnmatch
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
ASK_MODES = ("default", "plan", "acceptEdits")   # plan_claims.py's (a test pins it)
PUNCT = set(";&|()")
WORD_END = set(" \t\n;&|()<>")
# Words that run the command after them: {word: (options taking a value,
# positional words before the command)}. Shell keywords run what follows.
PREFIXES = {
    "env": ({"-u", "--unset", "-C", "--chdir"}, 0),
    "sudo": ({"-u", "--user", "-g", "--group", "-h", "--host", "-p", "--prompt",
              "-C", "--close-from", "-D", "--chdir", "-r", "--role", "-t", "--type",
              "-U", "--other-user", "-T", "--command-timeout"}, 0),
    "nice": ({"-n", "--adjustment"}, 0),
    "ionice": ({"-c", "--class", "-n", "--classdata", "-p", "--pid"}, 0),
    "timeout": ({"-s", "--signal", "-k", "--kill-after"}, 1),
    "flock": ({"-w", "--timeout", "-E", "--conflict-exit-code"}, 1),
    "watch": ({"-n", "--interval", "-d"}, 0),
    "stdbuf": ({"-i", "-o", "-e"}, 0),
    "nohup": (set(), 0), "command": (set(), 0), "time": (set(), 0), "setsid": (set(), 0),
    "exec": ({"-a"}, 0), "xargs": ({"-I", "-n", "-P", "-d", "-L", "-s", "-E", "-a"}, 0),
}
KEYWORDS = {"if", "then", "else", "elif", "do", "while", "until", "!", "{", "}", "fi", "done"}
SHELLS = {"bash", "sh", "zsh", "dash"}
HEREDOC_RUNNERS = SHELLS | {"ssh"}
SSH_VALUE_FLAGS = set("BbcDEeFIiJLlmOoPpQRSWw")
DECLARERS = {"export", "declare", "typeset", "readonly", "local"}
ASSIGN_RE = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)=(.*)$", re.S)
GIT_VALUE_OPTS = {"-C", "--git-dir", "--work-tree", "--namespace", "--exec-path",
                  "--super-prefix"}
# Options whose value is the next word, so it is text, never a flag.
GIT_SUB_VALUE_OPTS = {
    "commit": {"-m", "-F", "-C", "-c", "-t", "--message", "--file", "--template",
               "--reuse-message", "--reedit-message", "--author", "--date", "--trailer",
               "--cleanup", "--fixup", "--squash", "--pathspec-from-file"},
    "push": {"-o", "--push-option", "--repo", "--receive-pack", "--exec"},
}
# git commit's short options that take a value: in a cluster the rest of it
# (or the next word) is that value, never more flags (`-uno`, `-mnote`).
COMMIT_VALUE_SHORT = set("mFCctuS")
DOCKER_GLOBAL_VALUE_OPTS = {"--context", "-c", "-H", "--host", "--config", "-l",
                            "--log-level", "--tlscacert", "--tlscert", "--tlskey"}
DOCKER_BUILD_VALUE_OPTS = {
    "-t", "--tag", "-f", "--file", "--build-arg", "--target", "--platform", "--label",
    "--network", "--cache-from", "--cache-to", "--secret", "--ssh", "-o", "--output",
    "--progress", "--iidfile", "--add-host", "--build-context", "--shm-size", "--ulimit",
    "-m", "--memory", "--memory-swap", "--cpu-shares", "--cpuset-cpus", "--isolation",
    "--metadata-file", "--builder", "--allow", "--attest", "--annotation",
    "--cgroup-parent", "--iidfile"}
REDIRECT_RE = re.compile(r"(\d*)(&>>|&>|>>|>&|>\||<&|<>|>|<)")


class Refusal(Exception):
    pass


def _word_end(text: str, j: int) -> int:
    """The end of the shell word starting at text[j] (quotes kept inside)."""
    quote = None
    while j < len(text):
        c = text[j]
        if quote:
            if c == "\\" and quote == '"':
                j += 2
                continue
            if c == quote:
                quote = None
        elif c in "'\"`":
            quote = c
        elif c == "\\":
            j += 2
            continue
        elif c in WORD_END:
            break
        j += 1
    return j


def _segment_runs_heredoc(out: list) -> bool:
    """Whether the simple command written so far feeds a heredoc to a shell
    or `ssh` (`bash <<EOF`, `ssh host bash -s <<EOF`)."""
    seg = "".join(out)
    cut = max(seg.rfind(ch) for ch in ";|&(")
    try:
        words = shlex.split(seg[cut + 1:])
    except ValueError:
        words = seg[cut + 1:].split()
    return any(os.path.basename(w) in HEREDOC_RUNNERS for w in words)


def _piped_to_shell(tail: str) -> bool:
    """Whether the rest of a heredoc's line pipes it into a shell reading
    stdin (`cat <<EOF | bash`), not one given `-c`."""
    for seg in tail.split("|")[1:]:
        try:
            words = shlex.split(seg)
        except ValueError:
            words = seg.split()
        words = [w for w in words if w not in ("sudo", "env") and not ASSIGN_RE.match(w)]
        if words and os.path.basename(words[0]) in SHELLS and not any(
                re.match(r"^-[a-zA-Z]*c[a-zA-Z]*$", w) for w in words[1:]):
            return True
    return False


def _scan(text: str, i: int, extras: list, closer: str = "") -> tuple:
    """Read shell from text[i] to `closer` (")" or "`") or the end. Returns
    (the text as one line for shlex, the index after the closer). Comments,
    redirections and heredoc bodies are dropped and each newline outside
    quotes is a `;`. Every command substitution's body, and every heredoc
    fed to a shell, is appended to `extras` for its own check; an unquoted
    heredoc's substitutions too."""
    out, quote, depth, pending, cases, arith = [], None, 0, [], 0, 0
    while i < len(text):
        c = text[i]
        at_word = not out or out[-1][-1:] in " \t\n;&|()"
        if quote is None and at_word:
            m = re.match(r"(case|esac)(?=[\s;)]|$)", text[i:])
            if m:
                cases += 1 if m.group(1) == "case" else -1 if cases else 0
                out.append(m.group(1))
                i += 4
                continue
        if quote == "'":
            out.append(c)
            quote = None if c == "'" else quote
            i += 1
            continue
        if c == "\\" and i + 1 < len(text):
            if text[i + 1] == "\n":
                i += 2                    # a line continuation
            else:
                out.append(text[i:i + 2])
                i += 2
            continue
        if closer == "`" and c == "`":
            return "".join(out), i + 1
        if text.startswith("$(", i) and not text.startswith("$((", i):
            start = i + 2
            _, i = _scan(text, start, [], ")")   # its own check finds what it holds
            extras.append(text[start:i - 1])
            out.append("SUBST")
            continue
        if c == "`" and closer != "`":
            start = i + 1
            _, i = _scan(text, start, [], "`")
            extras.append(text[start:i - 1])
            out.append("SUBST")
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
        if quote is None and text.startswith("((", i):
            arith += 1                    # `(( ))` / `$(( ))`: `<<` is a shift there
            out.append("((")
            i += 2
            continue
        if quote is None and arith and text.startswith("))", i):
            arith -= 1
            out.append("))")
            i += 2
            continue
        if arith:
            out.append(c)
            i += 1
            continue
        if text.startswith("<<<", i):
            j0 = i + 3 + len(text[i + 3:]) - len(text[i + 3:].lstrip(" \t"))
            j = _word_end(text, j0)
            _substitutions(text[j0:j], extras)
            if _segment_runs_heredoc(out):
                try:
                    extras.append(" ".join(shlex.split(text[j0:j])))
                except ValueError:
                    extras.append(text[j0:j])
            out.append(" ")
            i = j
            continue
        if text.startswith("<<", i):
            j = i + 2
            dash = text.startswith("-", j)
            j += dash
            while j < len(text) and text[j] in " \t":
                j += 1
            k = _word_end(text, j)
            raw = text[j:k]
            if raw:
                word = raw.replace("'", "").replace('"', "").replace("\\", "")
                pending.append((dash, word, raw != word, _segment_runs_heredoc(out), len(out)))
                out.append(" ")
                i = k
                continue
        if c in "<>" and text.startswith("(", i + 1):
            start = i + 2                 # process substitution
            _, i = _scan(text, start, [], ")")
            extras.append(text[start:i - 1])
            out.append("SUBST")
            continue
        m = REDIRECT_RE.match(text, i)
        if m and (not m.group(1) or not out or out[-1][-1:] in " \t;&|()"):
            j = m.end()
            while j < len(text) and text[j] in " \t":
                j += 1
            i = _word_end(text, j)        # the redirection and its target
            _substitutions(text[j:i], extras)
            out.append(" ")
            continue
        if c == "(":
            depth += 1
        elif c == ")" and cases and depth == 0:
            pass                          # a case pattern's `)`
        elif c == ")":
            if closer == ")" and depth == 0:
                return "".join(out), i + 1
            depth -= 1
        if c == "\n":
            out.append(" ; ")
            i += 1
            for dash, word, quoted, to_shell, at in pending:
                to_shell = to_shell or _piped_to_shell("".join(out[at:]))
                lines = text[i:].split("\n")
                for n, line in enumerate(lines):
                    if (line.lstrip("\t") if dash else line) == word:
                        body = "\n".join(lines[:n])
                        i += sum(len(x) + 1 for x in lines[:n + 1])
                        if to_shell:
                            extras.append(body)
                        elif not quoted:
                            _substitutions(body, extras)
                        break
            pending = []
            continue
        out.append(c)
        i += 1
    return "".join(out), i


def _substitutions(body: str, extras: list):
    """The `$(...)` and backticks in an unquoted heredoc's body, which bash
    runs when it expands it."""
    i = 0
    while i < len(body):
        if body[i] == "\\":
            i += 2
            continue
        if body.startswith("$(", i) and not body.startswith("$((", i):
            _, j = _scan(body, i + 2, [], ")")
            extras.append(body[i + 2:j - 1])
            i = j
        elif body[i] == "`":
            _, j = _scan(body, i + 1, [], "`")
            extras.append(body[i + 1:j - 1])
            i = j
        else:
            i += 1


def simple_commands(text: str, extras: list = None) -> list:
    """The command split into simple commands, each a list of words; the
    texts to check on their own are appended to `extras`."""
    flat, _ = _scan(text, 0, extras if extras is not None else [])
    lex = shlex.shlex(flat, posix=True, punctuation_chars=";&|()")
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


def _strip_prefixes(words: list, state: dict) -> list:
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
                elif w == "env" and x in ("-S", "--split-string") and i + 1 < n:
                    check_text(words[i + 1], dict(state))   # env -S 'CMD'
                    i += 2
                elif w == "env" and x.startswith("--split-string="):
                    check_text(x.split("=", 1)[1], dict(state))
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


def _expand(word: str, variables: dict) -> list:
    """`word` with each `$NAME`/`${NAME}` replaced by every value a literal
    `for NAME in ...` or `NAME=...` earlier in the command gave it; a
    variable it cannot resolve stays, so the caller reads it as unknown."""
    m = re.search(r"\$\{?([A-Za-z_][A-Za-z0-9_]*)\}?", word)
    if not m or m.group(1) not in (variables or {}):
        return [word]
    out = []
    for v in variables[m.group(1)][:50]:
        out += _expand(word[:m.start()] + v + word[m.end():], variables)
    return out or [word]


def _push_reaches_dev(rest: list, flags: list, cwd: str, variables: dict = None) -> bool:
    """Whether a push may update `dev`, the one ref the pre-push gate checks
    (tools/hooks/pre-push): a refspec onto it, `--all`/`--mirror`, or a
    push of the current branch (no refspec, or `HEAD`) while on `dev`."""
    if any(f in ("--all", "--mirror", "--branches") for f in flags):
        return True
    value_opts, pos, skip = GIT_SUB_VALUE_OPTS["push"], [], False
    for a in rest:
        if skip:
            skip = False
        elif a.startswith("-"):
            skip = a in value_opts
        else:
            pos.append(a)
    if any(f == "--repo" or f.startswith("--repo=") for f in flags):
        pos = [""] + pos                  # the remote is --repo's: every word is a refspec
    if any(f in ("-d", "--delete") for f in flags):
        return any(_ref_name(r) == "dev" for r in pos[1:])
    refspecs = pos[1:]
    if not refspecs:
        return _current_branch(cwd) in ("dev", "")
    for r in [x for spec in refspecs for x in _expand(spec, variables)]:
        dest = r.lstrip("+").split(":")[-1] if ":" in r else r.lstrip("+")
        if dest == "HEAD":
            dest = _current_branch(cwd)
        name = _ref_name(dest)
        if name in ("dev", "") or "$" in name or ("*" in name and fnmatch.fnmatch("dev", name)):
            return True
    return False


def _ref_name(ref: str) -> str:
    return ref[len("refs/heads/"):] if ref.startswith("refs/heads/") else ref


def _current_branch(cwd: str) -> str:
    """The branch checked out in `cwd`; '' when it cannot be told (or the
    push runs on another host)."""
    if cwd is None:
        return ""
    try:
        r = subprocess.run(["git", "-C", cwd, "symbolic-ref", "-q", "--short", "HEAD"],
                           capture_output=True, text=True, timeout=3)
    except (OSError, subprocess.TimeoutExpired):
        return ""
    return r.stdout.strip() if r.returncode == 0 else "(detached)"


def _repo_hooks(value: str) -> bool:
    v = os.path.normpath(os.path.expanduser(value))
    return v == REPO_HOOKS or (os.path.isabs(v) and v.endswith("/" + REPO_HOOKS))


def _flags(sub: str, rest: list) -> list:
    """`rest`'s option words, without the values of options that take one."""
    value_opts, out, skip = GIT_SUB_VALUE_OPTS.get(sub, set()), [], False
    for a in rest:
        if skip:
            skip = False
            continue
        if a == "--":
            break
        if a.startswith("-"):
            out.append(a)
            skip = a in value_opts
            if sub == "commit" and re.match(r"^-[a-zA-Z]{2,}$", a):
                first = next((ch for ch in a[1:] if ch in COMMIT_VALUE_SHORT), "")
                skip = bool(first) and a.index(first) == len(a) - 1 and first not in "uS"
    return out


def check_git(args: list, cwd: str = ".", variables: dict = None):
    sub, i, hooks_off = None, 0, False
    while i < len(args):
        a = args[i]
        if a == "-c" and i + 1 < len(args):
            setting = args[i + 1]
            i += 2
        elif a.startswith("-c") and not a.startswith("--"):
            setting = a[2:]
            i += 1
        elif a.startswith("--config-env"):
            setting = a.split("=", 1)[1] if "=" in a else (args[i + 1] if i + 1 < len(args) else "")
            hooks_off = hooks_off or setting.lower().startswith("core.hookspath")
            i += 1 if "=" in a else 2
            continue
        elif a in GIT_VALUE_OPTS:
            if a == "-C" and i + 1 < len(args) and cwd is not None:
                cwd = os.path.join(cwd, os.path.expanduser(args[i + 1]))
            i += 2
            continue
        elif a.startswith("-"):
            i += 1
            continue
        else:
            sub = a
            break
        key, _, value = setting.partition("=")
        if key.lower() == "core.hookspath" and not _repo_hooks(value):
            hooks_off = True
    rest = args[i + 1:] if sub else []
    if hooks_off and (sub == "commit" or (sub == "push" and _push_reaches_dev(
            rest, _flags(sub, rest), cwd, variables))):
        raise Refusal(f"git's `-c core.hooksPath` on `{sub}` skips the repo's "
                      + ("push gate" if sub == "push" else "claim hook"))
    flags = _flags(sub, rest)
    no_verify = any(a.startswith("--no-veri") for a in flags)
    if sub == "commit" and no_verify:
        raise Refusal("`git commit --no-verify` skips the claim hook")
    if sub == "push" and no_verify and _push_reaches_dev(rest, flags, cwd, variables):
        raise Refusal("`git push --no-verify` to `dev` skips the push gate")
    if sub == "commit":
        for a in flags:
            if a.startswith("--"):
                continue
            for ch in a[1:]:
                if ch == "n":
                    raise Refusal("`git commit -n` (no-verify) skips the claim hook")
                if ch in COMMIT_VALUE_SHORT:
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
    if pos and pos[0] in ("set", "unset", "get", "list", "edit", "rename-section",
                          "remove-section"):
        verb, pos = pos[0], pos[1:]
    elif any(f.startswith("--unset") for f in flags):
        verb = "unset"
    elif any(f in ("--remove-section", "--rename-section") for f in flags):
        verb = flags[[f in ("--remove-section", "--rename-section") for f in flags].index(True)][2:]
    elif any(f.startswith("--get") or f in ("-l", "--list") for f in flags):
        verb = "get"
    else:
        verb = "set" if len(pos) >= 2 else "get"
    if verb in ("remove-section", "rename-section"):
        if pos and pos[0].lower() == "core":
            raise Refusal(f"`git config {verb} core` drops `core.hooksPath`, turning off the "
                          "repo's git hooks for every session")
        return
    if not pos or pos[0].lower() != "core.hookspath":
        return
    if verb == "unset":
        raise Refusal("unsetting `core.hooksPath` turns off the repo's git hooks for "
                      "every session")
    if verb == "set" and not (len(pos) > 1 and _repo_hooks(pos[1])):
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
        return (f"{path} does not exist yet; create the clean origin/dev checkout in one "
                "call and build in the next, so this hook can check it")
    try:
        head, dev = git("rev-parse", "HEAD"), git("rev-parse", "origin/dev")
        if head.returncode or dev.returncode:
            return f"{path} is not in a checkout with an origin/dev"
        if head.stdout.strip() != dev.stdout.strip():
            return f"{path} is at {head.stdout.strip()[:9]}, not origin/dev"
        st = git("status", "--porcelain", "--untracked-files=all", "--", ":/service/slam")
    except subprocess.TimeoutExpired:
        return f"git took over 3 s to answer about {path} (a busy desktop?); try again"
    if st.returncode or st.stdout.strip():
        return f"{path}'s checkout has changes under service/slam/"
    return ""


def check_docker(args: list, cwd: str):
    i = 0
    while i < len(args) and args[i].startswith("-"):
        i += 2 if args[i] in DOCKER_GLOBAL_VALUE_OPTS else 1
    words = args[i:]
    while words[:1] and words[0] in ("buildx", "image", "builder", "container"):
        words = words[1:]
    verb, opts = (words[0], words[1:]) if words else ("", [])
    if verb in ("tag", "commit", "import"):
        pos = [a for a in opts if not a.startswith("-")]
        if len(pos) >= 2 and _is_shared(pos[1]):
            raise Refusal(f"`docker {verb}` onto `{SHARED_IMAGE}:latest` gives every session "
                          "an image no one built from origin/dev")
        return
    if verb != "build":
        return
    tags, files, pos, i = [], [], [], 0
    while i < len(opts):
        a = opts[i]
        if a.startswith("--") and "=" in a:
            name, value = a.split("=", 1)
            (tags if name == "--tag" else files if name == "--file" else []).append(value)
        elif re.match(r"^-[a-zA-Z]+$", a) and a[-1] in "tf" and i + 1 < len(opts) \
                and a not in DOCKER_BUILD_VALUE_OPTS:
            (tags if a[-1] == "t" else files).append(opts[i + 1])   # -qt NAME
            i += 1
        elif a in DOCKER_BUILD_VALUE_OPTS and i + 1 < len(opts):
            if a in ("-t", "--tag"):
                tags.append(opts[i + 1])
            elif a in ("-f", "--file"):
                files.append(opts[i + 1])
            i += 1
        elif a.startswith("-t") and len(a) > 2 and not a.startswith("--"):
            tags.append(a[2:])
        elif a.startswith("-f") and len(a) > 2 and not a.startswith("--"):
            files.append(a[2:])
        elif not a.startswith("-"):
            pos.append(a)
        i += 1
    if not any(_is_shared(t) for t in tags):
        return
    if len(pos) != 1 or pos[0] == "-":
        raise Refusal(f"`{SHARED_IMAGE}:latest` is built only from a clean origin/dev "
                      "checkout, and this build's context cannot be read")
    places = [os.path.join(cwd, os.path.expanduser(pos[0]))]
    places += [os.path.dirname(os.path.join(cwd, os.path.expanduser(f))) for f in files]
    for p in places:
        why = clean_dev_checkout(os.path.normpath(p))
        if why:
            raise Refusal(f"`{SHARED_IMAGE}:latest` is built only from a clean origin/dev "
                          f"checkout, and {why}; build your own tag (`plan_section.py env`)")


def check_simple(words: list, state: dict):
    if len(words) >= 3 and words[0] == "for" and words[2] == "in":
        state.setdefault("vars", {})[words[1]] = words[3:]     # for b in x y z
        return
    if words and all(ASSIGN_RE.match(w) for w in words):
        for w in words:                   # plain NAME=VAL: kept for refspecs
            m = ASSIGN_RE.match(w)
            _check_assignment(m.group(1), m.group(2))
            state.setdefault("vars", {})[m.group(1)] = [m.group(2)]
        return
    rest = _strip_prefixes(words, state)
    if not rest:
        return
    prog, args = os.path.basename(rest[0]), rest[1:]
    if prog in DECLARERS:
        for a in args:
            m = ASSIGN_RE.match(a)
            if m:
                _check_assignment(m.group(1), m.group(2))
    elif prog in ("cd", "pushd"):
        target = args[-1] if args else "~"
        if target != "-":
            state["cwd"] = os.path.join(state["cwd"], os.path.expanduser(target))
    elif prog == "git":
        check_git(args, None if state.get("remote") else state["cwd"], state.get("vars", {}))
    elif prog in ("docker", "podman"):
        if not state.get("remote"):       # a remote build has no checkout here to check
            check_docker(args, state["cwd"])
    elif prog == "ssh":
        remote = _ssh_command(args)
        if remote:
            check_text(" ".join(remote), {"cwd": state["cwd"], "remote": True})
    elif prog == "eval":
        check_text(" ".join(args), state)
    elif prog in SHELLS:
        for k, a in enumerate(args):
            if re.match(r"^-[a-zA-Z]*c[a-zA-Z]*$", a) and k + 1 < len(args):
                check_text(args[k + 1], dict(state))
                break
    elif "--" in args:                    # a wrapper's `-- CMD` (desktop_lock, board_lock)
        check_simple(args[args.index("--") + 1:], dict(state))


def _ssh_command(args: list) -> list:
    """The remote command's words of `ssh [options] host [command]`."""
    i = 0
    while i < len(args) and args[i].startswith("-"):
        a = args[i]
        i += 2 if len(a) == 2 and a[1] in SSH_VALUE_FLAGS else 1
    return args[i + 1:]


def check_text(text: str, state: dict):
    extras = []
    for words in simple_commands(text, extras):
        check_simple(words, state)
    for body in extras:
        check_text(body, dict(state))


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
