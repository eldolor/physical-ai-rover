"""
tools/plan_claims.py -- discrete plan steps and live claims, so parallel
Claude Code sessions never work on the same thing (PLAN 3.75 and 3.139,
docs/guides/PARALLEL-SESSIONS.md rules 8 and 12).

    python tools/plan_section.py board [--all]     every open step: claimed, active, waiting, blocked or free
    python tools/plan_section.py next              the first step this session may claim
    python tools/plan_section.py claim <step> [--accept-overlap]
                                                   take a step (refused if it would collide; a whole
                                                   section: `claim 3.N`); --accept-overlap: a file
                                                   shared with an active branch holding no claim was
                                                   checked by hand
    python tools/plan_section.py release [<step>]  give back this session's claims (or one)
    python tools/plan_section.py release --force <step>
                                                   free anyone's claim: THE USER'S WORD ONLY

    python tools/plan_claims.py allow <step>       the USER agreed this session may edit inside that claim
    python tools/plan_claims.py hook-edit          PreToolUse hook: refuse an edit to a section's file or
                                                   an open step's Touches without a claim there; ask (or
                                                   deny, in auto mode) inside another session's claim
    python tools/plan_claims.py hook-commit        pre-commit hook: refuse a commit on plan/3.N-* without
                                                   a claim in 3.N; warn on another session's claim
    python tools/plan_claims.py hook-start         SessionStart hook: the board's short form

Steps are rows of a `## Steps` table in their section's file:

    | Step | Title | Needs | Waits | Touches | State |
    | 3.70.S | Survey mode | 3.50.D | -- | `brain/survey.py`, `control/` | open |

A claim is a BRANCH ON ORIGIN, `claim/<step>` (3.139): an orphan commit
whose message names the holder, pushed create-only (`--force-with-lease`
with an empty lease), so origin's server picks exactly one winner and every
machine sees it. Only after that push is the local claim file written in
the shared .git (`plan-claims/<step>.json`, 3.75), which the hooks read.
The holder is the Claude Code session (`CLAUDE_CODE_SESSION_ID`) when both
sides carry one, else the Claude Code process (the nearest `claude`
ancestor, with its start time), on its host. On this host a claim whose
process has exited may be taken over; another host's is held until it is
released (or freed by the user: `release --force`). Origin unreachable:
`claim` fails -- there is no local-only claim.

`PICAR_CLAIM_BYPASS=1` lets a PERSON past the three hooks. An agent never
sets it.

Standard library only: the hooks run under whatever `python3` is on PATH.
"""

import fcntl
import fnmatch
import hashlib
import json
import os
import re
import socket
import subprocess
import sys
import time

DIR = "docs/plans/ros-alignment"
INDEX = f"{DIR}/README.md"
FILE_RE = re.compile(r"^(3\.(\d+)([a-z]?))-(.+)\.md$")
BRANCH_RE = re.compile(r"^(?:origin/)?plan/(3\.\d+[a-z]?)-")
STEP_ID_RE = re.compile(r"3\.\d+[a-z]?\.[A-Za-z0-9_-]+")
SECTION_ID_RE = re.compile(r"3\.\d+[a-z]?")
WAITS = ("user", "rover")
STATES = ("open", "done", "dropped")
CLOSED = ("done", "dropped")
ACTIVE_S = 2 * 3600          # a section branch committed within this is "active"
CLAIM_REF = "refs/heads/claim/"
STALE_S = 24 * 3600          # another host's claim reads "claimed (stale?)" after this quiet
NET_TIMEOUT_S = 20           # one call to origin
START_TIMEOUT_S = 5          # the session-start board's calls to origin
BYPASS = "PICAR_CLAIM_BYPASS"
CLAIM_CMD = "python tools/plan_section.py claim"
MSG_KEYS = ("step", "section", "title", "session", "worktree", "branch", "host", "pid", "start", "at")


# ---------- git and processes ----------

def git(*args, cwd=None, check=True) -> str:
    r = subprocess.run(["git", *args], capture_output=True, text=True, cwd=cwd)
    if check and r.returncode != 0:
        raise SystemExit(f"git {' '.join(args)}: {r.stderr.strip()}")
    return r.stdout.strip()


def net(args, cwd, timeout=NET_TIMEOUT_S):
    """A git call that talks to origin: never prompts, never hangs past
    `timeout`. (returncode, stdout, stderr); 124 on a timeout."""
    try:
        r = subprocess.run(["git", *args], capture_output=True, text=True, cwd=cwd, timeout=timeout,
                           env={**os.environ, "GIT_TERMINAL_PROMPT": "0"}, stdin=subprocess.DEVNULL)
        return r.returncode, r.stdout, r.stderr.strip()
    except subprocess.TimeoutExpired:
        return 124, "", f"no answer from origin in {timeout} s"


def repo(cwd=None):
    """(worktree top, shared .git dir), both absolute; None outside a repo."""
    r = subprocess.run(["git", "rev-parse", "--show-toplevel", "--path-format=absolute",
                        "--git-common-dir"], capture_output=True, text=True, cwd=cwd)
    if r.returncode != 0:
        return None
    lines = r.stdout.strip().split("\n")
    return (lines[0], lines[1]) if len(lines) == 2 else None


def _digest(prefix: str, text: str) -> str:
    return prefix + hashlib.sha256(text.encode()).hexdigest()[:12]


def host() -> str:
    """This machine in a claim (PICAR_CLAIM_HOST: tests). Origin is a PUBLIC
    repository, so it is a digest, never the host name; and it includes the
    process-id namespace, so two containers sharing a host name (cloud
    sessions) are two machines, each unable to see the other's processes
    (3.139 review). Stable across reboots: no boot id."""
    forced = os.environ.get("PICAR_CLAIM_HOST")
    if forced:
        return forced
    try:
        ns = str(os.stat("/proc/self/ns/pid").st_ino)
    except OSError:
        ns = ""
    return _digest("h-", f"{socket.gethostname()}|{ns}")


def session_id() -> str:
    """This Claude Code session, as a digest of the id Claude Code exports
    to its tools (published on a public origin: never the id itself)."""
    raw = os.environ.get("CLAUDE_CODE_SESSION_ID") or os.environ.get("CLAUDE_SESSION_ID") or ""
    return _digest("s-", raw) if raw else ""


def bypassed() -> bool:
    """The person's escape from the hooks. An agent never sets it."""
    return os.environ.get(BYPASS) == "1"


def section_of(step: str) -> str:
    m = SECTION_ID_RE.match(step or "")
    return m.group(0) if m else ""


def processes() -> dict:
    """{pid: (ppid, start, command basename)} from one `ps` call."""
    # A fixed zone and locale: lstart is a rendered local time, and a caller
    # with another TZ or LC_TIME must read the same start for the same process.
    out = subprocess.run(["ps", "-A", "-o", "pid=,ppid=,lstart=,comm="], capture_output=True,
                         text=True, env={**os.environ, "TZ": "UTC", "LC_ALL": "C"}).stdout
    procs = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 8:
            continue
        procs[int(parts[0])] = (int(parts[1]), " ".join(parts[2:7]),
                                os.path.basename(" ".join(parts[7:])))
    return procs


def owner(procs=None):
    """{"pid", "start", "session", "host"} of the Claude Code session this
    process runs under. PICAR_CLAIM_PID names another owner (tests; a person
    in a terminal), whose session is PICAR_CLAIM_SESSION, not this one's."""
    procs = procs if procs is not None else processes()
    forced = os.environ.get("PICAR_CLAIM_PID")
    if forced:
        return owner_of(int(forced), procs, os.environ.get("PICAR_CLAIM_SESSION", ""))
    pid = os.getpid()
    for _ in range(64):
        if pid not in procs or pid <= 1:
            return None
        ppid, start, name = procs[pid]
        if name == "claude":
            return {"pid": pid, "start": start, "session": session_id(), "host": host()}
        pid = ppid
    return None


def owner_of(pid: int, procs=None, session: str = ""):
    """The owner record for a given process, or None if it is not running."""
    procs = procs if procs is not None else processes()
    return ({"pid": pid, "start": procs[pid][1], "session": session, "host": host()}
            if pid in procs else None)


def alive(claim: dict, procs: dict) -> bool:
    """The claim's process still runs -- the same process, not a reused PID.
    Only meaningful for a claim made on this host."""
    p = procs.get(claim.get("pid"))
    return bool(p) and p[1] == claim.get("start")


def same_process(claim: dict, me) -> bool:
    return (bool(me) and (claim.get("host") or host()) == (me.get("host") or host())
            and claim.get("pid") == me["pid"] and claim.get("start") == me["start"])


def same_owner(claim: dict, me) -> bool:
    """The session when both carry one (3.139: one `claude` process -- or
    a resumed one -- is not one owner; a session whose machine changed its
    name is still itself), else the process on one host. A claim made
    before 3.139 carries no host or session: this host, by pid."""
    if not me:
        return False
    cs, ms = claim.get("session") or "", me.get("session") or ""
    if cs and ms:
        return cs == ms
    if (claim.get("host") or host()) != (me.get("host") or host()):
        return False
    return claim.get("pid") == me["pid"] and claim.get("start") == me["start"]


def _here(claim: dict) -> bool:
    return (claim.get("host") or host()) == host()


def held(claim: dict, procs: dict, me) -> bool:
    """A claim that still holds: its owner is this session, or it is
    another host's (never judged from here), or its process on this host
    still runs -- even when that is this very process under another session
    (review: two conversations in one process must not take each other's
    claims; an earlier conversation's claim is freed by the user)."""
    return same_owner(claim, me) or not _here(claim) or alive(claim, procs)


def who(c: dict) -> str:
    """The holder, as a reader needs it."""
    s = c.get("session") or ""
    ident = f"session {s[:8]}" if s else f"pid {c.get('pid', '?')}"
    wt = os.path.basename(str(c.get("worktree") or "")) or "?"
    since = c.get("at") or c.get("claimed_at") or "?"
    return f"{ident} in {wt} on {c.get('host') or host()}, since {since}"


# ---------- claims on origin (3.139) ----------

def _clean(v) -> str:
    return re.sub(r"[\r\n]+", " ", str(v if v is not None else "")).strip()


def claim_message(c: dict) -> str:
    """The claim commit's message: a subject and one `key: value` line per
    field, readable on GitHub and parsed by `parse_claim_message`. Origin is
    public: the worktree is its folder name only, host and session are
    digests (`host()`, `session_id()`)."""
    pub = {**c, "worktree": os.path.basename(str(c.get("worktree") or "").rstrip("/"))}
    return f"claim {_clean(c['step'])}\n\n" + "\n".join(f"{k}: {_clean(pub.get(k))}" for k in MSG_KEYS)


def parse_claim_message(text: str) -> dict:
    out = {}
    for line in text.split("\n")[1:]:
        k, sep, v = line.partition(": ")
        if sep and k in MSG_KEYS:
            out[k] = v.strip()
    try:
        out["pid"] = int(out.get("pid", ""))
    except ValueError:
        out["pid"] = None
    return out


def _commits(top: str, shas: list) -> dict:
    """{sha: (committer time, message)} for commits present locally."""
    if not shas:
        return {}
    r = subprocess.run(["git", "cat-file", "--batch"], input=("\n".join(shas) + "\n").encode(),
                       capture_output=True, cwd=top)
    out, data, i = {}, r.stdout, 0
    for sha in shas:
        nl = data.index(b"\n", i)
        head = data[i:nl].decode()
        i = nl + 1
        if head.endswith("missing"):
            continue
        size = int(head.split()[2])
        raw = data[i:i + size].decode("utf-8", "replace")
        i += size + 1
        headers, _, msg = raw.partition("\n\n")
        ts = 0
        for h in headers.split("\n"):
            if h.startswith("committer "):
                try:
                    ts = int(h.rsplit(" ", 2)[-2])
                except (ValueError, IndexError):
                    pass
        out[sha] = (ts, msg)
    return out


def remote_claims(top: str, timeout: float = NET_TIMEOUT_S):
    """([claim, ...], None) for every `claim/*` branch on origin, read with
    one `ls-remote` (the holders of claims not seen before are fetched as
    objects, through `<ref>:` refspecs that store no ref); ([], reason)
    when origin cannot be read."""
    code, out, err = net(["ls-remote", "origin", CLAIM_REF + "*"], top, timeout)
    if code:
        return [], err or f"git ls-remote exited {code}"
    refs = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[1].startswith(CLAIM_REF):
            refs[parts[1][len(CLAIM_REF):]] = parts[0]
    have = _commits(top, list(refs.values()))
    missing = [s for s, sha in refs.items() if sha not in have]
    if missing:
        net(["fetch", "-q", "--no-tags", "--no-write-fetch-head", "origin",
             *[f"{CLAIM_REF}{s}:" for s in missing]], top, timeout)
        have.update(_commits(top, [refs[s] for s in missing]))
    claims = []
    for step, sha in sorted(refs.items()):
        ts, msg = have.get(sha, (0, ""))
        c = parse_claim_message(msg) if msg else {}
        c.update({"step": step, "section": section_of(step), "sha": sha, "remote": True,
                  "committed": ts})
        c.setdefault("host", "?")        # unreadable: someone else's, held
        claims.append(c)
    unread = [c["step"] for c in claims if c["host"] == "?"]
    return claims, (f"could not read the holder of {', '.join(unread)}" if unread else None)


def _identity_args(top: str) -> list:
    if git("config", "user.email", cwd=top, check=False):
        return []
    return ["-c", "user.name=plan-claims", "-c", "user.email=plan-claims@localhost"]


def _claim_commit(top: str, c: dict) -> str:
    tree = subprocess.run(["git", "hash-object", "-t", "tree", "-w", "--stdin"], cwd=top, input="",
                          capture_output=True, text=True, check=True).stdout.strip()
    r = subprocess.run(["git", *_identity_args(top), "commit-tree", tree, "-m", claim_message(c)],
                       cwd=top, capture_output=True, text=True)
    if r.returncode:
        raise SystemExit(f"claim: could not make the claim commit: {r.stderr.strip()}")
    return r.stdout.strip()


def _push_create(top: str, step: str, sha: str):
    """Create `claim/<step>` on origin only if it does not exist (an empty
    lease). (ok, rejected, message)."""
    ref = CLAIM_REF + step
    code, _, err = net(["push", "-q", f"--force-with-lease={ref}:", "origin", f"{sha}:{ref}"], top)
    return code == 0, code != 0 and _rejected(err), err


def _push_delete(top: str, step: str, sha: str):
    """Delete `claim/<step>` on origin only if it still holds `sha`.
    (ok, rejected, message)."""
    ref = CLAIM_REF + step
    code, _, err = net(["push", "-q", f"--force-with-lease={ref}:{sha}", "origin", f":{ref}"], top)
    return code == 0, code != 0 and _rejected(err), err


def _rejected(err: str) -> bool:
    """Origin answered and refused (the lease), as opposed to not answering."""
    e = err.lower()
    return "stale info" in e or "[rejected]" in e or "already exists" in e or "remote ref does not exist" in e


# ---------- paths ----------

def _is_glob(p: str) -> bool:
    return any(c in p for c in "*?[")


def matches(path: str, pattern: str) -> bool:
    """A repo path falls under a Touches entry: the same file, a file inside a
    directory entry (trailing `/`, or a plain path with files under it), or a
    glob match."""
    path = path.strip("/")
    if _is_glob(pattern):
        return fnmatch.fnmatch(path, pattern) or fnmatch.fnmatch(path, pattern.rstrip("/") + "/*")
    p = pattern.strip("/")
    return path == p or path.startswith(p + "/")


def overlap(a: list, b: list) -> list:
    """The pairs of Touches entries that can name the same file."""
    def lit(p):                    # a glob's literal part: "sim/maps/*.py" -> "sim/maps/";
        return re.split(r"[*?\[]", p, maxsplit=1)[0]   # "tools/inventory_*" -> "tools/inventory_"

    def under(plain, glob):        # files below a directory entry (trailing `/`) the glob can
        if not plain.endswith("/"):    # match: `*` crosses `/`, so "brain/*.py" covers
            return False               # "brain/survey/x.py"
        return (plain.strip("/") + "/").startswith(lit(glob))

    hits = []
    for x in a:
        for y in b:
            # Two globs share a path only if one's literal part begins the other's.
            if (matches(x, y) or matches(y, x)
                    or (_is_glob(x) and _is_glob(y)
                        and (lit(x).startswith(lit(y)) or lit(y).startswith(lit(x))))
                    or (not _is_glob(x) and _is_glob(y) and under(x, y))
                    or (_is_glob(x) and not _is_glob(y) and under(y, x))):
                hits.append((x, y))
    return hits


# ---------- the steps ----------

def _cells(line: str) -> list:
    return [c.strip() for c in line.strip().strip("|").split("|")]


def _items(cell: str) -> tuple:
    """(entries, stray text). A cell that says only "nothing" (empty, a
    dash, "none", "no" or "n/a": `_no_entry`) has no entries. Otherwise,
    once any entry is ticked, the ticked paths are the entries; in a plain
    cell, the comma-separated paths are. Everything else -- text beside the
    paths, a word such as "no" or "none", an entry with a space or `;` in
    it, a ticked list -- is stray text, in the author's order, which
    `assess` shows rather than guess at (3.76 J8: a plain entry beside ticked
    ones was dropped without a word; rounds 14-25). Dashes are dropped."""
    found = re.findall(r"`([^`]+)`", cell)
    if not found:
        parts = [x.strip() for x in cell.split(",")]
        if all(_no_entry(x) for x in parts):
            return [], ""
        entries = [x for x in parts if not _no_entry(x) and not re.search(r"\s|;", x)]
        text = [x for x in parts if x not in entries and not _no_entry(x, words=False)]
        return entries, ", ".join(text)
    pieces = re.split(r"(`[^`]+`)", cell)                  # plain, ticked, plain, ...
    plain = [t for p in pieces[::2] for t in re.split(r"[,;]", p)]
    if all(_no_entry(x) for x in found + plain):
        return [], ""                                      # the whole cell says "nothing"
    entries, text = [], ""
    for i, p in enumerate(pieces):
        if i % 2:                                          # ticked
            x = p[1:-1].strip()
            if _no_entry(x, words=False):
                continue
            if _no_entry(x) or re.search(r"[\s,;]", x):   # a word, or "`a.py, sim/`": a list
                text += f" {x} "
            else:
                entries.append(x)
                text += " , "                              # where a path was: a break
        else:                                              # the author's text and separators
            text += "".join(t for t in re.split(r"([,;])", p) if not (t.strip() and _no_entry(t, words=False)))
    text = re.sub(r"\s+", " ", text)
    text = re.sub(r"\s*([,;])(\s*[,;])*\s*", r"\1 ", text)   # one separator, as written first
    return entries, text.strip(" ,;")


def _word(cell: str, allowed: tuple):
    """The cell's first word if it is one of `allowed` and stands alone
    (markup around it, then the end, a space, a bracket or a full stop):
    `**done**`, `` `rover` ``, `done (sweep met)`; not `done?`, `done/dropped`,
    `done-ish` or `~~open~~ done`."""
    w = re.match(r"[*_`]*([a-z]+)[*_`]*(?=$|[\s(.,;:])", cell.strip().lower())
    return w.group(1) if w and w.group(1) in allowed else None


def _state(cell: str) -> str:
    """The State cell's first word, so `done (sweep met; ...)` is done. A cell
    that starts with anything but open, done or dropped keeps its text, and
    `assess` never offers that step."""
    return _word(cell, STATES) or (cell.strip().lower() or "open")


def _waits(cell: str) -> str:
    """`user` or `rover` by the same rule as State; empty or `--` is no
    wait; any other text is kept, and `assess` shows it (round 13: a ticked
    or annotated `rover` read as no wait, and the step as free)."""
    cell = cell.strip().lower()
    if _no_entry(cell):
        return ""
    return _word(cell, WAITS) or "?" + cell


def _no_entry(cell: str, words: bool = True) -> bool:
    """Empty, or a dash, "none", "no" or "n/a" (with `words=False`: a dash
    only), ticked or in balanced bold/italic (round 14: a ticked `--` or an
    em dash read as unknown text, and the step was never free)."""
    # Bold or italic around the word only: a bare `*` or `**` is a glob (the
    # step touches everything), never "no entry" (round 19).
    word = r"(?:-|--|\u2013|\u2014|none|no|n/a)" if words else r"(?:-|--|\u2013|\u2014)"
    return re.fullmatch(rf"[`\s]*(?:(\*\*|\*|__|_){word}\1|{word})?[`\s]*",   # balanced (round 20)
                        cell, re.IGNORECASE) is not None


def parse_steps(text: str, section: str) -> list:
    """The rows of a section file's `## Steps` table."""
    steps, inside = [], False
    for line in text.split("\n"):
        if line.startswith("## "):
            inside = line.strip().lower() == "## steps"
            continue
        if not inside or not line.lstrip().startswith("|"):
            continue
        c = _cells(line)
        if len(c) < 6 or not STEP_ID_RE.fullmatch(c[0]):
            continue
        waits = _waits(c[3])
        touches, stray = _items(c[4])
        steps.append({
            "id": c[0], "section": section, "title": c[1],
            "needs": STEP_ID_RE.findall(c[2]),
            "waits": waits,
            "touches": touches,
            "stray": stray,
            "state": _state(c[5]),
        })
    return steps


def _cat(root: str, specs: list) -> dict:
    """{"ref:path": text} for many blobs in one `git cat-file --batch`."""
    if not specs:
        return {}
    r = subprocess.run(["git", "cat-file", "--batch"], input=("\n".join(specs) + "\n").encode(),
                       capture_output=True, cwd=root)
    out, data, i = {}, r.stdout, 0
    for spec in specs:
        nl = data.index(b"\n", i)
        head = data[i:nl].decode()
        i = nl + 1
        if head.endswith("missing"):
            continue
        size = int(head.split()[2])
        out[spec] = data[i:i + size].decode("utf-8", "replace")
        i += size + 1
    return out


def worktrees(root: str) -> dict:
    """{branch: worktree path} for every checked-out branch."""
    out, path = {}, None
    for line in git("worktree", "list", "--porcelain", cwd=root).splitlines():
        if line.startswith("worktree "):
            path = line[9:]
        elif line.startswith("branch refs/heads/") and path:
            out[line[18:]] = path
    return out


def dirty(wt: str):
    """(newest mtime, paths) of a worktree's uncommitted changes to TRACKED
    files (staged new files count). Untracked files do not: a finished
    section's worktree may still collect logs."""
    r = subprocess.run(["git", "status", "--porcelain"], capture_output=True, text=True, cwd=wt)
    newest, paths = 0.0, []
    for line in r.stdout.splitlines():
        if line.startswith("??"):
            continue
        rel = line[3:].split(" -> ")[-1].strip('"')
        paths.append(rel.rstrip("/") + ("/" if rel.endswith("/") else ""))
        try:
            newest = max(newest, os.stat(os.path.join(wt, rel)).st_mtime)
        except OSError:
            pass
    return newest, paths


def section_branches(root: str) -> dict:
    """{section: [(branch, last activity, files changed, files committed)]} for every plan/*
    branch with commits not on origin/dev or uncommitted work in its worktree.
    Activity is the newer of its last commit and its newest uncommitted edit,
    so a session that has not committed yet still shows."""
    out = {}
    wts = worktrees(root)
    refs = git("for-each-ref", "--format=%(refname:short) %(committerdate:unix)",
               "refs/heads/plan/", cwd=root).splitlines()
    for line in refs:
        b, ts = line.rsplit(" ", 1)
        m = BRANCH_RE.match(b)
        if not m:
            continue
        files, last, committed = [], 0, []
        base = git("merge-base", "origin/dev", b, cwd=root, check=False)
        if base and git("rev-parse", b, cwd=root) != base:
            files = git("diff", "--name-only", base, b, cwd=root).splitlines()
            committed, last = list(files), int(ts)
        if b in wts and os.path.isdir(wts[b]):
            mt, paths = dirty(wts[b])
            files += [f for f in paths if f not in files]
            last = max(last, mt)
        if files:
            out.setdefault(m.group(1), []).append(
                (b, last, [f for f in files if f != INDEX], committed))
    return out


def _dev_moved(root: str, branch: str, path: str, dev_path: str = "") -> bool:
    """origin/dev changed `path` after `branch` forked, so the branch does not
    hold dev's copy: dev's table is read until the branch merges dev. A branch
    merged under other shas, or abandoned, stays "unmerged" for good, and its
    old table must not replace dev's (3.76 J6). Content, not commit times:
    this repo merges dev into a plan branch and then fast-forwards dev, so no
    commit records when a change reached dev (round 9). The cost: a live
    branch's new rows are hidden while dev has moved under it -- visible
    (`claim` says "no step"), and merging dev ends it. `dev_path` is the
    section's file on dev, checked too: a branch that renamed the file has
    no copy at dev's name to compare (round 10)."""
    base = git("merge-base", "origin/dev", branch, cwd=root, check=False)
    blob = lambda ref, p: git("rev-parse", "-q", "--verify", f"{ref}:{p}", cwd=root, check=False)
    if not blob(branch, path):
        return True                   # the branch deleted or renamed it: not its copy (round 8)
    return bool(base) and any(blob("origin/dev", p) != blob(base, p)
                              for p in {path, dev_path} - {""})


def load_steps(root: str, branches=None) -> list:
    """Every step in every section, each section read from its newest ref:
    the newest of its plan/* branches that changed the file and holds
    origin/dev's copy of it (`_dev_moved`), else origin/dev."""
    branches = branches if branches is not None else section_branches(root)
    files = {}                                         # section -> (ref, path)
    for f in git("ls-tree", "--name-only", f"origin/dev:{DIR}", cwd=root, check=False).splitlines():
        m = FILE_RE.match(f)
        if m:
            files[m.group(1)] = ("origin/dev", f"{DIR}/{f}")
    on_dev = {sec: path for sec, (_, path) in files.items()}
    for sec, bs in branches.items():
        for b, ts, _, committed in sorted(bs, key=lambda x: x[1]):   # the newest wins
            for f in committed:            # committed text only: a dirty file is not yet published
                m = FILE_RE.match(os.path.basename(f))
                if (f.startswith(DIR + "/") and m and m.group(1) == sec
                        and not _dev_moved(root, b, f, on_dev.get(sec, ""))):
                    files[sec] = (b, f)
    blobs = _cat(root, [f"{ref}:{path}" for ref, path in files.values()])
    for sec, bs in branches.items():                   # a branch with no section file yet
        if sec not in files:
            files[sec] = (bs[0][0], "")
    steps = []
    for sec in sorted(files, key=lambda s: tuple(int(x) for x in re.findall(r"\d+", s))):
        ref, path = files[sec]
        text = blobs.get(f"{ref}:{path}", "")
        found = parse_steps(text, sec)
        if found:
            steps += found
        elif sec in branches:                          # a section with no table yet
            bs = branches[sec]
            steps.append({
                "id": sec, "section": sec, "implicit": True,
                "title": (re.sub(r"^#+\s*3\.\d+[a-z]?\s*", "", text.split("\n", 1)[0])
                          or bs[0][0])[:70],
                "needs": [], "waits": "", "state": "open",
                "touches": sorted({f for _, _, ch, _ in bs for f in ch}),
            })
    return steps


# ---------- claims ----------

class ClaimRefused(SystemExit):
    """Another session holds it (or would collide): the step is not free.
    `rival`: this claim reached origin, then backed off from one that landed
    at the same moment (both may back off; a retry after a pause settles it)."""
    rival = False


def claims_dir(common: str) -> str:
    d = os.path.join(common, "plan-claims")
    os.makedirs(d, exist_ok=True)
    return d


def read_claims(common: str) -> list:
    d = os.path.join(common, "plan-claims")
    out = []
    for f in sorted(os.listdir(d)) if os.path.isdir(d) else []:
        if f.endswith(".json"):
            try:
                with open(os.path.join(d, f)) as fh:
                    c = json.load(fh)
                if isinstance(c, dict) and isinstance(c.get("step"), str):
                    out.append(c)
            except (OSError, ValueError):
                pass
    return out


def claim_section(c: dict) -> str:
    return c.get("section") or section_of(c.get("step", ""))


class _Lock:
    """One flock over every claim change, so check-then-write is atomic."""

    def __init__(self, common: str):
        self.path = os.path.join(claims_dir(common), ".lock")

    def __enter__(self):
        self.fh = open(self.path, "a")
        fcntl.flock(self.fh, fcntl.LOCK_EX)
        return self

    def __exit__(self, *exc):
        fcntl.flock(self.fh, fcntl.LOCK_UN)
        self.fh.close()


def active_branches(branches: dict, now: float) -> dict:
    """{section: [branch, ...]} whose last commit is within ACTIVE_S."""
    return {sec: [b for b, ts, _, _ in bs if now - ts < ACTIVE_S]
            for sec, bs in branches.items() if any(now - ts < ACTIVE_S for _, ts, _, _ in bs)}


def plan_times(root: str) -> dict:
    """{section: newest commit time} over its plan/* branches, local and
    origin's (3.139: what keeps another host's claim from reading stale)."""
    out = {}
    for line in git("for-each-ref", "--format=%(refname:short) %(committerdate:unix)",
                    "refs/heads/plan/", "refs/remotes/origin/plan/", cwd=root, check=False).splitlines():
        name, _, ts = line.rpartition(" ")
        m = BRANCH_RE.match(name)
        if m and ts.isdigit():
            out[m.group(1)] = max(out.get(m.group(1), 0), int(ts))
    return out


def claimed_why(c: dict, now: float, times=None) -> str:
    """Why a held step reads claimed. Another host's claim is never freed
    from here; after STALE_S with no commit on its claim branch or its
    section's plan branches it reads `(stale?)` -- and still only the user
    frees it."""
    base = f"claimed by {who(c)}"
    if c.get("remote") and not _here(c):
        newest = max(c.get("committed") or 0, (times or {}).get(claim_section(c), 0))
        if newest and now - newest > STALE_S:
            return (f"(stale?) {base}; no commit for {int((now - newest) // 3600)} h on its claim or "
                    f"section branches. Another host's claim is never freed by itself: only the user "
                    f"may free it (release --force {c['step']})")
        return base + " (another host: held until it is released)"
    return base


def assess(step: dict, steps: list, live: list, me, branches: dict, here_branch: str, now: float,
           accepted=None, times=None):
    """(status, reason) for one step from this session's point of view.
    status: done | dropped | mine | claimed | unknown | active | idle | waits | blocked |
    conflict | free."""
    if step["state"] in CLOSED:
        return step["state"], ""
    held_ = [c for c in live if c["step"] == step["id"]]
    if held_:
        others = [c for c in held_ if not same_owner(c, me)]
        theirs = [c for c in others if c.get("remote")] or (others if not any(
            same_owner(c, me) for c in held_) else [])
        if theirs:                               # origin's word first: it is the arbiter
            return "claimed", claimed_why(theirs[0], now, times)
        c = next(c for c in held_ if same_owner(c, me))
        return "mine", f"held by this session ({c.get('worktree', '')})"
    other = [c for c in live if claim_section(c) == step["section"] and not same_owner(c, me)]
    if other:
        c = other[0]
        return "claimed", (f"section {step['section']} is held by {who(c)} ({c['step']}): one "
                           "session per section, which owns its worktree, ports and ROS domain")
    act = active_branches(branches, now)
    mine_here = any(claim_section(c) == step["section"] for c in live if same_owner(c, me))
    if step.get("whole"):                         # a whole section: nobody else in it, nor active
        if step["section"] in act and not mine_here and here_branch not in act[step["section"]]:
            return "active", f"{', '.join(act[step['section']])} worked on in the last 2 h (no claim)"
        return "free", ""
    if step["state"] != "open":                   # after the claims: a held step stays its holder's
        return "unknown", f"State `{step['state'][:40]}` is not open, done or dropped"
    if step.get("stray"):
        return "unknown", (f"Touches has text that is not one path ({step['stray'][:40]}): "
                           "tick every path, one per comma")
    if step["waits"].startswith("?"):
        return "unknown", f"Waits `{step['waits'][1:41]}` is not user, rover or --"
    if step["section"] in act and not mine_here and here_branch not in act[step["section"]]:
        return "active", f"{', '.join(act[step['section']])} worked on in the last 2 h (no claim)"
    own = here_branch in [b for b, _, _, _ in branches.get(step["section"], [])]
    if step.get("implicit") and step["section"] not in act and not own:
        return "idle", "unmerged branch, idle over 2 h, no step table"
    if step["waits"]:
        return "waits", f"waits on the {step['waits']}"
    state = {s["id"]: s["state"] for s in steps}
    unmet = [n for n in step["needs"] if state.get(n) != "done"]
    if unmet:
        return "blocked", "needs " + ", ".join(unmet)
    for c in live:
        if same_owner(c, me):
            continue
        hits = overlap(step["touches"], c.get("touches", []))
        if hits:
            return "conflict", f"touches {hits[0][0]} -- claimed by {c['step']} ({who(c)})"
    for sec, bs in act.items():
        if (sec == step["section"] or here_branch in bs
                or any(claim_section(c) == sec for c in live)):
            continue                               # its own work, or covered by a claim
        files = sorted({f for b, ts, ch, _ in branches[sec] if b in bs for f in ch})
        hits = overlap(step["touches"], files)
        if hits and accepted is not None:
            accepted += [f"{bs[0]}: {y}" for _, y in hits]
        elif hits:
            return "conflict", f"touches {hits[0][1]} -- changed on active {bs[0]} " \
                               "(no claim there: checked by hand? claim --accept-overlap)"
    return "free", ""


class Board:
    def __init__(self, cwd=None, me=None, remote=True, timeout=NET_TIMEOUT_S):
        rp = repo(cwd)
        if not rp:
            raise SystemExit("not in a git repository")
        self.top, self.common = rp
        self.now = time.time()
        self.procs = processes()
        self.me = me or owner(self.procs)
        self.here = git("rev-parse", "--abbrev-ref", "HEAD", cwd=self.top, check=False)
        self.branches = section_branches(self.top)
        self.steps = load_steps(self.top, self.branches)
        self.times = plan_times(self.top)
        self.claims = read_claims(self.common)
        self.remote, self.remote_err = (remote_claims(self.top, timeout) if remote
                                        else ([], "not read"))
        self.split()

    def split(self):
        """live: every claim that holds, local and on origin; stale: claims
        on origin made on this host whose process has exited -- `claim`
        takes them over."""
        touches = {s["id"]: s["touches"] for s in self.steps}
        for c in self.remote:
            loc = next((x for x in self.claims if x["step"] == c["step"]
                        and x.get("sha") == c["sha"]), None)
            c["touches"] = (loc or {}).get("touches") or touches.get(c["step"], [])
        local = [c for c in self.claims if held(c, self.procs, self.me)]
        remote = [c for c in self.remote if held(c, self.procs, self.me)]
        self.stale = [c for c in self.remote if not held(c, self.procs, self.me)]
        self.live = local + remote

    def status(self, step, accepted=None):
        return assess(step, self.steps, self.live, self.me, self.branches, self.here, self.now,
                      accepted, self.times)

    def find(self, sid):
        return next((s for s in self.steps if s["id"] == sid), None)

    def whole(self, sid):
        """A whole section as one step (`claim 3.N`; what `new` takes): any
        section with a file on origin/dev or a plan/* branch."""
        if not SECTION_ID_RE.fullmatch(sid):
            return None
        known = (any(s["section"] == sid for s in self.steps) or sid in self.times
                 or any(BRANCH_RE.match(b) and BRANCH_RE.match(b).group(1) == sid for b in git(
                     "for-each-ref", "--format=%(refname:short)", "refs/heads/plan/",
                     cwd=self.top, check=False).splitlines()))
        if not known:
            return None
        return {"id": sid, "section": sid, "title": f"section {sid} (whole)", "needs": [],
                "waits": "", "touches": [], "stray": "", "state": "open", "whole": True}


# ---------- commands ----------

ORDER = ["mine", "claimed", "active", "free", "conflict", "blocked", "waits", "unknown", "idle"]


def cmd_board(show_all=False, short=False, cwd=None, b=None) -> str:
    b = b or Board(cwd)
    rows = []
    for s in b.steps:
        st, why = b.status(s)
        if st in CLOSED and not show_all:
            continue
        rows.append((st, s, why))
    ids = {s["id"] for s in b.steps}
    extra = {}
    for c in b.live:                          # whole sections, steps on branches not seen here
        if c["step"] not in ids:
            extra.setdefault(c["step"], c)
    for sid, c in sorted(extra.items()):
        mine = any(same_owner(x, b.me) for x in b.live if x["step"] == sid)
        title = "(whole section)" if SECTION_ID_RE.fullmatch(sid) else c.get("title", "")
        rows.append(("mine" if mine else "claimed", {"id": sid, "title": title},
                     f"held by this session ({c.get('worktree', '')})" if mine
                     else claimed_why(c, b.now, b.times)))
    rows.sort(key=lambda r: ORDER.index(r[0]) if r[0] in ORDER else 99)
    if short:
        rows = [r for r in rows if r[0] in ("mine", "claimed", "active", "free", "unknown")]
    lines = []
    if b.remote_err:
        lines.append(f"WARNING: origin unreachable ({b.remote_err}): claims on origin are not "
                     "shown, and `claim` will fail")
    for st, s, why in rows:
        label = "claimed (stale?)" if st == "claimed" and why.startswith("(stale?)") else st
        lines.append(f"{label:9} {s['id']:10} {s['title'][:60]}" + (f"\n{'':20} {why}" if why else ""))
    if len(lines) == (1 if b.remote_err else 0):
        lines.append("no open steps")
    return "\n".join(lines)


def cmd_next(cwd=None) -> str:
    b = Board(cwd)
    for s in b.steps:
        st, _ = b.status(s)
        if st == "mine":
            return f"this session already holds {s['id']}: {s['title']}"
    for s in b.steps:
        if not s.get("implicit") and b.status(s)[0] == "free":
            return (f"{s['id']}: {s['title']}\n  touches: {', '.join(s['touches']) or '--'}\n"
                    f"  take it: python tools/plan_section.py claim {s['id']}")
    return ("no free step: every open step is claimed, active, blocked, waiting or unknown "
            "(see `board`)")


def _utc() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def claim(sid: str, cwd=None, me=None, accept_overlap=False) -> dict:
    """Take a step for this session, or SystemExit with the reason
    (ClaimRefused when someone else holds it or it would collide).
    The claim is pushed to origin as `claim/<sid>`, create-only, BEFORE the
    local claim file is written: origin picks one winner across machines,
    and a claim that did not reach origin does not exist (3.139).
    accept_overlap: a file shared with an ACTIVE BRANCH THAT HOLDS NO CLAIM was
    checked by hand and is fine; it is recorded in the claim. It never
    overrides another session's claim."""
    b = Board(cwd, me)
    if not b.me:
        raise SystemExit("claim: no Claude Code process above this one (set PICAR_CLAIM_PID)")
    if b.remote_err:
        raise SystemExit(f"claim failed: cannot read the claims on origin ({b.remote_err}). A claim "
                         "is published on origin first, so none was taken (3.139)")
    s = b.find(sid) or b.whole(sid)
    if not s:
        raise SystemExit(f"claim: no step {sid} (steps are rows of a section's `## Steps` table, "
                         f"committed on origin/dev or its plan/* branch; a whole section is 3.N)")
    path = os.path.join(claims_dir(b.common), f"{sid}.json")
    with _Lock(b.common):
        b.claims = read_claims(b.common)              # re-read under the lock, against a
        b.procs = processes()                         # process list from now: an owner that
        b.split()                                     # started since the board is alive
        accepted = [] if accept_overlap else None
        st, why = b.status(s, accepted)
        if st not in ("free", "mine"):
            raise ClaimRefused(f"claim refused: {sid} is {st}" + (f" -- {why}" if why else ""))
        c = {"step": sid, "section": s["section"], "title": s["title"],
             "touches": s["touches"], "pid": b.me["pid"], "start": b.me["start"],
             "session": b.me.get("session") or "", "host": b.me.get("host") or host(),
             "worktree": b.top, "branch": b.here, "accepted_overlap": accepted or [],
             "claimed_at": time.strftime("%Y-%m-%dT%H:%M:%S"), "at": _utc()}
        ours = next((x for x in b.remote if x["step"] == sid and same_owner(x, b.me)), None)
        if ours:                                      # already published: the file follows origin
            local = next((x for x in b.claims if x["step"] == sid and same_owner(x, b.me)), None)
            if local and local.get("sha") == ours["sha"]:
                return local
            c.update({"sha": ours["sha"], "at": ours.get("at") or c["at"]})
            _write_claim(path, c)
            return c
        for x in b.stale:                             # take over: a lease on the sha we read, so
            if x["step"] == sid:                      # a claim made since is never deleted
                _push_delete(b.top, sid, x["sha"])
        sha = _claim_commit(b.top, c)
        ok, rejected, err = _push_create(b.top, sid, sha)
        if not ok:
            if rejected:
                now, _ = remote_claims(b.top)
                h = next((x for x in now if x["step"] == sid), None)
                mine_before = os.path.exists(path) and st == "mine"
                if mine_before:                       # a pre-3.139 local claim lost to origin's
                    os.remove(path)
                raise ClaimRefused(f"claim refused: {sid} is claimed on origin by "
                                   f"{who(h) if h else 'another session'}"
                                   + ("; this session's older local claim is dropped" if mine_before else ""))
            raise SystemExit(f"claim failed: could not push claim/{sid} to origin ({err}); "
                             "no claim taken")
        c["sha"] = sha
        # Origin arbitrates one ref name; the one-session-per-section rule
        # and Touches overlap span several. Read origin again: a rival that
        # landed meanwhile means this claim backs off (both may; neither
        # then holds twice) (review: two machines both won one section).
        after, err2 = remote_claims(b.top)
        procs = processes()
        rivals = [x for x in after if x["sha"] != sha and not same_owner(x, b.me)
                  and held(x, procs, b.me)
                  and (claim_section(x) == c["section"]
                       or overlap(c["touches"], (b.find(x["step"]) or {}).get("touches", [])))]
        if rivals:
            done, rejected, e = _push_delete(b.top, sid, sha)
            if not done and not rejected:
                raise SystemExit(f"claim: {rivals[0]['step']} was claimed on origin meanwhile by "
                                 f"{who(rivals[0])}, and withdrawing claim/{sid} failed ({e}): run "
                                 f"`python tools/plan_section.py release {sid}`")
            refused = ClaimRefused(f"claim refused: {sid} -- {rivals[0]['step']} was claimed on "
                                   f"origin meanwhile by {who(rivals[0])}; this claim was withdrawn")
            refused.rival = True
            raise refused
        if err2:                      # the exact ref is ours (origin said so); the cross-ref
            c["unchecked"] = err2     # check could not run: kept, and said
            print(f"claim: WARNING origin could not be read again to check for a rival in "
                  f"{c['section']} ({err2}); see `board`", file=sys.stderr)
        _write_claim(path, c)
    return c


def _write_claim(path: str, c: dict):
    """Swapped in whole: a reader never sees half a claim."""
    tmp = f"{path}.{os.getpid()}.tmp"
    with open(tmp, "w") as fh:
        json.dump(c, fh, indent=1)
    os.replace(tmp, path)


def release(sid=None, cwd=None, me=None, force=False) -> list:
    """Give back this session's claims (or one), on origin first, then the
    local file. Without a step only this session's own (3.139: no longer
    every claim of the same process, nor every one whose process reads
    dead). With a step, also one whose holder has exited on this host.
    force: anyone's -- the user's
    word only. Origin is deleted with a lease on the sha read, so a claim
    someone made since is never deleted."""
    rp = repo(cwd)
    if not rp:
        raise SystemExit("not in a git repository")
    if force and not sid:
        raise SystemExit("release --force names one step")
    top, common = rp
    gone = []
    with _Lock(common):
        procs = processes()           # under the lock: an owner started since is alive (J7)
        me = me or owner(procs)
        local = read_claims(common)
        remote, err = remote_claims(top)
        if err:
            raise SystemExit(f"release: cannot read the claims on origin ({err}); nothing released")
        for step in sorted({c["step"] for c in local + remote if not sid or c["step"] == sid}):
            loc = [c for c in local if c["step"] == step]
            rem = next((c for c in remote if c["step"] == step), None)
            if force:
                ok_l, ok_r = loc, rem
            else:
                ok_l = [c for c in loc if same_owner(c, me) or (sid and not held(c, procs, me))]
                ok_r = rem if rem and (same_owner(rem, me) or (sid and not held(rem, procs, me))) else None
                if sid and not ok_l and not ok_r:
                    h = rem or loc[0]
                    raise ClaimRefused(f"release refused: {sid} is held by {who(h)}. Only the user "
                                       f"frees another session's claim (release --force {sid})")
            if ok_r:
                done, rejected, e = _push_delete(top, step, ok_r["sha"])
                if not done and not rejected:
                    raise SystemExit(f"release: could not delete claim/{step} on origin ({e}); "
                                     f"released so far: {', '.join(gone) or 'nothing'}")
            for c in ok_l:
                try:
                    os.remove(os.path.join(claims_dir(common), f"{c['step']}.json"))
                except FileNotFoundError:
                    pass
            if ok_r or ok_l:
                gone.append(step)
    return gone


# ---------- hooks ----------

def foreign_claims_on(rel: str, common: str, procs: dict, me) -> list:
    """Live claims of other sessions covering rel, minus those whose owner the
    user let this session edit (`allow`)."""
    return [c for c in read_claims(common)
            if held(c, procs, me) and not same_owner(c, me)
            and not any(same_owner(a, me) for a in c.get("allowed", []))
            and any(matches(rel, t) for t in c.get("touches", []))]


def quick_steps(top: str, ref: str = "origin/dev", here: bool = True) -> list:
    """The steps the hooks fence, cheaply: every section table at `ref`,
    with this checkout's committed table for its own plan/3.N-* section."""
    files = {}
    for f in git("ls-tree", "--name-only", f"{ref}:{DIR}", cwd=top, check=False).splitlines():
        m = FILE_RE.match(f)
        if m:
            files[m.group(1)] = (ref, f"{DIR}/{f}")
    if here:
        m = BRANCH_RE.match(git("rev-parse", "--abbrev-ref", "HEAD", cwd=top, check=False))
        if m:
            for f in git("ls-tree", "--name-only", f"HEAD:{DIR}", cwd=top, check=False).splitlines():
                fm = FILE_RE.match(f)
                if fm and fm.group(1) == m.group(1):
                    files[m.group(1)] = ("HEAD", f"{DIR}/{f}")
    blobs = _cat(top, [f"{r}:{p}" for r, p in files.values()])
    steps = []
    for sec, (r, p) in files.items():
        steps += parse_steps(blobs.get(f"{r}:{p}", ""), sec)
    return steps


# Shared status, edited by whoever merges (PARALLEL-SESSIONS.md rule 3):
# never fenced by a step's Touches (another session's live claim still asks).
SHARED = ("CLAUDE.md", "docs/ARCHITECTURE.md", INDEX, "docs/plans/PLAN-ros-alignment.md")


def tracked(rel: str, steps: list):
    """(sections, step to claim) a path belongs to: a section's own file,
    or a path in an open step's Touches. ({}, "") for any other path, and
    for the shared status files (SHARED)."""
    m = FILE_RE.match(os.path.basename(rel))
    if m and os.path.dirname(rel) == DIR:
        return {m.group(1)}, m.group(1)
    if rel in SHARED:
        return set(), ""
    done = {s["id"] for s in steps if s["state"] == "done"}
    # Only a step someone could be working on fences: open, waiting on no
    # one, its Needs done (review: waiting and blocked steps fenced tests/,
    # control/ and web-twin/ while nobody could claim them).
    hits = [s for s in steps if s["state"] not in CLOSED and not s["waits"]
            and all(n in done for n in s["needs"]) and any(matches(rel, t) for t in s["touches"])]
    return {s["section"] for s in hits}, (hits[0]["id"] if hits else "")


def covered(rel: str, secs: set, mine: list, steps=()) -> bool:
    """This session may change rel: it holds a claim in a section rel
    belongs to, or rel is in the Touches of its own claim or of any step of
    a section it holds (`steps`: two sections both declare the file)."""
    held_secs = {claim_section(c) for c in mine}
    return (bool(held_secs & secs)
            or any(matches(rel, t) for c in mine for t in c.get("touches", []))
            or any(matches(rel, t) for s in steps if s["section"] in held_secs for t in s["touches"]))


def _refusal(rel: str, secs: set, hint: str) -> str:
    where = ("its section file" if not STEP_ID_RE.fullmatch(hint) or FILE_RE.match(os.path.basename(rel))
             else f"in the Touches of open step {hint}")
    return (f"{rel} belongs to section {', '.join(sorted(secs))} ({where}), and this session holds "
            f"no claim there. Claim first: `{CLAIM_CMD} {hint}` (`next` names a free step). If the "
            f"claim is refused, stop and report it (PARALLEL-SESSIONS.md rule 12).")


def allow(sid: str, cwd=None, me=None) -> dict:
    """The user agreed this session may edit inside another session's claim:
    recorded in that claim, for as long as it lives."""
    if not (STEP_ID_RE.fullmatch(sid) or SECTION_ID_RE.fullmatch(sid)):
        raise SystemExit(f"allow: {sid!r} is not a step id")
    rp = repo(cwd)
    if not rp:
        raise SystemExit("not in a git repository")
    path = os.path.join(claims_dir(rp[1]), f"{sid}.json")
    with _Lock(rp[1]):
        procs = processes()           # under the lock, as claim() and release()
        me = me or owner(procs)
        if not me:
            raise SystemExit("allow: no Claude Code process above this one")
        try:
            with open(path) as fh:
                c = json.load(fh)
        except (OSError, ValueError):
            raise SystemExit(f"allow: no claim on {sid}")
        if not alive(c, procs):
            raise SystemExit(f"allow: {sid}'s owner has exited; the claim is free")
        if not any(same_owner(a, me) for a in c.get("allowed", [])):
            c.setdefault("allowed", []).append({**me, "at": time.strftime("%Y-%m-%dT%H:%M:%S")})
        _write_claim(path, c)
    return c


# Modes in which Claude Code shows the user an "ask"; in the others (auto,
# bypassPermissions, ...) an "ask" never reaches a person, so the guard denies.
ASK_MODES = ("default", "plan", "acceptEdits")


def _decision(decision: str, reason: str) -> str:
    return json.dumps({"hookSpecificOutput": {
        "hookEventName": "PreToolUse", "permissionDecision": decision,
        "permissionDecisionReason": reason}})


def hook_edit(stdin_text: str) -> str:
    """PreToolUse on Edit/Write/NotebookEdit. Inside another live session's
    claim: ask the user -- or, where no one would see the question, deny
    with the reason. A section's file, or a path in an open step's Touches,
    with no claim of this session in that section: deny, naming the claim
    command (3.139). '' means no opinion; any failure is no opinion."""
    try:
        data = json.loads(stdin_text)
        ti = data.get("tool_input") or {}
        path = ti.get("file_path") or ti.get("notebook_path")
        if not path:
            return ""
        path = os.path.abspath(os.path.join(data.get("cwd") or os.getcwd(), path))
        d = os.path.dirname(path)
        while d and not os.path.isdir(d):
            d = os.path.dirname(d)
        rp = repo(d)
        if not rp:
            return ""
        rel = os.path.relpath(os.path.realpath(path), os.path.realpath(rp[0]))
        if rel.startswith(".."):
            return ""
        procs = processes()
        me = owner(procs)
        hits = foreign_claims_on(rel, rp[1], procs, me)
        if hits:
            c = hits[0]
            reason = (f"{rel} is inside step {c['step']} ({c.get('title', '')[:60]}), claimed by "
                      f"another live Claude Code session ({who(c)}). "
                      "Editing it here may collide with that session (PARALLEL-SESSIONS.md rule 8).")
            if data.get("permission_mode", "") in ASK_MODES:
                return _decision("ask", reason)
            return _decision("deny", reason + " Ask the user before going on; only if they agree, run "
                             f"`python3 tools/plan_claims.py allow {c['step']}` and retry.")
        if bypassed():
            return ""
        steps = quick_steps(rp[0])
        secs, hint = tracked(rel, steps)
        if not secs:
            return ""
        mine = [c for c in read_claims(rp[1]) if same_owner(c, me)
                or (held(c, procs, me) and any(same_owner(a, me) for a in c.get("allowed", [])))]
        if covered(rel, secs, mine, steps):
            return ""
        return _decision("deny", _refusal(rel, secs, hint))
    except Exception:                                  # a guard must never break editing
        return ""


def commit_check(cwd=None):
    """pre-commit: (warnings, refusal). A warning for each staged file
    inside another live claim; a refusal for a commit on a plan/3.N-*
    branch when this session holds no claim in 3.N (3.139) -- except a
    merge (MERGE_HEAD), and under the person's PICAR_CLAIM_BYPASS=1."""
    try:
        rp = repo(cwd)
        if not rp:
            return [], ""
        procs = processes()
        me = owner(procs)
        out = []
        if os.path.isdir(os.path.join(rp[1], "plan-claims")):
            for f in git("diff", "--cached", "--name-only", cwd=rp[0], check=False).splitlines():
                for c in foreign_claims_on(f, rp[1], procs, me):
                    out.append(f"pre-commit: WARNING {f} is inside step {c['step']}, claimed by "
                               f"{who(c)}")
        if bypassed():
            return out, ""
        branch = git("rev-parse", "--abbrev-ref", "HEAD", cwd=rp[0], check=False)
        m = BRANCH_RE.match(branch)
        if not m:
            return out, ""
        merge = git("rev-parse", "--git-path", "MERGE_HEAD", cwd=rp[0], check=False)
        if merge and os.path.exists(os.path.join(rp[0], merge)):
            return out, ""
        sec = m.group(1)
        if any(claim_section(c) == sec for c in read_claims(rp[1]) if same_owner(c, me)):
            return out, ""
        return out, (f"pre-commit: REFUSED -- {branch} is section {sec}'s branch and this session "
                     f"holds no claim in {sec} (PARALLEL-SESSIONS.md rule 12). Claim first: "
                     f"`{CLAIM_CMD} <step>` (`next` names one; the whole section: `{CLAIM_CMD} {sec}`). "
                     "If it is refused, stop and report it. (A person, never an agent, may "
                     f"override with {BYPASS}=1.)")
    except Exception:
        return [], ""


def hook_commit(cwd=None) -> str:
    """The warnings alone (3.75's interface)."""
    return "\n".join(commit_check(cwd)[0])


def push_refusals(base: str, sha: str, cwd=None, me=None) -> list:
    """For the pre-push gate (3.139): each file a push to dev changes that
    is a section's file, or a path in an open step's Touches (as of
    `base`), of a section this session holds no claim in. [] under the
    person's PICAR_CLAIM_BYPASS=1."""
    if bypassed():
        return []
    rp = repo(cwd)
    if not rp:
        return []
    top, common = rp
    r = subprocess.run(["git", "diff", "--name-only", "--no-renames", base, sha], cwd=top,
                       capture_output=True, text=True)
    if r.returncode:              # origin's dev moved since the fetch: fail closed
        return [f"origin's dev ({base[:9]}) is not in this clone: git fetch origin, merge or "
                "rebase, and push again"]
    files = r.stdout.splitlines()
    procs = processes()
    me = me or owner(procs)
    steps = quick_steps(top, ref=base, here=False)
    fenced = [(f, *tracked(f, steps)) for f in files]
    fenced = [x for x in fenced if x[1]]
    if not fenced:
        return []
    # A claim counts only while origin still holds it as this session's
    # (review: a claim freed with --force, or taken over, kept passing).
    remote, err = remote_claims(top)
    if err:
        return [f"cannot read the claims on origin ({err}), so this session's claims cannot be "
                "checked; push again"]
    on_origin = {(c["step"], c["sha"]) for c in remote}
    local = [c for c in read_claims(common) if same_owner(c, me)]
    mine = [c for c in local if (c["step"], c.get("sha")) in on_origin]
    pushed = quick_steps(top, ref=sha, here=False)      # the pusher's own tables, as pushed
    out = []
    for c in local:
        if c not in mine:
            out.append(f"this session's claim of {c['step']} is not on origin (made before 3.139, "
                       f"or freed there): re-run `{CLAIM_CMD} {c['step']}`")
    for f, secs, hint in fenced:
        if not covered(f, secs, mine, pushed):
            out.append(f"{f}: section {', '.join(sorted(secs))}"
                       + (f" (open step {hint})" if STEP_ID_RE.fullmatch(hint) else " (its file)")
                       + f" -- claim it: {CLAIM_CMD} {hint}")
    return out


def hook_start(cwd=None) -> str:
    """The board for a new conversation, and what this session (or an
    earlier conversation of this process) already holds."""
    try:
        b = Board(cwd, timeout=START_TIMEOUT_S)      # a session start never waits long
        board = cmd_board(short=True, b=b)
        mine = [c for c in b.live if same_owner(c, b.me)]
        earlier = [c for c in b.claims + b.stale
                   if not same_owner(c, b.me) and same_process(c, b.me)]
    except BaseException:
        return ""
    seen, head = set(), ""
    for c in mine:
        if c["step"] not in seen:
            seen.add(c["step"])
            head += (f"THIS SESSION ALREADY HOLDS {c['step']} ({c.get('title', '')[:60]}; worktree "
                     f"{c.get('worktree', '?')}). Continue it there, or give it back: "
                     f"`python3 tools/plan_claims.py release {c['step']}`.\n")
    for c in earlier:
        if c["step"] not in seen:
            seen.add(c["step"])
            head += (f"ANOTHER CONVERSATION OF THIS CLAUDE PROCESS HOLDS {c['step']} "
                     f"({c.get('title', '')[:60]}). It is not this session's: if that conversation "
                     f"is over, ask the user to free it (`release --force {c['step']}`).\n")
    return (head + "Plan steps (PARALLEL-SESSIONS.md rules 8 and 12). Claim before working on the "
            "plan: `python tools/plan_section.py next`, then `claim <step>`; if refused, stop and "
            "report. Board:\n" + board)


def main(argv):
    if not argv:
        raise SystemExit(__doc__)
    cmd, args = argv[0], argv[1:]
    if cmd == "board":
        print(cmd_board(show_all="--all" in args, short="--short" in args))
    elif cmd == "next":
        print(cmd_next())
    elif cmd == "claim" and len([a for a in args if a != "--accept-overlap"]) == 1:
        c = claim([a for a in args if a != "--accept-overlap"][0],
                  accept_overlap="--accept-overlap" in args)
        print(f"claimed {c['step']} for {who(c)}: {c['title']}\n"
              f"  on origin: claim/{c['step']} ({c.get('sha', '')[:9]})\n"
              f"  touches: {', '.join(c['touches']) or '--'}"
              + "".join(f"\n  accepted overlap: {x}" for x in c.get("accepted_overlap", [])))
    elif cmd == "release" and len([a for a in args if a != "--force"]) <= 1:
        rest = [a for a in args if a != "--force"]
        gone = release(rest[0] if rest else None, force="--force" in args)
        print("released " + (", ".join(gone) if gone else "nothing"))
    elif cmd == "allow" and len(args) == 1:
        allow(args[0])
        print(f"this session may now edit inside {args[0]} (recorded in its claim)")
    elif cmd == "hook-edit":
        out = hook_edit(sys.stdin.read())
        if out:
            print(out)
    elif cmd == "hook-commit":
        warnings, refusal = commit_check()
        for w in warnings:
            print(w, file=sys.stderr)
        if refusal:
            print(refusal, file=sys.stderr)
            sys.exit(1)
    elif cmd == "hook-start":
        print(hook_start())
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main(sys.argv[1:])
