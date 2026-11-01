"""3.132 Stage 2A criterion 2: every toast's words come from
web-twin/strings.js, and its operator voice is plain.

* No `showToast(...)` in app.js passes a string literal as its message:
  the message is `t("key", {...})`, optionally followed by text a server
  sent (an error's message), which is data, not ours to word.
* strings.js holds its table as strict JSON between two markers, in two
  voices: `op` (the default, a person at home) and `eng` (Engineer mode,
  today's wording). Every `op` string -- placeholders like {target}
  removed -- passes the M5 deny-list the harness measures with
  (evaluations/ux-3132/measure.py JARGON, imported).
* Every key app.js asks for exists in both voices, and `eng` has no key
  `op` lacks.

No browser needed.
"""

import importlib.util
import json
import re

from tests.conftest import REPO_ROOT

# Every twin script, not only app.js: code moves into web-twin/js/ (3.132 Stage 2B).
from tests.twin_sources import twin_source  # noqa: E402
STRINGS_JS = REPO_ROOT / "web-twin" / "strings.js"
BEGIN, END = "/* BEGIN STRINGS (strict JSON) */", "/* END STRINGS */"


def _jargon():
    spec = importlib.util.spec_from_file_location(
        "ux_measure", REPO_ROOT / "evaluations" / "ux-3132" / "measure.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.JARGON


JARGON = _jargon()


def jargon_hits(text):
    out = []
    for label, rx in JARGON:
        for m in rx.finditer(text):
            out.append((label, (m.group(1) if m.groups() else m.group(0)).strip()))
    return out


def table():
    src = STRINGS_JS.read_text()
    a, b = src.index(BEGIN) + len(BEGIN), src.index(END)
    return json.loads(src[a:b])


def calls(src, name):
    """The argument text of every `name(` call, by matching brackets and
    skipping string literals and comments."""
    out, i = [], 0
    while True:
        i = src.find(name + "(", i)
        if i < 0:
            return out
        if i > 0 and (src[i - 1].isalnum() or src[i - 1] in "_$."):
            i += 1
            continue
        if re.match(r"\s*function\b", src[max(0, i - 9):i]):
            i += 1
            continue
        j, depth, q = i + len(name) + 1, 1, None
        while depth and j < len(src):
            c = src[j]
            if q:
                if c == "\\":
                    j += 1
                elif c == q:
                    q = None
            elif c in "\"'`":
                q = c
            elif c == "/" and src[j:j + 2] == "//":
                j = src.index("\n", j)
            elif c in "([{":
                depth += 1
            elif c in ")]}":
                depth -= 1
            j += 1
        out.append(src[i + len(name) + 1:j - 1])
        i = j


def first_arg(args):
    depth, q = 0, None
    for k, c in enumerate(args):
        if q:
            if c == "\\":
                continue
            if c == q:
                q = None
        elif c in "\"'`":
            q = c
        elif c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
        elif c == "," and depth == 0:
            return args[:k]
    return args


LITERAL = re.compile(r"(?<![\w$])(\"(?:[^\"\\]|\\.)*\"|'(?:[^'\\]|\\.)*'|`(?:[^`\\]|\\.)*`)")


def message_literals(arg):
    """String literals in a toast's message that are not the key of a t()
    call."""
    stripped = re.sub(r"\bt\(\s*(\"[^\"]*\"|'[^']*')", "t(", arg)
    keys = set(table()["op"])
    # A literal with no letters (" ", ": ", " -- ") joins our words to a
    # server's; it says nothing of its own.
    return [lit for lit in LITERAL.findall(stripped)
            if re.search(r"[A-Za-z]", lit[1:-1]) and lit[1:-1] not in keys]


def test_no_toast_message_is_a_literal():
    src = twin_source()
    found = calls(src, "showToast")
    assert len(found) >= 40, f"the parser found only {len(found)} showToast calls"
    bad = [first_arg(a).strip()[:90] for a in found if message_literals(first_arg(a))]
    assert not bad, f"{len(bad)} toasts word themselves in app.js, not strings.js: {bad[:6]}"


def test_the_operator_voice_is_plain():
    t = table()
    bad = {}
    for key, text in t["op"].items():
        hits = jargon_hits(re.sub(r"\{\w+\}", "", text))
        if hits:
            bad[key] = hits
    assert not bad, f"operator-voice strings with jargon: {bad}"


def test_every_key_is_in_both_voices_and_every_asked_key_exists():
    t = table()
    assert set(t["eng"]) <= set(t["op"]), sorted(set(t["eng"]) - set(t["op"]))
    asked = set(re.findall(r"\bt\(\s*[\"']([\w.]+)[\"']", twin_source()))
    # "action." + verb: a prefix, so some key must sit under it.
    missing = sorted(k for k in asked if (k not in t["op"]) if not
                     (k.endswith(".") and any(o.startswith(k) for o in t["op"])))
    assert not missing, f"app.js asks for keys strings.js lacks: {missing}"


def test_the_checks_catch_what_they_are_for():
    """Re-introduce the bugs."""
    assert message_literals('"Connected to the brain service."')
    assert message_literals('"Could not stop: " + e.message')
    assert not message_literals('t("brain.connected")')
    assert not message_literals('t("brain.stopFailed", {why: e.message})')
    assert not message_literals('t("x") + " " + e.message')
    assert jargon_hits("Mission arrived_unconfirmed")
    assert jargon_hits("vision service hung (B3.2)")
    assert not jargon_hits("Reached it, but could not double-check it.")


def test_the_nav2_hold_says_only_a_start():
    """3.134: since 3.133 the bridge refuses `nav2_starting` only during a
    start; a nav2 that restarts after being active is `ros_unavailable`. No
    voice may call the hold a restart. Red on dev d46c5173 (Engineer)."""
    for voice, words in table().items():
        text = words.get("goal.hold.nav2_starting", "")
        assert text and "restart" not in text.lower(), (voice, text)
