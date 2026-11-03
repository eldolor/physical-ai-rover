"""3.132 Stage 2B: the twin's modules are published, served and only declare.

web-twin/app.js is the entry module, and code moves out of it into
web-twin/js/. Three ways that goes wrong quietly:

* A module the browser imports is missing from service/static/assets.json,
  so CloudFront 404s it. A failed import stops the whole module graph,
  app.js included, so the deployed page is dead while the local one works
  (criterion 5).
* A module imports app.js, or its top level does something. Either would
  change the order statements run in, which the move rule exists to keep
  (criterion 6). The browser half of that check (a module imported in a
  Web Worker, with no page to touch) is in tests/test_ui_modules.py.
* A file assigns a name it imports. Imports are read-only, so that is a
  TypeError, and only when that line runs (criterion 6).
"""

import json
import re

import pytest

from tests.conftest import REPO_ROOT
from tests.twin_sources import (APP_JS, DYNAMIC, MODULE_DIR, REEXPORT, TWIN, imported_names,
                                imports, module_graph, modules, specs, twin_scripts)


def manifest() -> dict:
    data = json.loads((REPO_ROOT / "service" / "static" / "assets.json").read_text())
    return {a["key"]: a for a in data["assets"]}


def unpublished(graph, published) -> list:
    """Modules in `graph` with no correct assets.json entry."""
    bad = []
    for path in graph:
        key = path.relative_to(TWIN).as_posix()
        entry = published.get(key)
        if entry is None:
            bad.append(f"{key}: not in assets.json")
        elif entry["src"] != f"web-twin/{key}":
            bad.append(f"{key}: published from {entry['src']}")
        elif entry["type"] != "application/javascript":
            bad.append(f"{key}: type {entry['type']} (a module must be JavaScript)")
        elif entry["cache"] != "none":
            bad.append(f"{key}: cache {entry['cache']}")
    return bad


def test_every_module_app_js_reaches_is_published():
    """Criterion 5. Followed from app.js, as the browser follows it."""
    assert not unpublished(modules(), manifest())


def test_every_module_lives_in_the_module_folder_and_is_reached():
    """A file outside web-twin/js/ has no route; a file in it that nothing
    imports is dead code that still ships."""
    graph = modules()
    for path in graph:
        assert path.parent == MODULE_DIR.resolve(), f"{path}: modules live in web-twin/js/"
    orphans = sorted(p.name for p in MODULE_DIR.glob("*.js") if p.resolve() not in graph)
    assert not orphans, f"modules nothing imports: {orphans}"


def test_every_import_is_relative():
    """Keys mirror the reference paths, and every reference is relative
    (tests/test_static_assets.py); a bare or rooted specifier breaks under
    the teleop prefix or does not resolve at all without a bundler."""
    for path in twin_scripts():
        for spec in specs(path.read_text()):
            assert spec.startswith("./") and ".." not in spec and spec.endswith(".js"), \
                f"{path.name}: import {spec!r}"


def imports_app_js(path) -> bool:
    return any((path.parent / spec).resolve() == APP_JS.resolve()
               for spec in specs(path.read_text()))


def test_no_module_imports_app_js():
    """Criterion 6: app.js is the top. A module importing it would run
    app.js's body first, before that module's own declarations."""
    bad = [p.name for p in MODULE_DIR.glob("*.js") if imports_app_js(p)]
    assert not bad, f"modules that import app.js: {bad}"


LOOP_HEAD = r"\bfor\s*\(\s*%s\s+(?:of|in)\b"
TARGET_END = re.compile(r"[\]}](?=\s*(?:=(?![=>])|(?:of|in)\b))")


def destructuring_targets(code: str) -> list:
    """The bracketed spans that are written to: `[...] = `, `{...} = `,
    `for ([...] of`, nested and with defaults; a subscript (`a[i] = `) is
    not one. Run on the raw text, as every check here is: a comment or a
    string can only cause a visible false alarm, never hide a write. A
    declaration that shadows (`const [x] = ...`) is included: conservative,
    and a smell anyway."""
    spans = []
    for m in TARGET_END.finditer(code):
        close = m.start()
        opener = "[" if code[close] == "]" else "{"
        depth, i = 0, close
        while i >= 0:
            if code[i] in "]}":
                depth += 1
            elif code[i] in "[{":
                depth -= 1
                if depth == 0:
                    break
            i -= 1
        if i >= 0 and code[i] == opener:
            before = code[:i].rstrip()
            if opener == "[" and before and (before[-1].isalnum() or before[-1] in "_$)]"):
                continue  # a subscript: a[i] = ..., f()[i] in o
            spans.append(code[i:close + 1])
    return spans

ASSIGN = r"(?<![\w$.])%s\s*(?:\*\*|<<|>>>|>>|&&|\|\||\?\?|[-+*/%%&|^])?=(?![=>])"
# `x++`, `x ++;`, `--x`, `-- x`; not `++x.count` (a property), and not
# prose's "found -- x" or "x -- don't": a postfix step must end its
# expression, and a prefix one must not follow a word.
STEP = (r"(?<![\w$.])%s\s*(?:\+\+|--)(?=\s*(?:[;,)\]}]|$))"
        r"|(?<![\w$)\]])(?<![\w$)\]] )(?:\+\+|--)\s*%s(?![\w$.\[])")


def writes_to_imports(text: str) -> list:
    """Names `text` imports and also assigns."""
    hits = []
    targets = destructuring_targets(text)
    for clause, _spec in imports(text):
        for name in imported_names(clause):
            n = re.escape(name)
            if (re.search(ASSIGN % n, text) or re.search(STEP % (n, n), text, re.M)
                    or re.search(LOOP_HEAD % n, text)
                    or any(re.search(r"(?<![\w$.])%s(?![\w$])(?!\s*:)" % n, t) for t in targets)):
                hits.append(name)
    return hits


def test_no_file_assigns_a_name_it_imports():
    """Criterion 6. An import is a read-only live binding."""
    bad = {p.name: writes_to_imports(p.read_text()) for p in twin_scripts()}
    bad = {k: v for k, v in bad.items() if v}
    assert not bad, f"files that assign an imported name: {bad}"


def test_the_checks_catch_what_they_are_for(tmp_path):
    """Each check above, on a planted violation."""
    # An import the manifest does not list.
    planted = MODULE_DIR / "planted.js"
    assert unpublished([planted], manifest()) == ["js/planted.js: not in assets.json"]
    assert unpublished([planted], {"js/planted.js": {
        "src": "web-twin/js/planted.js", "type": "text/plain", "cache": "none"}})
    # The import parser, on every shape the twin may use.
    text = ('import "./js/a.js";\nimport { b, c as d } from "./js/b.js";\n'
            'import {\n  e,\n  f,\n} from "./js/c.js";\nimport * as g from "./js/d.js";\n')
    assert [s for _c, s in imports(text)] == ["./js/a.js", "./js/b.js", "./js/c.js", "./js/d.js"]
    assert [n for c, _s in imports(text) for n in imported_names(c)] == ["b", "d", "e", "f", "g"]
    # A module importing app.js.
    mod = tmp_path / "js" / "m.js"
    mod.parent.mkdir()
    mod.write_text('import { x } from "../app.js";\n')
    (tmp_path / "app.js").write_text("")
    assert any((mod.parent / s).resolve() == (tmp_path / "app.js").resolve()
               for _c, s in imports(mod.read_text()))
    # Writes to an import, and the reads and look-alikes that are not.
    for bad in ("x = 1", "x += 1", "x++", "--x", "x ??= y", "x ++;", "x --)", "-- x;",
                "f(++ x)", "x++\n", "[x] = y", "[a, x] = y",
                "({ x } = o)", "for (x of list) {}", "for (x in o) {}", "[x = 1] = arr",
                "({ x = 0 } = o)", "({ a: { x } } = o)", "[a, [x]] = y", "for ([x, y] of p) {}",
                "for ({ x } of list) {}", "({ a: x } = o)", "({a:x} = o)"):
        assert writes_to_imports('import { x } from "./js/a.js";\n' + bad) == ["x"], bad
    for fine in ("x === 1", "y = x", "o.x = 1", "f(x)", "xy = 1", "{ x: 1 }", "x => 1",
                 "[x].map(f)", "const o = { x };", "f([x])", "[x] === y", "for (const y of x) {}",
                 "({ x: 1 } = o)", "const o = { a: x }; o.b = 1", "arr[x] = 1",
                 "if (map[x] in seen) {}", "f()[x] = 2", "++x.count", "// found x -- done", "// x -- don't", "a - -x"):
        assert writes_to_imports('import { x } from "./js/a.js";\n' + fine) == [], fine


def test_no_twin_script_imports_at_run_time():
    """A dynamic import() is loaded at run time, so the walk above cannot
    follow it and its file could ship unpublished. The twin has none."""
    bad = [p.name for p in twin_scripts() if DYNAMIC.search(p.read_text())]
    assert not bad, f"dynamic imports: {bad}"
    assert DYNAMIC.search('const re = /"/g; const m = await import("./js/x.js");')
    assert not DYNAMIC.search("importScripts(a); reimport(b); o.import(c)")
    assert REEXPORT.findall('export { a } from "./b.js";\nexport * from "./c.js";') == ["./b.js", "./c.js"]


def upload_order(rows) -> list:
    """Order faults in assets.json rows (upload order, sync.sh): a twin
    module before what it imports or after app.js, or index.html before
    a twin script or sheet."""
    keys = [r["key"] for r in rows]
    bad = []
    mods = [p.relative_to(TWIN).as_posix() for p in modules()]
    missing = [k for k in mods + ["app.js", "index.html"] if k not in keys]
    if missing:
        return [f"{k}: not in assets.json" for k in missing]
    for path, key in zip(modules(), mods):
        for spec in specs(path.read_text()):
            dep = (path.parent / spec).resolve().relative_to(TWIN.resolve()).as_posix()
            if keys.index(dep) > keys.index(key):
                bad.append(f"{key} goes up before {dep}, which it loads")
        if keys.index(key) > keys.index("app.js"):
            bad.append(f"{key} goes up after app.js, which imports it")
        # index.html preloads every module: one not yet uploaded is a 403
        # that CloudFront caches for seconds.
        if keys.index(key) > keys.index("index.html"):
            bad.append(f"{key} goes up after index.html, which preloads it")
    # The module tag must be live before app.js has an import line in it.
    if keys.index("index.html") > keys.index("app.js"):
        bad.append("index.html goes up after app.js, which it must load as a module")
    return bad


def test_the_upload_order_puts_what_is_loaded_before_what_loads_it():
    """Both reviewers: sync.sh puts objects in manifest order, so a new
    app.js uploaded before a new module imports a key not yet there; one
    failed import stops the whole graph and the page is dead until a
    reload. And index.html first (review round 2): an old page loads app.js
    as a classic script, where an import line is a SyntaxError. Seen to
    fail on the order this stage first wrote."""
    rows = json.loads((REPO_ROOT / "service" / "static" / "assets.json").read_text())["assets"]
    assert not upload_order(rows)

    def moved(key, to):
        row = next(r for r in rows if r["key"] == key)
        out = [r for r in rows if r is not row]
        out.insert(to(out), row)
        return out

    index_at = lambda rows_: next(i for i, r in enumerate(rows_) if r["key"] == "index.html")
    app_at = lambda rows_: next(i for i, r in enumerate(rows_) if r["key"] == "app.js")
    # Planted: index.html after app.js; and, with modules, one after
    # index.html, one after app.js, and ui.js before icons.js, which it imports.
    assert upload_order(moved("index.html", lambda r: app_at(r) + 1))
    if modules():
        assert upload_order(moved("js/qr.js", lambda r: index_at(r) + 1))
        assert upload_order(moved("js/qr.js", lambda r: app_at(r) + 1))
        assert upload_order(moved("js/ui.js", lambda r: 0))


PRELOAD = re.compile(r'<link rel="modulepreload" href="([^"]+)">')


def test_the_page_preloads_exactly_the_modules_app_js_imports():
    """index.html's modulepreload list fetches the modules beside app.js;
    a stale entry preloads a dead file, a missing one costs a round trip."""
    html = (TWIN / "index.html").read_text()
    preloaded = sorted(PRELOAD.findall(html))
    assert preloaded == sorted(p.relative_to(TWIN).as_posix() for p in modules())


@pytest.mark.parametrize("path", modules(), ids=lambda p: p.name)
def test_the_robot_server_serves_every_module(path):
    """Criterion 5: served as JavaScript, revalidated, at the root."""
    from fastapi.testclient import TestClient

    from robot.server import create_app

    with TestClient(create_app()) as client:
        resp = client.get("/js/" + path.name)
        assert resp.status_code == 200
        assert "application/javascript" in resp.headers["content-type"]
        assert "no-cache" in resp.headers.get("cache-control", "")
        assert resp.text == path.read_text()


def test_the_module_route_serves_only_the_module_folder(tmp_path, monkeypatch):
    """Criterion 5, non-vacuous from the first commit: a planted module is
    served at the root and under the teleop prefix, and nothing else is."""
    from fastapi.testclient import TestClient

    import robot.server as server

    folder = tmp_path / "js"
    folder.mkdir()
    (folder / "planted.js").write_text("export const x = 1;\n")
    (folder / "link.js").symlink_to(tmp_path / "secret.js")
    (folder / "sub").mkdir()
    (folder / "sub" / "deep.js").write_text("export const z = 3;\n")
    (folder / "notes.txt").write_text("not a module\n")
    (tmp_path / "secret.js").write_text("export const y = 2;\n")
    monkeypatch.setattr(server, "_TWIN_JS_DIR", folder)
    for prefix in ("", "/teleop-robot"):
        monkeypatch.setenv("ROUTE_PREFIX", prefix)
        with TestClient(server.create_app()) as client:
            resp = client.get(prefix + "/js/planted.js")
            assert resp.status_code == 200, prefix
            assert "application/javascript" in resp.headers["content-type"]
            assert "no-cache" in resp.headers.get("cache-control", "")
            for name in ("missing.js", "notes.txt", "..%2Fsecret.js", "%2E%2E%2Fsecret.js",
                         "sub%2Fdeep.js", "sub", ".js", "planted.js%00.txt", "link.js"):
                assert client.get(prefix + "/js/" + name).status_code == 404, (prefix, name)
            assert client.get(prefix + "/js/../secret.js").status_code == 404, prefix
            # Looked up per request: a module added while the server runs.
            (folder / "later.js").write_text("export const w = 4;\n")
            assert client.get(prefix + "/js/later.js").status_code == 200, prefix
            (folder / "later.js").unlink()


EXPORTED = re.compile(r"^export\s+(?:async\s+)?(?:function\*?|const|let|var|class)\s+([A-Za-z_$][\w$]*)", re.M)


def exported_values(code: str) -> list:
    """Names a module exports that are not functions: state another file
    could reach into."""
    return [m.group(1) for m in EXPORTED.finditer(code)
            if not re.match(r"export\s+(?:async\s+)?function", m.group(0))]


def test_connstatus_exports_no_registry():
    """3.132 Stage 3B, criterion 8: `voicedStatus` (the voiced statuses kept
    for a voice switch) is connstatus.js's own again; app.js reaches it only
    through its functions. Stage 2B had exported the Map."""
    code = (MODULE_DIR / "connstatus.js").read_text()
    assert exported_values(code) == [], exported_values(code)
    assert not any("voicedStatus" in g for g in re.findall(r"export\s*\{([^}]*)\}", code))
    assert exported_values("export const voicedStatus = new Map();\nexport function f() {}\n") \
        == ["voicedStatus"]
