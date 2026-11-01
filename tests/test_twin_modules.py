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
from tests.twin_sources import (APP_JS, MODULE_DIR, TWIN, imported_names, imports,
                                module_graph, modules, twin_scripts)


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
        for _clause, spec in imports(path.read_text()):
            assert spec.startswith("./") and ".." not in spec and spec.endswith(".js"), \
                f"{path.name}: import {spec!r}"


def imports_app_js(path) -> bool:
    return any((path.parent / spec).resolve() == APP_JS.resolve()
               for _clause, spec in imports(path.read_text()))


def test_no_module_imports_app_js():
    """Criterion 6: app.js is the top. A module importing it would run
    app.js's body first, before that module's own declarations."""
    bad = [p.name for p in MODULE_DIR.glob("*.js") if imports_app_js(p)]
    assert not bad, f"modules that import app.js: {bad}"


ASSIGN = r"(?<![\w$.])%s\s*(?:\*\*|<<|>>>|>>|&&|\|\||\?\?|[-+*/%%&|^])?=(?![=>])"
STEP = r"(?<![\w$.])%s\s*(?:\+\+|--)|(?:\+\+|--)\s*%s(?![\w$])"


def writes_to_imports(text: str) -> list:
    """Names `text` imports and also assigns."""
    hits = []
    for clause, _spec in imports(text):
        for name in imported_names(clause):
            n = re.escape(name)
            if re.search(ASSIGN % n, text) or re.search(STEP % (n, n), text):
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
    for bad in ("x = 1", "x += 1", "x++", "--x", "x ??= y"):
        assert writes_to_imports('import { x } from "./js/a.js";\n' + bad) == ["x"], bad
    for fine in ("x === 1", "y = x", "o.x = 1", "f(x)", "xy = 1", "{ x: 1 }", "x => 1"):
        assert writes_to_imports('import { x } from "./js/a.js";\n' + fine) == [], fine


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
            for name in ("missing.js", "notes.txt", "..%2Fsecret.js", "%2E%2E%2Fsecret.js"):
                assert client.get(prefix + "/js/" + name).status_code == 404, (prefix, name)
            assert client.get(prefix + "/js/../secret.js").status_code == 404, prefix
