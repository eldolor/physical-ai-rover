"""The twin's scripts, for the tests that read its source (3.132 Stage 2B).

web-twin/app.js is the entry module; the code moved out of it lives in
web-twin/js/, one ES module a file. A test that reads "the twin's code"
reads every one of them, or it passes vacuously the day the code it looks
for moves. The module graph is followed the way the browser follows it:
from app.js, through each file's static `import` lines.
"""

import re
from pathlib import Path

from tests.conftest import REPO_ROOT

TWIN = Path(REPO_ROOT) / "web-twin"
APP_JS = TWIN / "app.js"
MODULE_DIR = TWIN / "js"

# A static import, one line or several: `import "./x.js";`,
# `import { a, b as c } from "./x.js";`, `import * as ns from "./x.js";`.
IMPORT = re.compile(r"""^[ \t]*import\s+(?:([^'";]*?)\s+from\s+)?["']([^"']+)["']""", re.M)


def imports(text: str) -> list:
    """(binding clause or "", specifier) for each static import in `text`."""
    return [(m.group(1) or "", m.group(2)) for m in IMPORT.finditer(text)]


def imported_names(clause: str) -> list:
    """The local names an import clause binds."""
    names = []
    clause = clause.strip()
    if not clause:
        return names
    braces = re.search(r"\{([^}]*)\}", clause)
    rest = re.sub(r"\{[^}]*\}", "", clause)
    if braces:
        for part in braces.group(1).split(","):
            part = part.strip()
            if part:
                names.append(part.split(" as ")[-1].strip())
    star = re.search(r"\*\s+as\s+([\w$]+)", rest)
    if star:
        names.append(star.group(1))
        rest = rest.replace(star.group(0), "")
    default = rest.strip().strip(",").strip()
    if default:
        names.append(default)
    return names


def module_graph() -> list:
    """Every file app.js reaches through its imports, app.js first."""
    seen, todo = [APP_JS], [APP_JS]
    while todo:
        path = todo.pop(0)
        for _clause, spec in imports(path.read_text()):
            target = (path.parent / spec).resolve()
            if target not in seen:
                seen.append(target)
                todo.append(target)
    return seen


def modules() -> list:
    """The modules app.js reaches, without app.js itself."""
    return module_graph()[1:]


def twin_scripts() -> list:
    """app.js and every module in web-twin/js/, reached or not."""
    return [APP_JS] + sorted(MODULE_DIR.glob("*.js"))


def twin_source() -> str:
    """The text of every twin script, for a test that searches the code."""
    return "\n".join(p.read_text() for p in twin_scripts())
