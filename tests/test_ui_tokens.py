"""3.132 Stage 1: one design system for the twin.

No browser needed: these run where the UI tests skip. Criterion 10 (the
active mode button readable) is tests/test_ui_active_mode.py.

* Criterion 3: the twin's own sheet holds no colour literal; every colour
  is a token from web-shared/ui.css, where literals live only in custom
  property declarations.
"""

import re

from tests.conftest import REPO_ROOT

TWIN_CSS = REPO_ROOT / "web-twin" / "twin.css"
UI_CSS = REPO_ROOT / "web-shared" / "ui.css"
LAB_CSS = REPO_ROOT / "control" / "lab.css"
METRICS_JS = REPO_ROOT / "control" / "metrics.js"

# ---------- criterion 3: tokens only ----------

# CSS's named colours (CSS Color 4), so a page sheet cannot slip one in by
# name. `transparent` and `currentColor` are not palette colours and stay
# allowed.
NAMED = set("""
aliceblue antiquewhite aqua aquamarine azure beige bisque black blanchedalmond blue
blueviolet brown burlywood cadetblue chartreuse chocolate coral cornflowerblue cornsilk
crimson cyan darkblue darkcyan darkgoldenrod darkgray darkgreen darkgrey darkkhaki
darkmagenta darkolivegreen darkorange darkorchid darkred darksalmon darkseagreen
darkslateblue darkslategray darkslategrey darkturquoise darkviolet deeppink deepskyblue
dimgray dimgrey dodgerblue firebrick floralwhite forestgreen fuchsia gainsboro ghostwhite
gold goldenrod gray green greenyellow grey honeydew hotpink indianred indigo ivory khaki
lavender lavenderblush lawngreen lemonchiffon lightblue lightcoral lightcyan
lightgoldenrodyellow lightgray lightgreen lightgrey lightpink lightsalmon lightseagreen
lightskyblue lightslategray lightslategrey lightsteelblue lightyellow lime limegreen linen
magenta maroon mediumaquamarine mediumblue mediumorchid mediumpurple mediumseagreen
mediumslateblue mediumspringgreen mediumturquoise mediumvioletred midnightblue mintcream
mistyrose moccasin navajowhite navy oldlace olive olivedrab orange orangered orchid
palegoldenrod palegreen paleturquoise palevioletred papayawhip peachpuff peru pink plum
powderblue purple rebeccapurple red rosybrown royalblue saddlebrown salmon sandybrown
seagreen seashell sienna silver skyblue slateblue slategray slategrey snow springgreen
steelblue tan teal thistle tomato turquoise violet wheat white whitesmoke yellow
yellowgreen
""".split())
LITERAL = re.compile(r"#[0-9a-fA-F]{3,8}\b|\b(?:rgba?|hsla?|hwb|lab|lch|oklab|oklch)\s*\(")


def declarations(css):
    """(property, value) for every declaration, comments removed."""
    css = re.sub(r"/\*.*?\*/", "", css, flags=re.S)
    for m in re.finditer(r"([-\w]+)\s*:\s*([^;{}]+)", css):
        prop, value = m.group(1).strip(), m.group(2).strip()
        if prop.startswith("-") and not prop.startswith("--"):
            continue    # vendor prefixes still carry values; checked like any other
        yield prop, value


def colour_literals(value):
    hits = LITERAL.findall(value)
    hits += [w for w in re.findall(r"[a-zA-Z-]+", value) if w.lower() in NAMED]
    return hits


def test_the_twin_sheet_has_no_colour_literals():
    bad = [(p, v) for p, v in declarations(TWIN_CSS.read_text())
           if not p.startswith("--") and colour_literals(v)]
    assert not bad, f"web-twin/twin.css must use ui.css tokens, found literals: {bad[:8]}"


def test_the_lab_sheet_has_no_colour_literals():
    """3.132 Stage 4, criterion 2: the Lab console's own sheet uses ui.css's
    tokens, as the twin's does."""
    bad = [(p, v) for p, v in declarations(LAB_CSS.read_text())
           if colour_literals(v)]
    assert not bad, f"control/lab.css must use ui.css tokens, found literals: {bad[:8]}"


def test_the_metrics_charts_draw_in_tokens():
    """The charts' series colours were hex literals in metrics.js; they are
    read from the theme's tokens now, so they follow light and dark."""
    code = re.sub(r"/\*.*?\*/|//[^\n]*", "", METRICS_JS.read_text(), flags=re.S)
    assert not LITERAL.findall(code), LITERAL.findall(code)
    assert 'token("--accent-primary")' in code


def test_ui_css_keeps_literals_in_custom_properties_only():
    bad = [(p, v) for p, v in declarations(UI_CSS.read_text())
           if not p.startswith("--") and colour_literals(v)]
    assert not bad, f"web-shared/ui.css: a literal outside a token: {bad[:8]}"


def test_the_literal_check_catches_what_it_is_for():
    """Re-introduce the bug the two tests above guard against."""
    for bad in ("color: #6FA9F6", "background: rgba(0,0,0,0.5)", "border: 1px solid white",
                "color: hsl(10 50% 50%)", "fill: Teal"):
        p, v = next(declarations(bad))
        assert colour_literals(v), bad
    for fine in ("white-space: nowrap", "background: transparent", "color: currentColor",
                 "color: var(--text)", "border-color: var(--accent-primary)"):
        p, v = next(declarations(fine))
        assert not colour_literals(v), fine


def test_no_color_mix_in_the_sheets():
    """Safari before 16.2 and Chrome before 111 drop a color-mix()
    declaration whole -- the Guide edge glow, the found outline and the
    Robot-view zone tints vanished on an iOS 15 phone (3.132 Stage 1's bug
    pass). Tints are rgba tokens in ui.css instead."""
    for sheet in (TWIN_CSS, UI_CSS, LAB_CSS):
        css = re.sub(r"/\*.*?\*/", "", sheet.read_text(), flags=re.S)
        assert "color-mix(" not in css, sheet.name
