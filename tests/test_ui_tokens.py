"""3.132 Stage 1: one design system for the twin.

* Criterion 10 (red first): the active "Guide me" / "Robot view" button must
  be readable. On `dev` before Stage 1 it was the accent blue on the safe
  green (`button.active-mode` gives the fill, `.robot-mode-btn.active-mode`
  the text colour), about 1.1:1 -- written before the fix and seen to fail.
* Criterion 3: the twin's own sheet holds no colour literal; every colour
  is a token from web-shared/ui.css, where literals live only in custom
  property declarations.

`UI_SHOTS=<dir>` keeps the phone-size screenshots.
"""

import os
import re

import pytest

from tests.conftest import REPO_ROOT

TWIN_CSS = REPO_ROOT / "web-twin" / "twin.css"
UI_CSS = REPO_ROOT / "web-shared" / "ui.css"

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


# ---------- criterion 10: the active mode button is readable ----------

pytest.importorskip("playwright")

from tests.test_ui import browser, open_twin, twin_server  # noqa: F401,E402

CONTRAST = r"""(sel) => {
  const el = document.querySelector(sel);
  const parse = c => { const m = c.match(/rgba?\(([^)]+)\)/); if (!m) return null;
    const p = m[1].split(/[ ,/]+/).filter(Boolean).map(Number);
    return {r: p[0], g: p[1], b: p[2], a: p.length > 3 ? p[3] : 1}; };
  const lum = c => { const f = v => { v /= 255; return v <= 0.03928 ? v / 12.92 : ((v + 0.055) / 1.055) ** 2.4; };
    return 0.2126 * f(c.r) + 0.7152 * f(c.g) + 0.0722 * f(c.b); };
  // The effective background: the first opaque fill up the tree, blending
  // translucent ones over it.
  const layers = [];
  for (let n = el; n; n = n.parentElement) {
    const c = parse(getComputedStyle(n).backgroundColor);
    if (c && c.a > 0) { layers.push(c); if (c.a >= 1) break; }
  }
  let bg = {r: 255, g: 255, b: 255};
  for (let i = layers.length - 1; i >= 0; i--) {
    const c = layers[i];
    bg = {r: c.r * c.a + bg.r * (1 - c.a), g: c.g * c.a + bg.g * (1 - c.a), b: c.b * c.a + bg.b * (1 - c.a)};
  }
  const fg = parse(getComputedStyle(el).color);
  const L1 = lum(fg), L2 = lum(bg);
  return (Math.max(L1, L2) + 0.05) / (Math.min(L1, L2) + 0.05);
}"""


@pytest.mark.parametrize("scheme", ["dark", "light"])
def test_the_active_mode_button_is_readable(browser, twin_server, scheme):  # noqa: F811
    page, errors = open_twin(browser, twin_server)
    page.emulate_media(color_scheme=scheme, reduced_motion="reduce")
    for active in ("#btn-mode-robot", "#btn-mode-guide"):
        page.click(active)
        page.wait_for_function(f"() => document.querySelector('{active}').classList.contains('active-mode')")
        page.wait_for_timeout(100)
        ratio = page.evaluate(CONTRAST, active)
        assert ratio >= 4.5, f"{active} active in {scheme}: {ratio:.2f}:1 (AA needs 4.5)"
    if os.environ.get("UI_SHOTS"):
        os.makedirs(os.environ["UI_SHOTS"], exist_ok=True)
        page.screenshot(path=os.path.join(os.environ["UI_SHOTS"], f"active-mode-{scheme}.png"))
    assert errors == []
    page.close()
