"""3.132 Stage 1 criterion 10 (red first): the active "Guide me" / "Robot
view" button must be readable. On `dev` before Stage 1 it was the accent blue
on the safe green (`button.active-mode` gives the fill,
`.robot-mode-btn.active-mode` the text colour): 1.39:1 dark, 1.02:1 light --
written before the fix and seen to fail.

The background is read as the browser computes it, including Chromium's
`color(srgb r g b / a)` for a `color-mix()` fill (the review found the first
parser skipped those and measured against the panel instead).

`UI_SHOTS=<dir>` keeps the phone-size screenshots.
"""

import os

import pytest

pytest.importorskip("playwright")

from tests.test_ui import browser, open_twin, twin_server  # noqa: F401,E402

CONTRAST = r"""(sel) => {
  const el = document.querySelector(sel);
  // rgb()/rgba() in 0-255, or color(srgb ...) in 0-1 (a color-mix() fill).
  const parse = c => {
    let m = c.match(/^rgba?\(([^)]+)\)/);
    if (m) { const p = m[1].split(/[ ,/]+/).filter(Boolean).map(Number);
             return {r: p[0], g: p[1], b: p[2], a: p.length > 3 ? p[3] : 1}; }
    m = c.match(/^color\(srgb ([^)]+)\)/);
    if (m) { const p = m[1].split(/[ /]+/).filter(Boolean).map(Number);
             return {r: p[0] * 255, g: p[1] * 255, b: p[2] * 255, a: p.length > 3 ? p[3] : 1}; }
    throw new Error("unparsed colour: " + c);
  };
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
    page, errors = open_twin(browser, twin_server, engineer=False)
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
