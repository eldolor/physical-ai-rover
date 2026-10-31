"""
evaluations/ux-3132/pixels.py -- 3.132 Stage 1 criterion 2: dark-theme
screenshots of the twin's static states, for a pixel comparison across a
commit that must change nothing visible (the pure move of the <style> block).

    python evaluations/ux-3132/pixels.py shoot <dir>
    python evaluations/ux-3132/pixels.py compare <dirA> <dirB>

Reuses measure.py's servers, stubs and setup, so the states are the same.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import measure as M  # noqa: E402

LIVE = ("#brain-tel-watchdog", "#brain-tel-refusal", "#header-conn-dot .dot", "#btn-settings")
STATES = ("twin.first_run", "twin.guide", "twin.guide_robot_view", "twin.sim_idle", "twin.settings")


def shoot(out: Path):
    from playwright.sync_api import sync_playwright
    out.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(M.REPO / "tools"))
    import desktop_lock
    desktop_lock.hold_for_life("exclusive", "3.132 pixel comparison")
    with M.servers(out / "logs"):
        with sync_playwright() as p:
            b = p.chromium.launch()

            def snap(page, name):
                page.wait_for_timeout(600)
                page.wait_for_function(M.JS_SETTLED, timeout=10_000)
                # Live content, not style: the watchdog's seconds tick between
                # shots and the status dot can be caught mid-transition. Masked
                # (painted over in place), so the layout is still compared.
                live = [page.locator(sel) for sel in LIVE]
                page.screenshot(path=str(out / f"{name}.png"), full_page=True,
                                animations="disabled", mask=live, mask_color="#FF00FF")

            ctx, page, _ = M.new_twin_page(b, configured=False)
            snap(page, "twin.first_run")
            ctx.close()
            M.reset_robot_and_brain()
            ctx, page, _ = M.new_twin_page(b)
            snap(page, "twin.guide")
            page.click("#btn-mode-robot")
            snap(page, "twin.guide_robot_view")
            page.click("#btn-mode-guide")
            page.click('.tab-btn[data-tab="sim"]')
            page.wait_for_timeout(1500)
            snap(page, "twin.sim_idle")
            page.click("#btn-settings")
            snap(page, "twin.settings")
            ctx.close()
            b.close()


def compare(a: Path, b: Path) -> int:
    from PIL import Image, ImageChops
    bad = 0
    for name in STATES:
        ia, ib = Image.open(a / f"{name}.png").convert("RGB"), Image.open(b / f"{name}.png").convert("RGB")
        if ia.size != ib.size:
            print(f"{name}: size {ia.size} vs {ib.size}")
            bad += 1
            continue
        box = ImageChops.difference(ia, ib).getbbox()
        print(f"{name}: {'identical' if box is None else f'differs in {box}'}")
        bad += box is not None
    return bad


if __name__ == "__main__":
    if sys.argv[1] == "shoot":
        shoot(Path(sys.argv[2]))
    else:
        sys.exit(1 if compare(Path(sys.argv[2]), Path(sys.argv[3])) else 0)
