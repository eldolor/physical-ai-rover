# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
import sys
from playwright.sync_api import sync_playwright
out = sys.argv[1]
with sync_playwright() as p:
    for scheme in ("dark", "light"):
        b = p.chromium.launch()
        pg = b.new_page(viewport={"width": 390, "height": 844}, color_scheme=scheme)
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        reqs = []
        pg.on("request", lambda r: reqs.append(r.method + " " + r.url))
        pg.goto("https://d114x92g7i4syl.cloudfront.net/lab", wait_until="networkidle")
        pg.wait_for_timeout(1500)
        pg.screenshot(path=f"{out}/lab.{scheme}.png", full_page=False)
        sw = pg.evaluate("document.documentElement.scrollWidth")
        print(scheme, "title:", pg.title(), "scrollWidth:", sw, "errors:", errs)
        print("  api requests:", [r for r in reqs if "/recording/" in r or "/metrics/" in r])
        b.close()
