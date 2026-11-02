# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
# 3.132 deploy evidence: python shoot.py <out-dir>  (the deployed twin, 390x844, dark and light)
import json, sys
from playwright.sync_api import sync_playwright
URL = "https://d114x92g7i4syl.cloudfront.net/"
OUT = sys.argv[1]
report = {}
with sync_playwright() as p:
    b = p.chromium.launch()
    for scheme in ("dark", "light"):
        ctx = b.new_context(viewport={"width": 390, "height": 844}, device_scale_factor=2,
                            color_scheme=scheme, is_mobile=True, has_touch=True)
        page = ctx.new_page()
        errors, resp, failed = [], {}, []
        page.on("pageerror", lambda e: errors.append(str(e)))
        page.on("response", lambda r: resp.__setitem__(r.url.split("?")[0].replace(URL, ""), r.status))
        page.on("requestfailed", lambda r: failed.append(r.url))
        page.goto(URL, wait_until="networkidle")
        page.wait_for_timeout(1500)
        def shot(name):
            page.wait_for_timeout(600)
            page.screenshot(path=f"{OUT}/twin.{name}.{scheme}.png", full_page=True)
            page.screenshot(path=f"{OUT}/twin.{name}.{scheme}.fold.png")
        shot("guide")
        page.click(".tab-btn[data-tab=sim]"); shot("sim")
        page.click("#btn-settings"); shot("settings")
        page.locator("label[for=cfg-debug-readouts]").first.click()
        checked = page.is_checked("#cfg-debug-readouts")
        page.click("#btn-settings")
        page.click(".tab-btn[data-tab=sim]"); shot("engineer_sim")
        report[scheme] = {"page_errors": errors, "failed_requests": failed,
                          "engineer_checked": checked,
                          "assets": {k: resp.get(k) for k in ("", "strings.js", "app.js", "ui.css", "twin.css")},
                          "all_responses": resp}
        ctx.close()
    b.close()
json.dump(report, open(f"{OUT}/results.json", "w"), indent=2, sort_keys=True)
print(json.dumps({s: {k: v for k, v in r.items() if k != "all_responses"} for s, r in report.items()}, indent=1))
