# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.137.B's B5: 3.132's bars in the twin's three battery states, and the
existing twin states before and after the change.

Built on `evaluations/ux-3132/measure.py` (its servers, page set-up and
probes, imported unchanged): the robot server and brain on this section's
ports (sim), the twin served as deployed, at 390 x 844. The battery states
are the robot server's real `GET /battery` reply with its `level` and
`ladder` replaced by a Playwright route (the live check, `twin_live.py`,
drains a real pack); measured on the Guide tab in operator mode, the view a
person at home opens.

    eval "$(python tools/plan_section.py env)"
    python evaluations/battery-3137/twin_states.py --out evaluations/battery-3137/ux-after

Writes <out>/results.json and <out>/shots/. Holds the desktop's exclusive
lock (measure.py's timed windows), as measure.py does.
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / "evaluations" / "ux-3132"))
sys.path.insert(0, str(REPO))

import measure as m  # noqa: E402
from playwright.sync_api import sync_playwright  # noqa: E402

LEVELS = {
    "low": {"v": 10.43, "level": "low", "ladder": {"level": "low", "poweroff_in_s": None,
                                                    "reserve_s": None, "checking": None,
                                                    "shutdown": [], "poweroff_why": None}},
    "reserve": {"v": 10.18, "level": "reserve",
                "ladder": {"level": "reserve", "poweroff_in_s": 87.4, "reserve_s": 32.6,
                           "checking": None, "shutdown": [], "poweroff_why": None}},
    "poweroff": {"v": 9.98, "level": "poweroff",
                 "ladder": {"level": "poweroff", "poweroff_in_s": None, "reserve_s": 41.0,
                            "checking": None, "poweroff_why": "at rest",
                            "shutdown": [{"step": "wheels", "t_s": 0.0, "rc": 0},
                                         {"step": "sync", "t_s": 0.1, "rc": 0},
                                         {"step": "poweroff", "t_s": 0.1, "rc": None,
                                          "stub": True}]}},
}


def stub_battery(page, which):
    """The real reply, its ladder replaced (volts and levels kept as served
    otherwise)."""
    def handle(route):
        resp = route.fetch()
        body = resp.json()
        b = LEVELS[which]
        body.update({"usable": True, "v": b["v"], "level": b["level"]})
        body["ladder"] = {**(body.get("ladder") or {}), **b["ladder"], "v": b["v"]}
        route.fulfill(response=resp, body=json.dumps(body))
    page.route("**/battery", handle)


def battery_states(browser, axe, shots):
    states = {}
    for which in LEVELS:
        ctx = None
        try:
            ctx, page, _ = m.new_twin_page(browser)
            stub_battery(page, which)
            page.goto(m.TWIN)
            m.wait_twin_ready(page)
            page.wait_for_timeout(7000)          # B1's 6 s, and a poll or two more
            m.measure_into(states, page, f"twin.battery_{which}", axe, shots)
            states[f"twin.battery_{which}"]["banner"] = page.evaluate(
                "() => { const el = document.getElementById('battery-banner');"
                " return el ? {shown: el.checkVisibility(), text: el.innerText} : null; }")
        except Exception as e:  # noqa: BLE001
            states[f"twin.battery_{which}"] = {"error": repr(e)[:300]}
        finally:
            if ctx is not None:
                ctx.close()
        m.reset_robot_and_brain()
    return states


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", required=True, type=Path)
    args = ap.parse_args()
    out = args.out.resolve()
    shots = out / "shots"
    shots.mkdir(parents=True, exist_ok=True)
    axe = m.axe_source()
    sys.path.insert(0, str(REPO / "tools"))
    import desktop_lock
    desktop_lock.hold_for_life("exclusive", "3.137.B twin battery states (timed page windows)")
    commit = subprocess.run(["git", "rev-parse", "--short=9", "HEAD"], cwd=REPO,
                            capture_output=True, text=True).stdout.strip()
    result = {"commit": commit, "viewport": m.PHONE, "axe": m.AXE_VERSION, "states": {}}
    with m.servers(out / "logs"):
        with sync_playwright() as p:
            browser = p.chromium.launch(args=["--use-fake-ui-for-media-stream",
                                              "--use-fake-device-for-media-stream"])
            try:
                s, _ = m.twin_states(browser, axe, shots)
                result["states"].update(s)
                result["states"].update(battery_states(browser, axe, shots))
            finally:
                browser.close()
    (out / "results.json").write_text(json.dumps(result, indent=1, sort_keys=True))
    summ = m.summary({**result, "tasks": {}, "M6_count": {}})["states"]
    for k, v in result["states"].items():
        if "banner" in v:
            summ[k]["banner"] = v["banner"]
    print(json.dumps(summ, indent=1))


if __name__ == "__main__":
    main()
