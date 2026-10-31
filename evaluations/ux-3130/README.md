# evaluations/ux-3130 -- the UX harness (3.130)

`measure.py` measures the twin, the walks console and the metrics console at
a 390 x 844 phone viewport on 3.130's metrics (M1 taps per task, M2 axe-core,
M3 targets under 44 px, M4 sideways scroll, M5 jargon, M6 paid calls without
a click). The definitions, amendments and bars are in
`docs/plans/ros-alignment/3.130-ux-redesign.md`.

```bash
eval "$(python tools/plan_section.py env)"      # this section's ports
python evaluations/ux-3130/measure.py --out evaluations/ux-3130/<run>
```

It starts its own robot server and brain (sim), serves `web-twin/` and
`control/` from a plain static server (the deployed case), stubs every paid
route and the consoles' data, and holds the desktop's exclusive lock. axe-core
4.10.3 is fetched once from npm, checked against its sha512, and cached under
`~/.cache/vision-picar/`. Output: `<run>/results.json`, `<run>/shots/`.

`baseline/` is Stage 0's record on `dev` before any redesign.
