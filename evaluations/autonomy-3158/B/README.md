# 3.158.B: the rehearsal, as run (written 2026-11-08, before any counted mission)

3.158.A's B protocol, bars as written. What this run fixes, and where it differs:

* **Desktop, not the board.** A's stated fallback, by the orchestrator's limit
  (no board use). VE is a stand-in, so its identity numbers say nothing of
  VE's. So are the cloud ($0, no network) and the local tier: each answers
  from the simulator's report for the frame now, `evaluations/tiered-380/O`'s
  idea for any named object. C15 is the board's and is not read.
* **The board's configuration otherwise:** tiered, `tier_search: explore`,
  a local tier at `LOCAL_VLM_URL`, `drive: ros`, `world: ros`, the saved map in
  its own volume (`picar-maps-3158`, local only, removed after), `return_to_dock`,
  the drop guard, `explore_goal_policy` as `dev` ships it. `recharge_resume` off.
  Image `vision-picar-ros:plan-3.156` (no `service/slam/` commit since).
* **The house:** `home_first_floor_stairs`, which is `home_first_floor` plus
  the stairwell: A names both once built, and the one holds the other, so C8
  has an edge to read. All 13 rooms with reachable floor are on the sheet.
* **`sheet.json` is frozen** (`make_sheet.py`, seed 3158): 26 present finds,
  4 absent, 5 surveys, then the stretch of 8 finds and 2 surveys.
  **Deviation from C's rule:** at least 4 small targets are required; this
  house has two small things (red backpack 1 cell, side table 2). The other
  eight are furniture, so C1 here is easier than C's rule intends.
* **C1's denominator is the 26 present finds:** A's "(>= 24 of 30)" assumes 30
  present; the protocol's 30 finds include 4 absent. 80% of 26 is 21.
* **Truth.** `found` is right when the robot ends within 0.9 m of a cell of
  that object (3.80.G2). C7: `tests/footprint_sweep.py`'s chassis under 1.0 cm
  of a wall or object, one contact per unbroken touch, sampled at 2 Hz (a
  shorter touch is unseen). C8: `tests/drop_sweep.py`'s `drop_truth`. C10:
  `return` docked and the true pose within 0.10 m / 10 deg of that mission's
  own start (3.156 D1's dock), which is not always the first dock. C5: the
  true centre's room. C9: the harness sends no person's input, so it is 0 by
  construction, and says nothing.
* **Battery:** `sim/battery.py` 12.0 V, 0.0006 V/s (placeholder, roughly a
  pack's hour), the pad 0.02 V/s; an idle robot charges to 11.9 V before the next
  start (A's "spaced for the charge"), at most 240 s. C13's Wh are not read.
* **Batches:** one stack each, at most 75 min of missions, 90 s between, so the
  exclusive lock is free between them. A mission ending off the dock ends its
  batch, because only a fresh robot server puts the robot back.
* **A stretch ends** at the first mission off the dock, `failed`, contact or
  fall (A's D5, D7, D2); the rows say which.
* **Harness shakedown:** before the first counted mission, one find to a scratch
  file. Not counted, and not in this directory.
