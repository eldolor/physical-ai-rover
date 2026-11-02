# Board bench: the full stack on the Jetson

A running log. Each time `dev` is deployed to the Jetson and benched as a
whole, add a run below (newest first) and one row to the trend table. A
section's own board measurements stay in its plan file; this log is for
whole-stack runs, so a regression between them shows up in one place.

Raw records go in `board-bench/<date>/` beside this file (summaries and logs;
no maps, no house images, no secrets: check before committing). How to run
each check: `tools/jetson/README.md` section 4 (G4, headroom) and section 3
(perception). **G4 is judged on the fork firmware
(`SIM_BOARD_FIRMWARE=fork`)**; on the stock emulation it fails by design.

## Trend

Jetson Orin Nano Super, 15 W, every check under the board lock.

| Date | `dev` | Perception p50 / p90 (ms) | Perception p90 under the stack | Late wheel ticks | nav2 | `MemAvailable` min (MB) | OOM | Max temp | VE p90 | F-large choice p90, full / quiet (ms) | G4 | Suite |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2026-11-01 | `d1c5bc92` | 62 / 115 | 182 | 0 / 6,865 | 18/18 | 1,130 | 0 | 63 °C | 1.32 s (299/299) | 97.7 / 82.2 | 5/5 (fork) | 4,260 passed, 2 failed |

Bars: perception 250 ms; late ticks 0; nav2 all goals; memory floor
500 MB (the user, 2026-10-14); VE p90 1.32 s (3.78); goal choice 100 ms
(3.131); G4 five clean runs in a row.

## Runs

### 2026-11-01, `dev` `d1c5bc92`

The user asked for the whole stack deployed and its performance and
regression tested. The board's services, packages and units were updated to
`d1c5bc92` (the ROS image already matched). Tests ran from a separate
checkout on the board (`~/bench-dev`). The board was left as found.

**Performance: all within budget.**

* Perception alone: 62 ms median, 115 ms p90 (before: 61 / 110).
  `perception-15w.json`.
* Full-stack window (389 s): 0 late wheel ticks of 6,865 (max 77.6 ms),
  nav2 18/18, `MemAvailable` minimum 1,130 MB, 0 OOM, perception 182 ms p90
  over 1,549 frames, max 63 °C, no throttling lines. `full-stack/summary.json`.
* Local VLM (VE): 299/299 answered, 0 errors, p90 1.32 s.
  `full-stack/bench.log`.
* F-large's whole goal choice: 97.7 ms p90 under the full stack, 82.2 quiet
  (3.131 measured 95.2 / 81.8). About 2 ms of margin.
  `choice-full.json`, `choice-quiet.json`.

**Regression.**

* G4: first run 2 of 5 clean, a setup error: the recipes ran the stock
  firmware emulation, whose whole-centimetre odometers leave a turn up to
  4.8 deg off ground truth (2 deg bar). Turn comparison, 8 rounds per arm:
  fork 0 of 48 turns over 2 deg (median 0.17 deg), stock 11 of 48
  (`firmware-turns/`). Rerun on the fork: **5 of 5, each 18 passed**
  (`g4-fork/`). The recipes now set `SIM_BOARD_FIRMWARE=fork` (`a0ff74f9`).
* Full suite: 4,260 passed, 26 skipped, 2 failed (`suite/`):
  * `test_sim_fingerprint` (`home_first_floor`): a pinned digest that
    differs on ARM.
  * `test_intervention_log`: under parallel load a `/mission/stop` hung
    over 15 min (the brain's tick wait).
  * Vision service: 200 passed, 1 failed:
    `test_analyze_oversized_image_is_a_413` read 502 (the Bedrock client
    built before the size check).

  All three were fixed on `dev` (`15f2d305`, `8acd6c32`); the board runs
  `054e51aa` with them since 2026-11-01. A live Stop on the updated board
  has not been tried. Next run: confirm the suite is clean on the board.
* My first suite run was invalid (the driver exported the board's secrets
  into it: hundreds of 401s) and was replaced by the run above.
