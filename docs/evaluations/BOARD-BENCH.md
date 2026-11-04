# Board bench: the full stack on the Jetson

A running log of every recurring test on the Jetson: whole-stack benches,
steady-state performance windows (3.136), memory soaks (3.135.C), fault
tests (3.135.B) and chaos soaks (3.135.D). These tests are repeated
periodically to track trends (the user, 2026-11-03). **Every run is logged,
valid or not, passed or failed:** one row in its test's trend table and a
dated entry under Runs (newest first), written in the same push that records
the run in its plan file. Keep each table's columns as they are so rows
compare; add a column only with a note saying from which date it is filled.
A plan section's full analysis stays in its plan file; this log holds the
numbers and the pointer.

Raw records go in `board-bench/<date>/` beside this file, or in the plan
section's own evaluation directory, named in the run's entry (summaries and logs;
no maps, no house images, no secrets: check before committing). How to run
each check: `tools/jetson/README.md` section 4 (G4, headroom) and section 3
(perception). **G4 is judged on the fork firmware
(`SIM_BOARD_FIRMWARE=fork`)**; on the stock emulation it fails by design.

## What each test is for

The robot runs everything on one Jetson: perception, the local vision model
(VE), ROS (SLAM and nav2), the robot server and the brain, in 8 GB shared by
the CPU and GPU. Each test below asks one question about that board. The
bars live in the plan section named; this list says why the test exists.
Every test runs in sim mode (fake motor board and lidar) until the Rover
arrives, then again on the Rover.

### Whole-stack bench (after a deploy)

* **Question:** did this `dev` commit make the board slower, hotter or
  less healthy than the last one?
* **Covers:** perception speed, the wheel loop, nav2, free memory, VE's
  answer time, explore's goal choice, G4 and the full test suite.
* **Why:** a regression check. One row per deploy shows a drift that no
  single section would notice.
* **When:** each time `dev` is deployed to the board.

### Steady-state performance window (3.136.B)

* **Question:** with every part loaded at once, does each part still meet
  its own speed budget?
* **Covers:** perception per frame (P1) and with floor-hazard prompts
  added (P2), VE's answer time (P8), the wheel loop's timing when the
  heaviest jobs overlap (P4), whether SLAM and nav2 keep their pace
  (P11), and how long the map and room work holds up a mission (P12).
* **Why:** each budget was first measured alone. Several sit close to
  their bar, so the question is whether they still hold side by side.
* **Valid only if** the 30-minute window is clean: no late wheel ticks,
  no memory kills, nav2's goals succeed, the board at 15 W. An invalid run
  is repeated, not scored.

### Latencies (3.136.C)

* **Question:** how fast does the robot react when it matters?
* **Covers:** an obstacle appearing to wheels stopped (P5), a person's
  Stop to wheels stopped and nav2's goal cancelled (P6), and arriving at
  the target to a verdict on what it is (P7).
* **Why:** these are safety and responsiveness times that have never been
  measured on the board under load.

### Live choice, cold start, heat and power (3.136.D)

* **Question:** how does the board behave over a real day?
* **Covers:** explore's goal choice on live maps (P3), power-on to ready
  and a new target's first frame (P9), an hour of missions without
  throttling (P10), and the power drawn, which sets battery runtime (P14).
* **Why:** a cold start and sustained heat are what a person at home sees
  first. The power numbers feed the battery ladder (3.137).

### Memory soak, 4 h (3.135.C)

* **Question:** does the board leak memory slowly over hours?
* **Covers:** 4 hours of back-to-back missions, a new target each time,
  five houses in turn. Free memory and every process's size are sampled
  every 5 s.
* **Why:** every earlier memory number came from one mission. The margin
  above the 500 MB floor is a few hundred MB, so a leak of tens of MB an
  hour would run the board out of memory within a day and could kill a
  process mid-mission.
* **Passes if:**
  * free memory never drops below 500 MB;
  * it does not trend down faster than 10 MB an hour, and no process grows
    more than 50 MB;
  * there are no memory kills and no late wheel ticks;
  * at least 90% of missions find the target with no contacts, so a run
    cannot pass by standing still.
* **Why 4 hours:** about three charges of the Rover's battery, plus a
  session on the charger.

### Single faults (3.135.B)

* **Question:** when one thing breaks, does the robot stop safely and
  recover without a person?
* **Covers:** one fault at a time (F2-F18, and F24): memory runs out, VE
  crashes, the ROS container dies or hangs, the brain is killed or hangs,
  the robot server is killed, the lidar or wheel feedback stops, the cloud
  is down or hangs, the phone's link drops mid-drive, the board reboots,
  the disk fills, the clock jumps, a person's Stop under full load, two
  faults at once, and the battery running down mid-mission.
* **Why:** each fault has a guard that is supposed to catch it. This
  checks each guard on the real board, measured on the simulator's ground
  truth, not on the readings the guard itself uses.
* **The harness is checked first:** each test is run once with its guard
  switched off and must fail. A test that passes with no guard proves
  nothing.

### Chaos soak (3.135.D)

* **Question:** does the robot stay safe and keep working when faults
  arrive at random for hours?
* **Covers:** at least 40 faults, one every 3-7 minutes, drawn from the
  single-fault list, with a simulated battery charge cycle about every
  hour.
* **Why:** single faults are tested clean. Real faults overlap with
  missions in progress, half-finished recoveries and a running-down
  battery.
* **Passes if:** every fault meets its own stop bar, there are no
  contacts, a new mission is running within 90 s of every fault, and the
  memory soak's bars still hold.

### On the Rover (3.135.E, 3.136.E)

* **Question:** do the same numbers hold with the real camera, lidar,
  motor board and battery?
* **Adds:** a battery that sags, an unplugged motor board or lidar, a
  stalled motor, and brownouts from motor current. These cannot be tested
  in sim.
* **When:** after the Rover arrives (3.26).

## Trend

Jetson Orin Nano Super, 15 W, every check under the board lock.

| Date | `dev` | Perception p50 / p90 (ms) | Perception p90 under the stack | Late wheel ticks | nav2 | `MemAvailable` min (MB) | OOM | Max temp | VE p90 | F-large choice p90, full / quiet (ms) | G4 | Suite |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2026-11-01 | `d1c5bc92` | 62 / 115 | 182 | 0 / 6,865 | 18/18 | 1,130 | 0 | 63 °C | 1.32 s (299/299) | 97.7 / 82.2 | 5/5 (fork) | 4,260 passed, 2 failed |

Bars: perception 250 ms; late ticks 0; nav2 all goals; memory floor
500 MB (the user, 2026-10-14); VE p90 1.32 s (3.78); goal choice 100 ms
(3.131); G4 five clean runs in a row.

### Steady-state performance window (3.136.B)

A 1,800 s full-stack window with tiered explore missions running
(`evaluations/jetson-perf-3136/`). A run is valid only under the section's
rule (0 late ticks, 0 OOM, nav2 goals, 15 W; nav2's item under amendment 5
from run 2: aborted frontier goals do not count). Numbers from an invalid
run are information only.

| Date | Rev | Valid | P1 perception p90 (ms) | P2 hazards p90 (ms) | P8 VE p90 (s) | P4 late ticks | P11 controller (Hz) | P12 room pause | `MemAvailable` min (MB) | OOM | Max temp |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 2026-11-02 | harness `51f1e6e2` on `dev` `d8be7e09` | **no** (4 late ticks; 10 of 47 nav2 goals aborted) | 153 | 153 | **1.99** | **4** / 24,711 (max 114 ms) | 20.05 | 0.17% | 645 | 0 | 65.5 °C |

Bars: P1, P2 250 ms; P2 memory +100 MB; P8 1.5 s and the same answers as
3.78; P4 0 late; P11 >= 18 Hz; P12 pause <= 10%.

### Memory soak, 4 h (3.135.C)

Back-to-back tiered explore missions for 4 h, VE resident, a new target
each mission, five houses cycled, the cloud a local stub
(`evaluations/jetson-fmea-3135/`).

| Date | Rev | Length | Missions | Found | Contacts | `MemAvailable` min (MB) | Slope, last 3 h (MB/h) | Max RSS growth h1->h4 | OOM | Late ticks | Verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 2026-11-02/03 | `dd0a7c9f` | 14,677 s | 19 | **10 (53%)** | 0 | 700 | **-21.1** | none over 50 MB | 0 | **2** / 201,841 (max 104 ms) | **FAILED** (b, c, d; 3.142, 3.143, 3.141) |

Bars: (a) minimum >= 500 MB; (b) slope >= -10 MB/h and no process's
per-mission minimum RSS up > 50 MB; (c) 0 OOM, 0 late ticks; (d) >= 90%
found, 0 contacts.

### Fault tests (3.135.B) and chaos soak (3.135.D)

No runs yet. The 3.135.A smoke (2026-11-01, F13 only, not scored) is in
`evaluations/jetson-fmea-3135/board/smoke/`.

## Runs

### 2026-11-04, 3.143's A3: fix V with VE, in process (3.143 bar V1, not a recurring test)

`evaluations/soak-find-3143/board.py a3`, rev `00271a5c` (dev with
`139f97a3` and `3f0c3adc`), 07:16-07:24 under `board_lock` through 3.136's
`board_run` and lease. 15 W, the desktop and the twin's units stopped, VE
resident as found (MainPID 838, 0 restarts, before and after).
`MemAvailable` 2,838 MB before, 2,753 MB after. The board was left as found
(no restore actions) and the lease released. `ladder.py A3` ran the 40 soak
pairs in process with rendered frames and VE as the local tier, taking 489 s.
* **39/40 found (V1 met: bar A2's 38).**
* VE yes at 16 arrivals, **no at 21 true-target arrivals** (decided by
  fix V's geometry), unscored at 2.
* 1 stand-in call.

Records: `evaluations/soak-find-3143/board/a3-1/`.

### 2026-11-03, the vet's cost with 3.141's turn guard (3.141 criterion 2, not a recurring test)

`evaluations/wheel-loop-3141/vet_compare.py`: the branch's `robot/` copied to
`/tmp` on the board, dev's turn guard (`tests/pivot_reference.py`) beside
3.141's, one vet each, 15 W, under `board_lock`; every answer the same.

* 12:27, `83b5009e` (first chunk 8 returns), load 4.6. **Contaminated:** a
  3.135.B run orphaned after its desktop runner stopped (its sim stack:
  robot server, brain, `picar-ros-3135`, sensor programs) ran beside this
  bench from 12:27:00 to ~12:27:40; none of its numbers backs a bar.
  Read as such: corner-turn boxes p90
  2.45-3.28 ms (dev 20.6-23.3); the turn blocked at the first returns 0.13
  ms (dev 0.11): **+18%, the 10% bar FAILED** on that case.
  `evaluations/wheel-loop-3141/board/vet-compare-20261103-1226.txt`.
* 12:32, `c6496ea6` (first chunk 2), load 0.15: **corner-turn boxes p90
  2.33-2.36 ms** (dev 20.34-23.04; bar 5: met); a turn in the open 1.21 (dev
  2.47); blocked at the first returns 0.08 (dev 0.11); straight 0.62-0.65
  (dev 0.65): every case at or under dev's. Criterion 2 met.
  `evaluations/wheel-loop-3141/board/vet-compare-20261103-1230.txt`.

### 2026-11-03, one wheel-loop vet's cost (3.141.A, a diagnostic, not a recurring test)

The board quiet (load 0.7, 15 W, the board's checkout at its own rev, its
`robot/safety.py` last changed in `f9c2ede7`), 12:09-12:10 under
`board_lock`. `evaluations/wheel-loop-3141/vet_bench.py`: one
`SafetyController.vet_wheel_velocity()` on a 360-bin scan, by case.
Straight in the open 0.64 ms p50; turning in the open 2.47 ms; **a turn
clamped near a corner, where `pivot_scale()` searches, 20.1-22.8 ms p50**
(desktop 9-16 ms). That vet runs on the event loop inside the wheel tick, so
it counts toward the 100 ms late line. Record:
`evaluations/wheel-loop-3141/board/vet-bench-20261103-1209.txt`.

### 2026-11-02/03, memory soak run 1 (3.135.C), rev `dd0a7c9f`

On the board 21:43 -> 01:48 (14,677 s scored), 15 W, the board booted
12:55:42. **Criterion 1 FAILED: (a) met, (b), (c) and (d) failed.**

* (a) `MemAvailable` minimum 700 MB (791 by the per-process sampler).
* (b) The per-mission minimum fell 21.1 MB/h over the last 3 h (13
  missions; bar -10). No process's minimum RSS grew over 50 MB (largest
  +5.6 MB). Each mission's low is a dip 27-42 s in, while the text encoder
  encodes the new target; the fit's first point (mission 9, 1,253 MB, a
  repeated target) carries the slope (without it +5.0 MB/h), and the
  per-mission median rises 33 MB/h. One run's reading, not proof of no
  leak. Section **3.142**.
* (c) 0 OOM kills; 2 late wheel ticks (missions 1 and 18, max 104 ms) --
  the same symptom as 3.136.B's P4, held by 3.141.
* (d) 10 of 19 found (out of budget 8, exhausted 1), 0 contacts. By house:
  `home_first_floor` 4/4, `complex_house` 1/4, `procthor:val:0` 3/4,
  `procthor:val:2` 0/4, `procthor:val:3` 2/3. The in-process sweep finds
  39/40 on the same pairs, so the board's live find rate is the gap. Of
  the 9 misses, 5 reached the target and the local VLM (VE) said no on the
  simulator's rendered frames (scores <= 0.0025, no cloud call); 4 never
  arrived. Not nav2 (4 aborts in 19 missions), not memory. Section
  **3.143** (first: whether the sweep asks a local VLM at all).
* The desktop half of the harness died with its Claude session overnight;
  the records were fetched, the board checked as found (0 differences) and
  the run scored by hand on 2026-11-03. Records:
  `evaluations/jetson-fmea-3135/board/run1/`; analysis in
  `docs/plans/ros-alignment/3.135-jetson-fmea.md`.

### 2026-11-02, steady-state window run 1 (3.136.B), harness `51f1e6e2`

On the board 13:35 -> 14:10 (1,800.8 s), 15 W, max 65.5 °C, booted 12:55:42
(uptime 2,379 s at the start). **Invalid** (4 late wheel ticks; 10 of 47
nav2 goals aborted, explore's unreachable frontiers), so no row is scored.
Two missions ran (one out of budget at 1,200 s, one cut by the window), 0
cloud calls. Information only: P8's VE p90 1.99 s against 1.5 s (VE shared
with the missions' questions) and P4's 4 late ticks (3.141) are the two to
watch on run 2. Records: `evaluations/jetson-perf-3136/board/run1-20261102-1329/`;
analysis in `docs/plans/ros-alignment/3.136-jetson-perf.md`.

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
