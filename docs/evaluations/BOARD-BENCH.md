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
| 2026-11-04 | 3.141.C window (not run 2), `dev` `ae5cecff` | **no** (6 late ticks; 6 of 53 nav2 goals aborted, amendment 2's rule) | 166 | 150 | **1.99** | **6** / 24,925 (max 113 ms) | 20.05 | 0.32% | 648 | 0 | 64.4 °C |
| 2026-11-04 | run 2, `dev` `d69abc56` (3.141.E1 in) | **no** (1 late tick; 2 approach goals ABORTED, amendment 5 by id) | 169 | 152 | **1.99** | **1** / 28,807 (max 106.5 ms) | 20.04 | 0.22% | 780 | 0 | 65.2 °C |

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

One row per row or case run on the board (`evaluations/jetson-fmea-3135/`,
`run.py row`), 15 W, the twin's units stopped unless the case says
otherwise. The stack is in sim mode, judged on the simulator's ground truth.
"Runs" are scored / run in total; amendment 4 has the rules. The 3.135.A
smoke (2026-11-01, F13 only, not scored) is in `board/smoke/`.

| Date | Rev | Row (case) | Runs | Verdict | The row's measure (bar) | Contacts | Late ticks | `MemAvailable` min (MB) | OOM | Max temp | Recovery | As found |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2026-11-03 | `a3382c18` | F13 D-pad link lost | 5/5 | met | wheels zero 1.00-1.02 s (1.1) | 0 | 0 | 2,251 | 0 | 49.4 °C | 3.2-3.4 s | yes |
| 2026-11-03 | `a3382c18` | F8 robot server killed (harness) | 5/5 | met | wheels zero 1.48-1.49 s (1.6); never `found` | 0 | 0 | 2,250 | 0 | 49.5 °C | 3.2-3.3 s | yes |
| 2026-11-03 | `a3382c18` | F9 lidar stops | 5/5 | **FAILED** | **101.2-104.6 cm** moved while the scan was stale (2) | 0 | 0 | 2,247 | 0 | 49.8 °C | 3.0-3.2 s | yes |
| 2026-11-03 | `5b6b01cc` | F10 wheel feedback stops | 5/5 | met | 0.0-0.31 cm while stale (2); `no_feedback` refused 5/5 | 0 | 0 | 2,246 | 0 | 49.8 °C | 3.0-3.2 s | yes |
| 2026-11-04 | `ad1def4b` | F9 lidar stops (3.144.B, the fix) | 5/5 | met | 0.0 cm while stale (2) | 0 | 0 | 2,360 | 0 | 50.1 °C | 3.1 s | yes |
| 2026-11-04 | `d1d69eff` | F5 ROS bridge hung, a person's Stop | 5/5 | met | `/stop` 0.008-0.016 s (1.0); zero 0.008-0.021 s after it (0.5) | 0 | 0 | 2,099 | 0 | 51.8 °C | -- | yes |
| 2026-11-04 | `d1d69eff` | F4 ROS container killed | 5/5 | **FAILED** | **zero 1.08-1.14 s (0.5), 8.3-9.4 cm (5)**; ROS not named, readout `watchdog` | 0 | 0 | 2,070 | 0 | 51.3 °C | a new container, then `found`, 5/5 | yes |
| 2026-11-04 | `d1d69eff` | F6g brain killed, a nav2 goal | 5/5 | **FAILED** | **zero 2.80-3.33 s (1.1), 16.9-18.5 cm (15)** | 0 | 0 | 2,097 | 0 | 50.9 °C | 6.4-6.6 s | yes |
| 2026-11-04 | `d1d69eff` | F7g brain hung 30 s, a nav2 goal | 5/5 | **FAILED** | **zero 2.79-3.15 s (1.1), 17.1-18.3 cm (15)**; 0 driving after SIGCONT (met) | 0 | 0 | 2,082 | 0 | 50.7 °C | 4.9-5.7 s | yes |
| 2026-11-04 | `affb16fd` | F6 brain killed, a FORWARD | 5/5 | **FAILED** | zero 0.82-0.85 s (1.1); **23.8-24.8 cm (15)** | 0 | 0 | 2,463 | 0 | 55.9 °C | 4.4-4.8 s | yes |
| 2026-11-04 | `affb16fd` | F11a cloud refused from the start (tiered) | 5/5 | **FAILED** | 4 `arrived_unconfirmed`, **1 `blocked`** (any other end fails); 0 paid calls | 0 | 0 | 2,415 | 0 | 51.9 °C | -- | yes |
| 2026-11-04 | `affb16fd` | F11b cloud lost mid-mission (tiered) | 5/5 | **FAILED** | 2 `arrived_unconfirmed` (late confirmation final: `refused`), **3 `blocked`**; open paid calls max 1 | 0 | 0 | 2,376 | 0 | 49.6 °C | -- | yes |
| 2026-11-04 | `affb16fd` | F11c cloud refused (`vision`) | 5/5 | met | `failed` in 2 s, 0.0 cm (0.5) | 0 | 0 | 2,392 | 0 | 49.6 °C | -- | yes |
| 2026-11-04 | `affb16fd` | F12a cloud hangs (tiered) | 5/5 | **FAILED** | 3 `arrived_unconfirmed`, **2 `blocked`**; open paid calls max 1; parked 3.8-15.6 s (30) | 0 | 0 | 2,385 | 0 | 49.6 °C | -- | yes |
| 2026-11-04 | `affb16fd` | F16c clock control (no step) | 3/3 | met | `out_of_budget` at 400.9-401.8 s (the 400 s limit) | 0 | -- | 2,392 | 0 | 51.0 °C | -- | yes |
| 2026-11-04 | `affb16fd` | F16p clock +5 min | 0/8 | incomplete | every step landed in a pivot, not between verbs: invalid (amendment 15) | 0 | -- | 2,376 | 0 | -- | -- | yes |
| 2026-11-04 | `affb16fd` | F16m clock -5 min | 0/8 | incomplete | as F16p: invalid (amendment 15) | 0 | -- | -- | 0 | -- | -- | yes |
| 2026-11-04 | `affb16fd` | F15 disk full | 5/5 | met | `found` 5/5; failing-write lines 1 per writer (choices, inventory, intervention) in 38-100 s | 0 | 0 | 2,034 | 0 | 51.5 °C | -- | yes |
| 2026-11-04 | `5425f8ac` | F16p clock +5 min (re-run, amendment 15) | 5/5 | met | `out_of_budget` at 400.9-402.0 s (F16c's end, within 10 s of 400); 0 watchdog stops, 0 stale 503s; every step between verbs | 0 | -- | 2,196 | 0 | 49.7 °C | -- | yes |
| 2026-11-04 | `36b6a944` | F16m clock -5 min (re-run, amendment 15) | 5/5 | met | `out_of_budget` at 401.0-402.1 s; 0 watchdog stops, 0 stale 503s; every step between verbs | 0 | -- | 2,200 | 0 | 49.7 °C | -- | yes |
| 2026-11-04 | `53bcbe61` | F12b cloud hangs (`vision`) | 5/5 | met | `failed` at 61.4-61.5 s (300), B3.2's 3 x 20 s; open paid calls max 1; 0.0 cm | 0 | -- | 2,207 | 0 | 49.8 °C | -- | yes |
| 2026-11-04 | `1330ea8f` | F2 memory runs out (OOM), during a nav2 goal | 3/8 | **FAILED** | VE's driver the only kill 4/4, robot server, brain and ROS alive, 0 late ticks, every mission ended; **truth log holes 0.69 s and 3.01 s** (0.2) at 15-99 MB free; runs 5-8 not started (VE's start limit) | 0 | 0 | 37 | 4 | 59.9 °C | -- | yes |
| 2026-11-04 | `1330ea8f` | F3f VE killed, a frame question open | 2/8 | **FAILED** (harness) | VE back 40.8 s (60); not failed 2/2; handed-on read **None** (the kill before any tier stats: a harness bug); runs 4-8 not started (VE's start limit) | 0 | 0 | 2,054 | 0 | 55.4 °C | -- | yes |
| 2026-11-04 | `1330ea8f` | F3a VE killed, the arrival's question open, the stand-in up | 5/5 | **FAILED** | handed on 41 (>= 1), VE back 41 s; **every mission `exhausted`, not `found`**: the stand-in refuses the arrival, and once the cloud has spoken the arrival is the cloud's (3.91.C) | 0 | 0 | 2,006 | 0 | 59.8 °C | -- | yes |
| 2026-11-04 | `1330ea8f` | F3r VE killed, the arrival's question open, the cloud refused | 3/8 | incomplete | `arrived_unconfirmed` 3/3, handed on 3, VE back 40.7-40.8 s; runs 4-8: VE's start limit | 0 | 0 | 2,138 | 0 | 57.7 °C | -- | yes |
| 2026-11-04 | `1330ea8f` | F17g a person's Stop, a nav2 goal, VE resident | 5/5 | met | wheels zero 0.016-0.037 s after the `/stop` (0.2); `cancelled`/`stop` 5/5; 0 paid calls after it | 0 | 0 | 2,136 | 0 | 52.8 °C | -- | yes |
| 2026-11-04 | `1330ea8f` | F17q a person's Stop, a frame question open | 5/5 | met | wheels zero 0.000-0.020 s (0.2); `cancelled`/`stop` 5/5; 0 paid calls after it | 0 | 0 | 2,143 | 0 | 52.3 °C | -- | yes |
| 2026-11-04 | `1330ea8f` | F17a a person's Stop, the arrival's question open | 5/5 | met | wheels zero 0.001-0.007 s (0.2); `cancelled`/`stop` 5/5; 0 paid calls after it | 0 | 0 | 2,079 | 0 | 57.0 °C | -- | yes |

## Runs

### 2026-11-04, 3.141.F1: the map reply sliced, the P12 poller ON (`dev` `016cbd97`, traced)

30 min of 3.136.B's stack, 15 W, every vet traced, left as found. **0 late
wheel ticks** in 25,837 (max 98.4 ms; F0-on 2, max 108.9), valid by 3.136.B's
rule. 29 probe ticks over 80 ms (F0-on 55), all on a map reply (chance
14.5%); map replies as long as before (p50 95, p90 152 ms). Vets p90 2.2 ms,
max 23-24 ms. 2 found, 1 out of budget, 1 cut. `MemAvailable` min 507 MB,
max 63.8 °C. Records: `evaluations/jetson-perf-3136/board/3141F1-on-20261104-2318/`.

### 2026-11-04, 3.141.F0: the P12 map poller off, then on (`dev` `ee2e897d`, traced)

Two 30 min windows of 3.136.B's stack, 15 W, every vet traced, left as found.
* **Off (21:10): 0 late wheel ticks** in 23,937 (max 94.5 ms), valid; 22 of 25
  probe ticks over 80 ms overlap one of explore's own map fetches (chance
  9.5%). 1 found, 1 out of budget, 1 cut. `MemAvailable` min 777 MB.
* **On (22:01): 2 late** in 28,347 (max 108.9 ms); 54 of 55 over 80 ms and 6
  of 6 over 100 ms overlap a map reply (chance 16-17%). INVALID also on nav2:
  2 approach goals aborted right after the previous goal's cancel (3.154).
  2 found, 1 out of budget, 1 cut. `MemAvailable` min 567 MB.
* Records: `evaluations/jetson-perf-3136/board/3141F0-off-20261104-2110/`,
  `.../3141F0-on-20261104-2201/`.

### 2026-11-04, 3.141.E1's board check: every vet traced (`dev` `d69abc56`, 3.141.E criteria 3-4)

3.136.B's 30 min window, 15 W, max 64.4 °C, 0 OOM, `MemAvailable` minimum
720 MB, the board up ~16.7 h; the stall tracer with `TRACE_3141_VET_PARTS=1
TRACE_3141_VET_MS=0` (amendment 5: every vet a row, 63,187); left as found.
Not a 3.136.B row (the tracer runs in it).
* **Criterion 3, half met:** unclamped vets p90 **2.38 ms** on the loop
  (24,845), **1.91 ms** in the threadpool (34,196) -- bar 3 ms, met (E0:
  p90 12.0 over its slow rows); max **33.4 / 39.7 ms** -- bar 15 ms,
  FAILED (the slowest are `pivot_scale` at 26-38 ms, wall time under the
  load).
* **Criterion 4: FAILED, 3 late wheel ticks** in 25,005 moving (max 123 ms;
  run 2, untraced, 1).
* **What is still late:** 33 of the tracer's 35 probe ticks over 80 ms and
  all 4 over 100 ms overlap a `/world/map` reply, where chance would give
  12.7% / 14.2% (a reply over 20 ms in flight 6.2% of the window, each
  widened by the tick's length; `map_overlap.py`) (113-231 ms replies, often two at once: explore's fetch
  and the harness's P12 map poller). Inferred: the map reply's work holds
  the GIL against the loop; not measured inside the reply.
* Missions 2 `found`, 1 cancelled at the window's end; nav2 30 of 58
  succeeded, 12 frontier aborts (not counted), 0 other.
* Records: `evaluations/jetson-perf-3136/board/3141E1-20261104-1932/`
  (`vet_rows.json`).

### 2026-11-04, 3.136.B run 2, `dev` `d69abc56` (INVALID)

The board up ~16 h (warm), 15 W, max 65.2 °C, 0 OOM kills, `MemAvailable`
minimum 780 MB, desktop load <= 3.5; 18:57-19:30, 3.136's runner with its
lease, left as found. The board's `vision-picar-ros:latest` rebuilt from
`dev` `ed6875b5` just before (`561e9f553d12`; the old `748edc53d16c` kept as
`:pre-ed6875b5-20261031`), so nav2 is read by amendment 5, by goal id: 54
goals, 0 unmatched, 0 sends missing.
* **Invalid on two items:** 1 late wheel tick in 28,807 moving (max
  106.5 ms; P4 and 3.141.E criterion 4 FAILED), and 2 `approach` goals
  ABORTED (mission 2, 38 s apart). nav2's log shows both were the CLIENT's
  cancels that `bt_navigator` ended as aborted ("Failed to get result for
  follow_path in node halt!", then "Aborting handle"); under the rule as
  written they invalidate. 1 frontier goal aborted (not counted), 41
  succeeded, 10 cancelled.
* Rows, information only: P1 p90 169 ms, P2 152 (+-48 MB), P8 VE p90
  1.99 s (299/299, identical answers: FAILS its 1.5 s if it repeats in a
  valid run), P11 20.04 Hz (scan lag p90 11.7 ms), P12 pause 0.22%;
  missions 2 `found`, 1 cancelled at the window's end; 58 room-label calls,
  0 cloud calls.
* Records: `evaluations/jetson-perf-3136/board/run2-20261104-1857/`.

### 2026-11-04/05, 3.152.B: VE's unit in an OOM crash loop, six runs (criterion 10, not a recurring test)

`evaluations/local-vlm-3152/board.py <arm>`: rev `9dfc6d63` (d30 and d60's
first runs), `d463dc5f` (the re-runs), `0972c251` (d120, shipped). Each ran
21:43-01:05 under `board_lock` through 3.136's `board_run` and lease. 15 W,
the desktop and the twin's units off. 3.135.C's soak stack ran tiered
missions beside a VE crash loop, which a hog drove at `oom_score_adj` 100
with a 15-min window. The candidate unit was a runtime drop-in
(`StartLimitIntervalSec=0`, `RestartSec=D`); "shipped" ran with none. Every
run left the board as found; the unit was read back each time.

| Arm (board run) | Hog | Valid | VE kills in window | Late wheel ticks (per kill, per hour, max) | Servers down (robot / brain, of live samples) | Missions failed in the squeeze | VE ready share |
|---|---|---|---|---|---|---|---|
| d30, `d30-20261104-2132` | old (fixed size) | yes | 6 (loop paused ~270 s) | 4 (0.67, 16, 0.235 s) | 0 / 0 of 810 | m000: `GET /frame` 503 | -- |
| d60, `d60-20261104-2203` | old | **no**: 1 kill, no loop | 1 | 0 | 0 / 0 of 898 | m000: the ROS bridge timed out | -- |
| d30 re-run, `d30-20261104-2322` | topping-up | yes | 15 | **52** (3.5, 207, 0.946 s) | 3 / 2 of 699 | m000: no `/wheels` from the actuator | -- |
| d60 re-run, `d60-20261104-2321` | topping-up | **no** as written: an errored mission | 10 | 37 (3.7, 148, 0.267 s) | 1 / 1 of 767 | m000: the brain's loop hung (tick > 30 s) | -- |
| d120, `d120-20261105-0014` | topping-up | yes | 5 | 8 (1.6, 32, 0.659 s) | 1 / 3 of 716 | m000: the brain's loop hung (tick > 30 s) | 3.5% |
| shipped, `shipped-20261105-0045` | topping-up | yes (start limit hit after 4 kills, ~190 s in) | 3 (4 in all) | 20 (6.7, 80, 0.495 s) | 0 / 1 of 881 | none (m000 `found`) | 2.6% |

* Every kill was VE's; no stack process was killed. The memory after VE
  was back was >= 850 MB on every valid run.
* 0 late ticks and the servers answering were FAILED on every valid run.
* All but shipped lost a mission to a stack call timing out in a squeeze
  (3.153).
* d60 is invalid twice (see 3.152's section).

Records: `evaluations/local-vlm-3152/board/`.

### 2026-11-04, 3.141.E0: an unclamped vet split by call path, on the board (`d20d8eb7`)

3.136.B's 30 min window, 15 W, the stall tracer with `TRACE_3141_VET_PARTS=1`,
left as found. 12,652 vets over 5 ms (p90 12.0, max 61 ms): `pivot_scale`'s
own arithmetic 82.5% of their time (p90 10.6 ms), `forward_clearance`'s
12.8%, every sensor read ~3%. 7 late wheel ticks in 26,681 moving;
generation-2 GC at most 19.1 ms. Records: `evaluations/jetson-perf-3136/board/3141E0-20261104-1722/`.

### 2026-11-04, 3.141.D D1 run 2: the tracer smoke again (`835deeb5`, amendment 4)

15 W, left as found. Tracer CPU 0.97% of a core, the sampler alive, 0
errors, 0 rows dropped; **FAILED on (c)**: 7 of 164 stalls (4.3%, bar 1%)
right after a long drain of the tracer's own rows. 4 late wheel ticks;
generation-2 GC at most 14.1 ms. Records: `evaluations/jetson-perf-3136/board/3141D-smoke2-20261104-1556/`.

### 2026-11-04, 3.141.D D1 run 1: the stall tracer's smoke on the board, `dev` `aec7fd4b` + the tracer (`503638da`)

15 W, an 806 s steady window with `STEADY_TRACER=1`, left as found. The
tracer's own CPU 1.26% of a core (bar 2), heap rows every 30 s, 0 errors,
0 rows dropped; **FAILED on (c)**: one stall where a tracer thread (its
once-a-second `/proc` read) outran the loop. Seen beside it: 5 late wheel
ticks, generation-2 GC at most 15.8 ms, 5,480 vets over 5 ms (p90 12.6,
max 55 ms). Records:
`evaluations/jetson-perf-3136/board/3141D-smoke-20261104-1533/`; analysis in
3.141's section.

### 2026-11-04, the steady window as 3.141.C's check (not 3.136.B run 2), `dev` `ae5cecff`

The board booted 02:51 (the house power cut; up ~7 h), 15 W, max 64.4 °C,
0 OOM kills, `MemAvailable` minimum 648 MB, desktop load ~1.1; 3.136's
runner with its lease, 09:55-10:35, left as found. `dev` here includes
3.141.B (`4d8f472a`). Judged only for 3.141.C criterion 4 (P4's bar): the
harness had no amendment-5 join yet, so nav2 is read by amendment 2's rule
and this is not 3.136.B run 2.
* **P4: 6 late wheel ticks in 24,925 moving (max 113 ms; 70.7 ms before
  the window): criterion 4 FAILED** (run 1: 4 in 24,711, max 114 ms). One
  every 4-5 min (+298, +460, +708, +911, +1200, +1469 s), in P1, P8 and
  P2 alike, each 101-113 ms: a periodic pause near the line, not 3.141.B's
  corner-turn vet (now 2.3 ms).
* Others, information only: P1 p90 166 ms, P2 150, P8 VE p90 1.99 s
  (299/299, answers identical), P11 20.05 Hz, P12 pause 0.32%; nav2 47 of
  53 succeeded (6 aborted, 11 cancelled); missions 4 `found`, 1 cancelled
  by the window's end.
* Records: `evaluations/jetson-perf-3136/board/3141C-window-20261104-0955/`.

### 2026-11-04, 3.143's V3: fix V with VE, the live stack (3.143 bar V3, not a recurring test)

`evaluations/soak-find-3143/board.py v3`, rev `0c35794b`, 07:40-08:10
under `board_lock` through 3.136's `board_run` and lease. 3.135.C's soak
stack ran the five class V pairs of its run 1, once each. 15 W, the desktop
and the twin's units stopped, VE resident as found (MainPID 838, 0 restarts
at every mission). `MemAvailable` 1,718 MB after setup, 2,856 MB at the end.
The board was left as found and the lease released.
* **4/5 found (V3 met: bar 4), 0 contacts, 0 cloud calls.**
* At the 4 arrivals, VE said no at 3 (fix V's geometry decided them) and
  yes at 1.
* The miss, `complex_house` slippers, ran to its 1,200 s limit with no
  arrival (closest 1.23 m).
* 0 `ahead_not_observed`.

Records: `evaluations/soak-find-3143/board/v3-1/`.

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

### 2026-11-04, F9 re-run on the fixed build (3.144.B, a 3.135.B row)

`run.py row F9 --runs 5` at `ad1def4b` (3.144.A's `forward_clearance()` fix,
3.136.F's lease), 15 W, 07:10-07:12, one board hold. **Met 5/5: 0.0 cm moved
while the scan was stale** (bar 2 cm; the shipped build on 2026-11-03: 101-105
cm, FAILED, 3.135.B's record). Wheels zero 2.0 s after the fault by the
watchdog (no FORWARD starts on a stale scan); 0 contacts, 0 late ticks, 0 OOM,
`MemAvailable` minimum 2,360 MB, max 50.1 °C; recovery 3.1 s; left as found.
Records: `evaluations/jetson-fmea-3135/board/rows/F9-3144B-20261104-0710/`.

### 2026-11-04, single faults (3.135.B): the VE rows F2, F3, F17

On the board 14:56-18:54, one hold per row, at `1330ea8f`. 15 W, the twin's
units stopped and put back. The full stack: ROS, VE resident behind a
recording proxy, the $0 stand-in cloud, `SIM_ARRIVAL_CONFIRM=cloud`, and the
stack at `oom_score_adj` -900. One planted run per case on the board before
its scored runs (amendment 8).

* **F17 met in all three cases** (a nav2 goal, a frame question, the
  arrival's question):
  * the wheels were zero 0.000-0.037 s after a person's `/stop`;
  * every mission ended `cancelled` by `stop`;
  * no paid call followed the Stop.
  * The `no_latch` plant was caught in every case: its missions ran on or
    ended `found`, and F17g's wheels zeroed only after 2.3 s.
* **F2 FAILED on the truth log, not on a product bar:**
  * the guard held in all 4 runs that started: the OOM killer took only
    VE's driver, everything else stayed up, 0 late wheel ticks;
  * but the simulator's truth log had holes of 0.69, 1.04 and 3.01 s at
    15-99 MB free (bar 0.2 s);
  * the planted run (VE at OOM score 0) was caught: the kernel killed the
    hog instead, so the guard discriminates.
* **VE's start limit stopped every F2/F3 row after its third or fourth
  run:**
  * `picar-local-vlm` allows 4 starts in 15 min (`StartLimitBurst=4`,
    `StartLimitIntervalSec=900`), and each run kills VE once;
  * so 15 runs could not start ("VE is not ready at the start");
  * restore put VE back (`started picar-local-vlm (was failed)`), so the
    board was left as found.
* **F3a FAILED 5/5:**
  * VE's questions were handed to the cloud (41) and VE came back in 41 s;
  * but no mission could end `found`: the stand-in says "not visible", and
    once the cloud has spoken the arrival stays the cloud's (3.91.C);
  * the bar assumed a later local yes could still find it.
* **F3f** read its handed-on count as None, because the kill lands before
  the brain publishes any tier stats (a harness bug), so its planted check
  is not valid either.
* **F3r incomplete:** its 3 scored runs passed (`arrived_unconfirmed`,
  handed on 3, VE back in ~41 s).
* Every run: 0 contacts. The board was left as found, with no harness
  leftovers.
* Records: `evaluations/jetson-fmea-3135/board/rows/`.

### 2026-11-04, single faults (3.135.B): F6, F11, F12a, F15, F16

On the board 10:27-13:27, one hold per row, released between rows (the
queue waited behind 3.141's 30-min window), at `affb16fd`. 15 W, the twin's
units stopped and put back. The mover was a frontier mission in
`scaled_house` (F6, F16), the cloud stack of amendment 5 (F11, F12a), or an
explore mission with `explore_ask` on the ROS stack (F15).

* **F6, the FORWARD case, FAILED 5 of 5:**
  * the wheels were zero 0.82-0.85 s after the kill (met);
  * but the robot moved 23.8-24.8 cm (bar 15): the verb runs to its end,
    and the watchdog skips a busy verb;
  * F6 as a whole is FAILED (F6g failed too);
  * the fix section is **3.147**.
* **F11a, F11b and F12a FAILED:** 6 of their 15 tiered missions ended
  `blocked` (1, 3 and 2 runs).
  * Each wedged at one of two places, at a true gap of ~20 cm: (2.97, 1.72)
    or (4.48, 1.66), facing east.
  * The cloud's own guards held in every run:
    * never two paid calls open at once;
    * F12a parked in 3.8-15.6 s;
    * F11b's late confirmation reached a final state;
    * nothing was driven after the end.
  * Whether the fault causes the wedge is unmeasured (no fault-free tiered
    control); the fix section is **3.148**.
* **F11c met:** a `vision` mission ended `failed` in 2 s without moving.
* **F12b met** (14:47-14:54, after its planted check was re-run on the
  desktop): a `vision` mission against a hanging cloud ended `failed` at
  61.4 s on B3.2's three 20 s timeouts, with one paid call open at a time
  and without moving.
* **F15 met:** every mission ended `found`, with 0 late ticks. Each of the
  three writers logged its failing write once.
* **F16c met** (the control): `out_of_budget` at the 400 s limit, 3/3.
* **F16p and F16m incomplete, 0 of 8 scored each, a harness fault:**
  * every step landed in a pivot, because `set-ntp` outlasted the
    0.38-0.5 s gap between verbs;
  * every run still ended `out_of_budget` at ~401 s with no watchdog stop
    (information only);
  * fixed by amendment 15 and re-run 13:30-14:39: **F16p and F16m met
    5/5 each** (every step between verbs; `out_of_budget` at 401-402 s, as
    F16c; 0 watchdog stops, 0 stale 503s), so F16 is met.
* Every run: 0 contacts, 0 OOM, `MemAvailable` minimum 2,034 MB, max
  55.9 °C. The board was left as found, with no harness leftovers.
* Records: `evaluations/jetson-fmea-3135/board/rows/`.

### 2026-11-04, single faults (3.135.B): F5, F4, F6g, F7g

On the board 08:12-08:34, one hold per row, released between rows, at
`d1d69eff` (`dev` with 3.144.A and 3.136.F's lease). 15 W, the twin's units
stopped and put back. The mover is amendment 5's goal stack: an explore
mission on the ROS container, the fault 0.15 s into a FORWARD on an active
nav2 goal.

* **F5 met:** with the bridge frozen (`docker pause`), a person's `/stop`
  answered in 0.008-0.016 s, the wheels were zero 0.008-0.021 s after it,
  and nothing drove after unpause.
* **F4 FAILED, 5 of 5:**
  * after the container's kill the wheels reached zero only at the 1 s
    watchdog (1.08-1.14 s, bar 0.5), after 8.3-9.4 cm (bar 5);
  * the mission ended `failed` on a bare 503 that names no ROS, and the
    readout showed `watchdog`, not ROS DOWN;
  * the recovery (a new container, then `found`) held 5/5;
  * its fix section is **3.146**.
* **F6g and F7g FAILED, 5 of 5 each:**
  * with the brain killed or frozen, nav2 drove its goal on for 2.8-3.3 s
    and 17-18.5 cm (bars 1.1 s, 15 cm): nav2's wheel posts keep the
    watchdog fed;
  * F7g's other half was met (nothing drove after SIGCONT);
  * their fix section is **3.145**, with a design question for the user.
* Every run: 0 contacts, 0 late ticks, 0 OOM, `MemAvailable` minimum
  2,070 MB, max 51.8 °C. The board was left as found, and no harness
  leftovers remained (drop-in, tmpfs, NTP).
* Records: `evaluations/jetson-fmea-3135/board/rows/`.

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

### 2026-11-03, single faults (3.135.B): F13, F8, F9, F10

On the board 11:58-12:13, one board hold per row, released between rows.
15 W, the twin's units stopped and put back. Amendment 2's mover: a
frontier mission in `scaled_house`, no ROS, no VE.

* **F13 met**: wheels zero 1.00-1.02 s after the D-pad client's kill
  (bar 1.1).
* **F8's harness case met**: the fake board zeroed its wheels 1.48-1.49 s
  after the robot server's SIGKILL (bar 1.6), and the mission never ended
  `found`. F8 as a row also needs its production-unit case (amendment 6).
* **F9 FAILED, 5 of 5**: with the fake D500 stopped for 10 s, the robot
  drove 101-105 cm on the stale scan. It stopped only once the scan came
  back, 11.0 s after the fault. 0 contacts. `forward_clearance()` falls back
  to the depth cone alone when the scan is unusable, as 3.135.A saw on the
  desktop. It needs a fix section of its own (a safety-path change).
* **F10 met**: 0.0-0.31 cm while the board's feedback was silent (bar 2),
  and a `no_feedback` refusal every run.
* Every run had 0 contacts, 0 late wheel ticks and 0 OOM kills.
  `MemAvailable` minimum 2,246 MB, max 49.8 °C. A new mission was driving
  3.0-3.4 s after every fault. The board was left as found after each row
  (0 differences).
* Records: `evaluations/jetson-fmea-3135/board/rows/`; analysis in
  `docs/plans/ros-alignment/3.135-jetson-fmea.md`.

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
