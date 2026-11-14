# Merged plan sections: what is true because of them

What each merged `docs/plans/ros-alignment/3.N-*.md` section changed, one
entry per section, grouped by area. Moved out of `CLAUDE.md` section 3b on
2026-11-12 so it is read when a session needs it, not loaded into every
session's context. It states what is true now, like `CLAUDE.md`; the
section's own file has the numbers, failed criteria and review rounds.

* **Read the area you are about to touch before working in it**, or grep
  for a section number (`3.145`) or a name (`tier_local_probe_s`).
* **Whoever merges a section adds its line here**, in its area, not a
  paragraph. `CLAUDE.md` section 3b keeps open work only; a rule every
  session must follow goes to `CLAUDE.md` section 6 instead.
* `python tools/claude_md_budget.py` keeps `CLAUDE.md` under its size
  budget; its docstring is the trim procedure.

## Identity and perception

* 3.78, 3.80: the local tier is **VE** (Qwen2.5-VL-3B INT4 AWQ on TensorRT
  Edge-LLM v0.9.1), `service/local_vlm` on `127.0.0.1:8005`, unit
  `picar-local-vlm`; on at boot on the board since 3.108, off elsewhere
  (`tier_local_vlm_url` / `LOCAL_VLM_URL`). Order: perception, local, then
  the cloud after `tier_local_max_unable` (2) or with no local VLM.
  `tier_search` defaults `cloud` (`explore` needs nav2 and a local tier).
  Memory floor 0.5 GB, the VLM killed first.
  3.80.D-T: no other local VLM, nor GPT-6 Astra, replaced it; Opus 4.5
  stays `/navigate`'s default; where to look comes from the map.
* 3.81: text features from a cache or a CPU child
  (`brain/text_encoder.py`); `foxglove_bridge` only with `PICAR_FOXGLOVE=1`.
* 3.91.C-D: `tier_remember_no` on; a lower tier never undoes a higher. 3.92: no code. 3.94, 3.95: the ladder is three py_trees
  2.6.0 trees (`brain/tree.py`, `tier_tree:`).
* 3.98, 3.143: on the simulator's frames an arrival and a local no are
  judged from geometry; real frames always ask the tiers.
* 3.148: a tiered mission with no nav2 and no steering tier (no cloud, no
  local VLM) can end `blocked` at a door jamb on its straight line (desktop
  2 of 10; 0 of 10 with the cloud up). That is the designed end (routing is
  nav2's; the pinned jamb tests stand), so 3.135.B's F11a/F11b/F12a now
  allow it. No code change; a sidestep is kept unmerged
  (`evaluations/tiered-wedge-3148/sidestep-option-a.patch`).
* 3.152.A: a local VLM found absent (refused, dropped, 503) is held down,
  the policy then exactly the no-local-tier one, and its free `/health`
  probed every `tier_local_probe_s` (5 s) until ready; a hang stays the
  unable rule's. `status.tier.stats.local_vlm` (`up`/`down`/`unwatched`/
  `off`); the twin's Local VLM row and an operator line. 3.152.B: VE's
  unit keeps its start limit (4 in 900 s). No restart-forever delay
  (30/60/120 s) met the stack's bars in a board OOM crash loop, and every
  run but the shipped unit's lost a mission in a squeeze. Kept by the
  orchestrating session; nothing installed.
* 3.47, 3.49, 3.53, 3.56, 3.61: offline is a mode; an unreachable cloud at
  arrival ends `arrived_unconfirmed` (a late confirmation follows, never
  `found`); the confirmation's 8 s deadline (`arrival_confirm_timeout_s`).
* 3.90: `robot/camera_stream.py`; `tier_frame_vote` off.
* 3.46, 3.55, 3.60: missions record an inventory; nothing reads it. 3.51, 3.54: rooms from
  CLIP, not wired. 3.41: TensorRT not adopted.
* 3.165 (3.76 G2): a landmark holds at most 32 weighted members (`FOLD_M`
  0.05 m), its position still the mean of every observation; the sweep is
  `dev`'s to the number; criterion 2 FAILED as written (1.56x, bar 1.5x).
* 3.178.A: the watch list (keys, wallet, phone, glasses, remote) has 0 labelled frames in 3.92's
  pinned set (`evaluations/watch-list-3178/counts.json`); B1-B6 and C's bars are written. Design: one
  YOLOE prompt and one CLIP encode per crop for target and list; P(target) never sees the list's texts.
* 3.167: the sweep's seed-3463 precision 97.1% -> 93.8% is 3.74 B's floor
  band moving the robot (`fc5c2122`), one seed's variance (four seeds pooled
  96.5% -> 96.9%). 3.60's precision bar is judged on seeds 346, 3461,
  3462, 3463 pooled (the user, 2026-11-07); `dev` `b45c949d` 96.85%, met.

## Explore and goal choice

* 3.125: `explore_goal_policy` defaults to `rooms` (in-process sweeps keep
  `rule`). 3.65, 3.96, 3.100, 3.101, 3.102, 3.103, 3.110, 3.111, 3.114,
  3.115, 3.117, 3.119: learned choosers land ~0.22 under the oracle;
  closed until the Rover's data. 3.116: `find_room_labels` off.
* 3.77, 3.109, 3.118, 3.120, 3.130, 3.131: goal choice, room cut and walk in
  numpy, exactly the old answers (frozen references, section 6). 3.79:
  the extra goals come from the speed; why is untested.
* 3.43, 3.44, 3.45: goals keep 0.3265 m from walls; 3.31's
  frontier-search batch is unblocked. 3.69: the ProcTHOR sweep pins 99.
  3.66: `SIM_MAP=procthor:val:N`. 3.72: random house.
* 3.83, 3.85: nav2's keepout filter and the escape; marks clear at a
  mission's end. 3.89: wall-clock verbs end
  at `VERB_MAX_S` (7 s). 3.106: the in-sight livelock is bounded.
* 3.88: `explore_ask` off.
* 3.87, 3.123, 3.124, 3.126: an unchanged map is a 304 on both hops.
* 3.160.D: a find with no labeller types the rooms policy's goals from the person's room names on the saved map (asks nothing; `status.room_labels.person_rooms`). In process, oracle names: reward +0.18 (95% [+0.08, +0.30]), finds unchanged, off is off 0/200; D7 FAILED live (`out_of_budget`: the washer's laundry is off the saved map).
* 3.177.A FAILED, B dropped: CLIP RN50's whole-frame score does not rise before the target comes into view on the rig walks (object-balanced AUROC 0.45, bar 0.65; the other objects' words 0.58). 9 walks, 4 objects, no backpack judged; another scorer would need new bars.

## Safety, stops and arbitration

* 3.76: the pre-rule review is done.
* 3.159.B: a drop is a wall at the end of seen floor (`robot/drop_guard.py`,
  the depth grid's `drop` block, sim `v` cells, `*_stairs` houses); 0 falls
  in every sweep and 160 missions; D2/D4 (turns within a cell of an edge)
  failed in B. 3.159.B2-B3: the turn check measures the gap to a lip's line
  (D2 met, D4 failed: 6 of 315 turns away refused); the costmap mark failed
  its bar twice and is held (`evaluations/drop-3159/mark-held.patch`). Off
  on every real body.
* 3.164: `/health` keeps `last_stop_reason` beside the watchdog's record and
  `watchdog.trip` (a brain silent for 0.8 x `tick_timeout_s` mid-mission, kept
  until the next Start); live 30/30 hangs, 0 trips in 20 ends. The twin's rows
  are not yet deployed.
* 3.170: Settings' Shut down (`POST /power/shutdown`: a person's Stop, then 3.137's power-off, once; 403 with no secret wherever it could really power off); stub only so far, 3.137.C owes the polkit rule and `shutdown_steps`; not deployed.
* 3.171: a local brain's non-zero `/wheels` obeys the ladder (Power-off, Reserve, a check); the brain closes its mission (`POST /autonomy/mission-closed`), so a lost final `/stop` is no false trip; live 10/10 each, a control with the close lost too trips 3/3.
* 3.163.B: schedules (`control/scheduler.py`, Settings' Schedules, Home's next run); a start never lifts a Stop (`/autonomy/resume {"lift": false}`); off on a real robot (`schedules_on_robot`); an accelerated week: 15 of 15 on time and docked, 0 contacts, 6 of 6 skips with their reason; the sim charger needed 15 top-ups (the dock drifts, a limit for 3.156 and 3.163.C); not deployed.
* 3.82, 3.84, 3.86, 3.97, 3.104: a person's Stop latches the brain out
  until a mission starts (`POST /autonomy/resume`); a person's move ends a
  nav2 goal under every drive; a Stop during the arrival's confirmation
  ends it `cancelled`; the twin names each refusal.
* 3.145: a brain's nav2 goal is leased (`POST /autonomy/heartbeat` every
  0.25 s): 1 s silent ends it and latches the brain out `autonomy_lapsed`
  until a start; F6g/F7g met on the desktop and the board (10/10, 0.84-1.08 s; bar 1.1).
* 3.141.B, E1: `_PivotRows`, `_ScanPoints` (the scan's points once per scan), the corridor in numpy; exactly dev's answers (`tests/vet_reference.py`). 3.144.A: no FORWARD without a usable scan
  (`ahead_not_observed`).
* 3.137.A-B: the battery ladder (`robot/battery_ladder.py`; 10.5 / 10.2 /
  10.0 V, placeholders) and the twin's banner. 3.74.A-B: the floor band in series with the cone and
  corridor; gap: a creep or pivot can carry a sock inside its 14 cm blind
  range. 3.99: interventions logged, local only (`control.intervention_report`).
* 3.155: the robot server runs through `robot/locked_serve.py` (memory-locked,
  `MCL_ONFAULT`, only under an unlimited memlock limit); F2 on the board 0
  late wheel ticks 6/6 at 11-13 MB free. `picar-stack.service`'s
  `LimitMEMLOCK=infinity` installed on the board 2026-11-06 (its brain now `bdcf782c`).
* 3.153: a failed camera read is a budgeted transient fault
  (`CameraUnavailable`, 3 in a row end `failed`, 0.5 s between,
  `status.camera_failures`); one 503 no longer fails a mission (27/27 in
  process; live, sensors stopped 4 s: `found` as the reference).

## Missions

* 3.172.A: an escape whose REVERSE is refused and whose front is over the bar creeps forward 10 cm, once a place; the return treats 45 s of no travel as a stall, shifts, re-sends, and ends `no_motion` after 3 shifts (was 600 s). 20 of 20 reproduced traps, 0 contacts; not reproduced live. F4 held.
* 3.172.B: frontier places that stalled twice are set aside for the next 3 missions on the saved map (never for an approach; tried last, not never; local only). Live, 13 missions: aisle median 0 s (was 453), finds 8 of 11, 13 of 13 docked, 0 contacts; FAILED B4 (a survey 10 of 13 rooms, cause open).
* 3.172.C: a find goes first to where earlier missions on the saved map saw the target (`recall` goals: carried landmarks of its name, belief >= 0.8, at most 3, one try each). Live 8 finds: 8 of 8 found, all first goals recalls, median 134 s (was 502), 0 contacts; a target moved between missions not measured.
* 3.172.E: 3.158.B's 45-mission rehearsal again on `dev`: C1 26 of 26 (was 13), finds median 132 s (was 1,126), C10, C12-C14 and D4 now met, 0 contacts, 0 falls, 0 wrong; FAILED C4 (0 of 5 exhausted) and C5 (surveys 11 of 13 rooms, never laundry or pantry); 3.173.B's small-room goal refused `preempted` 30 of 30 and never drove. Surveys still end on the time limit (`out_of_budget`), never `exhausted`.
* 3.173: a survey sends a `small_room` goal (0.25 m clearance) into each region of the cut no goal stop reaches; a `preempted` refusal is waited through. Live 5 surveys: laundry 5/5, pantry 5/5 (was 0 of 9), 13 of 13 rooms each, 0 contacts, all docked; the breakfast region stalls every time (a narrower rule would save 3-7 min a survey).
* 3.158.B: the simulator rehearsal (45 missions, drop guard on, stand-in tiers) FAILED C1 (13 of 26), C4 (0 of 5), C5, C10 (33 of 35), C12, C13, C14 and D4; 0 wrong found, 0 contacts, 0 falls. One chair aisle took 49.7% of mission time. Fixes: 3.172.
* 3.156.B: a mission's end goes home (`brain/dock.py`; `return_to_dock` on in
  the config, off in the runner); the dock is the start pose; a Stop or D-pad
  ends the return. Live 10/10 docked (median 0.032 m), 0 contacts, C7 20/20;
  FAILED in process C1 49/90 (stand-in nav), C5 at 0.01-0.02 V/s, C6.
* 3.156.D: a battery end charges at the dock and resumes with what it kept
  (`recharge_resume`, off; simulator stand-in only, 3.156.F on the Rover);
  in process 45/45 charges resumed, E3-E8 met; FAILED E1 live (a return ended
  `not_aligned`, cause open) and E2 (rooms 78%, finds 79% of no-drain).
* 3.156.E (code merged, step OPEN): under `world: ros` on a matched reload the
  dock is the saved map's `dock_pose` (`/slam/pose` -> the pose's optional
  `dock` -> `status.dock.source` `saved_map`; else the start pose); a mission
  that does not explore gets a return-only navigator (no goal before the end;
  no nav2 = `skipped`, `no_nav2`, never a refused start), so the return
  reserve now ends it early too; the twin's Return line and row. Met G2,
  G4 in process (0/20 differ off), G6, G7; NOT RUN G1, G3, G5, G4 live (no
  Docker). Known limit: a bridge still starting at a mission's start reads
  as `no_nav2` for that mission.
* 3.70: four end states (`found`, `exhausted`, `out_of_budget`,
  `cancelled`), read through `read_outcome()`; the budget is
  `mission_time_limit_s` (1200 s), `battery_floor_v` and, for `vision`,
  `vision_max_calls`. Survey mode; `room_label_source: cloud` off.
* 3.175.A: a target moved between missions (in process, 75 trials): a stale recall costs nothing at the median but p90 507 s over no memory (A2 FAILED), and the next mission recalls the old place again in 30 of 75 (A3 FAILED: one stale look keeps belief >= 0.8); found counts hold, 0 wrong, 0 contacts. The harness carries landmarks, not the map, so no recall went first (A5 FAILED, instrument). 3.175.B needed, with the map carried.
* 3.175.B: the moved-target sweep now carries the map (B0). `dev` as it was and with B's changes both FAILED A1-A5 (A3 41 -> 23 of 75; finds unchanged within a trial; A6, A7 met). On `dev` now: a carried landmark not yet seen this mission is held to `STALE_BELIEF` 0.7 on its first independent miss, any label (one dropout demotes a present object until a new-view sighting), and recalls go newest sighting first (`seen_seq`, kept off the S3 copy). The rest of A3 is an aborted recall spent after one try (candidate next step, not built). Pre-existing: the slow tier's `test_walk_dist_sweep` label-call pin fails on clean dev.
* 3.176.A (criteria only, nothing built): a find asked in plain words is parsed at the twin before Start (rule parser, offline; `brain/request.py` is B's), and the brain runs what was shown. A room hint is its own `room_hint` field, never `target_room`, acting only as a `rooms`-policy prior override (the hinted type set to 1.0): inert without room labels or under `rule`, never a paid call, never ends a search; a null target is refused. The 152-sentence request set (`evaluations/request-3176/`, dev 76 / test 76) is sha256-pinned by `tests/test_request_set_3176.py`. B-D's bars are numeric in the section file (parser >= 90% target / 95% room; sim right, wrong and off arms; the model arm paid, the user's go).

## ROS and maps

* 3.38-3.40: smear 0.03 m. 3.67: inflation 0.35 m.
* 3.50.D: the bridge saves the pose graph at every mission end
  (`PICAR_MAP_DIR`), localises on it from the dock at start, falls back to a
  fresh map; re-maps on entering unknown or on request (`POST /world/remap`);
  the scan-contradiction trigger is off (`PICAR_REMAP_CONTRADICTION=1`);
  `PICAR_MAP_RELOAD=0` maps from empty.
  D5 (done 2026-11-07): carried landmarks keyed by the saved map; ten
  sessions held memory (1.05x), Jetson reload 2.04 s, fall-backs carry 0;
  FAILED as written: scan time (localisation node 1.8x the mapping node),
  `home_first_floor`'s graph (54.6 MB after one session), its carry (4 of 30
  over 15 cm: session 1's placements among the dining chairs).
* 3.154: a new nav2 goal supersedes the live one with no cancel first (the
  bridge defers the old one's cancel 2 s; a Stop clears it); explore sends
  no DELETE before an approach. Desktop: approach aborted 14/30 -> 0/30.
  The board's 3.136.B run 3 owed.
* 3.105, 3.127, 3.133, 3.134: goals wait for nav2 and SLAM; a 503 names
  its `hold`. 3.136.G: nav2's goal id comes back in `POST /goal`'s reply
  (`status.explore.sent`); the desktop's image rebuilt from `dev`
  `ed6875b5` (2026-11-04), the Jetson's not yet.

## The twin, consoles and tooling

* 3.162: what a mission saw: a report written once at `MissionRunner._end`, kept on the robot's disk, on Home (the judged frame, each thing's thumbnail); room labels 81 of 123 (bar 95%: FAILED), the Jetson's RSS owed; not deployed.
* 3.132 Stages 0-4: tokens in `web-shared/ui.css`, Engineer mode, every
  string in `web-twin/strings.js`, ES modules in `web-twin/js/`, four tabs
  with a STOP bar on each, one Lab console (`control/lab.html`) that
  confirms every paid call. 3.64: older review bugs. 3.71: the Jetson
  serves the deployed twin (`picar-stack`, `picar-ngrok`).
* 3.93, 3.128, 3.129, 3.139, 3.121, 3.113: locks, claims and the fast tier
  (section 3c; 3.113 in section 6).
* 3.168: `tools/rover/`: read-only bring-up tools for L0.3-L0.6, L0.8 (udev rules, board compare, lidar capture and score, stock firmware dump, camera check), dry-run on the fakes; nothing can write to the board.
* 3.150: `tools/bash_guard.py`, a PreToolUse hook on Bash, refuses
  `PICAR_SKIP_GATE=1` / `PICAR_CLAIM_BYPASS=1`, `--no-verify` or a
  `core.hooksPath` skip on a commit or a push that can reach `dev`, and
  `vision-picar-ros:latest` built outside a clean `origin/dev` checkout
  (asks where you see it, denies in auto mode). Build `:latest` in a second
  call after creating its checkout.
