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
* 3.180.A: explore's view goals may reuse a look-from spot once, facing more than 30 deg (`VIEW_REUSE_DEG`, from either origin) from its first look, within 4 reuses a mission (`VIEW_REUSE_BUDGET`); before, any spot once used refused every later patch, which kept 3.176.B S7's right-hint misses (195, 194) from ever looking at the target. Paired against `dev` (4 arms x 200, the baseline run from inside a `dev` worktree): 0 finds lost, finds equal or up, medians equal, A6 max 4 reuses (after two amendments; the first two runs failed it at 9). 195 and 194 now find; 247 does not (the seen-record case: 3.180.B is next). Found on the way: evaluations/explore-365's `sys.path.insert(0, ".")` loads the launching checkout's brain, so a `dev` arm launched elsewhere is void; the 3.176/3.180 harnesses now refuse it, 3.176.B's S6 was re-run and holds, and 3.160.D D4 and 3.116.B c7 are undetermined (candidates).
* 3.180.B FAILED, not shipped: counting the object and wall faces camera rays reach (explore's coverage of where a target could be) found 247 but lost 9 finds elsewhere (`exhausted` 9 vs 1; view goals per mission up in every arm); the code stays on a branch, 247 stays a known miss, and a narrower try (object-sized faces only, "looked" only when seen) is a candidate in the section file.
* 3.191: an approach goal that ends `succeeded` with the robot under `STUCK_MOVED_M` from its start is a retry failure of its sighting (dropped at 3); inside `_at_target()`'s band it is kept on record instead (close-in / 3.106 path). Before, `_band_point` could answer the robot's own cell for a sighting just past the band and the goal was re-sent forever (3.70.M2 k 19: 4,662 goals, now 12 and `found`); M2's 20 unsurveyed finds otherwise unchanged under both stand-in navs (criterion 3, >= 10 m driven, FAILED as written: k 19 found first at 5.26 m). Known: a sighting 0.70-0.80 m out ended in place is given up after 3 tries, not closed in on; close-in/3.106 counters restarting on alternating frames is a pre-existing candidate.
* 3.200 (A-C): a recall retried from another side did not pay: A1 failed (surveyed Found 7 and 6 against the bar 9), so nothing ships and `dev` keeps one try; closed by the user.
* 3.213.B: explore replaces a live approach only with a sighting whose place is better supported (strictly more of the mission's sightings within `SUPPORT_M`, 0.5 m; `_keeps_approach`). Broad sweep: found 176 to 180 (rooms) and 173 to 180 (rule), p90 time to find -31%/-33%. In the simulator a false sighting is a true detection the lidar ranged on a wall or furniture. Wrong-found FAILED as a proxy (stand-off) and was accepted. Support counts ticks in sight: 3.213.C re-measures on the Rover.

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
* 3.199: D5 refuses a person's goal within 0.125 m of a zone (`GOAL_CLEAR_M` against the drawn reach 0.120953 m; `no_go_zone`, `retryable`), so no goal lands in a drawn cell (7,200 goals: 0 in a cell, was 3,905; 28 failed to plan on `dev`); a brain goal and the D-pad are untouched (G1-G8 met).

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
* 3.175.E: one more recall goal after an abort, from another point, FAILED E1-E3 and E6 (A3 23 -> 30 of 75; stale found 21 -> 17, after 19 -> 14) and is not shipped: a recall place still gets one goal. Shipped from its review: a recall goal ended by a charge trip (`set_aside`) puts its place back. Candidates (a) abort-as-miss and (b) a clearance-chosen approach side are in the section file, not opened.
* 3.175.F: an aborted recall counted as a miss (held stale for the chain) fixed A3 (23 -> 0 of 75) but FAILED F3 (stale found 21 -> 20, after 19 -> 17) and F-risk (11 of 23 unmoved targets forgotten) and is not shipped: a nearby move makes the old place a useful lead. `dev` gets the sweep's `p0_reported` field only. Candidates (c) recall-after-frontiers and (d) keep B-fix are in the section file, not opened.
* 3.184.A: a second blind request set, frozen and unscored, held out for 3.176.E (`evaluations/request-blind-3184/`, 110 rows in 16 kinds, sha256 pinned by `tests/test_request_blind_set_3184.py`; a room said before the target is expected read, a measured gap). 3.184.B scores it once after 3.176.E lands.
* 3.184.B: the blind set scored once: 3.176.E's parser (`761c2c4c`) met P1-P4 (108, 109, 0, 110 of 110; misses b020 "I think I left ..." and b036 "check the ROOM for X"); `dev`'s `bc7509f9` 97, 107, 0, 110 (P1 FAILED: polite and indirect openings, all six room-first rows). Scorer `evaluations/request-blind-3184/score.py`.
* 3.176.A (criteria only, nothing built): a find asked in plain words is parsed at the twin before Start (rule parser, offline; `brain/request.py` is B's), and the brain runs what was shown. A room hint is its own `room_hint` field, never `target_room`, acting only as a `rooms`-policy prior override (the hinted type set to 1.0): inert without room labels or under `rule`, never a paid call, never ends a search; a null target is refused. The 152-sentence request set (`evaluations/request-3176/`, dev 76 / test 76) is sha256-pinned by `tests/test_request_set_3176.py`. B-D's bars are numeric in the section file (parser >= 90% target / 95% room; sim right, wrong and off arms; the model arm paid, the user's go).
* 3.176.B: `brain/request.py` parses a find's sentence by rules (offline); the brain's `POST /mission/request/parse` serves it, and `/mission/start` takes `room_hint` (a vocabulary type or a named room on the saved map) and `request` (record only). The hint is a `rooms`-policy prior override (`hinted_prior`: the hinted room 1.0); with no hint, no labels or a room the house lacks, 0 of 200 missions differ. P1-P6 met (test 76/76; one author wrote set and parser). FAILED: S2 (mode U goals 0.833x, bar 0.8x), S5 (a wrong hint costs 1.59x median, 4.41x p90 seconds), S7 (3 of 400 hinted missions `exhausted`; cause explore's camera-seen record, not B). L1 not run (no nav2 in the cloud). No UI sends a hint yet (3.176.C). Candidates: a softer hint, a blind request set, explore's seen record, a capitalised room name gets no prior.
* 3.176.E: the find-in-plain-words parser also reads polite and indirect openings, a greeting closed by a comma, a modal after the thing, a room said before it and "look ... for X"; held out on 3.184's blind set (scored once by its author): target 108/110 (was 97), room 109/110, 0 wrong rooms; 3.176's 152 sentences unchanged. Misses kept as candidates: "I think I left ...", "check the <room> for X".
* 3.176.C (merged, NOT deployed): the twin's Find box reads a sentence through the brain's `POST /mission/request/parse` (`web-twin/js/request.js`) and shows what it understood before Start; Start sends exactly that (`target_object`, `room_hint` unless "Search everywhere", `request`). A plain target sends the body sent before; `room_hint` only for `explore`/`tiered`; empty box or no single thing: Start off. Full reading needs a brain with the parse route: an older one (the board's `bdcf782c`) answers 404 and gets the words as typed, as before. C1-C10 met (`tests/test_ui_request.py`, 15; screenshots `docs/evaluations/twin-3.176c-*.png`).
* 3.181 (3.181.A done): a blind request set for 3.176's parser, written from 3.176's rules only (`evaluations/request-3181/`, 100 sentences, sha256-pinned by `tests/test_request_blind_3181.py`): R1 target 82 of 92 FAILED (bar 90%; 3.176's own set 76/76), R2-R5 met (room 91/92, 0 wrong rooms, attributes 92/92, plain 8/8). Nine of ten target misses are polite or indirect openings kept in the target ("could you please look for", "do you know where", "I'm looking for"); candidates for 3.176: a wider opening, modals end the target, "search the <room>" as no target, and the rules to name a room said first. The score is a floor in the suite.
* 3.183: (A) `tests/test_walk_dist_sweep.py`'s slow-tier label-call pin re-pinned to 3.173.B's intended survey change (bisected to `67b775d3` and `808075da`; the walk distances unchanged). (B) `RoomGoalPolicy.extend` reads a room type without case or edge spaces, so a person's "Kitchen" gets the kitchen prior (on `dev` before, capitalised names behaved as no names: 175 of 200 missions differed from lower-case names; now 0, and lower-case and no-names arms 0 of 200 changed). Candidate: a name that is a vocabulary synonym ("Master Bedroom", "Den") still reads as no type.

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
* 3.189: a cloud session builds the ROS image with `service/slam/build-cloud.sh` (`ARG BASE_IMAGE` in the Dockerfile, default unchanged; a local base trusts the HTTPS proxy CA and takes ROS apt from the OSUOSL mirror) in ~7 min (398 s, 3.95 GB), and nav2 reached a goal on `scaled_house` (13.7 s, 0.096 m). Outcome-judged live runs are valid there; timing-judged ones are not (load peaked 4.68 on 4 cores, bar 4: FAILED). A cloud session exports `APP_SHARED_SECRET`, which fails ~40 tests: run `ready` and pushes with `env -u APP_SHARED_SECRET -u LOCAL_SECRET` (~17 min there).

## The twin, consoles and tooling

* 3.162: what a mission saw: a report written once at `MissionRunner._end`, kept on the robot's disk, on Home (the judged frame, each thing's thumbnail); room labels 81 of 123 (bar 95%: FAILED), the Jetson's RSS owed; not deployed.
* 3.132 Stages 0-4: tokens in `web-shared/ui.css`, Engineer mode, every
  string in `web-twin/strings.js`, ES modules in `web-twin/js/`, four tabs
  with a STOP bar on each, one Lab console (`control/lab.html`) that
  confirms every paid call. 3.64: older review bugs. 3.71: the Jetson
  serves the deployed twin (`picar-stack`, `picar-ngrok`).
* 3.93, 3.128, 3.129, 3.139, 3.121, 3.113: locks, claims and the fast tier
  (section 3c; 3.113 in section 6).
* 3.179: a cloud claim's host is a digest of its container's `CLAUDE_CODE_REMOTE_SESSION_ID`
  (`CLAUDE_CODE_REMOTE=true`), so cloud sessions see each other's claims as held; the desktop's
  digest is unchanged. An ended cloud container's claim stays held until the user's
  `release --force`; the 8 claims made under the old shared digest need that once. Candidate:
  release by a lease-guarded update to a "released" record (no branch delete).
* 3.179.B: where origin refuses branch deletes (the cloud: HTTP 403), `release` leaves a leased
  `released <step>` record that every claim reader takes as free and the next `claim` replaces under a
  lease (deletes still used where they work). An older checkout reads one as that step held by
  `h-released` (its section blocked until it updates). Released records pile up until deleted.
* 3.168: `tools/rover/`: read-only bring-up tools for L0.3-L0.6, L0.8 (udev rules, board compare, lidar capture and score, stock firmware dump, camera check), dry-run on the fakes; nothing can write to the board.
* 3.150: `tools/bash_guard.py`, a PreToolUse hook on Bash, refuses
  `PICAR_SKIP_GATE=1` / `PICAR_CLAIM_BYPASS=1`, `--no-verify` or a
  `core.hooksPath` skip on a commit or a push that can reach `dev`, and
  `vision-picar-ros:latest` built outside a clean `origin/dev` checkout
  (asks where you see it, denies in auto mode). Build `:latest` in a second
  call after creating its checkout.
* 3.185: `RoomGoalPolicy.extend` reads a room name the prior does not name through the shared vocabulary (`canonical_room`, whole names only), so a person's "Lounge", "Master Bedroom" or "Washroom" gets its type's prior; a name hint still matches the room's own name first. On the 100-house sweep synonym names now equal canonical names (mode R goals 9.85 -> 6.32; on `dev` before they equalled no names, 175 of 200 differing); canonical and no-names arms 0 of 200 changed; near names ("Bathroom Closet") still read as none. Reported: the no-names baseline moved 15 of 200 since 3.183.B, probably 3.180.A (unbisected).
* 3.186: the `evaluations/explore-365` harnesses (and five more scripts that put `"."` first) load the robot code of their own checkout, never the launch directory's; `harness.py` records the brain it loaded (`harness.BRAIN`, a `brain:` stderr line) and refuses `brain`/`control`/`sim`/`world`/`tests` from another checkout. Re-run from their own `dev` worktrees, 3.160.D D4 and 3.116.B criterion 7 are confirmed (200 of 200 rows each).
* 3.201: the twin shows a toast for a tap in or beside a zone, and the Home target field follows an API find (never overwriting what a person typed); deployed 2026-11-21 from `dev` `b584df63`.
* 3.192: explore reads a no-go zone as floor for its frontiers, the room cut and the goal features (the zone still vetoes a goal and a path); a find's rule is unchanged, 90 of 90 against `dev` (+0.6%). F1' (rooms) FAILED: time +16.3% (`dev` +5.08%), rate 89 vs 87; accepted as FAILED by the orchestrator under the user's delegation, no new section. C2-C11, F9, F10 met (live found, 0 entries).
* 3.207: the documentation's gaps, closed by parallel docs workers: `docs/guides/SETUP.md`, `GLOSSARY.md`, `TROUBLESHOOTING.md` (an index by symptom), `PRIVACY-AND-SECURITY.md` (a description, with open questions); `docs/decisions/README.md` (the dated decisions, indexed); READMEs for `tools/hooks`, `tools/trt`, `sim/maps` and six `service/` directories; `/spec-review` runnable (`.claude/commands/`); architecture rationale for what-I-saw, the mission report, the dock return, the room prior, the watch list, the tree, the twin's page and the local VLM; `docs/README.md` reaches every doc. Fenced parts wait on `docs/3.207-held` for the user.
* 3.209: bare `pytest` runs `tests/` only (`pytest.ini`); `brain/what_i_saw.py` has no default directory (the brain server's rule is the only one); the local VLM server names its missing variables; `docs/decisions/README.md` is not a spec to the linter; `FEATURES.html` deleted.
* 3.203: the twin's first-run page makes no robot poll (`/frame`, `/battery`) before a robot is connected: `robot/server.py` injects `<meta name="picar-robot-origin">` into the page it serves and `app.js` prefills and connects only when it is present, so a static host is never taken for a robot. `tests/test_ui_first_run_polls.py`; deployed 2026-11-21 from `dev` `06c76aa0`.
* 3.151: F2's truth recorder holds under memory pressure (sampling apart from the writes; the body memory-locked, `FMEA_TRUTH_LOCK`); F2's new record is 3.135.B's.
* 3.141.D (closed unmet): `pivot_scale`'s arithmetic, inferred; 3.141.E1's board checks stay open in CLAUDE.md 3b.
* 3.212: a live run's logs outlive its container: `tests/demo_explore.keep_logs(dest)` gzips each run directory's top-level `*.log` files (never maps, inventory, other files or symlinks; never overwrites; never raises); `evaluations/find-last-seen-370/live.py` keeps them in `<rows stem>-logs/`, any other harness by `PICAR_KEEP_LOGS=<dir>` (at exit, SIGTERM and SIGHUP included). 3.211's keepout finding corrected: one goal inside the dresser, the mask lines are the BT's costmap clears.
* 3.210: a find's second look-again round (C1, 3.192.B) also takes the places the first round left out, those first and then the rest latest first (`_left_out` in `brain/explore.py`); caps unchanged (2 rounds, 3 places a round, never a survey or a refused place). val:44 s0-s2 rooms/none are found 3 of 3 (`dev` 0 of 3); net +5 finds and -1 across 3.192's arms and a 180-house broad sweep. E2 and E4 FAILED (val:44 s1 rule/zones lost to the same cap-of-3 crowding in reverse) and were accepted by the orchestrator under the user's delegation. Known limit: with 3 a round, any order loses a true place once more than 3 compete; root cause is the false short-range sightings (a good approach is replaced in ~50% of rows), no section opened.
* 3.161: no-go zones: a person draws rectangles on the twin's map, the brain keeps them, the bridge masks them for nav2, explore never picks a goal in one, and a zone over the dock is refused; the keeper `ae065dcd` hands a stopped mission's zones back. Z1-Z4 and Z6-Z10 met; **Z5 FAILED** (finds 82 of 96 with zones against 87 without; the 3.191.A guard never fires; accepted as FAILED, a design question). A stopped mission's reduced zone set is left open.
* 3.198: the return's dock creep no longer livelocks: `brain/dock.py` `ReturnTrip` handles whole 0.30 m creep moves (`3d3e344f`); Q1-Q10 met, the exact-verbs arm identical to `dev` on 60 starts, c-1's `not_aligned` gone and docked live 0.021-0.025 m.
* 3.205: a find on a carried map re-searches by sight when its target has no inventory entry (`MissionRunner._seed_inventory`, `0157d902`): 20 of 20 found, equal to the unsurveyed arm (`dev` 12 of 20 `exhausted`); N2-N7 met, nothing else moved. **N8 (live) is invalid for load** (two runs over 4): it is owed on the desktop under its lock or the Jetson.
* 3.211: nav2's NavFn runs `use_astar: false` (`1056d348`): 0 of 840 plans failed against 44 with A*, and the live return docked. The corrected record: the 15 survey failures were one goal inside a dresser, not the keepout marks. **The shared `vision-picar-ros:latest` and the Jetson's image still hold A* until rebuilt from `dev`** (CLAUDE.md 3b).
* 3.208: the brain's robot link has a budget (`9409ae38`): a request with no HTTP answer (`RobotUnreachable`, `WorldUnreachable`, counted in `LinkHealth`) ends no mission at once; a step tick that lost one sends no move (`LinkLost`), stops the car, records no step, and nothing is ever re-sent; `failed` "robot link lost" at 3 failed ticks in a row plus 3.0 s with no answer, "robot link failing" at 20. K7-all (5% of every request) found 29 of 30 against `dev`'s 0 of 30; criterion 4's first run (`/action` only, 25 of 30) FAILED and is kept.
