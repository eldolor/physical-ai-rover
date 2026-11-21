# Glossary

Terms the specs, the plans and `CLAUDE.md` use. Each entry is one or two
lines and ends with where the term is defined; where
[`INTRODUCTION.md`](INTRODUCTION.md)'s "Words you will hear" already defines
a term, this page says "see" and does not give a second definition. Read
from the code and docs on 2026-11-21; `dev` `7c0bedf`.

Alphabetical, ignoring punctuation; phase-ID prefixes are in the table at the
end. See also [SETUP.md](SETUP.md).

---

**allocentric** -- in the world's frame (a place on the map), as against
*egocentric*. `get_pose` and `get_map` are allocentric. *Defined:*
`CLAUDE.md` section 2 rule 2; `docs/world/ARCHITECTURE.md`.

**arrival rule** -- the target centred and the lidar under 0.40 m for two
frames; then one cloud call confirms identity. *Defined:*
`brain/arrival.py`; `docs/mission/ARCHITECTURE.md`.

**arrived_unconfirmed** -- a mission outcome: parked at an arrival the cloud
never answered. Never shown as `found`. *Defined:*
`docs/mission/ARCHITECTURE.md` (the outcomes table); `control/mission_runner.py`
(3.47).

**blocked** -- a mission outcome (a fault): the safety layer kept refusing
forward (`stuck_after` forwards in a row; default 5). *Defined:*
`docs/mission/ARCHITECTURE.md`; `control/mission_runner.py` (`DEFAULT_STUCK_AFTER`).

**body** -- everything that is the robot's own: motors, wheel encoders, the
lidar, the depth camera, the battery, behind `RobotInterface`. *Defined:*
`CLAUDE.md` section 2 rule 1; `docs/body/ARCHITECTURE.md`.

**body state / world state** -- body state is what the body measures about
itself (egocentric); world state is what is known about it in the house
(allocentric). One interface each: `RobotInterface`, `WorldInterface`.
*Defined:* `CLAUDE.md` section 2 rules 1-2; `docs/world/ARCHITECTURE.md`.

**brain (the)** -- the mission service on :8001 (`control/brain_server.py`):
runs the mission, perception every frame, and the cloud on triggers; reaches
the robot only over HTTP. See `INTRODUCTION.md` for the plain-language
sense. *Defined:* `docs/ARCHITECTURE.md` (the process table);
`docs/mission/ARCHITECTURE.md`; `CLAUDE.md` section 2 rule 5.

**bridge (the)** -- the one HTTP door into the ROS container; its routes are
capped and generic pass-through routes are banned. *Defined:*
`docs/ros/ARCHITECTURE.md`; `CLAUDE.md` section 6 (ROS).

**cancelled** -- a mission end state: a person ended it, by Stop
(`cancelled_by: stop`) or by taking the robot with the D-pad or a tap
(`cancelled_by: dpad`). Deliberately not `failed`. *Defined:*
`docs/mission/ARCHITECTURE.md`; `control/mission_runner.py` (3.70 E5).

**claim** -- a step taken by one session, held as the branch `claim/<step>`
on origin; refused means stop and report. *Defined:*
`docs/guides/PARALLEL-SESSIONS.md` rules 8 and 12 (3.139).

**cloud call / `/navigate`** -- the one paid vision-model question to
Amazon Bedrock (Claude), asked only on triggers by `tiered` and every step by
`vision`. *Defined:* `docs/cloud-vision/ARCHITECTURE.md`;
`docs/policy/ARCHITECTURE.md`.

**collision_monitor** -- nav2's velocity check, in series before the robot
server's safety controller; either can only slow or stop. *Defined:*
`docs/ARCHITECTURE.md` ("The walls", wall 4); `docs/safety/ARCHITECTURE.md`.

**desktop lock** -- `.git/heavy-run.lock`, held by every heavy job on the
desktop: *exclusive* for anything whose numbers depend on timing, *shared*
for other heavy work. *Defined:* `docs/guides/PARALLEL-SESSIONS.md` rule 7
(3.128); `tools/desktop_lock.py`.

**Detector** -- see `INTRODUCTION.md` ("Words you will hear"). In the code: YOLOE
crops, then a CLIP match gate (`brain/perceive.py`); `docs/perception/ARCHITECTURE.md`.

**Digital twin / twin** -- see `INTRODUCTION.md` ("Words you will hear"). The
code is `web-twin/` (`index.html`, `app.js`), served by the robot server and
deployed static to S3 + CloudFront; it draws what the servers report and
decides nothing. *Defined:* `docs/twin/ARCHITECTURE.md`; `docs/guides/FEATURES.md`.

**dock** -- the start position the robot returns to, recorded as the mission
starts (`dock: {x_m, y_m, theta, source}`); `return_to_dock` is a brain key.
*Defined:* `docs/mission/ARCHITECTURE.md`; `docs/engineering/mission/ENGINEERING.md`.

**driver** -- who is moving the robot; every `/action` names its driver in
the `x-driver` header. See *drivers and ranks* below. *Defined:*
`robot/interface.py` (`DRIVER_PRIORITY`); `CLAUDE.md` section 6 (Architecture).

**drivers and ranks** -- `stop` is no driver: everyone's right, always.
Then, highest first: **manual (30)** `twin-dpad`, `teleop-operator` (and any
unnamed caller: a person with `curl` is ranked as a person); **autonomous
(20)** `brain`, `teleop`, `ros`; **local (10)** `twin-local-brain` (dead
since the in-browser brain was deleted 2026-09-25). Equal manual ranks pass
(two D-pad taps never fight); the autonomous rank is exclusive until it
lapses; authority lapses on silence, with no release call. *Defined:*
`robot/interface.py`; `robot/server.py` (`arbitrate`); `docs/ARCHITECTURE.md`.

**egocentric** -- in the body's own frame (distance ahead, the depth grid,
odometry, the scan). Opposite: *allocentric*. *Defined:* `CLAUDE.md` section 2
rule 2; `docs/body/ARCHITECTURE.md`.

**end state** -- one of four ways a mission ends: `found`, `exhausted`,
`out_of_budget`, `cancelled` (`END_STATES`); every other terminal outcome is
a fault or a goal reached. *Defined:* `docs/mission/ARCHITECTURE.md` (3.70).

**exhausted** -- a mission end state: nowhere reachable left to look (no
frontier, no unseen floor, no labelled room left to enter). A finding about
the house, not a failure; was `searched` before 3.70. *Defined:*
`docs/mission/ARCHITECTURE.md`; `control/mission_runner.py` (3.31, 3.70 E1).

**explore** -- the fourth policy value, the one on the hardware path: a search
over SLAM's map (frontier, view and approach goals sent to nav2). A frontier
*policy* of the same name in other words is `frontier` below. *Defined:*
`docs/policy/ARCHITECTURE.md`; `control/brain_config.py` (the policy list).

**failed** -- a mission outcome (a fault): B3.2's vision budget, B3.3's tick
deadline, or a body fault (`no_feedback`). *Defined:*
`docs/mission/ARCHITECTURE.md`.

**failsafes (B3.1, B3.2, B3.3)** -- three, each covering a different failure,
never collapsed. **B3.1** the robot watchdog: no command within
`watchdog_timeout_s` stops the motors. **B3.2** the mission runner's vision
timeout and consecutive-failure budget (the car is blind). **B3.3** the
brain server's dead-man: a tick that never returns. *Defined:*
`docs/mission/ARCHITECTURE.md` (the table at "B3.1 watchdog");
`docs/guides/AGENT-HARNESS.md`; `PLAN-brain-relocation.md`.

**found** -- a mission end state: the target seen at the arrival rule's range,
the cloud said yes, the robot parked beside it. Only a cloud yes makes it
`found`. *Defined:* `docs/mission/ARCHITECTURE.md`.

**frontier (map)** -- the edge between explored and unexplored floor on a
map; `explore` sends nav2 goals to frontiers. *Defined:*
`docs/policy/ARCHITECTURE.md`.

**frontier (policy)** -- the free, rule-based explorer (peek, prefer unvisited
directions), kept as a deterministic test of safety and memory; not extended.
The first policy to try: needs no key and no cloud. *Defined:*
`docs/policy/ARCHITECTURE.md`; `CLAUDE.md` section 5 ("Not on the critical path").

**gate (merge gate)** -- the pre-push hook that runs the spec lint (docs-only
push) or the fast test tier before a push to `dev`. *Defined:*
`docs/guides/PARALLEL-SESSIONS.md` rule 6; `tools/hooks/pre-push`.

**house (simulator)** -- one of the maps in `sim/maps/`, chosen by `SIM_MAP`:
`starter_house` (30 cm doors), `scaled_house` (90 cm; nav2 is judged here),
`home_first_floor`, `complex_house`, and `procthor:val:<0-999>`.
*Defined:* `docs/simulator/ARCHITECTURE.md`; `config/robot.yaml` (`sim_map`).

**job** -- what a mission is for: `find` (a target, or a room) or `survey`
(no target: map the house and name its rooms; ends `exhausted`, never `found`).
*Defined:* `control/mission_runner.py` (`JOBS`, 3.70 Part A).

**landmark / inventory** -- what a mission saw itself (labels and positions,
no pixels), kept for later missions on a saved map. *Defined:*
`docs/policy/ARCHITECTURE.md` (the data-flow table, 3.46, 3.50.D5).

**Lidar** -- see `INTRODUCTION.md`. The Rover's is a D500, read by
`robot/lidar_ld19.py` in the robot server, not by ROS. *Defined:*
`docs/body/ARCHITECTURE.md`.

**local VLM** -- the on-board vision-language model service
(`service/local_vlm/`, :8005) asked first for identity; the first process
killed under memory pressure; a local VLM that is down still reads as
configured, so its questions go to the paid cloud. *Defined:*
`docs/ARCHITECTURE.md` (the process table); `CLAUDE.md` section 6 (3.80).

**Mission** -- see `INTRODUCTION.md`. One `MissionRunner` runs exactly one.
*Defined:* `docs/mission/ARCHITECTURE.md`.

**odometry / pose** -- odometry is what the body says about itself and never
jumps; pose is what the world (the mapper) says about the body and jumps on
loop closure. *Defined:* `CLAUDE.md` section 2 rule 2;
`docs/world/ARCHITECTURE.md`; pinned by `tests/test_world_contract.py`.

**outcome** -- the word a mission ends with: `found`, `exhausted`,
`out_of_budget`, `cancelled` (the four end states), `room_reached`, and the
faults `blocked`, `arrived_unconfirmed`, `failed`; `running` and `idle` are not
terminal. The first outcome wins and later stops cannot rewrite it. Old
records' `searched`, `max_steps`, `stopped`, `preempted` are *read* as
`exhausted`, `out_of_budget`, `cancelled`, `cancelled` and never rewritten.
*Defined:* `docs/mission/ARCHITECTURE.md`; `control/mission_runner.py`
(`read_outcome`). Note: `nav2_aborted` is a *reason* on a dock return's
status, not an outcome (3.70 plan file; `tests/test_ui_dock.py`).

**out_of_budget** -- a mission end state: the time limit, the battery floor,
the derived step backstop, or a `vision` mission's cap on paid calls was
reached; was `max_steps` before 3.70. *Defined:* `docs/mission/ARCHITECTURE.md`.

**Phone style (ok / warn / error / neutral)** -- how the twin colours an
outcome. *Defined:* the outcomes table, `docs/mission/ARCHITECTURE.md`.

**plan** -- `docs/plans/PLAN-*.md`: the dated history the specs cite.
*Defined:* `CLAUDE.md` section 2 rule 6.

**policy** -- what picks the robot's next move in a mission: `frontier`
(rule-based, free), `vision` (the cloud picks every step, paid), `tiered`
(local YOLOE + CLIP every frame, cloud only on triggers) and `explore` (a
search over SLAM's map). `config/robot.yaml`'s `brain:` block picks per job.
*Defined:* `docs/policy/ARCHITECTURE.md`; `control/brain_config.py`.

**replay** -- a recorded walk run again under another model or through the
real mission loop (open loop: `sim/replay_robot.py`). It outlives its HTTP
response: poll `GET /recording/walks/{walk}/replays`, never stack replays.
*Defined:* `docs/recordings/ARCHITECTURE.md`; `CLAUDE.md` section 6.

**rig walk** -- a recording made with a phone on a wheeled rig at floor
height (about 10-13 cm), lens forward and level, pushed with a broom handle;
the rig height goes in the walk's note. *Defined:* `CLAUDE.md` section 5
("How to record a Stage 0 walk"); `docs/recordings/ARCHITECTURE.md`.

**room_reached** -- a mission outcome (a goal): a room-only mission reached its
room. *Defined:* `docs/mission/ARCHITECTURE.md`.

**ROS 2** -- see `INTRODUCTION.md`. Here it lives only inside `service/slam/`:
SLAM, nav2, `ros2_control`, `twist_mux`. "Plumbing goes in; judgement and
safety stay out." *Defined:* `docs/ros/ARCHITECTURE.md`; `CLAUDE.md` section 2 rule 3.

**Safety veto** -- see `INTRODUCTION.md`. The code is `robot/safety.py`'s
`SafetyController`: a depth-grid cone, the chassis' swept corridor off the
scan, a pivot guard and guarded verbs, on every path. *Defined:*
`docs/safety/ARCHITECTURE.md`; `CLAUDE.md` section 2 rule 4.

**scorecard** -- `walk_eval`'s review of a recorded walk, flagging the
failure shapes the project has hit. *Defined:*
`docs/recordings/ARCHITECTURE.md`; `control/walk_eval.py`.

**section** -- a numbered unit of `PLAN-ros-alignment.md` (`3.N`), one file in
`docs/plans/ros-alignment/`, one branch and worktree. *Defined:*
`docs/guides/PARALLEL-SESSIONS.md` rules 1-2.

**Simulator** -- see `INTRODUCTION.md`. The code is `sim/`; it drives the same
API the hardware will. *Defined:* `docs/simulator/ARCHITECTURE.md`.

**SLAM** -- see `INTRODUCTION.md`. Here `slam_toolbox`, in the ROS container.
*Defined:* `docs/ros/ARCHITECTURE.md`.

**step** -- (1) a plan row in a section's `## Steps` table
(`3.70.S`) that a session claims; (2) in the mission loop, one decision on
one frame (counted only as a backstop; the budget is time and battery).
*Defined:* `docs/guides/PARALLEL-SESSIONS.md` rule 8;
`docs/mission/ARCHITECTURE.md` ("The budget is time and battery").

**tiered** -- the policy value: local perception on every frame, the cloud
only on triggers (mission start, candidate sighting, cold search, staleness,
arrival confirmation). *Defined:* `docs/policy/ARCHITECTURE.md`;
`brain/tiered.py`.

**Touches** -- a step row's cell listing the paths the step will change
(trailing `/` a directory, `*` globs); it fences those paths from other
sessions' claims. *Defined:* `docs/guides/PARALLEL-SESSIONS.md` rule 8.

**twist_mux** -- the ROS node that arbitrates velocity inputs by rank (inside
the ROS container); the same order as the robot server's drivers. *Defined:*
`robot/interface.py` (the `ros` comment); `docs/ros/ARCHITECTURE.md`.

**unusable** -- a depth zone or sensor the body reports it cannot read. It
never enters a comparison: as a distance it would stop the robot on every
dropout; as clear it would drive through what it could not see. *Defined:*
`docs/body/ARCHITECTURE.md`; `CLAUDE.md` section 6 (M3).

**Vision model** -- see `INTRODUCTION.md`. *Defined:* `docs/cloud-vision/ARCHITECTURE.md`.

**walls (the four)** -- (1) body state through `RobotInterface` only; (2) body
state and world state kept apart; (3) ROS stays inside its container; (4)
safety in series with the robot server last. Each is enforced by a test.
*Defined:* `docs/ARCHITECTURE.md` ("The walls"); `CLAUDE.md` section 2.

**Watchdog** -- see `INTRODUCTION.md`; it is failsafe B3.1 above (1.0 s,
`watchdog_timeout_s`). *Defined:* `docs/safety/ARCHITECTURE.md`.

**world** -- the allocentric side: `WorldInterface` (`get_pose`, `get_map`),
picked by `world/factory.py` from `world.mode`. *Defined:*
`docs/world/ARCHITECTURE.md`.

---

## Phase-ID prefixes

A phase or step ID names a plan. `plan_section.py board` lists the open ones.

| Prefix | Plan | Meaning |
|---|---|---|
| `R*` (R0-R7) | `docs/plans/PLAN-ros-alignment.md` | the ROS alignment's first phases (continuous-pose simulator R0, URDF/TF R3, `ros2_control` R4, `slam_toolbox` R5, nav2 R6, the motor board R7) |
| `3.*` (3.N, 3.N.X) | `PLAN-ros-alignment.md`, one file per section in `docs/plans/ros-alignment/` | the governing plan since 2026-09-25; `3.N` a section, `3.N.X` a step in its `## Steps` table |
| `P*` | `docs/plans/PLAN-onboard-perception.md` | on-board perception (P22-P24: YOLOE + CLIP on the Jetson) |
| `S*` | `docs/plans/PLAN-sim-hardening.md` | making the simulator honest (S1-S2b vision path, S4 time in the loop, S5 noise, S7 chaos and soak, not started) |
| `B*` | `docs/plans/PLAN-brain-relocation.md` | the brain as a service (B0-B3; B3.1-B3.3 the failsafes; B4 the twin as observer; B5 systemd at boot) |
| `M*` | `docs/plans/PLAN-microduck-transplants.md` | ideas moved in from the Microduck project (M3 `unusable`, M4 arbitration by driver rank, M5 health verdicts) |
| `T*` | `docs/plans/PLAN-teleop-robot.md` | a phone camera as a body (`mode: teleop`); its T4 is history |
| `N*` | `docs/plans/PLAN-mapping.md` | mapping while searching (N1: the world map is discovered, not copied) |
| `K*` | `docs/plans/ros-alignment/3.76-legacy-review.md` | the review's findings, `K1`-`K5` batches, `K3.4`, `K5.2` a finding in one; section 3.76's own numbering, not a plan |
| `G1`-`G4` | `docs/ros/ARCHITECTURE.md` (D8), 3.24 | the four gates before ROS became the car's drive mode: G1 a reliable chain (20 live runs), G2 the same safety on either path, G3 a real fallback, G4 the suites on the Jetson. All four met. (Other sections have their own `G1...` criteria; a plan file's own table governs.) |

`PLAN-aws-cost-redesign.md`, `PLAN-guarded-verbs.md` and `PLAN-ar-guidance.md`
sit beside them; their IDs are written in their own files. The decision
records are `docs/decisions/` (0001 so far).
