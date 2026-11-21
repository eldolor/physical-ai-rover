# Troubleshooting: an index by symptom

*Plan section 3.207, workstream J (2026-11-21).*

You have a symptom and need the page that explains it. This is an index, not a
manual: each row gives what you see, the likely cause in a few words, and a
link to the section that holds the signature and the fix. It copies no
procedure and restates no threshold, so a change in the linked section does not
make this page wrong. Quoted text is copied from the code (checked with
`grep -rn`); where a spec quotes text the code no longer prints, the row
describes the symptom in words instead.

**How to use it.** Find the place you are looking at (the headings below),
scan the "You see" column, follow the link. Two link kinds recur:

* **FM** = a domain's *Failure modes and resilience targets* table in its
  `ARCHITECTURE.md`: what the component does and the bar it was accepted
  against. The anchor is `#failure-modes-and-resilience-targets`.
* **FS** = a *Failure signatures* table in an `ENGINEERING.md` (all sit under
  its `## Procedures`, so the anchor is `#procedures`) or in
  [`service/slam/README.md`](../../service/slam/README.md#failure-signatures).

The last section, [Every failure section, by domain](#every-failure-section-by-domain),
links each spec's failure sections so none is out of reach of this page.

**Contents.** [The twin](#the-twin-your-phone-or-browser) ·
[A mission's outcome](#a-missions-outcome) ·
[Starting a mission](#starting-a-mission-the-400-and-422-answers) ·
[The robot server](#the-robot-server-8000) · [The brain](#the-brain-8001) ·
[The ROS container](#the-ros-container) · [The Jetson](#the-jetson) ·
[The suite and the push gate](#the-suite-and-the-push-gate) ·
[A deploy](#a-deploy-and-the-cloud) · [Recorded walks](#recorded-walks-and-the-console) ·
[Standing traps](#standing-traps-from-claudemd)

---

## The twin (your phone or browser)

| You see | Likely cause | Where |
|---|---|---|
| The Robot view camera says "Load failed" and nothing else | The page is a different origin from the vision service, or the tunnel was restarted under the phone. The Settings connection check names the origin as the fault | [twin FM](../twin/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [cloud-vision FS](../engineering/cloud-vision/ENGINEERING.md#procedures) · [operations FS](../engineering/operations/ENGINEERING.md#procedures) (tunnel line) |
| The connection check says the vision service "publishes no CORS headers" | Right for the deployed service; the check flags every cross-origin vision URL, including a local service and the tunnel's route that do publish them | [twin FM](../twin/ARCHITECTURE.md#failure-modes-and-resilience-targets) (a stated gap) |
| "The robot server rejected that secret." / "The robot didn't accept that access code." (the host is named) | The endpoint answered, so the secret is wrong for that deployment. It is not a network fault | [twin FM](../twin/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [control-api FS](../engineering/control-api/ENGINEERING.md#procedures) (401 row) |
| "Secret rejected" from a Jetson-served twin | The twin holds the Mac's `LOCAL_SECRET`; the Jetson has its own file. (The current page text is the line above; the README quotes the older wording) | [`tools/jetson/README.md`](../../tools/jetson/README.md) section 5 ("The secret must be the Mac's") |
| "Lost the autopilot — retrying." / "Lost the brain service — retrying." (Start is disabled with the reason) | The brain stopped answering; the twin retries by itself | [twin FM](../twin/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| "Not recording: the autopilot isn't connected." / "...the brain service isn't connected." | Recording needs a brain; a walk is never silently unrecorded | [twin FM](../twin/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [recordings FM](../recordings/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| "odometry: not reported by this server" | A server older than the readout's route; absence never reads as zero | [twin FM](../twin/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [control-api FM](../control-api/ARCHITECTURE.md#failure-modes-and-resilience-targets) (a client newer than the server) |
| "The robot's navigation is still starting — try again in a few seconds." (hold `nav2_starting`) | nav2 activates about 10 s after the map is ready; the bridge refuses goals until then | [ros FM](../ros/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| The camera will not start | An insecure origin: the camera needs https or localhost | [twin FM](../twin/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| Guide me stops by itself | The per-session call cap: unattended spend is bounded | [twin FM](../twin/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| The page runs an old script after a deploy | The CDN invalidation has not completed | [A deploy](#a-deploy-and-the-cloud) below |
| The SLAM error readout shows "0.0 cm" after driving | The world's anchor fell back to `first_contact`: the bridge has no `start_truth` | [slam FS](../../service/slam/README.md#failure-signatures) · [world FS](../engineering/world/ENGINEERING.md#procedures) |

## A mission's outcome

The words a finished mission can carry (`control/mission_runner.py`):
`found`, `exhausted`, `out_of_budget`, `cancelled` (with `cancelled_by`:
`stop` or `dpad`), `blocked`, `failed` (with `error`), `arrived_unconfirmed`,
`room_reached`. Older records read `searched`, `max_steps`, `stopped` and
`preempted`; they are read as the new names, never rewritten.

| You see | Likely cause | Where |
|---|---|---|
| `failed`, `error` names a vision error or timeout | A run of consecutive blind steps (B3.2), or an arrival's identity check that never answered | [mission FM](../mission/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [AGENT-HARNESS](AGENT-HARNESS.md#6-the-failsafes) |
| `failed` with `brain loop hung: tick exceeded ...s` | The per-tick dead-man (B3.3) fired; also what the `tick_hang` drill shows | [mission FM](../mission/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [mission FS](../engineering/mission/ENGINEERING.md#procedures) |
| `failed`, `error` names ROS | The ROS container died or its bridge was unreachable under `drive: ros` | [ros FM](../ros/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [safety FM](../safety/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `failed` naming no fresh wheel feedback | The motor board has stopped reporting; nothing moves until frames return | [body FM](../body/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [control-api FS](../engineering/control-api/ENGINEERING.md#procedures) (`no_feedback` row) |
| `blocked` | The safety layer refused FORWARD a run of times (a wall or door jamb on the straight line); going around is nav2's job | [policy FM](../policy/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [mission FM](../mission/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `blocked` after stalled moves | A wheel stalled with live feedback (a snag on a rug or cable) | [body FM](../body/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `arrived_unconfirmed`, never `found` | The robot reached the target but the cloud never answered the identity question | [policy FM](../policy/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [perception FM](../perception/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `cancelled`, `cancelled_by: dpad` | A person took the robot; the mission stops and does not retry | [mission FM](../mission/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [safety FM](../safety/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `out_of_budget` | The time limit passed or the battery read under its floor twice in a row | [mission FM](../mission/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| A phone or replay walk ends `out_of_budget` even when it "arrived" | A walk with no lidar scan never judges arrival, and a landed cloud target-reached does not end the mission. Not a navigation result | [CLAUDE.md](../../CLAUDE.md) section 3c ("Tiered phone walks end `out_of_budget`") · [policy ENG, "Run a tiered mission over a recorded walk"](../engineering/policy/ENGINEERING.md#procedures) · [policy FM](../policy/ARCHITECTURE.md#failure-modes-and-resilience-targets) (no range sensor) |
| `exhausted` when the target is in the house | Explore ran out of reachable frontier. On the car explore reads `frame["detections"]`, which only the simulator provides | [CLAUDE.md](../../CLAUDE.md) section 3b ("Explore reads `frame[\"detections\"]`") · [world FM](../world/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| The free explorer ends `NOT FOUND after 150 steps` with no `Blocked` lines | The agent got no world; the right-hand rule wanders | [policy ENG](../engineering/policy/ENGINEERING.md#procedures) ("Failure signatures" after the explorer demo) |
| A run of `Blocked LEFT ... (scan_footprint)` lines | A refused turn is being chosen again (fixed 2026-10-03; the regression test is named in the spec) | [policy ENG](../engineering/policy/ENGINEERING.md#procedures) |
| A mission is alive but not ticking; the health verdict says unhealthy | The guard loop stopped or the tick overran | [operations FM](../operations/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [AGENT-HARNESS](AGENT-HARNESS.md#6-the-failsafes) |

## Starting a mission: the 400 and 422 answers

`POST /mission/start` refuses at the door what would otherwise fail after a
motor turns. The messages below are the ones in `control/brain_server.py`,
`control/mission_runner.py` and `control/drills.py`; the table is
[mission ENG, Failure signatures at start](../engineering/mission/ENGINEERING.md#procedures).

| You see | Likely cause | Where |
|---|---|---|
| 400 `The vision policy needs a vision service. Set brain.vision_url ...` (also `tiered`) | `vision_url` is empty | [mission FS](../engineering/mission/ENGINEERING.md#procedures) |
| 400 `... takes a target_object ...` | A room-only mission, or a blank target, under a cloud policy | [mission FS](../engineering/mission/ENGINEERING.md#procedures) |
| 400 `...characters; the vision service refuses it` | A target over the length cap under `vision` or `tiered` | [mission FS](../engineering/mission/ENGINEERING.md#procedures) |
| 400 `model_id '...' is not offered by the vision service. Available: ...` | A bad model (the same shape for `prompt_variant`) | [mission FS](../engineering/mission/ENGINEERING.md#procedures) · [cloud-vision FM](../cloud-vision/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| 400 `The tiered policy cannot start: ... pip install -r requirements-perception.txt` | The perception extras are not installed | [mission FS](../engineering/mission/ENGINEERING.md#procedures) · [perception FM](../perception/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| 400 `The tiered policy could not load its perception models: ...` | Weights failed to load or download, or a bad `perception_*` value | [mission FS](../engineering/mission/ENGINEERING.md#procedures) |
| 400 `Unknown policy: '...'. Known: ...` / `Unknown fault: '...'. Known drills: ...` | A bad `policy` or `fault` value | [mission FS](../engineering/mission/ENGINEERING.md#procedures) |
| 403 naming `brain.allow_drills` | A drill asked of a brain with drills off | [mission ENG](../engineering/mission/ENGINEERING.md#procedures) (drills) · [AGENT-HARNESS](AGENT-HARNESS.md#6-the-failsafes) |
| 422 naming `extra_forbidden`, or `int_parsing` / `int_type` | An unknown field, or a wrong type, in the start body | [mission FS](../engineering/mission/ENGINEERING.md#procedures) |
| `ValueError: Unknown keys in config brain block: [...]` at boot | A typo in `config/robot.yaml`'s `brain:` block. In the walks Lambda it is a cold-start failure: yaml and `control/brain_config.py` must move together | [mission FS](../engineering/mission/ENGINEERING.md#procedures) · [CLAUDE.md](../../CLAUDE.md) section 6 ("`config/robot.yaml` and `control/brain_config.py` move together") |
| The start button sits on "Starting" while weights download | The first tiered start downloads models inside the start call. Warm them first. (The spec quotes a `Starting...` label the code does not print: reported, not indexed by text) | [operations ENG](../engineering/operations/ENGINEERING.md#procedures) · [perception ENG](../engineering/perception/ENGINEERING.md) (Procedures) |

## The robot server (:8000)

| You see | Likely cause | Where |
|---|---|---|
| 401 on every route but `/` and `/health` (`Missing or invalid x-app-secret header.`) | `APP_SHARED_SECRET` is set and the client is not sending `x-app-secret`. With the secret unset the gate is inert | [control-api FS](../engineering/control-api/ENGINEERING.md#procedures) · [control-api FM](../control-api/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| 503 on `/frame`, other routes fine | The camera, not the server: a stalled teleop phone, or no driver on the car | [control-api FS](../engineering/control-api/ENGINEERING.md#procedures) · [body FM](../body/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| 400 on `/teleop/frame` | The robot is not `mode: teleop` | [control-api FS](../engineering/control-api/ENGINEERING.md#procedures) |
| 501 on `/world/goal` or `/sim/objects` | The world is not `ros`, or the body is not a sim body | [control-api FS](../engineering/control-api/ENGINEERING.md#procedures) · [world FS](../engineering/world/ENGINEERING.md#procedures) |
| `ValueError: world mode 'sim' needs the grid-world robot ...` at start-up | The shipped `world.mode: sim` on a body with no sim house (teleop, or hardware). Set `WORLD_MODE=none`. `WORLD_MODE=ros` or `none` with the fake board | [control-api FS](../engineering/control-api/ENGINEERING.md#procedures) · [world FS](../engineering/world/ENGINEERING.md#procedures) · [motor-board FS](../engineering/motor-board/ENGINEERING.md#procedures) |
| `ValueError: SIM_MOTOR_BOARD=fake runs the simulated body as separate programs ...` | `SIM_BODY_URL` / `SIM_SENSORS_URL` unset; start `sim/body_server.py` and `sim/sensor_server.py`, or use `service/tunnel/run.sh` | [control-api FS](../engineering/control-api/ENGINEERING.md#procedures) · [motor-board FS](../engineering/motor-board/ENGINEERING.md#procedures) · [simulator FM](../simulator/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `RuntimeError: no sim/body_server.py at ...` (or `sensor_server.py`) | The robot server waited 30 s for that program's `/health`. A sensor program dying with `no simulated body state ...` needs the body started first | [control-api FS](../engineering/control-api/ENGINEERING.md#procedures) · [motor-board FS](../engineering/motor-board/ENGINEERING.md#procedures) |
| FORWARD vetoed everywhere under the fake board; the scan reads `usable: false` | The sensor programs' bundle is older than `SENSOR_STALE_S` (0.15 s): one is dead or overloaded. Fail-safe by design | [control-api FS](../engineering/control-api/ENGINEERING.md#procedures) |
| `/health` `sim_map` null where a live suite expects a house | No sim house behind the body; a live suite skips on it, and a skip is not a pass | [control-api FS](../engineering/control-api/ENGINEERING.md#procedures) · [simulator FM](../simulator/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `ValueError: mode: hardware needs ROBOT_SERIAL ...` | The serial port is not set: set it, or `SIM_MOTOR_BOARD=fake` | [motor-board FS](../engineering/motor-board/ENGINEERING.md#procedures) · [body FM](../body/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `PermissionError` / EACCES opening the port | The user is not in `dialout`, or the udev rule is missing | [motor-board FS](../engineering/motor-board/ENGINEERING.md#procedures) |
| `/action` or `/wheels` refused `no_feedback`; `motor_board.fresh` false | The board has not reported for `stale_after_s`. `link_error` set: the serial line is gone. Null with `age_s` growing: it hears commands and no longer reports | [control-api FS](../engineering/control-api/ENGINEERING.md#procedures) · [body FM](../body/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `get_wheel_state()` stays `usable: false` | No `1001` frames arrive: wrong port or baud, feedback off, or another process holds the port | [motor-board FS](../engineering/motor-board/ENGINEERING.md#procedures) |
| `board_reboots` climbing | Brownouts: check the pack and the motor stall current | [motor-board FS](../engineering/motor-board/ENGINEERING.md#procedures) · [motor-board FM](../motor-board/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `Blocked FORWARD: distance=0.0cm < min=...cm (distance_sensor)`, or `forward clamped: 0.0cm <= ...cm (distance_sensor)` | A real board with no distance sensor reads 0.0, an obstacle at the bumper. Expected until the lidar is wired; turns still move | [motor-board FS](../engineering/motor-board/ENGINEERING.md#procedures) · [platform FM](../platform/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `Blocked REVERSE: rear clearance=0.0cm ... (astern_not_observed)`, or `settle refused ...` | No lidar scan yet: the safety layer will not back up blind | [motor-board FS](../engineering/motor-board/ENGINEERING.md#procedures) · [safety FM](../safety/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| A move stops short, or `Blocked <action>: ...` | The safety layer's cone or swept corridor vetoed it. Judge it on ground truth, not on the readings the veto uses | [safety FM](../safety/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [CLAUDE.md](../../CLAUDE.md) section 6 ("Safety") |
| A turn that will not complete near furniture | The pivot guard slowed or stopped a closing turn | [safety FM](../safety/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `seconds_since_watchdog_poll` growing | The watchdog task is dead; restart the server | [control-api FS](../engineering/control-api/ENGINEERING.md#procedures) · [control-api FM](../control-api/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `wheel_loop.late_ticks` above 0 | The event loop was starved while the wheels turned. A sync route returning a big dict is encoded on the loop; return a built `JSONResponse` | [control-api FS](../engineering/control-api/ENGINEERING.md#procedures) · [CLAUDE.md](../../CLAUDE.md) section 6 ("Carried from merged sections") |
| A driver's command refused `preempted` | A higher-ranked driver holds the robot (a person over the brain). Wait for the lapse, which takes `watchdog_timeout_s`. The mission is never retried | [safety FM](../safety/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [AGENT-HARNESS](AGENT-HARNESS.md#41-who-is-driving----the-authority-order--phase-m4) |
| `/wheels` or a verb refused `unsupported` | The body has no motors; never a silent no-op | [control-api FM](../control-api/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| The motors stop about a second after the brain or twin goes quiet | The watchdog working as designed (B3.1) | [AGENT-HARNESS](AGENT-HARNESS.md#6-the-failsafes) · [safety FM](../safety/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| A stop landed and the wheels kept turning for a moment | A known window between the verb loop's check and its next command; the watchdog ends it | [body FM](../body/ARCHITECTURE.md#failure-modes-and-resilience-targets) (the "stop during a verb" row) |
| A pose or map answered "unusable" | The mapper or the bridge is unreachable. No stale or invented pose is reported | [world FM](../world/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `{"accepted": false, "reason": "no SLAM session yet"}` | The bridge has not yet published a map-frame pose | [world FS](../engineering/world/ENGINEERING.md#procedures) |
| `{"accepted": false, "reason": "preempted"}` | The brain or a person holds the robot | [world FS](../engineering/world/ENGINEERING.md#procedures) |
| `POST /world/goal` answers 501 | The world is not `ros` | [world FS](../engineering/world/ENGINEERING.md#procedures) |
| The sim runs in the wrong house, or a house name is refused | Start-up refuses an unknown house or mover; `/health` reports the house actually built | [simulator FM](../simulator/ARCHITECTURE.md#failure-modes-and-resilience-targets) |

## The brain (:8001)

| You see | Likely cause | Where |
|---|---|---|
| Status `vision_failures` rising; the mission steps stop the car | The vision call errors or hangs; every blind step stops the car (B3.2) | [mission FM](../mission/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [AGENT-HARNESS](AGENT-HARNESS.md#6-the-failsafes) |
| The brain restarted and the mission is gone | Accepted gap: the mission is lost, the robot keeps its own state | [mission FM](../mission/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| A move arrives after the mission ended | Should not: the halt gate refuses it. A command sent just before the stop is what B3.1 is for | [AGENT-HARNESS](AGENT-HARNESS.md#6-the-failsafes) · [mission FM](../mission/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| Cloud answers are late or missing, status `cloud.state` `unreachable` | The vision service's free `/health` is not answering | [AGENT-HARNESS](AGENT-HARNESS.md#8-the-status-contract) · [mission FM](../mission/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| A local VLM is down but the cloud is being asked its questions | A local VLM that is down still reads as configured: its questions go to the paid cloud | [CLAUDE.md](../../CLAUDE.md) section 6 ("Carried from merged sections") · [MERGED-STATE](MERGED-STATE.md#identity-and-perception) |
| A detection is a false positive and the robot steers at it | Steering has no frame hysteresis; only `found` is gated, by the arrival rule and the cloud's identity check | [policy FM](../policy/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [perception FM](../perception/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| The camera frame is "unavailable" and the policy never calls the cloud | Local perception is down; a dead camera never reads as an empty room | [perception FM](../perception/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [policy FM](../policy/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| Missions run without a map and explore badly | The world is unreachable; the right-hand rule takes over, and a mapper outage costs efficiency, not a mission | [policy FM](../policy/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [mission FM](../mission/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| The robot spins and never advances | The spin guard forces a forward after a fixed rotation; `turns.spinning` in the status shows it | [policy FM](../policy/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [policy ENG](../engineering/policy/ENGINEERING.md#procedures) ("Read a live mission") |
| Inventory landmarks missing or not placed | An observation with no lidar range, no pose or another map is counted, not placed | [perception FM](../perception/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| Perception over its latency budget on the board | Over 250 ms a frame at 15 W | [perception FM](../perception/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [platform FM](../platform/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| Metrics rows missing | The metrics service was unreachable; a row is dropped and never fails a mission | [operations FM](../operations/ARCHITECTURE.md#failure-modes-and-resilience-targets) |

## The ROS container

Every signature below is in one table,
[`service/slam/README.md`, Failure signatures](../../service/slam/README.md#failure-signatures);
the [ros engineering spec](../engineering/ros/ENGINEERING.md#procedures) points
there and holds no second copy.

| You see | Likely cause | Where |
|---|---|---|
| Container log `POST /wheels to ... failed (N in a row) -- still trying` | The robot server is down or unreachable (Linux without host networking), or rejects the secret (401: was `APP_SHARED_SECRET` exported before `docker run`?) | [slam FS](../../service/slam/README.md#failure-signatures) |
| Container log `the robot server reports no wheels (usable: false)` | A body with no wheels, or a robot server older than the `awaiting_feedback` handling. It is not a sign of `drive: direct` | [slam FS](../../service/slam/README.md#failure-signatures) · [ros FM](../ros/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `odom_age_s: null` from the start; `picar` not active; `drive.ros_up` never true | Deactivated at start-up by a "no wheels" read | [slam FS](../../service/slam/README.md#failure-signatures) |
| Controllers read `active` but nothing moves; `drive.ros_up` false with `ros_post_age_s` climbing | Deactivated mid-run; an image older than 3.34. Rebuild | [slam FS](../../service/slam/README.md#failure-signatures) |
| `drive.ros_up` false, `drive.bridge_up` false, `ros_post_age_s` small | The bridge failed while the plugin still posts; a person drives on the direct path, autonomy is refused | [slam FS](../../service/slam/README.md#failure-signatures) · [ros FM](../ros/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [safety FM](../safety/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `/action` refused with reason `ros_unavailable` | `drive: ros` and the bridge did not accept the twist; also an `/action` with no `x-driver` header (the bridge answers 400 `unknown driver ...` and does not mark ROS down) | [slam FS](../../service/slam/README.md#failure-signatures) · [control-api FM](../control-api/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| The robot moves again after Stop during a nav2 goal; log `stop: could not end the nav2 goal yet, retrying` | The bridge is not answering and the hold stays on; or the stop was the brain's, which spares a goal by design | [slam FS](../../service/slam/README.md#failure-signatures) · [safety FM](../safety/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| nav2 log `Transform data too old` and the robot never moves | `map -> odom` is stale: the image lacks the pinned `slam_toolbox` (`restamp_tf`). Rebuild | [slam FS](../../service/slam/README.md#failure-signatures) |
| nav2 "reaches" every goal in about 0.08 s | TF listeners frozen: the tf2 deadlock, or Fast DDS instead of Cyclone. Do not "simplify" the Dockerfile back to apt | [slam FS](../../service/slam/README.md#failure-signatures) · [ros FM](../ros/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [CLAUDE.md](../../CLAUDE.md) section 6 ("ROS") |
| Goals abort "off the global costmap" | nav2 cannot plan into a room SLAM has not seen. Map first | [slam FS](../../service/slam/README.md#failure-signatures) · [world FS](../engineering/world/ENGINEERING.md#procedures) |
| `/diagnostics` shows the brain STALE | `BRAIN_URL` prefix mismatch, or the brain is down | [slam FS](../../service/slam/README.md#failure-signatures) |
| `/world/pose` reads `usable: false` under `WORLD_MODE=ros` | The bridge is unreachable, or there is no `map -> base_footprint` transform yet | [slam FS](../../service/slam/README.md#failure-signatures) · [world FS](../engineering/world/ENGINEERING.md#procedures) |
| Log `no start truth recorded in the ...` | The bridge recorded no `start_truth` in its window; the readout falls back (see the twin's "0.0 cm") | [slam FS](../../service/slam/README.md#failure-signatures) |
| House-frame goals land off by about 20 degrees for a whole session | An image older than 3.36. Rebuild | [slam FS](../../service/slam/README.md#failure-signatures) |
| `odom_age_s` climbs to minutes while `ros2 topic hz` shows 20 Hz | An image older than 3.36: a blocking scan poll starved the odometry subscription. Rebuild | [slam FS](../../service/slam/README.md#failure-signatures) |
| The pose drifts or jumps in a big furnished room while odometry stays right | Scan matching locked onto a wrong pattern. Jumps are limited; drift is an open item | [ros FM](../ros/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| A live ROS suite skips | The stack is not up, or the secret is exported as `APP_SHARED_SECRET` instead of `LOCAL_SECRET` | [slam README, section 5](../../service/slam/README.md#5-instruments-and-live-tests) |

## The Jetson

| You see | Likely cause | Where |
|---|---|---|
| Setup stops with `STOP: expected L4T R36 (JetPack 6.x)` | Not JetPack 6 | [platform ENG](../engineering/platform/ENGINEERING.md#procedures) · [`tools/jetson/README.md`](../../tools/jetson/README.md) section 3 |
| `torch.cuda.is_available() is False`; `a requirement replaced torch with a CPU build`; `CLIP is not on cuda`; `the detector ran on <device>, not cuda` | No working CUDA torch, or a model silently on the CPU: the bring-up stops at the check | [platform ENG](../engineering/platform/ENGINEERING.md#procedures) · [platform FM](../platform/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `budget 250 ms: OVER` from the bench | Perception too slow at the power mode used | [platform ENG](../engineering/platform/ENGINEERING.md#procedures) · [platform FM](../platform/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `missing ~/.vision-picar-local-secrets` from `run.sh` or the install script | The secrets file is absent | [operations ENG](../engineering/operations/ENGINEERING.md#procedures) |
| `ERR_NGROK_334` | A second ngrok agent on the one free domain: stop the other machine's | [`tools/jetson/README.md`](../../tools/jetson/README.md) section 5 · [operations ENG](../engineering/operations/ENGINEERING.md#procedures) |
| A bench refuses to run | Another session holds the board lock | [`tools/jetson/README.md`](../../tools/jetson/README.md) section 4 · [CLAUDE.md](../../CLAUDE.md) section 3c ("One session on the Jetson at a time") |
| G4 counts a run that does not read `18 passed` | A skip is not a pass: a missing brain, `SIM_BOARD_FIRMWARE=fork`, `SIM_MAP`, container name or secret each forces a skip or a failure | [`tools/jetson/README.md`](../../tools/jetson/README.md) section 4 (the requirements table) |
| The robot server refuses to start by hand with the fake board | `SIM_BODY_URL` / `SIM_SENSORS_URL` unset; use `run.sh` | [`tools/jetson/README.md`](../../tools/jetson/README.md) section 4 · [control-api FS](../engineering/control-api/ENGINEERING.md#procedures) |
| The pack sags or the board browns out under motor peaks; the lidar zero faces left | Placeholders to measure on the Rover | [platform FM](../platform/ARCHITECTURE.md#failure-modes-and-resilience-targets) |

## The suite and the push gate

| You see | Likely cause | Where |
|---|---|---|
| The UI tests skip, with "UI tests need `pip install pytest-playwright` + `playwright install chromium`" | No Playwright or no browser; the rest of the suite still runs. Without the browser installed the Jetson run saw them ERROR, not skip | [CLAUDE.md](../../CLAUDE.md) section 1 · [platform ENG](../engineering/platform/ENGINEERING.md#procedures) · [`tools/jetson/README.md`](../../tools/jetson/README.md) section 3 |
| Tests disagree with a colleague's on a few FastAPI cases | The system Python, not `.venv`: trust `pytest` from `.venv` | [CLAUDE.md](../../CLAUDE.md) section 1 |
| A timing test passes alone and fails in the suite | It has not been run beside its neighbours | [CLAUDE.md](../../CLAUDE.md) section 6 ("Tests and coverage") |
| A claim hook or the push hook refuses an edit, commit or push | No claim on the step, or the gate has not run (`ready`). Stop and report; never bypass | [PARALLEL-SESSIONS](PARALLEL-SESSIONS.md) (rules 10 and 12) · [CLAUDE.md](../../CLAUDE.md) section 3c |
| A push warns of code commits with no recorded review | `plan_section.py reviewed` was not run | [CLAUDE.md](../../CLAUDE.md) section 6 ("Code review") |
| The spec lint reports an error or a W2 warning | A mechanical spec rule | [docs/README.md](../README.md) · `tools/spec_lint.py` |
| Pytest by the dozen fail with 401 when run | `APP_SHARED_SECRET` is in pytest's environment; export `LOCAL_SECRET` | [slam README, section 5](../../service/slam/README.md#5-instruments-and-live-tests) |
| The recordings or perception numbers cannot be compared | GPU and CPU rows are never merged; the frame set must be pinned first | [CLAUDE.md](../../CLAUDE.md) section 3c ("Perception numbers") |

## A deploy and the cloud

| You see | Likely cause | Where |
|---|---|---|
| A `web-twin/` change is not live | A change is shipped only when `sync.sh` ran and the invalidation reports `Completed`; compare `curl .../app.js` with `git show HEAD:web-twin/app.js` | [CLAUDE.md](../../CLAUDE.md) section 6 ("The deployed twin is static") · [operations FM](../operations/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [twin FM](../twin/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| A new route works locally and is dead when deployed | The route has no API Gateway or CloudFront entry; the failure is quiet. The route-guard tests catch it | [CLAUDE.md](../../CLAUDE.md) section 6 ("A new public route needs its route-guard entry") · [operations FM](../operations/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| A page loads but a file it references is missing | The publisher did not upload it; a test compares the pages' references with the manifest | [operations FM](../operations/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `STOP: <name> is empty ...` from `build.sh` | An export line was skipped before a deploy | [operations ENG](../engineering/operations/ENGINEERING.md#procedures) |
| An unauthenticated request to a gated route succeeds instead of 401 | A deploy with an empty secret parameter omits `APP_SHARED_SECRET`: no authentication on that function | [operations ENG](../engineering/operations/ENGINEERING.md#procedures) |
| `restart.sh` prints `FAILED: expected every server on <rev> ...` | A server runs old code. (The operations spec quotes `expected both servers`; the script now says `every server`) | [operations ENG](../engineering/operations/ENGINEERING.md#procedures) · [operations FM](../operations/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `OK: robot and brain both running <rev>` then `WARNING: ngrok is not running` | The tunnel is down; the deployed twin cannot reach the machine | [operations ENG](../engineering/operations/ENGINEERING.md#procedures) |
| `python -m control.health` says unhealthy or unreachable | A half is down, its guard loop stopped, or a mission is alive but not ticking. A parked robot is never unhealthy | [operations FM](../operations/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [control-api FM](../control-api/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `Runtime.ImportModuleError` at cold start | The zip was not flat, or a dependency is missing | [cloud-vision FS](../engineering/cloud-vision/ENGINEERING.md#procedures) · [recordings FS](../engineering/recordings/ENGINEERING.md#procedures) |
| 403 and nothing in the function's logs | Invocation through a resource policy (a Function URL, or an integration with no `CredentialsArn`) | [cloud-vision FS](../engineering/cloud-vision/ENGINEERING.md#procedures) |
| 403 from API Gateway through CloudFront | The viewer's `Host` header is forwarded (wrong origin request policy) | [cloud-vision FS](../engineering/cloud-vision/ENGINEERING.md#procedures) |
| 502 with `ValidationException ... data retention mode` | The model is not enabled in this region; pin it. Kept 502 on purpose | [cloud-vision FS](../engineering/cloud-vision/ENGINEERING.md#procedures) · [cloud-vision FM](../cloud-vision/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| 502 with `AccessDeniedException` | The model is in the catalogue but not invokable on the account | [cloud-vision FS](../engineering/cloud-vision/ENGINEERING.md#procedures) |
| The vision service answers `STOP`, not visible, not reached | The model replied with non-JSON: the safe default, never an arrival | [cloud-vision FM](../cloud-vision/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| 400/413 from the vision service before any model call | A model id or wording off the allow-list, an oversized image, or a bad encoding | [cloud-vision FM](../cloud-vision/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| A pinned model answers from one region only | The region pin in `vision_core.MODEL_REGIONS`; re-confirm from the deployed region | [CLAUDE.md](../../CLAUDE.md) section 6 ("Model strings differ ...") · [cloud-vision FM](../cloud-vision/ARCHITECTURE.md#failure-modes-and-resilience-targets) |

## Recorded walks and the console

| You see | Likely cause | Where |
|---|---|---|
| The console shows 0 walks after a walk | The twin's brain URL pointed at a brain with recording off | [recordings FS](../engineering/recordings/ENGINEERING.md#procedures) |
| Every save fails with 403 | `allow_recording` is false and no proxy is set | [recordings FS](../engineering/recordings/ENGINEERING.md#procedures) |
| Replay answers 503 "No vision service configured" | `vision_url` is empty in this process; expected on the deployed walks function as templated | [recordings FS](../engineering/recordings/ENGINEERING.md#procedures) |
| Replay answers 200 but the sidecar is `unusable`, frames read `HTTP 401` | `VISION_SHARED_SECRET` is unset, so frames were signed with the walks secret | [recordings FS](../engineering/recordings/ENGINEERING.md#procedures) |
| Replay answers 200 but the sidecar is `unusable`, frames read `HTTP 404` | `VISION_URL` points at something other than the vision service (typically the tunnel proxy on :8080) | [recordings FS](../engineering/recordings/ENGINEERING.md#procedures) |
| A replay row shows `unusable` / `incomplete` | Coverage below the bar; per-frame reasons are in `diff[].error` | [recordings FS](../engineering/recordings/ENGINEERING.md#procedures) · [recordings FM](../recordings/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| The walks function fails at import with `S3WalkStore needs a bucket name.` | `RECORDING_BACKEND=s3` with no `RECORDING_BUCKET`: the stack was deployed without the bucket import | [recordings FS](../engineering/recordings/ENGINEERING.md#procedures) · [recordings FM](../recordings/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| `aws s3 sync` uploads nothing after a label edit | `--size-only` was used and the edit kept the file's size | [recordings FS](../engineering/recordings/ENGINEERING.md#procedures) |
| A walk ended with no finish marker and is not scored | A closed tab or a dead battery; a person's Evaluate scores it | [recordings FM](../recordings/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| A frame is counted "dropped" or "orphaned" | It landed after its walk ended; it is never filed under another walk | [recordings FM](../recordings/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| A deleted walk or frame must come back | Previous object versions remain for the retention window | [recordings FM](../recordings/ARCHITECTURE.md#failure-modes-and-resilience-targets) · [recordings ENG](../engineering/recordings/ENGINEERING.md#procedures) |
| A name is refused | A separator or parent element: no traversal on either backend | [recordings FM](../recordings/ARCHITECTURE.md#failure-modes-and-resilience-targets) |
| A replay that never finishes seems to have vanished | A replay outlives its HTTP response: poll `GET /recording/walks/{walk}/replays` for a fresh `replayed_at`, and never stack replays | [CLAUDE.md](../../CLAUDE.md) section 6 ("A replay outlives its HTTP response") |

## Standing traps from CLAUDE.md

These read as symptoms and are recorded in `CLAUDE.md`; the rows above link
them where they surface, and this list is the one place they sit together.

| You see | Likely cause | Where |
|---|---|---|
| A phone walk ends `out_of_budget` | No lidar scan, so arrival is never judged | [section 3c](../../CLAUDE.md#3c-standing-facts-a-session-needs) |
| The deployed twin does not show a change | The invalidation has not reported `Completed` | [section 6](../../CLAUDE.md#6-things-to-know-before-touching-the-code) (Cloud and deployment) |
| A `ValueError` at the walks Lambda's cold start | An unknown config key: `config/robot.yaml` and `control/brain_config.py` did not move together | [section 6](../../CLAUDE.md#6-things-to-know-before-touching-the-code) · [mission FS](../engineering/mission/ENGINEERING.md#procedures) |
| A feature shipped dead | A route has no route-guard entry | [section 6](../../CLAUDE.md#6-things-to-know-before-touching-the-code) |
| A UI test skips | No Playwright browser | [section 1](../../CLAUDE.md#1-setup) |
| A local VLM that is down still reads as configured | Its questions go to the paid cloud | [section 6](../../CLAUDE.md#6-things-to-know-before-touching-the-code) (Carried from merged sections) |
| `late_ticks` after a big route reply | A sync route returning a dict blocks the wheel loop | [section 6](../../CLAUDE.md#6-things-to-know-before-touching-the-code) |
| A tiered mission seems to ignore a hold-bearing idea | `tier_hold_bearing` stays OFF; a 1-in-3 detector needs a range-anchored goal | [section 3c](../../CLAUDE.md#3c-standing-facts-a-session-needs) |
| A model's `obstacle_ahead` is 0 or 100% on the same frames | Uncalibrated; the lidar is the obstacle sensor | [section 3c](../../CLAUDE.md#3c-standing-facts-a-session-needs) |
| A live run is invalid | Laptop load above about 4, or no `PICAR_ROS_NETWORK=host` on Linux | [section 6](../../CLAUDE.md#6-things-to-know-before-touching-the-code) (Live runs) |

---

## Every failure section, by domain

Each spec's failure sections, so that nothing a spec states is out of reach
of this page. The rows above that reach each one are named by their group.

| Domain | Architecture: failure modes | Engineering: failure signatures | Reached from |
|---|---|---|---|
| body | [FM](../body/ARCHITECTURE.md#failure-modes-and-resilience-targets) | none stated | robot server, outcomes |
| cloud-vision | [FM](../cloud-vision/ARCHITECTURE.md#failure-modes-and-resilience-targets) | [FS](../engineering/cloud-vision/ENGINEERING.md#procedures) | a deploy, the twin |
| control-api | [FM](../control-api/ARCHITECTURE.md#failure-modes-and-resilience-targets) | [FS](../engineering/control-api/ENGINEERING.md#procedures) | robot server, ROS |
| mission | [FM](../mission/ARCHITECTURE.md#failure-modes-and-resilience-targets) | [FS at start](../engineering/mission/ENGINEERING.md#procedures) | outcomes, starting, brain |
| motor-board | [FM](../motor-board/ARCHITECTURE.md#failure-modes-and-resilience-targets) | [FS](../engineering/motor-board/ENGINEERING.md#procedures) | robot server |
| operations | [FM](../operations/ARCHITECTURE.md#failure-modes-and-resilience-targets) | [FS (deploy, tunnel)](../engineering/operations/ENGINEERING.md#procedures) | a deploy, the Jetson |
| perception | [FM](../perception/ARCHITECTURE.md#failure-modes-and-resilience-targets) | none stated | brain, starting |
| platform | [FM](../platform/ARCHITECTURE.md#failure-modes-and-resilience-targets) | [FS (setup)](../engineering/platform/ENGINEERING.md#procedures) | the Jetson |
| policy | [FM](../policy/ARCHITECTURE.md#failure-modes-and-resilience-targets) | [FS (explorer demo)](../engineering/policy/ENGINEERING.md#procedures) | outcomes, brain |
| recordings | [FM](../recordings/ARCHITECTURE.md#failure-modes-and-resilience-targets) | [FS](../engineering/recordings/ENGINEERING.md#procedures) | recorded walks |
| ros | [FM](../ros/ARCHITECTURE.md#failure-modes-and-resilience-targets) | [points to the slam README](../engineering/ros/ENGINEERING.md#procedures) | the ROS container |
| safety | [FM](../safety/ARCHITECTURE.md#failure-modes-and-resilience-targets) | none stated | robot server, outcomes |
| simulator | [FM](../simulator/ARCHITECTURE.md#failure-modes-and-resilience-targets) | none stated | robot server |
| twin | [FM](../twin/ARCHITECTURE.md#failure-modes-and-resilience-targets) | none stated | the twin |
| world | [FM](../world/ARCHITECTURE.md#failure-modes-and-resilience-targets) | [FS](../engineering/world/ENGINEERING.md#procedures) | robot server, ROS |

Outside the domains: the ROS container's one table is
[`service/slam/README.md`, Failure signatures](../../service/slam/README.md#failure-signatures);
the Jetson's bring-up stop points are in
[`tools/jetson/README.md`](../../tools/jetson/README.md); the three failsafes
are [AGENT-HARNESS section 6](AGENT-HARNESS.md#6-the-failsafes); the standing
traps are `CLAUDE.md` sections [3c](../../CLAUDE.md#3c-standing-facts-a-session-needs)
and [6](../../CLAUDE.md#6-things-to-know-before-touching-the-code).
