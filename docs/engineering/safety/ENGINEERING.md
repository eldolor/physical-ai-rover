---
kind: engineering
domain: safety
status: current
verified: 2026-11-01
parent: docs/safety/ARCHITECTURE.md
---

# Safety -- engineering

How the safety layer, driver arbitration and the watchdog are built today.
The rules and why they are shaped that way are in the
[architecture spec](../../safety/ARCHITECTURE.md). This document is true only
until the implementation changes, and is updated in the same commit that
changes it.

## Implementation

| Where | What |
|---|---|
| `robot/safety.py` | `SafetyController`: every clearance judgement, the wheel-command vet, the guarded-verb runner and the settle pass. `path_zone_indices()` and `scan_points_cm()` are module functions. `SafetyViolation` is the veto exception |
| `robot/interface.py` | The rank table `DRIVER_PRIORITY`, `driver_priority()` (unknown names rank as manual), `Preempted`, and `carry_out_verb()`, the loop `run_verb()` supplies a limit to |
| `robot/server.py`, inside `create_app()` | `authority_holder()`, `arbitrate()`, `refuse()`, `goal_in_progress()`, `ros_up()`, `watchdog_loop()`, `wheel_loop()`, and the `motion_lock` that serialises everything that moves the robot. Under ROS drive, a second `SafetyController` (`fallback_safety`) runs over the wrapped body |
| `robot/battery_ladder.py` | 3.137: `BatteryLadder`, the battery ladder's rules (pure, no I/O): two distinct samples per level, the read at rest, Reserve's timer |
| `robot/server.py`, inside `create_app()`, 3.137 | `battery_poll()`, `enter_reserve()`, `battery_refusal()`, `power_off()`, `battery_loop()`; `battery_settings()` at module level |
| `control/remote_robot.py` | Maps refusal reasons to `Preempted` / `StoppedByPerson` / `BatteryReserve` / `RobotTransportError` / `SafetyViolation` |
| `brain/agent.py` | The brain's own `SafetyController` (an early out; the server's is authoritative). Its `min_distance_cm` comes from the `brain:` block |
| `tests/footprint_sweep.py`, `tests/verb_sweep.py`, `tests/ros_verb_sweep.py` | Ground-truth instruments: the URDF rectangle against the house's occupied cells, sharing no geometry code with `robot/safety.py` |

### What each motion is vetted against

| Motion | Path | Check |
|---|---|---|
| `/action` FORWARD | `check_and_execute()` | Refused outright (`ahead_not_observed`) on a body with wheels and no usable scan (3.144). Pre-check `forward_clearance() < required_clearance_cm(v, creep)` raises, where `v` is the verb's `speed` as m/s (`verb_speed_m_s()`). Then, if the body has a `verb_plan()`, `run_verb()` re-vets every 50 ms period with look-ahead against the same bar |
| `/action` REVERSE | same | `reverse_clearance()`, then `run_verb()`. Refused outright (`astern_not_observed`) on a body with wheels and no usable scan |
| `/action` LEFT / RIGHT | same | No pre-check. `run_verb()` limits each step with `pivot_scale()` (only on bodies with a plan). With no usable scan, never limited |
| `/action` LOOK_* / STOP | `_dispatch()` | None |
| `POST /wheels` (direct drive) | `vet_wheel_velocity()` on receipt, then every `wheel_loop()` tick | Forward/reverse look-ahead and clamp; rotation scaled by `pivot_scale()`. Its own early return passes a command through while the body's wheel state is unusable; on `HardwareRobot` the body then refuses it (`no_feedback`, 3.34), so nothing reaches the board unvetted. A tick reads `stop_count` before the command and skips its write, or zeroes after it, if a `/stop` (which never takes `motion_lock`) landed in between (3.76 A2) |
| ROS drive: verbs | `check_and_execute()` -> `RosDriveRobot` (no plan) | The pre-check, then the actuator's `/wheels` posts are vetted on receipt and every tick. `_refuse_if_nothing_achieved()` turns a verb that covered under 1 cm / 0.5 deg into a `SafetyViolation`, except one `RosDriveRobot` marks `superseded` (cut off by a newer verb or a stop, 3.76 T1): `/action` answers that one `preempted` when another driver now holds the robot, else as done |
| ROS drive, ROS down: a person's verbs | `fallback_safety.check_and_execute()` on the wrapped body | Exactly direct drive's path |

### Clearance sources

Each clearance method returns `(cm or None, source)`. `None` means nothing
within range, never "unknown".

| Method | Sources | Meaning |
|---|---|---|
| `path_clearance()` | `depth_grid_facing_away`, `depth_grid`, `depth_grid_no_target`, `scan_path`, `scan_path_no_target`, `distance_sensor` | The cone, tried in that order. A panned grid with no path zones left is no opinion (`None`); otherwise the nearest `ZONE_RANGE` in the path zones; otherwise, if any path zone answered, no target; otherwise, with a usable scan, its beams within `PATH_HALF_ANGLE_DEG` of ahead minus `LIDAR_TO_FRONT_BUMPER_CM` (3.42: without it a lidar-only car fell to the scalar's 0.0 and could never drive forward); otherwise the scalar `get_distance()`. Unusable zones are dropped. All values are bumper-relative (`sensor_to_bumper_cm` subtracted, floored at 0) |
| `footprint_clearance(+1 / -1)` | `scan_footprint`, `scan_footprint_no_target`, `no_scan` | Returns in the body frame within half-width + 3 cm of the centre-line and beyond the leading edge: distance past the edge. A return inside the outline reads 0.0 |
| `rear_clearance()` | `scan_rear`, `scan_rear_no_target`, `no_rear_sensor` | Beams within `PATH_HALF_ANGLE_DEG` of astern, minus `LIDAR_TO_REAR_BUMPER_CM` |
| `floor_clearance(grid)` | `floor_band`, `floor_band_no_target`, `floor_band_unusable`, `no_floor_band` | 3.74. The depth grid's `floor` block (the depth camera's floor band: body-fixed, `origin_x_cm` ahead of `base_link`, `fov_deg`, `min_range_cm`, zones of `{status, distance_cm, bearing_deg?}`; fields in `RobotInterface.get_depth_grid()`). Each `ZONE_RANGE` zone is a point in the body frame -- range from the origin along the zone's `bearing_deg` (null or absent: its centre) -- judged by `_corridor_room()`, the one corridor rule the scan's `footprint_clearance()` uses too: ahead of the front edge within half-width + 3 cm, the room past the edge; beside the body only within the half-width (0.0). Inside the band's minimum range nothing is seen or carried (Known gaps). Unusable zones never compared. `no_floor_band` for a grid without the block or with no usable `fov_deg` (every real body today). `floor_band_unusable` (3.76 K1) when no zone could be read (blind is not clear) or the band itself cannot be: a field that does not parse, or zones that disagree with its `cols`; a malformed zone is skipped as unusable, never raised out of the check (a raise made every FORWARD a 400 and skipped the wheel loop's re-vet) |
| `forward_clearance()` | `ahead_not_observed` (0.0), `path_not_observed` (0.0), or the least of cone, corridor, floor band and drop; then aged (below) | The cone and the band read ONE depth grid, the corridor and the band ONE scan. `(0.0, "ahead_not_observed")` when the scan is missing or unusable **and** `_has_wheels()`, whatever the cone says (3.144, FMEA F9: the lidar stopped and the cone alone let the board drive 101-105 cm). Both directions ask one `_blind(scan)`. Otherwise `path_not_observed` when the grid faces away and there is no scan (a body without wheels); the band does not lift either |
| `reverse_clearance()` | `astern_not_observed` (0.0), or the least of rear, corridor astern and drop astern, off one scan; then aged (below) | `(0.0, "astern_not_observed")` when the scan is missing or unusable **and** `_has_wheels()` (the body's `get_wheel_state()` reports `usable: true`; a body without the method, or whose call raises, has none). Otherwise, with no scan, `(None, "no_rear_sensor")` and the reverse proceeds. The rule and its reasons: [architecture spec](../../safety/ARCHITECTURE.md), "A body that cannot see astern does not reverse" |
| `pivot_blocked(omega)` / `pivot_scale(omega)` | reason string or None / `(fraction, reason)` | A return whose distance to the rectangle would end under `PIVOT_MARGIN_CM` **and** shrink. Binary search to 1/64 of the turn rate. No usable scan: never blocked. **Aged rotation** (3.42): `_turn_since_scan()` is the turn the scan has missed -- the body's own answer (`turn_since_scan()`, `HardwareRobot` from its encoder heading history) or else yaw rate x `sensor_age_s()` -- and the check runs from `PIVOT_AGE_SAMPLES` + 1 rotations across that arc. Zero for a body whose readings have no age. **One set of rows for the search** (3.141): `pivot_scale()` builds a `_PivotRows` once -- the returns in the body frame across the aged arc, those within `PIVOT_NEAR_CM` of the chassis now, built in one numpy pass from the scan's exact body-frame points (3.141.E: `_ScanPoints`, below) -- and every pass of its search uses it, so `_turn_since_scan()` is read once per search. Each pass looks ahead in numpy as a filter only (`PIVOT_FILTER_SLACK_CM`); every row it passes is decided in scan order by the exact per-point expressions, so the first blocking return and its reason are the old loop's (`tests/pivot_reference.py`, `tests/test_pivot_3141.py`). Before, a turn clamped beside furniture cost ~22 ms on the board, on the wheel loop's event loop. **One conversion per scan** (3.141.E): `_ScanPoints.of(scan)` keeps the last scan's body-frame points (`scan_points_cm()`'s exact arithmetic, as read-only numpy arrays), keyed every call by the scan's angles, `LIDAR_X_M` and its ranges (no identity shortcut: a scan changed in place gets new points), so the corridor, the turn guard and a tick's two vets (the wheel loop's and `/wheels`') convert a scan once. `footprint_clearance()` is `_corridor_rooms()`, `_corridor_room()`'s rule over every return in numpy (a test pins the two equal; `floor_clearance()` keeps the scalar one) -- a product, differences and comparisons, the same IEEE operations as Python's, so the same numbers (`tests/vet_reference.py`, `tests/test_vet_cost_3141.py`). On the board an unclamped vet's turn guard was 82.5% of its time (3.141.E0) |
| `drop_clearance(direction, grid)` | `drop_edge`, `drop_unseen`, `drop_unseen_astern`, `drop_unusable`, `drop_floor_seen`, `no_drop_sensor` | 3.159 (`robot/drop_guard.py`). The depth grid's `drop` block (`cols`, `fov_deg`, `pan_deg`, `origin_x_cm`, `near_cm`, `far_cm`, zones of `{status, distance_cm, kind: edge|thing}`: how far floor runs from the lens and what ended it) is folded into the robot's floor memory (`guard_for(robot)`: one per robot, a `WeakKeyDictionary`), then the swept corridor (half-width + 3 cm, sampled every 3 cm across and 2.5 cm along, out to `RUN_CM` 60) is followed ahead or astern. Memory: 5 cm cells, `floor`/`edge`/`thing`, in the body's dead-reckoned frame (wheel travel, odometry heading); seeded with a 30 cm disc at the first reading (the dock); the body's own outline is floor every update; an edge is sticky (nothing but an edge rewrites one); cells seen more than 3 m of travel ago are pruned once 4,000 are held. Per frame, floor first, then the ends (an edge or a thing), and two neighbouring columns' thing ends within 10 cm are joined into a line. When every corridor column sees floor at the MinZ ring, the band from the bumper to the ring (half a cell wider) is floor. In the search a thing excuses unseen floor on its line and every line outward of it on that side, and an unseen cell whose sightline from the lens crosses a thing is the thing's shadow; a remembered edge on any line is always the answer. An edge or an unseen cell returns its room past the front edge **less** `min_distance_cm - CREEP_MARGIN_CM` (17 cm), so a creep keeps the full bar from a drop. Each caller passes its own margin. `no_drop_sensor` until a block has been seen: every real body today, every house without a `v` cell; `drop_unusable` (0 cm) once on with no pose (no usable wheels or odometry yet). Astern, and for a turn, the grid is fetched until the first answer (`probed`), so a first move that is a reverse or a turn finds the memory switched on; a body that showed no block then is not asked again (a house without a drop senses exactly as before) |
| `drop_pivot_blocked(omega)` | `drop_pivot_edge` or None | 3.159: a turn that carries an outline point (3 cm spacing) from remembered floor onto a remembered edge, looked at in 5-degree steps out to the larger of 5 degrees and `PIVOT_HORIZON_S` (0.1 s) of the turn; `drop_unusable` once on with no pose. Unseen floor does not refuse a turn (a corner swings 5.5 cm at most). Outside `pivot_scale()`, whose answers `tests/pivot_reference.py` freezes; `vet_wheel_velocity()` zeroes omega on it, `run_verb()` refuses the step. Off (None) until the memory is active |

**Aged readings** (`_aged(result, direction)`, `PLAN-ros-alignment.md`
3.36). The look-ahead (3.24 G2) stops a move AT the line only if the
clearance is the clearance now. A body that reports how old its readings
are (`sensor_age_s()`: `sim/body_client.py`'s `SimBodyClient`, whose
polled bundle is up to `SENSOR_STALE_S` 0.15 s old, and since 3.42
`HardwareRobot` with a lidar, whose scan is 0-150 ms old) has the way
covered since subtracted: `clearance - max(0, direction x v) x age`, with
`v` the mean wheel speed times `wheel_radius_m`, floored at 0.0 and
rounded to 0.1 cm, and the source gains `, aged <n> ms`. It is skipped,
leaving the result untouched, when the clearance is None or 0, the body
has no `sensor_age_s` or reports no age, the wheel state is unusable or
raises, or the robot is not closing (moving away, or still). Every
in-process backend reads on demand and is unchanged. 3.36's sweep fed
three-period-old readings: worst travel-to-contact 18.2 cm without it,
19.0 cm with it (`tests/test_footprint_safety.py`). On the car a lidar scan
is 0-100 ms old when it is read, which is the rule's reason beyond the
sim.

Comparison edges: the verb pre-check refuses at `< need`; the wheel vet
clamps at `<= need` and otherwise limits speed to `(clearance - need) /
0.05 s`. `run_verb()`'s limit allows `clearance - need` metres, so it ends
`clamped` at the line.

**The bar, `need` (3.44):** `required_clearance_cm(v, creep)` is
`CREEP_MARGIN_CM` (3 cm) when `creep` and `|v| <= CREEP_M_S` (0.03 m/s),
else `max(min_distance_cm, stop_distance_cm(v))` with
`stop_distance_cm(v) = 3 + 100 (|v| x 0.15 + v^2 / (2 x 0.5))` cm -- 20 cm
up to about 0.39 m/s, 25 at 0.4, 35.5 at 0.5, 48 at 0.6. `v` unknown
(None): `min_distance_cm`. `creep` is true only while a straight verb
ASKED at creep speed runs: `check_and_execute()` sets `_asked_v_m_s` and
`_creep_asked` through `_asking()` for the verb's whole run and clears them
in `finally`. The server shares one `SafetyController` between `/action`
and the wheel loop, so under ROS drive the twists of a creep verb are
vetted at the creep bar while it runs; nav2's twists, which no verb asked,
never are. The wheel vet computes `need` from the COMMANDED `v` before
slowing it, and `run_verb()` from the asked speed, so the vet's own slowing
never earns the creep margin.

### The battery ladder (3.137)

The robot server reads the pack (`robot.get_battery()`, the board's `v`) every
`BATTERY_POLL_S` of the robot's clock -- the simulator's in process, the
monotonic clock otherwise, wrapped so it never runs backwards across a house
swap -- from `battery_loop()` (off the event loop), and, only when that loop
is not running (in process: tests, evaluations), at the top of `/action`,
`/wheels` and `/world/goal`; `/battery` polls (rate-limited, forced only
before the first reading) and returns that poll's reading. Each
read goes to `BatteryLadder.observe(v, now, at_rest)`; `at_rest` is nothing
having turned the wheels for `BATTERY_REST_S`, stamped where motion happens
(a verb's start and end, a non-zero wheel command, a moving wheel-loop tick)
and sampled at the poll; unusable wheel state counts as turning.
"Autonomy" below is every rank under a person's (`!= DRIVER_MANUAL`).

| Level (`battery:`) | Crossed by | What the server does | Refusal |
|---|---|---|---|
| Low, `low_v` 10.5 | Two distinct samples under it; not latched | Reports it (`/battery` `level`); the brain refuses a start (`battery_low`) and its own `battery_floor_v` ends a mission | none here |
| Reserve, `reserve_v` 10.2 | Two distinct samples under it, at rest (else CHECK); latched | `enter_reserve()`: lapses an autonomous claim, counts a goal end (the Stop's goal-ending loop and hold, no person's latch), `robot.stop()`. Autonomy refused; a person's verb has its `speed` capped at `reserve_speed` (30), a person's `/wheels` scaled by it; no goal from anyone; `ros` wheel posts zeroed under `drive: direct` (under `drive: ros` they carry the person's verbs) | `battery_reserve` (autonomy only) |
| Power-off, `poweroff_v` 10.0, or Reserve held `reserve_max_s` (120 s) | As Reserve; latched | Everything Reserve does, then `power_off()` on its own thread, once: the wheels at rest (up to 2 s), each `shutdown_steps` command in order (60 s each), `os.sync()`, `poweroff_cmd`. Every step is recorded (`/battery` `ladder.shutdown`) | `battery_poweroff` (everyone; `/stop` still answers) |
| CHECK (a crossing read with the wheels turning) | -- | `robot.stop()`; autonomy held, and under `drive: ros` every `ros` wheel post zeroed (nav2's goal held still, not ended), until the next distinct sample: over every level clears it (a sag), under one and at rest fires the highest level the last two distinct samples are both under (one glitch at rest is not two); `check_max_s` (15 s) without a sample at rest fires if the last two are still under | `battery_check` (autonomy only, and never a camera pan; the brain reads it as a veto it does not count toward `stuck_after`) |

`battery_refusal()` is checked where `person_stop_refusal()` is: before
arbitration, and again at execution after the verb's stop baseline is set,
so a level latched in between stops the robot and ends the verb. The brain
ends a mission `out_of_budget` on `battery_reserve` / `battery_poweroff`
(`BatteryReserve`, a `Preempted`), and also on reading `level` reserve or
power-off from `GET /battery` with its floor set. `poweroff_cmd` defaults to
`systemctl poweroff` only in `mode: hardware` with a real board; the
simulator and `SIM_MOTOR_BOARD=fake` get the logged stub, so a desktop drain
test cannot power off the desktop.

### Driving without a lidar

The canonical statement of what a body that moves but has no scan may do;
other specs link here. Today that body is the car: `HardwareRobot` with no
`sensors` (no scan, the default all-unusable depth grid, `get_distance()`
0.0). Once the board's first feedback frame has arrived (`get_wheel_state()`
usable):

| Motion | Result | Why |
|---|---|---|
| FORWARD verb, or a forward component of `/wheels` | Refused / clamped, `ahead_not_observed` | `forward_clearance()` with `_has_wheels()` and no scan (3.144; before it, the cone's fallback to the scalar, 0.0) |
| REVERSE verb, a reverse component of `/wheels`, a settle pass astern | Refused / clamped, `astern_not_observed` | `reverse_clearance()` with `_has_wheels()` and no scan |
| LEFT / RIGHT verb, rotation in `/wheels`, a turning settle pass | Allowed, never limited | `pivot_blocked()` / `pivot_scale()` never refuse with no usable scan |

So on the car only turns move until its lidar driver lands, and a straight
verb that overshoots keeps its overshoot. **Before the first feedback
frame** (and whenever feedback is stale) the body refuses every non-zero
command and verb `no_feedback` (3.34), so the table's gaps there are moot
(`tests/test_startup_race.py` pins the first-frame case). The decision
behind the reverse row:
[architecture spec](../../safety/ARCHITECTURE.md), "A body that cannot see
astern does not reverse".

## Interfaces

**`SafetyController(robot, min_distance_cm=20.0, sensor_to_bumper_cm=0.0)`**

| Method | Returns / raises |
|---|---|
| `check_and_execute(action, speed=, duration=, angle=)` | The body's result dict. Raises `SafetyViolation` on a veto, or when a guarded verb achieved under `VERB_MIN_MOVE_M` / `VERB_MIN_TURN_DEG`; `ValueError` on an unknown action. A straight verb's `speed` (default 50) sets its bar (3.44) |
| `required_clearance_cm(v_m_s, creep=False)` | The straight-move bar in cm, as above |
| `scan_hint_m()` | The range the vet's scans ask for: half the chassis length plus 1.5 x the bar at `MAX_LINEAR_M_S` (about 0.85 m) |
| `vet_wheel_velocity(left_rad_s, right_rad_s)` | `(left, right, reason)`. `reason` is None when only slowed. Pass-through if the body's wheels are unusable (`robot/safety.py:665`). A forward or reverse component on a blind body with wheels is clamped to zero with `ahead_not_observed` / `astern_not_observed` in the reason; the loop writes the clamped command back, so a scan that returns does not restart it -- only a new command drives (3.144); rotation is untouched |
| `run_verb(plan)` | `carry_out_verb()`'s `{"done", "ended", "reason"}`, after the settle pass when the plan asks for one |
| `forward_clearance()`, `reverse_clearance()` (both aged), `path_clearance()`, `footprint_clearance(direction, scan)`, `rear_clearance(scan)`, `pivot_scale(omega, scan)`, `pivot_blocked(omega, scan, exact)` | See above |

**Driver ranks** (`DRIVER_PRIORITY`): `twin-dpad` 30, `teleop-operator` 30,
any unnamed or unknown name 30 (`DRIVER_UNKNOWN = "unknown"`), `brain` 20,
`teleop` 20, `ros` 20, `twin-local-brain` 10 (a dead rank: the in-browser
brain was deleted 2026-09-25).

**`arbitrate(driver, now)`**, in order:

1. If `driver` is autonomous and not `ros`, and the world reports a nav2 goal
   `pending` or `active`: refuse `preempted`. An unreadable bridge counts as
   no goal.
2. If nobody holds the robot, or the holder is `driver`: allow. The holder
   is `driver` while any `/action` verb is being carried out, or a goal
   naming a person is with the bridge (`state["verbs_running"]`, every drive
   path, 3.76 A1; the goal since 3.82), else `driver` until
   `watchdog_timeout_s` after `driver_at`. A verb's end re-stamps
   `driver_at` if its driver still holds the robot, before the count drops,
   so a verb longer than the timeout never lapses mid-run; a person's goal re-stamps it only when
   the bridge took the goal, and otherwise gives the slot back to whoever
   held it if nobody has driven since.
3. Lower rank than the holder: refuse `preempted`.
4. Both autonomous: refuse `preempted`.
5. Otherwise allow (equal manual rank, or a higher rank taking over).

Allowed `/action` and non-zero direct `/wheels` set `driver`, `driver_at`
and `last_command_at` as soon as `arbitrate()` passes, before the command
runs. The claim stands even if the command is then refused or fails:
`ros_unavailable`, `safety_distance`, `no_feedback`, a 400 for an unknown
action, or `unsupported` from `/wheels` on a body without motors. `/stop` sets only `last_command_at`. A zero direct
`/wheels` is never arbitrated and sets none of the three: from the holder
it zeroes the wheels, from anyone else it returns `ignored: true` (direct
drive only). Under ROS drive, `/wheels` is accepted only from `ros`, is not
arbitrated, and sets `last_command_at` and `last_ros_post_at`, never
authority. `POST /world/goal` arbitrates as `ros` and does not feed the
watchdog.

**A person's move ends a nav2 goal too** (3.97, the user 2026-10-24:
one rule under every drive). `do_action` calls `begin_goal_end()` for a
person's (`DRIVER_MANUAL`) `FORWARD`, `REVERSE`, `LEFT` or `RIGHT`
(`BODY_MOVES`; never `LOOK_*`) before it reads `goal_stop["stops"]`: on a
world with `cancel_goal` and only while `goal_in_progress()` (a bridge
read new on a person's move -- arbitration reads the goal only for the
brain -- skipped under `drive: ros` while the ROS stack is down), it counts a stop and starts the same loop as
`/stop` below (`_count_goal_end()`, `_start_goal_end()`), without the
latch or `robot.stop()` -- the move itself runs. A move the safety layer
then refuses still ends the goal (the press is the take-over), and an
unnamed caller ranks as a person, so a script's `/action` move ends one too. Counted before the verb's
own count is read, so its own loop never reads as a stop that overtook it.
Under `drive: ros` the bridge's cancel on a `twin-dpad` twist (3.15) is
kept; the two cancels are harmless together.

**Stop ends a nav2 goal** (handoff 2026-10-02 1b; completed after
`docs-review/SPEC-REVIEW-3.md` V1-V2). After `robot.stop()` returns,
`POST /stop` -- unless its `x-driver` is autonomous and not `ros` (the
brain), which spares the goal -- bumps `goal_stop["gen"]`, sets
`goal_stop["hold"]`, and starts a daemon thread `_end_goal_after_stop(gen)`
(`stop-end-goal`) when the world has `cancel_goal` (`RosWorld`). Every
`GOAL_STOP_POLL_S` the thread, under `goal_stop_lock` (held across the
bridge calls), reads
`world_model.get_goal()` and re-sends `cancel_goal()` (`DELETE /goal`)
while the goal is `pending` or `active`; once it is neither it waits
`GOAL_STOP_SETTLE_S` and clears the hold. An exception (bridge down,
timeout) is logged once and retried, still holding. While the hold is set,
a non-zero `/wheels` from `ros` is treated as zero: under ROS drive it is
applied as zero; under direct drive it takes the zero-command branch. A
newer stop or an accepted `POST /world/goal` bumps the generation, so one
loop runs at a time and never cancels a new goal (the goal route sets it
under the same lock). A stop that ends a goal (every stop but the
brain's) also counts itself in `goal_stop["stops"]`, which only those stops
move. The goal route reads that count
as the request arrives -- before arbitration and any wait on
`goal_stop_lock` -- and, only when `set_goal()`'s answer is `accepted: true`
and no stop has been pressed since, bumps the generation and clears the hold
(3.82). A goal a stop overtook while it waited for the lock is answered
`{"accepted": false, "reason": "stopped"}` without being sent; another goal
accepted meanwhile does not make it stale. A goal the bridge took while a
stop was pressed is answered `stopped` too (the stop's loop ends it, and the
person's claim is given back):
a refused goal (bridge unreachable, its 409, nav2 down, `accepted: false`)
never existed, so the stop's loop keeps ending the goal it was for and
keeps its hold; and a stop that landed while `set_goal()` was in flight owns the
newer generation, so its loop, waiting on `goal_stop_lock`, ends the goal
just taken. `goal_stop`'s two fields have their own lock,
`goal_gen_lock`, never held across a bridge call; `/stop` takes only that
one, so it answers at once while the bridge hangs (3.76 A8). The loop
clears the hold only if its generation is still current (and then sets
`goal_stop["ending"]`, the generation `/stop` gave it, back to None). A verb
allowed through on the ROS drive path lifts the hold, as it lifts
`RosDriveRobot`'s own stop hold (`lift_hold_for_verb()`, 3.84): under
`goal_gen_lock`, and only if `goal_stop["stops"]` is still the count `/action`
read as the request arrived -- a verb a stop overtook is not run and is
refused `stopped`, and the stop keeps its hold. `goal_stop["lifted"]`
counts the verbs running with the hold lifted, keyed by the stop count they
lifted it under; when the LAST one lifted under the newest stop ends
(`rehold_after_verb()`, in a `finally`; one lifted before a newer stop
neither counts nor re-holds), the hold is set again if a stop's
loop is still ending a goal and is the current one (`ending == gen`; an
accepted goal clears `ending`), so nav2's `ros` commands for the stopped goal
stay at zero after a person's move; the loop keeps cancelling and lifts it
when it settles. The `/action` STOP verb and the watchdog do none of this; they
call `robot.stop()`.

**A person's Stop is the last word over autonomy** (3.84, decided with the
user 2026-10-15). A `POST /stop` whose driver ranks as a person
(`driver_priority() == DRIVER_MANUAL`: `twin-dpad`, `teleop-operator`, an
unnamed caller, any unknown name) sets `goal_stop["autonomy_stopped"]`
(`{"by", "at"}`, under `goal_gen_lock`) BEFORE `robot.stop()`, whatever the
world. While it is set, `person_stop_refusal()` refuses every command from
an autonomous driver other than `ros` (`is_autonomy()`: `brain`, `teleop`)
with `reason: "person_stop"`, before arbitration: `/action`, a non-zero
direct `/wheels`, and a `POST /world/goal` naming `brain` (explore names
itself since 3.84; the latch is re-read under the goal lock, and a brain
goal under it answers `person_stop` whether or not it is stale, 3.76 K1).
A brain command refused on arrival lets go of the robot too
(`lapse_refused_claim()` with the claim's stamp read on arrival, 3.76 K1).
`/stop` counts its stop (`stops`, `gen`, the hold) in the same
`goal_gen_lock` step as the latch, before `robot.stop()`, and starts the
goal-ending loop even if `robot.stop()` raises (3.76 K1: counted after it,
a goal landing in between was accepted -- a brain goal under the latch, or
a person's tap the Stop then cancelled). Every `/action STOP`, from anyone,
is answered by `/stop` (never arbitrated). It is read again
at execution, because an autonomous verb's arbitration can wait on the
bridge: a direct verb under `motion_lock`, a ROS verb in
`lift_hold_for_verb()` under `goal_gen_lock`. Just before that read the
server takes the robot's `stop_count` and hands it to the verb as
`robot.interface.VERB_STOP_BASELINE` (a context variable): `carry_out_verb()`,
`SafetyController.run_verb()`'s settle and `RosDriveRobot._begin()` use it
as their stop baseline instead of reading the count later, so a Stop that
lands between the read and the verb's first motion (the Stop latches before
its `robot.stop()`) ends the verb unmoved -- `_begin()` then returns an
already-superseded generation and leaves the Stop's own hold in place
(`RosDriveRobot.stop_count` is its own stops plus the inner robot's, so a
stop made on the inner robot directly counts too). Under a person's
latch set by the time a brain verb ends, every answer but a fault (`FAULTS`:
`no_feedback`, `ros_unavailable`) becomes `person_stop` -- cut by the Stop, ended unmoved by the
baseline, or read as a veto because of the Stop's zeroing; a move that ran
is recorded as that refusal once, a refusal already recorded is not
recorded again -- and any answer that is not a move lets go of the robot. A brain command refused at execution
(`person_stop`, or `stopped`) had been given the robot by arbitration; its
claim lapses at once (`lapse_refused_claim()`), only if `driver_at` is still
that command's own stamp (`verb_running(stamped=...)`), never a newer
one's; `autonomous_at` keeps it (the house swap stays refused: the safe
side). A direct verb sets `verb_active` before taking
the baseline, so a watchdog stop cannot land in between. Nothing is logged
under `goal_gen_lock`. A direct `/wheels` takes
`stop_count` before its vet and, if any stop landed by the end of its write
(`/stop` never takes `motion_lock`), zeroes the wheels again and answers
`person_stop` (the brain, latched) or `stopped`. Never refused:
any `/stop`, `DELETE /world/goal`, a person's verbs and goals, unnamed
goals (ranked `ros`, today's behaviour), and `ros`'s `/wheels` (the hold
governs those). `brain`, `teleop` and `ros` stops set nothing. Only
`POST /autonomy/resume` clears it, which the brain sends when a mission
starts (`RemoteRobot.resume_autonomy()`, first in `start_mission`): a
person's next verb or goal does not, so a mission the person stopped can
never drive again. A start refused after the lift (400, 403, 409, any
error) began no mission, so the brain puts the latch back with
`POST /autonomy/relatch` (3.76 K2), which only latches and keeps a latch
a Stop during the build set since.

**The brain's goal lease** (3.145; 3.135.B's F6g/F7g). Under `drive: ros`
nav2's wheel posts feed the watchdog, so a goal the brain sent drove on to
its end after the brain died or froze. Now a goal lives only while its
brain does. `RemoteNavigator` names itself with `x-lease-id` (a uuid per
navigator, so a restarted brain beating as `brain` never keeps a dead one's
lease) and, from its first goal to its close, beats
`POST /autonomy/heartbeat` every `HEARTBEAT_S` (0.25 s) on its own thread
(a long tick never starves it). The route is `async` on the event loop and
only records the beat under `goal_gen_lock`. An accepted goal from an
autonomous driver whose `(driver, lease id)` beat within
`watchdog_timeout_s` arms the lease (`lease`: owner, id, the goal
generation `gen`); its reply carries `leased`, and the navigator warns on a
goal accepted unleased. A brain that never beats (older than 3.145) arms
nothing and keeps the old behaviour; a person's or an unnamed goal replaces
any lease. Every watchdog wake, `lease_lapsed()` drops a lease whose
generation moved on (a Stop, a person's move, another goal) and, when the
owner has been silent past `watchdog_timeout_s`, ends the goal the way a
Stop does -- the hold set at once (nav2's next post zeroed), a new
generation for `_end_goal_after_stop` on its own thread -- without counting
a stop (a person's command in flight is never read as overtaken), counts it
(`/health` `goal_lease.lapses`, `last_refusal` `autonomy_lapsed`) and sets
the autonomy latch with `lapsed: true` unless a person's Stop holds it. While
latched so, the brain's verbs and goals are refused `autonomy_lapsed`
(`RemoteRobot` and the runner's gated navigator raise `AutonomyLapsed`, and
the mission ends `failed` naming it), until `POST /autonomy/resume`; a
relatch for a start then refused puts it back as a lapse (`resumed_from`).
`DELETE /autonomy/heartbeat` (the navigator's close) RELEASES the lease:
the next wake ends the goal if still the brain's, with no latch and no
lapse, so a mission's end ends its goal whatever its cancel managed; a
`DELETE /world/goal` keeps the lease. A wake more than three periods late
(the event loop held, the beats waiting with it) skips the judgement once,
never twice running. Known limits: the lease proves the brain's process is
alive, not its loop (B3.3's tick guard ends a wedged mission, whose close
then releases the lease); after a lapse with no goal active the latch still
holds the brain out until a start.

**ROS liveness** (handoff 1d; refined by `docs-review/SPEC-REVIEW-3.md`
V7-V9). Under ROS drive, `ros_up(now)` is true when the actuator's last
`/wheels` post is younger than `ROS_SILENCE_S` AND `robot.bridge_up()`.
`RosDriveRobot._send()` marks the bridge down on a transport error
(connect, timeout) or a 5xx; a 4xx is the bridge answering, so it does not
mark it down and clears an earlier mark, as does any successful send.
`bridge_up()` never blocks: while down it answers false and starts at most
one background `GET /health` probe per `BRIDGE_PROBE_S` (the probe's
constants are in [body engineering](../body/ENGINEERING.md)); a 200 marks
the bridge up. So the first `bridge_up()` after a mark always answers false,
and the probe runs only when something asks. While the bridge is down, an
actuator `/wheels` post still stamps `last_ros_post_at` but is answered
`ros_unavailable` (unlogged, at 20 Hz) and moves nothing, so a person on
the fallback is the only writer. `/health` publishes `drive.bridge_up` and
`drive.ros_post_age_s` beside `drive.ros_up`.

**The watchdog** (`watchdog_loop()`): every `WATCHDOG_POLL_INTERVAL_S` it
skips while `verb_active`, else calls `robot.stop()` once
`now - last_command_at > watchdog_timeout_s`. Detection is therefore the
timeout plus up to one poll (1.1 s today). Under ROS drive the actuator's
20 Hz posts feed it, so it fires only when the actuator itself goes silent;
a silent ROS input is dropped first by `twist_mux` (0.25 s,
`service/slam/src/picar_bringup/config/twist_mux.yaml`) and then
`diff_drive_controller`'s `cmd_vel_timeout` (0.25 s,
`service/slam/src/picar_bringup/config/controllers.yaml`). The watchdog's
`robot.stop()` runs on the event loop and must not block; `RosDriveRobot`'s
does not wait on the bridge ([body engineering](../body/ENGINEERING.md)).

**Refusal reasons** (body `{"executed": false, "reason", "detail"}`,
counted in `/health` `refusal_counts`, last one in `last_refusal`):
`safety_distance`, `preempted`, `ros_unavailable`, `not_the_actuator`,
`unsupported`, `no_feedback` (3.34: the body's wheel feedback is stale, so
it refuses to move them; from driver `ros` on `/wheels` it is answered
unlogged, as `ros_unavailable` is), and `watchdog` (recorded in
`last_refusal` only, once per silence).

## Parameters and configuration

| Key / constant | Value | Unit | Read in | Why |
|---|---|---|---|---|
| `safety.min_distance_cm` | 20 | cm | `robot/server.py` | The stopping floor. 20 on both server and brain since S5: 3.3 sigma clear of one cell under sensor noise; 30 vetoed ~45% of legal moves |
| `brain.min_distance_cm` | see mission | cm | `control/brain_config.py` -> `brain/agent.py` | The brain's early out. Value and code default: [mission engineering](../mission/ENGINEERING.md), the canonical home |
| `safety.watchdog_timeout_s` | 1.0 | s | `robot/server.py` | Watchdog and authority lapse |
| `safety.sensor_to_bumper_cm` | 0.0 | cm | `robot/server.py` | Subtracted from cone ranges. To measure on the car |
| `WATCHDOG_POLL_INTERVAL_S` | 0.1 | s | `robot/server.py` | Watchdog wake rate; published in `/health` for `control/health.py` |
| `WHEEL_LOOP_INTERVAL_S` / `VERB_PERIOD_S` / `PIVOT_LOOKAHEAD_S` | 0.05 | s | `robot/server.py`, `robot/interface.py`, `robot/safety.py` | nav2's controller rate. A tick over 0.1 s counts as late |
| `ROS_SILENCE_S` | 0.5 | s | `robot/server.py` | Ten missed 20 Hz actuator posts = ROS down (3.24 G3) |
| `battery.low_v` / `reserve_v` / `poweroff_v` | 10.5 / 10.2 / 10.0 | V | `robot/server.py` | 3.137's ladder; placeholders until 3.137.E measures the Rover's discharge curve (the user, 2026-11-01). 10.0 is `HARDWARE-BOM.md` 6's clean-shutdown level, above the Jetson's 9 V input floor |
| `battery.reserve_max_s` | 120 | s | `robot/server.py` | Reserve held this long powers off, whatever the voltage |
| `battery.reserve_speed` | 30 | 0-100 | `robot/server.py` | A person's speed cap at Reserve (the user's call: drive slowly to the charger) |
| `battery.shutdown_steps` / `poweroff_cmd` | `[]` / null | | `robot/server.py` | The power-off's order; env `BATTERY_SHUTDOWN_CMDS` (JSON pairs) and `BATTERY_POWEROFF_CMD` ("" = stub) override. Set on the board by 3.137.C |
| `BATTERY_POLL_S` / `BATTERY_REST_S` | 0.5 / 0.5 | s | `robot/server.py` | Read rate; how long the wheels must be still for a read to count as at rest |
| `READINGS` / `SAMPLE_S` | 2 / 12 | count / s | `robot/battery_ladder.py` | M2.7's two distinct samples: a repeat counts again after the board's 10 s refresh plus slack |
| `GOAL_STOP_POLL_S`, `GOAL_STOP_SETTLE_S` | 0.25, 0.6 | s | `robot/server.py` | How often a stop re-reads and re-cancels a goal until nav2 reports it over, and how long `ros` wheel commands stay held after that. The settle is `STOP_HOLD_S`'s derivation: twist_mux's input timeout plus the controller's `cmd_vel_timeout` (they add) plus one plugin period |
| `FOOTPRINT_LENGTH_M`, `FOOTPRINT_WIDTH_M`, `LIDAR_X_M`, `LIDAR_TO_REAR_BUMPER_CM`, `LIDAR_TO_FRONT_BUMPER_CM` | see platform | m / cm | `robot/safety.py` (defined here; the footprint is imported by `sim/grid_world.py`, `LIDAR_X_M` is the sim's scan origin) | The chassis geometry every check places returns against. `LIDAR_TO_REAR_BUMPER_CM` is derived: half the length plus `LIDAR_X_M`; `LIDAR_TO_FRONT_BUMPER_CM` half the length minus it (3.42). Values and sources: [platform engineering](../platform/ENGINEERING.md), the canonical table. Held equal to the xacro and nav2 by `tests/test_wall_linters.py` |
| `FOOTPRINT_SIDE_MARGIN_CM` | 3.0 | cm | `robot/safety.py` | Starter-house doors are 30 cm against a 23.1 cm chassis. 3 cm less the march's 1.5 cm over-read; larger refuses every door (`tests/chassis_fit.py`) |
| `PIVOT_MARGIN_CM` | 1.3 | cm | `robot/safety.py` | 1.2 at 3.19. Raised after one guarded turn read 0.96 cm once the lidar moved 4 cm ahead (3.27). Moved after seeing data, as recorded |
| `PIVOT_MIN_LOOKAHEAD_DEG` | 1.0 | deg | `robot/safety.py` | Minimum look-ahead for a slow turn |
| `PIVOT_AGE_SAMPLES` | 4 | | `robot/safety.py` | Rotations checked across the turn a scan has missed (3.42) |
| `PIVOT_NEAR_CM` | 21.3 | cm | `robot/safety.py` | `PIVOT_MARGIN_CM` + 20: a return farther from the chassis now is never looked ahead for (as before 3.141) |
| `PIVOT_FILTER_SLACK_CM` | 1e-9 | cm | `robot/safety.py` | 3.141: the numpy pre-filter's slack; rounding is ~1e-15 cm, so it never hides a blocking return |
| `SAFETY_SCAN_RANGE_M` | 0.6 | m | `robot/safety.py` | Scan hint floor. Effective `max(0.6, scan_hint_m())`, about 0.85 m since 3.44: the sim reports beams past the hint as nothing there, so a hint under the top speed's bar made the SIMULATOR blind where the car sees. A real lidar ignores it. Without a hint the sim cast 360 rays to 12 m, about 13 ms a period |
| `CREEP_M_S` / `CREEP_MARGIN_CM` | 0.03 / 3.0 | m/s / cm | `robot/safety.py` | 3.44: a verb asked this slowly keeps only the corridor's side margin. 3.43's wedged robot stood 19.3 cm from what was behind it |
| `T_REACT_S` | 0.15 | s | `robot/safety.py` | One control period plus the R4 actuator chain (40-150 ms). Excludes the sensor's age, which `_aged()` already subtracts |
| `DECEL_M_S2` | 0.5 | m/s^2 | `robot/safety.py` | **[PLACEHOLDER]** braking, until 6.9(d) measures it on the car |
| `MAX_LINEAR_M_S` / `VERB_FULL_SPEED_M_S` | 0.6 / 0.6 | m/s | `robot/safety.py` | The chassis cap (controllers.yaml), for the scan hint; a verb's speed 100 (`moves_for()`'s two 0.30 m moves a second) |
| `CHASSIS_WIDTH_CM` / `PATH_REFERENCE_CM` | 16.5 / 30.0 | cm | `robot/safety.py` | Cone half-angle `PATH_HALF_ANGLE_DEG` = atan(8.25/30), about 15.4 deg. Errs narrow against the 23.1 cm Rover; the corridor covers the gap |
| `PATH_FRACTION` | 0.5 | | `robot/safety.py` | Only for a grid with no `fov_deg`. Warns above 8 columns |
| `VERB_MIN_MOVE_M` / `VERB_MIN_TURN_DEG` | 0.01 / 0.5 | m / deg | `robot/safety.py` | Under this, a verb is a refusal (3.22 decision 1) |
| `SETTLE_TOLERANCE_DEG` / `SETTLE_TOLERANCE_M` | 0.5 / 0.003 | deg / m | `robot/safety.py` | Same tolerances as `robot/ros_drive.py` |
| `SETTLE_TURN_RAD_S` / `SETTLE_LINEAR_M_S` | 0.1 / 0.02 | rad/s, m/s | `robot/safety.py` | 10 ms late = 0.06 deg / 0.2 mm |
| `SETTLE_WAIT_S` / `SETTLE_PASSES` | 0.15 / 3 | s / count | `robot/safety.py` | Wheels stopped plus three 50 ms feedback frames. Only for `wall_clock` plans with `settle` (fork firmware, 3.29) |

## Procedures

**Run the safety suites** (from `.venv`; the ground-truth ones take a
minute or two):

```bash
.venv/bin/pytest tests/test_depth_veto.py tests/test_footprint_safety.py \
  tests/test_pivot_safety.py tests/test_pan_safety.py tests/test_guarded_verbs.py \
  tests/test_mission_guarded_verbs.py tests/test_wheels_command.py \
  tests/test_ros_verb_safety.py tests/test_mover_safety.py -q
.venv/bin/pytest tests/test_authority.py tests/test_goal_arbitration.py \
  tests/test_ros_fallback.py tests/test_stop_cancels_goal.py tests/test_server.py \
  tests/test_watchdog_integration.py -q
```

A failure names a criterion (`test_criterion_N_...`) and prints the three
worst runs with their house, pose, `T` (travel-to-contact) and `G` (gap).

**Run the full ground-truth sweeps** after any change to `robot/safety.py`,
`sim/grid_world.py`'s collision, or a chassis constant:

```bash
python -m tests.demo_footprint_sweep              # 40 starts/house x 24 headings x 2 directions
python -m tests.demo_footprint_sweep --unclamped  # no clamp: judges the sim's own collision (no run > 0.5 cm deep)
python -m tests.demo_verb_sweep
python -m tests.demo_mover_sweep
```

Each prints `PASS`/`FAIL` per criterion and the closest travel-to-contact.
Judge only on these. Never judge on `/depth`'s `veto_cm` or the log line,
which are the readings the vet itself used.

**Diagnose "why is the robot not moving"** from `GET /health`:
`authority_holder` (who holds it, or null once lapsed), `last_refusal`
(`reason`, `detail`, `driver`, `seconds_ago`), `refusal_counts`,
`seconds_since_last_command`, `drive.ros_up`. Then `GET /depth`'s `path`
gives `blocked`, `veto_cm`, `veto_source` and `clearance_cm`.

- `preempted` with a holder named: wait for `watchdog_timeout_s` of silence,
  or have the holder stop driving.
- `person_stop`: a person pressed Stop; the brain stays out until a mission
  starts (3.84).
- `safety_distance` with `veto_source` `path_not_observed`: the camera is
  panned and there is no scan. Send `LOOK_CENTER`.
- `ros_unavailable`: the container is down. A person can still drive.
- `no_feedback`: the motor board has not reported for
  `FEEDBACK_STALE_S`. Read `/health` `motor_board`: `link_error` set means
  the serial line is gone (cable, USB, udev); `link_error` null with a
  growing `age_s` means the board is heard-but-silent (feedback switched
  off, firmware hung). Nothing moves until frames return.
- `safety_distance` with `ahead_not_observed`: a FORWARD (or a positive
  `/wheels`) on a body with wheels and no usable scan: the lidar has stopped
  or gone stale (`STALE_S`). Read `/health`'s lidar block; turn instead.
- `safety_distance` with `astern_not_observed`: a REVERSE (or a negative
  `/wheels`, or a settle pass astern) on a body with wheels and no usable
  scan. Expected on the car until its lidar driver lands; turn instead.
- `wheel_loop.late_ticks` above 0: periods the robot moved unvetted.
  Investigate CPU load before anything else.

**Raise a margin**: change the constant, run the full sweeps, and record the
before/after table in the plan entry. A margin moved after seeing data is
recorded as such (3.27 did this for `PIVOT_MARGIN_CM`).

## Verification

| Test (collected) | Pins |
|---|---|
| `tests/test_depth_veto.py` (28) | Tri-state handling; unusable zones never compared; fallback to the scalar; angle-based zone selection |
| `tests/test_footprint_safety.py` (12) | 3.18: seeded sample of the oblique-approach sweep over three houses. 3.36: the same bars hold on readings `SENSOR_STALE_S` old, and `_aged()` subtracts `v x age` closing and nothing moving away or with no age |
| `tests/test_pan_safety.py` (6) | 3.18 part 2: zones by body bearing; `path_not_observed` |
| `tests/test_stale_scan_forward.py` (8) | 3.144 (FMEA F9): `ahead_not_observed` with the cone seeing open floor; a verb under way stops within 2 cm of true path once the scan goes; through the wheel loop, a standing `/wheels` stops and stays stopped when the scan returns |
| `tests/test_pivot_safety.py` (4) | 3.19: no contact from turning; turns with room complete |
| `tests/test_pivot_3141.py` (9) | 3.141: the shared rows give the frozen reference's answers on random scans; five planted bugs caught; rows of another scan are never used |
| `tests/test_lidar_safety.py` (4) | 3.42 criterion 4: the 3.18/3.19 samples with the vet reading the D500's timed scan (no depth grid): straight runs pass; pivots 0/120 within 1 cm with the rotation aged, and fail without it |
| `tests/test_guarded_verbs.py` (8) | 3.22: verbs stop at the line; a stop ends a verb within 100 ms on the fake board. 4a judges "clear" against each speed's own bar (3.44 amendment) |
| `tests/test_speed_clearance.py` (15) | 3.44: today's speeds keep 20 cm; the creep margin only when asked; an unasked 0.02 m/s twist and a look-ahead-slowed verb stop at 20 cm; creep sweep samples (verb instant and lidar-timed, wheel loop) never within 2 cm, and the instrument fails with no margin; 0.4/0.5 m/s sweeps; the scan hint covers the top speed's bar |
| `tests/test_drop_guard_3159.py` (20) | 3.159: a stairwell ahead stops the robot >= 15 cm short at 0.1-0.6 m/s with the rule and makes it fall without; reverse toward floor never seen is refused, over driven floor allowed; an unusable block is never clear; a pivot carrying a corner onto an edge is refused and the other way is not; a grid with no `drop` block answers exactly as before; the sim's block is the floor seen and what ended it; a pinned sample of the sweeps; the review fixes (no pose refuses, an edge outlives a thing's end, a centre-line thing hides no edge, each controller's own margin, no extra grid astern without a drop, a fast turn looked at over 0.1 s, a first turn switches the guard on, an arc with a blind lidar folds its own frame, a missing first frame does not spend the probe). `tests/drop_sweep.py` is the sweeps' instrument (ground truth: a fall is the centre over a `v` cell) |
| `tests/test_mission_guarded_verbs.py` (3) | 3.32: in-process missions use guarded verbs through `_HaltGate` |
| `tests/test_wheels_command.py` (8) | R2b's six criteria plus the zero-heartbeat rules |
| `tests/test_ros_verb_safety.py` (7) | 3.24 G2: the same bars through the ROS path |
| `tests/test_mover_safety.py` (3) | 3.30: a crossing person |
| `tests/test_blind_reverse.py` (5) | 2026-10-02: `HardwareRobot` over the fake board with no sensors refuses a REVERSE verb without moving and clamps a standing reverse to zero; a standing turn (`vet_wheel_velocity()`) passes; a teleop body still reverses; a body with a scan is judged by it. Not pinned: a turn VERB, and a settle pass astern (Known gaps) |
| `tests/test_authority.py` (13) | M4: ranks, lapse, stop claims nothing, reasons on the wire, preemption ends a mission |
| `tests/test_review_376a.py` (5) | 3.76 batch A: a verb longer than the watchdog keeps its driver's authority; the wheel loop never writes over a stop; lost feedback under ROS drive is `no_feedback` and stops the robot; a second stop answers while the bridge hangs; a person taking over a ROS verb is not stopped by it, and the cut-off verb reads `preempted` |
| `tests/test_review_376k1.py` | 3.76 K1: a person's `/action STOP` latches and ends the goal, the brain's is never refused; a brain goal or a person's tap landing inside a Stop is not accepted; a Stop whose `robot.stop()` raises still ends the goal; a brain refused on arrival lets go; the floor band never raises (bad fields, non-dict, NaN, `cols` mismatch or Infinity are `floor_band_unusable`), a blind band says so, an open-floor band reads clear; battery outside 6-30 V is no reading and one glitch frame keeps the last pack reading |
| `tests/test_goal_lease.py` | 3.145: a beating brain's goal, once its beats stop, is held at zero within the lease plus a period and cancelled, the brain refused `autonomy_lapsed` (a verb, a goal, `RemoteRobot` raising `AutonomyLapsed`) until `/autonomy/resume`; the planted build (`evaluations/goal-lease-3145/robot_plant.py`) drives on; a beating brain keeps its goal 5 s, and its close releases it (ended, no lapse); a never-beating brain's, a person's and an unnamed goal are not leased; a `DELETE /world/goal` keeps the lease; another navigator beating as `brain` does not keep a dead one's lease; a person's latch survives a lapse; a relatch puts a lapse back as a lapse; a frozen explore mission ends `failed` naming `autonomy_lapsed` |
| `tests/test_person_stop.py` | 3.84: a person's Stop (`twin-dpad`, unnamed, `teleop-operator`) during explore: nav2 at 0.0 rad/s for 3 s, explore's next goal never reaches the bridge, the mission ends `cancelled`/`stop`; under `drive: ros` a verb arbitrated before a Stop (held between arbitration and the hold lift) leaves its hold set, and a person's verb during the Stop's loop turns > 30 deg, then nav2 is held at 0.0 until the loop ends the goal and lifts it; a brain verb or `brain`-named goal after it is refused `person_stop`; a mission over `RemoteRobot` ends `cancelled`/`stop`; a `brain` or `ros` stop latches nothing; a person's verb and goal, and an unnamed goal, still drive; `/autonomy/resume` and a new mission lift it |
| `tests/test_goal_arbitration.py` (9) | 3.23: a goal is an autonomous driver |
| `tests/test_battery_ladder.py` | 3.137: the ladder's rules on planted samples (one sample never fires; a repeat inside 12 s is one sample; a sag read while driving clears at rest or on recovery; a flat pack read at rest fires; a check that times out never fires on a recovered pack; Reserve's timer; latched levels); the sim pack's sag and refresh; in process, Reserve refuses the brain by name on `/action` and `/wheels`, caps a person's speed, lets go of the brain's claim and stops the wheels; a nav2 goal ends at Reserve and no goal is taken; Power-off refuses everyone, runs its steps in order and the power-off once; Reserve's timer powers off; the stub off the car; the brain ends `out_of_budget` and refuses a start at Low |
| `tests/test_stop_cancels_goal.py` (13) | Handoff 2026-10-02 1b: with an active goal and a fake nav2 that drives while it holds one, the wheels are still at zero `STOP_HOLD_S` + 0.1 s after `/stop`; a new goal can be set afterwards; a hung cancel costs the stop nothing. Spec review 3 V1: the brain's `/stop` spares the goal, and a mission preempted by a goal (`MissionRunner` over `RemoteRobot`) ends `preempted` with the goal still active; a person's stop still ends it. V2, nav2 ticking at 20 Hz from the moment of the stop: zero wheels while a 0.3 s cancel is in flight, through a failing cancel (retried once the bridge returns), and for a goal accepted after the stop; a new goal drives again |
| `tests/test_ros_fallback.py` (10) | 3.24 G3, all four criteria; handoff 1d: a dead bridge behind a live plugin reads `drive.ros_up` false, a person's verb runs direct, an autonomous one is refused `ros_unavailable`, and ROS is back once the bridge answers. Spec review 3 V8-V9: with the bridge down and the plugin alive, the plugin's non-zero `/wheels` post is answered `ros_unavailable` and moves nothing, and `/health` names which half is down |
| `tests/test_watchdog_integration.py` (6) | The watchdog against a live `uvicorn` subprocess with a 0.3 s timeout |
| `tests/test_server.py` (23) | `watchdog_should_stop()`, sensing does not feed the watchdog, safety over HTTP |
| `tests/test_wall_linters.py` | Footprint and `LIDAR_X_M` equal across the ROS wall; both collars still exist |
| `tests/test_floor_hazards.py` | 3.74: the band vetoes what the scan passes over; no block, no change; unusable zones ignored; a 30 cm corridor's walls never stop the robot; beside the body only inside its width; a null bearing falls back to the zone centre; band and scan points share one frame |
| `tests/test_floor_band_safety.py` | 3.74 B1 and B2a samples (slow tier): a sock ahead stops every run at >= 18 cm, and without the band runs touch it; with nothing low the band changes no run |

**Recorded numbers** (all on ground truth; plan sections in
`PLAN-ros-alignment.md`):

| Phase | Result |
|---|---|
| R2b (3.10) | Standing 0.1 m/s into a wall stops at 19.5 cm; 0.0 cm with the clamp removed |
| 3.18 | 5760 runs: 0 under 18 cm (closest 19.5), 0 contacts, progress 98.8%. Unfixed: 25 of 1440 under 18 cm, 9 contacts. Panned camera: 0 (unfixed 42 / 43) |
| 3.19 | 360 pivots: 0 within 1 cm (closest 1.02); 83 of 83 with room turned (worst 2.7 deg short). Unfixed: 120 of 120 |
| 3.24 G2 | 1440 ROS-path verbs: 0 under 18 cm (closest 19.73); 0 of 252 tiny verbs unrefused; clear turns 100% within 0.64 deg (worst 0.49) |
| 3.24 G3, live | Mission ended 2.04 s after a container kill; a D-pad verb ran 0.36 s after it |
| 3.27 | 2880 runs: 0 under 18 cm; 120 pivots: 0 touching (worst gap 1.03). With a centred-lidar assumption: 177 under 18 cm (worst 15.7, astern), 56 pivots touching |
| 3.29 | Fork-firmware turns: 120 of 120 within 1 deg (worst 0.84), +0.16 s a turn |
| 3.30 | 2021 crossing runs: 0 under 18 cm, 0 contacts. Clamp off: 1522 failures |
| 3.74 | A 5 cm object ahead, 600 runs at 0.1 and 0.2 m/s: 0 under 18 cm (closest 19.76), 0 contacts; band off 600 / 600 under, 437 contacts. Nothing low, 1920 runs: exact, no run changed; with S5 noise, 10.2% stopped > 3 cm earlier (bar 1%, failed; a property of per-zone noise, see the plan), progress -0.87 points |

Change checklist: confirm each new criterion RED against the old code
first; run the sweeps, not just the pinned samples; run the suite in module
order; record the table in the plan entry.

## Known gaps

- **The battery ladder (3.137).** With the robot server dead nothing reads
  the pack, and the pack's own cutoff is the backstop (3.135 F14's atomic
  saves). A repeated reading counted after 12 s may have been measured by the
  board before the wheels stopped (its refresh instant is not reported). The
  simulator's pack restarts its drain on a house swap (3.72). Levels are
  placeholders until 3.137.E. A check holds nav2 still (its `ros` wheel
  posts zeroed, the goal not ended) for up to 15 s; nav2's progress checker
  may abort the goal meanwhile, and explore then counts that place failed.

- **The floor band's blind range (3.74).** Inside the depth camera's
  minimum range (about 14 cm past the bumper) the band sees nothing, and
  nothing carries an earlier sighting across it. A FORWARD at the normal
  bar (20 cm) never gets there; a creep (3 cm bar, 3.44) after the band
  stopped the robot, or a pivot that swings a low object in, can. A hold
  carried by the wheels was built and removed in review: each of the
  robot server's two controllers and the brain's own kept a separate one,
  `/depth` raced it, and it failed open on a feedback blip. Options are in
  `PLAN-ros-alignment.md` 3.74.

- **Drops (3.159).** The mark is not built: a seen edge does not reach
  nav2's keepout until step B2 (after 3.145 frees `robot/server.py`), so
  nav2 keeps planning through a stairwell and the veto keeps refusing.
  The memory dead-reckons, so its floor drifts with distance (pruned at
  3 m of travel). The blind band's fill and the wall shadows are geometric
  inferences (a straight edge; a wall stops the body first). Seeding
  assumes the dock is at least 30 cm from any drop. Rugs and thresholds,
  and the real camera's reach on a grazing floor, are the Rover's (3.159.C).
  From B's reviews, left open on purpose: a cell's age is not checked when
  it is read (only pruned), so old floor can drift over a lip until the
  prune; a remembered thing that has left still excuses unseen floor
  behind it (never an edge); an arc is judged as the straight corridor
  plus a pivot in place, not its swept curve; a noisy range that overreads
  paints floor up to the noise past a lip (the margin covers it: S5's
  smallest stop 16.62 cm); the grid and the wheel reading are a period
  apart (~3 cm at 0.6 m/s, inside the margin). The turn check is
  quantised to 5 cm cells: within a cell of an edge it refuses some turns
  that open the gap (D4, failed) and let one step of a turn from under
  1 cm cross 0.17 cm (D2, failed). Round 2 adds: an edge is sticky
  against everything but its own kind, so a wrong edge (drift, one noisy
  frame) blocks that way until its cell is pruned; an edge remembered
  beyond a wall still shortens a move toward the wall; the turn looks
  0.1 s ahead, so a wheel-loop tick that is very late (3.153's 0.95 s)
  can turn past its look; the corridor search reads every line out to
  60 cm each tick (no early end), unmeasured on the board.

- **The car only turns until its lidar driver lands**: see "Driving
  without a lidar" above. By decision.
- **`tests/test_blind_reverse.py` pins less than the rule.** It covers the
  REVERSE verb, a standing reverse and a standing turn. Nothing pins that a
  turn VERB still runs blind on the car, or that a settle pass astern is
  refused `astern_not_observed` (`_settle()` -> `run_verb()` in
  `robot/safety.py`); both are read from the code, not tested.
- **`vet_wheel_velocity()` still passes a command through when the body's
  wheel state is unusable** (`robot/safety.py`, the early return). Harmless
  on every body today: `HardwareRobot` refuses motion without fresh feedback
  (3.34; handoff 2b closed), and the sim's bodies always have wheels. A new
  backend that reports unusable wheels and still moves would reopen it.
- **The verb that finds the bridge dead is refused, a person's too.** The
  first failed `RosDriveRobot._send()` raises inside the verb, which
  `/action` refuses `ros_unavailable` after stopping the robot directly
  (`WheelFeedbackLost`, also a `RuntimeError`, is refused `no_feedback`
  instead, after the same direct stop: 3.76 A5); it
  is not retried on the direct path, because some of it may already have
  gone through ROS. Only the next verb sees `ros_up()` false and falls back.
- **A dead `twist_mux` or `diff_drive_controller` behind a live bridge**
  still reads as alive (decided 2026-10-02 to leave it): verbs achieve
  nothing and `_refuse_if_nothing_achieved()` returns `safety_distance`,
  which fails toward stop.
- **Turns on a body with no `verb_plan()`** (the remote body, the ROS
  wrapper) get no pivot check inside `check_and_execute()`. The remote body
  is vetted again by the server, and the ROS path by the wheel vet, so the
  car is covered. An in-process caller holding such a body directly is not.
- **`CHASSIS_WIDTH_CM` is 16.5** (the PiCar-X), so the cone errs narrow
  against the Rover's 23.1 cm. The corridor check covers the difference.
  Re-measure on the car.
- **`sensor_to_bumper_cm` is 0.0** and applies to the cone only; the
  corridor and rear checks place returns with `LIDAR_X_M` instead.
- **Closed 2026-10-05 (handoff 3a):** unnamed, `teleop` and
  `teleop-operator` drivers under ROS drive used to be refused
  `ros_unavailable` (`docs-review/REPORT.md` V10); `robot/ros_drive.py`'s
  `ros_input_for()` now sends every person on the D-pad's `twist_mux`
  input and every other autonomous driver on the brain's. **And a person's
  goal is a person driving (handoff 3b):** `POST /world/goal` with a
  manual-rank `x-driver` arbitrates as that person -- it takes the robot
  from the brain, whose next command is refused `preempted` -- while a goal
  naming no one keeps 3.23's `ros` rank.
- **Until `ROS_SILENCE_S` (0.5 s) of silence after a container dies**,
  `ros_up()` is still true, so a person's verb goes to the dead bridge and
  is refused `ros_unavailable`. That window is inside G3's 2 s bar. The
  bridge-failure rule (built 2026-10-02) shortens it: the first failed send
  marks ROS down, so only that one verb is refused.
- **`DECEL_M_S2` is a placeholder** (3.44). Above about 0.39 m/s the bar
  is set by it, unmeasured; raising nav2's speed also waits on 3.42's
  relayed-scan time stamp (6.9).
- **A creep verb plans a whole 30 cm cell**, as every verb does
  (`moves_for()`), so it crawls to the 3 cm line: up to 10 s at 3 cm/s,
  the brain's HTTP timeout. The explore escape creeps only toward an end
  the 20 cm bar already refuses (at most 17 cm, ~6 s). Whether the Rover's
  board holds 0.75 rad/s wheels smoothly is a hardware-day check.
- **Late safety ticks on the Jetson** are measured only against the fake
  motor board: 0 in 14,289 under full load (3.33, with 3.37). With the real
  lidar driver and serial board in the process they are unmeasured.
