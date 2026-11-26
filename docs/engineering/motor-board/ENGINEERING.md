---
kind: engineering
domain: motor-board
status: current
verified: 2026-11-25
parent: docs/motor-board/ARCHITECTURE.md
---

# Motor board -- engineering

This is how the Rover's motor board, its firmware fork, the serial backend
and the fake board are built today. The what and the why are in the
[architecture spec](../../motor-board/ARCHITECTURE.md). The protocol below
was read from the firmware source: `waveshareteam/ugv_base_ros`, directory
ROS_Driver, at commit `2e7df97`. `sim/fake_esp32.py` is the executable
record of that reading, with each rule citing its firmware function.

`HARDWARE-BOM.md` section 4.2 describes a *different* board, the General
Driver (`ugv_base_general`). That board was open-loop in mainType 2 and had
no odometers. Its correction 5 applies to that board only. Do not use its
command table for the Rover.

## Implementation

| File | What it does |
|---|---|
| `robot/hardware_robot.py` | `HardwareRobot(port, sensors=None, baud=115200, track_scrub=1.0, lidar=None, camera=None, command_url=None, gimbal=False)`, a `RobotInterface` backend. With `gimbal` (3.220) the set-up also sends `T:4 cmd 2` (the gimbal module type, RAM only) and a centring `T:133`; each look sends one `T:133` (pan +/-`LOOK_PAN_DEG` or 0, clamped to `PAN_LIMIT_DEG`, tilt `LOOK_TILT_DEG` clamped to `TILT_LIMITS_DEG`, whole degrees) under `_write_lock`, whatever the feedback says; with the wheels at rest it then waits until the camera is settled or faulted (at most `PAN_SETTLE_TIMEOUT_S` + `FEEDBACK_STALE_S`), never while they turn (the server holds `motion_lock` over a verb). It answers `pan` (None if the line refused the command), `goal_pan_deg`, the measured `pan_deg`/`tilt_deg`, `settled`, `fault`, and a `gimbal_unmeasured` reason with no measured angle. The reader keeps `pan`/`tilt` from each frame (times `PAN_SIGN`), and re-sends `T:4` when fresh frames have carried no angle for `GIMBAL_MODULE_RESEND_S` (a reboot the odometers did not show). `gimbal_status()` (and `feedback_status()['gimbal']`) give the measured angle only from a frame within `FEEDBACK_STALE_S` and with no fault, else `None`; a fresh measured pan not within `PAN_SETTLED_DEG` of the goal after `PAN_SETTLE_TIMEOUT_S` is a fault until a goal is reached or a new one sent (the tilt is reported, not judged). `get_camera_frame()` overwrites the frame's `pan_deg`/`tilt_deg` with the measured ones, and raises when the pan is unknown (a 503 at `/frame`): every consumer reads a missing pan as dead ahead. Off, the looks go to `sensors` as before. With `command_url` (3.217) `_send()` hands every command to `robot/board_http.py` instead of writing the port, and `_zero_if_stale()` also zeroes a standing command when that route is stale; everything else in this row is unchanged. It opens the tty raw at 115200 and starts a `select()`-polling reader thread, which parses only `T:1001` frames and drops (counts, with the last reason for `/health`) bad lines: not JSON, or a `T:1001` frame with a missing or non-finite speed (3.76 B2); a line that is JSON but not an object, or not `T:1001`, is ignored and not counted. It ends, zeroing any standing command, on a read error or an empty read from a hung-up line (3.76 B3). It sends its set-up (`T:136`, `T:131`) at start and again after a detected reboot. It turns wheel commands into `T:1`; deciding a command and writing it are one step under `_write_lock` (with `advance()`'s re-send, `stop()` and `_run()`'s stop check), so the board's last command is always `_cmd` (3.76 B1). A zero-degree pivot sends nothing (B6). It estimates each wheel's travel: a trapezoidal speed integral, clamped by `_anchor()`, with `_absorb_reboot()` and `_track_board_clock()` for the fork; the host-minus-board offset keeps its minimum but may rise by `CLOCK_DRIFT_MAX` (200 ppm) of board time, so a slow crystal does not age every frame with uptime (3.76 B7). The cost: the least-delayed frame is forgotten within tens of seconds (about 25 s at 5 ms of jitter), so a sustained rise in host latency reads as a younger frame by that latency, milliseconds against `EXTRAPOLATE_MAX_S`. `verb_plan()` and `verb_done()` let `SafetyController.run_verb()` carry verbs out on the wall clock. The scan is `lidar`'s when one is given (3.42) and the frame `camera`'s (3.90 A); otherwise camera, lidar, depth and (with `gimbal` off) pan are delegated to `sensors`; with none, they answer the interface's unusable defaults (`get_camera_frame()` raises). |
| `robot/board_http.py` | 3.217, a stopgap for a board whose serial receive is dead: `HttpCommandSender(base_url, timeout_s=0.10)`. `submit()` never waits on the network. One thread sends `GET /js?json=<command>`, one at a time, on a kept-alive connection. Wheel commands (`T:1`) are one slot, newest wins. A waiting stop is sent first, then set-up commands in their order, then a speed; a speed that waited longer than `COMMAND_STALE_S` is sent as a zero (`expired`). A request that fails or times out is abandoned, the connection dropped, and the newest wheel command sent again (a set-up command is retried, unless the board answered with a non-200 status: then it is dropped and counted). A connection the board closed while idle is retried once on a new one and is not a failure. After any failure the newest wheel command is sent `REASSERT_SENDS` more times, `REASSERT_PERIOD_S` apart. With a command undelivered for `COMMAND_STALE_S`, `submit()` refuses a non-zero wheel command (returns False) and queues a zero in its place. `status()` is what `/health` shows; `flush(settled=True)` lets `close()` wait for the last stop and its repeats, 0.3 s at most. Any exception in the sender is a failure with its reason kept, never the end of the thread. |
| `sim/fake_esp32.py` | `FakeEsp32Http(board, delay_s=0.0)` (3.217) is the board's web route on a local port: `GET /js?json=` into the same `_handle()` the pty feeds, with a reply delay, `lose_next(n, hang_s)` (requests the board never sees), `late_next(n, late_s)` (requests the board acts on late, after the host gave up) and `refuse()`. `FakeEsp32(body, main_type=2, drop_rate=0.0, seed=0, firmware=None, millis_at_boot=0, battery_v=None, drain_v_per_s=None, module_type=None)`. Since 3.220 it models the gimbal (`gimbal_module.h`): `T:133`/`T:134` clamp X to -180..180 and Y to -30..90 and map to servo ticks with Arduino's long `map()` (whole degrees), in any module type; `T:135` holds where the servos are; `T:137` steady mode; `T:4` sets the module type in RAM, `T:900` also saves it, and a reboot restores the saved one (`module_type`, else `SIM_BOARD_MODULE`, default 0 as this Rover's board). The servos travel at `GIMBAL_DEG_PER_S` (270, the ST3215's no-load speed) and survive a board reboot; the `1001` frame carries `pan`/`tilt` from their positions only in module type 2. Once a gimbal command has arrived it sets the body's camera pan (`world.pan`, in 90-degree units) from the pan servo every loop; before, the body server's `/look` still sets it. It opens a pty (`.path`) and runs a firmware loop thread that turns `body`'s wheels through `set_wheel_velocity()` and `advance()`. It keeps integer encoder counts and measures speeds from count deltas. It sends rate-limited `1001` frames, runs the heartbeat, and has `reboot()`. Its wire is lossy (`drop_rate`) and loses only whole lines. `sent_truth` records the body's wheel travel at each frame actually sent, which is what tests judge against. |
| `robot/factory.py` | `mode: hardware`. With `SIM_MOTOR_BOARD=fake` (`PLAN-ros-alignment.md` 3.36), it builds `SimBodyClient(SIM_BODY_URL, SIM_SENSORS_URL)`, then `HardwareRobot(client.board_path, sensors=client, track_scrub=1.0, gimbal=...)` (the gimbal on unless `ROBOT_GIMBAL` says off, 3.220): the board and its body are in `sim/body_server.py`'s process, and the robot server opens the board's pty as a serial device, as it opens the real one on the car. Without both URLs it raises. `build_fake_body(config)` (`MockRobot` in the `SIM_MAP` house, then `FakeEsp32(body)`) is what the body program calls. Otherwise it builds `HardwareRobot(ROBOT_SERIAL or hardware.serial_port, track_scrub=...)`, and raises if neither is set. Both branches add the lidar (`ROBOT_LIDAR`) and camera (`ROBOT_CAMERA`) drivers when configured. The real-board branch passes `ROBOT_COMMAND_URL` (or `hardware.command_url`) as `command_url` when set (3.217), and `gimbal` only when `ROBOT_GIMBAL` or `hardware.gimbal` turns it on (3.220). |
| `firmware/ugv_base_ros/0001-feedback-fine-odometers-and-board-time.patch` | Our GPL-3.0 fork. It adds 13 lines inside `baseInfoFeedback()` (`ROS_Driver/ugv_advance.h`) and removes none. |
| `firmware/ugv_base_ros/build.sh` | Compiles a checkout, stock or patched, with `arduino-cli`. Compile only. |
| `firmware/ugv_base_ros/README.md` | How to apply, build, dump and restore the firmware. |
| `robot/safety.py` | `SafetyController.run_verb()` and `_settle()` carry out and settle wall-clock verbs. Owned by the safety domain; listed here because `HardwareRobot.verb_plan()` turns the settle pass on. A correction runs through `run_verb()` and is vetted like a move: since 2026-10-02 `reverse_clearance()` answers `(0.0, "astern_not_observed")` on a body whose wheels report but whose scan is unusable, so a correction that would back up ends `clamped` ("settle refused") on `HardwareRobot` with no `sensors`, and so does one that would go forward, because every FORWARD is refused there too (`get_distance()` is 0.0). The rule for a body with no sensors is the safety domain's, specified once in the [safety engineering spec](../safety/ENGINEERING.md) (Known gaps). Over the fake board the sim body supplies the scan (`sensors=body`), so the settle behaves as recorded below. |

## Interfaces

### Serial link

The link is newline-delimited JSON at 115200 baud, 8N1, raw. The board
writes compact JSON (ArduinoJson). The host writes `json.dumps()` output,
which has spaces; the firmware accepts either.

**Commands the host sends:**

| Command | When | Meaning |
|---|---|---|
| `{"T":136,"cmd":200}` | At start, and after a reboot | Heartbeat: zero the wheel speeds after 200 ms without a `T:1` or `T:11` |
| `{"T":131,"cmd":1}` | At start, and after a reboot | Continuous feedback on (already the stock boot default) |
| `{"T":1,"L":<m/s>,"R":<m/s>}` | Every wheel command; re-sent by `advance()` while non-zero | Closed-loop wheel surface speeds, `rad/s x WHEEL_RADIUS_M`, rounded to 5 places. The host scales both speeds by one factor into the board's +/-`BOARD_MAX_WHEEL_M_S` (2.0) first, keeping their ratio, because the firmware drops an out-of-range `T:1` and keeps the OLD setpoint running (handoff 4j). A non-finite command is sent as a stop (3.76 M1: `NaN` compared false against the window and reached the board, which dropped it the same way). |

**The web route (3.217, a stopgap).** With `ROBOT_COMMAND_URL` set the
same three commands go out as `GET http://<board>/js?json=<compact JSON,
URL-quoted>` and nothing is written to the serial line; the feedback frame
is still read from it. The board's reply body is read and ignored. Measured
on the Rover's board from the Jetson over the house Wi-Fi, 2026-11-25
(`T:130` at 20 Hz, 2,600 requests): p50 24 ms, p90 36-39 ms, p99 73-116 ms,
6 unanswered in 1 s. **`T:405` on that route returns the Wi-Fi password in
clear: never log its reply.**

**Other commands the fake models** (the host never sends these):

| Command | Behaviour in the fake |
|---|---|
| `T:0` | `setGoalSpeed(0, 0)`, as the firmware does (`uart_ctrl.h`). Bus-servo torque is not modelled. |
| `T:11` | Raw PWM +/-255: turns the PID off and feeds the heartbeat. |
| `T:13` | `{X, Z}` twist. Does NOT feed the heartbeat. |
| `T:130` | One frame now, subject to the rate limit. |
| `T:142` | Feedback interval in ms. |
| `T:900` | `{main}` selects a mainType. |

### Feedback frame `T:1001` (board to host)

| Key | Type | Meaning |
|---|---|---|
| `L`, `R` | float m/s | MEASURED wheel speeds: whole encoder edges per board loop, times `pi x WHEEL_D / pulses`, divided by dt |
| `ax ay az gx gy gz mx my mz` | int | IMU. Zeros in the fake. |
| `odl`, `odr` | int, cm | Each wheel's travel since boot, `(long)(en_odom * 100)`, truncated toward zero. So 0 means (-1, 1) cm. |
| `v` | int, 1/100 V | Pack voltage. Read by `HardwareRobot.get_battery()` (3.70 E4); outside 6-30 V (`BATTERY_PLAUSIBLE_V`: a board on USB, a failed INA219 read) it is no reading, never a battery below the floor (3.76 K1). The fake sends 1200 unless given a drain: `FakeEsp32(battery_v=, drain_v_per_s=)` or `SIM_BATTERY_V` / `SIM_BATTERY_DRAIN_V_PER_S`, a straight line in wall time from power-up (a reboot does not recharge it). Since 3.137 (`sim/battery.py`) also `SIM_BATTERY_REFRESH_S` (the real board's 10 s refresh: the value repeats until then; 0, the default, measures every frame) and `SIM_BATTERY_SAG_V` / `SIM_BATTERY_SAG_S` (a sag for that long after a non-zero `T:1` from rest, while the setpoint is non-zero). |
| `odlt`, `odrt` | int, 0.1 mm | **Fork only.** `(long)(en_odom * 10000)`. One encoder edge is 0.38 mm. |
| `ms` | unsigned long, ms | **Fork only.** `last_feedback_time`, the board's `millis()` when the frame was built. Zero at boot; wraps at 2^32. |
| `pan`, `tilt` | float, deg | **Module type 2 only** (3.220): `panAngleCompute(pos)` = (pos - 2047) x 360 / 4095 and `tiltAngleCompute()` = (2047 - pos) x 360 / 4095, from the positions `getGimbalFeedback()` reads off the servo bus every loop. A servo that stops answering keeps its last position here. |

Frames are sent at most once every 50 ms (`feedbackFlowExtraDelay`, set by
`T:142`), whether streamed or requested with `T:130`.

**Frame size** (measured on the fake): stock 125 bytes and the fork 160
bytes. At 20 Hz, the fork uses about 28% of the link.

### mainType constants (`movtion_module.h` `mm_settings()`)

| mainType | Robot | `WHEEL_D` (m) | pulses/rev | `TRACK_WIDTH` (m) |
|---|---|---|---|---|
| 1 | RaspRover | 0.0800 | 2100 | 0.125 |
| 2 | **UGV Rover** (ours) | 0.0800 | 660 | 0.172 |
| 3 | UGV Beast | 0.0523 | 1092 | 0.141 |

The host never sends `T:900`. It assumes mainType 2, the Rover's stock
setting. The 660 is 11 lines x 2 (half quadrature) x 30:1.

### What the backend reports

The shapes are owned by the body contract.

- **`get_wheel_state()`** gives `position_rad` from the estimated travel,
  carried forward up to `EXTRAPOLATE_MAX_S`. Its `velocity_rad_s` is the
  last *commanded* speed, not a measured one. It also gives `counts` (from
  the estimate, at 660/rev), `wheel_radius_m`, `track_width_m` and
  `counts_per_rev`. `usable` turns true at the first frame and goes false again, with `awaiting_feedback: true`, when no frame has arrived for `FEEDBACK_STALE_S` (3.34).
- **`get_odometry()`** gives `distance_m`, the body path: the change in the
  wheels' average, so a pivot adds nothing. Its `heading_deg` comes from
  the wheel difference, clockwise, measured from where the process started.
- **`verb_plan()`** returns `{kind, left_rad_s, right_rad_s, target,
  wall_clock: True, settle, [moves]}`. `settle` is true only while fork
  frames are arriving.

## Parameters and configuration

| Key or constant | Default | Unit | Where read | Why this value |
|---|---|---|---|---|
| `mode` / `ROBOT_MODE` | `sim` | -- | `robot/factory.py` | `hardware` selects this backend. |
| `ROBOT_SERIAL` or `hardware.serial_port` | none (required) | path | `robot/factory.py` | Not in the shipped yaml. Use a udev symlink on the car, never `/dev/ttyUSB0` by enumeration order. |
| `SIM_MOTOR_BOARD` | unset | -- | `robot/factory.py`, `service/tunnel/run.sh` | `fake` runs the backend over `sim/fake_esp32.py` on a pty, in the body program (3.36). |
| `SIM_BODY_URL`, `SIM_SENSORS_URL` | none (required with `fake`); `run.sh` defaults `:8002` and `:8003,:8004` | URL | `robot/factory.py`, `sim/body_client.py` | The body program (board, physics, truth) and the sensor programs (first: the safety bundle; last: full scans and frames). Simulator detail: [simulator engineering](../simulator/ENGINEERING.md). |
| `SIM_BOARD_FIRMWARE` | `stock` | -- | `sim/fake_esp32.py`, in the body program | `fork` adds `odlt`, `odrt` and `ms`. It re-runs any suite over the fork. |
| `SIM_BOARD_SILENT_S` | 0 | s | `sim/fake_esp32.py` (`silent_until`), in the body program | The fake board sends no feedback frame for this long after power-up, to reproduce the start-up race a ROS container started first must ride out (handoff 2a; `tests/test_startup_race.py`). |
| `WHEEL_RADIUS_M`, `TRACK_WIDTH_M`, `COUNTS_PER_REV` | 0.040, 0.172, 660 | m, m, edges/rev | `robot/hardware_robot.py` | What the backend converts with: mainType 2's `WHEEL_D` / 2, `TRACK_WIDTH` and pulses (table above). The physical facts, their sources and every other copy are in the [platform engineering spec](../platform/ENGINEERING.md); `tests/test_wall_linters.py` and `tests/test_ros_driver_board.py` pin the copies equal. |
| `FEEDBACK_STALE_S` | 0.25 | s | `robot/hardware_robot.py` | 3.34: no frame for this long and the wheels and odometry answer "unusable", a standing command is zeroed by this process, and a non-zero command or a verb raises `WheelFeedbackLost`. Five feedback intervals, below the server's 1.0 s watchdog, so it acts first. It watches board-to-host; the heartbeat watches host-to-board, so the two are not ordered. |
| `TRACK_SCRUB` env, else `hardware.track_scrub` | 1.0 | ratio | `robot/factory.py` | Skid steer's effective track over the geometric one. `[PLACEHOLDER]` until measured on the Rover. Pinned to 1.0 for the fake board (its sim body does not scrub). The ROS container reads the same `TRACK_SCRUB`. |
| `ROBOT_COMMAND_URL` or `hardware.command_url` | unset | URL | `robot/factory.py` | 3.217: `http://<IP address>[:<port>]` and nothing else (no path, query or credentials, and no host name: resolving one is not bounded by the timeout; anything else raises). Set, commands go over the board's web route; unset, the serial line. Not in the shipped yaml. Not read under `SIM_MOTOR_BOARD=fake`. |
| `COMMAND_TIMEOUT_S` | 0.10 | s | `robot/board_http.py` | One request's patience (per socket operation, not a total). Half the 200 ms heartbeat, so one lost request and its retry fit inside it; above the measured p99 of 73-80 ms on 60 s runs (116 ms on one 10 s run). |
| `COMMAND_STALE_S` | 0.25 | s | `robot/board_http.py` | A command undelivered this long and non-zero wheel commands are refused: the same five periods as `FEEDBACK_STALE_S`, for the other direction. |
| `REASSERT_SENDS`, `REASSERT_PERIOD_S` | 3, 0.05 | count, s | `robot/board_http.py` | After a failure the newest wheel command is repeated, because the board may act late on a request the host gave up on. |
| `RETRY_PAUSE_S`, `SETUP_QUEUE_MAX` | 0.02, 32 | s, count | `robot/board_http.py` | A refused connection fails at once: the pause keeps the retry loop off a core. The set-up is two commands; the cap only bounds a route that never comes up. |
| `HEARTBEAT_MS` | 200 | ms | `robot/hardware_robot.py` | 3.216: four 50 ms wheel periods. The longest gap between sends while moving was 52 ms in every kill-sweep run. A dead robot server's last setpoint runs at most this long, and `robot/safety.py`'s `BOARD_HEARTBEAT_S` copies it into the stop distance. Not ordered against the 1.0 s watchdog, which watches the brain (1500, until 3.216). The firmware default is 3000. |
| `ROBOT_GIMBAL` or `hardware.gimbal` | off on a real board, on under `SIM_MOTOR_BOARD=fake` | on/off | `robot/factory.py` | 3.220: the robot server drives the pan-tilt. Off on the real board until 3.220's board step (criterion 6) has been run with the user. Anything but on/off raises. |
| `LOOK_PAN_DEG`, `PAN_LIMIT_DEG` | 90, 90 | deg | `robot/hardware_robot.py` | A look's pan, as the simulator's `PAN_ANGLE_RAD`, and the most any command asks: the URDF's `pan_limit`, set by the camera cable. `[PLACEHOLDER]` until the board step. |
| `LOOK_TILT_DEG`, `TILT_LIMITS_DEG` | 0, (-30, +30) | deg | `robot/hardware_robot.py` | A look holds the camera level; -30 is the firmware's own floor, +30 `[PLACEHOLDER]`. |
| `PAN_SIGN` | +1 | -- | `robot/hardware_robot.py` | The firmware's +X taken as clockwise (right). `[PLACEHOLDER]` until a look is watched on the board. |
| `PAN_SETTLED_DEG`, `PAN_SETTLE_TIMEOUT_S` | 3.0, 1.5 | deg, s | `robot/hardware_robot.py` | Measured within 3 deg of the goal is reached (the command is whole degrees). Not reached in 1.5 s is a fault: a 90-degree swing takes about 0.35 s at the servo's top speed; the rest is the web route's retries. |
| `REBOOT_JUMP_M` | 0.03 | m | `robot/hardware_robot.py` | On stock firmware, a reboot needs both odometers near zero AND the estimate this far outside their bucket. A rule on the distance alone fired falsely in 1 of 6 runs (3.25). |
| `EXTRAPOLATE_MAX_S` | 0.10 | s | `robot/hardware_robot.py` | Two feedback intervals. Without carry-forward, a FORWARD overshot to 30.6-32.8 cm (3.25). |
| `TENTH_MM`, `CM`, `MILLIS_WRAP` | 1e-4, 0.01, 2^32 | m, m, ms | `robot/hardware_robot.py` | Units. Millimetres left the at-rest estimate up to 1.26 degrees out (3.29). |
| `MOVE_M`, `MOVES_PER_SECOND_AT_FULL_SPEED` | 0.30, 2.0 | m, 1/s | `robot/hardware_robot.py` | A verb means what it means on `MockRobot`. |
| Turn rate in `verb_plan()` | 1.2 | rad/s (body) | `robot/hardware_robot.py` | The same as `robot/ros_drive.py`'s `TURN_RATE_RAD_S`. |
| `SETTLE_TOLERANCE_DEG`, `SETTLE_TOLERANCE_M` | 0.5, 0.003 | deg, m | `robot/safety.py` | The ROS path's tolerances (3.29). |
| `SETTLE_TURN_RAD_S`, `SETTLE_LINEAR_M_S` | 0.1, 0.02 | rad/s, m/s | `robot/safety.py` | A stop arriving 10 ms late then costs 0.06 deg or 0.2 mm. |
| `SETTLE_WAIT_S`, `SETTLE_PASSES` | 0.15, 3 | s, count | `robot/safety.py` | Lets the wheels stop and three 50 ms frames arrive. |
| `BOARD_LOOP_S` | 0.01 | s | `sim/fake_esp32.py` | The firmware's `loop()` runs at about 100 Hz. Real timing is unmeasured. |
| `DEFAULT_FEEDBACK_INTERVAL_MS`, `DEFAULT_HEARTBEAT_MS` | 50, 3000 | ms | `sim/fake_esp32.py` | `ugv_config.h`. |
| `NO_LOAD_WHEEL_M_S` | 1.3 | m/s | `sim/fake_esp32.py` | Full PWM for `T:11` only: the Rover's rated top speed, assumed. |
| `OUT_BUFFER_BYTES` | 4096 | bytes | `sim/fake_esp32.py` | A USB bridge's buffer. Unread lines are lost whole, and the firmware loop never blocks. |
| `build.sh` pins | esp32 core 3.2.1, INA219_WE 1.3.8, plus nine other libraries | -- | `firmware/ugv_base_ros/build.sh` | Stock `2e7df97` does not compile against INA219_WE 1.4 (renamed constants) or esp32 3.3 (the ESP-NOW callback type changed). |

## Procedures

**Run the whole stack over the fake board** (3.36: the board, its body and
the body's sensors are separate programs; `service/tunnel/run.sh` starts
them before the robot server, on ports 8002-8004, and `restart.sh` stops
them too and checks the body program's git revision):

```bash
SIM_MOTOR_BOARD=fake ROBOT_MODE=hardware WORLD_MODE=ros bash service/tunnel/run.sh
SIM_MOTOR_BOARD=fake SIM_BOARD_FIRMWARE=fork ROBOT_MODE=hardware WORLD_MODE=ros bash service/tunnel/run.sh
```

By hand, the order is the body (`uvicorn sim.body_server:app --port 8002`),
the sensors (`uvicorn sim.sensor_server:app --port 8003`, again on 8004),
all with the same `SIM_BODY_SHM`, then the robot server with
`SIM_BODY_URL=http://127.0.0.1:8002` and
`SIM_SENSORS_URL=http://127.0.0.1:8003,http://127.0.0.1:8004`. The robot
server waits up to 30 s for both to answer `/health`. `WORLD_MODE=sim` is
refused with a remote body; use `ros` or `none`.

`/health` answers as usual, and its `sim_map` names the house the sim body
stands in (since 2026-10-02; it was `null` under the fake board, which made
the live ROS suites skip). `GET /wheels` turns `usable: true` within one
feedback interval. With `drive: ros` added (`ROBOT_DRIVE=ros`), R4's live
chain suite runs over the serial line (3.16 criterion 5); start the ROS
container only after `/wheels` reports `usable: true` (the start-up order in
the [ros engineering spec](../ros/ENGINEERING.md)).

**Build the fork** (compile only; about 1 GB of toolchain on first run):

```bash
git clone https://github.com/waveshareteam/ugv_base_ros && cd ugv_base_ros && git checkout 2e7df97
git apply /path/to/vision-picar/firmware/ugv_base_ros/0001-feedback-fine-odometers-and-board-time.patch
bash /path/to/vision-picar/firmware/ugv_base_ros/build.sh .
```

- **Success:** `build/*.bin`. The recorded size is 1,223,030 bytes, 93% of
  the app partition and 236 bytes over stock.
- **Missing toolchain:** `arduino-cli` is found on `PATH`, in `ARDUINO_CLI`,
  or at `~/.local/bin/arduino-cli`.
- **Unpinned libraries:** errors naming `PG_320`, `BRNG_16` or
  `BIT_MODE_9`, or an ESP-NOW callback type, mean the build ran without the
  pins.

**Flash.** This happens on hardware day, after the arrival checks in
`JETSON-BOM.md` 9.5. **UNCONFIRMED: not run.** The commands are
`arduino-cli`'s and esptool's documented ones, with the core `build.sh` pins
(esp32 3.2.1, which carries its own esptool). Stop the robot server and
Waveshare's `ugv_jetson` app first, so nothing else holds the port.

1. Dump the stock image: `esptool.py --port <port> read_flash 0 ALL
   stock-ugv_base_ros.bin`. Keep it off the car.
2. Upload the build from the checkout `build.sh` compiled:

   ```bash
   CLI="${ARDUINO_CLI:-$HOME/.local/bin/arduino-cli}"
   "$CLI" upload --fqbn esp32:esp32:esp32 --port <port> --input-dir <checkout>/build
   ```

   Board: "ESP32 Dev Module" (`esp32:esp32:esp32`). Expected: esptool's
   `Connecting...`, `Chip is ESP32-D0WD...`, `Writing at 0x...` lines,
   `Hash of data verified.` for each region, then `Hard resetting via RTS
   pin...`.
3. Confirm the fork's keys with the heartbeat check below: frames now carry
   `odlt`, `odrt` and `ms`.
4. To restore: `esptool.py --port <port> write_flash 0 stock-ugv_base_ros.bin`,
   or Waveshare's ESP32 Download Tool with their published image.

| Symptom | Meaning |
|---|---|
| `A fatal error occurred: Failed to connect to ESP32: No serial data received` | The port is held (the robot server, `ugv_jetson`), the wrong port, or the board did not enter its bootloader: hold BOOT while the upload starts. |
| `Permission denied` / `could not open port` | Not in `dialout`, or the udev rule is missing (below). |
| `Hash of data verified` missing, or a `MD5 of file does not match` error | A bad write. Upload again before power-cycling; restore the dump if it repeats. |
| `Error during Upload: ... no such file` | `--input-dir` does not point at `build.sh`'s `<checkout>/build`. |

**A stable device name (udev).** Skip this if the kit wires the Jetson's
header UART (`/dev/ttyTHS1` is already stable). For the board's USB bridge,
fill the values from the car, never from a guess: the board and the lidar
can both be CP210x, so `idVendor` and `idProduct` alone may match both, and
the `serial` attribute is what tells them apart.

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial' | head
```

`/etc/udev/rules.d/99-picar-motor-board.rules` (the values are to be filled
from that output on the car):

```text
SUBSYSTEM=="tty", ATTRS{idVendor}=="<vid>", ATTRS{idProduct}=="<pid>", ATTRS{serial}=="<serial>", SYMLINK+="picar-motor-board", GROUP="dialout", MODE="0660"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/picar-motor-board          # -> ttyUSB<n>
```

`ROBOT_SERIAL=/dev/picar-motor-board` from then on. A
`/dev/serial/by-id/` path is an acceptable alternative.

**On first contact with the real board: the heartbeat, with the robot
server STOPPED.** The server cannot do this check: with no sensors it
refuses every FORWARD and clamps a standing forward `/wheels` to zero
(safety domain). Talk to the board directly instead. Wheels off the floor.
pyserial is installed alongside esptool (`pip install pyserial` otherwise).
UNCONFIRMED: not run against a board.

```python
import json, serial, time
s = serial.Serial("/dev/picar-motor-board", 115200, timeout=0.1)  # opening may reset the board
def send(c): s.write((json.dumps(c) + "\n").encode())
def frames(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        line = s.readline().decode(errors="replace").strip()
        if '"T":1001' in line:
            f = json.loads(line); print(round(time.monotonic() - t0, 2), f["L"], f["R"], f["odl"], f["odr"], f.get("ms"))
send({"T": 136, "cmd": 200}); send({"T": 131, "cmd": 1})
t0 = time.monotonic()
for _ in range(20):                      # 2 s at 0.1 m/s, re-sent every 0.1 s (under the 0.2 s heartbeat)
    send({"T": 1, "L": 0.1, "R": 0.1}); frames(0.1)
print("--- stopped sending T:1 ---")
frames(2.5)
send({"T": 1, "L": 0, "R": 0}); s.close()
```

Expected:

- `T:1001` lines about every 50 ms, carrying `L`, `R`, `odl`, `odr` (plus
  `odlt`, `odrt`, `ms` once flashed with the fork).
- While sending, `L` and `R` near 0.1 and the odometers climbing.
- After "stopped sending", `L` and `R` still near 0.1 until about 0.2 s
  later, then 0 (3.216's heartbeat): the board's heartbeat (`T:136`) stopped the wheels on its
  own. That is the check.
- If the wheels never stop, the heartbeat is not being honoured: do not run
  the car. If the wheels stutter WHILE sending (stops between re-sends), the
  firmware's loop does not check the heartbeat often enough for 200 ms:
  3.216.D, do not run the car until it is settled. If they stop at about 3 s, the `T:136` was lost and the firmware
  default applied.
- **Do not send `T:0` to stop the wheels.** It also drops the pan-tilt's
  torque and does not latch ([architecture](../../motor-board/ARCHITECTURE.md) D3). A `T:1` of zero
  stops them.

**Odometry over a measured metre.** The same session, on the floor, with a
tape along a straight line and the robot server still stopped. UNCONFIRMED:
not run.

1. Mark the chassis' front edge against the tape and note the last frame's
   `odl` and `odr` (and `odlt`, `odrt` on the fork).
2. Send `{"T":1,"L":0.1,"R":0.1}` every 0.2 s for 10 s (about 1 m), then
   zero, and wait for `L` and `R` to read 0.
3. Note the frame's odometers again, and measure the front edge's travel on
   the tape.

Compare each odometer's change with the tape, not with what was commanded:
the board's speed loop and the stop's slip are not the odometry. A ratio
that is consistently off 1.0 on both wheels means `WHEEL_D` is off for this
unit; a difference between the wheels on a straight run is a hardware-day
calibration item. Record both numbers against `PLAN-ros-alignment.md`'s
hardware-day entry; the bars are not yet written.

**Then start the robot server** with no world, because a board with no sim
body has no house to map, and the shipped `world.mode: sim` refuses it at
start-up:

```bash
ROBOT_MODE=hardware ROBOT_SERIAL=/dev/picar-motor-board WORLD_MODE=none uvicorn robot.server:app --port 8000
```

`GET /wheels` should turn `usable: true` within one feedback interval, and
turns run (FORWARD and REVERSE are refused until a lidar is configured: `ROBOT_LIDAR`, 3.42).
**Free the port** first: the stock `ugv_jetson` app must be disabled, or it
holds the port (3.26).

**Failure signatures:**

| Symptom | Meaning |
|---|---|
| Start-up `ValueError: mode: hardware needs ROBOT_SERIAL` | Set the port, or `SIM_MOTOR_BOARD=fake`. |
| Start-up `ValueError: SIM_MOTOR_BOARD=fake runs the simulated body as separate programs` | The URLs are unset: start `sim/body_server.py` and `sim/sensor_server.py` and set `SIM_BODY_URL` / `SIM_SENSORS_URL`, or use `service/tunnel/run.sh` (3.36). |
| Start-up `RuntimeError: no sim/body_server.py at ...` (or `sim/sensor_server.py`) | `SimBodyClient` waited 30 s for that program's `/health`: it is not running or is on another port. A sensor program that itself dies with `no simulated body state ...` waited 30 s for the body's shared memory: start the body first, with the same `SIM_BODY_SHM`. |
| Start-up `ValueError: world mode 'sim' needs the grid-world robot` | A real board with the shipped `world.mode: sim`. Add `WORLD_MODE=none`. |
| A FORWARD blocked with `Blocked FORWARD: distance=0.0cm < min=...cm (distance_sensor)`, or a standing forward `/wheels` answered `forward clamped: 0.0cm <= ...cm (distance_sensor)` | The real board with no sensors: `get_distance()` is 0.0, which reads as an obstacle at the bumper. Expected with no lidar configured (`ROBOT_LIDAR`, 3.42); only turns move (safety domain). |
| A REVERSE blocked with `rear clearance=0.0cm ... (astern_not_observed)`, or a settle ending "settle refused" | The real board with no lidar scan yet: the safety layer will not back up blind, and a straight correction either way is refused (safety domain). Expected with no lidar configured (`ROBOT_LIDAR`, 3.42). |
| `PermissionError` / EACCES on open | The user is not in `dialout`, or the udev rule is missing. |
| `get_wheel_state()` stays `usable: false` | No `1001` frames are arriving: wrong port or baud, feedback off, or another process holds the port. |
| `board_reboots` climbing | Brownouts. Check the pack and the motor stall current. |

## Verification

Counts are from `pytest --collect-only`, 2026-11-22.

| Test file | Tests | What it pins |
|---|---|---|
| `tests/test_board_speed_clamp.py` | 2 | Handoff 4j: an over-fast command (3.2 m/s) reaches the fake board scaled into +/-2.0 with its ratio kept, not dropped. |
| `tests/test_board_http.py` | 24 | 3.217, commands over the fake's web route, feedback over its pty: a 1 m drive with no command on the pty; `stop()` returns in under 20 ms and lands within 150 ms under 30 ms replies (200 stops); the board ends on the last command issued (1,000 seeded interleavings); a refused route raises `WheelFeedbackLost` within 0.35 s, never traps a stop, and the board's heartbeat stops the wheels; a route that returns drives again; one lost request does not trip the 200 ms heartbeat (20 runs); `set_wheel_velocity()` p99 under 2 ms; `/health`'s description; the factory's switch; the URL's shape. From its reviews: a speed the board acts on late is stopped by the repeated stop, not the heartbeat; an unanswered set-up command cannot hold a stop or let a speed past; a speed that waited out a dead route is sent as a stop; an idle-closed connection is not a failure; a failed constructor leaves no thread. |
| `tests/test_pan_tilt.py` | 18 | 3.220: the fake's gimbal rules (clamps and whole-degree `map()`, the angle in feedback only in module type 2, `T:4` RAM-only against `T:900` saved, travel at the servo's speed, `T:135`, the web face); every host command within +/-90 deg and tilt 0; the set-up's module type and centring; off is exactly as before; at rest the frame's `pan_deg` within 1.0 deg of the servo; while moving `settled: false`; a look at rest returns settled, one while the wheels turn returns at once; a board not reporting the angle gives no frame (never pan 0) and a missed reboot gets `T:4` again; a lost Wi-Fi command never reads as reached, faults within 1.75 s and recovers when the route returns; a look never changes a wheel command; the twin's `LOOK_*` through the robot server reach -90 +/- 1 deg within 1.0 s; the factory switch is off on a real board unless set. |
| `tests/test_fake_esp32.py` | 11 | R7's criteria on the ROS Driver fake: `T:1` closed loop and its +/-2.0 guard, `T:13`, `T:11` turning the PID off, `T:130` and `T:131`, the heartbeat in every mode, wheels over the wire, a severed host stopped by the board's own heartbeat. |
| `tests/test_ros_driver_board.py` | 24 | 3.25: frame keys, cm odometers truncated toward zero, integer edges, measured speeds, at most one frame per 50 ms, `T:142`, `T:900`, 660 in one place, the anchor bound and its mutation, reboot absorption with no false reboot, set-up re-sent, a clear FORWARD within `STOCK_STRAIGHT_BAR_CM` (1.34). Stock turns within +/-1 deg is a NON-STRICT XFAIL with a guard: the mean of ten within 2.0, none past 6. |
| `tests/test_firmware_fork.py` | 21 | 3.28 and 3.29: the patch adds only three keys inside `baseInfoFeedback()`; it applies to `2e7df97` and compiles (both skip unless `PICAR_FIRMWARE_SRC` names a checkout, and the compile also needs `arduino-cli`); the fork's keys and units; the board clock and its wrap; frame size; the 0.1 mm bound; reboots on the board clock; partial keys read as stock; fork straights and turns. |
| `tests/test_settle_pass.py` | 8 | 3.29: fork turns within +/-1 deg, a FORWARD is a cell on either firmware, an overshoot corrected when clear, a correction toward furniture clamped, a `stop()` during the settle ends it. |
| `tests/test_robot_contract.py` | (the `hardware` backend) | `HardwareRobot` over the fake passes the backend-agnostic body contract. |
| `tests/test_guarded_verbs.py` | (hardware cases) | 3.22's guarded verbs over the fake board. |

Recorded numbers:

| Measure | Stock (3.25) | Fork, 0.1 mm (3.29) |
|---|---|---|
| Worst wheel-travel error, 30 s stop-go, 5% lines lost | 1.02 cm (1.63 in the zero bucket); speed integral alone 9.57 | 0.047 cm, 0/1028 over the bar |
| Clear turns within +/-1 deg (120) | 60/120 and 51/120 in two identical runs; worst 4.6-5.2 | 120/120 with the settle, worst 0.84; 118/120 without |
| Median turn time | 0.66 s | 0.82 s |
| Clear FORWARD, 30 cm | 117-119 of 120 within 1.0 cm, worst 1.13 | worst 0.13 cm over ten |
| Reboot mid-drive | 8/8 detected, at most 1 cm move | 8/8 from `ms`, at most 0.058 cm move |

R7's heartbeat drill (3.16, criterion 4, `tests/test_fake_esp32.py`): a
host severed mid-motion, and the board's own heartbeat stopped the wheels
1.5 s later, within one loop.

Checklist for a change:

- Run `pytest tests/test_fake_esp32.py tests/test_ros_driver_board.py
  tests/test_firmware_fork.py tests/test_settle_pass.py
  tests/test_robot_contract.py`.
- Run them again with `SIM_BOARD_FIRMWARE=fork`.
- Judge accuracy on `FakeEsp32.sent_truth` or on the body's truth, never on
  what the host reports about itself.
- A change to the patch must keep `odl` and `odr` untouched, and must
  re-record the compile size here.

## Known gaps

- **Not flashed; no real board yet.** Every number above is against the
  fake.
- **The pan-tilt is unproven on the board** (3.220 criterion 6): whether
  `T:4 cmd 2` makes this board report `pan`/`tilt`, `PAN_SIGN`, the
  limits by eye, the lidar's self box with the camera panned, and the
  feedback rate with the servo bus read every loop. A servo that stops
  answering reports its last position (firmware), which is caught only
  when a later goal is not reached.
- **The web route is unproven on the board** (3.217). Its tests are
  against the fake. Not known: whether an HTTP `T:1` feeds the board's
  heartbeat, whether the board's unanswered requests are lost on the way
  (as the fake models) or a busy board, and the rate with motors and lidar
  running. A request the host abandoned can still be acted on late: the
  stop is repeated three times, 50 ms apart, so a stale speed that lands
  after it runs for about one period (the heartbeat, 200 ms, is the bound
  if the repeats are lost too). A stop issued while another request is on
  the wire waits for it: at most `COMMAND_TIMEOUT_S` (0.10 s) longer than
  over serial.
  `service/tunnel/run.sh` and the fake-board programs (3.36) do not carry
  the route, so no live simulator run uses it.
- **`T:0`'s servo half is not modelled.** The firmware's `T:0` is
  `emergencyStopProcessing()` (`st.EnableTorque(254, 0)`, every bus servo,
  the pan-tilt included) then `setGoalSpeed(0, 0)` (`uart_ctrl.h` at
  `2e7df97`; settled 2026-10-18, 3.76 M2.8). The fake does the wheel half.
  The host never sends it.
- **The fake's limits.** Its PID is ideal: no `THRESHOLD_PWM` deadband, no
  dynamics. Its IMU fields are zero and its voltage is constant unless given a drain. Its loop
  and feedback timing are assumed.
- **No straight driving without a lidar.** On the car with no lidar configured and no sensors,
  every FORWARD is refused (`get_distance()` 0.0) and every REVERSE too
  (`astern_not_observed`), so no straight verb runs and no straight
  correction either; only turns move and settle. The rule is the safety
  domain's ([safety engineering](../safety/ENGINEERING.md), Known gaps).
- **Stock turns miss the +/-1 degree bar.** Stock frames carry no
  timestamp, so arrival jitter limits heading. On the car, heading wants the
  gyro (`gz`) or SLAM.
- **Not reported by the backend:** the measured wheel speed in
  `velocity_rad_s` (it reports the commanded one) and the IMU. The battery
  voltage is reported since 3.70 (`get_battery()`); the fake's drain is a
  straight line, not a pack's discharge curve, and no sag under load.
- **Two hardware-day measurements:** `wheel_separation_multiplier` (the
  skid-steer effective track) and the real device path (`/dev/ttyTHS1`
  versus USB).
