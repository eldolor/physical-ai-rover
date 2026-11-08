# Rover arrival runbook

For one person with the car on the bench. Claude reads the board and the logs
over ssh; you do what needs hands. Written 2026-11-08 (3.169).

The days: today is Sunday 11-08 (delivery, afternoon). You are home Monday
11-09 and Tuesday 11-10, then away 11-11 to the week of 11-23. The Amazon
return window closes **2026-12-08**.

Row numbers such as L0.3 are from `docs/plans/ros-alignment/3.140-rover-live-checks.md`.
Its bars are the bars here. Claude records each result there.

## Hard rules

Read these first. They hold on every step.

1. **Power-off and cable pulls are by your hand only.** Claude never powers
   anything off and never asks another program to.
2. **No motor motion unless you are there and say go.** That covers the wheels
   and the pan-tilt. Claude asks, you type "go", for each new motion. "Go" for
   one step does not carry to the next.
3. **Wheels on the stand first.** All wheels clear of everything. Spin each by
   hand to check before you say go. No floor run happens in these two days.
4. **The Stop and the power switch stay in reach** during every motion step.
   Keep one hand free.
5. **A surprise ends the session.** Contact, motion nobody asked for, smoke,
   heat, a smell or a brownout: power switch off, then tell Claude. The row is
   recorded FAILED. Nothing further runs until you say so.
6. **Never type the Jetson password into a chat.** If a step seems to need it,
   stop and say so.
7. **Maps never get committed or uploaded.** The same goes for photos and video
   of the house. Keep them on your own phone or the board.
8. **Safety stays on.** Nothing bypasses `robot/safety.py`, the watchdog or the
   board heartbeat, even on the stand.
9. **Claude only records your own answers.** When a run needs the checklist
   (on the stand, Stop in reach), Claude asks and writes down what you say.

## Have on hand

* Calipers, and a steel rule.
* A tape measure, and a carpenter's square if you have one.
* A stand that lifts **all six wheels** clear and does not tip. Sturdy boxes
  work. The body is 253 x 231 mm.
* Masking tape and a marker.
* A narrow cardboard box, **15 cm wide or less** and about 20 cm tall, for the
  lidar and camera targets. A wider box reads wrong at the lidar.
* Small screwdrivers and hex keys (M2.5, M3, M4).
* Spare M2.5 and M3 screws, nylon or steel washers, and M2.5 standoffs.
* A multimeter.
* An Ethernet cable long enough to reach the bench.
* A phone with slow-motion video.
* Your four Molicel P26A cells, and the Jetson's stock 19 V adapter.

## What must be settled before 2026-12-08

Keep the box and all packing until then. These are `JETSON-BOM.md` 9.5's
checks. A bad result on any of them is a reason to think about returning it.

| Check | When |
|---|---|
| The Jetson fits, or an adapter plate is practical (a misfit alone is not a reason to return it) | today, Monday |
| The ESP32 module's shield is read | today |
| The serial route is found | Monday |
| `T:1001` carries `odl` and `odr` | Monday |
| The stock firmware is dumped | Monday |
| 15 W stress with motors on the stand | Tuesday |
| The charger check | Tuesday, in part; finish on your return |
| Odometry over a measured metre (floor) | **on your return** |
| The safety sweep against the real lidar (floor) | **on your return** |
| Flash our firmware fork, only after all of the above | **on your return** |

You are back the week of 11-23 and Thanksgiving is 11-26. That leaves about
two weeks. Do the two floor checks first when you get back.

## Where every L0 and L1 row goes

| Row | What | Day | Step |
|---|---|---|---|
| L0.1 | The Jetson fits the deck | today, Mon | T5, M2 |
| L0.2 | No stock software holds the port | Mon | M5 |
| L0.3 | Serial route, stable names, 3 reboots | Mon | M5 |
| L0.4 | Dump the stock firmware | Mon | M7 |
| L0.4 | Flash the fork | **does not fit** | Rule above: it comes after the return-window checks, and two of those are floor runs |
| L0.4 | `T=1` reads as m/s | Tue | U1 |
| L0.5 | The protocol matches the fake | Mon | M6 |
| L0.6 | The D500: capture, rate, wall, box | Mon | M8 |
| L0.7 | Lidar against real rooms | **part** | U7, one room if time. The whole survey is 3 or more spots in every room the missions use: on your return |
| L0.8 | Both cameras stream | Mon | M10 |
| L0.9 | Camera field of view | Tue, if time | U6 |
| L0.10 | Starts at boot, 5 cold boots | Tue, if time | U8 |
| L0.11 | The charger | **part** | U5. The row wants a full tiered mission running. That needs SLAM and nav2 on the car, which are L3 |
| L0.12 | 15 W stress with motors | Tue | U4 |
| L1.1 | The geometry block | today, Mon | T6, M9 |
| L1.2 | Chassis width, lidar to bumper | today, Mon | T6, M9 |
| L1.3 | Pivot scrub | **does not fit** | Floor. Floor runs need every L0 row met first, and each floor type |
| L1.4 | Stopping distance | **does not fit** | Floor: 25 or more filmed stops per floor type on a taped grid |
| L1.5 | Straight-line drift | **does not fit** | Floor |
| L1.6 | The move quantum | **does not fit** | Floor |
| L1.7 | The heartbeat, cable pulled | Tue | U3 |
| L1.8 | `T:0` is never sent | Claude, no hands | any time |
| L1.9 | Gyro against pivots | **does not fit** | Floor |

What needs you on your return: the rest of L0 and all of L1's floor rows. Plan
on several days for those.

## Who runs the serial tools

The tools are in `tools/rover/` (`tools/rover/README.md`). Each serial tool only
listens: none can send the board a wheel or servo command.

Claude runs them on the Jetson over ssh, from the repository root, under
`tools/jetson/board_lock.py`, with `picar-stack` stopped first. On real ports
they must run under `sudo`, so a program holding a port cannot hide. Your
account on the Jetson has passwordless sudo for development, so Claude uses
`sudo -n` and **nobody types a password**.

If `sudo -n` ever asks for a password, Claude stops. Then you run that one
command yourself, in a terminal on the Jetson, and paste back the output. Never
the password.

---

## Today, Sunday: no power to the motors

About 2 to 3 hours, plus charging.

### T1. Check the box

Do: open it and lay everything out. Look for a crushed corner, a cracked deck
or a bent wheel. Keep all packing.

Look for:
* The chassis, the ROS Driver board and the UPS.
* The D500 lidar, the OAK-D Lite and the pan-tilt camera.
* The 12.6 V charger.
* The double-row ribbon cable for the 40-pin header.
* The DC5525 Jetson power lead.
* The USB leads.
* The hardware bag: silver M2.5\*8 pillars, copper pillars, M2.5\*8 countersunk,
  M4\*6, M3\*6 hex.
* **A wireless card and antennas.** A separate card in the box suggests the kit
  was designed for Waveshare's carrier, not NVIDIA's. Our Jetson should already
  have one.

Read back: anything missing or damaged, and whether there is a wireless card.

Stop if: there is damage, or the ribbon or charger is missing. Do not power it.
Tell Claude.

### T2. Read the board's name

Do: read the text on the motor board. Read the shield on the ESP32 module.
Remove a cover only if the screws are obvious.

Read back: the board's name, and the module's name. Expected: "ROS Driver for
Robots", and ESP32-WROOM-32.

Stop if: the board says "General Driver". On that board a wheel command of 0.5
is full power. No motion until Claude has re-checked everything.

### T3. Fit the cells

Do: inspect all four cells. Look for tears near the + end, and for dents.
Pick three. Open the bay (four M3\*6 screws). Match the holder's polarity.
Close it. The fourth is a spare.

Read back: all clean, and three fitted.

Stop if: any wrap is torn or dented on the three you need, or a cell gets warm.

### T4. Charge

Do: plug in the 12.6 V charger. The UPS stays off until you do. Stay nearby the
first time. Do not leave it charging overnight or unattended.

Read back: the charger's light, and the time you started.

Stop if: heat or a smell. Unplug the charger.

### T5. Calipers on the Jetson (L0.1)

Do: with the Jetson still on the desk, measure its four mounting holes, centre
to centre, in both directions, and the hole diameter. Do not touch the board
itself. Then measure the same spans on the deck.

Read back: the two spans and the diameter.

The deck has 86.00 x 58.00 mm and 86.00 x 23.61 mm. Deck holes are 3.68 mm. M2.5
holes still mount with washers.

Stop if: neither span matches. That is not a reason to return the car. The
Jetson also carries the top plate, so an adapter must carry that too. Claude
will draw the plate. Until then the Jetson sits beside the car for Monday's
steps, if the ribbon reaches.

### T6. Measure part A

Do: fill in part A of the sheet below. Only what needs no Jetson.

Read back: the whole part.

Stop if: a repeat differs by more than 2 mm. Measure again.

### T7. Set up the stand

Do: put the car on the stand. Spin each wheel by hand. Check that nothing
touches.

Read back: all six clear.

Stop if: it rocks. Fix the stand first.

---

## Monday: mount, wire, first power, no motor motion

A full day. If the Jetson needs an adapter, M2 slips and the rest can run with
the Jetson beside the car.

### M1. Free the Jetson

Do: tell Claude "clear the board". Wait for Claude to say so. Other sessions
may be using it, and their work has to be off it first. Then shut it down
yourself, unplug the 19 V adapter and move it.

Read back: that it is off.

Stop if: Claude says someone still holds it. Wait.

### M2. Mount it (L0.1)

Do, following Waveshare's Acce video (youtu.be/R0-QG33DznY):
1. Remove the copper pillars that carry the top plate.
2. Set the Jetson on the silver M2.5\*8 pillars.
3. Refit the copper pillars above it.
4. Fit the D500 to the top plate with M2.5\*8 countersunk screws.
5. Fit the OAK-D Lite with M4\*6 screws.

Read back: which span you used, and whether the board bows when you tighten.

Stop if: the carrier board bends or a screw forces. Back off.

### M3. Wire it, all power off

Do:
1. Ribbon from the Jetson's 40-pin header to the driver board. Check pin 1 at
   both ends.
2. DC5525 lead into the Jetson's barrel jack. The stock 19 V adapter stays out.
3. D500 into the port on the driver board.
4. Driver board, USB camera and OAK-D Lite into three USB-A ports on the
   Jetson.
5. Ethernet to the bench.

Read back: that the ribbon reaches without strain, and which ports you used.

Stop if: the ribbon does not reach or fit. Do not force it.

### M4. First power, on the stand

Do: hand on the power switch. Turn on the UPS. Watch the wheels and the
pan-tilt.

Read back: what moved, if anything, and the lights. Claude reads back the ssh
login, `/health` and the pack voltage.

Stop if: anything moves, there is smoke or a smell, or Claude cannot reach it
after 3 minutes. Power switch off.

### M5. Stock software and the serial route (L0.2, L0.3)

Do: nothing with your hands unless Claude asks. You say go for each of the
three reboots.

Our Jetson runs our own image, so Waveshare's app should not be there. Claude
checks anyway.

Tools, run by Claude:
* `python -m tools.rover.serial_route --out r.json --rules-out 99-picar-rover.rules`
  listens on each port and says which is the board and which the lidar.
* `bash tools/rover/install_udev.sh 99-picar-rover.rules` shows what it would
  install and changes nothing.
* The same command with `--yes` installs the names and adds the user to
  `dialout`. Claude shows you the first output and waits for your go.

Read back: "go" for the install, and for each reboot.

Claude reports: nothing but our code can hold the port, whether the D500 shows
up as its own USB serial device, and both devices open by name after 3 reboots.
Expected serial route: `/dev/ttyTHS1`.

Stop if: the board does not answer on any port. Check the ribbon, power off
first.

### M6. Protocol against our fake (L0.5)

Claude runs `python -m tools.rover.board_compare --port /dev/ttyTHS1 --seconds 60
--out c.json`, a 60 second capture. During those 60 seconds, when Claude says
start:

1. **Turn each wheel by hand**, a few turns, left side and right side. Motors
   are not driven in this step. If you skip this, the speeds and odometers read
   `not_exercised` and the row is not met.
2. Press the driver board's reset button once, about halfway through.

Read back: that you turned wheels on both sides, and when you pressed reset.

Claude reports: `T:1001` has `odl` and `odr`, the battery reads about 12 V, how
long the board is silent after a reset, and each difference from our fake. The
tool only listens, so it cannot tell whether the board's input stays deaf for
longer than its output is silent.

Stop if: there is no `odl` or `odr` in the frames. It means the wrong board or
firmware. No motor motion until this is explained.

### M7. Dump the stock firmware (L0.4, first half)

Claude first runs `python -m tools.rover.dump_firmware --check`, which touches
nothing. Then `python -m tools.rover.dump_firmware --port /dev/ttyTHS1`, with
`--before no_reset` on the 40-pin UART.

Do: on the 40-pin UART, when Claude says, **hold BOOT and tap EN** on the driver
board, then let go of BOOT. The tool only reads the chip.

Read back: that you did it, and any light that changed.

Claude reports: the dump's size, a matching second read, and that it is kept in
`~/rover-firmware/` on the Jetson. The `--before no_reset` option has not been
tried against a real esptool, so Claude checks esptool's own help first.

Stop if: the two reads differ. Do not flash anything, now or later.

### M8. The lidar (L0.6)

Do: car still on the stand.
1. Set the car with a plain wall **to one side**, about 1 m away. Measure from
   the lidar's centre to the wall, square to the wall. Write it down.
2. Put the narrow box dead ahead, about 0.6 m away, on the car's centreline, at
   the lidar's height, with its face square to the lidar. Measure from the
   lidar's centre to its face.
3. Clear the front: nothing nearer than the box within 30 degrees either side
   of it.
4. Note which way the D500's zero mark faces. Waveshare's drawing says left.

Claude runs `python -m tools.rover.lidar_check capture --port /dev/picar-lidar
--out cap.bin`, then `python -m tools.rover.lidar_check score cap.bin
--wall-bearing B --wall-m M --box-m M` with your numbers.

Read back: which side the wall is on, the wall distance, the box's distance, and
the zero's direction.

Claude reports, against the bars: under 0.1% bad packets, 10 Hz within 10%, the
wall within 2 cm, the box at 0 degrees within 2.

Stop if: the wall reads more than 2 cm off twice. Do not trust the lidar for
anything else until explained.

### M9. Measure part B

Do: fill in part B of the sheet. Everything is mounted now.

Read back: the whole part.

Stop if: a repeat differs by more than 2 mm. Measure again.

### M10. The camera check (L0.8)

Do: nothing with your hands. Claude runs `python -m tools.rover.camera_check
--device /dev/videoN --seconds 620`, one camera at a time, **10 minutes each**,
so about 20 minutes for both. It runs beside the stack, with the stack's own
camera unset, because the tool opens the device itself. Other no-hands work can
go on meanwhile, but not on that camera.

Claude reports: for each camera, frame age p90 (bar 100 ms) and no stale frames
served.

Stop if: a camera is not seen. Check its USB lead with the power off.

### M11. Charge

Do: charge the pack while you are nearby. Unplug it before bed.

Read back: the pack voltage from Claude.

---

## Tuesday: first motion, on the stand, with you there

A full day. Do the steps in order. U1 to U5 are the ones that matter. Stop
adding when you need the last hour for U9.

### U1. First wheels (L0.4, third part)

Do: confirm all six wheels are clear, your hand is at the switch, and the Stop
is open on your phone. Say go. Claude runs the robot server in hardware mode
and sends one short, slow command at 0.1 m/s.

Read back: did both sides turn the same way, forward, and at a walking pace or
less?

Claude reports: the measured speeds against the 0.1 m/s asked for. Next it asks
for one go for a verb, and one for a stream of `T:1`. These two count as the
"stand first" runs the floor runs need later.

Stop if: a wheel runs fast, a side runs backward, or anything keeps moving
after the command ends. Power switch off.

### U2. The pan-tilt

Do: say go. Claude moves it a few degrees each way, then centres it.

Read back: that it moved as asked, and that the cable does not pull.

Stop if: it strains against the cable.

### U3. The heartbeat, cable pulled (L1.7)

Do: start slow-motion video of your hand and the wheels. Say go. With the wheels
spinning slowly, pull the ribbon straight off, in one motion, by your own hand.

The ribbon carries the serial lines beside power pins. Pull it straight, never
at an angle. If you would rather not pull it live, say so. Claude can test
silence by stopping the host instead, and the row is recorded as not done by
cable.

Read back: the time from the pull to the wheels stopping, from the video.

Bar: stops within 1.5 s, plus one frame.

Stop if: the wheels do not stop within 3 s. Power switch off. No more motion.

Reseat the ribbon with the power off.

### U4. 15 W stress with motors (L0.12)

Do: say go. Claude loads the Jetson at 15 W with the wheels working on the
stand, for about 20 minutes. Put the multimeter on the Jetson's feed if you can
reach it.

Read back: the lowest voltage you see.

Claude reports: no brownout, no throttling. That decides whether the separate
Jetson battery gets bought.

Stop if: the Jetson reboots or the voltage falls under 10.5 V.

### U5. The charger, in part (L0.11)

Do: with the stack running, plug the charger in and out when Claude asks, 10
plugs and 10 unplugs, about 15 seconds apart. Then leave it plugged in for 30
minutes.

Read back: nothing, unless something changes.

Claude reports: no reboots, the lowest `v`, and whether the pack gains charge
while running. The row is still open, because it wants a tiered mission running
(see the table above).

Stop if: the Jetson reboots.

### U6. Camera field of view, if time (L0.9)

Do: tape marks at known bearings, Claude will give the list. Stand the box on
each mark in turn.

Read back: when the box is on each mark.

Bar: within 1 degree across the frame.

### U7. One room for the lidar survey, if time (L0.7)

Do: with the wheels free and no motion, carry the car to 3 taped spots in one
room with glass or a mirror, for 30 seconds each.

Read back: the room, and the surfaces there.

### U8. Cold boots, if time (L0.10)

Do: power switch off, then on, 5 times, when Claude asks. Claude times each one.

Bar: healthy within 120 s, 5 of 5.

Stop if: a boot does not come back in 5 minutes.

### U9. Leave it safe for the away period

Allow an hour. By your hand:

1. Tell Claude to stop the stack. Shut the Jetson down yourself.
2. UPS off. Power switch off.
3. Unplug the kit's DC5525 lead from the Jetson.
4. Pull the ribbon from the 40-pin header.
5. Take the three cells out. Store them somewhere cool and non-flammable.
6. Plug in the Jetson's stock 19 V adapter, and keep Ethernet in. Check that
   the kit's lead is out first, so the two supplies are never joined.
7. Power the Jetson on. Tell Claude. Claude confirms it can reach it, and that
   nothing can reach the motors.

Left powered: the Jetson on the wall adapter. Not the pack. No motion is
possible, because the motor supply has no cells behind it, and no work while you
are away may ask for any.

Read back: each of the seven, done.

Stop if: Claude cannot reach it. Fix that before you leave.

If you would rather keep the motor board and lidar live for remote work, the
only route is the 12.6 V charger with the cells in. That means lithium cells on
a charger for two weeks with nobody home. Do not do that unless U5 passed and
you decide to.

---

## Measurement sheet

Measure each value three times. Bar: within 2 mm of each other. Write all three
in mm unless it says otherwise. "CAD" is the value in the code today, from
Waveshare's drawing or the parts list, and is not a target.

**The reference point.** The rotation centre, called R below, is the midpoint
between the front and rear axle centres, on the car's centreline. Mark it with
tape on the side of the body. "Ahead of R" is forward along the centreline, with
behind as negative. Heights are from the floor with the car on the floor and the
pack fitted. Do not rest on the stand for heights.

### Part A: today, no Jetson needed

| Name | CAD | How to measure | 1 | 2 | 3 |
|---|---|---|---|---|---|
| `wheel_separation` (front wheels) | 172 (drawing 174.5) | Calipers or rule. Outer tyre edge to outer tyre edge, then inner to inner. Average the two. | ___ | ___ | ___ |
| `wheel_separation` (rear wheels) | 172 | The same, at the rear. | ___ | ___ | ___ |
| `wheel_width` | 42.5 | Calipers across the tyre tread. | ___ | ___ | ___ |
| wheel diameter (`wheel_radius` is half) | 80 | Calipers across the tyre, unloaded. | ___ | ___ | ___ |
| `CHASSIS_WIDTH_CM` (width with wheels on) | code says 16.5 cm, wrong; expected about 19.8 cm or the body's 23.1 cm | The widest point of the car, wheels included. Give it in **cm**. | ___ | ___ | ___ |
| body length | 253 | Front edge to rear edge, widest points. | ___ | ___ | ___ |
| body width | 231 | Side to side, the shell. | ___ | ___ | ___ |
| front axle centre to rear axle centre | 171 | Tape along the side, centre of hub to centre of hub. | ___ | ___ | ___ |
| front body edge to front axle centre | not known | Along the side. This places R on the body (Claude works out `axle_x`, CAD 0). | ___ | ___ | ___ |
| `deck_height` | 40 | Calipers: the chassis plate's thickness at an exposed edge. | ___ | ___ | ___ |

### Part B: Monday, everything mounted

| Name | CAD | How to measure | 1 | 2 | 3 |
|---|---|---|---|---|---|
| `laser_x` (lidar ahead of R) | 40 | Horizontal, R to the lidar's spin axis. Safety depends on this one. | ___ | ___ | ___ |
| `laser_z` (lidar height) | 120 from floor | Floor to the lidar's window centre. | ___ | ___ | ___ |
| lidar zero direction | turned 90 deg, facing left | The mark on the housing. Say which way, in words. | ___ | | |
| `safety.sensor_to_bumper_cm` | 0.0, wrong | Horizontal, lidar axis to the frontmost point of the car, in **cm**. | ___ | ___ | ___ |
| `pan_x` (pan axis ahead of R) | -9 | Horizontal, R to the pan axis. Behind is negative. | ___ | ___ | ___ |
| `pan_z` (pan joint height) | 168 from floor | Floor to the pan joint. | ___ | ___ | ___ |
| `camera_x` (lens ahead of the pan axis) | 48 | Horizontal, pan axis to the lens front, camera centred. | ___ | ___ | ___ |
| `camera_up` (lens height) | 210 from floor | Floor to the lens centre. Claude subtracts the pan joint. | ___ | ___ | ___ |
| `camera_pitch` | 15 deg down | A phone inclinometer flat on the camera's face, centred. In **degrees**. | ___ | ___ | ___ |
| `pan_limit` | 90 deg each way | Power off. Turn the camera gently by hand until the cable just goes tight. Do not force it. Left and right, in **degrees**. | L ___ | R ___ | |
| OAK-D ahead of R | 65 | Horizontal, R to the lens. | ___ | ___ | ___ |
| OAK-D lens height | 102 from floor | Floor to the lens. | ___ | ___ | ___ |

Claude changes every copy of these together, and `tests/test_urdf.py` and
`tests/test_depth_veto.py` are re-run. You only measure.
