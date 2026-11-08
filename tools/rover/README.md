# tools/rover: bring-up tools for the Rover (3.168)

For 3.140.B (L0). Run from the repository root on the Jetson with its
`.venv`, under `tools/jetson/board_lock.py`. Stop `picar-stack` first for the
serial tools (they refuse a held port); run them under `sudo` so a holder
cannot hide. **Each serial tool opens its port read-only:
none can send the board a wheel or servo command.** Flashing is not here.

| Row | Tool | What it needs from the day |
|---|---|---|
| L0.3 | `python -m tools.rover.serial_route --out r.json --rules-out 99-picar-rover.rules`, then `bash tools/rover/install_udev.sh 99-picar-rover.rules [--yes]` | Which port is which, and whether the D500 shows up as its own USB serial device behind the driver board. Without `--yes` the script changes nothing |
| L0.5 | `python -m tools.rover.board_compare --port /dev/ttyTHS1 --seconds 60 --out c.json` | Press reset during the capture for the boot text and the silence, and turn a wheel by hand (the row is not met while speeds and odometers stay at zero). Every difference goes into `sim/fake_esp32.py` |
| L0.6 | `python -m tools.rover.lidar_check capture --port /dev/picar-lidar --out cap.bin`, then `score cap.bin --wall-bearing B --wall-m M --box-m M` | A taped wall and a narrow box (15 cm or less, face square to the lidar) dead ahead, nothing nearer within 30 degrees of it; distances from the lidar's centre |
| L0.4 | `python -m tools.rover.dump_firmware --check`, then `--port /dev/ttyTHS1 [--before no_reset]` | esptool on the board. On the 40-pin UART: hold BOOT, tap EN. Reads only; check `esptool --help` still takes `read_flash`, `--before no_reset` (not tried against a real esptool). The image stays in `~/rover-firmware/` |
| L0.8 | `python -m tools.rover.camera_check --device /dev/videoN --seconds 620` | Each camera in turn, 10 minutes, beside the running stack with the stack's own camera unset (this tool opens the device itself) |

Dry-run on the fakes by `tests/test_rover_tools.py`. `board_compare` reads
the fake's real output as its reference, so the two cannot drift apart.
What it cannot see read-only: whether the board's INPUT is deaf after a reset
for longer than its output is silent.
