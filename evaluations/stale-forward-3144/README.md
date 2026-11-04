# 3.144: F9 on the desktop against the fix (criterion 4)

`python evaluations/jetson-fmea-3135/fmea.py F9 --out desktop/F9-run<i>`, five
runs, 2026-11-04, rev `c55367cb`, under the exclusive desktop lock: 3.135's
stack (`SIM_LIDAR=fake`, `SIM_MOTOR_BOARD=fake`, no ROS), a frontier mission
in `scaled_house`, the fake D500 SIGSTOPped 10 s, 0.15 s into a FORWARD.

| Run | true path, fault + 0.3 s to SIGCONT (bar 2 cm) | contacts | FORWARDs refused `ahead_not_observed` | recovery | verdict |
|---|---|---|---|---|---|
| 1 | 0.0 cm | 0 | 5 | 2.51 s | pass |
| 2 | 0.0 cm | 0 | 5 | 2.60 s | pass |
| 3 | 0.0 cm | 0 | 5 | 2.62 s | pass |
| 4 | 0.0 cm | 0 | 5 | 2.63 s | pass |
| 5 | 0.0 cm | 0 | 5 | 2.56 s | pass |

The FORWARD in flight at the fault had stopped 0.23-0.29 s after it (its
end); every FORWARD the brain sent in the outage was refused, and its turns
went ahead in place (turns stay allowed blind), so the wheels' last non-zero
setpoint ends ~1.8 s after the fault (`wheels_zero_s`), with no travel.
Before the fix the shipped build moved 104.5 cm here (3.135.A amendment 2)
and 101-105 cm on the board. Each run's `result.json`, logs and gzipped
truth log are in `desktop/F9-run<i>/`.
