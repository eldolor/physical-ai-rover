# Contributing

Copyright (C) 2026 Anshu Gaind.

This project is licensed under the GNU AGPL-3.0-only (`LICENSE`). The firmware
patch in `firmware/ugv_base_ros/` is GPL-3.0. `sim/fake_esp32.py` and its tests
(`tests/test_fake_esp32.py`, `tests/test_ros_driver_board.py` and
`tests/test_firmware_fork.py`) are GPL-3.0-only, because they
reimplement and pin behaviour from Waveshare's GPL-3.0 firmware.

## Contributor licence agreement

Contributions are accepted under a contributor licence agreement:

* you license your contribution under the AGPL-3.0-only, like the rest of the
  project (the firmware patch under the GPL-3.0, and `sim/fake_esp32.py`
  and its tests under the GPL-3.0-only); and
* you also grant the copyright holder the right to relicense your
  contribution under other terms.

By opening a pull request, you agree to these terms.

You keep the copyright in your contribution. Please ask before opening a
substantial pull request.

## Working rules

Where the rules live (read them there; they are not copied here):

* **Parallel sessions and claiming work:** `docs/guides/PARALLEL-SESSIONS.md`.
  Claim a plan step first (`python tools/plan_section.py claim <step>`, rule
  12); if it is refused, stop and report.
* **Code review:** `CLAUDE.md` section 6, "Code review" (`/code-review` on
  every code change; the Thermos bug pass on a risky one).
* **The push gate:** `tools/hooks/pre-push` runs the checks described in
  `docs/guides/PARALLEL-SESSIONS.md` rule 6.
* **Two specs per component:** an architecture spec (what and why) and an
  engineering spec (how), kept in step with the code. See `docs/README.md`
  and `docs/decisions/0001-architecture-vs-engineering-specs.md`.
