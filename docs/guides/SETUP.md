# Setting up a machine

A fresh machine to a working checkout: the suite passing, the robot server
and the brain running, the phone UI (the "twin") open in a browser, and one
simulated search finished. For a person at a keyboard. A Claude Code cloud
session does steps 2-4 itself (`.claude/hooks/session-start.sh`).

Every command below was run on 2026-11-21 in a second, fresh clone, in a new
virtual environment, on Linux. What each printed is noted where it matters.
Nothing here spends money: no API key is needed, and the simulator needs no
cloud.

Words you may not know (twin, brain, driver, frontier, outcome): see
[GLOSSARY.md](GLOSSARY.md). The system on one page:
[`docs/ARCHITECTURE.md`](../ARCHITECTURE.md).

## 1. Python

Use a Python 3.10 or newer, in a virtual environment.

* **Verified here: Python 3.13** (a fresh `python3 -m venv`, then
  `pip install -r requirements.txt`, 14 s; 6,560 tests collected).
* **Also verified, on the Jetson: Python 3.10.12** (JetPack 6's system
  Python; `docs/plans/ros-alignment/3.33-*.md` records the suite passing
  from that board's `.venv`). Python 3.10 is a floor the board forces, not a
  preference.
* Not verified: 3.11 and 3.12 (nothing in `requirements.txt` rules them out).

**Trap: the system Anaconda Python.** It has an older FastAPI and disagrees
about a few tests (`CLAUDE.md` section 1). Make the venv from a plain Python
and run everything from it.

## 2. The virtual environment and the requirements

```bash
git clone <this repository> physical-ai-rover && cd physical-ai-rover
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`requirements.txt` carries the runtime (FastAPI, uvicorn, Pillow, numpy,
scipy, boto3), the test tools (pytest, pytest-xdist, pytest-cov,
pytest-playwright, cfn-lint) and `py_trees`. It needs no API key.

## 3. A browser for the UI tests

The UI tests (`tests/test_ui*.py`, `tests/test_frame_source.py`) drive the
real twin in a real Chromium at a 390 px phone viewport. **Without a browser
they SKIP or ERROR, not fail** (`requirements.txt`'s comment says skip;
`tools/jetson/README.md` records ERROR in one run), so a "green" suite on a
machine with no browser has not run them.

```bash
python -m playwright install chromium     # once, on your own machine
```

Check that it launches:

```bash
python -c "from playwright.sync_api import sync_playwright
with sync_playwright() as p: p.chromium.launch().close(); print('launch ok')"
```

**A trap in a cloud image (seen 2026-11-21).** The image preinstalls its own
Chromium (`PLAYWRIGHT_BROWSERS_PATH=/opt/pw-browsers`) and forbids `playwright
install`. `pip install -r requirements.txt` then chose Playwright 1.65.2,
which printed `Executable doesn't exist at
/opt/pw-browsers/chromium_headless_shell-1255/...` (the image holds build
1243). `pip install playwright==1.63.0` fixed it (`launch ok`). The session
hook does this choice for you; on your own machine `playwright install` is
the answer and none of this applies.

## 4. The perception models (optional)

```bash
pip install -r requirements-perception.txt   # YOLOE + CLIP, for policy "tiered"
```

Only the `tiered` policy needs them, and the automated suite never does (it
mocks every model call). Skip this step unless you will run `tiered` missions
(`docs/perception/ARCHITECTURE.md`). Not run for this page: a first
`tiered` start downloads weights.

## 5. Run the suite

```bash
pytest tests/ -q                      # no API key; every model call is mocked
pytest service/vision_analyze/tests/ -q   # the cloud vision service's own suite
```

* **Always name `tests/`.** Bare `pytest` also collects `evaluations/` and
  stops on two collection errors (checked 2026-11-21:
  `evaluations/rover-live-3140/test_rover_live.py`, `AttributeError: module
  'rows' has no attribute 'load_plan'`; and a duplicate `test_readers.py`
  basename under `evaluations/tiered-380/`). `pytest tests/` is the suite.
* **How many tests?** Ask the tree, not this page: `pytest tests/
  --collect-only -q | tail -1` (it printed `6560 tests collected` here).
* **It is long.** With `pytest-xdist` (in `requirements.txt`) add `-n 4`
  (or `-n auto`): the project's fast tier runs on half the cores. On a
  4-core cloud container a full run took well over half an hour (it was at
  9% after about 35 minutes with `-n 4`, and the run was stopped there; so the
  completed-suite claim is not made for this page). A UI test that fails
  alone and passes in the suite, or the reverse, is a real finding: run
  timing tests in their module's real order, not only with `-k`
  (`CLAUDE.md` section 6).
* A quicker first check: `pytest tests/test_robot_contract.py -q` (one
  conformance suite run against every robot backend).

## 6. The two servers and the twin

Two processes, on purpose (`CLAUDE.md` section 6: a brain loop inside the
robot server would block the event loop the watchdog runs on). Start each in
its own terminal, from the repo root, with the venv active:

```bash
uvicorn robot.server:app --port 8000      # the body: safety, who drives, watchdog
uvicorn control.brain_server:app --port 8001   # the brain: missions
```

By default the robot is the **simulator** (`mode: sim` in
`config/robot.yaml`), in the `starter_house` map. Check that both are up:

```bash
python -m control.health
```

It printed `OK`, then one block per server (`robot  ok  http://127.0.0.1:8000`,
`mode sim`, `watchdog_timeout_s 1.0` ...; `brain  ok
http://127.0.0.1:8001`) and exited 0. A `last_refusal` of `watchdog` on an
idle robot is normal: nothing was commanding it.

Then open `http://127.0.0.1:8000/` in a browser (the robot server serves the
twin; `/` answered 200). In **Settings**, connect both servers (robot
`http://127.0.0.1:8000`, brain `http://127.0.0.1:8001`). The Home tab starts a
mission; Drive has the D-pad, map and depth strip. Every feature:
[`FEATURES.md`](FEATURES.md).

**Traps.**

* **Restart the robot server to put the robot back at its start.** There is
  no reset endpoint, on purpose.
* **`APP_SHARED_SECRET` unset means no authentication.** Both servers check
  an `x-app-secret` header only when that environment variable is set
  (`robot/server.py` `require_secret`); `/health` and `/` are never checked.
  That is right for a laptop and wrong for any server a network can reach:
  set it before you expose a port (the tunnel scripts read it, see section 9).
* Another house: `SIM_MAP=scaled_house uvicorn robot.server:app --port 8000`
  (`home_first_floor`, `complex_house` and `procthor:val:<0-999>` also
  work; `docs/engineering/simulator/ENGINEERING.md`).

## 7. A first mission, by `curl`

With both servers up (no key, no cloud: the `frontier` policy is a free,
rule-based explorer):

```bash
curl -X POST localhost:8001/mission/start -H 'content-type: application/json' \
     -d '{"target_object": "red backpack", "policy": "frontier"}'
curl localhost:8001/mission/status        # poll: "outcome" goes running -> ...
curl -X POST localhost:8001/mission/stop  # stops the loop AND the car
```

Run here, on the default map: `/mission/start` returned `"started":true` with
`"policy":"frontier"` and `"outcome":"running"`; status polled every 5 s read
`running` three times and then **`found`** (61 steps, 19.7 s of mission time;
the log's last lines: `step 60: FORWARD (ok) -- hallway ... sees red
backpack`, `mission ended (found): Hallway, living room searched. Found.`).
`/mission/stop` after the end answered `"stopped":true` and left the outcome
`found`: the first outcome wins.

Outcome words (`found`, `exhausted`, `out_of_budget`, ...) are in the
glossary. A mission with no target maps the house instead:
`-d '{"job": "survey"}'`.

**Do not** start a mission with `"policy": "vision"` or `"tiered"` unless you
mean to: `vision` makes a paid cloud call every step, and `tiered` needs
step 4's models and calls the cloud on triggers.

## 8. The git hooks

For anyone who will commit:

```bash
git config core.hooksPath tools/hooks     # the merge gate (pre-push) and pre-commit
git config rerere.enabled true
git config rerere.autoupdate true
```

`git config core.hooksPath` then printed `tools/hooks`. The pre-push hook runs
`tools/plan_section.py gate` on a push to `dev` (a docs-only push runs the
spec lint; anything else runs the fast test tier). The rules for working in
this repo with other sessions, including claiming a plan step before
editing it, are `docs/guides/PARALLEL-SESSIONS.md` (read it before touching
the plan). Overriding the gate is the project owner's call alone.

## 9. Further than this page (pointers only)

| To | Read |
|---|---|
| ROS 2: SLAM and nav2, off by default (Docker; `ROBOT_DRIVE=ros WORLD_MODE=ros`) | `service/slam/README.md`, `docs/ros/ARCHITECTURE.md` |
| Reach the local stack from a phone anywhere (an ngrok domain; sets `APP_SHARED_SECRET`) | `service/tunnel/run.sh` and `restart.sh`, `docs/operations/ARCHITECTURE.md` |
| The Jetson: setup, systemd services, benches (one session on the board at a time) | `tools/jetson/README.md` |
| Run the motor-board code against a fake ESP32 | `ROBOT_MODE=hardware SIM_MOTOR_BOARD=fake WORLD_MODE=none bash service/tunnel/restart.sh` (root `README.md`, "Other entry points") |
| The real car: `ROBOT_MODE=hardware ROBOT_SERIAL=/dev/picar-board` | `docs/hardware/HARDWARE-READINESS.md`, `docs/hardware/ROVER-ARRIVAL-RUNBOOK.md`, `docs/motor-board/ARCHITECTURE.md` |
| Deploy the static twin (S3 + CloudFront; each deploy is the project owner's go) | `service/static/sync.sh`, `docs/operations/ARCHITECTURE.md` |
