# vision-picar

**A robot car that finds things by looking.**

Say "find my red backpack". The robot drives off, searches the house room
by room, recognises the backpack when it sees it, and stops in front of it.
Nobody gives it a floor plan: it maps the house as it drives, decides for
itself where to look next, and confirms what it found before it says so.

It runs on an NVIDIA Jetson Orin Nano Super riding a Waveshare UGV Rover
(D500 lidar, OAK-D Lite depth camera, ESP32 motor board). Perception,
identity, mapping and planning all run on board; a cloud VLM is the last
tier, and the robot still works with the internet unplugged.

---

## Why this is hard

Finding a named object in an unknown home stacks four hard problems:

* **Open-vocabulary recognition on an edge budget.** The target is free
  text, not a trained class, and homes are full of near-misses: a teal bin
  pulls large cloud VLMs to 68% precision on our identity set. The models
  that discriminate well are multi-billion-parameter VLMs, and the Jetson
  has 8 GB of unified memory shared with detection, SLAM, nav2 and the
  safety loop.
* **Search with no prior map.** With the target out of view, no VLM we
  measured, local or cloud, predicts its direction better than a base-rate
  guess. Where to look has to come from the occupancy map the robot is
  still building.
* **Hard real time beside heavy inference.** The wheel loop runs at 20 Hz
  on a Python event loop that also serves maps and telemetry; one
  synchronous JSON encode is enough to make ticks late.
* **Safety independent of the planner.** nav2, the brain and a person on
  the D-pad all issue motion. None of them may be trusted to be right about
  clearance.

## How it works

### Perception: three tiers, cheapest first

1. **Every frame, on board.** YOLOE-11s-seg proposes open-vocabulary
   crops; CLIP RN50 scores each against the target text and gates at
   P >= 0.8. Median 61 ms, p90 110 ms a frame on the Jetson at 15 W.
2. **Identity, on board.** A candidate goes to Qwen2.5-VL-3B, INT4 AWQ with
   a vocabulary trimmed to 17,920 tokens and FP8 embeddings, built and run
   with TensorRT Edge-LLM behind a local HTTP service. 97.2% precision on
   the 299-frame identity set (the cloud model: 67.9%); 1.32 s p90 with the
   whole stack running; arrival-to-verdict p90 1.91 s.
3. **Escalation to the cloud.** Claude on Amazon Bedrock is asked only
   after repeated "unable" answers from the local tier, or when it is down.
   A lower tier never overrides a higher one's answer.

Across 21 simulated houses the ladder took cloud calls from 765 to 6.
Arrival is geometric as well as visual: the target centred and the lidar
under 0.40 m for two frames before identity is asked. With the cloud
unreachable, an arrival the local tier cannot settle ends
`arrived_unconfirmed`, never `found`.

### Search: SLAM, nav2 and a learned frontier ranker

The D500 lidar feeds **slam_toolbox** (tuned scan-matcher smear: two tours
agree within 4.2 cm in the furnished test house), and **nav2** plans and
follows routes, with `collision_monitor` and `twist_mux` in front of
`ros2_control`'s diff-drive controller. All of ROS 2 lives in one container;
nothing outside it imports `rclpy`, and a capped HTTP bridge is the only
door. The pose graph is saved per mission and reloaded in localisation
mode on the next start.

**Where to look next is learned.** Candidates are the map's frontiers and
unseen patches that the chassis can actually leave; a small network ranks
them on room structure (unentered rooms, doorways, a per-object room
prior), travel distance and the robot's own trail. It is behaviour-cloned from an
oracle with the target's true position, then fine-tuned with **PPO** over
ProcTHOR-10K floor plans. On 588 held-out houses: 14-22% fewer goals than
the hand-written rule, finds within 1%. The full choice is 95 ms p90 on the
Jetson under load, after the grid work was vectorised in numpy with
bit-identical answers.

### Decisions: behavior trees

The tiered policy's three decisions -- what to do this frame, when to ask,
whom to ask -- are **behavior trees** on py_trees (0.85 ms p90 a frame on
the Jetson). Rung order and node parameters are configuration, validated
at load. Invariants stay outside the trees: safety, a person's Stop, the
arrival rule, and who may declare `found`.

### Safety: enforced in one place, on every path

* **Every motion passes `robot/safety.py`** in the robot server: a
  depth-grid cone chosen by body bearing, the chassis' swept corridor off
  the 360-degree scan, a pivot guard, a floor band for objects below the
  lidar, no motion without fresh wheel feedback, and stall detection.
  nav2's `collision_monitor` sits in series before it; the robot server has
  the last word.
* **Arbitration by rank:** `stop > person > brain`, with nav2 goals ranked
  as a driver. A person's Stop latches autonomy out and cancels nav2 goals
  until a new mission starts.
* **Three failsafes for three failures:** the robot watchdog (no command,
  motors stop), the mission runner (stuck vision calls, failure budget),
  the brain server (a tick that never returns).
* **Memory pressure kills the VLM first** (`OOMScoreAdjust=1000`), never
  the robot stack.

### The pieces

```
 phone (digital twin) ──HTTP──┐
                               ▼
 brain  ──────HTTP──────▶  robot server  ──────▶  body: the simulator,
 missions, perception,     safety, who is          or the Rover's ESP32
 behavior trees,           driving, watchdog       motor board and lidar
 local VLM, cloud calls        ▲   │
                               │   ▼ HTTP
                       ROS 2 container:
                       SLAM (slam_toolbox), routes (nav2),
                       velocity arbitration, wheel kinematics
```

* **Brain:** runs the mission and picks each move. It reaches the robot only
  over HTTP, so it runs on a laptop or the Jetson unchanged.
* **Robot server:** safety, arbitration of who is driving, the watchdog.
* **ROS 2:** mapping and route planning from standard packages.
* **Body:** anything that implements one interface, the simulator or the
  real Rover. Switching is a configuration change.
* **Phone app (digital twin):** a D-pad, a live map and mission control. It
  draws what the servers report and decides nothing.

[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) is the whole system on one
page.

## How it is built

* **Simulation first.** The software cannot tell the simulator from the
  car: continuous differential-drive kinematics, sensor noise, moving
  people and pets, and the Rover's ESP32 firmware and LD19 lidar protocol
  emulated on ptys. Four hand-built houses plus 1,000 ProcTHOR floor plans.
* **Data decides.** Every feature has a pass/fail number written down
  *before* it is measured, measured through the real path, and pinned in a
  test. A failed number stays recorded as a failure.
* **Spec-driven.** Every component has an architecture spec (what and why)
  and an engineering spec (how) that changes in the same commit as the
  code.
* **Built with Claude Code** throughout: about 155,000 lines of Python, a
  thin layer of C++, and a phone UI in plain JavaScript.

## Status

The Jetson is in hand and the whole stack runs on it under full load:
perception, the local vision-language model, SLAM, nav2 and the safety loop,
with no late safety tick. The Rover chassis is arriving; nothing has driven
real wheels yet. Every number above is measured in simulation or on the
Jetson, not yet in a real home. `CLAUDE.md` section 3 has the detailed
state and the open work.

---

## Quick start

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -m playwright install chromium          # so the browser UI tests run, not skip
pip install -r requirements-perception.txt     # only for policy "tiered" (YOLOE + CLIP)

pytest tests/ -q                               # no API key needed; every model call is mocked
pytest service/vision_analyze/tests/ -q        # the cloud vision service's own suite
```

**Run the robot and the brain, and drive from the twin:**

```bash
uvicorn robot.server:app --port 8000
uvicorn control.brain_server:app --port 8001
# open http://127.0.0.1:8000/  -> Settings: connect both -> Sim tab
```

Restart the robot server to put the robot back at its start position.

**Start a mission without the page:**

```bash
curl -X POST localhost:8001/mission/start -H 'content-type: application/json' \
     -d '{"target_object": "red backpack"}'
curl localhost:8001/mission/status
curl -X POST localhost:8001/mission/stop        # stops the loop AND the car
curl -X POST localhost:8001/mission/start -H 'content-type: application/json' \
     -d '{"job": "survey"}'                     # no target: map the house, name its rooms
```

**Other entry points:**

| To | Run / read |
|---|---|
| Turn on SLAM and nav2 | `service/slam/README.md` (Docker; `ROBOT_DRIVE=ros WORLD_MODE=ros`) |
| Drive the car's motor code against a fake board | `ROBOT_MODE=hardware SIM_MOTOR_BOARD=fake WORLD_MODE=none bash service/tunnel/restart.sh` (`WORLD_MODE=ros` with the container) |
| Use another house | `SIM_MAP=scaled_house`, `home_first_floor`, `complex_house` or `procthor:val:<0-999>` |
| Add people and pets that move | `SIM_MOVERS=<scenario>` |
| Replay a recorded phone walk through a policy | `python -m tests.demo_replay_mission recordings/<walk> "<target>" --policy tiered` |
| Check both servers' health | `python -m control.health` |
| Reach the local stack from a phone anywhere | `service/tunnel/` (an ngrok domain split by path) |
| Set up the Jetson, run the stack there as systemd services, benchmark it | `tools/jetson/README.md` |

## What is in the repo

| Path | What |
|---|---|
| `robot/` | The robot server, `RobotInterface`, the safety layer, the motor-board and lidar drivers |
| `world/` | `WorldInterface`: pose and map, from the simulator or from SLAM |
| `brain/` | Mission policies, on-board perception, the behavior trees, the goal chooser, the arrival rule |
| `control/` | The brain as a service, failsafes, health, recorded-walk tools |
| `sim/` | The simulator: houses, physics, renderer, movers, the fake motor board and lidar |
| `service/` | The ROS 2 container, the local VLM service, the cloud vision service, deployment and tunnel scripts |
| `firmware/` | Our patch to the Rover's ESP32 firmware (GPL-3.0) |
| `web-twin/` | The phone UI |
| `tools/` | The Jetson's setup and benches, the spec linter, the parallel-session and lock tools |
| `evaluations/` | Measurement harnesses and their recorded results, by plan section |
| `docs/` | Specs (architecture and engineering per component), plans, guides, hardware, evaluations |

## Reading list

| Doc | For |
|---|---|
| [`docs/guides/INTRODUCTION.md`](docs/guides/INTRODUCTION.md) | A non-technical introduction |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | The system on one page |
| `CLAUDE.md` | Orientation for working in the code: the rules, the current state, the gotchas |
| [`docs/README.md`](docs/README.md) | The index: a spec pair for each component, and the reading path |
| [`docs/plans/PLAN-ros-alignment.md`](docs/plans/PLAN-ros-alignment.md) | The governing plan: every phase, criteria written first, results recorded |
| [`docs/guides/FEATURES.md`](docs/guides/FEATURES.md) | Every feature of the twin, end to end |
| [`docs/guides/AGENT-HARNESS.md`](docs/guides/AGENT-HARNESS.md) | How the brain service works: the tick, seams, failsafes |
| [`docs/hardware/JETSON-BOM.md`](docs/hardware/JETSON-BOM.md) | What was bought, and what is left before the Rover drives |

The build journal this file used to carry is in
`docs/archive/README-build-journal-2026-10-06.md`, verbatim.
