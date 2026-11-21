# Specifications

Every component of the robot has two specifications
([decision 0001](decisions/0001-architecture-vs-engineering-specs.md)):

- **Architecture** (`docs/<domain>/ARCHITECTURE.md`) covers what the
  component is and why it is shaped that way: its boundaries, its
  decisions and the alternatives rejected, its contracts with neighbours,
  and its failure modes. It stays true if the implementation is rewritten.
- **Engineering** (`docs/engineering/<domain>/ENGINEERING.md`) covers how
  it is built today: modules, signatures, routes, config keys, thresholds,
  procedures and the tests that pin them. It is true only until the code
  changes, and changes in the same commit as the code.

The heuristic: if a competent engineer could build it more than one
reasonable way and the document does not care which, it is architecture.
Once it commits to one way, it is engineering.

`tools/spec_lint.py` checks the rules a script can check, and
`tests/test_spec_lint.py` runs it in the suite. [SPEC_REVIEW_PROMPT.md](SPEC_REVIEW_PROMPT.md)
covers the rules that need judgement; the `/spec-review` command
(`.claude/commands/spec-review.md`) runs it, for the domains you name or
all of them, and only reads. New specs start from
`templates/`.

The `PLAN-*.md` documents in [plans/](plans/) are the dated history: criteria
written first, then results, then corrections. Specs describe each
component as it is now and cite the plan sections that record how it got
there.

## Reading path

0. **[Architecture overview](ARCHITECTURE.md)** is the whole system on
   one page: the six processes (four on the Jetson, one in the cloud, and the phone;
   the local VLM is the one the rest can lose), the four walls, the ROS container, and
   one command traced end to end. Start there, then read the specs below.
1. **[Decision 0001](decisions/0001-architecture-vs-engineering-specs.md)**
   explains why there are two documents per component.
2. **The two interfaces everything else is built around.** Read
   [body](body/ARCHITECTURE.md) (what the robot says about itself) and
   [world](world/ARCHITECTURE.md) (what is true about the house, and where
   the robot is in it).
3. **[Safety](safety/ARCHITECTURE.md)** covers what may move the wheels,
   and who outranks whom.
4. **The two processes:**
   - [control-api](control-api/ARCHITECTURE.md) is the robot server.
   - [mission](mission/ARCHITECTURE.md) is the brain service and its
     failsafes.
5. **What decides each move:** [policy](policy/ARCHITECTURE.md),
   [perception](perception/ARCHITECTURE.md) and
   [cloud-vision](cloud-vision/ARCHITECTURE.md).
6. **Below the body:** [ros](ros/ARCHITECTURE.md) (the container behind
   the wall), [motor-board](motor-board/ARCHITECTURE.md) and
   [platform](platform/ARCHITECTURE.md) (the hardware).
7. **Around it:** [simulator](simulator/ARCHITECTURE.md),
   [twin](twin/ARCHITECTURE.md), [recordings](recordings/ARCHITECTURE.md)
   and [operations](operations/ARCHITECTURE.md).

## Domains

| Domain | Architecture | Engineering | Covers |
|---|---|---|---|
| body | [body/](body/ARCHITECTURE.md) | [engineering/body/](engineering/body/ENGINEERING.md) | `RobotInterface`, the body contract, and its backends |
| world | [world/](world/ARCHITECTURE.md) | [engineering/world/](engineering/world/ENGINEERING.md) | `WorldInterface`: the map, the pose, goals, and the body/world line |
| safety | [safety/](safety/ARCHITECTURE.md) | [engineering/safety/](engineering/safety/ENGINEERING.md) | The local safety layer, driver arbitration and the watchdog |
| control-api | [control-api/](control-api/ARCHITECTURE.md) | [engineering/control-api/](engineering/control-api/ENGINEERING.md) | The robot server: the HTTP API the brain, the twin and ROS talk to |
| mission | [mission/](mission/ARCHITECTURE.md) | [engineering/mission/](engineering/mission/ENGINEERING.md) | The brain service: mission lifecycle, the tick, failsafes |
| policy | [policy/](policy/ARCHITECTURE.md) | [engineering/policy/](engineering/policy/ENGINEERING.md) | What decides each move: frontier, vision, tiered, and arrival |
| perception | [perception/](perception/ARCHITECTURE.md) | [engineering/perception/](engineering/perception/ENGINEERING.md) | The on-board perception tier: detector, encoder and gate, and the local VLM service asked before the cloud |
| cloud-vision | [cloud-vision/](cloud-vision/ARCHITECTURE.md) | [engineering/cloud-vision/](engineering/cloud-vision/ENGINEERING.md) | The vision service on Bedrock: `/navigate` and its siblings |
| ros | [ros/](ros/ARCHITECTURE.md) | [engineering/ros/](engineering/ros/ENGINEERING.md) | The ROS 2 container behind the HTTP wall |
| motor-board | [motor-board/](motor-board/ARCHITECTURE.md) | [engineering/motor-board/](engineering/motor-board/ENGINEERING.md) | The ESP32 driver board, its firmware fork, and the serial backend |
| platform | [platform/](platform/ARCHITECTURE.md) | [engineering/platform/](engineering/platform/ENGINEERING.md) | The hardware: compute board, chassis, sensors, power |
| simulator | [simulator/](simulator/ARCHITECTURE.md) | [engineering/simulator/](engineering/simulator/ENGINEERING.md) | The grid-world sim, its renderer, maps and movers |
| twin | [twin/](twin/ARCHITECTURE.md) | [engineering/twin/](engineering/twin/ENGINEERING.md) | The web twin: the phone UI over both servers |
| recordings | [recordings/](recordings/ARCHITECTURE.md) | [engineering/recordings/](engineering/recordings/ENGINEERING.md) | Recorded walks, their storage, and the evaluation instruments |
| operations | [operations/](operations/ARCHITECTURE.md) | [engineering/operations/](engineering/operations/ENGINEERING.md) | Deployment, the tunnel, secrets, health and metrics |

## Beyond the specs

Everything else that used to sit at the repo root, grouped by what it is.
None of these are specs, and the linter skips them. They are cited by file
name, and every name is unique, so a search for a name still finds it.

| Folder | Holds |
|---|---|
| [plans/](plans/) | `PLAN-*.md` (`PLAN-ar-guidance.md`, `PLAN-aws-cost-redesign.md`, `PLAN-brain-relocation.md`, `PLAN-guarded-verbs.md`, `PLAN-mapping.md`, `PLAN-microduck-transplants.md`, `PLAN-onboard-perception.md`, `PLAN-sim-hardening.md`, `PLAN-teleop-robot.md`): each phase's criteria, results and corrections, dated. `PLAN-ros-alignment.md` is the governing plan; its 3.N sections are one file each in `plans/ros-alignment/` |
| [hardware/](hardware/) | What to buy and why: `JETSON-BOM.md` (the build), `HARDWARE-BOM.md` (parts, wiring, the board protocol), `HARDWARE-READINESS.md`, `BOM-COMPARISON.md` (verified prices), `PI-VS-JETSON.md`, `GUIDE-robot-base.md`, `UGV-ROVER-MOUNTING.md` (the Rover's deck hole pattern, from Waveshare's CAD), `ROVER-ARRIVAL-RUNBOOK.md` (the hands-on steps for the Rover's arrival, by day), and the superseded `BOM.md` |
| [guides/](guides/) | Explainers: `INTRODUCTION.md`, `FEATURES.md` (every UI feature end to end), `AGENT-HARNESS.md` (how `control/` works), `PARALLEL-SESSIONS.md` (several sessions on the plan at once: one branch per section), `MERGED-STATE.md` (what each merged plan section made true), `SETUP.md` (a fresh machine to a working checkout), `GLOSSARY.md` (the terms the specs and plans use), `TROUBLESHOOTING.md` (an index by symptom), `PRIVACY-AND-SECURITY.md` (what the robot sends, stores and exposes), and `FEATURES.html`, a hand-made copy of `FEATURES.md` that has not followed it (its own top note says so; read `FEATURES.md`) |
| [evaluations/](evaluations/) | Write-ups of one-off measurements: `EDGE-PERCEPTION-BENCH.md`, `EVAL-navigate-models-2026-09-22.md`, `PROCTHOR-HOUSES-2026-10-11.md` (the simulator choice with the ProcTHOR import), `LEARNING-APPROACHES-2026-10-22.md` (learning approaches for the tiered policy), `INVENTORY-NAMERS-2026-10-10.md` (Qwen3.5-4B against Gemma 4 E2B / E4B as inventory namers) and `LOCAL-VLM-SHORTLIST-2026-10-19.md` (local VLMs to try on the Rover); the folders `ux-3201/` (twin screenshots, 3.201), `local-vlm-3152/` (twin screenshots, 3.152) and `board-bench/` (the raw results behind `BOARD-BENCH.md`, by date); [BOARD-BENCH.md](evaluations/BOARD-BENCH.md), [BOARD-BENCH.md](evaluations/BOARD-BENCH.md), the running log of every recurring Jetson test (whole-stack benches, steady-state windows, memory and chaos soaks, fault tests), kept to track trends |
| [media/](media/) | Images the root README embeds (`twin-found.png`) |
| [templates/](templates/) | The starting files for a new architecture or engineering spec |
| [handoffs/](handoffs/) | Session handoffs, dated; the newest says what is open |
| [archive/](archive/) | Older material moved out of `CLAUDE.md`, verbatim: `CLAUDE-history-2026-09.md` (the dated reversals) `CLAUDE-2026-10-06.md` (the whole file before its 2026-10-06 rewrite), `CLAUDE-2026-11-04.md` (the whole file before 3.149 cut section 3b to status lines), and the 2026-10-06 copies of `JETSON-BOM.md` and the root README's build journal |

## Decisions

| # | Decision |
|---|---|
| [0001](decisions/0001-architecture-vs-engineering-specs.md) | Architecture specs and engineering specs are separate documents |
| [decisions/README.md](decisions/README.md) | The index of the project's dated decisions, one row each, pointing at the file that records it; the index makes no decision |

## Outside `docs/`

| Where | Holds |
|---|---|
| [`evaluations/`](../evaluations/README.md) | The top-level `evaluations/` holds harnesses and recorded results, one directory per plan section; `docs/evaluations/` holds the write-ups. `evaluations/README.md` indexes the scored perception records |
| [`docs-review/`](../docs-review/) | Review reports: `SPEC-REVIEW.md`, `SPEC-REVIEW-2.md` and `SPEC-REVIEW-3.md` (the three spec reviews, the third with one file per domain group in `spec-review-3/`), `REPORT.md`, `REPORT-robot-base.md` and `REPORT-ros2-for-vision-picar.md` (documentation reviews, 2026-09), and `THERMOS-BUG-PASS.md`, the bug pass's rubric (CLAUDE.md section 6, "Code review") |
| READMEs a reader needs | [service/slam/](../service/slam/README.md) (the ROS 2 container and how to run it), [service/local_vlm/](../service/local_vlm/README.md) (the local VLM service), [service/tunnel/](../service/tunnel/README.md) (the local robot and brain, reached from the deployed twin), [tools/jetson/](../tools/jetson/README.md) (Jetson setup and benches), [tools/rover/](../tools/rover/README.md) (Rover bring-up), [tools/hooks/](../tools/hooks/README.md) (the git hooks every checkout uses) and [sim/maps/](../sim/maps/README.md) (the houses the simulator loads) |
