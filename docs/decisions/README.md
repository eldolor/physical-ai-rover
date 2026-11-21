---
kind: decision
domain: decisions
status: current
verified: 2026-11-21
---

# The decision index

An index of the project's dated decisions, one row each, pointing at the
file that records it. It is not a decision record and makes no decision.
The one full record is
[0001](0001-architecture-vs-engineering-specs.md).

## Context

Most of this project's decisions were never written as records. They live
as dated asides in the working files: "decided 2026-10-02", "the user,
2026-10-15". Plan section 3.207 (finding 7 of
`docs/plans/ros-alignment/3.207-docs-gaps.md`) asked for one place that
lists them.

**What is indexed.** Every dated decision cited by one of four sources, as
they read on `dev` `7c0bedf` (2026-11-21):

* `CLAUDE.md`;
* `docs/plans/PLAN-ros-alignment.md` sections 0, 1 and 6 (section 6 is the
  list the task named; 0 and 1 hold the decisions `CLAUDE.md` cites from it);
* `docs/guides/PARALLEL-SESSIONS.md`;
* `.claude/skills/orchestrate/SKILL.md`.

The older reversals come from `docs/archive/CLAUDE-history-2026-09.md`.

**How to read a row.**

* **Decision** restates the source, or quotes it where the wording matters.
  Nothing is added and nothing is tidied.
* **Date** is the date the source gives. "not dated" means the source
  states the decision and gives no date for it.
* **Who** is "the user" only where the source says so. "not stated" means
  the source names nobody; it does not mean a session decided it.
* **State** is `stands`, `reversed` (with the date and the row that
  reversed it), `open` (written down, waiting on a trigger) or `closed`.
* **Recorded in** is the file and section to read. **That file governs,
  not this row.**

**What is not indexed.** Undated standing rules (for example `CLAUDE.md`
section 3c's "`tier_hold_bearing` stays OFF"), decisions recorded only in
a section file under `docs/plans/ros-alignment/` or in a spec and cited by
none of the four sources, and measured results. Two entries of the plan's
section 6 are questions "for the user to decide" and so have no row: 10
(gyro-based heading) and 11 (Isaac ROS 5.0). Each has a written trigger.

## Decisions, by area

### Hardware

| Decision | Date | Who | State | Recorded in |
|---|---|---|---|---|
| "no Jetson, on cost" | 2026-09-13 | not stated | reversed 2026-09-17 (next row) | `docs/archive/CLAUDE-history-2026-09.md`, "Superseded, kept for the reasoning -- REVERSED 2026-09-17 to the JETSON" |
| Reversed to the Jetson: "the Jetson is now the defensible buy" (P19: OWLv2 as a crop source reads 83% against the tier's 50%; the price gap is $59-86) | 2026-09-17 | not stated | stands, closed by the row for 2026-09-19 | same file, "REVERSED 2026-09-17 by P19" |
| The Hailo-10H is "DROPPED 2026-09-17 on verified pricing" | 2026-09-17 | not stated | stands | same file, "The Hailo-10H -- DROPPED 2026-09-17" |
| "HARDWARE DECISION -- CLOSED 2026-09-19: the board is a JETSON, and the Hailo path is not being pursued." In `CLAUDE.md`: a Jetson Orin Nano Super; "The Hailo / Pi path is closed; do not reopen it or spend on Hailo runs." | 2026-09-19 | the user | stands | `CLAUDE.md` section 3c; the archive file above, "HARDWARE DECISION" |
| The Jetson runs JetPack 6.2.1 on the NVMe at 15 W, "kept 2026-10-05" | 2026-10-05 | not stated | stands | `CLAUDE.md` section 3a, the Jetson row |
| The host never sends the motor board's `T:0` (M2.8, "settled against the firmware source 2026-10-18") | 2026-10-18 | not stated | stands | `CLAUDE.md` section 3c |
| Leaving Humble before its end of life: "the user agreed to plan it, not to start it"; not during hardware bring-up; target Q1 2027 | 2026-10-06 | the user | open | `docs/plans/PLAN-ros-alignment.md` section 6, question 12 |

### ROS scope

| Decision | Date | Who | State | Recorded in |
|---|---|---|---|---|
| "THE NEXT PHASE, decided 2026-09-19: map the house while it searches it": ROS 2 enters as exactly one containerised service exposing `GET /pose`, `GET /map`, `POST /goto` and nothing else; "Never (c)'s full adoption" | 2026-09-19 | not stated | reversed 2026-09-25 (next two rows) | `docs/archive/CLAUDE-history-2026-09.md`, "THE NEXT PHASE" |
| "ROS 2 is adopted properly, not minimally": `nav2`, `slam_toolbox`, `ros2_control`, `tf2` and `twist_mux` all run | 2026-09-25 | the user | stands (corrected 2026-10-01: `robot_localization` was listed as running and is not in the container) | `docs/plans/PLAN-ros-alignment.md` section 0, item 1 |
| "`POST /goto` as specified is withdrawn": the wall moves down to `hardware_interface`, and up to guard `brain/` | 2026-09-25 | the user | stands | same file, section 0, item 2 |
| "No Gazebo." | not dated (the plan is dated 2026-09-25) | not stated | stands | same file, section 1 |
| "hardware day is a BACKEND swap, not a plugin swap": `picar_sim_hardware` stays on the car too; the seam is `RobotInterface` | 2026-09-26 (R7, 3.16) | not stated | stands (it revised the plan's first "Hardware day is a plugin swap") | same file, section 1, "Revised at R7" |
| "The rule: plumbing in, judgment and safety out." Outside ROS: the brain, `robot/safety.py` and the watchdog, the phone | discussed 2026-10-01, decided 2026-10-02 | the user | stands | same file, section 1.1; `CLAUDE.md` section 2, rule 3 |
| "a sensor that feeds a veto stays outside, even though a driver is plumbing" (the motor board's serial port, the lidar driver, the OAK-D's floor depth, M4's arbitration, the direct-mode verb executor) | 2026-10-02 | the user | stands | same two places |
| Perception goes inside ROS "later, on a measured trigger only"; the brain keeps the judgment | 2026-10-02 | the user | open (the trigger has not fired) | `docs/plans/PLAN-ros-alignment.md` section 1.1 |
| Question 1, renaming the ROS service directory to "nav": "Dropped 2026-10-01" | 2026-10-01 | "closed by Claude, delegated by the user" | closed | same file, section 6, question 1 |
| Question 2: 3.15 decided "the tier keeps steering by verbs for now" | not dated (noted 2026-10-01) | not stated | reopened by question 6's frontier search; "Taken up as 3.31's `explore` policy", which sends nav2 goals | same file, section 6, question 2 |
| Question 5: the lidar driver runs in the robot server, and the bridge republishes the scan ("DECIDED by the user 2026-10-02: the second") | 2026-10-02 | the user | stands | same file, section 6, question 5 |
| Question 6: the map is backed up to AWS S3, "a private, encrypted, versioned prefix"; "privacy, not cost (under 1 MB), drives the design" | not dated in section 6 | the user | stands as a decision; see `docs/guides/PRIVACY-AND-SECURITY.md` for what is built | same file, section 6, question 6 |
| Question 6: "A saved map is a starting guess, not the truth": reload the pose graph and keep mapping on top of it; "Not lifelong mode" | 2026-10-01 | the user ("let's go with your recommendation") | stands | same file, section 6, question 6 |
| Question 6: "An unreachable frontier is retried, not dropped" | 2026-10-01 | agreed by the user | stands | same file, section 6, question 6 |
| Question 7: things that move in the simulator, "first in order" | 2026-10-01 | agreed by the user | stands; "BUILT as 3.30" | same file, section 6, question 7 |

### Safety

| Decision | Date | Who | State | Recorded in |
|---|---|---|---|---|
| Question 3: "the two collars run in SERIES", nav2's `collision_monitor` before `robot/safety.py`, "with `safety.py` the last word on every path" | not dated (decided in 3.15) | not stated | closed | `docs/plans/PLAN-ros-alignment.md` section 6, question 3; `CLAUDE.md` section 2, rule 4 |
| Question 9: speed set by clearance, "after 7, and after the speed-dependent stop"; the robot server's stop must become speed-dependent before any speed is raised | 2026-10-01 | agreed by the user | open: the speed-dependent stop is built (3.44, merged 2026-10-07); criteria (a), (c) and (d) are open | same file, section 6, question 9 |
| 3.42 / 3.76 A4: "fix `/scan`'s stamp (the request time) before raising nav2's speed" | 2026-11-07 | the user | open, waiting on its trigger | `CLAUDE.md` section 3b, "Decided, each waiting on its trigger"; the plan's section 6, question 9, last bullet |
| Question 13: the safety loop in another language (Rust): "not now"; the trigger is more than 0 late wheel ticks per 10,000 traced to the interpreter or the GIL, "decided on the Rover, not before" | 2026-10-30 | raised by the user; "the trigger agreed the same day" | open, waiting on its trigger | `docs/plans/PLAN-ros-alignment.md` section 6, question 13 |

### Cloud

| Decision | Date | Who | State | Recorded in |
|---|---|---|---|---|
| The ECS/NLB/ALB stacks "were deleted 2026-09-05" | 2026-09-05 | not stated | stands | `CLAUDE.md` section 6, "Cloud and deployment", History; `docs/plans/PLAN-aws-cost-redesign.md` |
| 3.56: "re-check the 8 s arrival deadline after 500+ real calls" | 2026-11-07 | the user | open, waiting on its trigger | `CLAUDE.md` section 3b, "Decided, each waiting on its trigger" |
| "Cloud sessions can deploy the twin only"; "each deploy needs the user's go" | not dated | not stated | stands for every session but the orchestrator (see the 2026-11-21 deploy row, Working rules) | `CLAUDE.md` section 3b, "Waiting on the user" |
| The board's brain: "update it only when asked" | not dated | not stated | stands | same place |
| 3.190: "one cloud call at a find's start for the room prior of a target the shipped prior lacks"; wired, off by default | 2026-11-19 | the user | open (F FAILED: nothing ships; B's call 7 and C's Lambda deploy wait on the user) | `CLAUDE.md` section 3b, "Open now" |
| 3.206: "a frontier model's direction labels train Qwen's steering" | 2026-11-21 | the user | open (B-C's calls wait on the user) | same place |
| 3.175-3.178: "features from similar products, bars first" | 2026-11-12 | the user | open | same place |

### Working rules

| Decision | Date | Who | State | Recorded in |
|---|---|---|---|---|
| Every component has an ARCHITECTURE spec and an ENGINEERING spec | 2026-10-02 | the user | stands | [0001](0001-architecture-vs-engineering-specs.md); `CLAUDE.md` section 2, rule 6 |
| Several sessions work on the plan at once; "Read `docs/guides/PARALLEL-SESSIONS.md` before working on the plan" | 2026-10-07 | the user | stands | `CLAUDE.md` section 3c; `docs/guides/PARALLEL-SESSIONS.md`, opening |
| "Run `/code-review` on every code change"; on a risky change, also the Thermos bug pass | 2026-10-09 | the user | stands | `CLAUDE.md` section 6, "Code review" |
| An unrecorded review warns at the push and never blocks: "warn first, block only if warnings are pushed past" | 2026-10-09 | the user | stands | same place, "Record it" |
| The merge gate is "fast on push, full at night"; the full suite on every push was "too heavy". "Only the user overrides the gate (`PICAR_SKIP_GATE=1`)." | 2026-10-09 | the user | stands (it replaced a gate that ran the full suite on every push) | `docs/guides/PARALLEL-SESSIONS.md` rule 6 |
| Claim a step before working on it (3.75) | 2026-10-12 (the day its Why records) | not stated | stands, extended by rule 12 | same file, rule 8 |
| "Each session pushes to `dev` on its own judgement"; "Spending, deploys, force pushes and `PICAR_SKIP_GATE=1` are still asked." | 2026-10-15 | the user | stands; a deploy is still asked, except the orchestrator's twin deploy (the 2026-11-21 row below) | same file, rule 10; `CLAUDE.md` section 3c |
| "Delete a branch once its work is in `dev`"; a branch with work not in `dev` only on the user's word | 2026-10-15 | the user | stands | same file, rule 11; `CLAUDE.md` section 3c |
| "Reviewers run on Sonnet" | 2026-10-18 | the user | stands | `CLAUDE.md` section 6, "Code review" |
| The desktop lock: "timed runs alone, other heavy work together" (3.128) | 2026-10-31 | the user | stands | `docs/guides/PARALLEL-SESSIONS.md` rule 7; `CLAUDE.md` section 3c |
| "One session on the Jetson at a time (3.93)" | not dated | not stated | stands | `CLAUDE.md` section 3c; `docs/guides/PARALLEL-SESSIONS.md` rule 7 |
| "Claim first, on origin; if refused, stop and report" (3.139). "`PICAR_CLAIM_BYPASS=1` is the user's alone"; a session never runs `release --force` without the user's word | 2026-11-01 | the user | stands | `docs/guides/PARALLEL-SESSIONS.md` rule 12; `CLAUDE.md` section 3c |
| "A detached run's end is checked, not awaited" (3.174.B) | 2026-11-11 | not stated | stands | `docs/guides/PARALLEL-SESSIONS.md` rule 13 |
| `CLAUDE.md` has a size budget and a weekly trim; merged sections' lines moved to `docs/guides/MERGED-STATE.md` | 2026-11-12 | not stated | stands | `CLAUDE.md` section 6, "CLAUDE.md's size", and section 3b |
| The plan's open-step count is "Updated at merge by whoever changes the counts" | 2026-11-15 | the user | stands | `docs/plans/PLAN-ros-alignment.md`, the "Open steps" line; `.claude/skills/orchestrate/SKILL.md` section 4 |
| The orchestrator runs "at most 4 workers at once", on a daily budget (default "6 worker runs a day") | not dated | not stated | reversed 2026-11-21: no fixed worker cap (the worker-count row below) | `.claude/skills/orchestrate/SKILL.md`, opening and section 2 |
| "At most two worker sessions at once by default"; "More only on the user's word and a fresh `/usage` reading." | 2026-11-15 | not stated | reversed 2026-11-21 (the worker-count row below) | same file, section 2 |
| "every new step starts in a FRESH session" | 2026-11-16 | the user | stands | same file, section 3 |
| "The orchestrator's own size: check it, and hand over early" (above 300K tokens hand over; no new batch above 400K) | 2026-11-16 | the user | stands | same file, section 2 |
| "Standing decision: the orchestrator decides": what it recommends counts as approved, recorded as "the orchestrator, under the user's delegation (2026-11-21)"; "Ask the user only before spending money"; still never a force push, `PICAR_CLAIM_BYPASS`, `PICAR_SKIP_GATE`, `release --force`, or a claim taken from another session | 2026-11-21 | the user | stands | same file, "Standing decision" |
| The orchestrator may deploy the twin without asking, after a web-twin change has landed on `dev`; every other session asks the user before any deploy; Lambda and CloudFormation deploys unchanged; spending still asked by everyone | 2026-11-21 | the user | stands | `docs/plans/ros-alignment/3.207-docs-gaps.md` ("The user's answers"), answer 4; `.claude/skills/orchestrate/SKILL.md`, "Standing decision"; `docs/guides/PARALLEL-SESSIONS.md` rule 10 |
| The orchestrate skill states no fixed worker cap: the orchestrator chooses the number of workers and the pacing; the 15% weekly reserve and the runaway-worker rule stay | 2026-11-21 | the user | stands | `docs/plans/ros-alignment/3.207-docs-gaps.md` ("The user's answers"), answer 7; `.claude/skills/orchestrate/SKILL.md`, opening and section 2 |

### Definition of done

| Decision | Date | Who | State | Recorded in |
|---|---|---|---|---|
| "done when someone holding a phone can watch it" | 2026-09-25 ("stated non-negotiable that morning") | not stated (the user replaced it) | reversed 2026-09-25, the same day (next row) | `docs/plans/PLAN-ros-alignment.md` section 0, "The definition of done -- CHANGED 2026-09-25" |
| "remove that rule, use logs and data to make a data-driven decision": a phase is done when its acceptance data meets thresholds written down before the run, measured through the real mission path and pinned in a test | 2026-09-25 | the user | stands | `CLAUDE.md` section 7; the plan's section 0 |
| What stays from the retired rule: the twin, the UI tests, a phone-size screenshot as evidence for any change to the page, drills | 2026-09-25 | not stated | stands | `CLAUDE.md` section 7 |
| "The 39-walk standing-height corpus was deleted 2026-09-07 as invalid." | 2026-09-07 | not stated | stands | `CLAUDE.md` section 5, Stage 0 |

## Where the sources disagree

None listed as of 2026-11-21. The two that were here (who approves a twin
deploy; how many workers) were decided by the user that day and are rows
above.

## Alternatives rejected

This index rejects nothing about the robot. Each row's own source holds
the alternatives that decision rejected. About the index itself, plan
section 3.207's brief ruled out one thing: writing a new numbered record
for every row. The rows point at their sources instead, and a decision is
not restated in a second place that could drift from the first.

## Consequences

* The source file governs. A row that disagrees with its source is wrong.
* The index is true as of the `dev` commit named under Context. A decision
  made after it is not here until someone adds its row; this file sets no
  rule about who does.
* A decision significant enough for a full record still gets one, as
  record 0001's "Consequences" says.
