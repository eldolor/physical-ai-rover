# Handoff 2026-11-23: the orchestrator, 2026-11-21 13:46 to 2026-11-23 15:40 UTC

*Written by orchestrator session session_01JbXvZtTf4j1NzXYWvwRdSS at 523K
tokens (past the skill's 300K handover line). Read the orchestrate skill
first (Standing decision block; at most 2 workers, the user 2026-11-22), and
CLAUDE.md's top rule: questions for the user one at a time, each its own
pop-up with a recommendation (the user, 2026-11-23).*

## State

* **Nothing is running.** Every worker session of this orchestrator is
  archived. No claim is held by this session.
* **No free plan step is cloud-doable** (2026-11-23): the free ones need the
  Rover, the Jetson, a paid GPU host, or are parked by the user (3.178.B,
  3.187.E). 3.206 / 3.214 are held by another session on another host.

## Landed since HANDOFF-2026-11-21b

3.205.A 0157d902 (N8 invalid by load) · 3.198 3d3e344f · 3.161 closed
(keeper ae065dcd) · 3.211 1056d348 (NavFn Dijkstra; record corrected:
the 15 survey failures were one goal inside a dresser) · 3.202 partial
cbc88467 (full soak 3.202.C waits on the desktop's exclusive lock) · 3.212
b10ac0b6 (live runs keep logs, `PICAR_KEEP_LOGS`) · 3.210 99de4c5b (E2/E4
accepted FAILED) · 3.208 9409ae38 (robot-link budget; 3.178.B's Touches
narrowed to its own files, the user's approval) · 3.213 61e97a8b (C1 ON:
false sightings no longer replace a better-supported approach; 3.213.C waits
on the Rover) · 3.156 harness x-driver f3ad21a4 · 3.190.B criterion 7 and
3.190.G (room-prior models: Opus 5.5 kept as default, the user 2026-11-23;
`docs/evaluations/ROOM-PRIOR-MODELS-2026-11-23.md`, artifact
https://claude.ai/artifact/C8bZtqHV8BoiPmoV3J2wbA).

## For the user (desktop or account)

* Run on the desktop under the exclusive lock: 3.202.C (40-fault soak,
  5-6 h); 3.205 N8 (live, load <= 4).
* 3.211: check nav2 path shape on `scaled_house` under Dijkstra, then
  rebuild `vision-picar-ros:latest` and the Jetson's image from `dev`.
* Push `docs/3.207-held` (the docs orchestrator's handoff,
  HANDOFF-2026-11-22-docs-orchestrator.md).
* The 3.190.C Lambda deploy session (session_01JbXvZr8BjNUrtk2EKkfRtj)
  waits on deploy permissions or a laptop deploy.
* Cloud sessions cannot delete remote branches. All fully in `dev`:
  plan/3.161-nogo-keeper, plan/3.161-no-go-zones-on-twin,
  plan/3.198-return-goal-race, plan/3.202-sim-chaos-soak,
  plan/3.205-carried-map-no-memory, plan/3.208-action-failure-budget,
  plan/3.210-reseen-crowded-out, plan/3.211-navfn-plan-failures,
  plan/3.211-navfn-plan-failures-rebased, plan/3.212-keep-live-logs,
  plan/3.213-false-sighting-replaces-approach, plan/3.190-b7,
  plan/3.190-b7-sonnet, plan/3.190-g-sonnet5, plan/3.190-g-sonnet5-r,
  claim/3.210; plus HANDOFF-2026-11-21b's list. plan/3.190-prior-at-start
  is older: check before deleting.
* Bedrock: Sonnet 5.5 and Fable 5.2 are not yet available on the account
  (first call answers, the next is refused with a Marketplace error).

## Candidates, not opened

* Checking explore's goals against nav2's global costmap before sending
  (one abort per survey seen; no rate measured; 3.211's Left).
* Docs follow-ups left to the docs orchestrator (policy ARCHITECTURE's stale
  goal-chooser passages; a `/spec-review` to move `verified:` dates).

## Facts

* In a worker session in auto mode, a claim the classifier denies cannot be
  approved from the orchestrator: the user acts in that session.
* Worker containers can build the ROS image: start dockerd, then
  `service/slam/build-cloud.sh <own tag>` (packages.ros.org is HTTP-only).
  A 4-core container's load passes 4 with the stack up: live runs there are
  traces, not measurements.
