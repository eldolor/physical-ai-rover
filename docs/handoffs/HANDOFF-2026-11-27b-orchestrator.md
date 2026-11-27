# Handoff 2026-11-27b: the orchestrator, 2026-11-27 15:26 to 21:15 UTC

*Written by orchestrator session_01JbXvZpeQ7RiW9hJsEc7q6w (Opus) at ~283K
tokens of context (near the skill's 300K line), with nothing running.
Read the orchestrate skill first, then CLAUDE.md's top rule (questions one
at a time, as pop-ups with a recommendation), then this file, then
`HANDOFF-2026-11-27-orchestrator.md` and `HANDOFF-2026-11-27-evaluations.md`.*

## Messaging and scheduling

* The trust card was NOT approved (the user's choice, 2026-11-27):
  workers can message the orchestrator, but the orchestrator can neither
  start nor message desktop sessions. For a desktop step, write the brief
  to the scratchpad, send it with SendUserFile, and the user pastes it into
  a new desktop session; relay "push to dev" messages through the user
  with the session's title.
* The every-3-hours check is `trig_01Rf9ZkRvmDmE5mPBmJBsq26`. It fires
  into session_01JbXvZpeQ7RiW9hJsEc7q6w: delete it and create your own
  (same cron `11 */3 * * *`, the queue below in its prompt).
* Cloud containers can push but cannot delete refs on origin (403); a
  desktop session deletes leftover `plan/*` and `claim/*` refs.

## Quota

* `/usage` 2026-11-27 16:40 UTC: 88% all models, 78% Fable; the app warned
  of running out before the reset. Resets Sat 2026-11-28 13:00 UTC. After
  it, ask for a fresh reading (one pop-up), then 2 workers at once.

## Done 2026-11-27 (all on dev, a9fd5197)

* 3.227 explore sees the target on real frames (YOLOE + CLIP on a real
  camera; no identity tier, so a real arrival is `arrived_unconfirmed`;
  limit (a) shipped by the orchestrator's decision: "note the place and
  search on" plus local-VLM identity, decided on L4's false-match rate).
* 3.228 diagnosis: the trip home and frontier goals fail. 3.229 (F1): the
  brain's safety check gets the self box (`_HaltGate`), proven on the
  Rover in 3.229.B (B1, B2, B4 met; 0 refusals at 0 cm).
* 3.229.B's record: the trip home fails at nav2's PLANNER (28 of 28 "failed
  to create plan", a start in a tight spot); survey 2 came within 0.52 m of
  the dock, its last back-off refused `astern_occluded`.
* 3.230 written up (stuck at a mat's edge: traction). 3.231 written up
  (floor obstacles from the pan-tilt camera), with the user's decision.
* Jumper loopback (3.140, 07481cca): the Jetson's UART works; the fault is
  the driver board's receive. Waveshare reply SENT (the user). Return or
  replacement stands; window closes Dec 8.
* Archived: the 3.202.C soak worker; 3.227, 3.229, 3.228 workers. Deleted
  (the user's yes): the two 3.206 photo artifacts.

## The user's decisions today

* Low floor items the lidar misses (a photo: the Rover climbed shoes on
  the mat): BOTH. 3.222's OAK-D floor band is the veto (finish 3.222
  first; its handoff is on dev at eb721a10; K7 FAILED 155.6 vs 150 ms;
  K1-K5 need the user at the Rover). And 3.231: the pan-tilt camera at a
  fixed downward tilt on commanded angles, servo reads off (3.223 option
  2), until the replacement board reads the servos; it steers, never vetoes.
* 3.229 follow-up (d), decided by the orchestrator under delegation: "let
  the brain pass an `astern_occluded` REVERSE to the server, which keeps
  the last word" goes with 3.228 F3.

## Workers

* Paused: 3.147 in `session_01JbXvZpcr6L4YgDEBeNeJ8U` (cloud). No claim,
  no edits. On resume: `claim 3.147`, continue on the existing branch
  `plan/3.147-verb-outlives-dead-brain` (never `new`). The bug: a FORWARD
  runs ~24 cm after its driver dies (3.202 c1; 3.135 F6).
* Idle desktop sessions the user may close: "Rover 3.229 brain self-box
  survey" (`session_01JbXvZpmS2ccr1Um4yChZtF`), and the 3.222 session
  (`session_01JbXvZsaDvSj96XhrRTX2bb`, handed over).
* 3.232 is claimed by a session that is not the orchestrator's.

## Queue after the reset

1. 3.228 F3/F4: make a tight-spot start plannable (footprint or inflation)
   or run the brain's escape before the trip home's goal; with follow-up
   (d). Judge in the sim first (cloud), then a Rover survey (desktop brief).
2. Resume 3.147 (above).
3. 3.222 (desktop brief from its handoff) and 3.231.A (cloud half).
4. 3.230.A (from 3.229.B's trace; desktop: the traces are on the desktop
   in `~/picar-traces-3229/`).
5. Then 3.223.A; Sonnet jobs (the board shows 3.230.C-E and 3.231.A-D as
   "unknown": fix their Needs/Waits cells; the CLAUDE.md trim; the pre-push
   hook fix; ~50 stale `claim/*` refs on origin, deleted only from a
   desktop session); 3.188.B (HF Jobs, $10 stop approved).
6. Not in any plan yet: a local-VLM source for room names (today only
   `frame`, sim, or paid cloud).
* The board's brain is dev `1f6ef3be` since 3.229.B (sim mode); the pack
  needs charging before the next run.
