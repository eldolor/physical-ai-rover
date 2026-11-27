# Handoff 2026-11-27: the orchestrator, 2026-11-26 19:00 to 2026-11-27 15:30 UTC

*Written by orchestrator session_01JbXvZsJtBazSzMLFNo4DaT (Opus) at ~366K
tokens of context (the skill's 300K handover line). Read the orchestrate
skill first (its new "cloud by default" and "evaluations are the
orchestrator's" rules), CLAUDE.md's top rule (questions one at a time, as
pop-ups with a recommendation), then this file, then
`HANDOFF-2026-11-27-evaluations.md`.*

## Messaging and scheduling

* This session approved the trust card (2026-11-27 ~12:30 UTC): it could
  send to desktop sessions and start them (`create_session` with
  environment `env_0121F6AWpQkBc2tfz8hNGWHG`, the desktop bridge), but every
  message INTO it was refused. A fresh orchestrator starts unapproved:
  workers can message it, it cannot message desktop sessions. Ask the user
  before approving the card; it is permanent for that session.
* Briefs say: "put your full report in your final message; never ask the
  user to relay". Read workers with `list_events` (kinds assistant,
  limit 30-100; big pages go to a file).
* The every-3-hours check `trig_01Lrj5GC6Qx6FeK4pN3Q7CPj` fires into
  THIS session: point it at the new one (update_trigger can't change the
  target; delete it and create a new one) or delete it.

## Quota

* `/usage` 2026-11-27 13:36 UTC: 87% all models, 78% Fable; past the 85%
  reserve. Resets Sat 2026-11-28 13:00 UTC. Start no workers before then.
  Don't use the full reset (expires Dec 10) unless the user hits the limit
  mid-task.

## Workers

None running. Archived today: 3.140 (x2), 3.216, 3.136.B, 3.207, 3.219,
3.220 (x2), 3.221, the 3.224 explore worker. Left: 3.222 OAK-D
(`session_01JbXvZsaDvSj96XhrRTX2bb`, desktop, holds claim 3.222; K7 FAILED
155.6 ms vs 150; K1-K5 wait on the user at the Rover; ~360K tokens: a fresh
session is cheaper). The 3.202.C soak worker (`session_01JbXvZr5cCmPoQubYu5Fs3n`)
was asked to release its 3.202 claims (they show released now); archive it.

## Done 2026-11-26/27 (all on dev)

* 3.220 pan-tilt: tilt buttons, twin deployed from `a444892a`. Criterion 6
  NOT met (no servo position reads; signs and clearance met) -> 3.223
  opened. ROBOT_GIMBAL stays off.
* 3.224 reverse along its own track: criterion 6 met on the Rover.
* 3.225 battery reserve: met on the Rover (surveys run ~10 min, end on
  the time reserve).
* 3.221: step B DROPPED by the user (revisit if missions fail).
* The board: dev `ea9a5e4f`, sim mode; reach it at 192.168.86.28
  (picar-jetson.lan no longer resolves). The user shuts it down at night.

## Queue after the reset (the user's decisions, 2026-11-26/27)

Ask for a fresh `/usage` first. 2 workers at once, cloud by default:
1. The trip home fails 4/4 (`nav2_aborted`) and survey 2 failed 9/10
   goals: investigate from the run logs (`~/survey3224a.jsonl`,
   `~/survey3224b.jsonl` on the board; 3.140's log). Top Rover blocker.
2. Explore on the car can't see a target (reads `frame["detections"]`,
   sim only, 3.76 E): build real-frame detections into explore (cloud),
   then L4 on the Rover, then the first find mission (L5).
3. 3.202 c1 fix (a FORWARD runs on after its driver dies; Opus, cloud;
   soak5 later on the desktop).
4. 3.223.A (pan-tilt never read back: bars first; the user decides
   whether to trust commanded angles).
5. Sonnet, one at a time: the sim picture tilting with the camera (3.220
   amendment), the CLAUDE.md trim (over SOFT), the pre-push hook fix
   (skip pushes already on origin/dev), the ~48 old-branch list.
6. Evaluations (now the orchestrator's): 3.188.B is next (HF Jobs, $10
   stop already approved).
7. 3.226 (twin Drive page live data) is reserved by the user's ask; bars
   not written.

## For the user

* Amazon return decision for the defective board, about Dec 1 (window
  closes Dec 8).
* Delete two private photo artifacts from 3.206 (see the evaluations
  handoff)? Ask, delete only on yes.
