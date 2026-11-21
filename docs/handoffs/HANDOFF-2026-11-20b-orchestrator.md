# Handoff 2026-11-20 (b): the orchestrator, 10:30 to 19:20 UTC

*Written by the second orchestrator of 2026-11-20 (session
session_01JbXvZtTf4j1NzXYWvwRdSS after a /clear) at its handover line.
The next orchestrator reads this, the skill, and the "Open steps" line in
`PLAN-ros-alignment.md`.*

## Running

Nothing. One worker stands by holding a claim:
3.190.D, `session_01JbXvZpGXUd6aCqu78u9KNZ`, branch `plan/3.190-prior-at-start`
at 1f052aad (pushed, not on dev; `ready` passed). Stopped because the deployed
`/room-prior` answered 401 to the session's `VISION_SECRET`. Built: the twin's
`#mission-prior` line and `tests/test_ui_room_prior.py`; amendment 4; the live
harness `evaluations/room-prior-3190/live/live.py`. Criterion 14 not met
(invalid: http 401); 15 half met (two 390 px screenshots; the cloud one owed).
Left: a secret the route accepts, then one re-run (~45 min, 1 paid call
~$0.002), then the twin's deploy (the user's).

## Landed (all on `dev`)

| Step | dev sha | Outcome |
|---|---|---|
| 3.187.C | 7c9f8283 | dropped (the user: not needed) |
| 3.192 / 3.192.B | f5ddadea | zones read as floor for frontiers, cut, features; F1 86 vs 87 (dev 82), F1' time +5.08% (dev +18.53%), F9, F10 FAILED; open |
| 3.70.M2 live half | 0cd3e56d | L1-L3 met 5/5 on cloud ROS; time bar FAILED earlier; open |
| 3.193 | fb255eb8 | the bridge marks every cell a zone touches; D4 boxing rule; /health `suspended` ids; E2 failed as written |
| 3.194 | 0b2cf5cb | a dock read late (pose unusable at start) still goes home; L1-L5 met |
| 3.195 | d649a398 | zones drawn grown 0.05 m, rounded; 0 entries / 3,565 samples; C3 FAILED (one preempted-return dock race) |
| 3.196 | 53992f4f | diagonal pinches beside a zone closed; unmoved aborts near a zone charged; G1-G11 met |

## The `/room-prior` deploy (done, 16:12 UTC)

Stack `vision-picar-serverless` UPDATE_COMPLETE with code key
`lambda/vision-20261120-055320.zip` (built from dev ca5c1c27). Four earlier
tries rolled back cleanly on missing IAM (one rollback stuck on
`lambda:UpdateFunctionCode` and was continued skipping VisionFunction, which
had never changed). The user's inline policy `serverless-stack-deploy` did
NOT grant Lambda code, API Gateway or CloudFront distribution access; a
temporary inline policy `TempRoomPriorDeploy` did. The user was asked to
delete `TempRoomPriorDeploy` and to review `serverless-stack-deploy`;
confirm it is gone.

## Waiting on the user

1. 3.190.D's secret: a network secret for `d114x92g7i4syl.cloudfront.net`
   (header `x-app-secret`, the stack's `VisionSharedSecret`), or the laptop
   run. A `request_credential` call was interrupted by a worker restart.
2. Candidate steps (orchestrator's recommendation in brackets):
   the return refused "preempted" when a find ends with nav2's approach goal
   active (3.195 C3) [open]; a finer D4 that frees only the robot's own
   cells at slanted edges and corners (3.195 Left c) [open]; D5 refusing a
   person's goal within the drawn margin (3.195 Left a) [hold]; a refused
   recall keeps its place (3.70.M2) [hold].
3. 3.190.B criterion 7 [skip until 3.190.D shows the prior's value live].

## Facts verified, not yet elsewhere

* Cloud containers run Docker: start `dockerd` by hand, build with
  service/slam's `build-cloud.sh` under your own tag (3.70.M2, 3.192-3.196 did).
* Cloud sessions cannot delete remote branches (HTTP 403 or a silent no-op).
  Stale, content in dev: `plan/3.70-m2-find-last-seen`,
  `plan/3.192-nogo-reads-as-floor`, `plan/3.193-nogo-mask-edge-redraw`,
  `plan/3.194-late-dock`, `plan/3.195-nogo-corner-cut`,
  `plan/3.196-nogo-abort-loop`, plus the earlier handoffs' lists.
* The shared `vision-picar-ros:latest` and the board's image predate 3.193 and
  3.195's bridge changes.
* Quota: last reading 51% at 2026-11-20 ~08:00 UTC; resets Sat 6:30 PM the
  user's time; stop at 85%. Eight worker runs since that reading.
