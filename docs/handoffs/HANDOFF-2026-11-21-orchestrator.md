# Handoff 2026-11-21: the orchestrator, 2026-11-20 19:20 to 11-21 05:30 UTC

*Written by orchestrator session session_01JbXvZtTf4j1NzXYWvwRdSS at its
300K handover line. Read this, the skill, `HANDOFF-2026-11-20b-orchestrator.md`
(the day before), and the "Open steps" line in `PLAN-ros-alignment.md` (last
updated at b5c46876: re-run `board --all` and update it at the next merge).*

## Running (workers report to the orchestrator; tell each your session id)

| Session | Step | State at handover |
|---|---|---|
| session_01JbXvZpGXUd6aCqu78u9KNZ | 3.190.D | push to dev approved (branch e519ab1b); 15 met, 14 FAILED (first goal a dining-room frontier); $0.0037. Confirm the dev sha, then it releases. |
| session_01JbXvZrSwqmKDj3yX4gyk5U | 3.198 return/dock | re-scoped (amendment 1, f348d047) to brain/dock.py: the creep's livelock (0.30 m quantised verbs under drive: ros; pivot cycles to 40 actions) plus one bounded short nav2 goal. Building and measuring. |
| session_01JbXvZsp865PBmY9c95uuFm | 3.192.B re-measure (Sonnet) | F1, F1', F9, F10 on current dev; no code change. |
| session_01JbXvZqpYaPT5kkPoBiyonG | 3.199 (D5: refuse a goal inside a zone's drawn margin) | building. |
| session_01JbXvZpZWBkPXDDsgpPaUms | new: a refused recall keeps its place | just started; stops with a finding if 3.175.E's reason holds. |

Queue (the user, 2026-11-21): 3.70.M3 (free since M2 was marked done), then
3.187.D (re-scoped onto recorded answers; free). At most 4 workers at once.

## Landed since the last handoff

| Step | dev sha | Outcome |
|---|---|---|
| 3.70.M2 done; 3.187.D/E re-scoped | 974ae9bb | the user's decisions |
| 3.197 | b673d89a | a boxed robot frees only a short vent; whole-zone suspensions 2,913 -> 0 of 12,000; 1,122 positions that could not plan out -> 0; live 10/10 found and docked; the vent not opened live (L6) |

## Done outside the repo

* The deployer's main policy was replaced (2026-11-21) by a least-privilege
  one for the serverless stacks (CloudFormation on vision-picar-*, three
  buckets, both functions, API lajnfx1qc2, distribution E1VYDK2ACPS00K,
  stack roles, Lambda logs, Bedrock); ECS/ELB/EC2/ECR/Secrets Manager
  dropped. `TempRoomPriorDeploy` deleted. Read checks all pass without it.
* A network secret on this environment: `x-app-secret` for
  `d114x92g7i4syl.cloudfront.net` (the deployed vision secret). Every
  session in the environment can make paid /navigate and /room-prior calls.
  The user may delete it once 3.190.D is merged; remind them.

## Waiting on the user

1. The twin's deploy (3.190.D's mission-card line changed web-twin/).
2. 3.190.D question 4: keep the goal chooser's prior-vs-distance balance, or
   weight a cloud prior more (its own step; 3.190.A's harness measures free).
3. The Home form's target field reads "red backpack" while an API-started
   find searches for something else (a candidate UI fix).
4. 3.190.B criterion 7 (recommended skip).

## Facts

* Quota: 60% at 2026-11-21 ~04:13 UTC (the user's screenshot), resets
  Sat 6:30 PM the user's time; stop at 85%.
* Remote branches to delete from the desktop (content in dev), adding to the
  last handoff's list: `plan/3.197-nogo-local-free`, and each of today's
  once merged.
* The shared `vision-picar-ros:latest` and the board's image predate
  3.193/3.195/3.197's bridge changes.
