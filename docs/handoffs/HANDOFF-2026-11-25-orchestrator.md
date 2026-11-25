# Handoff 2026-11-25: the orchestrator, 2026-11-23 15:40 to 2026-11-25 13:30 UTC

*Written by orchestrator session session_01JbXvZtTf4j1NzXYWvwRdSS at 348K
tokens (past the skill's 300K line). Read the orchestrate skill first: since
`ee1c5e53` the worker count follows measured usage, not a fixed cap, and all
orchestration work, code and docs, lands with the orchestrator (the user,
2026-11-25). CLAUDE.md's top rule: questions one at a time, as pop-ups with
a recommendation.*

## Quota (the user's `/usage`, 2026-11-25 13:27 UTC)

All models **68%** used, Fable only 74%; resets **Sat 2026-11-28 12:59 UTC**.
About 57% of the week has gone, so use is **ahead of pace**. The app warns it
runs out Thursday night. By the skill: start no new worker until the week is
back on pace, or until the user uses the free full reset (offered to the user,
expiring Dec 10; it is theirs to use). Finish what is running. A worker that
waits on a background run costs little; a reviewing or coding worker costs a
lot.

## Workers (all on the user's desktop; bridge environment env_0121F6AWpQkBc2tfz8hNGWHG)

Desktop sessions' `send_message` to a cloud orchestrator is REFUSED (trusted
device). Read their final messages with `list_events` (kinds `assistant`, a
high limit), and brief every new worker to put its report in its final
message.

| Session | Step | State |
|---|---|---|
| session_01JbXvZssvXjcCCcNzQFTaYt | New section (number it assigns): a robot-server death mid-move stops inside the margin (soak3's K3 contact). Bars first on ground truth; candidates: a shorter heartbeat in our firmware fork, a speed cap tied to clearance; both reviews (safety path) | running since 09:11; push only on the orchestrator's word |
| session_01JbXvZq3Bpc6wRUGXLrrfCP | 3.136.B, the Jetson's steady state | told to write amendment 11 (an aborted `recall` goal counts like frontier/approach, the orchestrator under delegation) and push runs 4-5 (both INVALID) plus it; keeps the claim for the next window |
| session_01JbXvZr5cCmPoQubYu5Fs3n | 3.202.C, the soak | soak3 PARTIAL on dev `ccf5ad13`; holds the 3.202 claim; stands by |
| session_01JbXvZsm7WXQDPb1JA9GFqg | 3.140 Rover live checks (the user's own session, managed) | ended by the user: the motor board doesn't hear the host and now doesn't start |

## Decided, waiting on its trigger

* **Re-run 3.202.C's soak after the K3 fix merges** (the user: "fix first,
  then re-run"). Start it detached from any Claude session (`systemd-run
  --user` or `setsid`): the desktop's Claude host exited at 06:15 UTC on
  2026-11-25 (no reboot, sleep or OOM; Claude Code still 2.1.309) and
  SIGTERMed soak3 4.0 h in, and 3.136.B's run 5 at 00:21 local.
* **3.136.B's next window** when the user powers the Jetson (shut down
  2026-11-25). A valid window also needs 0 late wheel ticks (P4 failed in runs
  1, 2 and 4). P8 (local VLM p90 1.94-1.99 s against 1.5 s, four windows) is a
  candidate for its own section.
* **Queued, user-approved 2026-11-24:** (1) explore's goals checked against
  nav2's global costmap, a new section whose step A only measures the rate
  (in-process sweep, cloud-doable); (2) a small trace of nav2's clearance,
  0.34-0.39 m in 3.211.C against R6's 0.165 m (history of inflation/costmap
  settings, then one desktop run).

## For the user

* `claim/3.207` is still on origin (`ee253b7e`): its landing worker asked for
  the user's typed OK to `release --force 3.207` and was archived before
  getting it. Housekeeping only.
* About 48 branches and 49 claim refs on origin whose content is not in `dev`
  were left for the user's review by the housekeeping worker (archived).
* The Rover: the motor board's serial receive is deaf on both routes (3.140's
  log), and it no longer starts. The replacement's selection criteria are in
  `JETSON-BOM.md` (`86c56910`). Return window closes 2026-12-08.

## Landed 2026-11-23 to 25

3.211.C `755728f6` (all six bars met; `vision-picar-ros:latest` from dev
`aebbe919`) · the Jetson's ROS image `c5614b27` (`8bb47dc8c21c`, old kept as
`:pre-86c56910-20261124`) · 3.205.B `4388ef28` (N8 met on one valid live find)
· `docs/3.207-held` `90c64259` · soak3 partial `ccf5ad13` · the skill's
worker rule `ee1c5e53`. 3.190.C's Lambda was already deployed (2026-11-20
16:12 UTC).
