# Handoff 2026-11-18: the orchestrator, after 2026-11-16 to 18

*Written by the orchestrator session at its 300K-token handover line
(`/orchestrate` section 2). The next orchestrator reads this, the skill,
and the "Open steps" line in `PLAN-ros-alignment.md`.*

## What landed (all on `dev`, every claim released)

| Step | dev sha | Outcome |
|---|---|---|
| 3.178.B | 50a5a8c8 | The watch list in the simulator. B1 met (thin sample), B2 FAILED, B4's sim half met; B3-B6 not run (desktop frames, Jetson). The list is ON by default (the user, 2026-11-16). |
| 3.175.C | 0bdde1ad | "Where did you last see it": `GET /mission/last-seen`, the twin's Home row. C1-C7 met. |
| 3.175.G | 23532315 | C's second review's fixes. G11 recorded. Took 7 review rounds: since then every brief caps review-fix rounds at two. |
| 3.175.H | b11edd07 | The runner reports its inventory save state; the route reads it (G11 fixed). An ended mission's sightings drop out of the answer once its save lands (as designed: the next find carries the same). |
| 3.178.D | 0f6d2e8b | The two-look pin rule: D1, D2, D4 met; D3 FAILED (4 of 5 passes). Not shipped. |
| 3.178.E | c0c5e0df | The same on a placed sweep (50 passes): D3 FAILED 41/50, and dev's rule 44/50, because 6 passes never reached a report. Not shipped. |
| 3.178.F | fd2cbd95 | Why passes miss a report: 3 were one-view glimpses (a measurement fix: a pass needs two independent views, accepted by the user 2026-11-17), 4 are ranging losses (open). Under two-view passes: label 46/47, votes 43/47; wrong pins label 7, votes 3. No product code changed. |
| 3.178.G | 97ebf657 | Box-extent ranging: STOPPED at the honesty check. The sim has no box; no item sizes exist; `Perception.watch` has no path into the inventory (3.76 E). Waits on the Rover. The ground-plane (box bottom edge) cue is noted. |
| 3.178.H | 27ef8b52 | The votes rule's verdict under the two-view pass: parked (the user, 2026-11-17), waits on the Rover. |
| 3.187.A | 0c35099f | Three arms (the user, 2026-11-18), criteria amended before any paid data, split, the pilot's 20 frames, B's desktop recipe. Nothing paid. |

## Waiting on the user

* **3.187.B, the 20-frame paid pilot:** the user's go is given (2026-11-18),
  to run from the desktop after Nov 23 (the app secret is there). Recipe:
  section A6 of `3.187-cloud-call-bandit.md`; `count.py` first. Stops at
  $1.00 spent. C's full replay (estimate ~$27) needs the user's go on B's
  figure.
* **The desktop (after Nov 23):** 3.175.C's live-stack ask (one find, one
  ask over HTTP); 3.178.B's B3 and B4's frame half; 3.161.B, 3.70.S4,
  3.70.M1 (Docker); 3.188.A2 (frames).
* **The Jetson:** 3.178.B's B5-B6.
* **Remote branches to delete** (all content in `dev`; cloud sessions
  cannot delete): `plan/3.178-watch-list`, `plan/3.175-last-seen`,
  `plan/3.175-save-state`, `plan/3.178.D-pin-rule`,
  `plan/3.178.D-pin-rule-rebased`, `plan/3.178.D-r2`,
  `plan/3.178.E-passes`, `plan/3.178f-reach`, `plan/3.178.G-range`,
  `plan/3.187-cloud-call-bandit`.

## Lessons for the next orchestrator

* **A step row's Touches must be plain ticked paths,** one per comma, no
  notes in brackets: `claim` refuses anything else (3.175.G's first row).
* **A follow-up on a step marked done needs its own row**: `claim` refuses
  a done step.
* **Pushing a plan-row edit needs a claim on the section**: claim it,
  push, release at once (the push gate refuses otherwise).
* **A step that needs only another step's merged code, not its open bars,
  should name the earlier step in Needs** (3.178.D), or the board
  shows it blocked.
* **Brief a box/sensor fix with an honesty check first** (3.178.G stopped
  in minutes instead of building on a cheating simulator).
* Quota: 18% (2026-11-16 07:36 UTC) -> 22% (11-16 ~10:20) -> 30%
  (11-18 ~13:15) for 11 worker runs. Resets Sat 2026-11-21 12:59 UTC.

## Later on 2026-11-18: the twin deployed from the cloud

* The user's go: `dev` `413a806b` deployed by a cloud session (bucket
  `vision-picar-serverless-staticbucket-2jbqpgdcnvtv`, distribution
  `E1VYDK2ACPS00K`, invalidation `IRI5JIQRQM9JB74EF1VCEG7KD` Completed). All
  26 `assets.json` paths match `HEAD` by sha256; content types right; the
  Home tab loads with no page errors against an older brain (last seen,
  Find, schedules each degrade quietly on a 404).
* Cloud sessions now have valid AWS keys (IAM user `vision-picar-deployer`,
  us-east-2), the AWS CLI from the environment's setup script, and the
  CloudFront host on the network allowlist. The key is for the twin deploy
  only; Lambda deploys stay on the desktop.
* Advised the user to drop the old ECS statements from that user's policy
  (`iam:PutRolePolicy` + `iam:PassRole` + ECS is a path to admin, and the
  key sits in plain text in the environment). Not verified whether they did.
* The cloud deploy item in `HANDOFF-2026-11-16-desktop-return.md` section 1
  is done; that section's remaining note (the board's brain lacks 3.176.B's
  parse route) still holds.

## 2026-11-18 to 19: the desktop's work moved to the cloud

* **Landed on `dev`:** 3.187.B (`f78ceac2`: the pilot, 20 calls, $0.2129,
  $0.010645 a frame; C estimated $20.10 for 1,888 frames, $24.07 for 2,261;
  C waits on the user's go); 3.188.A2 (`de7b1118`); 3.178.B's frame half
  (`6400e9fb` tool, `e47a1ef7` results: B3 met, B4 FAILED, 413 vs 422);
  3.189 (`f8f2d363`: the ROS image builds in a cloud session in ~7 min;
  outcome-judged live runs valid, timing-judged not, load peaked 4.68).
* **The cloud environment (Default) now has:** AWS keys for
  `vision-picar-deployer` (twin deploy, recordings bucket read); the AWS CLI
  from its setup script; allowed hosts `d114x92g7i4syl.cloudfront.net`,
  `huggingface.co`, `cas-server.xethub.hf.co`, `us.aws.cdn.hf.co`,
  `ftp.osuosl.org` (the ROS apt mirror over HTTPS; `packages.ros.org` is
  HTTP-only and unusable behind the HTTPS-only proxy); the app secret as
  `VISION_SECRET` (renamed 2026-11-19).
* **`APP_SHARED_SECRET` in a session's environment breaks the suite**
  (`require_secret()` goes live: server tests 401 and the run hangs). The
  variable was renamed; a session still carrying it runs
  `env -u APP_SHARED_SECRET -u LOCAL_SECRET python tools/plan_section.py ready`.
* **Workers' own permission checks** refused, each needing the user's own
  words in the worker's session: the paid `pilot.py --pay`, a
  `pip install git+https://github.com/ultralytics/CLIP.git`, and claiming
  another section's step to pass the push gate. A message from the
  orchestrator does not count; tell the user at once and name the session.
* **More remote branches for the desktop to delete** (content in `dev`):
  `plan/3.188-qwen-finetune`, `plan/3.178.B-frames`,
  `plan/3.187-cloud-call-bandit` (pre-rebase copy), `claude/pensive-hamilton-e0il1d`.
  Keep `claude/fervent-brown-kzq854` (3.180.B's failed code, kept as a record)
  unless the user says.
