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
