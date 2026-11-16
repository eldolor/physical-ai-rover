# Handoff: back at the desktop (after 2026-11-23)

Written 2026-11-16 by the orchestrator session (cloud), for the user's return
to the desktop. Cloud sessions cannot deploy (the environment's AWS keys are
rejected), run Docker, reach the Jetson, or delete GitHub branches; these are
the things left for the desktop. Check each against `dev` before acting.

## 1. Deploy the twin (the user's go, 2026-11-16)

* The live twin is from `dev` `5e88b9de` (2026-11-05). Deploying ships
  everything since: 3.176.C's Find box, 3.156.E's Return row, 3.163.B's
  schedules, 3.160's room names, 3.152.A's readout.
* `bash service/static/sync.sh <bucket> <distribution-id>`; wait for the
  invalidation to report `Completed`; check parity
  (`curl https://<dist>/app.js` vs `git show HEAD:web-twin/app.js`).
  CLAUDE.md "The twin's deploy" has the upload order.
* The Find box reads sentences only with a brain that has
  `POST /mission/request/parse` (3.176.B). The board's brain is `dev`
  `bdcf782c`, older: with it the twin sends the words as typed. Updating the
  board's brain is the user's call (`tools/jetson/README.md`).
* Optional, separate: fix the cloud environment's AWS keys so cloud sessions
  can deploy too.

## 2. 3.156.E's live bars (the user, 2026-11-14)

* G1, G3, G5 and G4's live half need the ROS container. Code is on `dev`
  (`89c6e09f`); the step stays open. Its claim is held by a cloud session:
  take it over from the desktop with `plan_section.py claim 3.156.E` (the
  holder is another host, so the user's `release --force 3.156.E` first if
  refused).
* The harness's start goal must be sent as a person (`x-driver: twin-dpad`);
  a run at load above ~4 is invalid. Three low review notes for whoever runs
  it are in 3.156's section.

## 3. The old desktop sessions

* "3.156: return to dock (build)" ran 3.175.B; "3.162: what I saw" ran
  3.156.E. Both hold unpushed copies of work that was redone in the cloud and
  is on `dev`. Do NOT let them push; delete their local branches
  (`plan/3.175-memory-moved-b`, `plan/3.156e-deferred`) and worktrees.

## 4. GitHub branches to delete (cloud sessions can't)

* Test refs: `claim-test/3179b-probe-1`.
* Released claim records (3.179.B made cloud releases work by overwriting;
  the records can now be deleted): every `claim/*` whose message starts
  `released` (`git ls-remote origin 'refs/heads/claim/*'`, then check each).
* Cloud session branches whose content is on `dev` (rule 11 check first):
  `claude/3.176-C-find-box`, `claude/3.186-harness-repo-path`,
  `claude/fervent-brown-kzq854`, `claude/new-session-1uc42o`,
  `claude/new-session-cxvjl7`, `claude/pensive-hamilton-e0il1d` (this one
  also holds 3.175.E/F's unshipped rule code; it is recorded in their
  results), `claude/orchestrator-agent-workers-1vhpq5` (the /orchestrate
  skill is on `dev` since `1ba8b11e`).

## 5. Facts to know

* `dev`'s first-parent history carries 3.175.E's unshipped retry commits as
  ancestors (via 3.175.F's row push, `ebc25bf6`); `dev`'s files are not
  affected. Making it linear would need a force push: the user's call only.
* The open-step count is at the top of `PLAN-ros-alignment.md`; the
  `/orchestrate` skill keeps it current.
