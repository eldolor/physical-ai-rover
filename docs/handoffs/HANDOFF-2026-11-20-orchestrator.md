# Handoff 2026-11-20: the orchestrator, after 2026-11-19 23:30 to 11-20 08:10 UTC

*Written by the orchestrator session at its handover line (367K context,
`/orchestrate` section 2). The next orchestrator reads this, the skill, and
the "Open steps" line in `PLAN-ros-alignment.md` (updated at 5bbb800b).*

## Nothing is running

Every worker started in this stretch has pushed and released its claim. No
claim is held by an orchestrator worker.

## What landed (all on `dev`)

| Step | dev sha | Outcome |
|---|---|---|
| 3.190.A | 0fa92b99 | Criteria 1-3 met: cloud prior vs none +0.100 reward [+0.051, +0.150], mostly fewer metres; free (cached replies). |
| 3.70.M2 | cb659290 | FakeNav: found 19/20, recall first 20/20 (met); InflatedNav: found 7/20, recall 17/20, time FAILED; live half not run. Row kept open. |
| 3.190.C | 0c6da335 | `find_prior_source` (off): 8, 9b, 10, 11, 13 met; 9 and 12 FAILED as written. |
| 3.191.A | 6ced2820 | New section: explore approach livelock (an approach that "succeeds" without moving is a retry failure). k19 4,662 goals -> 12, found; bar 3 FAILED as written (5.26 m < 10 m). |
| 3.187.G | 4afa5122 | Value of information: "sample too small to tell" (G1, G3, G4 met; G2 (i) too few, (ii) not met). $0. |
| 3.190.B | fb8560f4 | 40 targets on Bedrock Opus 5.5: 4-6 met (40/40, 20/20, 39/40); $0.0741. Row open for criterion 7 (not approved). |
| 3.161.B | eca15f02 | Dock-zone refusal E-a..E-f met. Z5 re-run FAILED identically (82 vs 87): zones read as walls by the room cut and goal features (not 3.191's bug). Row open. |
| 3.156.E | f79dc71c | Live bars on cloud ROS: G1, G3, G4 live, G5 (re-run; run 1 INVALID, recorded) met. |
| Twin deploy | live 275f9dc2 | The user's go; invalidation IEJAIHZKWZJSD1VQS6HTG1RG81; 26/26 paths match. |

Orchestrator docs commits: fc43bfff (spec-lint E6 fix left by 3.190.C),
3b0ad5d1 (3.190.C row done), 68be5461 (the hint rule, the user's decision),
2643fcc6 (3.178.B: B4's fix waits on the Rover, the user's decision),
03d85b32 (3.190.B's go), eee8ccaa (3.187.G row: a stray empty cell made the
board read it free), 5bbb800b (open-step count).

## Waiting on the user (exact questions)

1. **The serverless deploy for `/room-prior` (3.190.D needs it).** A dry-run
   change set (deleted after) showed: add RouteRoomPrior, modify the
   Distribution and VisionFunction code, and Dynamic re-puts of
   ApiInvokeRole's policy and both integrations; no replacement or delete.
   The code zip is uploaded: `lambda/vision-20261120-055320.zip` (walks zip
   `lambda/walks-20261120-055320.zip` unused). The user added an inline
   policy `serverless-stack-deploy` (change sets, the two functions, the API
   `lajnfx1qc2`, the distribution, PassRole to the stack's roles); executing
   still needs `iam:PutRolePolicy` on ApiInvokeRole. Options given: (a) add a
   temporary statement for it and remove it after, (b) deploy from the laptop
   on Nov 23. Recommended (a). Not answered. If the user adds the
   deploy policy, verify whether they keep it afterwards (it widens a
   plain-text key).
2. **3.187.C:** the orchestrator recommends recording "not needed" (G's sample
   limit is frames and walks, not answers: 2,243 of 2,261 frames already
   carry a recorded answer). The user said "keep 3.187 open for now"; the
   "not needed" note is not yet written.
3. **Z5:** a step so no-go zones stop reading as walls in the room cut and
   goal features? Not answered.
4. 3.190.B criterion 7 (another model's arm) not approved.

## Facts verified, not yet elsewhere

* The environment's AWS key works again (the user re-entered the original
  secret; a resumed session got SignatureDoesNotMatch, a fresh one worked).
  `APP_SHARED_SECRET` is deleted from the environment (fresh sessions only).
* vision-picar-deployer now has Bedrock invoke on Opus 5.5 (inline
  `bedrock-opus-5-5-invoke`, the user 2026-11-20); it lacks
  `lambda:GetFunctionConfiguration` unless the deploy policy was added.
* Workers' own permission checks need the user's words in THAT session
  (the twin's `sync.sh` was blocked until the user typed it there).
* `release --force` on another session's stale claim was run by the
  orchestrator on the user's explicit words ("run release --force 3.156.E
  for me").
* A plan row with an extra `|` cell reads as free on the board
  (`plan_claims._cells`); check `awk -F'|' '{print NF}'` when a done row
  shows free.
* `tests/test_person_stop.py::test_a_persons_goal_or_a_scripts_after_a_stop_is_accepted_and_drives[True]`
  failed once in a full run, 3/3 in module order (recorded in 3.161).
* A Stop with no `x-driver` reads "stop: unknown pressed Stop" (3.156.E's
  note, for whoever owns reason strings).
* Quota: 31% (11-18 ~22:45 UTC) -> 51% (11-20 ~08:00 UTC, the user's
  screenshot); resets Sat 6:30 PM the user's local time. The user: stop at
  85%, do not worry about pace.

## Remote branches for the desktop to delete (content in `dev`)

`plan/3.190-prior-at-start`, `plan/3.70-m2-find-last-seen`,
`plan/3.191-explore-approach-livelock`,
`plan/3.191-explore-approach-livelock-rebased`,
`plan/3.187-cloud-call-bandit`, `plan/3.156-return-to-dock`, plus the
earlier handoffs' lists.

## Next cloud-doable work

None free without the user: 3.190.D waits on the deploy; Z5's step on the
user's word. Everything else free needs the Jetson, the Rover, or a paid GPU.
