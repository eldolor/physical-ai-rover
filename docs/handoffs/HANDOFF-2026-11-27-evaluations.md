# Handoff 2026-11-27: the evaluations, to the orchestrator

*Written by session session_01JbXvZqrpG7ymk8YXJkbcKQ for the orchestrator,
on the user's instruction (2026-11-27): "I want to make sure that the
orchestrator manages any evaluation that's been done." The standing rule is
now in `.claude/skills/orchestrate/SKILL.md` ("Evaluations are the
orchestrator's"). This session holds no claims after this commit.*

## Next to start (free, its spend already approved)

* **3.188.B, the fine-tune** (3.188 amendment 1, 2026-11-27): Hugging Face
  Jobs, the box answer, 3.206.C's 314 labels, 3.206's 200-frame test draw
  held out of every fold. The stock baseline, fold 1 as the cost pilot,
  then folds 2-6 and the all-objects model. **$10 stop, approved by the
  user 2026-11-17**; frames to Hugging Face accepted for this training on
  2026-11-23. 3.206.D's criteria 4-10 are read from the same run. Opus
  for the training script (new code, paid, stoppable with calls in flight:
  both reviews); the launcher pattern is `evaluations/teacher-probe-3214/`.

## Evaluations on this line, and where each stands

| Section | Done | Open | Waits on |
|---|---|---|---|
| 3.214 teacher probe (72B, Alpamayo 2 Super) | all; no teacher proposed; $8.15 | -- | -- |
| 3.206 steering and identity teacher | A (197-frame test set; local order 85.3% unflagged), B0, B1, B2 (235B failed identity: 46 false yeses vs VE's 26; $1.05), C (314 labels; criterion 2 106/106; the user's audit 50/50) | D (= 3.188.B), E, R | D: 3.188.B; E: 3.188.C and the Rover; R: the Rover |
| 3.188 local VLM fine-tune | A, A2 | B, then C-E | B: free to start; C: the user (a second host for the export); E: the Rover |
| 3.190 cloud room prior | A-D, F, G (Opus 5.5 stays; Sonnet 5 ties at half the cost) | E | the Rover |
| 3.187 cloud-call decision | A, B, D, G | E, F | E: D's bars; F: the Rover |

## Decisions the user made on this line (each in its section file)

* 3.206: train the box answer; the 72B is the teacher on in-view frames
  only; a label is kept when the 235B's box overlaps it (IoU >= 0.5);
  border frames scored apart; the test labels made by a model and
  spot-checked by the user (20 of 20 agreed); `frame-0184` corrected to
  FORWARD; training on Hugging Face Jobs.
* 3.206.R (2026-11-23): every measurement is run again on the Rover's
  feed before anything trained on phone frames ships to the car; the
  current decisions stand until then.

## Loose ends

* Two private artifacts holding photos of the home have done their job:
  the spot-check (https://claude.ai/artifact/RDyuTthWa4QWX9e3H23qK4) and
  the box audit (https://claude.ai/artifact/V1vpEPGwqXHYFW7nnqMr5L). The
  user has not yet said whether to delete them; ask, and delete only on a
  yes.
* The explainer "Learning When to Ask"
  (https://claude.ai/artifact/YPg3HkL29U9qGHxYPnKHsQ) is current to
  3.206.A; it needs a line once 3.188.B reports.
* The deploy bucket's private `gpu-eval/teacher-probe-3214/` holds the
  72B's full replies, which 3.206.C read; keep it until 3.188.B is done.
