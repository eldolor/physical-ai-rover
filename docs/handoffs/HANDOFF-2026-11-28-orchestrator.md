# Handoff 2026-11-28: the orchestrator, 02:20 to 15:15 UTC

*Written by orchestrator session_01JbXvZpeQ7RiW9hJsEc7q6w (Opus) at ~314K
tokens of context (over the skill's 300K line). The user /clears this
session and it re-reads this file: same session id, so the every-3-hours
check `trig_01Rf9ZkRvmDmE5mPBmJBsq26` still fires into it (keep it; its
prompt carries the state below). Read the orchestrate skill first, then
CLAUDE.md's top rule, then this file.*

## Standing instructions from the user today

* **"I will go with your recommendations, stop asking me. All your
  recommendations are auto approved."** (~13:00 UTC). No pop-ups; act on
  the recommendation and record it as "the orchestrator, auto-approved by
  the user (2026-11-28)". Still never: force push, PICAR_CLAIM_BYPASS,
  PICAR_SKIP_GATE, release --force, another session's live claim, gaming
  the claim gate. New spending beyond a section's approval is reported.
* **"Pace yourself"** (12:57 UTC). The weekly quota reset Sat 2026-11-28
  12:59 UTC; next reset Sat 2026-12-05 12:59 UTC. At most 2 workers at
  once, ~6 worker runs a day, 15% reserve, Sonnet for docs. Pace by
  `rate_limit_info` (get_session); don't ask for /usage.
* The claim-to-pass-the-gate route (claiming a free step only so a push
  passes) is refused by the auto-mode classifier as "undermining
  oversight", even with the user's approval relayed. Narrowing a Touches
  row to a step's real scope is fine; narrowing one a step really needs is
  gaming.

## On dev today (f3c23db9 and docs after)

* 3.234.A disc movers + robot-caused metric; gate OPEN (19 inside 20 cm,
  all drive: ros). 3.234.B offline lidar tracker (B1, first-track FAILED).
  3.234.B2 revision: first track 45% -> 75.6%, B1 still FAILED (1.44/10 min);
  new 3.234.B4 (the Rover's own scans + B3 Jetson cost; waits rover);
  3.234.C needs B4.
* 3.228.B: the trip home backs out of a tight spot (F4) and the brain
  defers `astern_occluded` REVERSE to the server ((d)); B1 FAILED 25/40 vs
  80% (0/40 before). 3.228.C bars C1-C4 on dev.
* 3.236 "Speed on the car": A measured 3.221's capture stamp on truth
  (median +1.05 ms). B braking (rover), C speed by clearance (after B and
  3.222).
* Docs: MERGED-STATE lines for 3.232, 3.234.A/B, 3.228.B, 3.236.A; 3b's
  stale /scan line replaced; the board reads every row.

## Workers

* **3.237** explore with lidar + depth + pan-tilt (the user's ask; moved
  ahead of 3.223.A/3.188.B): session_01JbXvZpeChrCvzmrqfSt4Wv. B0 done
  (lidar only 319 low contacts in 110 missions; today's depth veto -54%,
  short of 80%). Decided: no force push (new branch name `-2`); 3.230.B's
  Touches narrowed to its traction files; 3.231.C split into C1 (sim, as
  arm 3) and C2 (Rover); 3.230.C left alone. Doing A, then B. Not on dev.
* **3.188.B** fine-tune on HF Jobs ($10 stop approved):
  session_01JbXvZqV2L1joLsetDKmEnY. Started 14:17.
* **3.147** done on `plan/3.147-verb-outlives-dead-brain-3` (1b738dfe,
  ready passed; FORWARD run-on 24-27 cm -> 0.3-2.5 cm). Push blocked: the
  gate wants 3.225.B claimed (robot/server.py) and the agent classifier
  refuses. Only the user can run `! python tools/plan_section.py claim
  3.225.B` (then push, then release). Worker session_01JbXvZpiMjw5Z4NhXLzmJ65.
* **3.228.C Rover test** (desktop, session_01JbXvZquVz435imNa6Cbpfv, holds
  claim 3.228.C): no run this morning (the user left). Board brain dev
  750d7ea9. Resumes this afternoon: survey + 2 tight-spot trips home, then
  3.232's live check at shutdown. Brief was the scratchpad file
  `brief-rover-2026-11-28.md`.
* Finished, the user may archive: 3.234.A, 3.234.B, 3.234.B2, 3.228.B,
  3.236.A, the docs job, the old paused 3.147 session
  (session_01JbXvZpcr6L4YgDEBeNeJ8U).

## Queue

1. **New bug, next cloud slot:** after a boot, hardware mode hangs in
   `robot/hardware_robot.py` `tty.setraw()` on /dev/ttyTHS1 (TCSAFLUSH
   waits to drain; port at 9600 with echo). Workaround: `stty -F
   /dev/ttyTHS1 115200 raw -echo` before the stack. Fix: open O_NONBLOCK,
   TCSANOW or a flush, plus a test (3.228's file has the finding).
2. 3.237 B after A; then the CLAUDE.md trim (over SOFT 38,036/38,000), the
   pre-push hook fix.
3. 3.223.A skipped: decided (3.231, commanded angles); readback waits on
   the replacement board.
4. Desktop-only: 3.228.C (today), 3.147 D1/D2, 3.234.B4, 3.222, 3.230.A,
   3.231.A, 3.236.A's half 6 and B.
* Leftover origin refs (cloud 403s): plan/3.228-trip-home-fails,
  plan/3.234-mover-prediction, plan/3.236-scan-stamp-speed,
  plan/3.147-verb-outlives-dead-brain(-2), plan/3.237 (pre-rebase), plus
  ~60 claim/* refs: a desktop session deletes them.
* A suggested task card: "Fix two timing-flaky tests seen under load"
  (test_sim_body_process, test_board_http).
