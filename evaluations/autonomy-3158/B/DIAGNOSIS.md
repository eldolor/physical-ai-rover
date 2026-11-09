# 3.158.B rehearsal: why it missed (written 2026-11-09 at 39 rows; 45-row figures added at the end)

Sources: rows.jsonl, each run's explore_brain.log / explore_robot.log / explore_ros log,
brain/explore.py, brain/dock.py, robot/drop_guard.py (at 491c02e4), sim truth geometry.

## One place explains most of it

The east end of the dining room, the aisle between the chairs at cells (30-31, 33) and (30-31, 36-37)
and the wall at x = 33 (box x 7.9-9.75 m, y 9.75-11.45 m).

* Dwell is measured between explore's events, so it is approximate.
* 18,865 of 37,682 s of mission time (50%) were spent there; 12,550 of 20,922 s (60%) in the missed
  and absent finds that ended out_of_budget.
* Almost every mission that enters the dining room sends the same two frontier goals, (9.53, 9.53)
  and (9.38, 11.58), about 3 times each, and they end `stalled`.

## 1. Missions 2 and 18: the return's timeout

Where: both at SLAM pose (9.56, 10.63), heading 51.5-53 deg (the same to the centimetre), cell (31.87, 35.43).

Why (each step from the logs or the geometry):
1. Frontier goal (9.38, 11.58) stalls there: m2 at 1794163546, m18 at 1794230552. So the robot was
   trapped from 233-245 s into each mission, 875 s before the return began (m18: 889 s).
2. The robot's centre is 17.1 cm from the top edge of chair (31, 36); the chassis corner radius is
   17.1 cm. On truth geometry the gap is 0.0 cm at headings 36-48 and 132-144, so it can only swing
   between about 52 and 128 deg (the log: LEFT and RIGHT 90 run between 51.5 and 128.5, every further
   turn refused).
3. nav2 rotates first and is refused: 11,208 "turn left clamped: a corner would come within 1.2cm <
   1.3cm (scan_footprint)" lines in m2's 600 s return, at 20 Hz. 32 "Failed to make progress".
4. Every REVERSE is refused: "rear clearance=0.0cm < min=20cm (drop_unseen_astern)", including the
   return's own back-offs (1794164678, 1794164936), and nav2's backup fails.
5. The escape's creep does not fire: no `creep` event after the trap. `_creep_ends` returns [] unless
   BOTH ends are under the bar. Facing the wall at 52 deg the forward end is about 30 cm from my
   geometry, over the 20 cm bar. (The clearance itself is not logged: inferred.)
6. The return has no escape of its own (only the REVERSE), so it waits out 600 s.

Why only 2 of 39: other missions reached the same spot at headings 240, 261 and 306 and left by a
creep (run 131638 REVERSE creep; run 144904 FORWARD creep). Heading 52 is the bad one.

It is safe, as designed: 0 contacts, the veto is right. It is a dead end the autopilot cannot leave.

## 2. breakfast table, washer, tv console 0 of 3; side table 3 of 3

| Target (missions) | Room entered? | Closest | Sightings | s in the aisle |
|---|---|---|---|---|
| breakfast table (1, 8, 38) | never | 5.55-5.80 m | 0 | 359, 766, 405 |
| washer (12, 18, 24) | never (nor the garage hall) | 6.20-6.28 m | 0 | 356, 938, 427 |
| tv console (21, 26, 35) | 21 yes, by the truth track (see the 45-row notes); 26, 35 never | 7.73-7.88 m | 0 | 595, 903, 965 |
| side table (13, 14, 31) | at 28-29 s, the first room visited | 0.51-0.52 m | found | 0 |

**Never reached those rooms.** Closest is taken at explore's events (sparse). All 15 present misses:
13 never entered the target's room (the nine above but 21, plus 3, 4, 19 kitchen; 16, 22 half bath), 21 did, and they had
no sighting; only mission 30 (bookcase) saw it and did not arrive: approach sent 8.6 s before the
reserve ended it at 1,095 s, at 0.61 m.

**The stand-in is not the cause.** The nine made 1,275-1,613 identity calls each. The stand-in's answers are not logged, but the brain
logged "target not visible" throughout with 0 sightings, and the robot was in other rooms, so the
simulator's report could not have held the target. The same label path found 14 others; `breakfast table`,
`tv console` and `washer` are in the survey's inventory, so the simulator reports those names.
Unproven: no mission tested a yes for those three names, since none got there.

Side table is 3 of 3 because the living room is first on explore's route.

Also: laundry was entered by 0 of 39 missions and pantry by 0 of 7 surveys (all 45 rows: laundry 0 of 45). The inventory shows
46 landmarks carried from the first survey, including those three targets, and no goal uses them
(goal kinds: frontier, view, approach only).

Counterfactual unproven: that removing the aisle's time gets those rooms reached. Missions that left
the aisle reached breakfast or the kitchen at about 500-900 s.

## 3. Surveys 6 of 6 out_of_budget, against 3.70

Same failure as 3.70 S1 (live, `home_first_floor`: out_of_budget at 1800 s, 46 goals, about 39 s each,
26 frontiers open, 95.2% coverage). Here: 1,729-1,772 s, 25-62 frontier goals, 11-21 candidates still
open at the last goal, rooms 7-11 of 13 (laundry and pantry 0 of 7; mission 11 only 7).

The aisle makes it worse, 124-907 s a survey, but is not the whole cause: survey 0 lost only 124 s,
ran 62 goals (28.5 s each) and still ended with 19 candidates. The list does not drain: frontiers
behind furniture never close, and the saved map does not remove them.

## Proposed fixes (none made here), and how to verify

| # | Fix | Owner | Verify |
|---|---|---|---|
| F1 | Escape: when a full round (reverse, both turns) moved nothing, creep forward even with the forward end over the bar | 3.85/3.106's escape | Start at (9.56, 10.63, 52) in `home_first_floor_stairs`, 20 runs: leaves the aisle within 60 s, 0 contacts; `tests/footprint_sweep.py` on truth |
| F2 | The return uses that escape on a stall, and fails early rather than waiting 600 s when the pose has not changed | 3.156 | Same start with the return from there: docked 20 of 20 |
| F3 | Goals that stalled twice stay set aside across missions on a saved map (3.50.D5's keying), or no frontier goal through a gap under the chassis' pivot width | 3.85 / 3.50 | Re-run a subset of the sheet: median aisle seconds per mission from about 450 to under 60; C1 re-scored |
| F4 | Astern floor memory: `_corridor_room` excuses shadowed lines only for direction > 0 (line 333), so clutter beside the corridor reads as unseen. Decide whether the excuse or the driven track covers astern | 3.159 | REVERSE here: 335 refused `drop_unseen_astern`, 101 ran, the rest refused for other reasons; D4's reverse bars must hold (never-seen floor refused 100%) |
| F5 | A find goes first to a carried landmark of that name | 3.70.M1 / 3.91 | The three 0-of-3 targets on the saved map, found rate |
| F6 | A survey ends `exhausted` when every sheet room was entered and remaining frontiers have stalled | 3.70 S1 | C4's 4 of 5 |

F4 only matters while the drop guard is on; it is off on every real body today. F1-F3, F5 and F6 do not depend on it.

Why the floor astern reads unseen at this spot is read from the code, not observed (the guard's
memory is not logged): unproven at cell level.

## All 45 rows (added after the run ended)

* The aisle: 21,623 of 43,507 s (49.7%) of mission time; 14,645 of 25,082 s (58%) in finds that ended
  out_of_budget. Median per mission 453 s; per miss 634 s. (Dwell between explore's events, approximate.)
* Present misses, counted and stretch: 19. By the 2 Hz truth track 16 never entered the target's room. The other three: the bookcase
  (30, 39), which reached 0.59-0.61 m and ran out, and the tv console (21), which entered the family room
  (explore's sparse events, the earlier count's source, missed that).
* REVERSE verbs: 372 refused `drop_unseen_astern`, 122 ran, 34 refused `scan_footprint`, 50 other.
* Missions 39-44 added no new end: both returns that failed are still 2 and 18.
* **The rehearsal ran with the drop guard on; it is off on every real body.** So the REVERSE refusals in the
  trap may not occur the same way on the Rover, and neither may the trap's cost. The trapped pose's corner
  geometry (a pivot refused 1.2 cm from a chair) does not depend on the guard.
