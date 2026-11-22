---
kind: architecture
domain: mission
status: current
verified: 2026-11-22
---

# Mission -- architecture

The mission service is "the brain as a service": the process that owns one
mission at a time, drives its decision loop a tick at a time, and guarantees
that a mission which ends -- for any reason -- leaves the robot stopped. It
does not decide moves (that is the [policy](../policy/ARCHITECTURE.md)
domain) and it does not vet them (that is the safety layer on the robot).
Read this to understand why the brain is shaped the way it is; read the
[engineering spec](../engineering/mission/ENGINEERING.md) for routes,
config keys, timeouts and the tests that pin them.

## Purpose

Without this component the autonomy loop would be a blocking script: nothing
outside it could start it, stop it, or ask what it is doing, and a phone tab
going to sleep would halt the robot. The mission service turns the loop
inside out so that:

- a mission can be started, stopped and inspected over the network, and
  survives the request (and the browser tab) that started it;
- the three ways an autonomous loop goes wrong on a real robot -- it goes
  blind, it hangs, or its process dies with the motors energised -- each have
  a guard that ends in the same place, a stopped robot;
- the brain can run on the robot's onboard computer or on a laptop across
  the room with nothing changing but one address.

Its dependants are the twin (which starts and observes missions), the
operations health check (which reads its liveness), and the recordings
tooling (which stores what a mission saw beside what it decided).

## Components and boundaries

```text
   twin / curl / health check
            |  HTTP/JSON
            v
  +---------------------- brain process ----------------------+
  |  service front     one mission slot, start-time validation,|
  |                    the hung-tick dead-man, drills          |
  |        |                                                   |
  |        v                                                   |
  |  mission runner    lifecycle, outcomes, vision-failure     |
  |                    budget, halt gate, status snapshot      |
  |        |  in-process "frame -> scene" seam                 |
  |        v                                                   |
  |  policy (agent + vision function)   -- policy domain       |
  |        |                                                   |
  |  body client  ---- HTTP ---->  robot server (control-api)  |
  |  world client ---- HTTP ---->  world routes (world)        |
  +------------------------------------------------------------+
```

| Part | Owns | May not |
|---|---|---|
| Service front | the single mission slot, request validation, the per-tick deadline, the drill surface | run decision logic; keep a second mission alive |
| Mission runner | one mission's lifecycle and outcome, the vision-failure budget, the halt gate, the status snapshot | decide a move; compute perception; outlive its mission (one runner, one mission) |
| Halt gate | refusing every movement and camera-pan command once the mission is over | block a stop or a sensor read |
| Brain config reader | the brain's block of the shared config file | import the robot runtime to read it |
| Drills | injecting exactly one fault into an otherwise ordinary mission | make the robot move |

**The boundary that matters most:** the mission package may not import a
robot backend, the simulator, the world factory or the robot server. It
reaches the robot only as an HTTP client, through the body contract
([body](../body/ARCHITECTURE.md)) and the world contract
([world](../world/ARCHITECTURE.md)). The rule is enforced by an automated
test, not by review (the mechanism is in the
[engineering spec](../engineering/mission/ENGINEERING.md)). If the brain can
reach a backend directly, "run the brain on the robot" stops being a
configuration change.

## Decisions

### The brain and the robot are two processes

**Decision.** The brain runs as its own service beside the robot server,
even when both run on the robot, and talks to it over localhost HTTP.

**Rejected.** Running the loop as a task inside the robot server: less code.
It was rejected for three reasons (`PLAN-brain-relocation.md`, "Why not one
process"): a synchronous block in the agent would block the event loop the
robot's watchdog polls on, so the one guard for a dead brain would die with
it; it merges the robot runtime with the decision-maker, the separation the
project has kept since Phase 0; and it loses the property that moving the
brain is a one-address change.

**Trade-off accepted.** A localhost round trip per robot call, against a
loop that spends seconds waiting on vision.

### The loop is driven from outside, one tick at a time

**Decision.** A mission exposes start, tick, stop and status; the service
drives ticks in the background. No decision logic moved when this was built
(B1), and a tick-driven mission was pinned to take the same steps as the old
blocking loop.

**Rejected.** Keeping the blocking loop and wrapping it: it cannot answer
"what are you doing" mid-mission or be stopped between steps.

### Three failsafes, kept separate

**Decision.** Three distinct failures get three distinct guards
(`PLAN-brain-relocation.md` B3, `AGENT-HARNESS.md` section 6):

| Guard | Failure it sees | Where it lives |
|---|---|---|
| B3.1 watchdog | motors energised by a process that then died | the robot (safety domain) |
| B3.2 vision budget | the car is blind: vision calls error or hang | the mission runner |
| B3.3 dead-man | the loop is alive but stuck | the service front |

**Rejected.** Collapsing them into one timeout. The watchdog cannot see a
brain that is alive but stuck (it keeps the event loop healthy while a tick
hangs on a worker thread); neither brain-side guard can see motors left
running by a crash. **Rejected also:** retrying a failed vision call inside
the policy. The failure budget *is* the retry policy, and a silent retry
while the robot keeps moving is the behaviour B3.2 exists to prevent.

### Stopping is enforced at the robot boundary, not only in the loop

**Decision.** The agent drives the robot through a halt gate that refuses
movement and pan commands the moment the mission ends, while always passing
stop and sensor reads. A blocking call on another thread cannot be
interrupted, so the gate is what makes "stop stops the car" true for a tick
already in flight. The residue -- a command dispatched microseconds before
the stop -- is the watchdog's job.

**The gate must be transparent to safety.** Whatever the safety layer needs
from the body to vet a move, the gate passes through. A gate that hides it
makes in-process missions run a weaker safety path than the car, and every
sweep measured through it would then overstate the car's safety
(`PLAN-ros-alignment.md` 3.32 is the record of that defect and its fix).

### One mission per brain; a second start is refused, not queued

**Decision.** The service holds one mission slot. A second start while one
runs is refused with a conflict. A person at the D-pad still outranks the
brain, but that is enforced by the robot (safety domain), and a mission that
loses the robot ends **cancelled** (by the D-pad).

**Rejected.** Queueing or replacing: two loops driving one robot is exactly
the failure the service exists to prevent, and a silent replace hides it.

### Refuse at start what would otherwise fail mid-mission

**Decision.** Everything that can be known before the first move is checked
at start: the policy needs a vision service and an object target; a
requested cloud model or prompt wording must be on the vision service's
published allow-list; a tiered mission must be able to load its local
models. Each is a refusal naming the cause, before a motor turns.

**Rejected.** Discovering these on the first tick, where they are counted as
vision failures, so the mission limps through its budget and dies reporting
"vision unavailable" -- the wrong diagnosis, after a robot has started
moving.

**Trade-off.** An *unreachable* allow-list endpoint is not a refusal: an
unknown value is a caller error, an outage is a soft problem, and refusing
to start over a failed validation call would turn one into the other.

### Outcomes say what happened, not just that it stopped

**Decision** (end states: 3.70, decided by the user 2026-10-11). A mission
ends in one of **four end states**, or in one of the outcomes kept beside
them. The first outcome wins and later stops cannot rewrite it.

| Outcome | Kind | Means | Phone style |
|---|---|---|---|
| **found** | end state | the target seen, at the arrival rule's range, and the cloud said yes; the robot parked beside it | ok |
| **exhausted** | end state | nowhere reachable left to look: no frontier, no unseen floor, no labelled room left to enter | warn |
| **out_of_budget** | end state | the mission's time limit or the battery floor was reached (or the step backstop derived from the time limit, or a `vision` mission's cap on paid calls); the robot stops and reports where it looked | warn |
| **cancelled** | end state | a person ended it from the phone: Stop (`cancelled_by: stop`) or a D-pad or tap-to-goal takeover (`cancelled_by: dpad`); it reports where it looked. A refusal naming another AUTONOMOUS driver is no person's doing and ends `failed` | neutral |
| room_reached | goal | a room-only mission reached its room | ok |
| blocked | fault | the safety layer kept refusing forward | warn |
| arrived_unconfirmed | fault | parked at an arrival the cloud never answered (never shown as found) | warn |
| failed | fault | B3.2's vision budget, B3.3's tick deadline, or a body fault | error |

`searched`, `max_steps`, `stopped` and `preempted` -- the names before 3.70
-- are no longer written. A record that carries one (a mission log, a
metrics row, a run file under `evaluations/`) is READ as `exhausted`,
`out_of_budget`, `cancelled` (by `stop`) and `cancelled` (by `dpad`), never
rewritten.

**The budget is time and battery, not steps** (3.70 E4, conflict 3). A step
was one decision on one frame, so 120 steps was about a minute of searching.
The step count stays only as a runaway backstop derived from the time limit
(limit / 0.5 s x 2), so it can fire only if the clock check fails.

Four are deliberately not "failed":

- **cancelled** -- a person ended it, with Stop or the D-pad. Filing a
  normal intervention beside a dead cloud link would invite a retry where
  retrying is wrong. Every mission's end is also one line in a local log
  (3.99): whether a person stepped in, and what the mission was doing then,
  so interventions per hour of autonomy can be counted and, once the Rover
  has data, learned from. A goal pick that answers explore's question is
  counted beside it, never as an intervention.
- **out_of_budget** -- it was still trying; the limit is the operator's.
- **blocked** -- the policy kept asking to drive forward and the safety
  layer kept refusing (R1b, `PLAN-ros-alignment.md` 3.4). Nothing broke;
  going around is route planning, and nav2 reports an unreachable goal the
  same way.
- **arrived_unconfirmed** -- the vision budget ran out while the robot was
  parked at an arrival whose identity the cloud never answered (3.47). Not
  "found" either: an arrival the local tier could not settle needs a cloud
  yes to be "found" ([policy](../policy/ARCHITECTURE.md), "The cloud
  confirms identity at arrival"). Since 3.80 the local VLM on the board
  answers first, and its yes makes "found" without the cloud, except for an
  arrival led by its own box, which only the cloud may confirm (3.80.O). In
  the simulator the frame's geometry answers (3.98).

### A survey is a mission with no target

**Decision** (3.70, decided by the user 2026-10-11). A survey -- started
from the phone, for a new house or after big changes -- is a mission whose
job is `survey`: explore's frontier loop with no target. It maps the house
and names its rooms, and ends **exhausted** when nothing reachable is left
(or out of budget, or cancelled). It never ends found and never asks the
cloud to confirm a target, because there is none. It is the same brain
driver as any mission (arbitration unchanged), with its own time limit.

**A room is asked about once, not once a frame.** On the car a room's name
is a paid cloud call, so a survey asks only when the robot stands in space
no earlier answer covers, and at most a capped number of times per survey;
reaching the cap leaves the rest of the house unlabelled and never ends the
survey. Which space an answer covers decides both the count of calls and
where labels land. The default reaches as far as walking distance, with a
doorway counting as far: the robot is asked again every couple of metres
and through every doorway, and an answer stops at the doorway (3.70 S2,
measured in the sim against one ask per room of the map's shape, which
costs a third of the calls and labels far worse). Rooms with no wall
between them (open plans) cannot be told apart by the map's shape, so
labels there land by distance at best.

### A mission that did not find says where it looked

**Decision** (3.70 E4 and E5, with the end states above). A mission that
ends **exhausted**, **out_of_budget** or **cancelled** carries a report of
where it looked: the rooms it entered, the goal points it reached, the map
it ran on, and how much floor that map holds. Without it "not found" would
tell a person nothing about where to look themselves or whether to run it
again.

**How much floor is an estimate on the robot's own map, and says so.** The
brain cannot know the house's true floor. Its share is the floor the map
knows, over that plus the unknown patches the map has walled in and that
touch known floor. Unknown space that reaches the map's edge counts as
outside the house; a patch walled in by obstacles alone is the inside of a
sofa, which no robot reaches. Floor inside a person's no-go zones counts
neither way (3.161).

**The report is built after the stop, never in its way.** Reading the map
can be a network call, and a person's Stop must not wait on it (3.70's bug
pass found the map read running on the path that answers Stop). The robot
is stopped first; the report follows, and a reader who asks in the moment
between gets none yet and asks again.

**Rejected: showing the share as "coverage".** 3.70 set the bar at within
2 points of the simulator's truth, and it FAILED: at the end of a finished
search the estimate was within 1.5 points, but part way through it read 20
to 77 points high, because the estimate cannot count floor the map has not
reached yet -- and "part way" is exactly when an out-of-budget or cancelled
mission reports. So the phone shows square metres mapped, a figure that is
true whenever it is read, and the share stays in the report for a finished
search. Whether to keep the share at all was left open for the user in
3.70's record.

### What a mission saw is kept on the robot for the person to check

**Decision** (3.162, asked by the user 2026-11-06, from a robot vacuum's
app that shows a photo of each obstacle on its map: people trust what they
can check). Every mission's end keeps, on the robot, what it saw: the frame
its arrival's identity check judged, a thumbnail of each thing it placed on
the map with its room and position, and the rooms. The phone shows the
judged frame on the ended mission's card, one tap away, and the rest as a
list. Before it the twin showed only an end state, and the object inventory
every mission already kept was read by nobody. It is also the instrument
the autonomy rehearsal is judged with: the user checks each **found**
against the frame shown here (3.158).

**Why it is shaped this way.**

- **The frame shown is the frame judged, byte for byte.** A re-rendered or
  later frame would show what the robot sees now, not what it called
  found. The stored frame carries the fingerprint of the one the identity
  check was sent (accepted at 24 of 24 in 3.162.C, 12 **found** and 12
  **arrived_unconfirmed**).
- **Local only.** Frames and thumbnails leave the robot only to the
  person's own phone, through the brain's authenticated route. They never
  go to a bucket, a log or the repository. This keeps 3.46's decision (the
  uploaded inventory carries labels and positions, no image) and the
  privacy rule in [policy](../policy/ARCHITECTURE.md).
- **Bounded, and whole or absent.** A mission's pictures are capped (at most
  1 MB), the store keeps the newest 50 missions (at most 50 MB) and deletes
  the oldest first, and a person can delete one from the phone, every file
  with it. A report is written whole or not at all, so no reader sees half
  of one.
- **Never in the mission's way.** It is written on a thread of its own,
  once, at the mission's real end: after the drive home, and not at a
  recharge that resumes. A save that fails is logged and never changes an
  outcome (the rule 3.99's log set). It reads and shows; nothing it writes
  reaches the robot.
- **It does not rewrite history.** A late confirmation (3.53) leaves the
  report saying the arrival could not be confirmed, as the outcome does.

**Rejected.** Uploading the frames beside the inventory: nothing new leaves
the house unless the user says so (3.162's scope). Showing only the end
state, as before: a person cannot tell a right **found** from a wrong one
without the picture.

**What it cannot show yet, recorded.** On the simulator's frames the list
names every object. On real frames it holds the target only: no namer for
camera frames passed its bar (3.46, closed by the user 2026-10-10). A
listed thing's room is the room the robot stood in when it first saw it, so
a thing seen through a doorway is filed under the wrong room: 81 of 123
right against a bar of 95%, FAILED (3.162.C); rooms a person named are
looked up by the thing's own position and do not have this error. A policy
with no identity check has no judged frame and shows the last frame it
took. The judged frame carries no box around the target yet.

### A mission's end goes home

**Decision** (3.156, asked by the user 2026-11-06 as part of full autonomy).
After an end that leaves the robot in the brain's hands, the brain drives
it back to the dock before the mission is over. Two reasons: a robot left
where it stopped needs a person to fetch it, and the saved map assumes the
robot starts on the dock (3.50.D: a reload localises from there), so a
robot that does not come back also breaks the next mission's start.

**The outcome is decided first, and the trip home never changes it.** The
return is a phase between the end's decision and the end itself, with its
own record beside the outcome: docked, failed, cancelled or skipped, each
with its reason. A **found** whose trip home failed is still **found**.

**Which ends go home.** Found, a room reached, exhausted, out of budget, an
unconfirmed arrival, and blocked (routing is the planner's, so a
straight-line wedge is no reason to stay). Not **cancelled**: the person
has the robot. Not **failed**: a fault wants a person. Not a battery at
its reserve level or below: the robot server refuses autonomy there, and
the brain does not ask.

**The route is the planner's; only the last stretch is the brain's.** The
trip is a navigation goal to a point just short of the dock, then a turn, a
short creep on the same guarded moves a mission uses, and a final turn to
the dock's heading. With no planner to ask, the return is skipped and says
so. **Rejected:** driving home in straight lines on the brain's own moves
(3.156 D2): going around things is route planning, and the brain has none.

**Safety and who-drives are unchanged.** The return is the brain driving,
so every move and goal is vetted and arbitrated as a mission's are: a
person's Stop or D-pad ends the return and leaves the robot where it is,
and the battery's reserve refuses it. It runs inside the tick loop, so the
three failsafes cover it, and the halt gate shuts after it. 3.156.B changed
nothing in the safety layer or the arbitration (its criterion C8), and
live a Stop and a D-pad each ended the return 10 times of 10.

**The dock is a known pose, never a guess.** On a saved map that reloaded
and matched, it is the map's own dock: the same point for every mission on
that map. Otherwise it is where the mission started, marked as such so
that a report never calls it verified. With neither, the return is skipped.
When the pose was unusable at the start, the dock is read again while the
mission runs (3.194, after a live find that found its target and then did
not go home). A saved map's dock is taken whenever it is read. A start
pose is taken late only while the wheels say the robot has not moved,
because a start pose read after a move is not the dock.

**The search ends early enough to get home.** A mission ends
**out_of_budget** when the time or the battery left is less than the trip
home is predicted to cost, with a margin. The prediction comes from the
path home on the brain's own map and this mission's own speed and drain.
**Rejected:** a fixed reserve (3.156's scope: "a budget from the distance
home, not a fixed number"). **Trade-off, measured and recorded as failed:**
ending early costs searching. In 3.156.B's paired sweep six missions ended
at the reserve where the other arm went on, finds were 18 against 19 and a
survey's rooms 259 against 263 (C6 FAILED). And a battery that drains fast
from a low start passes its reserve before the drain can be estimated (C5
FAILED at the two faster rates). Both are open for the Rover's step.

**Recharge and resume: built, off until the Rover** (3.156.D, the user's
call 2026-11-06, from a vacuum's "recharge and resume"). When the battery
is what ended the search, the mission may wait on the dock, charge, and
carry on with what it kept: the places visited and refused, the rooms, the
memory.

- Only a battery end charges. Time does not come back, so a time-caused
  end never does.
- The end already decided is the fallback: anything that stops the charge
  (a failed return, a Stop, no rise in voltage, the overall limit) ends
  the mission with it.
- The mission's time limit pauses only while charging, under a separate
  limit on the whole, and a mission charges a bounded number of times.
- A person still wins while nothing moves: the brain keeps asking the robot
  server for a harmless command, and a Stop or a D-pad refuses it.
- **Rejected:** a separate drive back to where the search stopped. A
  frontier search has no row to rejoin as a vacuum has, and the leg is
  wasted when that point was a dead end; the next goal is the drive back.

It is off because nothing yet sees a charger and its voltage levels are
placeholders (3.137.E). In the simulator 45 of 45 charges resumed, but the
progress bar FAILED (rooms 78% and finds 79% of a mission with no drain),
one live return ended unaligned with the cause not established, and a
D-pad tap that lapses between the brain's probes is seen only once the
robot has moved (3.156.D's known limit).

### An unconfirmed arrival is asked again when the cloud is back

**Decision** (3.53, approved by the user 2026-10-09). After an
`arrived_unconfirmed` ending the brain keeps the frame the arrival was
judged on, probes the cloud's free health route, and when it answers asks
the identity question ONCE, recording the answer beside the outcome
(`late_confirmation`), never instead of it. The outcome is not rewritten:
the first outcome wins, and a late yes is evidence for a person to read,
not a `found` the mission earned on its own terms. It is bounded (a probe
every 15 s for ten minutes, two paid attempts at most), it never moves or
stops the robot (the mission is over; a stop now would halt whoever drives
next), a new mission drops it, and a brain restart loses it. A Stop after
the ending does not drop it: it moves nothing, and the phone sends Stop on
every close.

**Rejected:** rewriting the outcome to `found` on a late yes (breaks "first
outcome wins", and the robot may have been moved since); keeping the
mission running until the cloud answers (holds the robot and the mission
slot for an unbounded time); probing with a paid `/navigate` call (a free
route exists).

### A mission starts with the camera centred

**Decision.** The first tick centres the camera through the halt gate,
before the first decision, uncounted as a step (3.20). A mission that ended
mid-peek otherwise left the next mission's first frame, depth grid and scene
cast sideways.

**Rejected.** Centring at the end of the previous mission: by then the brain
may no longer hold authority over the robot.

### The world is advisory; the body is load-bearing

**Decision.** If the world client cannot be built or cannot answer, the
mission proceeds without a map (the frontier policy falls back to its
right-hand rule). If the body cannot answer, the step fails and the mission
ends **failed** with the robot stopped. Odometry for pacing is attached to
each frame on a best-effort basis for the same reason.

### Offline is a mode, not a failure

**Decision** (3.49, asked by the user 2026-10-09). With the cloud
unreachable -- Wi-Fi down, the tunnel gone, a refused or hanging connection
-- these keep working, each pinned by a test with every socket connect
refused (mission start with an unreachable model allow-list is pinned only
by 3.49's live run, criterion 7):

- the safety veto and the watchdog: the robot server imports no cloud
  client;
- the tiered policy's local search, steering and arrival, ending **found**
  when the local VLM (3.80) or, in the simulator, geometry (3.98) confirms
  the target, and **arrived_unconfirmed** rather than **failed** when only
  the cloud could;
- the twin, served by the robot server on the LAN, loading nothing from the
  internet;
- mission start (an unreachable model allow-list is tolerated) and
  `/health`.

On the car, offline means the tiered policy: `frontier` and `explore` never
call the cloud, but their vision step is the rule-based one, which reads the
simulator. The `vision` policy is cloud-only and ends **failed** at once.

**Rejected.** Falling back to the rule-based explorer when the cloud drops
(`PLAN-onboard-perception.md` 2.5): on the car it has no detector, and the
tiered policy already searches without the cloud.

**Trade-off.** Offline, a mission ends **found** only on a local answer: on
the board the local VLM's yes, in the simulator geometry. Without the local
VLM (off the board, or killed for memory), or on an arrival only the cloud
may confirm, identity stays the cloud's question. A hanging cloud costs up to the failure budget times the
confirmation's deadline of parked time at arrival: about 25 s at the
confirmation's own 8 s deadline (3.56, the user's choice), down from 60 s
when the confirmation shared the 20 s vision timeout. 8 s sits above the slowest
cloud call recorded (6.78 s of 98); a slower but live cloud costs a retry,
never a wrong **found**.

**The operator sees the cloud** (3.56, asked by the user 2026-10-09). While
a mission that uses the cloud runs, the brain looks at the vision service's
free `/health` and reports `status.cloud`: unknown, reachable, or
unreachable since a time; the twin shows it. One thing probes at a time
(after a mission parks, 3.53's late check does), it never touches the robot
and it is not a failsafe -- B3.2 still decides. "Reachable" means the
service answers; Bedrock behind it can still fail.

### Fault drills exercise the real guards

**Decision.** A mission may be started with one named fault (vision errors,
vision hangs, the loop hangs) on shortened deadlines, so B3.2 and B3.3 can be
watched firing. With no fault the drill path builds the same object as the
production path. Drills are fail-safe by construction -- each can only end a
mission with the robot stopped -- and can be switched off for a brain
reachable beyond the LAN.

### Hand-rolled, not an agent framework

**Decision.** The loop is project code. The hard parts -- a stop that beats
an in-flight action, a veto in another process, a dead-man for a live but
stuck loop -- are physical, and no orchestration framework has opinions
about them (`AGENT-HARNESS.md` section 11). If the policy ever grows real
tools, a tool runner belongs inside the vision function, below the runner.

## Contracts

| With | Direction | Category | Ownership |
|---|---|---|---|
| Twin, scripts | they call the brain | HTTP/JSON: start, stop, status | the brain owns mission state; callers only read it or ask for a transition |
| Robot body ([body](../body/ARCHITECTURE.md), [control-api](../control-api/ARCHITECTURE.md)) | brain calls robot | HTTP/JSON through the body client, naming itself as the brain driver | the robot owns all body state and the final veto; the brain never assumes a move happened |
| World ([world](../world/ARCHITECTURE.md)) | brain calls world | HTTP/JSON through the world client | the world owns pose and map; the mission treats them as advisory |
| Policy ([policy](../policy/ARCHITECTURE.md)) | runner calls agent; agent calls the vision function | in-process: a frame goes in, a scene comes out | the policy owns the decision; the runner owns the budget, timeout and outcome |
| Cloud vision ([cloud-vision](../cloud-vision/ARCHITECTURE.md)) | brain calls service | HTTP/JSON, at start (allow-lists) and through the policy (decisions) | the vision service owns the model and wording allow-lists; the brain never keeps a copy |
| Operations | health check calls brain | HTTP/JSON | the brain reports liveness facts; operations alone turns them into a verdict |
| Recordings | twin calls brain | HTTP/JSON routes the brain mounts but does not own | the recordings domain owns the routes and storage |

## Failure modes and resilience targets

| Failure | What the component does | Target |
|---|---|---|
| A vision call errors or hangs | stops the robot on that step and counts it; a success resets the count | every blind step stops the car; a short run of consecutive failures ends the mission **failed** with the robot stopped, or **arrived_unconfirmed** if the run is an arrival's identity check that never got an answer |
| A tick never returns | the service abandons the tick thread, stops the robot and ends the mission | a stuck loop ends **failed** within one tick deadline, robot stopped, while status and health keep answering |
| The brain process dies mid-move | nothing here can act | the robot's watchdog stops the motors within one watchdog period (safety domain) |
| Stop lands while a tick is in flight | the halt gate refuses that tick's move or pan | after a mission ends, no further movement command from it reaches the robot |
| A person takes the robot | the mission ends **cancelled** (`cancelled_by: dpad`); it stops the car, not retries | a preempted brain never re-takes the robot |
| The time limit passes or the battery reads under its floor | the mission ends **out_of_budget**, the robot stopped, with a report of where it looked | ends within one tick of the limit; two low readings in a row that are distinct samples of the board (one is a motor's sag; the car's board measures only every 10 s, 3.76 M2.7) |
| The safety layer keeps refusing forward | the mission ends **blocked** after a short run of refusals | no mission spends its budget pushing into a wall |
| A body read or move raises anything else | the mission ends **failed** with the robot stopped | the brain never loops blind on a dead robot link |
| World unreachable | missions run without a map | a mapper outage costs exploration efficiency, never a mission |
| Bad model, wording, missing models or vision service | refused at start with the cause | no misconfiguration is discovered after a motor turns |
| A late async cloud answer arrives after the mission ends | the policy is closed on finish, so the answer is dropped | no decision from an ended mission is applied |
| The stop command itself fails | logged and swallowed | a failsafe never raises |
| The trip home cannot start, fails or is interrupted | the return is recorded as skipped, failed or cancelled with its reason; the robot is stopped where it is | the outcome decided before the trip is never changed by it, and the brain never drives home without the route planner |
| The dock is unknown at the start | read again while the mission runs; a start pose read after the robot moved is never used | no return to a guessed dock |
| The map is unusable when the where-it-looked report is built | the report carries no floor figures | a report never invents coverage |
| Saving what the mission saw fails | logged; the mission's outcome and its stop are untouched | a report is whole or absent, and never fails a mission |
| Brain restart | the mission is lost; the robot keeps its own state | accepted gap, see below |

## Open questions

- **Boot-time start on the robot (B5).** On the Jetson, units start the
  robot server, the brain and the tunnel at boot and restart them on failure
  (the brain's home moved from a Pi to the Jetson on 2026-09-19); none starts
  the ROS container yet. Settled when a mission starts from a phone after a
  reboot with no laptop on the network, on the Rover, decided by the user on
  hardware day.
- **The dock on the car (3.156.C, on the Rover).** "Docked" there is
  charging seen on the board, not a pose. A physical dock is an obstacle to
  the safety layer, so the last centimetres may be vetoed; whether the
  car's dock needs a reversed entry or anything else is a safety decision
  with its own criteria, not made yet. No path around the safety layer is
  on the table (3.156 D4).
- **Mission persistence.** A brain restart loses the mission. Worth revisiting
  only if a field run shows restarts happen.
- **Chaos and soak (S7).** Added latency, dropped requests and a 1000-step run
  are planned, not run.
