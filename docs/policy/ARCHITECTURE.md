---
kind: architecture
domain: policy
status: current
verified: 2026-11-22
---

# Policy -- architecture

The policy is what decides each move. Every tick, the mission runner hands it
one camera frame; it returns one action out of a small fixed set, which the
safety layer then vets. Three policies exist -- a free rule-based explorer, a
cloud vision policy, and a tiered policy that asks the cloud only on events
-- plus the arrival rule that lets a mission end `found`: the lidar judges
the distance, and one cloud call on the arrival frame confirms the identity.
Read this for why decisions are split the way they are; the
[engineering spec](../engineering/policy/ENGINEERING.md) has the constants,
precedence code, and the sweeps that pin them.

## Purpose

A mission needs something that turns "what the robot sees and where it is"
into "what to do next", in a way that can be swapped without touching the
mission lifecycle, the safety layer or the body. The policy domain owns that
decision and nothing else. It depends on perception (what is in view), the
world (where the robot is), the body's sensors (how much room there is), and
the cloud vision service (what the target is, when asked). The mission
service depends on it for every step, and the twin's readouts (turns, tier
counters, arrival) are its outputs.

## Components and boundaries

```text
            frame (pixels, pan, odometry, sim detections)
                         |
                         v
  +----------------- vision function -----------------+
  |  frontier: sensed scene (depth/scan clearance)    |
  |  vision:   one cloud navigation call -> scene     |
  |  tiered:   local perception every frame,          |
  |            cloud only on a trigger -> scene       |
  +---------------------------------------------------+
                         |  one scene schema
                         v
  arrival review (local perception + lidar; then one cloud call confirms
  identity) -> may rewrite to STOP/found
                         |
                         v
  decide: frontier preference | trust the model | stuck-breaker
                         |
                         v
           safety layer (robot side) -> body
```

| Part | Owns | Must not |
|---|---|---|
| Constrained agent | the allowed action set, the capture -> scene -> decide -> vet -> execute step, a stuck-breaker | bypass the safety layer; trust a model's distance claim over a sensor |
| Mission agent | recording every step into mission memory, arrival review, room backfill from the cloud's room guess, sighting poses from the world | read simulator state; import a world backend |
| Frontier explorer | rule-based coverage: peek, prefer unvisited directions | grow; it is kept, not extended (`PLAN-sim-hardening.md` 2.2) |
| Explore policy (`explore`) | a search over SLAM's map: frontier, view and approach goals sent to nav2, the retry rule, the escape when wedged; a find's first goals to places earlier missions on the saved map saw the target, tried once each (recall, 3.172.C: a find used nothing earlier missions had seen); in a survey, a goal into a room the usual goal clearance reaches no place in (small-room, 3.173.B: such rooms were never entered) | send a verb while a goal is live; choose a goal the robot could not drive away from (3.45) |
| Vision agent | trusting the model's action unless the mission is complete | peek (a pan costs a real move and buys nothing a photograph lacks) |
| Cloud vision step | one frame -> one cloud navigation answer -> the scene schema | retry; the mission's failure budget is the retry policy |
| Tier (trigger discipline) | when to spend a cloud call, and what to do on free frames | become a planner or a reactive goal executor |
| Arrival check | "arrived" from detection + bearing + lidar range | read the detector's own distance, or any simulator fact; call the cloud |
| Identity confirmation (the mission agent) | asking the cloud, once per arrival, whether the arrival frame shows the target; "found" only on its yes (with a local VLM on board that tier is asked first, and in the simulator the geometry answers: "The cloud confirms identity at arrival", below) | end a mission on a judgement it did not ask the cloud for |
| Goal pose | holding an anchored sighting across rotation | decide anything |
| Mission memory, room matching | rooms visited/searched, sightings, action history | drive control flow beyond completion |

The brain-wide rule applies with full force: nothing here imports a robot or
world backend or the simulator. Simulator-provided detections reach the tier
only as data in the frame (see [perception](../perception/ARCHITECTURE.md)).

## Decisions

### A constrained action set, one short move at a time

**Decision.** The policy picks one of seven actions (forward, left, right,
reverse, stop, look left, look right; the explorer's look-around also
re-centres the camera) each tick, every move short and
re-evaluated next tick. The safety layer re-checks every move against
sensors and can veto it.

**Rejected.** Longer commands or free-form plans from the model: errors here
are physical and not cheaply reversible, which argues for the least agentic
thing that does the job (`AGENT-HARNESS.md` section 11).
**Trade-off.** More ticks, and turn/forward alternation is visible by design.

### One scene schema behind one seam

**Decision.** Every policy produces the same scene -- obstacles, free space,
the important objects, the safest direction -- from a single "frame in, scene
out" function. Swapping the rule-based policy for the model changes nothing
downstream. A target counts as found only when it appears in the scene's
important objects, which the cloud policy fills only on **target reached**,
not on first sight -- otherwise a mission would end in a doorway across the
room from the backpack (`brain/navigate.py`'s note).

**Which paths can end a mission `found`, as the code behaves.** Under the
vision policy, the cloud's target-reached answer does. Under the tiered
policy with the cloud call dispatched asynchronously (the shipped mode), it
cannot: a landed cloud answer only becomes the held goal, and the scene every
frame returns is the local one, which never names the target. There, only
the arrival rule (below) ends a tiered mission `found`. With a blocking cloud
call, the trigger frame's cloud answer can end it as well. This, rather than
the arrival rule alone, is why a tiered phone walk (no range sensor) that
reaches its target ends at the step budget (P7e). That a landed
target-reached does not count was decided by the user on 2026-10-02 (see
"The cloud confirms identity at arrival; the lidar decides distance").

### The cloud-driven policies are the hardware path; the explorer is a test tool

**Decision** (`PLAN-sim-hardening.md` Q1, 2.2). The robot is vision-driven.
The frontier explorer is kept because it is free, deterministic and the
fastest way to exercise the safety layer and mission memory, and since the
ROS alignment it reads only robot sensors and the world's pose, never grid
cells (`PLAN-ros-alignment.md` 3.2). Without a world that can localise it
degrades to the right-hand rule rather than failing. nav2's frontier
exploration is its replacement on the car.

### Deliberation is event-driven, never periodic

**Decision** (`PLAN-onboard-perception.md` 2.4). The tier runs local
perception on every frame for free and calls the cloud only on: mission
start, a confirmed candidate sighting, a run of confirmed absence (cold
search), and a staleness floor so a robot that can see its target is not
silenced by things going well. An edge must hold for consecutive frames
before it is believed.

**Rejected.** A cloud call per frame (the vision policy's cost profile), and
firing on raw edges: 6.1 measured that about 40% of a naive trigger count is
field flicker; hysteresis took the saving from 2.8x to 4.1x. The three other
triggers in 2.4 (goal achieved, goal impossible, room change) need tiers that
do not exist and are named as unavailable rather than faked.

**With a local VLM on board, a question goes to it first** (3.80.D, approved
by the user 2026-10-14; it is off unless a local VLM is configured). The
order is strict: perception, then the local VLM, then the cloud. The four
events above are then asked of the local VLM, which costs no money and sends
no frame out of the house, and two more questions exist only for it: a
candidate that perception rates just under its gate, and a target outside
the detector's vocabulary that has stayed out of view. A frame's question
reaches the cloud in three cases only:

- **The local VLM did not answer at all** (its service is away). The cloud
  is asked about the same frame. A local VLM found away is held as if none
  were configured, and probed, until it is back (3.152).
- **It answered, but could not decide, a set number of times in a row.**
  The next question is the cloud's.
- **The tiers disagree** (3.91.C, the user 2026-10-23/24): the local VLM
  said no about a candidate perception still passes, in a view where its no
  cannot be trusted, or said yes about a candidate the cloud's no still
  holds. The cloud is asked once for that candidate. 3.92 measured why this
  and not the local VLM's own doubt: a learned router of its confidence
  reached AUROC 0.70 against a bar of 0.80, and the plain rule "ask when
  the tiers disagree" caught as many errors while flagging half as many
  frames.

Two limits keep these from becoming a cloud call per frame. A question the
local VLM asks at its own, faster pace reaches the cloud only once the
cloud's own bar for that event is met (3.80.P). And while explore closes in
on a target only the local VLM has seen, the local VLM is asked on every
frame it is free, and that question never goes to the cloud (3.80.O). The
mission's cap on paid calls stands over all of them.

**Scope of the hysteresis.** It gates *when the cloud is called*, and nothing
else. It does not gate steering: the next decision makes that explicit.

### Privacy is a reason for the trigger discipline

**Decision** (the user, 2026-10-09; `PLAN-ros-alignment.md` 3.52). The robot
photographs the inside of a home, so **every frame that does not go to the
cloud is a frame of the house that never leaves it.** That is a reason for
the event-driven tier in its own right, beside cost and latency: a change that
sends more frames out (a new trigger, a lower staleness floor, a per-step
policy on the car) has to argue against it, not only against the bill.

What leaves the house today, traced to the code:

| What | Where it goes | When |
|---|---|---|
| A camera frame, the target string, searched rooms | the cloud `/navigate` route (Bedrock), from `brain/navigate.py` | tiered: only on a trigger (mission start, candidate sighting, cold search, staleness, arrival confirmation; after an arrival the cloud could not be reached for, that one frame is asked about once more when the cloud is back, 3.53). With a local VLM configured those questions go to it first, on the robot, and a frame leaves only when it did not answer, could not decide, or disagrees with another tier ("Deliberation is event-driven", above; 3.80.D, 3.91.C). The `vision` policy: every step |
| A phone frame | `/navigate` or `/guidance`, straight from the twin's Guide tab | every guidance step |
| A camera frame, for the name of the room it shows (3.70 S2, 3.116.B) | the cloud vision service's room question | only when a survey's, or a find's, room labels are set to ask the cloud (both off by default); a capped number of asks a mission |
| Recorded walk frames and answers | the brain's walk store: local disk, or the private recordings bucket when the walk store is set to it | only while a walk is being recorded |
| Frames of a recorded walk, sent again | the cloud `/navigate` route (a replay), or the model service directly (label proposals) | only when a person replays a recorded walk under another model or wording, or asks for label proposals ([recordings](../recordings/ARCHITECTURE.md)) |
| The object inventory (3.46): labels and positions of what that mission saw itself, no pixels (since 3.50.D5 a mission also carries landmarks from earlier missions on the same saved map, with the map's and the mission's ids; those stay in the robot's local file, the user's call 2026-10-31) | the recordings bucket, under its inventory prefix, only when the deployment names a bucket for it | at mission end |
| A metrics row: outcome, counts, timings, config; no pixels | the metrics route, only when the deployment names one | at mission end |
| The target's words, no image (3.190) | the cloud vision service's room-prior question | once, at a find's start, only when that question is switched on and can act (see "A room prior for a target the shipped prior lacks") |
| The frame a mission's arrival was judged on, and thumbnails of what it saw (3.162) | the person's own phone, through the brain's authenticated route; kept on the robot's disk, never in a bucket | when the person opens the report ([mission](../mission/ARCHITECTURE.md), "What a mission saw") |

Nothing else sends pixels. `brain/vision.py`'s direct Anthropic call is
reached only from a manual test.

**Rejected, for now: blurring people before a frame leaves.** Measured in
3.52 on the shipped detectors and failed: at floor height the people the
corpus holds (one person lying on a sofa, 3-6 m off) were found on at most 7
of 43 frames, while the "person" boxes landed on furniture on 21% of
person-free frames, once squarely on the search target, which the blur then
hid from the cloud. A blur that misses people and erases targets protects
nothing and breaks the search. Revisit with a corpus of people as a floor
robot sees them and a detector that finds them; the section names the walk
that would close the gap.

### Arbitration split by question, and whoever sees the target steers

**Decision** (1.11, and Phase G of 2026-09-12). Identity belongs to the cloud
model; bearing to the local detector; range to the lidar; collision to the
safety layer, which never votes. On a frame where local perception has a
measured bearing, that bearing steers -- ahead of a cloud direction several
seconds old. The full precedence on a free frame is: a bearing measured this
frame, then the local VLM's located yes (3.80.O), then a dead-reckoned
bearing to an anchored sighting (built, off), then the bearing of a
candidate perception rates just under its gate (3.80.F), then the last held
goal, then a search turn.

**Rejected.** Letting the cloud keep both identity and direction: on the rig
walk that motivated the change, the robot turned right 28 times and forward
4 on 37 frames where the target was locally detected.

**Trade-off, stated as the code behaves.** A single detected frame steers --
the consecutive-frame hysteresis gates only the cloud triggers, not steering.
For **steering**, the only bound is the local verifier's per-frame
probability gate: a wrong object steers the robot on every frame it passes
that gate. The cloud's identity does not stop it. With the cloud call
dispatched asynchronously (the shipped mode) the cloud's answer only becomes
the held goal, which a local sighting outranks, so a cloud "not visible"
never overrides a frame where the wrong object is detected. With a blocking
call the cloud's answer decides the frame the call was made on -- unless the
arrival review, which runs after the vision step, rewrites that same frame
to a stop and `found` (it overrides the trigger's direction, never its
identity: `found` still needs the arrival confirmation's yes). For **ending `found`**, the
bound is the arrival rule's conditions on top of the gate (consecutive
frames, centred, a lidar range within the radius, one surface), all of them
local: a wrong object that keeps passing the gate can be driven to and
reported `found` -- until 2026-10-02, when the user decided, and the same
day it was built, that a cloud identity check closes the `found` half of
this. Steering stays as described (see "The cloud confirms identity at
arrival; the lidar decides distance"). The corroboration verdict
([perception](../perception/ARCHITECTURE.md)) still measures the exposure and
enforces nothing.

**Amended 2026-10-23 (3.91.C, the user's choice): a "no" about a candidate
is remembered.** The steering half of the trade-off above had a measured
cost: on the 2026-10-21 phone walk a correct cloud "not the target" was
overridden on the next frame by a false local sighting of a lookalike, about
nine times, and with the local VLM first perception still steered at a bag
it had refused on 54 of 54 frames. Now an identity "no" -- the local VLM's,
where its box is a fair view of the object, or the cloud's -- is evidence
about that candidate, not a passing opinion: the candidate is neither
steered at nor asked about again until it changes (the body moved or turned;
on a body without odometry, the candidate was lost and found again, or a
set time passed, and then it is asked about before it is steered at again)
or a later answer says yes. The local VLM's "no" where it cannot tell (the
far view, fabric only) is not remembered; it asks the cloud, once a
candidate -- escalation on the tiers' disagreement, which 3.92 found catches
as many errors as a learned router.

**The tiers' order (the user, 2026-10-24).** Each tier is more capable than
the one below it -- YOLOE + CLIP < the local VLM < the cloud VLM -- and its
answer about a candidate is never undone by a lower tier's: only a cloud
"yes" lifts the cloud's "no", a cloud "yes" stands against a local "no",
and where the two VLMs disagree, or the local one cannot tell, the cloud is
asked. Perception acts alone only where neither VLM has spoken about the
candidate. Only the robot's own safety layer overrides every tier. The
order is a hard rule beside the behaviour trees, not a node in them, so no
reordered or tuned tree can break it.
**Rejected:** leaving the steer to perception and fixing only the paid
re-asks (the wrong bag is still approached); a learned router (3.92: AUROC
0.70). **Trade-off:** a true target refused once by mistake is not
approached until the robot moves or the hold ends; arrival's own checks are
unchanged.

### Turns are sized; search turns are smaller than the field of view

**Decision** (R1, R1b, R1c -- `PLAN-ros-alignment.md` 3.3, 3.5, 3.6b). A turn
chosen from a measured bearing turns by that bearing (clamped); a turn with no
bearing behind it is a search step smaller than the camera's field of view;
corrections use a finer band than the reporting vocabulary; and the spin
guard that forces a forward after long unproductive turning counts degrees,
not turns.

**Why.** Quarter turns against a 10-degree centre band closed zero distance
and flipped left/right on 52 of 60 steps. Quarter-turn search steps left
blind gaps between views. And when the search step halved, a guard counted in
turns silently halved its meaning. **Lesson recorded:** a threshold counted in
steps changes meaning when the step does.

### Arrival is the rule the car runs, judged by the range sensor

**Decision** (P7e first half, `PLAN-ros-alignment.md` 3.11, decided by the
user). A mission is judged ARRIVED locally when the target is detected, centred
within the steering band, and the lidar reads it within the arrival radius at
that bearing, on consecutive frames. It refuses to judge -- rather than
guess -- with no local perception, no usable scan, or a panned camera, and it
refuses a window of returns that looks like an edge rather than one face
(3.32: a mission had stopped beside a door jamb and declared `found` 1.1 m
short).

**The panned-camera refusal holds on real frames only.** Sim frames carry no
pan angle and their reported bearings are already relative to the camera,
so in the sim arrival cannot tell a panned camera from a centred one. That is
harmless today: the cloud-driven policies never peek, and a mission starts
with the camera centred (`PLAN-ros-alignment.md` 3.20). It stops being
harmless the day a tiered policy pans.

**Rejected.** 3.8's sim-only rule using the simulator's detection distances:
it would pass in the sim while saying nothing about the robot. **Rejected
also:** the detector's box size as range. **Acceptance bars (commitments):**
of missions that physically arrive, at least 95% end `found`, at perfect and
at degraded detection; and no mission ends `found` more than 0.60 m from the
target.

### The cloud confirms identity at arrival; the lidar decides distance

**In the simulator, its geometry confirms instead** (3.98, the user
2026-10-24; `docs-review/SPEC-REVIEW-3.md` fix 7). A render is not a
photograph, and a paid call on a flat-shaded billboard tied every sim
outcome to how a model reads it. As the sim reports detections from its
geometry rather than run a detector on a render (1.12), it answers the
identity question from the same ground truth: the nearest object at the
centre of the arrival frame must be the target. A setting keeps the real
call for a run that means to test it. Frames that are not the simulator's always go to the tiers below.
**Trade-off:** sim missions no longer exercise the real confirmation call
by default, so it is measured on purpose (Stage 1's lit-renderer re-run,
the car), not in passing.

**Decision.** Decided by the user 2026-10-02; built the same day. Two rules,
decided together, that split the end of a tiered mission by question, as the
arbitration above splits steering:

- **Identity: the cloud confirms at arrival.** Before a tiered mission ends
  `found`, one paid cloud call on the arrival frame must agree that what the
  robot has stopped at is the target. If the cloud says no, or the mission's
  call budget is spent, the mission does not end `found`, and a refused
  arrival is not paid for again while the robot stays there. A call that
  FAILS is not an answer: it counts against the vision failure budget like
  any cloud failure, and the arrival is asked again. If the budget runs
  out while the arrival still holds, the mission ends `arrived_unconfirmed`
  (3.47, chosen by the user 2026-10-07): the robot reached what the local
  tier took for the target, and nothing checked what it is. It is never
  `found`, because a local false positive looks exactly like it, and it is
  not `failed`, because the robot did its part. A cloud that answers no is a
  refusal and never ends there.
- **Distance: the arrival rule only.** A cloud target-reached answer that
  lands under the asynchronous tier does not end a tiered mission. The
  lidar-judged arrival rule stays the only way a tiered mission ends `found`
  on the car.

**Amended 2026-10-14 (3.80.C) and 2026-10-17 (3.80.O): with a local VLM on
board, it is asked the identity question first.** Its clear yes confirms an
arrival and its clear no refuses one, with no cloud call. An answer in
between is no verdict: the arrival is asked again, and after a set number of
such answers, or when the local VLM does not answer at all, the cloud is
asked as above. Three limits keep the tiers' order. An arrival whose bearing
came from the local VLM's own box is confirmed by the cloud alone, because
the same model would confirm its own mistake (3.80.O). Once the cloud has
spoken about the candidate in view, the identity question is the cloud's:
its held no refuses the arrival without a call (3.91.C). And a local VLM
that is away or cannot decide never ends a mission; 3.47's
`arrived_unconfirmed` stays the end when the cloud cannot be reached either.
It is off unless a local VLM is configured. Measured on the board (3.80.E):
precision 97.2% and recall 94.6% on the 299 pinned frames with no cloud
call, and the verdict in 1.91 s at p90.

**Rejected, for identity:**

- Enforcing the corroboration verdict ([perception](../perception/ARCHITECTURE.md))
  at arrival. It is free, but it was measured net-negative on the search
  walk and is unproven on out-of-vocabulary targets.
- Extending the hysteresis to steering. It reduces wrong-object chases, but
  a wrong object that keeps passing the gate for consecutive frames still
  ends `found`.
- Leaving it as is: the per-frame probability gate is then the only bound on
  a wrong `found`.

**Rejected, for distance:**

- Applying a landed cloud target-reached. It reintroduces a distance judged
  from a photograph -- uncalibrated, and exactly what the arrival rule was
  built not to trust -- onto the car.
- Applying it only on bodies with no range sensor. It lets phone walks end
  `found`, but makes a mission's ending depend on which body ran it, and the
  phone walk is not the car.

**Trade-off.** One paid cloud call per arrival (a synchronous tier whose
trigger fires on the arrival frame pays twice for that frame), spent on the judgement the
cloud model is good at (identity) and never on the one it is not (range).
Steering is unchanged: a wrong object can still be driven to; it can no
longer be reported `found`. Accepted consequence: a tiered phone walk, which
has no range sensor, cannot end `found`, by design, and ends at its step
budget (P7e stays).

### Hold the cloud's goal; never block on it

**Decision** (2.5, Phases A and F). The cloud call is dispatched without
blocking the tick, at most one in flight, and the last cloud direction is
held on free frames. An answer that lands after the mission ends is dropped.
**One exception, by design:** the identity confirmation at arrival blocks the
tick, because the mission is about to end on its answer. It waits for any
call already in flight before it is made, so "at most one in flight" holds
for it too (`docs-review/SPEC-REVIEW-3.md` fix 8).
**Rejected.** Blocking the tick on a multi-second call (a stall every few
frames), and scanning on every free frame, which on five rig walks outvoted
the cloud five to one.

### A sighting can outlive the frame -- but only when data says so

**Decision** (P25 / P7c item 2). A goal pose anchors a sighting in the
odometry frame so the bearing survives rotation; without a range it holds a
direction, exact under rotation and wrong under translation, and says so. It
is built and wired in, and **off** until a run shows it helps: the problem it
targets is measured, the cure is not, and a 1-in-3 detector needs a
range-anchored point, not a direction (R1).

### A model's distance guess is never a safety input

**Decision.** No brain-side veto reads the model's obstacle or distance
claim. One once stopped a forward when the model said the obstacle was within
one step, on a backend with no distance sensor and off by default; it was
deleted in 3.64 (the user, 2026-10-09), because models disagree on obstacle
presence from about 0% to about 100% on the same frames and the lidar is the
obstacle sensor. The field is kept as a report and never acted on.

### Explore's goals are places the robot can drive away from

**Decision (3.45, 2026-10-07).** A frontier or view goal keeps the chassis'
half-length plus the safety layer's stopping distance from anything the map
shows occupied. Stood there, at any heading, the straight move ahead and
the one behind each have the full bar.

**Why.** The safety layer, not nav2, decides whether the robot may move,
and it is stricter than nav2's inflated costmap. A goal nav2 can reach but
the safety layer will not let the robot leave is a trap: nav2 spends its
120 s goal timeout pushing against refusals, which is the whole parked
stall. On the furnished-home maps explore really had, the old 0.22 m
clearance chose goals that could be left at every heading only 36% of the
time on ground truth; 0.30 m reached 52%; the half-length plus 20 cm
reached 98%.

**What it costs.** Fewer stopping places in tight spots. A frontier is
seen from where the robot stops, and the lidar sees 360 degrees, so a goal
set back from a corner still clears it. Coverage is the progress check.
Approach points toward a seen target keep the old clearance: the arrival
rule judges them at the lidar's 0.40 m, and they are not left again.

### Explore's next goal may be learned; nothing else in explore is

**Decision (3.65 phase 2, 2026-10-11).** Which goal explore sends next may
come from a learned score instead of the hand-written order, behind a
setting. The default is the room-feature ranker (since 3.125): a small
network over seventeen facts explore has about each candidate -- ten
geometric ones and seven about the candidate's room (3.96). The two
earlier scores stay as settings: the hand-written order, and 3.65's
first learned score, a linear one over the ten geometric facts, with a
variant that adds a small network to it (`brain/goal_policy.py`
`POLICIES`). The learning is reinforcement learning, first in its
evolutionary form: the linear score's weights were found by the
cross-entropy method on simulated missions, each mission scored +1 if
found, minus 0.02 per goal and 0.01 per metre. The network variants were
trained by a gradient method, the room-feature one by cloning and then PPO.

**Why this decision and not the driving.** The misses 3.69 fixed were all
decisions; nav2 reaches every goal it is sent; and the simulator is
kinematic, so a learned controller would learn a world without slip. The
choice of goal is where better data changes the outcome, and it is scored
by a fast sweep against ground-truth reachability.

**Why evolutionary first, then a network.** With ten weights, a reward
mostly decided at the mission's end, and missions about a second each,
scoring whole missions avoids per-step credit assignment, and the result
is ten numbers anyone can read. The decision allowed a small network, and
a gradient method such as PPO to train it, only after the linear score
failed its criteria on record. It did (3.65, phase 2 results: criterion 1
FAILED), and the network was then built (3.65, amendment 2).

**What keeps it safe.** The policy only ranks candidates the rules would
send: unleavable, set-aside and twice-reached places are never offered.
Sightings, approaches, escapes, the retry book, the end of the search,
nav2 and `robot/safety.py` stay rules. A bad weight makes a search slower,
never a move unsafe. It is judged on houses it was not trained or selected
on, and live on nav2, before the default could change.

### Where to look is chosen from the map, never from a frame

**Decision (3.80.G and 3.80.I, 2026-10-16/17).** With the target out of
view, the next place to look comes from what the robot knows beyond the
current frame -- its map, its memory and common sense about rooms -- never
from a model asked "which way?" about one picture.

**Why: the answer is not in the picture.** 3.80.I asked every tier, local
and cloud, for a search direction on 42 out-of-view renders whose true
answer is the first metre of the shortest path to the target. The bar was
the majority side's share plus 15 points (58%). Nothing passed: the best
were RoboBrain2.0-3B 41% and Opus 4.5 38%, then GPT-6 Astra 29% and Opus
5.5 21%, all below always naming the more common side (43%); the local
Qwens said FORWARD on all 42. The mirror test shows why: flipped left to
right, the out-of-view answer swaps sides only 21-67% of the time, so it is
mostly a guess. With the target in view the same models steer well (Astra
56 / 60): seeing is solved, searching is not a vision question.

**The layers, each feeding the one below it:**

| Source of the answer | What it adds | State |
|---|---|---|
| **A map of what's unexplored** -- explore's frontier, view and approach goals over SLAM's map | the candidate places to go; coverage that eventually finds anything reachable | **built, the design** (3.80.G: arrivals 2 -> 16, cloud calls 765 -> 6) |
| **A learned choice among those frontiers** -- explore's goal policy (3.65) | the order in which the frontier's candidates are visited | built; the room-feature ranker is the default since 3.125 (the user, 2026-10-31: without room labels on 588 test houses, median goals -14% in mode R and -22% in U, finds within 1%, mean metres +10% in R). Its whole choice on the Jetson under the full stack is 95.2 ms p90 on the benchmark maps (bar 100, met in 3.131 with about 5 ms to spare; 132.9 in 3.125); with the cells explore's camera has really seen, a live choice there still runs over 100 ms (3.130's caveat). Before it, the hand-written order (3.65's network saved 11% of median goals on fresh houses, bar 20%). With every room labelled 3.96's ranker saved 43%, but a find's map has room types only from a person's names on a saved map (3.160.D) or from its own asks, which are off by default (3.116.B) |
| **Memory of where things were seen** -- the object inventory (3.46) | a find's first goals: the places earlier missions on the saved map saw the target. A candidate's distance to such a place, as a ranker input, is not built | built as a rule ahead of the ranker, not as an input to it (3.172.C: live in the simulator, 8 of 8 found, every first goal a remembered place, median 134 s against 502). A target moved between missions FAILED its bars (3.175.A, B). Empty on real frames: naming every object was closed (3.46.C) and the car's watch list notices nothing yet (3.178) |
| **Common sense about rooms** -- one language-model call ranks room types for the target ("a charger: office, kitchen, bedroom") | a candidate's room prior | built for ten targets: the default ranker's prior knows them (3.96); for any other target the one-call ranking is wired in the code and off by default, and acts only on a map with room labels (3.190, "A room prior for a target the shipped prior lacks", below) |
| **What is visible toward a frontier** -- a model rates how promising each unexplored edge looks for this target (published object-goal navigation work) | a candidate's visual promise | not built: the one scorer tried FAILED offline and its build step was dropped (3.177) |

**How reinforcement learning ties to the frontier.** The frontier map is
where the candidates come from; RL only decides their order. 3.65's policy
scores each candidate the rules allow (unleavable, set-aside and
twice-reached places are never offered) and explore sends the best one;
training rewards finding the target in fewer goals and metres. 3.65's
ten inputs were all geometric -- path length, turn, frontier length, view-gap
area, unknown and unseen share nearby, past failures and reaches nearby
(`brain/goal_policy.py` `FEATURES`) -- which is why it gained so little over
the rule: from geometry alone, one frontier looks much like another. The
layers above enter as new inputs to that same ranker, and training learns
how much each is worth. The first has entered: since 3.96 the default ranker
reads seven facts about a candidate's room beside the ten (whether the robot
has stood in it, its size, whether its type is known, and the target's prior
for that type). An inventory distance and a visual promise would be further
columns; neither is built. So the search improves without a new policy or a
new safety argument: the ranker still only orders places the rules already
allow, and a bad weight makes a search slower, never a move unsafe.

**What has been built, what failed, and what is next** (2026-11-21; the
three steps are the order this decision first set):

1. **The room prior as a ranker input: built (3.96), and the default
   (3.125).** Trained on 5,464 ProcTHOR houses and tested once on 588 it had
   not seen, against the hand-written order. With every room labelled:
   median goals 7 to 4 where the target is one of the house's own objects
   and 9 to 5 for a backpack in a random room, finds held (3.96's criteria
   2-4 met; with half the labels withheld, 7 to 5). Recorded as FAILED in
   the same section: criterion 1 (the new facts pick the oracle's candidate
   on 31.0% of held-out choices, bar 50%), criterion 5 (1 of 1,171 missions
   hit the tick cap, a close-in livelock in explore's approach), criterion 6
   (the choice took 312.9 ms p90 on the Jetson, bar 100; the bar was met
   later, on the same benchmark, in 3.131)
   and criterion 8 (three tests that pin the hand-written order's own
   choices). Live on nav2: found 3 of 3 with no contact (criterion 7 met).
   * **The prior acts only where the map has room types,** and a find's
     map has them only from the two sources below. Without labels the
     ranker still beat the hand-written order (3.125, the figures in the
     table above), which is why it became the default; with no label the
     prior reads as nothing, so that saving is not the prior's.
   * **Labels asked during a find: built, off by default (3.116.B).** In the
     simulator, true answers kept about 61% of the full-label gain (reward
     +0.091, interval +0.032 to +0.151) at a median of 5 asks a mission;
     with 15% of answers wrong the gain was +0.034, its interval spanning 0
     (3.116.A). For the backpack, whose prior is nearly flat, the asked
     labels cost reward (-0.080), so a find asks nothing for a target whose
     prior is flat or missing (3.116's amendment 7, criteria 11-13 met).
   * **On real frames the labeller failed its bar.** The cloud's room answer
     was wrong on 51.4% of 734 recorded rig-walk frames (interval 34.3 to
     66.2; bar 15%), nearly all of it one open basement and gym; the living
     room and stair hall were right on 106 of 107. That path waits for the
     Rover's camera and frames of the rooms the walks lack (3.122).
   * **Rooms a person names once: built (3.160.B), and a find reads them
     without asking anything (3.160.D).** In process, with each house's true
     names: reward +0.18 (interval +0.08 to +0.30), finds unchanged. Its one
     live find FAILED (D7: it ran out of time; the target's room was not on
     the saved map). Naming the real house's rooms waits on the Rover
     (3.160.C).
   * **A prior for any other target** is the next decision's subject
     (3.190): the wiring is in the code, off by default.
   * **More learning did not close the gap.** No later chooser beat 3.96's:
     more facts (3.101, criterion 1 FAILED; 3.117, both arms FAILED), the
     evolutionary method on the same facts (3.102: worse than its gradient
     twins where the target is one of the house's own objects), the
     network's shape and attention over the pool (3.103, criterion 1 FAILED
     for each), tie-aware labels (3.114, FAILED), the chooser's own states
     (3.115, criterion 3 FAILED) and per-choice credit (3.119, both arms
     FAILED). They land at 0.55-0.56 mean reward on the test houses where
     the oracle scores 0.78 (3.111). The line is closed until the Rover's
     data exists; the next lever named there is the target's room from real
     perception, not the learner (3.119).
2. **The inventory distance: not built as a ranker input.** Its condition,
   real frames that name objects, has not been met: naming every object was
   closed (3.46.C), and the car's watch list notices nothing yet (3.178).
   What was built instead is a rule ahead of the ranker: a find goes first
   to where earlier missions on the saved map saw the target, one try a
   place, and only then to the ranked frontiers (3.172.C; the table above).
   Stale memory is recorded as failed, not solved: with the target moved
   between missions the sweep FAILED its bars on `dev` and with the first
   fix (3.175.A, B), and two further rules FAILED and did not ship (3.175.E,
   F). A real object moved between two finds waits on the Rover (3.175.D).
3. **Visual promise: tried offline, FAILED, not built.** CLIP RN50's
   whole-frame score does not rise before the target comes into view on the
   rig walks (object-balanced AUROC 0.45, bar 0.65; 3.177.A, 2026-11-12), so
   the step that would have built it was dropped (3.177.B). Another scorer
   would need new bars.

**Still next, each waiting on the Rover:** the asked labels and both priors
in a real house (3.116.D, 3.190.E), the person's names on the real map
(3.160.C), room answers on frames of the rooms the walks lack (3.122), and
a moved target (3.175.D). Learning the choice from a person's goal taps is
parked until a learned chooser on the board shows a gap there (3.112).

### A room prior for a target the shipped prior lacks is asked once, at the start

**Decision** (3.190, asked by the user 2026-11-19: "What if we made a cloud
call to find out which room type a target is likely to be in, before
starting a mission?"; built on 3.116.C's question). The default goal ranker
orders explore's goals by how likely each room type is to hold the target,
and the prior it ships with knows ten targets. A find's target is free
text: any other target reads an empty prior, and the ranker then runs as if
the map had no room labels at all. So for such a target the brain may ask
the cloud one text-only question as the find starts: for each room type the
ranker knows, what share of such rooms hold at least one such object. The
answer sits where the shipped prior's rates would, for that mission only.

**State (2026-11-22).** The wiring is in the code, off by default. Of
3.190's steps, A, C, D and F are done and B and E are open. D and F are
recorded as FAILED (D on criterion 14's first-goal clause, F on F1 and F2):
nothing ships from them.

**Why it is shaped this way.**

- **Asked only when it can act.** The job is a find, the room ranker is the
  one choosing goals, the target is not one the shipped prior knows, and
  the map has room labels (without them the prior is inert, 3.176.A).
  Otherwise no call is made, and the status says why.
- **It only reorders goals.** It never ends a search and never adds a goal,
  so the argument that keeps the learned ranker safe covers it unchanged: a
  bad prior makes a search slower, never a move unsafe. Measured on 583
  test houses (3.190.A): against no prior, reward +0.100 (interval +0.051
  to +0.150) and 19% fewer median metres; a deliberately wrong prior still
  found as many targets, at more metres.
- **The start never waits long, and the prior never arrives mid-mission.**
  The call has a short deadline inside the start. A reply after it is kept
  for the next find of that target and is not applied to this one, so a
  mission's goal order does not change under it part way.
- **A bad reply is no prior, never a guess.** Anything but the expected
  rates, a refusal, a timeout or no connection: the mission starts as it
  does today, and the status records which. Offline never blocks a start
  (3.49).
- **Paid for once.** Replies are kept by model and target, so a repeat find
  costs nothing and needs no connection.
- **It goes through the cloud vision service,** as every paid call does,
  on the model the question was measured on.
- **A person's typed room hint goes first** and the prior orders the rooms
  after it (the user, 2026-11-20; 3.190's criterion 12 stays FAILED as
  written, and this is the design that answers it).
- **It never unlocks more paid calls.** Whether a find asks for room labels
  is still decided on the shipped prior alone (3.190's open question 1,
  assumed, not yet answered by the user).

**Rejected.**

- **Weighting the cloud's prior more, so the first goal goes to its top
  room.** Live, the prior's top room was the kitchen and the first goal was
  a near frontier in the dining room beside it (criterion 14 FAILED on that
  clause). The user's answer was "measure first" (2026-11-21); 3.190.F
  swept a heavier weight, and it lost reward (-0.017) and sent the first
  goal to the top room no more often (0.46 against 0.44; F1 and F2 FAILED).
  The learned ranker's balance stays: a prior reorders, it does not force.
- **A prior shown before Start.** Reading a typed request makes no paid
  call (3.176), and this keeps that.
- **The local VLM as the source.** Text-only questions were never measured
  on it; a new section if wanted.

**What leaves the house:** the target's words, no image (the privacy table
above).

### A short watch list rides along on every mission

**Decision** (3.178, asked by the user 2026-11-12, from a robot vacuum that
notices wallets and keys while it cleans and pins them on its map). A
search drives through most of a house while perception asks about one
thing, the day's target. Beside the target, a mission also watches for a
short list of things people lose (by default keys, wallet, phone, glasses
and remote; a handful at most; set in the brain's configuration, and from
the phone later). A sighting is placed
by the lidar and remembered in the object inventory like any other, and the
mission's report lists where each item was pinned. That is the memory
"where did you last see it" (3.175) and a find's recall (3.172.C) read.

**Why a short list, and not everything.** Naming every object on real
frames failed its bar with every namer tried and was closed by the user
(3.46.C, 2026-10-10), which left the car's memory knowing only the target
of the day. A short fixed list is an easier problem: the on-board pipeline
is already asked one name a mission, and this asks it a handful.

**Why it is shaped this way** (3.178.A's design, written before measuring).

- **One pass, no new model.** The list rides on the detector and encoder
  the target already uses. A design that needed another model was to fail
  as written and stop for the user.
- **The target's answer cannot move.** A crop's probability of being the
  target is computed exactly as without the list; the list's words never
  enter it.
- **Precision first.** A wrong pin costs trust and, through recall, a
  drive. So list items compete with each other, the target and the
  distractors for one crop, and the list's gate is stricter than the
  target's, fixed before the frames that judge it.
- **An item that names the day's target is counted once,** as the target.
- **No policy reads the list.** It changes what is remembered, never what
  the robot does next (3.178's stability bars: actions and outcomes equal
  with the list on and off, 40 of 40 in the simulator).

**State, as the code behaves.** In the simulator the inventory reads the
simulator's own labels, so the list tests the plumbing and the report, not
a namer. On the car the on-board pipeline is not prompted with the list
yet (the user, 2026-11-20): on recorded frames, prompting the detector with
the list changed its boxes and cost target detections (413 against 422
without it, bar 417; 3.178.B's B4 FAILED). The fix is judged on the Jetson,
where its cost per frame can be measured. Until then the car's watch list
notices nothing. This is the same gap as explore's (3.76 E): a detection
from a real frame has no path into the inventory yet.

**Rejected.**

- **Naming every object** (above).
- **A stricter pin: at least two looks voting for the item's own name and
  no dispute** (3.178.D, E). It cut wrong pins, but it pinned 41 of 50 true
  passes against a bar of 90%, fewer than the shipped rule's 44, so it did
  not ship. Its verdict is parked until list items can be ranged on real
  frames (3.178.G, H, on the Rover).

**Failure mode, measured and open.** A wrong pin. In the simulator, with
false list labels injected on 5% of frames, 6 wrong pins reached reports in
40 missions (0.15 a mission against a bar of 0.1; B2 FAILED). Two different
false labels ranged to one wall spot fuse into one remembered thing and are
reported under one of the names. A thing seen from one viewpoint only is
never reported, by the inventory's own rule that a report needs two
independent looks (3.178.F). Recall on real frames is unmeasured: the
recorded walks hold no list item (3.178.A).

## Contracts

| With | Direction | Category | Ownership |
|---|---|---|---|
| Mission runner ([mission](../mission/ARCHITECTURE.md)) | runner calls the agent's step; the agent calls the vision function, and at an arrival the identity confirmation, both through the runner's guard | in-process | runner owns budget, timeout, outcome; policy owns the action |
| Safety layer (safety domain) | agent submits each action | in-process controller over the gated body, re-vetted on the robot | the robot's veto is final; a refusal is a normal outcome |
| Body ([body](../body/ARCHITECTURE.md)) | agent reads frame, distance, depth, scan, odometry | in-process interface, HTTP underneath | body owns all readings; the policy never fabricates one |
| World ([world](../world/ARCHITECTURE.md)) | agent reads pose and map resolution | in-process interface, HTTP underneath | advisory: failure degrades exploration only |
| Perception ([perception](../perception/ARCHITECTURE.md)) | tier calls the pipeline once per frame | in-process: frame -> tri-state with bearing | perception owns what is seen; the policy owns what to do about it |
| Cloud vision ([cloud-vision](../cloud-vision/ARCHITECTURE.md)) | vision step calls the service; the tier also asks it on the arrival frame, reading only its visibility flag | HTTP/JSON | the service owns model, wording and the decision vocabulary |

## Failure modes and resilience targets

| Failure | Response | Target |
|---|---|---|
| Cloud call errors or hangs | raised to the mission's failure budget (an async failure on the next frame). Under explore's search, where the map and the local tiers carry the mission, a trigger's failed call is counted and not charged to the budget (3.80.G); the arrival's identity call is charged either way | every blind step is counted, never swallowed |
| Cloud unreachable at arrival | the identity confirmation raises into the same budget and is retried while the arrival holds; when the budget runs out the mission ends `arrived_unconfirmed` | an offline mission that reaches the target says so, and never says `found` (3.47: 24 / 24 offline arrivals, sync and async) |
| Local perception unavailable (camera wedged, model error) | never a trigger, never advances the cold-search count, never read as absent | a dead camera never looks like an empty room |
| Detector misses frames | trigger hysteresis, held goals, degree-counted spin guard | at 90% per-frame detection, at least 95% of missions arrive (3.6b's bar) |
| Target directly on the straight line through a door jamb | refused forwards; the mission ends `blocked` | no policy spends its budget pushing into a wall; going around is nav2's job |
| No range sensor (phone walk, replay) | arrival not judged; under the shipped asynchronous tier nothing else can end the mission `found`, so it runs to its step budget (P7e) | a mission never ends `found` on a guess. By design since 2026-10-02: a landed cloud target-reached does not end it |
| An edge beside the target | arrival refused | zero false arrivals beyond 0.60 m |
| World unreachable | right-hand rule | a mapper outage never ends a mission |
| Local false positive | steers on every frame it passes the probability gate; one frame is enough; the cloud's answer does not override a local sighting | target: a false positive never ends a mission `found`. **Met** (the cloud confirms identity at arrival; decided and built 2026-10-02): a wrong object can be driven to, and the arrival is refused unless the cloud agrees. Steering on a false positive is accepted |
| Spin with no detection | forced forward after a fixed amount of rotation | a held turn cannot repeat forever |

## The tiered ladder is a behavior tree

Since 3.94 the tiered policy's decisions are drawn as behavior trees: what
to do this frame, when to ask, and who to ask. Each is a few small nodes,
read in order. Each node has its own settings, and the tree prints itself.

**Why.** The ladder grew as nested code. Every new rule was an edit to a
long function, and its order of precedence could only be read off the
code. As a tree:
* a new behaviour is a new node;
* an order that may be tuned is a setting;
* the scenario simulator and a later learned node act on named nodes,
  not on lines.

**What the tree owns, and what it does not.**
* **It owns the choices among allowed actions:**
  * which evidence steers;
  * which event earns a question;
  * which tier is asked.
* **Hard rules stay outside or fixed:**
  * the safety veto;
  * a person's Stop;
  * the arrival rule;
  * "`found` needs the identity check", and the cloud's alone for an
    arrival the local VLM's box led to (3.80.O);
  * B3's failsafes;
  * the order of who-to-ask's guards (the call cap, one call at a time
    and "mission ended" come before any question).

  No setting reaches these, so a later learned node can choose only
  among what they allow (the shield in
  `LEARNING-APPROACHES-2026-10-22.md`).

**Redrawn first, changed later.** 3.94 changed no behaviour. The new tree
was checked against the code it replaced on thousands of random states.
Changes of behaviour, such as 3.92's escalation on disagreement and 3.91.C's
rules, are new nodes in their own sections, each with its own bars.

**On a standard library (3.95).** The trees run on `py_trees`, the common
open-source Python behavior-tree library, rather than code of our own. The
richer trees still to come need what it already has: nodes that span frames,
decorators such as timeouts and retries, and drawings of the tree. It also
reads BehaviorTree.CPP's XML, the format nav2's trees are written in. The
move changed no decision.

**Rejected.**
* **Leaving the ladder as nested code,** for the reasons under "Why".
* **A tree module of our own.** 3.94 first built the trees on a small
  module of the project's, to avoid a new dependency on the Jetson, with
  the library's semantics so that a move would be mechanical. The user
  reversed that the next day (3.95, 2026-10-23), for the reasons in the
  paragraph above.
* **Changing behaviour in the same step as the redraw.** A redraw that also
  changed a decision could not be checked against the code it replaced.
* **Not drawn as a tree, on purpose:** what an answer changes (the held
  goal, the remembered "no", the pacing) stays code that the nodes call
  (3.94), as does the plumbing around the trees.

**When it goes wrong.**
* **A tree that could not decide is refused before a mission runs.** A
  configured tree with no default branch, an unknown or repeated node, a
  setting that names a hard rule, or a steering band wider than the
  arrival rule's is refused when the configuration is loaded (3.94.D:
  sixteen bad trees, each refused). The band may only narrow: a wider one
  would stop turning toward a target the arrival rule never counts as
  centred, and the robot would never arrive.
* **An error inside a node is never swallowed.** It reaches the policy's
  caller unchanged (3.95's criterion 3), and a tree whose branches choose
  nothing raises instead of skipping the move.
* **A wrong redraw is caught by the frozen reference.** The code the trees
  replaced is kept as a reference, and the suite compares the two on random
  states (0 differences on 10,000; planted bugs were each caught). On
  recorded missions, 0 of 41,585 frames differed from the code before the
  trees (3.94 2b, 3.95 2b).
* **Recorded as failed:** 3.94's one live mission ended out of budget, not
  found (its criterion 6). The cause traced to explore's close-in move near
  furniture, not to a node; two diagnostic runs on the tree both ended
  found, and the criterion stays failed as written.
* **Cost:** the three trees together take under a millisecond a frame on
  the Jetson at 15 W (p90 0.85 ms, bar 5 ms; 3.95).

## Open questions

- **The room-level planner** -- an LLM reasoning over mission memory to
  choose which room to search next -- is not built as a planner, and since
  3.80.I it is not meant to be one: it became the second-to-last layer of
  "where to look is chosen from the map" (above), a room prior fed to
  explore's goal ranker (decided by the user). That layer is built for
  the ten targets the ranker's prior was learned for on the training houses
  (3.96). For any other target, one text-only cloud question at a find's
  start is wired in the code and off by default (3.190; 3.116.C measured
  the question first, and its agreement bar, criterion 8, FAILED). Which rooms were
  already searched reaches the ranker as a fact from the robot's own trail
  (3.96), not from a model. What stays open:
  * whether either prior helps in a real house (3.116.D and 3.190.E, on the
    Rover). In the simulator a prior acts only where the map has room types,
    and on real frames the room labeller failed its bar (3.122);
  * the language-model half as first designed: mission memory's context
    block still feeds nothing (the engineering spec's known gaps).
- **Dead-reckoned bearing**: stays off until a run both ways shows the run
  length of commands rise without losing arrival.
- **Arrival on real pixels**: needs the real detector to keep recognising the
  target at 40 cm (it relabelled a bottle as a vase); a rig walk on the robot
  answers it.
- **P7e's second half** (a held cloud STOP versus steering) no longer matters
  on the lidar path and is left alone.
- **Steering by verbs versus nav2 goals**: the tier keeps steering by verbs
  for now (R6). Revisit with continuous motion.
- **Learning explore's goal choice from a person** (raised 2026-10-20, not
  built). The ideas it rests on:
  * *Reinforcement versus imitation.* Reinforcement learning learns from a
    reward over many tries (3.65's +1 found, minus goals and metres).
    Imitation (behaviour cloning) learns "choose what the expert chose"
    from examples. A person at the controls gives examples, not a reward,
    so teleoperation feeds imitation.
  * *Learn at the level of the decision.* What is learned is the choice
    among the goals the rules allow, so the example worth recording is the
    person's pick among those same candidates (a tap on the map), not
    their D-pad verbs. Wheel commands do not map onto "which candidate",
    and the driving is nav2's.
  * *Warm start.* Cloned choices give a sensible starting policy; RL in
    simulation then improves past the expert instead of being capped at
    the person's skill.
  * *Intervention learning (DAgger and its human-gated form).* The robot
    runs on its own, a person corrects it when it goes wrong, and each
    correction is a training example. Efficient, because the corrections
    land exactly where the policy is weak.
  * *The feature gap.* A person may choose by cues the ten candidate facts
    cannot express (a hallway's look, where bins live); cloning through
    those facts loses exactly that, so the payoff is unknown until
    measured.
  * *How much data.* About 10-30 choices a mission: a few hundred taps can
    fit the linear score over the ten geometric facts, not the default
    ranker's network over seventeen.

  **Decided by the user (2026-10-20): corrections are goal taps, never
  takeovers.** A person's Stop and D-pad still end explore and keep the
  brain out (the safety domain), unchanged. A person instead taps another
  candidate while explore runs; explore sends that goal and logs the pick
  as a training example, so the correction needs no exception to the
  safety rule. It must be its own explicit message, not the person's map
  goal, which explore deliberately neither adopts nor cancels (3.76 E).
  Not built; no criteria yet.
- **Learning the tiered ladder** (raised 2026-10-22, after 3.91 B6; the
  tree is built, nothing learns on it yet). The ladder's numbers are config
  (the `tier_*` keys); its flow was code: who is asked, and what an answer
  does. B6 showed a local "no" changes no steering, and the user asked
  whether the rules could be learned. What was built since: the decisions
  are behavior trees whose order and numbers are settings, redrawn with no
  change of behaviour (3.94), on a standard library (3.95; "The tiered
  ladder is a behavior tree", above). That was the prerequisite the survey
  below names for tuning or learning a node. Recorded as failed: 3.94's one
  live mission (its criterion 6; the cause was explore's close-in move, not
  a node). Still open: every learner.
  [`LEARNING-APPROACHES-2026-10-22.md`](../evaluations/LEARNING-APPROACHES-2026-10-22.md)
  surveys twelve ways and orders them:
  * value of information over calibrated confidences;
  * then the ladder as a behavior tree whose numbers are tuned black-box,
    against a simulator that emulates perception's reports calibrated on
    real frames;
  * then, on the Rover, conformal bounds on the local tier's unchecked
    errors, and logged randomised choices so that bandits and routers can be
    judged from logs;
  * end-to-end RL last, after a realistic simulator (evaluated and parked by
    the user 2026-10-22).

  The same rule as explore's goal choice holds: a learner only chooses among
  actions the rules allow. Safety, a person's Stop and the arrival guards are
  its shield.
