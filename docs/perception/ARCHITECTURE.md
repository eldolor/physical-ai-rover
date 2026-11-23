---
kind: architecture
domain: perception
status: current
verified: 2026-11-21
---

# Perception -- architecture

The on-board perception tier answers one question per camera frame, locally
and for free: **is the mission's target in this frame, and at what bearing?**
It is what lets the tiered policy call the cloud only on events, what gives
the robot a bearing to steer on, and what the arrival rule pairs with the
lidar. Since 3.46 it has a second, passive product: an **inventory** of
every object a mission saw, and where. It does not decide moves ([policy](../policy/ARCHITECTURE.md)) and it
is not meant to confirm identity -- a vision-language model is, at arrival
too (see "Detect, then verify"): since 3.80 the **local VLM** on the board
first, then the cloud. The local VLM is a separate service with its home in
this spec ("The local VLM: a separate service, asked before the cloud").
Read this for why the tier
is a detector plus a verifier with a three-state answer; read the
[engineering spec](../engineering/perception/ENGINEERING.md) for the shipped
models, thresholds, commands and recorded numbers.

## Purpose

A cloud vision call per frame costs money and seconds; a robot that only
sees through the cloud cannot steer between calls. Perception supplies
the cheap, fast, per-frame half of the split the project settled in
`PLAN-onboard-perception.md` 1.11: **local perception says *where*, the cloud
says *what*.** Its dependants are the tiered policy (triggers and steering),
the arrival rule (a centred detection), the twin's readouts (state,
probability, model names), and every perception finding in the plan, which is
measured through the same pipeline the robot runs.

## Components and boundaries

```text
   frame (JPEG, pan angle)                  sim frame (reported detections)
        |                                              |
        v                                              v
  +-- perception pipeline ------------------+   +-- frame-reported pipeline --+
  | detector  -> region proposals           |   | read the detections the sim |
  | crop rule -> which proposals to verify  |   | computed from its geometry, |
  | verifier  -> P(target | crop, texts)    |   | occlusion-aware; marked     |
  | gate      -> detected / absent /        |   | "synthesised"               |
  |              unavailable, with bearing  |   +-----------------------------+
  +-----------------------------------------+               |
        |                                                   |
        +-------------- one result shape -------------------+
                               |
                               v
                     tiered policy, arrival rule, status

  local VLM (3.80; its own process, on the board only):
    a frame + the target words -> a score for "is this the target?"
      (and, when asked, where in the frame)
      -> the tiered policy, which owns the thresholds
    not there (not installed, loading, killed) -> the cloud is asked instead

  object inventory (3.46; reported, and carried to later missions on a saved map):
    every frame's detections + the lidar + the map pose
      -> placed observations -> landmarks with a belief and a class vote
      -> the mission's report (local file, private bucket)

  measuring instruments (off the mission path):
    corpus scorer  - every labelled walk, against human labels
    target probe   - does a target string fire on unrelated frames?
    latency bench  - per-frame GPU/CPU split on pinned frames
    lab backends   - candidate models behind the same seams
```

| Part | Owns | Must not |
|---|---|---|
| Detector seam | turning image bytes into region proposals | decide identity |
| Verifier seam | scoring a crop against the target string and a fixed set of distractors | be read as a probability of presence on its own (only the softmax over target + distractors is) |
| Region-proposer seam | class-agnostic regions (a floor mask today; off) | replace the detector's boxes -- only add to them |
| Pipeline | crop rule, gate, tri-state, bearing | load any model at import time; run on a raycaster render |
| Frame-reported pipeline | the sim's stand-in, read from frame data | compute anything; claim a model ran |
| Local VLM service (3.80) | one resident vision-language model on the board, answering one question at a time about one frame with a raw score | decide what its score means (the policy's thresholds do); drive, steer or veto; be needed for a mission to start or to continue; hold memory the robot stack needs (it is the first process killed) |
| Local VLM client (in the brain) | asking the service, and telling *unable* (it answered, but not usefully) from *absent* (nothing there to answer) | turn a hang into absence, or absence into a "no" |
| Object inventory | every object a mission saw: placed by the lidar, fused into landmarks, weighted by independent looks | be read by a policy within the mission that built it (a later mission's recall reads what was saved); place an object without a range; store an image |
| Lab backends | candidates on equal footing (open-vocabulary detectors, a small VLM, SAM) | be imported by the shipped pipeline |
| Corpus scorer, target probe, latency bench | measurement | feed the robot; the recordings domain owns the corpus itself |

**Boundaries.** The seams are protocols, so a fake drives every test and a
different runtime (a TensorRT engine, say) can replace a backend without the
pipeline changing. The model libraries are an optional install, imported
lazily inside the concrete backends; nothing in the automated suite needs
them.

## Decisions

### Perception runs in the brain process

**Decision.** The pipeline is built and run inside the brain, in the same
process as the mission loop, called once per frame by the tiered policy.
That is why a tiered mission needs the model libraries wherever the brain
runs, and why the brain runs on a machine that can hold the models (a laptop
today, the board later) rather than in the cloud deployment.

**Rejected, for now:** perception as its own process that publishes
features, not frames (`PLAN-onboard-perception.md` 2.7). That would let a
stalled detector degrade perception instead of stalling the control loop,
but it adds a second service, a second wire format and a second liveness
signal before any board measurement says the detector stalls.
**Trade-off.** A hung model call hangs the tick. The mission's per-call
vision timeout and the hung-tick dead-man ([mission](../mission/ARCHITECTURE.md))
are what catch it, and they end the mission rather than degrade it. Kept as
an open question below.

### Detect, then verify

**Decision.** A detector proposes regions; an image-text verifier scores
each crop against the target string and a set of bland distractors; a gate on
the resulting probability decides. The verifier raises a candidate and never
confirms one -- identity belongs to the cloud (1.11). Which models fill the
two seams is an engineering choice, measured and recorded in the
[engineering spec](../engineering/perception/ENGINEERING.md).

**At arrival too.** "Never confirms" holds for the cloud policy's `found`,
and since 2026-10-02 for arrival as well. On the tiered path the arrival
rule ([policy](../policy/ARCHITECTURE.md)) judges distance from a local
detection centred at close lidar range; before that ends a mission `found`,
one paid cloud call on the arrival frame must agree the target is in it
(decided by the user 2026-10-02, built the same day); since 3.80 the local
VLM is asked first and the cloud only when it cannot answer (below). Until then a verifier
false positive that kept passing the gate could end a mission at the wrong
object. Corroboration (below) still measures the local tier's agreement and
is not enforced. The rule, its rejected
alternatives and its trade-off are the [policy](../policy/ARCHITECTURE.md)'s
("The cloud confirms identity at arrival; the lidar decides distance"). It
fits this domain's split: the cloud owns identity, and the local tier is
recall-first.

**Rejected.** A single open-vocabulary detector as the whole pipeline (OWLv2,
Grounding DINO, YOLO-World): measured as replacements in P7, P9 and P21, an
open-vocabulary model won as a *crop source* rather than as a classifier,
because the gain is its regions, not its own confidence. A small local VLM
(Qwen) was not run in P7 because its score is not separable into a gate and it
had a known upstream performance bug. **Rejected also:** a third model per
frame. The floor-mask proposer measured +8 points on an early detector, then
lost on 9 of 11 checkpoints with the shipped detector at six times the whole
tier's cost (P24), so the tier is two models (P20).

### A text-prompted detector, and the open-vocabulary crop rule

**Decision.** The detector takes the target string as its prompt, and every
proposal reaches the verifier regardless of label. The model family and
variant are tuning, chosen by corpus score and latency at a matched
false-positive budget. The current choice and its evidence are in the
engineering spec.

**Rejected.** 4.2's original rule -- use a closed-vocabulary COCO detector and
keep only crops labelled with the target's COCO word. On the first valid rig
walk it lost 11 of 18 true positives, because at close range the detector
relabels the object (a bottle becomes a vase), so the gate discarded exactly
the frames where the target filled the view. **Rejected also:** choosing a
model on a subsample of the corpus. A ranking that leads on a fraction of the
frames has inverted on the whole labelled set before (P23), so a model choice
is made on every labelled frame.

**Trade-off.** Recall is worth more than precision here by design: a miss
means driving past the target. The price is not small. A false positive that
passes the gate is a sighting: it may fire a paid cloud call, it steers the
robot on that frame (steering has no frame hysteresis, and the cloud's answer
does not override a local sighting), and near the wrong object it can satisfy
the arrival rule's distance half. It cannot end the mission `found` there:
the cloud check at arrival (see "At arrival too" above, and the failure-mode
table) refuses it. The rest stands.

### Gate on a probability, not a raw similarity

**Decision** (2026-09-07). The gate is P(target | crop, target + distractors),
a softmax over the verifier's scores. **Rejected:** a threshold on the raw
margin over the best distractor, because an image-text similarity is not comparable
across text queries -- no single margin served both rig walks (one that found
none of the bottle's sightings, and a lower one that gave the backpack walk
false positives); under the softmax one bar served both.

### Three states, never two

**Decision** (1.12, M3's argument one sensor over). Every frame is
**detected** (with a bearing), **absent** (the frame was good and the target
is not in it -- information), or **unavailable** (no frame, undecodable, a
model raised -- the absence of information). **Rejected:** collapsing
unavailable into absent, which makes a wedged camera read as "the target is
gone" and would let a stalled capture end a search.

### The simulator reports detections; no detector ever runs on a render

**Decision** (1.12, built at R1). In the sim, detections come from the
simulator's geometry -- in the field of view and at least partly visible,
reported at the bearing of the visible part and not at all when hidden (3.32)
-- and are marked synthesised. The sim tests the
detector's *consumers*, never the detector. **Rejected:** keeping the sim out
of the loop (no steering could be tested), and running real models on
raycaster frames or on textured sprites -- "the detector finds a sprite"
reads as success while measuring nothing. Accuracy claims come only from real
photographs.

**Consequence.** The brain chooses the sim stand-in by the frame's provenance
at mission start; if it cannot tell, it assumes a real camera, because loading
models that see nothing is cheaper than steering a real robot on detections
nobody measured.

### The board is a Jetson; the Hailo path is closed

**Decision** (2026-09-19, by the user). Perception runs on a Jetson Orin Nano
Super's GPU. **Rejected:** Pi 5 + Hailo-8L, chosen earlier on cost and power
(4.9, "DECISION 2026-09-13") -- the compile loop showed OWLv2 cannot be
allocated on the 8L (P6), and a dataflow accelerator makes compilability the
dominant model-selection criterion; also the IMX500 AI camera, which cannot be
fed a recorded frame (1.10). **Consequence.** The only axes left for a model
are recall and latency. Do not re-open the Hailo path or fund a compile run.

### A latency budget, measured on the board

**Commitment** (`PLAN-ros-alignment.md` 3.33, confirmed by the user
2026-10-02): **250 ms a frame at the board's 15 W mode.** Over it, the first
fix is moving image handling off the CPU (P26), then TensorRT. P26 is built
only if the board's own GPU/CPU split says the handling matters; its
acceptance bar is a plan criterion (`PLAN-onboard-perception.md` P26), not a
commitment of this spec. **Measured 2026-10-04:** the shipped pipeline is
well inside the budget on the board at 15 W (3.33), so P26 is not needed to
meet it.

### Measure against people, at a matched cost

**Decision** (P3). Perception is scored against each walk's adjudicated human
labels, never against the cloud model's own claims (on one search walk the
cloud claimed the target on 44 frames, 34 of them a storage bin). A run
records every frame's best score once and sweeps the gate afterwards; two
configurations are compared only at the same false-positive budget; GPU and
CPU rows are never merged (P24); the frame set is pinned before comparing.
**Rejected:** recall without its false positives -- that is how a
low-threshold open-vocabulary detector looks like a win.

### Corroboration and vocabulary are reported, not enforced

**Decision.** Two verdicts are computed every time they apply and published
on the twin and in recorded walks, and change no decision:

- **Corroboration** (1.11a, proposed 2026-09-07, not decided): does local
  perception support a sighting the cloud claimed, at a lower bar than
  detection? Measured live on three walks it was net negative -- it caught no
  false claim and suppressed one true, distant sighting.
- **Vocabulary**: whether the target string has a COCO word. It is evidence
  about the detector's vocabulary, never a capability claim (an in-vocabulary
  bottle was still lost at close range).

**Rejected:** enforcing either now. A target the local tier cannot see would
make every cloud sighting "unclear" and leave the robot steering forever
without committing. **Rejected also, on 2026-10-02, as the guard on a wrong
`found`:** enforcing corroboration at arrival. It is free, but measured
net-negative and unproven; the user chose a cloud identity check at arrival
instead ([policy](../policy/ARCHITECTURE.md)).

### Every object seen is kept (3.46)

A search walks most of a house and used to remember only its target. The
inventory keeps the rest so a person can ask what the robot saw, and so a
later search can start where a thing was last seen: a mission on a saved
map starts from the last inventory saved on it (3.50.D5), and a find goes
first to the places earlier missions on that map saw its target (3.172.C,
owned by the [policy](../policy/ARCHITECTURE.md)). A mission's own
inventory is **recorded and reported only**: no policy is handed it, and a
test pins that a search is move-for-move identical with it on and off.
Three choices carry it:

- **The detector points, the lidar measures** -- the rule the target has
  followed since R1. A detection without a lidar range is never placed; a
  monocular guess would drift confidently.
- **Belief from independent looks.** Repetition raises belief only from a
  new viewpoint, because a detector that errs on an image errs the same
  way on the next one; counting frames would make a robot standing still
  certain of its own mistake. Looking at a landmark's place and not seeing
  it lowers belief. Only landmarks over a belief bar are reported; the
  rest are kept as candidates.
- **The class is a vote.** Repetition shows consistency, not correctness,
  so a landmark keeps the labels it was given and is flagged disputed when
  they disagree.

It is uploaded to the project's private recordings store (decided by the
user 2026-10-07) as labels and positions only -- never an image -- because
it is a list of what is in someone's home.

### The local VLM: a separate service, asked before the cloud (3.80)

This section is the one place that says what the local VLM is, why it is
its own service, which questions go to it and which to the cloud, and what
happens when it is down. How it is built, run and measured is in the
[engineering spec](../engineering/perception/ENGINEERING.md) ("The local
VLM service"); the thresholds and the order of asking are the
[policy](../policy/ARCHITECTURE.md)'s.

**What it is.** A small vision-language model resident on the board's GPU,
behind an HTTP service on the board's loopback address. It is handed one
camera frame and the target's words and answers one question at a time. It
returns a raw score and a status, never a verdict: the brain's tiered
policy decides what a score means. It is the middle tier of identity, in
the order the user set (2026-10-13 and 2026-10-14): perception every frame,
then the local VLM, then the cloud. Which model and runtime fill it is an
engineering choice (3.78 chose it; 3.80.D-T tried the others and none
replaced it).

**Why it exists.** 3.78 measured a small VLM on the board that meets the
identity bars 3.48 set, answers while the robot waits at arrival, and on
the same 299 frames is more precise than the cloud model (97.2% against
67.9%). So most identity questions can be answered on the rover, free,
with the internet unplugged. And so the cloud does not overrule a
confident local answer: it resolves what the local tiers *could not
answer* (3.80).

**Why a separate service.** It is the largest single user of the board's
memory, about half of the 8 GB the CPU and GPU share. The user set a floor
of 0.5 GB free with the VLM the first thing killed (2026-10-14). A process
of its own is what lets the kernel take the VLM and nothing else: the
robot server, the brain and perception keep running, and the identity
question goes to the cloud (3.80.B: its own process, so that an
out-of-memory kill takes it and nothing else). The price is a second local
service with its own liveness, start-up and wire format, which the brain
must be able to lose at any moment.

**Which questions go to it.** With a local VLM configured, every question
the tiered policy used to send to the cloud goes to the local VLM first:

- **The arrival question** (3.80.C): the lidar rule holds, the robot is
  stopped, "is this the target?". A confident yes ends the mission
  **found** with no cloud call; a confident no refuses the arrival, as the
  cloud's no does.
- **A frame's question during the search** (3.80.D): when perception has a
  candidate it is unsure of, when the target is a word the detector has no
  class for, and on each event that used to trigger a cloud call (mission
  start, a candidate sighting, a cold search, staleness). Here a yes also
  says where in the frame, and the policy may steer on it.
- **A search direction** ("left, right or forward?", 3.80.F) is built and
  **off**: its acceptance failed (the model said forward on 10 of 10
  frames). Where to look comes from the map
  ([policy](../policy/ARCHITECTURE.md), "Where to look is chosen from the
  map, never from a frame"). Training the model to steer is open work
  (3.188, 3.206).

**Which go to the cloud.**

- A question the local VLM was **unable** to answer: a score between its
  yes and its no, a reply that could not be read, its deadline passed, or
  it was busy with another question. After a small fixed number of unable
  answers in one arrival the cloud is asked.
- Every question while the local VLM is **absent** (below).
- A local "no" given where the model cannot tell (the candidate too small
  or too large in the frame, 3.91.C): the cloud is asked once.
- What was never the local VLM's: room names, and the late confirmation
  of an arrival that ended unconfirmed (3.53).

A higher tier's answer is never undone by a lower one (the user,
2026-10-24). Cloud questions keep the cloud's own pace and call cap, so a
lost local tier makes a mission slower and paid; it does not multiply the
cloud calls (3.152).

**When it is down.** The brain tells two failures apart, because they call
for opposite responses:

- **Unable**: the service is there and did not answer usefully, a hang
  included. Counted, as above. A wedged service costs a bounded number of
  questions, then the cloud is asked; a slow VLM is kept.
- **Absent**: there is nothing to ask. The connection is refused or
  dropped (a kill drops it mid-request), or the service says it is still
  loading or has lost its model. The question that found it absent is
  handed to the cloud, and the absence is **remembered** (3.152): until
  the service's free health check says it is ready, no question is sent
  to it and every decision is exactly the one a mission with no local
  tier makes. The first ready answer brings it back, with its counts
  starting afresh.

A mission never waits for the local VLM: not to start, not to continue.
If the cloud is unreachable as well, the search goes on with perception
alone, and an arrival ends **arrived_unconfirmed**, never **found**
([mission](../mission/ARCHITECTURE.md)).

**Down is not "off".** Whether a local tier is *configured* and whether it
is *up* are two facts, reported apart. A brain configured for a local VLM
that is down still says it has one, and its questions are going to the
paid cloud. The live state (up, down since when, unwatched, off) is in
the mission's status and on the twin, with a line for the operator while
it is down (3.152.A).

**Coming back.** The service ends itself when its model process dies, so
the board's service manager decides on a restart: after a delay, and only
a few times in a quarter of an hour. Past that it is left stopped, and
absent, until a person starts it. **Rejected:** restarting for ever with a
delay (3.152.B). In a crash loop where memory is short, every restart is a
load the size of half the board; no delay tried met the robot stack's
bars, and every one lost a mission in a memory squeeze.

**Where it runs.** On the board only, started at boot since 3.108 (the
user, 2026-10-26). A laptop, the automated suite and the cloud deployment
have no local VLM, and the shipped configuration leaves it off: there the
ladder is perception, then the cloud. On the simulator's frames an arrival
and a local "no" are judged from geometry (3.98, 3.143), so no model is
asked about a render (see "The simulator reports detections").
Measurements on the board hold the VLM off for their run and put it back
as they found it (3.108), because its load is the size of the thing most
of them measure.

## Contracts

| With | Direction | Category | Ownership |
|---|---|---|---|
| Tiered policy ([policy](../policy/ARCHITECTURE.md)) | policy calls the pipeline once per frame | in-process: frame in, tri-state result out | perception owns what was seen; the policy owns what to do |
| Arrival rule (policy) | reads the result's bearing and pan | in-process, via the scene | arrival refuses a panned bearing; perception only reports it. Holds on real frames only: sim frames carry the pan since 3.46, but the sim stand-in does not compose it and reports pan 0 with a camera-relative bearing (harmless while no tiered policy pans) |
| Mission service ([mission](../mission/ARCHITECTURE.md)), inventory | feeds the inventory every frame, with the pose and scan of that moment; serves and stores the report | in-process; a report out at mission end | the mission owns when and where it is stored; nothing in the inventory can fail a mission |
| World ([world](../world/ARCHITECTURE.md)) | supplies the map pose an observation is placed in | the pose and its map id | an observation from another map is refused, never mixed |
| Mission service ([mission](../mission/ARCHITECTURE.md)) | builds the pipeline at mission start | in-process construction | a pipeline that cannot load is a start-time refusal, never a mid-mission vision failure |
| Body ([body](../body/ARCHITECTURE.md)) | supplies the frame and its pan angle | frame dict | the body owns pixels and servo angles; perception never asks for more |
| Simulator ([simulator](../simulator/ARCHITECTURE.md)) | supplies reported detections in sim frames | frame data | the simulator owns visibility geometry |
| Recordings ([recordings](../recordings/ARCHITECTURE.md)) | instruments read walks and human labels | files | recordings owns the corpus and its labels |
| Cloud vision ([cloud-vision](../cloud-vision/ARCHITECTURE.md)) | none directly | -- | the cloud owns identity; perception never overrides it |
| Tiered policy ([policy](../policy/ARCHITECTURE.md)), local VLM | the policy asks the service through the brain's client: at arrival, on a frame's question | HTTP on the board's loopback, behind the shared secret; a frame and the target in, a status and a raw score out | the service owns the model and its answer; the policy owns the thresholds, the count of unable answers, and when the cloud is asked |
| Mission service ([mission](../mission/ARCHITECTURE.md)), local VLM | builds the client per mission and closes it with the policy; publishes the tier's live state | in-process construction; never probed at start | a local VLM that is not there never refuses or delays a mission's start |
| Operations ([operations](../operations/ARCHITECTURE.md)), local VLM | the board's service manager starts it at boot, restarts it within a limit, and kills it first when memory runs out | a service unit beside the robot stack's | operations owns the units and the kill order; this domain owns what the service does |
| Twin ([twin](../twin/ARCHITECTURE.md)), local VLM | reads the tier's state from the mission's status | HTTP, read-only | the twin shows it and decides nothing |

## Failure modes and resilience targets

| Failure | Response | Target |
|---|---|---|
| Model libraries not installed, or weights fail to load | the mission refuses to start, naming the install command | never discovered after a motor turns |
| Camera frame missing or undecodable; a model raises | **unavailable** | a broken sensor never reads as an empty room |
| Target the detector has no word for | open-vocabulary path; the vocabulary verdict says so; the cloud's cold search covers it | absence from a blind detector is never trusted as absence |
| Close-range relabelling | crops are not gated on label | the frames where the target fills the view are kept |
| Local false positive | each frame that passes the gate is a sighting. It may fire a cloud trigger. It steers the robot on that frame, because steering has no frame hysteresis. Near the object it can satisfy arrival. | for steering, the per-frame probability gate is the only bound today. For ending `found`, the arrival rule adds consecutive frames, a centred bearing, a lidar range within the radius and one surface -- all local. The cloud's identity does not override a local sighting. Before `found`, the cloud must confirm identity on the arrival frame. The target "a false positive never confirms a target" is **met** on the tiered arrival path (the cloud confirms identity at arrival; decided by the user 2026-10-02 and built the same day, owned by [policy](../policy/ARCHITECTURE.md)). A false positive can still steer the robot to the wrong object |
| Camera panned | on real frames: pan is added to the bearing; tilt is not corrected; arrival refuses to judge. The sim stand-in does not compose the pan its frames carry, so neither happens in the target pipeline in the sim (harmless today: the cloud-driven policies never peek and a mission starts centred); the inventory does compose it | no panned bearing is treated as body-relative without composing it with range |
| Over the latency budget on the board | P26, then TensorRT | 250 ms a frame at 15 W |
| Inventory: no range, no pose, or another map | the observation is counted and not placed | no object placed without a lidar range in the current map |
| Inventory: the detector errs (drops, wrong labels) | belief needs two independent looks; a wrong label is a minority vote | reported landmarks within 1 m of a real object of their class: 95% (3.46) |
| Inventory: the upload fails | logged; the local file stands | never fails or delays the end of a mission |
| Local VLM absent: not installed, loading, killed for memory, its model process dead | the question goes to the cloud; the absence is held and health-checked until it is ready again; the policy is the no-local-tier policy meanwhile | no mission waits on it; nothing is sent to a service known to be down (3.152) |
| Local VLM unable: a hang, an unreadable reply, a score between yes and no, busy | counted; after the limit the cloud is asked | a wedged VLM costs a bounded number of questions, never a mission |
| Local VLM down and the cloud unreachable | the search continues on perception; an arrival ends **arrived_unconfirmed** | never **found** without an identity answer |
| Local VLM keeps dying (a memory crash loop) | restarted a few times, then left stopped until a person starts it | a load of half the board's memory is not repeated for ever |
| Memory runs out on the board | the kernel kills the local VLM first | the robot stack is never killed before it; 0.5 GB free is the floor (the user, 2026-10-14) |
| Local VLM down, unnoticed | the mission's status and the twin say down, and since when | "configured" is never read as "up" |
| A real camera mistaken for the sim | provenance read once; failure assumes a real camera | real frames never get synthetic detections |

## Open questions

- **Enforce corroboration (1.11a)?** Decided by the user after two
  out-of-vocabulary searches and one control walk under the tiered policy
  (`PLAN-onboard-perception.md`, "What to record next"). Not as the guard on
  arrival: that was decided 2026-10-02 in favour of a cloud identity check
  ([policy](../policy/ARCHITECTURE.md)).
- **Is the matcher or the proposer the weak half?** The labels carry
  per-frame booleans and no boxes, so the two failures are not separable on
  this corpus.
- **INT8 on the Jetson** is unmeasured; TensorRT INT8 destroyed OWLv2 (P7d).
- **Tilt correction** needs camera intrinsics nobody has measured, and the
  Rover's camera differs from the one the field of view was set for.
- **The inventory on the car.** Real frames carry no detections of
  anything but the target until an on-board detector that names
  everything exists (3.46 C was closed on 2026-10-10 after every namer tried
  failed its bar), so today the inventory fills only in the simulator.
- **Should a tied class vote be reported?** Since one miss is forgiven
  (3.46 amendment 2, recall up), a landmark whose labels tie can be
  reported under the wrong one; 3.46 records one such case. Unchanged
  until it is judged on missions it has not seen.
- **Can the local VLM steer?** Asked for a direction it failed (3.80.F);
  its box steers only while the target is in view. Training it on a
  frontier model's direction labels is open (3.188, 3.206). A larger open
  teacher was measured first (3.214): Qwen2.5-VL-72B missed the steering
  bar by one frame (54 of 57; the user reads it as inside chance) and
  Alpamayo 2 Super failed every bar. Then 3.206.B0-B1: Florence-2, the
  72B and Qwen3-VL-235B (Bedrock), alone and in pairs, each fail the 95%
  bar by one to four frames, every miss on a frame whose label sits on a
  third's line; how such frames count is decided before the next test set
  is measured. No teacher is proposed yet.
- **More image detail.** The board sees at most 512 image tokens (about
  0.4 MP) whatever the camera's resolution; the camera is not the limit.
  A crop around a candidate is the untested way to use more detail
  (3.214.F); a larger whole frame costs latency and did not help a model
  tuned to a smaller budget (3.214.D2).
- **The local VLM on the Rover's own camera.** Every accuracy number it
  has comes from phone frames shot at floor height; 3.80.U re-measures
  them on the Rover's frames.
- **Its own process** (2.7: features, not frames) so a stalled detector
  degrades perception rather than the control loop -- proposed, not built
  (see "Perception runs in the brain process").
