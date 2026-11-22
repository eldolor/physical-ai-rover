---
kind: architecture
domain: twin
status: current
verified: 2026-11-22
---

# Web twin -- architecture

The web twin is the project's one user interface. It is a phone-first web
page, and it is a real HTTP client of the robot server, the brain service
and the cloud vision service. It holds no movement, safety, decision or
map logic of its own. It shows what those servers publish, and it lets a
person drive, start missions, walk a real house with the phone's camera,
and configure where everything lives. Read this for what the page may and
may not do, and why. Read the
[engineering spec](../engineering/twin/ENGINEERING.md) for files, routes,
constants and the test suite.

## Purpose

The page tests three different layers, and they are not the same question
asked three times:

- **Home and Drive** (one tab, Sim, until `3.132-ux-redesign.md` Stage 3)
  exercise the control software, against the simulator today and the car
  later: mission memory, the safety collar, the failsafes, driver
  arbitration, and the discovered map. Against the simulator they are also
  the only safe place to fire the failsafe drills.
- **Guide me** has nothing to do with the robot. A person is the actuator
  and the phone is the sensor. It answers the one question the simulator
  cannot: can the vision model read a real room?
- **Robot view** asks the same real-pixel question as Guide me, but in
  the robot's vocabulary: "what move would you make from here". It can
  also record the walk for later replay (the recordings domain). It can
  also drive a real brain mission closed-loop, with the person as the
  motor ("Drive via brain").

People depend on it to operate the system from a phone on any network.
Developers depend on it as a regression surface: every UI bug this project
shipped was found on a phone, not by the Python suite. Without it, the
brain and robot remain reachable only by command line, and the real-pixel
walks that the corpus is made of could not be recorded.

## Components and boundaries

```text
 +------------------------------- web twin (browser) --------------------------------+
 | battery and shutdown banner (every tab)                                           |
 | Home             | Drive               | Record              | Settings           |
 |  Find or Survey  |  D-pad, turn step,  |  Guide me: steer a  |  robot, brain and  |
 |  Start or Stop   |  look               |   person            |  vision addresses  |
 |  mission card,   |  camera frame,      |  Robot view: the    |  and secrets,      |
 |  trip home,      |  depth strip,       |   robot's move      |  health, setup     |
 |  what it saw,    |  odometry, the map  |   + Record this walk|  code, schedules,  |
 |  next schedule   |  (goals, rooms,     |   + Drive via brain |  Shut down,        |
 |  (telemetry rows |  no-go zones,       |                     |  Engineer mode,    |
 |  in Engineer     |  re-map)            |                     |  debug             |
 |  mode)           |                     |                     |                    |
 | STOP bar (every tab, while anything moves)                                        |
 +------------------+---------------------+---------------------+--------------------+
        |                     |                      |                    |
        v                     v                      v                    v
  brain service          robot server         cloud vision service   robot server and
  (missions, what the    (body, safety,       (same origin when      brain service
  robot keeps)           world, battery)      deployed)
```

- **Home.** Starts and observes a mission running in the brain service:
  the job (find an object, or survey the house), Start or Stop, and one
  sentence that names the mission's state. Under it: the trip back to the
  dock, the picture the robot judged to be the target, and the next
  scheduled run. The telemetry rows are Engineer mode's.
- **Drive.** Manual control, readouts of what the robot senses, and the
  discovered map. On a saved map the map is also where a person sends a
  goal, names rooms, draws no-go zones and asks for a re-map.
- **Record.** Opens the phone camera full-screen and runs a timed loop of
  vision calls, drawing an overlay from each answer. It was the Guide tab
  until Stage 3.
- **Settings.** The three endpoints and their secrets, per-endpoint
  connection checks, the combined health line, a setup QR code that
  configures a second phone, schedules, the Shut down button, the
  Engineer mode switch, and a debug log.
- **On every tab.** The battery and shutdown banner at the top, and a STOP
  bar above the tab bar while anything moves.

**Boundaries.** The page may not:

- **Move the robot except through the robot server's command routes.**
  Safety, arbitration and the watchdog all live on the server. Direct
  commands (the D-pad, look and stop) name the page as their driver, and
  since 2026-10-05 so does a goal tapped on the map: a person's tap is a
  person driving, and outranks a running mission (see "A person outranks
  the brain"; handoff 3b).
- **Decide anything a policy decides.** Missions run in the brain service
  and survive the tab closing.
- **Hold a copy of the house, the renderer, or any physical constant.**
  The map is the server's discovered map. The camera picture is the
  server's frame. The safety threshold it displays is read from the
  server.
- **Invent a verdict or a value.** It renders what a server reported. It
  says "not reported by this server" where a route is missing, and it
  draws nothing where there are no pixels, because blank and "no
  obstacles" must not look alike.
- **Talk to a simulator or a robot backend directly.**

## Decisions

### A thin client: no logic that a server already owns

**Decision.** The page carries rendering, input and configuration only.
On 2026-09-25 the ROS alignment deleted the in-page "local brain" (a JS
frontier explorer and a JS vision autopilot), the hardcoded copy of the
starter house, and the JS raycaster (`PLAN-ros-alignment.md` 3.2).

**Rejected.** Keeping the JS brain as a no-backend development path. Once
the twist multiplexer puts exactly one writer between any driver and the
wheels, a brain that dies with a browser tab cannot be that writer. Also
rejected: keeping the JS raycaster. It read a cell and a cardinal heading.
Under a continuous pose it drew a direction the robot was not facing, and
one call site fed that picture to the model (`tests/test_frame_source.py`
records the history).

**Trade-off.** The Sim tab now needs the brain service running to show
autonomy. The server's refusal of a second mission and the driver
priority order enforce "one brain at a time", where the page used to.

### Draw what the server publishes, and say so when it publishes nothing

**Decision.** Every readout comes from a route. Each one distinguishes
three states: a value, "this server does not report that", and "no
reading". Examples:

- The frame-source line says the server sent the frame, or that none
  came.
- Odometry says it has no encoders rather than showing zero.
- The depth strip outlines exactly the zones the server says the veto
  reads.
- The health line renders per-half answers using the verdict rule that
  belongs to the operations domain's health command.

**Rejected.** Computing a reading locally where the server's is missing,
or reducing the depth grid in the page. A second copy of a rule drifts,
and the page is the copy people believe.

### Phone-first, and tested in a real browser at phone width

**Decision.** The layout targets a phone viewport. The automated UI tests
drive the real page in a real browser at that width.

**Rejected.** DOM-only tests (jsdom). The two bugs that motivated the
suite were a model picker that never populated and one that rendered as a
40 px chevron. The second exists only under real layout. Each UI test is
confirmed red against its bug before it is trusted.

**Trade-off.** The UI tests skip without an installed browser, so a green
run on a fresh machine may have tested nothing here.

### A live camera with a 2D overlay, not WebXR

**Decision.** Camera access plus DOM and SVG overlays positioned over the
video.

**Rejected.** WebXR. Its support on iPhone was unsettled when this was
designed, and "AR-style" meant a directional arrow, not spatial anchoring
(`PLAN-ar-guidance.md` section 2).

**Consequence.** The page must be served from a secure origin (HTTPS or
localhost), and that drove the hosting decision below.

### Static hosting on the CDN, same origin as the vision service; the robot and brain through a tunnel

**Decision.** The deployed page is static files on a private bucket
behind the same CDN distribution that fronts the vision service. Vision
calls are same-origin and never preflighted. The robot and brain run on
the developer's machine. A single tunnel domain reaches them, split by
path (the operations domain owns the tunnel).

**Rejected:**

- **The page served by the robot server on ECS behind the shared load
  balancers.** This is how it ran until 2026-09-05. It was deleted with
  the VPC (`PLAN-aws-cost-redesign.md`).
- **Third-party static hosts,** which `PLAN-ar-guidance.md` section 2
  listed. Same-origin with the vision service is what makes vision calls
  work without CORS.
- **Opening CORS on the vision service.**

**Trade-off.** Locally the robot server still serves the page, so there
are two ways to load it. A saved endpoint can follow a person between
deployments. The page shows a notice whenever an endpoint's host differs
from the page's, except when the page itself was loaded from the
developer's own machine, where splitting the page and the services across
ports is the normal layout. It is a notice and not an error, because a
tunnel is a legitimate reason for them to differ.

### No content-hashed filenames: revalidate every load, and publish an explicit asset list

**Decision.** On the deployed site, pages and scripts are served
"revalidate every time". An explicit manifest lists every published file
with its type and cache policy. A test checks the manifest against what
the pages reference. Locally the robot server marks the page, the scripts and the
stylesheets "revalidate" too, so the decision's guarantee (page and
script never a version apart) holds on both.

**Rejected.** Hashed filenames. They need HTML templating and a build
step, for one page served to one household. Also rejected: a guessing
upload tool. The console's page has no extension, so a guessed type would
download it instead of rendering it.

**Why the test.** A file the page references but nobody published loads
the page with one feature silently dead. That has shipped five times.

### Throughput for a person, one-at-a-time for a robot

**Decision.** In Guide me and Robot view, vision calls overlap (a small
fixed number in flight), so a person walking gets a steady stream of
decisions. An answer overtaken by a newer one is discarded. An answer from
a previous session is discarded. Calls in flight count against the
paid-call budget, so overlap can never overshoot it. When a real mission
is driving (Drive via brain), the page allows one call in flight.

**Rejected.** A strictly serial loop. Its real cadence was the throttle
plus the model's latency, about 3 s on the model in use when this was
decided (dated; not re-measured on today's default). Also rejected:
unbounded overlap, which Bedrock throttles and which competes with
production for the account's quota.

**Trade-off.** Overlap raises throughput, not freshness. Each answer still
describes a frame one round trip old. A robot executing every answer
would be worse off, hence the single in-flight call under a mission.

### A person outranks the brain, and the page says who it is

**Decision.** Every command from the D-pad names the D-pad as its driver.
The server's priority order lets a tap pre-empt a mission (the safety
domain). The page shows who holds the robot and the last refusal with its
reason.

**Rejected.** Unnamed commands where the last writer wins.

**A tapped goal is a person driving too** (decided by the user 2026-10-05,
handoff 3b). The page names the D-pad as the driver of a goal tapped on the
map, so the server ranks it as a person: a tap during a mission takes the
robot, and the mission ends pre-empted, as a D-pad press does. A goal sent
by a program that names no one stays an autonomous navigation goal (the
safety domain's rule). The robot STOP button is not the same thing: it
holds the motors and claims no authority; the one thing it ends is a
navigation goal (decided by the user 2026-10-02), and a non-zero D-pad
movement cancels one too. To resume, a person sends the goal again. The
rule belongs to the [safety domain](../safety/ARCHITECTURE.md).

**Rejected.** Keeping the tap autonomous, refused while a mission holds
the robot: it contradicted the order this section exists for, and left a
person who had already said where the robot should go with no way to say
it except the D-pad.

**While explore asks, a tap on the map is a pick, never a goal** (the
user, 2026-10-20; `3.88-goal-tap-learning.md`). With "Ask me at each
goal" on, explore pauses at each choice and the page draws its candidates
on the map. A tap on one sends that pick to the brain. A tap anywhere
else sends nothing.

**Rejected.** Sending the tap as the person's own map goal. That goal
outranks the brain, so a correction would be a takeover and would end the
mission it was meant to teach.

### Two voices on one page, four tabs, and a STOP bar on all of them

**Decision** (`3.132-ux-redesign.md`, approved by the user 2026-10-31).
The page's first reader is a person at home with the robot, on a phone,
with no project knowledge. By default it speaks to them: the brain is
"the autopilot", an ending is a sentence, and the engineering detail is
hidden. Engineer mode, a switch in Settings, shows every readout in the
project's own words. Every string the page says exists in both voices in
one catalogue. The layout follows the tasks (Stage 3): Home to start and
watch a mission, Drive to drive by hand, Record for the camera walks,
Settings. While anything moves, a STOP bar is one tap away on every tab.

**Why the page and not a server.** Wording is the page's job. The servers
publish states and reasons by name, and the page chooses the words.

**Rejected:**

- **One page in the engineer's words.** This is what was built first. The
  review behind 3.132 found toasts printing raw outcome names, a HUD of
  fifteen three-letter codes, an idle robot's watchdog in alarm red, and a
  mission that could be watched only by scrolling past the D-pad.
- **A second page for operators.** One page keeps one set of calls and
  one set of tests. Engineer mode keeps every readout.
- **A framework and a build step.** The user's choice: plain scripts, and
  the deploy unchanged.
- **A STOP that does something new.** The bar calls only what a person's
  STOP already called, so arbitration, the person-Stop latch and the
  safety path are unchanged.
- **Changing the driver ranks so that a Start right after driving is not
  refused** (3.132 finding F2). A person's claim on the robot outlives
  their last press by the watchdog's period, so a Start pressed straight
  after driving into place ended at once, in the person's own name. The
  page now waits for the claim to clear and says so on the button. The
  ranks stay the safety domain's.

**Trade-off.** The UI suite was written against the engineer's wording,
so its tests open in Engineer mode unless they ask for the operator view.
Engineer mode's words fail the plain-language check on purpose.

### A mission's ending is an outcome, never an error; a survey is a job, not a policy

**Decision** (`3.70-survey-and-mission-ends.md`, the user 2026-10-11; S5
and E6). The brain decides how a mission ends and the page shows each
ending in its own words and style: Found in the success style, Exhausted
and Out of budget in the warning style, Cancelled uncoloured. An arrival
whose identity the cloud could not check is never shown as Found. No
ending is drawn as an error toast or a lost link.

A survey is offered beside Find as the other job, not as one more policy:
it is the brain's own frontier search with no target. Choosing it hides
the target, policy and drill controls, and states the brain's cap on
room-name calls before Start, because on the car each of those is a paid
call. While it runs the page names it a survey and lists the rooms named
against that cap. A survey started from another phone sets the picker.

"Where it looked" gives the rooms entered and the floor mapped as an
area.

**Rejected.** Showing the report's coverage as a share of the house. The
brain cannot count floor its map has not reached: part way through a live
search the estimate was 69 and 77 points above the truth (E4, recorded as
failed), and the page is the copy people believe.

**Failure mode.** When the cap is reached the row says so in the warning
style, and when nothing can name rooms it says so in the alarm style.

### The trip home is its own line; the outcome stands

**Decision** (`3.156-return-to-dock.md` D2, D6 and step E, 2026-11-12).
After a mission has decided its ending, the brain drives the robot back
to its dock. The brain decides the outcome first and the return never
changes it, so the page shows the return as a line of its own under the
mission's sentence: going back, back on the dock, could not get back,
stopped going back, or did not go back, the last three with the reason in
plain words. A mission that paused to charge says so, with the level it
waits for, and that it will carry on. Engineer mode shows the same state
with the brain's own reason, and where the dock's position came from: the
saved map's own dock, or only the place this mission started.

**Why the source is shown.** A dock taken from the mission's start is the
best one known, not a verified one (D1), and a return to it must not read
as a return to the charger.

**Rejected.** Folding the return into the outcome ("Found, and docked").
A find that succeeded and then could not get home would read as a failed
find, or a failed return would hide behind a green Found.

**Failure modes.** A return that was skipped says why (no route planner
on this robot, no known dock); it is never silent. A reason the page does
not know reads "something went wrong", not a raw name. When the brain is
unreachable the line is hidden, because the last status is no longer
news. A resumed mission shows no return, since the brain clears it.

### The battery level is the robot server's, and its last word stays on screen

**Decision** (`3.137-battery-ladder.md` step B, 2026-11-02). The robot
server reads the pack and owns the three levels; the brain is told and
obeys, and the page only shows. A banner at the top of every tab, with no
tap, says: battery low, and a new search cannot start; battery reserve,
with the countdown to the robot's own power-off, and that a person can
still drive slowly to the charger; powered off, unplug the battery. The
volts and the levels' thresholds are Engineer mode's. A Start refused at
Low is worded as a low battery, not as the brain's refusal code.

**Why "unplug the battery".** A software power-off leaves the motor board
draining unprotected cells (`HARDWARE-BOM.md` section 6), so the robot's
last instruction is the one thing only a person can do.

**Why it outlives the robot's answers.** Power-off is the moment the
robot stops answering, so the banner cannot depend on an answer. The
powered-off message stays through silence and a reload, and clears only
when the same robot answers again with a settled reading at another
level. A reserve countdown that runs out while the robot is quiet becomes
powered-off, because the robot powers off at its end regardless.

**Rejected:**

- **Judging the level in the page from the volts.** A second copy of the
  ladder would drift, and the ladder reads twice, at rest, exactly so
  that a sag under motor load is not taken for a flat pack.
- **A banner that follows the latest answer only.** The first build hid
  the level on one missed read and hid the powered-off message when the
  robot went quiet (found in review, fixed red first).
- **A readout in Settings or on one tab.** A person recording a walk or
  driving would not see it.

**Shutting down by hand** (`3.170-twin-shutdown-button.md`, the user
2026-11-08). The Rover's power button is a hard cut, so Settings has a
Shut down button that needs only the robot server. It asks first and
names any search it would end. The same banner then says how far the
shutdown got. The phone cannot see the power, and a dropped connection
looks the same as a finished shutdown, so even the best case tells the
person to wait for the light on the robot's computer. A shutdown asked by
a person is kept apart from the battery levels, so it is never read as a
flat pack.

**Rejected.** Saying "safe" on silence alone. Silence before the
power-off began proves nothing, and the page says it cannot tell.

### What the robot keeps about a home is edited from the page and stays on the robot

**Decision** (the user, 2026-11-06, from what a robot vacuum's app
offers; `3.160-rooms-named-on-twin.md`, `3.161-no-go-zones-on-twin.md`,
`3.162-what-i-saw.md`, `3.163-scheduled-missions.md`). The page is where
a person names rooms, draws no-go zones, sets schedules, and looks at
what a mission saw. The brain keeps all of it on the robot's own disk,
with the saved map where it belongs to one. The page holds none of it and
uploads none of it: a home's floor plan, routine and pictures reach only
the person's own phone, over the brain's authenticated routes.

- **Room names and zones need a saved map.** Without one the page says
  "Survey first" and offers no tools.

  **Rejected.** Keeping them for one session: a name or a zone that
  silently disappears is worse than none.
- **A no-go zone is not a safety stop**, and the page says what it is.
  Autonomy never plans into one, and a goal tapped in or too close to one
  is refused with words that say so. The D-pad is not refused. A zone the
  robot is inside, or one over its dock, is drawn dashed and called
  paused or ignored, because the robot must be able to leave.

  **Rejected.** Drawing every stored zone as in force.
- **What it saw.** After a mission, Home shows the picture the robot
  judged to be the target and says whether its identity was confirmed;
  people trust what they can check (3.162). A person can delete a
  mission's pictures, in two taps.

  **Failure mode.** On real frames only the target is named, so on the
  car the list is the target's sightings until a namer for camera frames
  exists.
- **Schedules.** The page sets them and the brain runs them. Home says
  the next run, that schedules are off, or why the last run was skipped,
  in plain words. A schedule never lifts a person's Stop, and on a real
  robot schedules stay off until the unattended checks pass, on the
  user's word; the panel says so.
- **While an editor is open, a tap on the map is about rooms or zones
  only**, never a goal.

**Trade-off.** Edits from two phones are not merged. A room edit made
against an older list is refused and the rooms reloaded; the schedule
list is last write wins.

### Configuration lives on the device, and shares by QR, never through a third party

**Decision.** Endpoints, secrets and preferences persist in the browser's
storage. Every access is guarded, because private browsing can throw.
"Show setup code" draws a QR code of a magic link carrying the secrets.
The code is generated in-page, shown only on a tap, and hidden again
on a short timer or when the person leaves the tab.

**Rejected.** A QR web service or library CDN. Handing the secrets to a
third-party image API would leak them.

### Tunnel workarounds are scoped to the tunnel

**Decision.** The header that skips the free tunnel's HTML interstitial
is sent only to that provider's hostnames.

**Rejected.** Sending it everywhere. It is a custom header, so it would
force a CORS preflight on the frequent health poll of every LAN
setup.

## Contracts

| Neighbour | Direction | Protocol | Ownership |
|---|---|---|---|
| Robot server | page calls server | HTTP/JSON, robot secret header, driver header on direct commands and on a goal tapped on the map | Server owns pose, map, frame, depth, odometry, safety, arbitration, the watchdog, the battery levels and the shutdown. The page renders them. |
| Brain service | page calls brain | HTTP/JSON, the brain's own secret | Brain owns the mission and its status, the return to the dock included. The page starts and stops it and polls the status. The brain owns recording when the page records, and it owns what the robot keeps about a home: room names, no-go zones, schedules and each mission's pictures. The page edits them through the brain and stores none. |
| Cloud vision service | page calls service | HTTP/JSON, vision secret, same origin when deployed | Service owns model ids, wordings and answer vocabularies. The pickers fill from it. |
| Static hosting and CDN | page is served from it | HTTPS | Operations owns the deploy. The asset manifest owns the file list. |
| Tunnel | page calls through it | HTTPS, one domain split by path | Operations owns it. The page only adds the interstitial header for tunnel hosts. |
| Recordings | page writes frames and a finish marker through the brain URL | HTTP/JSON | Page owns the walk name and frame numbering. Recordings owns storage. |

## Failure modes and resilience targets

| Failure | Behaviour | Target |
|---|---|---|
| The brain stops answering | Marks it lost, disables Start with the reason, retries on its own | A restarted brain reconnects without a person pressing Connect |
| The brain answers with an error | Stays connected and shows the error | "Answered badly" is never shown as "gone" |
| A secret is rejected by a different deployment | The error names the host it went to | A 401 is never misread as a wrong secret when the endpoint is wrong |
| The vision service is healthy but cross-origin | The connection check names the origin as the fault | "Load failed" always has a diagnosis. Not met precisely: the check flags EVERY cross-origin vision URL as publishing no cross-origin headers, including a local service and the tunnel's vision route, which do publish them and work. The diagnosis is right only for the deployed service. |
| The tab is backgrounded mid-mission | The mission continues in the brain, and the page catches up on return | Closing the page never stops or orphans a mission |
| A stale call returns after Stop or a newer answer | Dropped before it can touch the overlay or the recorder | No previous session's answer is ever drawn |
| A forgotten Guide session | Pauses at the per-session call cap | Unattended spend is bounded |
| A server older than a readout's route | "Not reported by this server" | Absence never reads as zero or clear |
| A frame with no pixels | Draws nothing and says so | The page never shows a picture it invented |
| Recording with no brain connected | A toast says recording is off | A walk is never silently unrecorded |
| The robot powers off, or stops answering at Reserve | The banner keeps the last level; a countdown that ends in silence becomes powered-off; it clears only on a settled reading from the same robot | The robot's last word is never lost to the silence it causes. Not met across addresses: it is remembered for one robot address, so the same robot reached another way misses it |
| A low-battery reading nobody confirms | Dropped after a few missed reads | A stale Low never blocks a person indefinitely |
| The robot goes quiet during a shutdown a person asked for | Before the power-off began: "can't tell". After: wait, then safe once the light is out | The page never says "safe" on silence alone |
| The return to the dock fails or is skipped | Its own line says so with the reason; the mission's outcome is unchanged | A failed trip home never recolours a successful find |
| The robot's local vision model is down during a mission | One plain line under the mission's sentence says it is restarting and that cloud checks cost a little; Engineer mode shows the state by name, and only while the mission runs (`3.152-local-vlm-down.md` criterion 8) | A paid fallback is never silent, and an ended mission's stale state is never shown |
| An insecure origin for the camera | The start error explains the secure-context rule | |
| A stale script after a deploy | No-cache plus a CDN invalidation | A redeploy never runs an old script against new HTML |

## Open questions

- **The environment banner fires on the deployed twin only if the stack
  is labelled.** It asks the origin that served the page. Under the CDN
  that origin is the vision service, which reports an environment label
  since 3.64 (`3.64-older-review-bugs.md` B1), empty unless the stack sets
  one; 3.64's deploy left it unset. Whether the live stack has one is
  unchecked, and operations decides.
- **The health line re-implements the verdict thresholds** rather than
  reading a verdict. It has already drifted from the command
  (`control/health.py`): the page judges only the watchdog loop and the
  brain's tick against its bare deadline, where the command also counts a
  robot with ROS down under the ROS drive (handoff 4e) and gives the tick
  deadline a second of slack (3.76 N1). Whether the servers should publish
  the verdict itself is open.
- **The dock is not drawn on the map.** `3.156-return-to-dock.md` D6
  names the dock "painted on the twin's map"; the page draws no dock
  mark, only the return's words. Whether the map should show it is the
  dock section's to decide.
- **The battery banner at Low is covered by the Record tab's full-screen
  camera.** Reserve and powered-off float above it. Whether Low should
  too is undecided (3.137.B's review recorded it).
- **The S1 "Check robot contract" button** is still owed
  (`CLAUDE.md` section 7).
- **Press-and-hold wheel velocity from the D-pad** waits for the twist
  path to be the default drive (`PLAN-ros-alignment.md` R4/3.24).
