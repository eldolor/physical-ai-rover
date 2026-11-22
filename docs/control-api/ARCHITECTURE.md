---
kind: architecture
domain: control-api
status: current
verified: 2026-11-22
---

# Control API (the robot server) -- architecture

The robot server is the robot's runtime process and its one network
surface. The brain, the web twin, the ROS container's bridge, the phone on
a teleop walk and the health command all reach the robot through it, over
HTTP. Inside it sit the one body, the world model, the safety layer, the
driver arbitration, the watchdog, the control loop that keeps a standing
wheel command vetted, and the battery's levels with the clean power-off. Read this for what the server is responsible for and
why it is a separate process behind plain HTTP. The routes, payloads,
status codes and environment are in the
[engineering spec](../engineering/control-api/ENGINEERING.md).

## Purpose

Two things must hold whoever is driving: every motion is vetted on the
robot, and the motors stop when the driver goes quiet. The robot server is
where both happen. It also lets every client be written once. The brain
drives the server the same way on a laptop, on the Jetson, or through a
tunnel from a phone; only the address changes.

Who depends on it:

- **The brain service** ([mission](../mission/ARCHITECTURE.md)), through the
  remote [body](../body/ARCHITECTURE.md) and remote
  [world](../world/ARCHITECTURE.md) clients.
- **The twin** ([twin](../twin/ARCHITECTURE.md)), which drives the D-pad,
  draws the sensors and the map, and is served from here when run locally.
- **The ROS container** ([ros](../ros/ARCHITECTURE.md)): its actuator plugin
  reads and writes the wheels here, and its bridge reads the scan.
- **The health command and the tunnel**
  ([operations](../operations/ARCHITECTURE.md)).

## Components and boundaries

```text
            brain      twin/phone     ROS bridge     health command
               \           |              |              /
                +----------+---- HTTP/JSON+-------------+
                                 |
   +--------------------------- robot server ---------------------------+
   | access gate (shared secret)     CORS                                |
   | motion routes ----> arbitration ----> safety vet ----> body          |
   | body sensing routes -------------------------------> body          |
   | world routes (pass-through) --------------------> world model       |
   | autonomy routes (Stop's latch, mission start/end, brain heartbeat)   |
   | battery and shutdown ------> battery ladder ------> clean power-off |
   | teleop ingress, sim-only routes, health, the twin's static files     |
   | control loops: watchdog (silence -> stop), wheel loop (re-vet),     |
   |                battery loop (read the pack, move the ladder)        |
   +--------------------------------------------------------------------+
```

| Part | Owns | May not |
|---|---|---|
| Motion routes (verbs, stop, standing wheel command) | Entry for anything that moves the robot | Move anything without arbitration and the vet. Three things skip arbitration only: a stop; under ROS drive, the ROS actuator's standing commands, which are still vetted; and a zero standing command, which can only stop the wheels and only for the driver holding them |
| Body sensing routes | One route per sensor, answering exactly what the body said | Compute a reading of their own |
| World routes | The house map, pose, goals and (in the sim) ground truth, under their own namespace | Compute world state. They pass through to the world model. The one exception is the sim-only error readout, which compares two of the world model's own answers (estimate against truth) and is never an input to a decision |
| No-go zones, keepout marks, re-map | A copy of a person's no-go zones for the saved map the robot is on, passed on to the planner; the marks this server makes where its own vet keeps refusing the planner, and their clearing at a mission's end; a person's request to map again | Veto motion. A zone or a mark changes where the planner plans, never what the vet allows: the D-pad is never refused for one (3.161, 3.68, 3.50.D4) |
| Autonomy routes | The latch a person's Stop sets on autonomy, its lifting at a mission's start and its putting back when that start fails; the mission's start and end as the brain reports them; the brain's heartbeat and the lease it gives the brain's own goal | Move anything, or let autonomy drive: each can only refuse autonomy, record, or end a goal (3.84, 3.145, 3.164, 3.171) |
| Battery and shutdown | The pack's reading, the level it is at, the refusals each level brings, and the one clean power-off, whether the battery or a person asks for it | Let anyone override a power-off once begun; be open to a caller without the secret wherever a real power-off is possible (3.137, 3.170) |
| Teleop ingress | Accepting a phone's frame for a teleop body | Accept a frame no body can read |
| Sim-only routes | Inspecting and rearranging the simulated house, and putting the plain simulator in another one (3.72) | Exist silently off the simulator |
| Control loops | The watchdog, the wheel loop and the battery loop | Block behind a motion in progress (that would freeze every route, stop included) |
| Health and identity | Facts about this process and the robot | Decide a verdict. That belongs to the health command |

The server never imports the brain (`control/`), never imports ROS, and
never names a backend type, with one exception: the sim-only house swap
checks that it holds the plain in-process simulator before it touches it
(3.72; "No reset" below). Everywhere else it discovers what a body can do by
asking, and answers "unsupported" by name when it cannot.

## Decisions

### The robot server and the brain are two processes

This domain owns the process topology; what a silent driver means, and the
order in which drivers win, belong to [safety](../safety/ARCHITECTURE.md).
Even on one board. **Rejected:** running the mission loop as a task inside
this server. It is less code. But a synchronous block in the brain would
freeze the event loop the watchdog runs on, it merges the robot runtime with
the decision-maker, and it loses the property that moving the brain is an
address change. The cost is one local HTTP round trip per call, against a
loop that waits seconds on vision (`PLAN-brain-relocation.md`, "Why not one
process").

### Plain HTTP/JSON is the wall to everything, ROS included

The ROS stack reaches the wheels the way every other client does: over HTTP,
through arbitration and the vet. **Rejected:** ROS inside this process, or a
ROS plugin owning the wheels. Either takes the safety layer out of the
navigation path, and either makes this process depend on ROS being healthy
(`PLAN-mapping.md` section 4, "(b+)"; `PLAN-ros-alignment.md` 3.15-3.16). A
containment test forbids ROS imports outside the container.

### Backend-agnostic by construction

The server holds whatever body the factory chose and never checks its type.
Optional abilities (accepting pushed frames, having a simulated house,
taking navigation goals) are discovered by asking. When an ability is
missing, the route says so with an error that names it. **Rejected:**
type checks per backend, which would make the hardware swap edit this file.
**Rejected:** silent no-ops, which read as success. The one type check is
the sim-only house swap's (3.72), which must refuse every body but the plain
simulator.

### Body and world are separate namespaces

Every body route answers "about me". The world routes live under their own
prefix and answer "about the house". The prefix is the seam, not decoration:
a reader who has to ask which kind a route is has already lost the
distinction (`PLAN-mapping.md` N1). The world routes are pass-throughs in
this process. **Rejected:** a separate world service with its own address.
The map's durability is a storage decision, not a process-topology one
(`PLAN-mapping.md` section 4).

### One route per sensor

The depth grid, odometry, wheel state, scan and camera each have their own
route. **Rejected:** folding depth into the camera frame. A wedged camera
must not take the clearance reading down with it (M2, M9).

### The server publishes the safety layer's own reductions

Which depth zones are "the path", what they read, whether a forward move
would be vetoed and the stopping floor in force are published by this
server. **Rejected:** letting the twin recompute them. Safety logic
duplicated in a browser drifts from the real one (CLAUDE.md section 6).

### A shared secret is the access control; CORS is permissive

Once reachable from the internet, motion and sensing routes require a
shared-secret header. The page itself and the health route stay open: a
monitoring probe must be able to ask whether the robot is alive without
holding the secret, and a person must load the page before typing the
secret into it. (The health route was first left open for a cloud load
balancer's health check, which cannot send headers. That load balancer was
deleted on 2026-09-05; the reason survives it for any external probe.) **Rejected:** CORS as access control.
It constrains browsers, not attackers. Unset, the gate is inert, which is
how local development and the test suite run.

### Old and new coexist on the wire

A client asking a server that predates a route gets "not found", and the
client reads that as "this server has no such sensor". Every other error is
a real error. Services are redeployed one at a time, so a mixed fleet is a
normal state, not an edge case.

### The control loops live with the motors

The watchdog and the wheel loop run in this process, beside the motors they
guard, not in the brain. A standing wheel command is re-vetted here as the
room changes, not trusted from the caller. A control loop never waits for a
motion in progress, because waiting once froze every route, the stop route
included (`PLAN-ros-alignment.md` 3.22). What silence means, and how long
it may last, is [safety](../safety/ARCHITECTURE.md)'s.

### Under ROS drive, the server stays the arbitration point

People and programs still drive through the verb route, where arbitration
decides between them. The standing-wheel route becomes the ROS actuator's
alone (`PLAN-ros-alignment.md` 3.13). A stop zeroes the robot directly
first; telling ROS its inputs are zero comes afterwards and cannot hold the
stop up. How the server decides ROS is down, and who may drive while it is,
are [safety](../safety/ARCHITECTURE.md)'s ("When ROS dies, only a person
drives"). **Every person has a path through ROS** (decided 2026-10-05):
the robot server sends a manual-rank driver's commands on the people's
input and an autonomous one's on the autonomy's, so ROS needs one input per
rank, not per client -- the server has already decided between them.

### Health reports facts; the verdict is elsewhere

The health route reports what this process knows: mode, drive, the
watchdog's own liveness, who holds the robot, the last refusal and which
build answered. Whether that adds up to "healthy" is decided by one health
command, which counts only conditions a release can be blamed for
(`PLAN-microduck-transplants.md` M5). Like every process, this one names
its build at start-up and on health ([operations](../operations/ARCHITECTURE.md),
"Every process says which build it is").

### A person's Stop latches autonomy out; only a mission's start lifts it

A stop from a person is the last word over autonomy (decided with the user
2026-10-15, 3.84). The server records it before the wheels stop and, while
it stands, refuses the brain's every command by name, so the mission ends
cancelled by that Stop; a person's own commands are never refused by it.
Before it, a D-pad Stop during a mission held the robot only until the
brain sent its next goal. Only the brain lifts the latch, and only when a
mission starts: a person pressed Start. A start that then fails puts the
latch back (3.76 K2), and a scheduled start never lifts it at all: under a
latch it opens no mission (3.163.B). **Rejected:** lifting it on a person's
next move or goal, which would let the mission the person stopped drive
again. The same routes carry the mission's start and its end, so the server
can report a brain that fell silent mid-mission apart from a watchdog that
fired in an ordinary wait; that record moves nothing (3.164, 3.171). The
order of drivers these rules sit in is
[safety](../safety/ARCHITECTURE.md)'s.

### A goal the brain sent lives only while the brain does

Under ROS drive the planner's wheel commands arrive on the actuator's
route, which is not arbitrated and feeds the watchdog, so a goal drove on
to its end after the brain that sent it was killed or frozen (3.135.B's
rows F6g and F7g, failed on the board; 3.145). Decided 2026-11-06: such a
goal is the brain driving. The brain's navigator sends a heartbeat for as
long as it lives, and a goal accepted from a brain that beats holds a lease
on the watchdog's own clock. When the beats stop, the server ends the goal
the way a Stop does and latches autonomy out until a mission starts, so a
brain thawed after a freeze cannot drive on; when the navigator closes, the
lease is released and its goal ended with no latch. A person's goal, an
unnamed one and a brain that never beats are unchanged. **Rejected:**
leaving it to the watchdog: it guards a silent driver, and the planner is
not silent.

### The robot server decides on the battery, and powers off once, cleanly

The server owns the motor board's reading and the safety layer, so it
decides and the brain is told and obeys (3.137; asked by the user
2026-11-01). Three levels: low is reported; at reserve all autonomy is
refused and a person may still drive, slowly, to the charger; at power-off,
or when reserve has been held too long, nothing drives and one clean
power-off runs in order, which nobody may override. A level is crossed only
on two distinct readings, and before reserve or power-off acts on a reading
taken while the wheels turned, the wheels are stopped and the pack is read
again at rest, because a motor's inrush sags the pack. The ladder has its
own loop, so it moves with nobody driving. A person can ask for the same
power-off before pressing the Rover's power button, which is a hard cut
(3.170): it is a person's Stop first, it leaves the battery's level alone,
and it runs once whoever asks. That route can power the robot off through
a public tunnel, so its gate is stricter than every other route's: with no
secret configured it answers only where nothing can really be powered off.
The levels are placeholders until the Rover's discharge curve is measured,
and the real power-off has not been run on the board (3.137.C, 3.137.E).

### No-go zones, keepout marks and a re-map pass through here; none is a veto

A person's no-go zones are kept by the brain and sent here as a whole set
for the saved map the robot is on; the server keeps a copy, passes it to
the planner, and refuses a person's goal inside a zone or close beside one
before it takes the robot from anyone (3.161, 3.199). Autonomy keeps out of
zones by its own choice and the planner's mask, and the trip home is never
refused for one. The planner cannot hear a refusal, so where this server's
own vet keeps refusing its forward moves the server marks a spot for it to
plan around (3.68, 3.83), and a mission's marks are cleared when it ends
(decided by the user 2026-10-21, 3.76 K5.3). A person's request to map
again is passed to the mapper; it is not arbitrated, because nothing moves
on it (decided 2026-10-23, 3.50.D4). **None of the three is read by the
vet.** A zone is a person's wish about where the robot plans, not a
measurement, so the D-pad is never refused for one.

### No reset

There is no route that puts a real robot back at its start. Real hardware
has none, and a sim that had one would invite tests the car cannot run.
Restarting the server is the reset. **One narrow exception, sim-only**
(3.72): the plain in-process simulator can be put in another house without
a restart, for the twin's random-house control. It is a reset, so it is
kept narrow: refused on every other body, refused while anyone drives or a
mission may be running, and the caller never names the house.

## Contracts

| Neighbour | Direction | Category | Ownership |
|---|---|---|---|
| Brain service | brain -> server | HTTP/JSON, driver named in a header; a heartbeat while its navigator lives; a mission's start and end; the no-go zones for the map in use | The server is authoritative for every refusal. The brain owns its mission state; the server knows only that one was started and ended, to lift the Stop's latch and to tell a hung brain from a waiting one. The brain owns the zones; the server holds a copy |
| Twin | browser -> server | HTTP/JSON polling; the static page from the same origin when local; a person's shutdown and re-map requests | The twin draws what the server publishes (the battery's level and a shutdown's progress included) and computes no safety state |
| ROS actuator plugin | container -> server | HTTP/JSON: reads wheel state, writes regular standing wheel commands as driver `ros` | Under ROS drive, the only wheel writer. Its posts are the liveness signal |
| ROS bridge | container -> server | HTTP/JSON reads of the scan and, in the sim, ground truth | Read-only |
| World model | server -> world | in-process | The world model owns map, pose and goals. The server passes them through, arbitrates goals, and hands it the keepout marks and the no-go zones for the planner |
| Body | server -> body | in-process | One body per process |
| Phone (teleop) | phone -> server | HTTP/JSON frame push | The body keeps only the latest frame |
| Health command | command -> server | HTTP/JSON | The command owns the verdict |

## Failure modes and resilience targets

| Failure | Behaviour | Target |
|---|---|---|
| Driver goes silent | The watchdog stops the motors and the claim lapses | Motors stop within the watchdog's bound ([safety](../safety/ARCHITECTURE.md)). The next driver starts clean |
| The watchdog task itself dies | Health reports its poll age growing. The health command turns that into unhealthy | A dead guard is visible, never silent |
| One wheel-loop tick throws | Logged. The loop continues | One bad tick never ends the loop |
| Camera fails or a teleop phone stalls | Only the frame route answers "service unavailable", with the body's own message | Other sensors keep answering |
| ROS bridge unreachable during a verb | Stop the robot directly. Refuse with a ROS-unavailable reason | |
| ROS bridge hung (accepts, never answers) when a stop arrives | The stop reaches the motors directly; zeroing ROS's inputs proceeds on its own and may fail | A stop never depends on the container, and never stalls the server's other routes |
| ROS bridge unreadable when checking for a goal | Treated as "no goal" | An outage never becomes a lockout of every autonomous driver |
| Part of the ROS container dies while the actuator lives (the bridge, or the velocity multiplexer or controller) | With the bridge gone, the first failed send is refused as ROS-unavailable and marks ROS down, so a person drives on the direct path and autonomy is refused until the bridge answers again. With the multiplexer or controller gone, ROS still reads as alive and verbs achieve nothing and are refused as unsafe | **Met for the bridge** (built 2026-10-02): a person can drive, as with a dead container. The rule is [safety](../safety/ARCHITECTURE.md)'s ("When ROS dies, only a person drives"); the multiplexer-or-controller case is left failing toward stop |
| Unknown verb, malformed body | Client error, nothing moves | |
| Body without motors gets a wheel command | Refused "unsupported" | Never a silent no-op |
| Sim-only or goal route on a world or body that cannot serve it | "Not implemented", naming why. Two sim routes answer otherwise, on purpose: setting the simulated pack is "not found" off the simulator (to a real body the route does not exist, 3.163.B), and the house swap is a conflict naming why (3.72) | Never a silent no-op |
| The brain is killed or frozen while its own goal drives (ROS drive) | The goal's lease lapses: the planner's wheel commands are held at zero, the goal is cancelled, and autonomy is refused until a mission starts | Wheels at zero within the watchdog's bound plus its wake-up interval of the brain's last beat (3.145; met on the desktop and on the board) |
| A person presses Stop during a mission | Autonomy is latched out; the brain's next command is refused by name and the mission ends cancelled | The stopped mission never drives again; a person's commands are never refused by the latch (3.84) |
| The pack falls to reserve, or to power-off | Reserve: autonomy refused, a goal ended, a person drives slowly. Power-off: everyone refused, then the clean power-off, once | A sag under motor load alone never moves a level. Levels are placeholders, and the real power-off is not yet run on the board (3.137) |
| A shutdown is asked with no secret configured, where a real power-off is possible | Refused | A route that can power the robot off is never open (3.170) |
| The planner does not take the zones, or a keepout mark | The server keeps its copy and says the set was not sent; a mark not taken is undone, so the next one can be made | A lost send is mended by the next whole-set send; nothing here changes the vet |
| Wrong or missing secret | "Unauthorized". Nothing moves, nothing is read | |
| A client newer than the server | "Not found" on the new route, read by the client as an absent sensor | Mixed versions keep working |

## Open questions

- **Boot units on the car (B5)** are decided in
  [operations](../operations/ARCHITECTURE.md). Units exist for the Jetson
  (`tools/jetson/systemd/`) for the stack (robot server, brain and proxy),
  the tunnel and the local VLM; none starts the ROS container.
- **CPU sharing with sensor drivers.** The lidar driver is decided to live
  in this process (`PLAN-ros-alignment.md` 6, question 5). Today the
  simulator's own ray casting, sharing this process, already dominates the
  HTTP tail at the control rate (3.17). On the Jetson that was confirmed and
  fixed: the simulated body now runs as separate programs (3.36), and with
  the full stack loaded the control loop ran no late ticks (3.33, 3.37). A
  real serial reader's share is still to be measured on the car.
- **Bridge route budget.** New world features (sighting geometry, map
  save) add bridge routes against a deliberate budget
  (`PLAN-ros-alignment.md` 1.1).
