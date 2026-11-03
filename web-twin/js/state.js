// web-twin/js/state.js -- moved verbatim from web-twin/app.js (3.132 Stage 2B),
// dedented by the IIFE's two spaces, with `export` added. The rule a move
// follows is in app.js's banner.

// Fallback only, used until the connected server's /health reply has
// actually been read into state.minDistanceCm (renderWatchdog(), in app.js)
// -- e.g. before a connection exists at all, or against an older
// deployment that hasn't been redeployed to include the field yet.
// Every actual usage in the twin reads state.minDistanceCm, not this
// constant, so a running page stays in sync with whatever
// config/robot.yaml's safety.min_distance_cm really is on the server
// it's talking to (PLAN-sim-hardening.md definition of done, item 10 --
// this used to be a second hardcoded copy that could silently drift).
const MIN_DISTANCE_CM_FALLBACK = 20;
// ---------- twin state (mission bookkeeping only -- no physics) ----------

export const state = {
  serverUrl: "", serverSecret: "", connected: false, lastFrame: null,
  // Phase M2. `depthUnsupported` latches on the first 404 so a server
  // that predates /depth is asked once and then left alone, rather than
  // producing one failed request per step for the rest of the session.
  lastDepth: null, depthUnsupported: false,
  // N1. The WORLD model, kept apart from every field above it because
  // those are body state and these are not. `lastMap` is the big one --
  // ~10^5 cells on a real house against a handful of numbers everywhere
  // else -- which is why `map_version` exists and why the two are
  // fetched on different clocks (see refreshWorld).
  lastPose: null, lastMap: null, worldUnsupported: false, lastMapFetchAt: 0,
  searchedRooms: new Set(),
  sightings: [], found: false, foundSighting: null,
  log: [], step: 0,
  safetyFlashUntil: 0,
  guidanceMode: "robot", // "guide" -> /guidance (steer a person); "robot" -> /navigate (what would the robot do)
  guidanceRunning: false, guidanceStarting: false, guidanceTimerId: null, guidanceTarget: null,
  guidanceInFlight: 0, guidanceSeq: 0, guidanceLastRenderedSeq: 0,
  // Which run a vision call belongs to. Bumped on every stop, captured at
  // dispatch, compared on return -- see stopGuidance() and guidanceStep().
  guidanceEpoch: 0,
  guidanceCallCount: 0, guidanceStream: null,
  guidanceMuted: false, guidanceNotVisibleStreak: 0,
  guidancePaused: false, guidanceFoundStreak: 0,
  guidancePanSpeed: 0, guidanceLastOrientation: null,
  guidanceConsecutiveSkips: 0, guidanceErrorStreak: 0, guidanceCaptionIsAnswer: false,
  connecting: false,
  // 3.137.B: the robot server's battery ladder ({url, level, v, ladder,
  // readAt, answeredAt}, or null when unknown), and its poll and countdown
  // timers (app.js, "the battery ladder, shown").
  battery: null, batteryTimerId: null, batteryTickId: null,
  // Remote brain (phase B4): this page observes a mission it does not run.
  brainUrl: "", brainSecret: "", brainConnected: false, brainConnecting: false,
  brainMissionRunning: false, brainPollTimerId: null, brainLogSignature: "",
  // What the running mission is hunting. Read by recordObservation() so a
  // sighting the twin notices can be matched against the mission's target
  // -- which used to come from this page's own local loops, now deleted.
  brainTarget: null,
  // Brain liveness (see startBrainLiveness): the retry timer, and whether
  // a connection was LOST (as opposed to never made), so its return can
  // be announced.
  brainLivenessTimerId: null, brainLost: false, brainPollErrored: false,
  // How far a D-pad LEFT/RIGHT turns, in degrees (R0). 90 is what every
  // tap sent before the pose went continuous, so it stays the default.
  turnStepDeg: 90,
  // The robot server's /health `mode` ("sim", "teleop", "hardware") --
  // read by the tiered hint, which must not name models a sim mission
  // will never load.
  robotMode: null,
  watchdogTimerId: null,
  // Recording a Robot-view walk to the brain, for replay (S2b).
  // The robot's own id for the most recently pushed teleop frame, so a
  // recorded frame can name the id the mission will report deciding on.
  lastTeleopSeq: null,
  streamWalkName: null, recordLost: 0,      // 3.90 E
  recordWalk: false, recordWalkName: null, recordPending: [], recordSaved: 0, recordFailed: 0,
  // Which WALK a frame belongs to, which is not the same lifetime as
  // guidanceEpoch's run -- see recordWalkFrame(). recordOrphaned counts
  // frames dropped because their walk ended before their answer landed.
  recordEpoch: 0, recordOrphaned: 0,
  recordSeq: 0,
  // /navigate model A/B (empty string = service default, i.e. omit
  // model_id entirely -- see fetchNavigateModels()).
  navigateModelId: "", navigateModelsLoaded: false,
  // The prompt wording axis, served from the same allow-list as the models.
  navigatePromptVariant: "",
  // The vision service's own default, and whatever the connected brain
  // pins -- both only for showing which model a mission would really use.
  navigateServiceDefault: "", brainNavigateModelId: null,
  brainNavigatePromptVariant: null,
  // Which policy the Remote brain panel starts a mission under. "frontier"
  // is the free rule-based explorer; "vision" spends a model call a step
  // and is the one on the hardware path (PLAN-sim-hardening.md 2.2);
  // "tiered" runs YOLO + CLIP inside the brain process on every frame and
  // spends a call only on a trigger (PLAN-onboard-perception.md 2.4, P2).
  brainPolicy: "frontier",
  // 3.88 A: start explore in ask mode, and the choice it has open now
  // (`status.explore.ask`), drawn on the map and answered by a tap.
  brainAsk: false,
  brainAskOpen: null,
  // 3.70 S5: the panel's job -- "find" (a target) or "survey" (no target,
  // explore only). Not remembered across loads: a survey on the car spends
  // paid room-name calls, so it is chosen each time. brainLabelCap is the
  // brain's own cap on those calls (its /health): null for no cap,
  // undefined when not known.
  // brainJobChosen is this phone's own pick, which a watched mission's job
  // overrides only while that mission runs.
  brainJob: "find", brainJobChosen: "find", brainLabelCap: undefined,
  // What the connected brain says about its own perception models --
  // whether they are installed at all, and which two would load. Null
  // until a brain has been asked: "not known yet" and "not available"
  // are different answers and the hint says so differently.
  brainPerception: null,
  // Driving Robot view through a real MissionRunner mission instead of a
  // one-off /navigate call (PLAN-teleop-robot.md, Phase T3). driveViaBrain
  // is the live toggle; guidanceViaBrain is latched at Start so Stop knows
  // whether *this* running session actually started a brain mission, even
  // if the toggle or brain connection changes mid-walk.
  driveViaBrain: false, guidanceViaBrain: false,
  // Which policy a "Drive via brain" walk runs. Separate from
  // brainPolicy above because they start two different missions -- that
  // one drives the grid world, this one drives a phone on a wheeled rig
  // -- and because "frontier" is not on offer here: a photograph carries
  // no grid coordinates and TeleopRobot has no distance sensor.
  //
  // This is the only path in the app where the tiered policy sees real
  // pixels. PLAN-onboard-perception.md 4.10: the twin cannot test a
  // detector by 1.12's design, and a phone on a rig is the
  // highest-fidelity pre-hardware test available.
  drivePolicy: "vision",
  // 3.90 E: under Drive via brain the camera is pushed on its own timer at
  // this rate (newest wins), apart from the 500 ms status poll; and the
  // tiered policy's "combine frames" vote, sent at Start.
  streamFps: 4,
  frameVote: "off",
  // The running push loop (null when none), and whether the brain is the
  // one recording this walk (from the stream) rather than this page.
  framePush: null,
  recordByBrain: false,
  // Synced from the connected server's own /health reply
  // (renderWatchdog()) -- see MIN_DISTANCE_CM_FALLBACK above.
  minDistanceCm: MIN_DISTANCE_CM_FALLBACK,
};
