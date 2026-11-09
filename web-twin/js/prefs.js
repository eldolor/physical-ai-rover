// web-twin/js/prefs.js -- moved verbatim from web-twin/app.js (3.132 Stage 2B),
// dedented by the IIFE's two spaces, with `export` added. The rule a move
// follows is in app.js's banner.

// ---------- persisted preferences ----------
// Everything a returning user would otherwise have to re-enter or
// re-choose. This page is installable as a PWA, so it gets reopened cold
// constantly; previously only the two shared secrets and the target
// history survived a reload, which meant every launch started
// disconnected with an empty vision URL on whichever tab happened to be
// first. localStorage can throw outright in some private-browsing modes,
// so every access is wrapped -- a storage failure must never take down
// page init or a click handler.
export const PREF = {
  serverUrl: "vp_server_url",
  brainUrl: "vp_brain_url",
  serverSecret: "vp_cfg_server_secret",   // shared with prefillTesterSecrets (app.js)
  visionUrl: "vp_vision_url",
  visionSecret: "vp_cfg_secret",          // shared with prefillTesterSecrets (app.js)
  tab: "vp_active_tab",
  muted: "vp_guide_muted",
  onboarded: "vp_guide_onboarded",
  rotateHintDismissed: "vp_guide_rotate_dismissed",
  debugReadouts: "vp_debug_readouts",
  recordWalk: "vp_record_walk",
  driveViaBrain: "vp_drive_via_brain",
  drivePolicy: "vp_drive_policy",
  streamFps: "vp_stream_fps",       // 3.90 E
  frameVote: "vp_frame_vote",       // 3.90 E
  turnStepDeg: "vp_turn_step_deg",
  navigateModelId: "vp_navigate_model_id",
  navigatePromptVariant: "vp_navigate_prompt_variant",
  brainPolicy: "vp_brain_policy",
  // 3.137.B: the robot server whose battery last read Power-off, so its
  // "unplug the battery" survives a reload against a robot that is off.
  batteryPoweroff: "vp_battery_poweroff",
  // 3.170: a shutdown under way (JSON), so what the phone knew survives a
  // reload against a robot that has gone quiet.
  shutdown: "vp_shutdown",
};
export function prefGet(key) {
  try { return localStorage.getItem(key); } catch (e) { return null; }
}
export function prefSet(key, value) {
  try { localStorage.setItem(key, value); } catch (e) { /* best-effort */ }
}
