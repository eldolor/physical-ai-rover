// web-twin/js/connstatus.js -- moved verbatim from web-twin/app.js (3.132 Stage 2B),
// dedented by the IIFE's two spaces, with `export` added. The rule a move
// follows is in app.js's banner.

// ---------- connection status ----------
// These two readouts are how you find out, standing in the middle of a
// room with a phone, whether the thing you are about to drive is actually
// reachable. They used to be dim grey body text that read identically
// whether the service was live or dead, with a long URL burying the one
// word that mattered. State goes in the label, colour and dot; the URL is
// demoted to a detail line that is allowed to wrap or truncate.
//
// Writes through the label/detail spans rather than the container's
// textContent, which would blow away the dot and the structure.
const CONN_STATE_CLASS = {ok: "is-ok", err: "is-err", busy: "is-busy", idle: ""};

// 3.132 Stage 2A: a status whose words depend on the voice is kept as the
// function that draws it, and drawn again when Engineer mode is switched
// (revoice()). Any plain setConnStatus() on the same element replaces it.
// Stage 3: the registry is this module's own, reached only through these
// functions (Stage 2B exported it for app.js's revoice() to walk).
const voicedStatus = new Map();
export function setVoicedConnStatus(el, draw) {
  draw();
  if (el) voicedStatus.set(el, draw);
}
// Draws every voiced status again in the voice now chosen. Each draw goes
// through setConnStatus(), which drops its element's entry; re-registering
// keeps it for the next switch (Thermos: only the first switch changed the
// voice). A snapshot, so the loop is not walking a map it is rewriting.
export function redrawVoicedStatus() {
  Array.from(voicedStatus).forEach(function (entry) { setVoicedConnStatus(entry[0], entry[1]); });
}
export function setConnStatus(el, stateName, label, detail) {
  if (!el) return;
  voicedStatus.delete(el);
  el.classList.remove("is-ok", "is-err", "is-busy");
  const cls = CONN_STATE_CLASS[stateName];
  if (cls) el.classList.add(cls);
  const labelEl = el.querySelector(".conn-status-label");
  const detailEl = el.querySelector(".conn-status-detail");
  if (labelEl) labelEl.textContent = label;
  if (detailEl) detailEl.textContent = detail || "";
}
