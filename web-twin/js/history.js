// web-twin/js/history.js -- moved verbatim from web-twin/app.js (3.132 Stage 2B),
// dedented by the IIFE's two spaces, with `export` added. The rule a move
// follows is in app.js's banner.

// ---------- recent search-target history ----------
// Shared between the Guide and Vision Autopilot target fields (same
// <datalist>, same localStorage key) since they're conceptually the
// same "what am I looking for" input -- typing "red backpack" fresh
// every session was pure friction once you've searched for the same
// handful of things more than once.
const TARGET_HISTORY_KEY = "vp_target_history";
const TARGET_HISTORY_MAX = 8;
function loadTargetHistory() {
  try { return JSON.parse(localStorage.getItem(TARGET_HISTORY_KEY) || "[]"); }
  catch (e) { return []; }
}
export function renderTargetHistory(list) {
  const dl = document.getElementById("target-history-list");
  if (!dl) return;
  dl.innerHTML = "";
  (list || loadTargetHistory()).forEach(function (t) {
    const opt = document.createElement("option");
    opt.value = t;
    dl.appendChild(opt);
  });
}
export function saveTargetToHistory(target) {
  if (!target) return;
  try {
    let list = loadTargetHistory().filter(function (t) { return t.toLowerCase() !== target.toLowerCase(); });
    list.unshift(target);
    list = list.slice(0, TARGET_HISTORY_MAX);
    localStorage.setItem(TARGET_HISTORY_KEY, JSON.stringify(list));
    renderTargetHistory(list);
  } catch (e) { /* best-effort -- localStorage unavailable, just skip persisting */ }
}
