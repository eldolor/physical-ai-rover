// web-twin/js/debuglog.js -- moved verbatim from web-twin/app.js (3.132 Stage 2B),
// dedented by the IIFE's two spaces, with `export` added. The rule a move
// follows is in app.js's banner.

export const DEBUG_LOG_KEY = "vp_debug_log";
const DEBUG_LOG_MAX = 20;
export function pushDebugLog(entry) {
  try {
    const log = JSON.parse(localStorage.getItem(DEBUG_LOG_KEY) || "[]");
    log.push(Object.assign({ t: new Date().toISOString() }, entry));
    localStorage.setItem(DEBUG_LOG_KEY, JSON.stringify(log.slice(-DEBUG_LOG_MAX)));
  } catch (e) { /* best-effort -- never let logging itself throw */ }
}
