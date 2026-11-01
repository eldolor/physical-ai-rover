// web-twin/js/ui.js -- moved verbatim from web-twin/app.js (3.132 Stage 2B),
// dedented by the IIFE's two spaces, with `export` added. The rule a move
// follows is in app.js's banner.

import { ICON } from "./icons.js";

// ---------- shared UI helpers ----------

// Values rendered here come from the vision service's JSON response, not
// from this page -- escape them rather than concatenating them straight
// into innerHTML.
export function escapeHtml(value) {
  return String(value == null ? "" : value)
    .replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

// A blank dark rectangle before anything has happened reads as broken
// rather than empty, so every log/result surface gets a real "nothing
// yet" state. Marked with data-empty so the first real entry knows to
// clear it rather than appending underneath it.
export function setEmptyState(el, icon, text) {
  if (!el) return;
  el.innerHTML = '<div class="empty-state">' +
    '<span class="empty-state-icon">' + (ICON[icon] || "") + '</span>' +
    '<span class="empty-state-text">' + text + '</span></div>';
  el.dataset.empty = "1";
}
export function clearEmptyState(el) {
  if (el && el.dataset.empty) { el.innerHTML = ""; delete el.dataset.empty; }
}

export const LOG_EMPTY = ["clock", "No moves yet — start a mission to see the robot's steps."];

// Transient confirmations for actions whose outcome would otherwise only
// appear as rewritten text in a panel the user may not be looking at.
const TOAST_ICON = { ok: "target", err: "warning", warn: "warning", info: "clock" };
export function showToast(message, kind) {
  const stack = document.getElementById("toast-stack");
  if (!stack) return;
  const el = document.createElement("div");
  el.className = "toast " + (kind || "info");
  el.innerHTML = '<span class="toast-icon">' + (ICON[TOAST_ICON[kind || "info"]] || "") + "</span>" +
    "<span>" + escapeHtml(message) + "</span>";
  stack.appendChild(el);
  setTimeout(function () {
    el.classList.add("leaving");
    setTimeout(function () { el.remove(); }, 220);
  }, kind === "err" || kind === "warn" ? 5200 : 3200);
}

// Buttons keep their label but gain a spinner and stop accepting input
// while the request they started is in flight.
export function setButtonBusy(btn, busy, busyLabel) {
  if (!btn) return;
  if (busy) {
    if (!btn.dataset.idleHtml) btn.dataset.idleHtml = btn.innerHTML;
    btn.innerHTML = '<span class="spinner"></span>' + escapeHtml(busyLabel || "Working…");
    btn.classList.add("is-busy");
    btn.setAttribute("aria-busy", "true");
  } else {
    if (btn.dataset.idleHtml) { btn.innerHTML = btn.dataset.idleHtml; delete btn.dataset.idleHtml; }
    btn.classList.remove("is-busy");
    btn.removeAttribute("aria-busy");
  }
}
