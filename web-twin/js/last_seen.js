// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Anshu Gaind
// web-twin/js/last_seen.js -- 3.175.C: "Where did you last see it?" on Home.
//
// A tap asks the brain's GET /mission/last-seen (read-only: from what the
// robot already keeps, nothing moves) for the Find box's target; the answer is
// shown under the box in plain words with its thumbnail, and "Go check"
// starts an ordinary find for that name, the same body and path as Start
// (docs/plans/ros-alignment/3.175-memory-when-things-move.md, C6-C7). The
// module only declares (3.132 Stage 2B's rule): app.js makes the asker and
// hands it the brain call and the redraw.

// The asker's states: "idle" (not asked), "asking", "ok" (an answer),
// "old" (a brain without the route), "error" (no answer).
export function createLastSeen(ask, onChange) {
  let seq = 0;
  const r = { name: null, status: "idle", answer: null, error: null };

  function set(fields) { Object.assign(r, fields); onChange(r); }

  async function askFor(name) {
    name = (name || "").trim();
    if (!name) return;
    const mine = ++seq;
    set({ name: name, status: "asking", answer: null, error: null });
    try {
      const got = await ask(name);
      if (mine !== seq) return;              // an older question's reply is never shown
      // A 200 that is not an answer (a tunnel's page) is no answer.
      if (!got || typeof got.status !== "string") throw new Error("not an answer from the robot");
      set({ status: "ok", answer: got });
    } catch (e) {
      if (mine !== seq) return;
      // Only FastAPI's own 404 is a brain built before 3.175.C (as 3.176.C).
      if (e && e.status === 404 && e.message === "Not Found") {
        set({ status: "old", answer: null, error: null });
        return;
      }
      set({ status: "error", answer: null, error: (e && e.message) || String(e) });
    }
  }

  return {
    reading: r,
    ask: askFor,
    // Forget the shown answer (app.js: the box changed, a mission started, another brain).
    clear: function () { seq++; set({ name: null, status: "idle", answer: null, error: null }); },
  };
}

// "2 hours ago" from an epoch-seconds `ended`, or "" when it is unknown.
export function agoWords(ended, nowMs, t) {
  if (typeof ended !== "number" || !isFinite(ended)) return "";
  const s = Math.max(0, nowMs / 1000 - ended);
  if (s < 90) return t("lastSeen.justNow");
  if (s < 90 * 60) return t("lastSeen.minutesAgo", { n: Math.round(s / 60) });
  if (s < 36 * 3600) return t("lastSeen.hoursAgo", { n: Math.round(s / 3600) });
  return t("lastSeen.daysAgo", { n: Math.round(s / 86400) });
}

function roomOf(a) {
  // The head row's room is the mission's own label ("living_room"): it reads
  // "the living room". A report's room that differs from it is 3.160's
  // person-given name there, shown as given. With no label to compare (the
  // head row's "unknown"), a report room in label form (lower case,
  // underscores) is taken as a label. "unknown" is no room.
  const p = a.place || a.doubtful || {};
  const label = p.room && p.room !== "unknown" ? String(p.room) : null;
  const w = a.when && a.when.room && a.when.room !== "unknown" ? String(a.when.room) : null;
  const words = function (s) { return "the " + s.replace(/_/g, " "); };
  if (w && w !== label) return label || !/^[a-z0-9_]+$/.test(w) ? w : words(w);
  return label ? words(label) : null;
}

// The row's words: { text, go, thumb } -- `text` is HTML built from escaped
// parts (the brain's labels and rooms are data, never markup); `go` whether
// "Go check" applies; `thumb` {mission_id, thumb} or null.
export function lastSeenView(r, t, esc, nowMs, engineer) {
  if (r.status === "asking") return { text: esc(t("lastSeen.asking", { name: r.name })), go: false, thumb: null };
  if (r.status === "old") return { text: esc(t("lastSeen.old")), go: false, thumb: null };
  if (r.status === "error") return { text: esc(t("lastSeen.failed", { why: r.error })), go: false, thumb: null };
  if (r.status !== "ok" || !r.answer) return { text: "", go: false, thumb: null };
  const a = r.answer;
  const name = "<b>" + esc(a.name) + "</b>";
  const lines = [];
  const thumb = a.when && a.when.thumb && a.when.mission_id
    ? { mission_id: a.when.mission_id, thumb: a.when.thumb } : null;
  const room = roomOf(a);
  const ago = a.when ? agoWords(a.when.ended, nowMs, t) : "";
  if (a.status === "seen") {
    const key = room ? (ago ? "lastSeen.seenInAgo" : "lastSeen.seenIn") : (ago ? "lastSeen.seenAgo" : "lastSeen.seen");
    lines.push(t(key, { name: name, room: "<b>" + esc(room || "") + "</b>", ago: esc(ago) }));
    if (a.places.length > 1) lines.push(esc(t("lastSeen.more", { n: a.places.length - 1 })));
  } else if (a.status === "doubtful") {
    lines.push(t(room ? "lastSeen.doubtfulIn" : "lastSeen.doubtful",
                 { name: name, room: "<b>" + esc(room || "") + "</b>" }));
  } else if (a.status === "not_seen") {
    lines.push(t("lastSeen.notSeen", { name: name }));
  } else {
    lines.push(esc(t("lastSeen.noMemory")));
  }
  if (engineer) {
    const p = a.place || a.doubtful;
    lines.push(esc(t("lastSeen.engineer", {
      status: a.status, source: a.source, frame: a.frame_id == null ? "none" : a.frame_id,
      where: p ? "(" + p.x_m + ", " + p.y_m + ") belief " + p.belief : "none",
      mission: a.when && a.when.mission_id ? a.when.mission_id : "none" })));
  }
  return { text: lines.map(function (l) { return "<div>" + l + "</div>"; }).join(""),
           // A find works whatever the memory says: "Go check" is offered on every answer.
           go: true, thumb: thumb };
}
