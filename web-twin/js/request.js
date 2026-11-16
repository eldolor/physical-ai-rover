// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Anshu Gaind
// web-twin/js/request.js -- 3.176.C: the Find box reads a sentence.
//
// What the person typed goes to the brain's POST /mission/request/parse (rules,
// no model, nothing moves) 250 ms after the last keystroke; the reading is
// shown under the box, and Start sends exactly what was shown
// (docs/plans/ros-alignment/3.176-plain-language-request.md, C1-C10). The
// module only declares (3.132 Stage 2B's rule): app.js makes the reader and
// hands it the brain call, the words and the redraw.

export const PARSE_DEBOUNCE_MS = 250;

// The reading's states: "empty" (nothing typed), "pending" (an edit not yet
// read), "ok" (read), "old" (a brain without the route: the words go as
// typed, `dev`'s start), "error" (no answer: Start stays off).
export function createFindReader(parse, onChange) {
  let seq = 0;
  let timer = null;
  const r = { text: null, status: "idle", parsed: null, error: null, roomDropped: false };

  function set(fields) { Object.assign(r, fields); onChange(r); }

  async function fire(text, mine) {
    try {
      const parsed = await parse(text);
      if (mine !== seq) return;          // C4: an older text's reply is never shown
      set({ status: "ok", parsed: parsed, error: null });
    } catch (e) {
      if (mine !== seq) return;
      // A brain that answered 404 has no parse route (one built before
      // 3.176.B): it still runs a find on the words as typed (C6).
      if (e && e.status === 404) set({ status: "old", parsed: null, error: null });
      else set({ status: "error", parsed: null, error: (e && e.message) || String(e) });
    }
  }

  function edit(text, delay) {
    text = (text || "").trim();
    if (text === r.text && r.status !== "idle") return;
    clearTimeout(timer);
    const mine = ++seq;
    if (!text) { set({ text: text, status: "empty", parsed: null, error: null, roomDropped: false }); return; }
    set({ text: text, status: "pending", parsed: null, error: null, roomDropped: false });
    timer = setTimeout(function () { fire(text, mine); }, delay == null ? PARSE_DEBOUNCE_MS : delay);
  }

  return {
    reading: r,
    edit: edit,
    // Read again now (a brain just connected, or the last read failed).
    refresh: function (text) { r.status = "idle"; edit(text, 0); },
    dropRoom: function () { if (r.status === "ok") set({ roomDropped: true }); },
    // What Start sends for `typed`, or null when Start must stay off.
    startFields: function (typed) { return findStartFields(r, typed); },
  };
}

// Start's fields from a reading, or null (C2, C5-C7, C9). Only a reading of
// the text in the box now counts: a stale one is null (C4).
export function findStartFields(r, typed) {
  typed = (typed || "").trim();
  if (!typed || r.text !== typed) return null;
  if (r.status === "old") return { target_object: typed };
  if (r.status !== "ok" || !r.parsed || !r.parsed.target) return null;
  const hint = r.roomDropped ? null : r.parsed.room_hint;
  // C7: a plain target is `dev`'s start, its typed case kept, no `request`.
  if (!hint && r.parsed.target === typed.toLowerCase()) return { target_object: typed };
  const fields = { target_object: r.parsed.target, request: typed };
  if (hint) fields.room_hint = hint;
  return fields;
}

export function roomWords(hint) {
  if (!hint) return "";
  return hint.name != null ? hint.name : "the " + hint.type;
}

// The line under the box. `t` is the page's words; `esc` escapes text from
// the brain or the person, which is data, never markup.
export function renderFindReading(el, r, t, esc, engineer, why) {
  const lines = [];
  if (why) {
    lines.push(esc(why));
  } else if (r.status === "pending" || r.status === "idle") {
    lines.push(esc(t("find.reading")));
  } else if (r.status === "empty") {
    lines.push(esc(t("find.empty")));
  } else if (r.status === "old") {
    lines.push(esc(t("find.asTyped")));
  } else if (r.status === "error") {
    lines.push(esc(t("find.readFailed", { why: r.error })));
  } else if (r.status === "ok") {
    const p = r.parsed;
    const hint = r.roomDropped ? null : p.room_hint;
    if (p.target) {
      lines.push(t(hint ? "find.lookForIn" : "find.lookFor",
                   { target: "<b>" + esc(p.target) + "</b>", room: "<b>" + esc(roomWords(hint)) + "</b>" }));
    } else {
      lines.push(esc(t("find.noTarget")));
      if (p.room_hint) lines.push(t("find.roomOnly", { room: "<b>" + esc(roomWords(p.room_hint)) + "</b>" }));
    }
    const a = p.attributes || {};
    const bits = (a.colours || []).concat(a.size ? [a.size] : []);
    if (p.target && bits.length) lines.push(esc(t("find.looks", { what: bits.join(", ") })));
    if (p.unused) lines.push(esc(t("find.readPast", { words: p.unused })));
    if (engineer) lines.push(esc(t("find.engineer", { names: p.names_read == null ? "?" : p.names_read })));
  }
  el.innerHTML = lines.map(function (l) { return "<div>" + l + "</div>"; }).join("");
  const drop = r.status === "ok" && r.parsed.target && r.parsed.room_hint && !r.roomDropped;
  return !!drop;
}
