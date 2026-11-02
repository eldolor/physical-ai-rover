// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Anshu Gaind
/* control/metrics.js -- the Lab console's Metrics tab (3.132 Stage 4; was
 * the tier-metrics page, control/metrics.html).
 *
 * Reads GET /metrics/summary and draws it. The secret is control/lab.js's
 * (window.Lab), shared with the Walks tab. Deliberately dependency-free
 * and inline-SVG: the page is served from the same CloudFront origin as
 * the twin, and a chart library would be the only third-party script in
 * the whole deployment for a job that is two polylines.
 *
 * THE RULE THIS PAGE ENFORCES, and it is the reason several obvious
 * simplifications are absent: a percentile belongs to the run that
 * produced it. Averaging p90s across runs does not give the p90 of the
 * pair, so nothing here does it. Cross-run figures are a MEDIAN OF
 * PER-RUN VALUES and are labelled as such, or they are a range.
 */
(function () {
  "use strict";

  // 3.70 E1: the outcome names written before 3.70, read as today's
  // (control/mission_runner.py read_outcome()). Stored rows are never rewritten.
  var LEGACY_OUTCOME = { searched: "exhausted", max_steps: "out_of_budget",
                         stopped: "cancelled", preempted: "cancelled" };
  function readOutcome(o) { return (o && LEGACY_OUTCOME[o]) || o; }
  // 3.76 C9: who cancelled -- the row's own `cancelled_by`, else what an
  // old name said (`stopped` was the operator, `preempted` the D-pad).
  var LEGACY_CANCELLED_BY = { stopped: "stop", preempted: "dpad" };
  function cancelledBy(r) { return r.cancelled_by || LEGACY_CANCELLED_BY[r.outcome] || null; }

  var Lab = window.Lab;
  var $ = function (id) { return document.getElementById(id); };

  // The charts draw in the page's own tokens (ui.css), as CSS on each mark,
  // so they follow a theme change without a redraw.
  function token(name) { return "var(" + name + ")"; }

  function fmtMs(v) { return v == null ? "–" : Math.round(v) + "ms"; }
  function pct(v) { return v == null ? "–" : Math.round(v * 100) + "%"; }

  /* Median of a list of per-run values. Used everywhere a headline number
   * spans runs -- see the module rule above. */
  function median(xs) {
    var v = xs.filter(function (x) { return typeof x === "number"; }).sort(function (a, b) { return a - b; });
    if (!v.length) return null;
    var m = Math.floor(v.length / 2);
    return v.length % 2 ? v[m] : (v[m - 1] + v[m]) / 2;
  }

  // 3.76 F14: every stored value is text from whoever POSTed the row (a
  // mission's target is typed by a person), and this page holds the walks
  // secret in localStorage. Escaped on the way into innerHTML, all of it.
  var esc = Lab.escapeHtml;

  function card(label, dotClass, value, unit, foot) {
    return '<div class="card"><div class="label">'
      + (dotClass ? '<i class="dot ' + dotClass + '"></i>' : "")
      + esc(label) + '</div><div class="value">' + esc(value)
      + (unit ? '<small>' + esc(unit) + '</small>' : "")
      + '</div><div class="foot">' + esc(foot || "") + '</div></div>';
  }

  function renderKpis(runs) {
    var cloud = runs.map(function (r) { return (r.stats || {}).cloud_ms || null; });
    var local = runs.map(function (r) { return (r.stats || {}).perception_ms || null; });
    var calls = runs.map(function (r) { return (r.stats || {}).cloud_calls || 0; });
    var frames = runs.map(function (r) { return (r.stats || {}).frames || 0; });
    var detected = runs.map(function (r) {
      var p = (r.stats || {}).perception || {};
      var tot = Object.keys(p).reduce(function (a, k) { return a + p[k]; }, 0);
      return tot ? (p.detected || 0) / tot : null;
    });
    var totalCalls = calls.reduce(function (a, b) { return a + b; }, 0);
    var totalFrames = frames.reduce(function (a, b) { return a + b; }, 0);
    var pick = function (arr, k) {
      return arr.map(function (x) { return x ? x[k] : null; });
    };
    var found = runs.filter(function (r) { return r.outcome === "found"; }).length;

    $("kpis").innerHTML = [
      card("Cloud p50", "cloud", fmtMs(median(pick(cloud, "p50"))), "",
           "median of " + runs.length + " runs"),
      card("Cloud p90", "cloud", fmtMs(median(pick(cloud, "p90"))), "",
           "median of per-run p90"),
      card("Cloud p99", "cloud", fmtMs(median(pick(cloud, "p99"))), "",
           "median of per-run p99 · read with n"),
      card("Perception p50", "local", fmtMs(median(pick(local, "p50"))), "",
           "per frame, on-board tier"),
      card("Frames per call", "", totalCalls ? (totalFrames / totalCalls).toFixed(1) : "–", "",
           totalFrames + " frames / " + totalCalls + " calls"),
      card("Target detected", "local", pct(median(detected)), "",
           "median share of frames"),
      card("Missions", "", String(runs.length), "",
           found + " reached the target")
    ].join("");
  }

  /* ---- charts: inline SVG, no library ---- */

  function lineChart(el, series, opts) {
    opts = opts || {};
    var W = 1000, H = 210, P = { l: 46, r: 12, t: 12, b: 26 };
    var all = [];
    series.forEach(function (s) {
      s.values.forEach(function (v) { if (v != null) all.push(v); });
    });
    if (!all.length) {
      el.innerHTML = '<div class="empty">No data in this window.</div>';
      return;
    }
    var max = Math.max.apply(null, all) * 1.12;
    var min = opts.zero === false ? Math.min.apply(null, all) * 0.9 : 0;
    var n = Math.max(1, series[0].values.length - 1);
    var x = function (i) { return P.l + (i / n) * (W - P.l - P.r); };
    var y = function (v) { return P.t + (1 - (v - min) / (max - min || 1)) * (H - P.t - P.b); };

    var parts = [];
    for (var g = 0; g <= 4; g++) {
      var gy = P.t + (g / 4) * (H - P.t - P.b);
      var gv = max - (g / 4) * (max - min);
      parts.push('<line class="gl" x1="' + P.l + '" y1="' + gy + '" x2="' + (W - P.r) + '" y2="' + gy + '"/>');
      parts.push('<text class="ax" x="' + (P.l - 8) + '" y="' + (gy + 3.5) + '" text-anchor="end">'
        + Math.round(gv) + (opts.unit || "") + '</text>');
    }
    series.forEach(function (s) {
      // Gaps are BREAKS, not interpolations: a run with no cloud call has
      // no latency, and drawing a line through it would invent one.
      var seg = [], segs = [];
      s.values.forEach(function (v, i) {
        if (v == null) { if (seg.length) segs.push(seg); seg = []; }
        else seg.push(x(i) + "," + y(v));
      });
      if (seg.length) segs.push(seg);
      segs.forEach(function (pts) {
        if (pts.length === 1) {
          var c = pts[0].split(",");
          parts.push('<circle cx="' + c[0] + '" cy="' + c[1] + '" r="2.6" style="fill:' + s.color + '"/>');
        } else {
          parts.push('<polyline fill="none" style="stroke:' + s.color + '" stroke-width="2"'
            + ' stroke-linejoin="round" stroke-linecap="round" points="' + pts.join(" ") + '"/>');
        }
      });
    });
    parts.push('<text class="ax" x="' + P.l + '" y="' + (H - 6) + '">oldest</text>');
    parts.push('<text class="ax" x="' + (W - P.r) + '" y="' + (H - 6) + '" text-anchor="end">newest</text>');
    el.innerHTML = '<svg viewBox="0 0 ' + W + ' ' + H + '" preserveAspectRatio="none" '
      + 'style="height:' + H + 'px">' + parts.join("") + '</svg>';
  }

  function renderCharts(runs) {
    var old = runs.slice().reverse();   // oldest -> newest, which is how a trend reads
    var get = function (k, f) { return old.map(function (r) { var s = (r.stats || {})[k]; return s ? s[f] : null; }); };
    lineChart($("chart-cloud"), [
      { color: token("--accent-primary"), values: get("cloud_ms", "p50") },
      { color: token("--accent-warn"), values: get("cloud_ms", "p90") },
      { color: token("--accent-alert"), values: get("cloud_ms", "p99") }
    ], { unit: "ms" });

    lineChart($("chart-local"), [
      { color: token("--accent-safe"), values: get("perception_ms", "p50") },
      { color: token("--accent-warn"), values: old.map(function (r) {
          var p = (r.stats || {}).perception || {};
          var tot = Object.keys(p).reduce(function (a, k) { return a + p[k]; }, 0);
          return tot ? Math.round((p.detected || 0) / tot * 100) : null;
        }) }
    ], {});
  }

  function renderRuns(runs) {
    if (!runs.length) {
      $("runs").innerHTML = '<tr><td class="empty">No runs in this window.</td></tr>';
      return;
    }
    var head = '<tr class="runs-head"><th>When</th><th>Outcome</th><th>Steps</th><th>Target</th>'
      + "<th>Cloud p50/p90/p99</th><th>n</th><th>Perc p50</th><th>Detected</th>"
      + "<th>Calls</th><th>Build</th><th>Config</th></tr>";
    var rows = runs.map(function (r) {
      var s = r.stats || {}, c = s.cloud_ms, l = s.perception_ms, p = s.perception || {};
      var tot = Object.keys(p).reduce(function (a, k) { return a + p[k]; }, 0);
      var when = r.finished_at
        ? new Date(r.finished_at * 1000).toLocaleString(undefined,
            { month: "short", day: "numeric", hour: "2-digit", minute: "2-digit" })
        : (r.day || "–");
      var outcome = readOutcome(r.outcome);
      var cls = outcome === "found" ? "found"
        : (outcome === "failed" ? "bad" : "");
      var by = outcome === "cancelled" ? cancelledBy(r) : null;
      var cfg = Object.keys(r.config || {}).filter(function (k) {
        return r.config[k] !== null && r.config[k] !== "";
      }).map(function (k) {
        return k.replace(/^tier_|^perception_|^navigate_/, "") + "=" + r.config[k];
      }).join(" · ");
      // Each cell names its column (data-label): at phone width the table
      // is drawn as one card per run, the labels beside the values, rather
      // than a table scrolling sideways (3.132 Stage 4).
      function td(label, html, cls) {
        return '<td data-label="' + esc(label) + '"' + (cls ? ' class="' + cls + '"' : "") + ">"
          + html + "</td>";
      }
      return "<tr>" + td("When", esc(when))
        + td("Outcome", '<span class="pill ' + cls + '">' + esc(outcome || "–")
          + (by ? " · " + esc(by) : "") + "</span>")
        + td("Steps", esc(r.steps == null ? "–" : r.steps))
        + td("Target", esc(r.target_object || "–"))
        + td("Cloud p50/p90/p99", esc(c ? c.p50 + " / " + c.p90 + " / " + c.p99 : "–"))
        // n beside every percentile, always: a percentile over a handful
        // of samples is one sample with a fancy name.
        + td("n", esc(c ? c.n : "0"), "mono")
        + td("Perc p50", esc(l ? l.p50 + "ms" : "–"))
        + td("Detected", tot ? Math.round((p.detected || 0) / tot * 100) + "%" : "–")
        + td("Calls", esc(s.cloud_calls == null ? "–" : s.cloud_calls))
        + td("Build", esc(String(r.git_revision || "–").slice(0, 7)), "mono")
        + td("Config", esc(cfg || "–"), "mono") + "</tr>";
    });
    $("runs").innerHTML = head + rows.join("");
  }

  // Only the newest load paints (3.76 N2): switching 7 -> 90 -> 7 days drew
  // the slow 90-day answer under a "7" selector.
  var loadSeq = 0;

  function clearView() {
    ["kpis", "chart-cloud", "chart-local", "runs"].forEach(function (id) { $(id).innerHTML = ""; });
    $("footnote").textContent = "";
  }

  // Loaded once the page is connected, and only while the tab is shown;
  // `stale` asks for a fresh load the next time it is.
  var stale = true;

  function load() {
    var seq = ++loadSeq;
    stale = false;
    $("error").hidden = true;
    // Not connected: nothing to show yet, and nothing has gone wrong (3.132
    // Stage 0: a first open showed an error naming WALKS_SECRET).
    $("metrics-prompt").hidden = Lab.connected;
    if (!Lab.connected) { clearView(); return; }
    fetch("metrics/summary?days=" + $("days").value, { headers: Lab.headers() })
      .then(function (r) {
        if (!r.ok) {
          throw new Error("HTTP " + r.status
            + (r.status === 401 || r.status === 403 ? " -- the secret was refused." : ""));
        }
        return r.json();
      })
      .then(function (d) {
        if (seq !== loadSeq) return;
        var runs = d.runs || [];
        renderKpis(runs);
        renderCharts(runs);
        renderRuns(runs);
        $("footnote").textContent = runs.length + " run(s) over " + d.days
          + " days. Percentiles are per run and are never blended: a cross-run"
          + " figure here is the median of per-run values, and n is shown so a"
          + " percentile over a handful of samples reads as what it is.";
      })
      .catch(function (e) {
        if (seq !== loadSeq) return;
        // Not the last window's numbers under this window's error (3.76 N2).
        clearView();
        $("error").hidden = false;
        $("error").textContent = "Could not load metrics: " + e.message;
      });
  }

  $("reload").onclick = load;
  $("days").onchange = load;
  Lab.onConnect(function () {
    stale = true;
    if (Lab.tab === "metrics") load();
  });
  Lab.onTab(function (name) {
    if (name === "metrics" && stale) load();
  });
})();
