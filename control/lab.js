/* control/lab.js -- what the Lab console's two tabs share (3.132 Stage 4).
 *
 * The walks console (admin.js) and the metrics console (metrics.js) were two
 * pages behind ONE secret (WALKS_SECRET guards /recording/*, /stats and
 * /metrics/*), each with its own field for it. They are one page now, and
 * this file holds what they share: the secret and Connect, the tabs, the
 * confirm dialog every paid action goes through, and the estimate it shows.
 * It loads first and exposes window.Lab; the other two scripts subscribe.
 */
(function () {
  "use strict";

  // The walks console's key. The metrics console kept its own, and either
  // one connects both tabs; once a secret works it is saved here only.
  var SECRET_KEY = "vp_admin_secret";
  var OLD_SECRET_KEYS = ["vp_metrics_secret"];
  var $ = function (id) { return document.getElementById(id); };

  function escapeHtml(s) {
    return String(s).replace(/[&<>"']/g, function (c) {
      return { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c];
    });
  }

  function shortModel(modelId) {
    if (!modelId) return "";
    return modelId.replace(/^(us|global)\./, "").replace(/^anthropic\.claude-/, "")
      .replace(/-\d{8}-v\d+:\d+$/, "").replace(/^amazon\./, "").replace(/^openai\./, "")
      .replace(/-v\d+:\d+$/, "");
  }

  /* Tolerate what a phone paste actually contains (from metrics.js).
   *
   * The documented way to find this value is `grep WALKS_SECRET
   * ~/.vision-picar-serverless-secrets`, whose output is the whole LINE.
   * Pasting that sends "WALKS_SECRET=abc..." as the header and is refused,
   * with nothing on screen to suggest why -- observed on an iPhone,
   * 2026-09-13. Shell quoting and a trailing newline arrive the same way.
   * None of `KEY=`, quotes or whitespace can be part of a real secret here:
   * the value has to survive being an HTTP header and a shell variable. */
  function cleanSecret(raw) {
    var v = (raw || "").trim().replace(/[\r\n]+$/, "");
    v = v.replace(/^[A-Z_][A-Z0-9_]*\s*=\s*/, "");
    v = v.replace(/^["']|["']$/g, "");
    return v.trim();
  }

  var Lab = {
    secret: "",
    escapeHtml: escapeHtml,
    shortModel: shortModel,
    cleanSecret: cleanSecret,
  };

  // `trying`: a secret being tried by Connect, sent on that request only.
  Lab.headers = function (extra, trying) {
    var h = Object.assign({}, extra);
    var s = trying === undefined ? Lab.secret : trying;
    if (s) h["x-app-secret"] = s;
    return h;
  };

  // ---------- the secret and Connect ----------

  var connectListeners = [];
  // Called with the walk list Connect fetched, once a secret has worked.
  Lab.onConnect = function (fn) { connectListeners.push(fn); };
  Lab.connected = false;

  var statusEl = $("conn-status");
  function setStatus(text, cls) {
    statusEl.textContent = text;
    statusEl.className = "status" + (cls ? " " + cls : "");
  }

  // Only the newest Connect speaks: two attempts answering out of order
  // must not leave the older one's verdict on screen (3.76 N2).
  var connectSeq = 0;

  function connect(auto) {
    var field = $("secret");
    var tried = cleanSecret(field.value);
    if (tried !== field.value) field.value = tried;
    var seq = ++connectSeq;
    setStatus("Connecting…", "busy");
    // /recording/walks, not /health: /health is deliberately unauthenticated,
    // so it cannot tell a right secret from a wrong one. This is also the
    // first real data load. Tried on this request alone and kept only once
    // it worked (3.76 N2): a mistyped secret must not replace the saved one.
    fetch("/recording/walks", { headers: Lab.headers({}, tried) })
      .then(function (res) {
        return res.json().catch(function () { return {}; }).then(function (data) {
          if (!res.ok) {
            var detail = typeof data.detail === "string" ? data.detail : "";
            var err = new Error(detail || ("HTTP " + res.status));
            err.status = res.status;
            throw err;
          }
          return data;
        });
      })
      .then(function (data) {
        if (seq !== connectSeq) return;
        Lab.secret = tried;
        Lab.connected = true;
        try { if (tried) localStorage.setItem(SECRET_KEY, tried); } catch (e) { /* private mode */ }
        setStatus("Connected.", "ok");
        connectListeners.forEach(function (fn) { fn(data); });
      })
      .catch(function (e) {
        if (seq !== connectSeq) return;
        var refused = e.status === 401 || e.status === 403;
        if (auto && !tried && refused) {
          // Opening the page with nothing saved is not a failure.
          setStatus("Paste the secret, then Connect.", "");
          return;
        }
        setStatus("Failed: " + (refused
          ? "the secret was refused. Paste only the value of WALKS_SECRET, not the whole line."
          : e.message), "err");
      });
  }

  $("btn-connect").onclick = function () { connect(false); };
  $("secret").addEventListener("keydown", function (e) {
    if (e.key === "Enter") connect(false);
  });
  $("reveal").onclick = function () {
    var f = $("secret");
    var hidden = f.type === "password";
    f.type = hidden ? "text" : "password";
    this.textContent = hidden ? "Hide" : "Show";
    this.setAttribute("aria-pressed", hidden ? "true" : "false");
  };

  // ---------- the tabs ----------

  var TABS = ["walks", "metrics"];
  var tabListeners = [];
  Lab.onTab = function (fn) { tabListeners.push(fn); };
  Lab.tab = null;

  function tabFromLocation() {
    var h = location.hash.replace(/^#/, "");
    if (TABS.indexOf(h) >= 0) return h;
    // The page is published at /admin, /metrics and /lab: each old address
    // opens where it used to.
    return /\/metrics\/?$/.test(location.pathname) ? "metrics" : "walks";
  }

  function showTab(name, focus) {
    Lab.tab = name;
    TABS.forEach(function (t) {
      var tab = $("tab-" + t);
      var on = t === name;
      tab.setAttribute("aria-selected", on ? "true" : "false");
      tab.tabIndex = on ? 0 : -1;
      $("panel-" + t).hidden = !on;
      if (on && focus) tab.focus();
    });
    try { history.replaceState(null, "", "#" + name); } catch (e) { /* file: or sandbox */ }
    tabListeners.forEach(function (fn) { fn(name); });
  }
  Lab.showTab = showTab;

  TABS.forEach(function (t, i) {
    var tab = $("tab-" + t);
    tab.addEventListener("click", function (e) { e.preventDefault(); showTab(t, false); });
    tab.addEventListener("keydown", function (e) {
      if (e.key !== "ArrowLeft" && e.key !== "ArrowRight") return;
      e.preventDefault();
      var next = (i + (e.key === "ArrowRight" ? 1 : TABS.length - 1)) % TABS.length;
      showTab(TABS[next], true);
    });
  });
  window.addEventListener("hashchange", function () {
    var t = tabFromLocation();
    if (t !== Lab.tab) showTab(t, false);
  });

  // ---------- keeping the keyboard inside a modal ----------

  var FOCUSABLE = "button:not([disabled]), [href], input:not([disabled]), select:not([disabled]), " +
    "textarea:not([disabled]), [tabindex]:not([tabindex='-1'])";

  // Tab and Shift-Tab cycle inside `el`; returns the function that ends it.
  Lab.trapFocus = function (el) {
    function onKey(e) {
      if (e.key !== "Tab") return;
      var items = Array.prototype.filter.call(el.querySelectorAll(FOCUSABLE), function (x) {
        return x.offsetParent !== null || x === document.activeElement;
      });
      if (!items.length) { e.preventDefault(); return; }
      var first = items[0], last = items[items.length - 1];
      if (!el.contains(document.activeElement)) {
        e.preventDefault();
        first.focus();
      } else if (e.shiftKey && document.activeElement === first) {
        e.preventDefault();
        last.focus();
      } else if (!e.shiftKey && document.activeElement === last) {
        e.preventDefault();
        first.focus();
      }
    }
    document.addEventListener("keydown", onKey, true);
    return function () { document.removeEventListener("keydown", onKey, true); };
  };

  // ---------- the confirm every paid action goes through ----------

  var dialog = $("confirm");
  var pending = null;        // the resolve of the question on screen

  function answer(yes) {
    var resolve = pending;
    pending = null;
    if (dialog.open) dialog.close();
    if (resolve) resolve(yes);
  }
  $("confirm-ok").onclick = function () { answer(true); };
  $("confirm-cancel").onclick = function () { answer(false); };
  dialog.addEventListener("cancel", function (e) { e.preventDefault(); answer(false); });
  dialog.addEventListener("close", function () { if (pending) answer(false); });

  // Resolves true only on Confirm. `lines` are plain text, one paragraph each.
  Lab.confirm = function (title, lines, okLabel) {
    if (pending) answer(false);          // a newer question replaces an older one
    $("confirm-title").textContent = title;
    $("confirm-body").innerHTML = lines.map(function (l) {
      return "<p>" + escapeHtml(l) + "</p>";
    }).join("");
    $("confirm-ok").textContent = okLabel || "Confirm";
    return new Promise(function (resolve) {
      pending = resolve;
      dialog.showModal();
      $("confirm-cancel").focus();       // the safe answer is the default one
    });
  };

  // ---------- what an action will cost ----------

  // The server's estimate for one walk, or null if it could not be had.
  Lab.estimate = function (walk, action, modelId) {
    var url = "/recording/walks/" + encodeURIComponent(walk) + "/estimate?action=" + action +
      (modelId ? "&model_id=" + encodeURIComponent(modelId) : "");
    return fetch(url, { headers: Lab.headers() })
      .then(function (r) { return r.ok ? r.json() : null; })
      .catch(function () { return null; });
  };

  function dollars(usd) {
    if (usd < 0.01) return "under $0.01";
    return "about $" + usd.toFixed(2);
  }

  function calls(n) { return n + (n === 1 ? " paid call" : " paid calls"); }

  // The confirm's cost lines for one estimate, or several summed (Score
  // them). An estimate missing from the list makes the total unknown.
  Lab.costLines = function (estimates) {
    var list = Array.isArray(estimates) ? estimates : [estimates];
    if (!list.length || list.some(function (e) { return !e; })) {
      return ["The cost could not be worked out: this may make paid calls."];
    }
    var total = 0, nav = 0, judge = 0, upper = false, unknown = false;
    list.forEach(function (e) {
      total += e.calls;
      nav += e.navigate_calls;
      judge += e.judge_calls;
      upper = upper || e.upper_bound;
      if (e.usd == null) unknown = true;
    });
    if (!total) {
      return [list[0].action === "score"
        ? "Free: the judge is off here, so only the free measurements run."
        : "Free: no frame to send."];
    }
    var parts = [];
    if (nav) parts.push(nav + " to " + (shortModel(list[0].model_id) || "the default model"));
    if (judge) parts.push(judge + " to the judge (" + shortModel(list[0].judge_model_id) + ")");
    var out = [(upper ? "Up to " : "") + calls(total) + ": " + parts.join(", ") + "."];
    out.push(unknown
      ? "Price unknown for this model, so no dollar estimate."
      : "Cost: " + dollars(list.reduce(function (a, e) { return a + e.usd; }, 0)) +
        " (an estimate, at this account's Bedrock rates).");
    return out;
  };

  window.Lab = Lab;

  // The page opens on the tab its address names, then connects with whatever
  // secret is saved -- or none: a local server run without a secret needs
  // none, and a refusal then just asks for one.
  try {
    var saved = localStorage.getItem(SECRET_KEY);
    for (var i = 0; !saved && i < OLD_SECRET_KEYS.length; i++) {
      saved = localStorage.getItem(OLD_SECRET_KEYS[i]);
    }
    $("secret").value = saved || "";
  } catch (e) { /* private mode */ }
  document.addEventListener("DOMContentLoaded", function () {
    showTab(tabFromLocation(), false);
    connect(true);
  });
})();
