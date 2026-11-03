// web-twin/js/icons.js -- moved verbatim from web-twin/app.js (3.132 Stage 2B),
// dedented by the IIFE's two spaces, with `export` added. The rule a move
// follows is in app.js's banner.

// ---------- icon system ----------
// One inline-SVG icon set (stroke-based, currentColor, 24x24 viewBox)
// replaces every OS emoji glyph that used to render inconsistently
// across platforms next to this otherwise monochrome dark UI. Static
// icons (tab bar, D-pad, onboarding rows, ...) are marked in the HTML
// with data-icon="name" and filled in by applyStaticIcons() below;
// dynamic ones (the guide caption icon, the mute button) are set
// directly via ICON.name at their call sites, same lookup table.
const ICON_ATTRS = 'viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"';
export const ICON = {
  chevronUp: '<svg ' + ICON_ATTRS + '><path d="M6 15l6-6 6 6"/></svg>',
  chevronDown: '<svg ' + ICON_ATTRS + '><path d="M6 9l6 6 6-6"/></svg>',
  chevronLeft: '<svg ' + ICON_ATTRS + '><path d="M15 6l-6 6 6 6"/></svg>',
  chevronRight: '<svg ' + ICON_ATTRS + '><path d="M9 6l6 6-6 6"/></svg>',
  eye: '<svg ' + ICON_ATTRS + '><path d="M2 12s3.5-7 10-7 10 7 10 7-3.5 7-10 7-10-7-10-7z"/><circle cx="12" cy="12" r="3"/></svg>',
  close: '<svg ' + ICON_ATTRS + '><path d="M18 6L6 18M6 6l12 12"/></svg>',
  volumeOn: '<svg ' + ICON_ATTRS + '><path d="M4 9v6h4l5 4V5L8 9H4z"/><path d="M16 8a5 5 0 0 1 0 8"/><path d="M19 5a9 9 0 0 1 0 14"/></svg>',
  volumeOff: '<svg ' + ICON_ATTRS + '><path d="M4 9v6h4l5 4V5L8 9H4z"/><path d="M17 9l5 5M22 9l-5 5"/></svg>',
  search: '<svg ' + ICON_ATTRS + '><circle cx="11" cy="11" r="7"/><path d="M21 21l-4.3-4.3"/></svg>',
  target: '<svg ' + ICON_ATTRS + '><circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="3.5"/></svg>',
  warning: '<svg ' + ICON_ATTRS + '><path d="M12 3l9.5 17H2.5L12 3z"/><path d="M12 10v4"/><path d="M12 17.3v.1"/></svg>',
  clock: '<svg ' + ICON_ATTRS + '><circle cx="12" cy="12" r="9"/><path d="M12 7v5l4 2"/></svg>',
  car: '<svg ' + ICON_ATTRS + '><path d="M4 16v-3l1.8-4.8A2 2 0 0 1 7.7 7h8.6a2 2 0 0 1 1.9 1.2L20 13v3"/><path d="M2 16h20"/><circle cx="7" cy="18.2" r="1.6"/><circle cx="17" cy="18.2" r="1.6"/></svg>',
  bot: '<svg ' + ICON_ATTRS + '><rect x="5" y="8" width="14" height="10" rx="2.5"/><path d="M12 8V4.5"/><circle cx="12" cy="3.2" r="1"/><circle cx="9" cy="13" r="1" fill="currentColor"/><circle cx="15" cy="13" r="1" fill="currentColor"/><path d="M9 16.5h6"/></svg>',
  camera: '<svg ' + ICON_ATTRS + '><path d="M4 8h3l1.6-2h6.8L17 8h3a1 1 0 0 1 1 1v9a1 1 0 0 1-1 1H4a1 1 0 0 1-1-1V9a1 1 0 0 1 1-1z"/><circle cx="12" cy="13.2" r="3.4"/></svg>',
  home: '<svg ' + ICON_ATTRS + '><path d="M3 11l9-7 9 7"/><path d="M5 9.5V20h5v-6h4v6h5V9.5"/></svg>',
  mapPin: '<svg ' + ICON_ATTRS + '><path d="M12 21s7-6.4 7-11.5A7 7 0 0 0 5 9.5C5 14.6 12 21 12 21z"/><circle cx="12" cy="9.5" r="2.3"/></svg>',
  settings: '<svg ' + ICON_ATTRS + '><circle cx="12" cy="12" r="3"/><path d="M19.4 15a1.7 1.7 0 0 0 .3 1.9l.1.1a2 2 0 1 1-2.8 2.8l-.1-.1a1.7 1.7 0 0 0-1.9-.3 1.7 1.7 0 0 0-1 1.6V21a2 2 0 1 1-4 0v-.1a1.7 1.7 0 0 0-1-1.6 1.7 1.7 0 0 0-1.9.3l-.1.1a2 2 0 1 1-2.8-2.8l.1-.1a1.7 1.7 0 0 0 .3-1.9 1.7 1.7 0 0 0-1.6-1H3a2 2 0 1 1 0-4h.1a1.7 1.7 0 0 0 1.6-1 1.7 1.7 0 0 0-.3-1.9l-.1-.1a2 2 0 1 1 2.8-2.8l.1.1a1.7 1.7 0 0 0 1.9.3H9a1.7 1.7 0 0 0 1-1.6V3a2 2 0 1 1 4 0v.1a1.7 1.7 0 0 0 1 1.6 1.7 1.7 0 0 0 1.9-.3l.1-.1a2 2 0 1 1 2.8 2.8l-.1.1a1.7 1.7 0 0 0-.3 1.9V9a1.7 1.7 0 0 0 1.6 1H21a2 2 0 1 1 0 4h-.1a1.7 1.7 0 0 0-1.6 1z"/></svg>',
  arrowRight: '<svg ' + ICON_ATTRS + '><path d="M5 12h13M13 6l6 6-6 6"/></svg>',
  glow: '<svg ' + ICON_ATTRS + '><circle cx="12" cy="12" r="4" fill="currentColor" stroke="none"/><circle cx="12" cy="12" r="9" opacity="0.45"/></svg>',
  pulse: '<svg ' + ICON_ATTRS + '><path d="M3 12h4l2-7 4 14 2-7h6"/></svg>',
  frame: '<svg ' + ICON_ATTRS + '><path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5"/></svg>',
  phone: '<svg ' + ICON_ATTRS + '><rect x="7" y="2" width="10" height="20" rx="2.2"/><path d="M11 18h2"/></svg>',
};
export function applyStaticIcons() {
  document.querySelectorAll("[data-icon]").forEach(function (el) {
    const name = el.getAttribute("data-icon");
    if (ICON[name]) el.innerHTML = ICON[name];
  });
}
