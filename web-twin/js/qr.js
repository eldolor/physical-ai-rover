// web-twin/js/qr.js -- moved verbatim from web-twin/app.js (3.132 Stage 2B),
// dedented by the IIFE's two spaces, with `export` added. The rule a move
// follows is in app.js's banner.

// ---------- QR encoder (byte mode, versions 1-20, EC level M or L) ----------
// Inline rather than a library: this page is strictly self-contained (no
// CDN, served by robot/server.py), and the payload here is the shared
// secrets -- handing those to a third-party QR web service would be a
// genuine leak, not just an inconvenience. Implements ISO/IEC 18004 byte
// mode; the block-structure and alignment tables below were generated
// from a reference implementation rather than typed out by hand.

// Generated from the ISO/IEC 18004 tables, not hand-transcribed.
// Per version 1-20 and EC level: [ecCodewordsPerBlock, group1Blocks,
// group1DataCodewords, group2Blocks, group2DataCodewords].
const QR_BLOCKS = {
  L: [[7,1,19,0,0],[10,1,34,0,0],[15,1,55,0,0],[20,1,80,0,0],[26,1,108,0,0],[18,2,68,0,0],[20,2,78,0,0],[24,2,97,0,0],[30,2,116,0,0],[18,2,68,2,69],[20,4,81,0,0],[24,2,92,2,93],[26,4,107,0,0],[30,3,115,1,116],[22,5,87,1,88],[24,5,98,1,99],[28,1,107,5,108],[30,5,120,1,121],[28,3,113,4,114],[28,3,107,5,108]],
  M: [[10,1,16,0,0],[16,1,28,0,0],[26,1,44,0,0],[18,2,32,0,0],[24,2,43,0,0],[16,4,27,0,0],[18,4,31,0,0],[22,2,38,2,39],[22,3,36,2,37],[26,4,43,1,44],[30,1,50,4,51],[22,6,36,2,37],[22,8,37,1,38],[24,4,40,5,41],[24,5,41,5,42],[28,7,45,3,46],[28,10,46,1,47],[26,9,43,4,44],[26,3,44,11,45],[26,3,41,13,42]],
};
// Alignment-pattern centre coordinates per version (empty for v1).
const QR_ALIGN = [[],[],[6,18],[6,22],[6,26],[6,30],[6,34],[6,22,38],[6,24,42],[6,26,46],[6,28,50],[6,30,54],[6,32,58],[6,34,62],[6,26,46,66],[6,26,48,70],[6,26,50,74],[6,30,54,78],[6,30,56,82],[6,30,58,86],[6,34,62,90]];

// GF(256) for Reed-Solomon, primitive polynomial 0x11D.
const GF_EXP = new Uint8Array(512), GF_LOG = new Uint8Array(256);
(function initGaloisField() {
  let x = 1;
  for (let i = 0; i < 255; i++) {
    GF_EXP[i] = x;
    GF_LOG[x] = i;
    x <<= 1;
    if (x & 0x100) x ^= 0x11D;
  }
  for (let i = 255; i < 512; i++) GF_EXP[i] = GF_EXP[i - 255];
})();
function gfMul(a, b) { return (a === 0 || b === 0) ? 0 : GF_EXP[GF_LOG[a] + GF_LOG[b]]; }

// Generator polynomial: product of (x - alpha^i) for i in [0, degree).
// Coefficients are highest-degree-first.
function rsGenerator(degree) {
  let poly = [1];
  for (let i = 0; i < degree; i++) {
    const next = new Array(poly.length + 1).fill(0);
    for (let j = 0; j < poly.length; j++) {
      next[j] ^= poly[j];                        // multiply by x
      next[j + 1] ^= gfMul(poly[j], GF_EXP[i]);  // ...and by alpha^i
    }
    poly = next;
  }
  return poly;
}

function rsRemainder(data, ecLen) {
  const gen = rsGenerator(ecLen);
  const buf = new Uint8Array(data.length + ecLen);
  buf.set(data);
  for (let i = 0; i < data.length; i++) {
    const factor = buf[i];
    if (factor === 0) continue;
    for (let j = 0; j < gen.length; j++) buf[i + j] ^= gfMul(gen[j], factor);
  }
  return buf.slice(data.length);
}

function qrDataCapacity(version, ecc) {
  const spec = QR_BLOCKS[ecc][version - 1];
  return spec[1] * spec[2] + spec[3] * spec[4];
}
// Byte mode's character-count indicator widens at version 10.
function qrCharCountBits(version) { return version <= 9 ? 8 : 16; }

function qrChooseVersion(byteLen, ecc) {
  for (let v = 1; v <= QR_BLOCKS[ecc].length; v++) {
    if (4 + qrCharCountBits(v) + byteLen * 8 <= qrDataCapacity(v, ecc) * 8) return v;
  }
  return 0;
}

// Data codewords -> padded, split into blocks, RS-encoded, interleaved.
function qrCodewords(bytes, version, ecc) {
  const spec = QR_BLOCKS[ecc][version - 1];
  const ecLen = spec[0], g1n = spec[1], g1d = spec[2], g2n = spec[3], g2d = spec[4];
  const totalData = g1n * g1d + g2n * g2d;

  const bits = [];
  const put = function (value, length) {
    for (let i = length - 1; i >= 0; i--) bits.push((value >>> i) & 1);
  };
  put(4, 4);                                  // byte mode
  put(bytes.length, qrCharCountBits(version));
  for (let i = 0; i < bytes.length; i++) put(bytes[i], 8);

  const capacity = totalData * 8;
  for (let i = 0; i < 4 && bits.length < capacity; i++) bits.push(0); // terminator
  while (bits.length % 8 !== 0) bits.push(0);
  for (let i = 0; bits.length < capacity; i++) put(i % 2 === 0 ? 0xEC : 0x11, 8);

  const dataCw = new Uint8Array(totalData);
  for (let i = 0; i < totalData; i++) {
    let v = 0;
    for (let j = 0; j < 8; j++) v = (v << 1) | bits[i * 8 + j];
    dataCw[i] = v;
  }

  const blocks = [];
  let off = 0;
  for (let i = 0; i < g1n; i++) { blocks.push(dataCw.slice(off, off + g1d)); off += g1d; }
  for (let i = 0; i < g2n; i++) { blocks.push(dataCw.slice(off, off + g2d)); off += g2d; }
  const ecBlocks = blocks.map(function (b) { return rsRemainder(b, ecLen); });

  // Interleave: column-wise across blocks, data first then EC.
  const out = [];
  const longest = Math.max(g1d, g2d);
  for (let i = 0; i < longest; i++) {
    for (let b = 0; b < blocks.length; b++) if (i < blocks[b].length) out.push(blocks[b][i]);
  }
  for (let i = 0; i < ecLen; i++) {
    for (let b = 0; b < ecBlocks.length; b++) out.push(ecBlocks[b][i]);
  }
  return out;
}

const QR_MASKS = [
  function (x, y) { return (x + y) % 2 === 0; },
  function (x, y) { return y % 2 === 0; },
  function (x, y) { return x % 3 === 0; },
  function (x, y) { return (x + y) % 3 === 0; },
  function (x, y) { return (Math.floor(y / 2) + Math.floor(x / 3)) % 2 === 0; },
  function (x, y) { return (x * y) % 2 + (x * y) % 3 === 0; },
  function (x, y) { return ((x * y) % 2 + (x * y) % 3) % 2 === 0; },
  function (x, y) { return ((x + y) % 2 + (x * y) % 3) % 2 === 0; },
];

// BCH(15,5), then XOR with the spec's fixed mask so an all-zero format
// never produces an all-zero pattern.
function qrFormatBits(ecc, mask) {
  const data = ((ecc === "L" ? 1 : 0) << 3) | mask;
  let rem = data;
  for (let i = 0; i < 10; i++) rem = (rem << 1) ^ ((rem >>> 9) * 0x537);
  return ((data << 10) | rem) ^ 0x5412;
}
function qrVersionBits(version) {
  let rem = version;
  for (let i = 0; i < 12; i++) rem = (rem << 1) ^ ((rem >>> 11) * 0x1F25);
  return (version << 12) | rem;
}

function qrBuildMatrix(version, ecc, codewords, mask) {
  const size = version * 4 + 17;
  const m = [], fixed = [];
  for (let i = 0; i < size; i++) {
    m.push(new Array(size).fill(0));
    fixed.push(new Array(size).fill(false));
  }
  const setFixed = function (x, y, v) {
    if (x < 0 || y < 0 || x >= size || y >= size) return;
    m[y][x] = v ? 1 : 0;
    fixed[y][x] = true;
  };

  // Finder patterns plus their separators (the ring of light modules).
  [[0, 0], [size - 7, 0], [0, size - 7]].forEach(function (o) {
    for (let dy = -1; dy <= 7; dy++) {
      for (let dx = -1; dx <= 7; dx++) {
        const inside = dx >= 0 && dx <= 6 && dy >= 0 && dy <= 6;
        const d = Math.max(Math.abs(dx - 3), Math.abs(dy - 3));
        setFixed(o[0] + dx, o[1] + dy, inside && d !== 2);
      }
    }
  });

  // Timing patterns.
  for (let i = 8; i < size - 8; i++) {
    setFixed(i, 6, i % 2 === 0);
    setFixed(6, i, i % 2 === 0);
  }

  // Alignment patterns, minus the three that would sit on a finder.
  const centres = QR_ALIGN[version];
  for (let a = 0; a < centres.length; a++) {
    for (let b = 0; b < centres.length; b++) {
      const cx = centres[a], cy = centres[b];
      const onFinder = (a === 0 && b === 0) ||
                       (a === 0 && b === centres.length - 1) ||
                       (a === centres.length - 1 && b === 0);
      if (onFinder) continue;
      for (let dy = -2; dy <= 2; dy++) {
        for (let dx = -2; dx <= 2; dx++) {
          setFixed(cx + dx, cy + dy, Math.max(Math.abs(dx), Math.abs(dy)) !== 1);
        }
      }
    }
  }

  // Reserve the format-info strips (written properly further down) and
  // the always-dark module, so data placement skips them.
  // Index 6 is skipped: the format strip runs alongside the timing
  // patterns but does not include them, and writing here would erase the
  // timing modules at (8,6) and (6,8).
  for (let i = 0; i <= 8; i++) {
    if (i === 6) continue;
    setFixed(8, i, false);
    setFixed(i, 8, false);
  }
  for (let i = 0; i < 8; i++) { setFixed(size - 1 - i, 8, false); setFixed(8, size - 1 - i, false); }
  setFixed(8, size - 8, true);

  if (version >= 7) {
    const vbits = qrVersionBits(version);
    for (let i = 0; i < 18; i++) {
      const bit = (vbits >>> i) & 1;
      setFixed(size - 11 + (i % 3), Math.floor(i / 3), bit);
      setFixed(Math.floor(i / 3), size - 11 + (i % 3), bit);
    }
  }

  // Data, in two-module columns snaking up and down, skipping column 6.
  let bitIdx = 0, upward = true;
  for (let right = size - 1; right >= 1; right -= 2) {
    if (right === 6) right = 5;
    for (let vert = 0; vert < size; vert++) {
      const y = upward ? size - 1 - vert : vert;
      for (let k = 0; k < 2; k++) {
        const x = right - k;
        if (fixed[y][x]) continue;
        let bit = 0;
        if (bitIdx < codewords.length * 8) {
          bit = (codewords[bitIdx >> 3] >>> (7 - (bitIdx & 7))) & 1;
        }
        bitIdx++;
        m[y][x] = bit ^ (QR_MASKS[mask](x, y) ? 1 : 0);
      }
    }
    upward = !upward;
  }

  const fbits = qrFormatBits(ecc, mask);
  const fbit = function (i) { return (fbits >>> i) & 1; };
  for (let i = 0; i <= 5; i++) setFixed(8, i, fbit(i));
  setFixed(8, 7, fbit(6));
  setFixed(8, 8, fbit(7));
  setFixed(7, 8, fbit(8));
  for (let i = 9; i < 15; i++) setFixed(14 - i, 8, fbit(i));
  for (let i = 0; i < 8; i++) setFixed(size - 1 - i, 8, fbit(i));
  for (let i = 8; i < 15; i++) setFixed(8, size - 15 + i, fbit(i));
  setFixed(8, size - 8, 1);

  return m;
}

// The four penalty rules from the spec; the lowest-scoring mask wins.
function qrPenalty(m) {
  const size = m.length;
  let score = 0;

  for (let pass = 0; pass < 2; pass++) {
    for (let i = 0; i < size; i++) {
      let run = 1;
      for (let j = 1; j < size; j++) {
        const cur = pass === 0 ? m[i][j] : m[j][i];
        const prev = pass === 0 ? m[i][j - 1] : m[j - 1][i];
        if (cur === prev) {
          run++;
          if (run === 5) score += 3;
          else if (run > 5) score += 1;
        } else run = 1;
      }
    }
  }

  for (let y = 0; y < size - 1; y++) {
    for (let x = 0; x < size - 1; x++) {
      const v = m[y][x];
      if (v === m[y][x + 1] && v === m[y + 1][x] && v === m[y + 1][x + 1]) score += 3;
    }
  }

  // 1:1:3:1:1 finder-like sequences, with four light modules either side.
  const PATTERNS = [
    [1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1],
  ];
  for (let pass = 0; pass < 2; pass++) {
    for (let i = 0; i < size; i++) {
      for (let j = 0; j + 11 <= size; j++) {
        for (let p = 0; p < 2; p++) {
          let hit = true;
          for (let k = 0; k < 11; k++) {
            const v = pass === 0 ? m[i][j + k] : m[j + k][i];
            if (v !== PATTERNS[p][k]) { hit = false; break; }
          }
          if (hit) score += 40;
        }
      }
    }
  }

  let dark = 0;
  for (let y = 0; y < size; y++) for (let x = 0; x < size; x++) dark += m[y][x];
  const percent = (dark * 100) / (size * size);
  score += Math.floor(Math.abs(percent - 50) / 5) * 10;
  return score;
}

// Returns { matrix, version, ecc, mask } or throws if the text cannot fit.
// forcedMask exists so the encoder can be checked module-for-module
// against a reference implementation at a known mask.
export function qrEncode(text, forcedMask) {
  const bytes = Array.from(new TextEncoder().encode(text));
  // Prefer M (roughly 15% recoverable) since this is scanned off a
  // screen; fall back to L only if the payload will not otherwise fit.
  let ecc = "M", version = qrChooseVersion(bytes.length, ecc);
  if (!version) { ecc = "L"; version = qrChooseVersion(bytes.length, ecc); }
  if (!version) throw new Error("Too much data for a QR code (" + bytes.length + " bytes).");

  const codewords = qrCodewords(bytes, version, ecc);
  if (typeof forcedMask === "number") {
    return { matrix: qrBuildMatrix(version, ecc, codewords, forcedMask), version: version, ecc: ecc, mask: forcedMask };
  }
  let best = null;
  for (let mask = 0; mask < 8; mask++) {
    const matrix = qrBuildMatrix(version, ecc, codewords, mask);
    const penalty = qrPenalty(matrix);
    if (!best || penalty < best.penalty) best = { matrix: matrix, penalty: penalty, mask: mask };
  }
  return { matrix: best.matrix, version: version, ecc: ecc, mask: best.mask };
}

// Drawn light-on-dark-agnostic: always true black on true white, never
// the page's theme colours -- scanners need the contrast, and a dark-mode
// QR in muted greys is measurably worse to read.
export function qrRenderToCanvas(canvas, matrix, targetPx) {
  const QUIET = 4;
  const size = matrix.length + QUIET * 2;
  const scale = Math.max(2, Math.floor(targetPx / size));
  const px = size * scale;
  const dpr = window.devicePixelRatio || 1;
  canvas.width = px * dpr;
  canvas.height = px * dpr;
  canvas.style.width = px + "px";
  canvas.style.height = px + "px";
  const c = canvas.getContext("2d");
  c.setTransform(dpr, 0, 0, dpr, 0, 0);
  c.fillStyle = "#FFFFFF";
  c.fillRect(0, 0, px, px);
  c.fillStyle = "#000000";
  for (let y = 0; y < matrix.length; y++) {
    for (let x = 0; x < matrix.length; x++) {
      if (matrix[y][x]) c.fillRect((x + QUIET) * scale, (y + QUIET) * scale, scale, scale);
    }
  }
}
