// engine/renderer.js — 2D side-view / 2.5D oblique canvas renderer + debug overlay.
// Consumes ONLY plain data: PhysicsWorld.getStatic()/getDynamic() and Vehicle.getShape()/getState().
// It never imports or calls the physics engine (trail.js is shared pure geometry: berms, zone dy, rut-grid lanes).
import { paintIcon } from './icons.js';
import { TRAIL, laneOffset, rutAt, laneZ, zoneAtZ } from './trail.js';
import { Spotter } from './spotter.js';
import { glowSprite, fanSprite, throwSprite } from './lights.js';

const TAU = Math.PI * 2;
const BODY_COLORS = { frame: '#2a2d33', hood: '#d4622b', cab: '#d4622b', bed: '#b9531f' };
const lerp = (a, b, t) => a + (b - a) * t;
const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
const smooth01 = t => (t <= 0 ? 0 : t >= 1 ? 1 : t * t * (3 - 2 * t));
// 2.5D camera: 3/4 rear-right oblique, depth z = vehicle right (+z = near side, R wheels), screen = (x + z·cos30°, y − z·sin30°)
const C30 = Math.cos(Math.PI / 6), S30 = 0.5;
// near-field headlight pool (_nearPool): strips, length m (from `back` m behind the front tyre face), width m, alpha x lamp,
// 2D band depth / lift above the surface line
const NEAR = { n: 7, len: 6, back: 0.35, w: 3.4, a: 0.42, band: 0.75, lift: 0.08 };
// rock step faces (_terrainSlab, r13): a quad rising >= steep x its run is a face (riser shade); faces >= min m tall get a dark
// foot line (foot m wide) + light crest line (crest m) and are recorded (≤ max per frame) for the night headlight catch
// (_faceCatch, in the slab pass: faces whose foot is -0.3..reach m past the front tyres, within half m of the beam centre
// line, warm additive fill a x lamp (strong: the night tint overlays it afterwards), full to `full` m then fading)
const FACE = { steep: 1.2, min: 0.12, foot: 0.07, crest: 0.035, max: 640, reach: 9, full: 3, half: 2.7, soft: 1.2, a: 0.55 };
// headlight catch (r15): beam fall-off ~1/(1 + (d/d0)^2) from the front, soft cone widening w + k·d, faces higher than
// `hi` m above the front ground fade to `top` (low beams aim down), world-fixed depth cells `cell` m (never re-cut by the
// moving slab bounds), K strength buckets, params low-passed over tau s, nothing below lamp0 atmosphere lamp (overcast / day)
// fractured rock faces (_faceFx, r15): world depth grid step (m), contact shadow width foot0 + foot1·noise, crest jag depth,
// share of grid cells with a crest notch / a fracture crack, crack half-width at the crest
const FRACT = { step: 0.16, foot0: 0.04, foot1: 0.12, jag: 0.06, notch: 0.3, crack: 0.42, crackW: 0.018 };
const CATCH = { d0: 3.5, w: 1.0, k: 0.22, soft: 1.1, hi: 0.7, hiR: 1.5, top: 0.25, cell: 0.35, K: 8, tau: 0.2, lamp0: 0.15 };
const GORGE = 0.3;                  // clearance (m) under a bridge plank's swept screen footprint for the draw-once path
const cmpItem = (a, b) => (a.x1 <= b.x0 ? 1 : b.x1 <= a.x0 ? -1 : a.ym - b.ym);   // same slab: nearer (−x) / higher last
// slab draw order: truck parts by local z; a plane lying on the slab's far bound (za) is behind every ranged piece of it
const atFar = (it, za) => it.za === it.zb && it.za - za < 0.02;
// screen boxes (z = 0 frame: x + z·cos30, y − z·sin30): prop p's box swept over depths a..b (a hexagon) vs truck part
// t's silhouette box hb -- separating axes x, y and the sweep normal (sin30, cos30), on which the sweep adds nothing
const overlap = (p, a, b, t) => { const h = t.hb;
  return p.x1 + b * C30 > h[0] && p.x0 + a * C30 < h[2] && p.y1 - a * S30 > h[1] && p.y0 - b * S30 < h[3]
    && p.x1 * S30 + p.y1 * C30 > h[0] * S30 + h[1] * C30 && p.x0 * S30 + p.y0 * C30 < h[2] * S30 + h[3] * C30; };
// Convex hull: typed scratch buffers (no per-call garbage: hulls run per tyre lug every frame; emptying a JS array with
// length = 0 drops its storage in V8 = a fresh allocation every push). hullN -> HB (floats written); insertion sort (n <= 512).
const HB = new Float64Array(1024), _hI = new Int32Array(512), _hL = new Int32Array(512), _hU = new Int32Array(512);
const _hCr = (F, o, a, b) => (F[2 * a] - F[2 * o]) * (F[2 * b + 1] - F[2 * o + 1]) - (F[2 * a + 1] - F[2 * o + 1]) * (F[2 * b] - F[2 * o]);
function hullN(F, nf) {
  const n = Math.min(512, nf >> 1), I = _hI;
  for (let i = 0; i < n; i++) {
    const x = F[2 * i], y = F[2 * i + 1]; let j = i - 1;
    while (j >= 0 && (F[2 * I[j]] > x || (F[2 * I[j]] === x && F[2 * I[j] + 1] > y))) { I[j + 1] = I[j]; j--; }
    I[j + 1] = i;
  }
  let nl = 0, nu = 0;
  for (let k = 0; k < n; k++) { const i = I[k]; while (nl > 1 && _hCr(F, _hL[nl - 2], _hL[nl - 1], i) <= 0) nl--; _hL[nl++] = i; }
  for (let k = n - 1; k >= 0; k--) { const i = I[k]; while (nu > 1 && _hCr(F, _hU[nu - 2], _hU[nu - 1], i) <= 0) nu--; _hU[nu++] = i; }
  let c = 0;
  for (let k = 0; k < nl - 1; k++) { HB[c++] = F[2 * _hL[k]]; HB[c++] = F[2 * _hL[k] + 1]; }
  for (let k = 0; k < nu - 1; k++) { HB[c++] = F[2 * _hU[k]]; HB[c++] = F[2 * _hU[k] + 1]; }
  return c;
}
/** Convex hull of a flat [x, y, ...] point list (monotone chain) -> new flat CCW list (kept results, e.g. a prop's it.hull). */
function hull(F) { const c = hullN(F, F.length), out = new Array(c); for (let k = 0; k < c; k++) out[k] = HB[k]; return out; }
/** Hull of the first nf floats of F straight into the current path (moveTo / lineTo / closePath). */
function hullPath(ctx, F, nf = F.length) { const c = hullN(F, nf); if (c < 2) return; ctx.moveTo(HB[0], HB[1]); for (let i = 2; i < c; i += 2) ctx.lineTo(HB[i], HB[i + 1]); ctx.closePath(); }
// tyre lug spans across the tread [w0, w1, off0, off1] (paddle: two of them) into a reused buffer
const LUG_G = new Float64Array(16), TREAD_F = new Float64Array(96), SEG_TMP = new Float64Array(8), LUG_LR = [0, -0.58, 0, -0.46, 0, -0.36, 0, 0.36, 0, 0.46, 0, 0.58];
/** Tiny stone / pebble as a hexagon subpath (rx, ry radii): at 2-8 px it reads exactly like an ellipse, but is 6 vertices
 *  instead of a tessellated curve -- gravel strips add thousands per frame (GPU path cost on phones). */
function hexDot(ctx, x, y, rx, ry) {
  ctx.moveTo(x + rx, y); ctx.lineTo(x + rx * 0.5, y + ry * 0.87); ctx.lineTo(x - rx * 0.5, y + ry * 0.87);
  ctx.lineTo(x - rx, y); ctx.lineTo(x - rx * 0.5, y - ry * 0.87); ctx.lineTo(x + rx * 0.5, y - ry * 0.87); ctx.closePath();
}
function lugSegs(S, pad, blk, k, a0, a1, f) {
  if (pad) { S[0] = a0; S[1] = 0; S[2] = 0; S[3] = f; S[4] = 0; S[5] = a1; S[6] = f; S[7] = 0; }
  else if (blk) { if (k & 1) { S[0] = a0; S[1] = 0.04; } else { S[0] = -0.04; S[1] = a1; } S[2] = 0; S[3] = 0; }
  else { S[0] = a0; S[1] = a1; S[2] = 0; S[3] = 0; }
  return S;
}
// box (out) of the truck parts in it.ks picked by mode: 'before' = parts this slab-k redraw is the last one before (in slab
// k after the prop, or in slab k + 1 before it), 'band' = parts drawn before the prop in slab k (it paints its slab band
// over them), 'all' = every part it may touch. false: none.
const scissor = (it, k, out, mode) => {
  let any = false;
  const ks = it.ks, ka = it.ka;
  for (let i = 0; i < ks.length; i++) {
    const t = ks[i];
    if (mode === 'before' ? !((t.slab === k && !ka[i]) || (t.slab === k + 1 && ka[i])) : mode === 'band' ? !(t.slab === k && ka[i]) : false) continue;
    const h = t.hb;
    if (!any) { out[0] = h[0]; out[1] = h[1]; out[2] = h[2]; out[3] = h[3]; any = true; }
    else { out[0] = Math.min(out[0], h[0]); out[1] = Math.min(out[1], h[1]); out[2] = Math.max(out[2], h[2]); out[3] = Math.max(out[3], h[3]); }
  }
  return any;
};
const cmpSlab = (a, b, za) => (a.ord != null && b.ord != null ? a.ord - b.ord
  : atFar(a, za) !== atFar(b, za) ? (atFar(a, za) ? -1 : 1) : cmpItem(a, b));

// -- 2.5D particles + tyre tracks (pure visuals, fed from getState()) -----------------------------
// Graphics quality (MENU -> GRAPHICS, setQuality): render resolution cap (x CSS px, never above the screen's DPR), surface
// detail (false = no slab / wall detail pass), particle cap, lite (thinned scenery, applied at the next setLevel). AUTO only moves
// the resolution between NOTCHES (sharpness, never content pops) on sustained evidence -- see _autoTick --; at 1x it sheds detail
// once (lite: fewer particles now, thinner scenery from the next level load), then caps at 30 fps. Its result is remembered
// per device (autoMemory / onAutoMemory -> glue localStorage); every RELAX launches without a new step down undoes one step.
export const QUALITY = { low: { dpr: 1, detail: false, fx: 70, lite: true }, med: { dpr: 1.5, detail: true, fx: 130 }, high: { dpr: 2, detail: true, fx: 180 } };
export const QUALITY_MODES = ['auto', 'low', 'med', 'high'];
const NOTCHES = [1, 1.25, 1.5, 1.75, 2, 2.5];
const AUTO = { win: 60, lag: 1.06, smooth: 1.025, downAfter: 2, upAfter: 10, cool: 4, warm: 2.5, relax: 3, minHz: 55 };   // frames / window, ratios to the refresh interval, windows, s
const FX_MAX = 180;                 // hard cap on live particles (mobile budget); per-tier cap q.fx, roost <= 83 % of it
const TRACK_MAX = 360;              // skid-mark segments in the ring buffer
// atmosphere presets (renderer.setAtmosphere; random per playthrough): sky gradient, 4 parallax ridges far->near,
// screen tint over the world, sun / moon, stars + headlights at night (lamp = headlight strength: night 1, dusk / overcast dim, day off)
export const ATMOS = {
  day:      { sky: ['#82b1da', '#dde8d6', '#d3c79f'], ridges: ['#adc3cf', '#93afab', '#6f957a', '#4a7450'], tint: null, sun: [0.8, 0.14, 0.045, '#fff7dc'] },
  dusk:     { sky: ['#3a4985', '#e48a62', '#f6c686'], ridges: ['#9a84a6', '#74658e', '#524d70', '#38404e'], tint: 'rgba(255,130,60,0.13)', sun: [0.18, 0.5, 0.07, '#ffd08a'], lamp: 0.45 },
  overcast: { sky: ['#8f9ba6', '#c3cac9', '#b9b6a6'], ridges: ['#a8b2b6', '#8f9e9a', '#738a7f', '#557062'], tint: 'rgba(110,122,132,0.16)', sun: null, lamp: 0.1 },
  night:    { sky: ['#060a1a', '#18223e', '#2a3146'], ridges: ['#28304a', '#1f273d', '#171d2d', '#11161f'], tint: 'rgba(6,12,36,0.52)', sun: [0.74, 0.16, 0.035, '#e8eef8'],
              stars: true, lights: true, lamp: 1 },
};
// foliage palettes: dark forest greens + bright parrot-green accents (so the vegetation reads at a glance)
const FOREST = ['#1f4219', '#295221', '#33652c', '#193815', '#40733a'], PARROT = ['#66cc30', '#7fdc42', '#55bd2b', '#93e352'];
// level `scenery` palettes [dark, bright]: lush forest (default), alpine conifers, dry ridge scrub, red-canyon desert scrub
const PALETTES = { forest: [FOREST, PARROT], alpine: [['#16301a', '#1d3b22', '#24462a', '#132a17'], ['#4f9a3a', '#5fae44', '#3f8a33']],
  dry: [['#4d4a2a', '#5a5631', '#3f3d24', '#646035'], ['#8f8a4a', '#a39b55', '#7d7a42']], canyon: [['#4a4326', '#57502c', '#3d3720'], ['#9a8c4c', '#b09d58', '#8a7c40']] };
// level `scenery`: far-edge plant mix (pine / bush / fern shares, rest grass), spacing gap0..gap0+gapR m, bright-palette share
const hashStr = t => { let h = 2166136261; for (let i = 0; i < t.length; i++) { h ^= t.charCodeAt(i); h = Math.imul(h, 16777619); } return h >>> 0; };
const rngOf = seed => { let a = seed || 1; return () => { a = (a + 0x6D2B79F5) | 0; let t = Math.imul(a ^ (a >>> 15), 1 | a); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; };
const h01 = (i, k = 0) => { const v = Math.sin(i * 127.1 + k * 311.7) * 43758.5453; return v - Math.floor(v); };   // stable per-index noise
const CLS = new Map();   // surface name -> slab-detail class 0 plain 1 rock 2 wet 3 water 4 gravel (regexes once per name)
const surfCls = n => { let c = CLS.get(n); if (c === undefined) CLS.set(n, c = isWater(n) ? 3 : isWet(n) ? 2 : isRock(n) ? 1 : isGravel(n) ? 4 : 0); return c; };
const isRock = n => /rock|stone|boulder/.test(n), isGravel = n => /gravel|scree/.test(n), isWet = n => /mud|bog|clay|slick|swamp/.test(n), isWater = n => /ford|water|river/.test(n);

const RIVER = '#357f8c';                                           // gorge river surface
/** Drawn tread depth (m): authored depth scaled by tyre size, thick enough to read at phone size. */
const treadDepth = (r, tr) => clamp((tr.depth || 0.05) * 1.7 * (r / 0.48), 0.055, 0.13);
const SHOW_PL = 3.6, SHOW_PW = 1.6;   // showroom glass stand half-length / half-width (m): fits every truck
const BODY_ROLL = 0.85;   // sprung body share of the mean solid-axle tilt (render-only body roll)
const LIP_DOTS = [['rgba(0,0,0,0.32)', -1], ['#8d8274', 0], ['rgba(255,245,225,0.35)', 1]];   // [colour, y nudge]: shadow, stone, highlight
const GRAVEL_DOTS = [['rgba(0,0,0,0.24)', 0], ['rgba(255,250,235,0.24)', 50]];
const SKY_LAYERS = [[0.03, 0.47, 0.12, 0], [0.07, 0.55, 0.09, 1], [0.13, 0.63, 0.07, 2], [0.22, 0.71, 0.05, 3]], SKY_BASES = SKY_LAYERS.map(l => l[1]);
const TRACK_LIFE = 30;              // s until a mark has faded out
const K = { BLOB: 0, DUST: 1, CHIP: 2, DROP: 3, SMOKE: 4, SPARK: 5 };
// roost particle kind per surface, matched by surface name (first regex hit wins)
const ROOST = [
  [K.BLOB, /mud|clay|bog|swamp/],
  [K.CHIP, /rock|stone|boulder|gravel/],
  [K.DROP, /wet|water|ice|snow|ford|river/],
  [K.DUST, /./],
];
const hexRgb = h => { if (!/^#[0-9a-f]{3,6}$/i.test(h)) return [120, 120, 120]; const n = parseInt(h.slice(1), 16); return h.length === 4
  ? [((n >> 8) & 15) * 17, ((n >> 4) & 15) * 17, (n & 15) * 17] : [(n >> 16) & 255, (n >> 8) & 255, n & 255]; };
const mix = (h, to, t) => { const a = hexRgb(h), b = hexRgb(to);
  return `rgb(${Math.round(lerp(a[0], b[0], t))},${Math.round(lerp(a[1], b[1], t))},${Math.round(lerp(a[2], b[2], t))})`; };
/** Grounded colour grade for authored terrain colours: a little more saturation (capped, never neon) + contrast around
 *  mid-grey, earth tones nudged warmer, greens deeper + richer, near-greys a faint warm stone cast, water a touch bluer-teal. */
function gradeHex(hex) {
  let [r, g, b] = hexRgb(hex).map(v => v / 255);
  const mx = Math.max(r, g, b), mn = Math.min(r, g, b); let h = 0, s = 0, l = (mx + mn) / 2;
  if (mx > mn) { const d = mx - mn; s = l > 0.5 ? d / (2 - mx - mn) : d / (mx + mn);
    h = (mx === r ? (g - b) / d + (g < b ? 6 : 0) : mx === g ? (b - r) / d + 2 : (r - g) / d + 4) * 60; }
  if (s < 0.08) { h = 34; s = Math.max(s, 0.07); }                       // grey rock / scree: faint warm stone
  else if (h >= 12 && h < 58) { h -= 3; s *= 1.12; }                     // earth / clay / mud: warmer, richer
  else if (h >= 58 && h < 170) { s *= 1.15; l *= 0.95; }                 // greens: deeper
  else if (h >= 170 && h < 215) { h += 4; s *= 1.12; }                   // water / teal
  s = clamp(s * 1.12 + 0.02, 0, 0.78); l = clamp(0.5 + (l - 0.5) * 1.1, 0.04, 0.94);
  const q = l < 0.5 ? l * (1 + s) : l + s - l * s, P = 2 * l - q, f = t => { t = (t + 1) % 1;
    return t < 1 / 6 ? P + (q - P) * 6 * t : t < 0.5 ? q : t < 2 / 3 ? P + (q - P) * (2 / 3 - t) * 6 : P; };
  return '#' + [f(h / 360 + 1 / 3), f(h / 360), f(h / 360 - 1 / 3)].map(v => Math.round(clamp(v, 0, 1) * 255).toString(16).padStart(2, '0')).join('');
}
/** Showroom-fresh paint: saturation x1.25 (capped) + a little contrast, so bodies don't read washed out under the tints. */
function paintHex(hex) {
  let [r, g, b] = hexRgb(hex).map(v => v / 255);
  const mx = Math.max(r, g, b), mn = Math.min(r, g, b); let h = 0, s = 0, l = (mx + mn) / 2;
  if (mx > mn) { const d = mx - mn; s = l > 0.5 ? d / (2 - mx - mn) : d / (mx + mn);
    h = (mx === r ? (g - b) / d + (g < b ? 6 : 0) : mx === g ? (b - r) / d + 2 : (r - g) / d + 4) * 60; }
  if (s < 0.12) return hex;                                             // greys / blacks stay as authored
  s = clamp(s * 1.25, 0, 0.92); l = clamp(0.5 + (l - 0.5) * 1.08, 0.05, 0.9);
  const q = l < 0.5 ? l * (1 + s) : l + s - l * s, P = 2 * l - q, f = t => { t = (t + 1) % 1;
    return t < 1 / 6 ? P + (q - P) * 6 * t : t < 0.5 ? q : t < 2 / 3 ? P + (q - P) * (2 / 3 - t) * 6 : P; };
  return '#' + [f(h / 360 + 1 / 3), f(h / 360), f(h / 360 - 1 / 3)].map(v => Math.round(clamp(v, 0, 1) * 255).toString(16).padStart(2, '0')).join('');
}
// dirt build-up per metre driven, by surface under the tyres (water rinses), + per m/s of wheelspin per s (roost)
const DIRT = { mud: 0.009, dirt: 0.0022, rock: 0.0008, water: -0.004, spin: 0.0012, col: '#5b4630', fleck: '#3e2f20' };
const mixHex = (h, to, t) => { const a = hexRgb(h), b = hexRgb(to);   // hex result (safe to feed _shades)
  return '#' + a.map((v, i) => Math.round(lerp(v, b[i], t)).toString(16).padStart(2, '0')).join(''); };

export class Renderer {
  constructor(canvas, opts = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d', { alpha: false });
    this.viewHeight = opts.viewHeight || 8.3;   // metres visible vertically (landscape; was 9.5: ~14 % closer, less sky)
    this.minViewWidth = opts.minViewWidth || 15;
    this.cam = { x: 0, y: 0, ready: false };
    this.level = null;
    this.segs = [];
    this.shape = null;
    this.t = 0;
    // particle pool: packed struct-of-arrays, swap-remove on death (no per-frame allocation)
    const F = n => new Float32Array(n);
    this.fx = { n: 0, x: F(FX_MAX), y: F(FX_MAX), vx: F(FX_MAX), vy: F(FX_MAX), life: F(FX_MAX), max: F(FX_MAX),
      z: F(FX_MAX), vz: F(FX_MAX), size: F(FX_MAX), grow: F(FX_MAX), alpha: F(FX_MAX), kind: new Uint8Array(FX_MAX), layer: new Uint8Array(FX_MAX),
      col: new Uint8Array(FX_MAX), stuck: new Uint8Array(FX_MAX), acc: new Float32Array(8) };
    this.palette = [];
    this.tracks = { head: 0, x0: F(TRACK_MAX), y0: F(TRACK_MAX), x1: F(TRACK_MAX), y1: F(TRACK_MAX), z: F(TRACK_MAX), born: F(TRACK_MAX).fill(-1e9),
      a: F(TRACK_MAX), w: F(TRACK_MAX), s: new Uint8Array(TRACK_MAX), last: [] };
    // game feel: camera trauma (0..1, shake ∝ trauma²), screen flash, rope-snap whip, per-source cooldowns + edge detectors
    this.shakeScale = opts.shake ?? (typeof matchMedia === 'function' && matchMedia('(prefers-reduced-motion: reduce)').matches ? 0.3 : 1);
    this.feel = { trauma: 0, t: 0, ph: [Math.random() * TAU, Math.random() * TAU], flash: 0, flashCol: '255,80,50', whip: null,
      cool: new Float32Array(8), comp: new Float32Array(4), axle: [false, false], wBroken: false, wPrev: null, upside: false, dmg: -1 };
    this.dims = null; this._shadeMap = new Map(); this._zcol = new WeakMap(); this._pa = [0, 0]; this._pb = [0, 0]; this._P = new Float64Array(40);
    this._dz = [C30, -S30]; this._zsBuf = []; this.trail = TRAIL; this.dyZones = [];
    this.setAtmosphere(opts.atmo);
    this.q = { mode: 'high', dpr: 2, detail: true, fx: FX_MAX, lite: false, cap: 0 };   // effective quality (setQuality); 2x = the old fixed cap
    this._auto = null;
    this.spotter = new Spotter();    // top-down SPOTTER inset (frame.spot {box, mode})
    this.autoMemory = null;          // glue: saved AUTO result {dpr, ceil, lite, cap, age} (set before the first setQuality)
    this.onAutoMemory = null;        // glue: (mem) => save; called when AUTO steps (and once at load when it relaxes)
    this.resize();
  }

  /** Graphics quality: 'auto' (default) | 'low' | 'med' | 'high'. AUTO starts at the old fixed setting (min(DPR, 2)) and adapts. */
  setQuality(mode = 'auto') {
    mode = QUALITY_MODES.includes(mode) ? mode : 'auto';
    if (mode === 'auto' && this._auto) return;                        // already adapting: keep its state (no reset, no jump)
    const dev = typeof window !== 'undefined' ? window.devicePixelRatio || 1 : 1;
    if (mode === 'auto') {
      // start where the picture already is (switching to AUTO mid-run changes nothing on screen); first load = old fixed 2x cap
      // first AUTO of a launch with a saved result: start there (relaxed one step every AUTO.relax launches without a step down)
      const top = NOTCHES.filter(n => n <= Math.max(1, dev) + 1e-6), M = !this._qSet && this._memIn();
      const cur = M ? M.dpr : this._qSet ? this.dpr : 2, idx = d => Math.max(0, top.filter(n => n <= d + 1e-6).length - 1);
      const start = top[idx(cur)];
      this._qSet = true;
      const ceil = M ? Math.max(idx(start), idx(M.ceil)) : this._mem ? Math.max(idx(start), idx(this._mem.ceil)) : top.length - 1;
      const lite = M ? M.lite : this._mem ? this._mem.lite : false, cap = M ? M.cap : this._mem ? this._mem.cap : 0;
      this._auto = { notches: top, i: idx(start), ceil, ivs: new Float32Array(AUTO.win), n: 0, last: 0,
        ref: Infinity, bad: 0, good: 0, since: -AUTO.cool, t: 0, detail: !lite, age: M ? M.age : this._mem ? this._mem.age : 0 };
      this.q = { mode, dpr: start, detail: !lite, fx: lite ? QUALITY.low.fx : FX_MAX, lite, cap };
      if (M) this._memOut();
    } else {
      this._auto = null; this._qSet = true;
      const Q = QUALITY[mode];
      this.q = { mode, dpr: Q.dpr, detail: Q.detail, fx: Q.fx, lite: !!Q.lite, cap: 0 };
    }
    this.resize();
  }

  /** Saved AUTO result -> sanitized start point; each launch ages it, every AUTO.relax launches it undoes its last step
   *  (30 fps cap -> lite -> one notch of ceiling), so a device that was hot / throttled once gets retried. */
  _memIn() {
    const m = this.autoMemory;
    if (!m || typeof m !== 'object' || !(m.dpr > 0)) return null;
    const M = { dpr: +m.dpr, ceil: Math.max(+m.ceil || 0, +m.dpr), lite: !!m.lite, cap: m.cap === 30 ? 30 : 0, age: (m.age | 0) + 1 };
    if (M.age >= AUTO.relax) {
      M.age = 0;
      if (M.cap) M.cap = 0;
      else if (M.lite) M.lite = false;
      else M.ceil = NOTCHES.find(n => n > M.ceil + 1e-6) || M.ceil;
    }
    this._mem = M;
    return M;
  }
  _memOut() {
    const A = this._auto;
    if (!A) return;
    this._mem = { dpr: A.notches[A.i], ceil: A.notches[A.ceil], lite: this.q.lite, cap: this.q.cap, age: A.age };
    try { this.onAutoMemory?.({ ...this._mem }); } catch (_) {}
  }

  /** Frame-rate cap the glue honours (0 = none, 30 = AUTO's last resort): skip rAF callbacks closer than ~1/cap s. */
  get frameCap() { return this.q.cap || 0; }

  /** Effective quality for the MENU readout: { mode, dpr (actual canvas scale), detail, fps (AUTO measurement, else null) }. */
  get quality() {
    const A = this._auto;
    return { mode: this.q.mode, dpr: this.dpr, detail: this.q.detail, lite: this.q.lite, cap: this.q.cap, fps: A && A.ref < Infinity && A.med ? Math.round(1000 / A.med) : null };
  }

  /** AUTO: per frame, from the real frame interval. Windows of AUTO.win frames: median vs the display's refresh interval (the
   *  lowest window median seen). Lagging windows in a row -> one notch down (and that notch becomes the ceiling: never retried);
   *  a long run of smooth windows below the ceiling -> one notch up. Cooldown between steps; pauses / hidden tabs (gaps > 100 ms)
   *  are ignored. Returns true when the canvas scale changed (the caller re-sizes before drawing: no blank frame). */
  _autoTick() {
    const A = this._auto, now = performance.now(), dt = now - A.last;
    A.last = now;
    if (!(dt > 0 && dt < 100)) return false;
    A.t += dt / 1000;
    A.ivs[A.n++] = dt;
    if (A.n < AUTO.win) return false;
    A.n = 0;
    const m = Array.from(A.ivs).sort((a, b) => a - b)[AUTO.win >> 1];
    let sum = 0; for (let i = 0; i < AUTO.win; i++) sum += A.ivs[i];
    const mean = sum / AUTO.win;                                     // judged on the MEAN: on 120 Hz screens a missed frame is one 25 ms
    A.med = m; A.mean = mean;                                        // interval among 16.7 ms ones -- the median never moved at ~52 fps
    if (A.t < AUTO.warm) return false;                                // first seconds after load / a mode change: just learn
    A.ref = Math.min(A.ref, Math.max(4, m));
    // refresh reference: the lowest window median, but never slower than AUTO.minHz (a device lagging from its first frame would
    // otherwise learn its lag as "the refresh rate" and climb); capped: judge against the cap's interval
    const ref = this.q.cap ? Math.max(Math.min(A.ref, 1000 / AUTO.minHz), 1000 / this.q.cap) : Math.min(A.ref, 1000 / AUTO.minHz);
    if (mean > ref * AUTO.lag) { A.bad++; A.good = 0; } else if (mean <= ref * AUTO.smooth) { A.good++; A.bad = 0; } else { A.bad = 0; A.good = 0; }
    if (A.t - A.since < AUTO.cool) return false;
    if (A.bad >= AUTO.downAfter) {
      A.bad = 0; A.since = A.t; A.age = 0;
      if (A.i > 0) { A.ceil = A.i - 1; A.i--; this.q.dpr = A.notches[A.i]; this._memOut(); return true; }
      // already 1x: shed detail + particles now, thin scenery from the next level load (lite), once; then a 30 fps cap
      if (A.detail) { A.detail = false; this.q.detail = false; this.q.fx = QUALITY.low.fx; this.q.lite = true; this._memOut(); }
      else if (!this.q.cap) { this.q.cap = 30; this._memOut(); }
      return false;
    }
    if (A.good >= AUTO.upAfter && A.i < A.ceil) { A.good = 0; A.since = A.t; A.i++; this.q.dpr = A.notches[A.i]; this._memOut(); return true; }
    return false;
  }

  resize() {
    const c = this.canvas, dpr = Math.max(0.5, Math.min(window.devicePixelRatio || 1, this.q ? this.q.dpr : 2));
    const w = Math.max(1, Math.round(c.clientWidth * dpr)), h = Math.max(1, Math.round(c.clientHeight * dpr));
    if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
    this.w = w; this.h = h; this.dpr = dpr;
    this.ppm = this.ppmBase = Math.min(h / this.viewHeight, w / this.minViewWidth);
  }

  /** Start-screen showroom (after setLevel; any setLevel clears it): draw() swaps sky + terrain + decor for a warm-lit backdrop and a
   *  dark glass platform under the truck; frame.showroom {x, y (camera focus, fixed), ppm?} pins the camera. Render-only. */
  /** Showroom truck swap: snapshot the current frame; the next ~0.35 s of showroom frames dissolve it over the new rig. */
  showroomSwap() {
    const c = this.canvas, X = this._xf0 || (this._xf0 = document.createElement('canvas'));
    if (X.width !== c.width || X.height !== c.height) { X.width = c.width; X.height = c.height; }
    X.getContext('2d').drawImage(c, 0, 0); this._xfT = 0;
  }

  setShowroom(on = true) {
    this.showroom = !!on;
    if (on) { this.flora = []; this.puddles = []; this.rivers = []; this.spotter.setLevel(null); }
  }

  setLevel(level) {
    this.level = level;
    this.showroom = false; this._xfT = null;
    this.dirt = 0;                                                   // every run starts with a clean, glossy truck
    this._catch = null;                                              // headlight catch filter restarts
    this._gradeMap = new Map();
    this.segs = [];
    this.trail = level.trail || TRAIL;
    this.dims = null;
    this.dyZones = (level.zones || []).filter(z => z.dy);
    this.zZones = (level.zones || []).filter(z => !z.dy && zoneAtZ(z, null) === false);   // surface-only lane zones: exact slab bounds
    this.spotter.setLevel(level);
    this._laneUse = new Uint8Array(this.trail.nz);
    // own copy of the ground: base = authored run (resampled), points = 2D side-view profile (under the near tyres,
    // re-shaped from dyn.ruts + lane offsets), ya / yb = 2.5D slab-bound profiles (segs share the points arrays)
    this.ground = level.ground.map(r => ({ surface: r.surface, points: r.points.map(p => [p[0], p[1]]), d: null,
      churn: mix((level.surfaces[r.surface] && level.surfaces[r.surface].color) || '#777', '#0b0604', 0.6),
      base: r.points.map(p => [p[0], p[1]]), ya: new Float64Array(r.points.length), yb: new Float64Array(r.points.length),
      i0: 0, i1: 0, vis: false }));
    for (const g of this.ground) { const c = this._surfColor(g.surface); g.top = mix(c, '#ffffff', 0.12); g.lane = mix(c, '#000000', 0.1); g.cap = mix(c, '#000000', 0.45);
      g.riser = mix(c, '#000000', 0.22); g.riserL = mix(c, '#000000', 0.3); g.lit = mix(c, '#ffffff', 0.06);   // camera-facing cross wall: barely lighter (a bright strip read as a glitch)
      g.rise = g.base.map((q, i) => { const n = g.base[i + 1]; return !!n && Math.abs(n[1] - q[1]) >= 0.9 * (n[0] - q[0]) - 1e-6; }); }
    for (const g of this.ground) if (isWater(g.surface)) {           // ford: water tones on the tops, teal near wall
      g.top = '#4f929c'; g.lane = '#43838e'; g.riser = '#3f7b86'; g.riserL = '#38717e'; g.lit = '#73b2b8'; g.col = '#2f6a77';
    }
    for (const g of this.ground) {                                   // static per-segment slab-detail class + zone candidates
      const b = g.base, n = Math.max(0, b.length - 1);
      g.segCls = new Uint8Array(n).fill(surfCls(g.surface)); g.segZones = new Array(n).fill(null);
      for (let i = 0; i < n; i++) {
        const x = (b[i][0] + b[i + 1][0]) / 2, y = (b[i][1] + b[i + 1][1]) / 2;
        const c = (level.zones || []).filter(q => x >= q.x0 && x <= q.x1 && y >= q.y0 && y <= q.y1);
        if (c.length) g.segZones[i] = c;
      }
    }
    this.runOrder = this.ground.slice().sort((a, b) => b.points[0][0] - a.points[0][0]);   // far (+x) first
    this.polyBox = level.polygons.map(q => { let x0 = Infinity, x1 = -Infinity, y = 0;
      let y0 = Infinity, y1 = -Infinity;
      for (const p of q.points) { x0 = Math.min(x0, p[0]); x1 = Math.max(x1, p[0]); y0 = Math.min(y0, p[1]); y1 = Math.max(y1, p[1]); y += p[1]; }
      return { x0, x1, y0, y1, ym: y / q.points.length }; });
    this.rutVer = -1;
    for (const run of this.ground) {
      for (let i = 1; i < run.points.length; i++) this.segs.push([run.points[i - 1], run.points[i]]);
    }
    {                                                                // groundY index (non-vertical segments, sorted by x0)
      const L = this.segs.filter(([a, b]) => Math.abs(b[0] - a[0]) >= 1e-6).sort((p, q) => Math.min(p[0][0], p[1][0]) - Math.min(q[0][0], q[1][0])), n = L.length;
      const G = this._gIdx = { n, maxLen: 0, x0: new Float64Array(n), x1: new Float64Array(n), ax: new Float64Array(n), ay: new Float64Array(n), bx: new Float64Array(n), by: new Float64Array(n) };
      L.forEach(([a, b], i) => { G.x0[i] = Math.min(a[0], b[0]); G.x1[i] = Math.max(a[0], b[0]); G.ax[i] = a[0]; G.ay[i] = a[1]; G.bx[i] = b[0]; G.by[i] = b[1]; G.maxLen = Math.max(G.maxLen, G.x1[i] - G.x0[i]); });
    }
    this.cam.ready = false;
    // ground-height lookup table (particles collide with it cheaply)
    let x0 = Infinity, x1 = -Infinity;
    for (const [a, b] of this.segs) { x0 = Math.min(x0, a[0], b[0]); x1 = Math.max(x1, a[0], b[0]); }
    if (!isFinite(x0)) { x0 = -10; x1 = 10; }
    const n = Math.ceil((x1 - x0) / 0.2) + 1;
    this.gTab = { x0, inv: 5, y: new Float32Array(n) };
    for (let i = 0; i < n; i++) this.gTab.y[i] = this.groundY(x0 + i * 0.2);
    // per-surface particle colours + roost kind
    this.palette = ['#26272a', '#55575c', '#ff7a12', '#ffc23a', 'rgba(150,200,225,1)'];   // 0 smoke 1 smoke-lt 2,3 spark 4 drop
    this.surfFx = {};
    for (const name in level.surfaces) {
      const c = this._surfColor(name), kind = ROOST.find(r => r[1].test(name))[0];
      const base = this.palette.length;
      this.palette.push(kind === K.DUST ? mix(c, '#e4d6b8', 0.55) : kind === K.CHIP ? mix(c, '#d8dadc', 0.35) : kind === K.DROP ? mix(c, '#a8d4ea', 0.7) : mix(c, '#000000', 0.15));
      this.palette.push(kind === K.BLOB ? mix(c, '#c9b49a', 0.25) : mix(c, '#000000', 0.25));
      const sd = level.surfaces[name].dig;
      this.surfFx[name] = { kind, col: base, dig: sd ?? (/mud|bog/.test(name) ? 1 : /clay/.test(name) ? 0.6 : 0) };
    }
    this._decor(level);                                              // needs groundY (segs, gTab)
    this.clearFx();
  }

  /** Drop all particles and tyre tracks (new level / restart). */
  clearFx() {
    this.fx.n = 0; this.fx.acc.fill(0);
    this.tracks.born.fill(-1e9); this.tracks.last.length = 0;
    const F = this.feel; F.trauma = F.flash = 0; F.whip = F.wPrev = null; F.dmg = -1; F.wBroken = F.upside = false; F.axle = [false, false]; F.cool.fill(0); F.comp.fill(0);
  }

  _gy(x) {
    const g = this.gTab; if (!g) return -1e9;
    const f = (x - g.x0) * g.inv, i = Math.floor(f);
    if (i < 0) return g.y[0];
    if (i >= g.y.length - 1) return g.y[g.y.length - 1];
    return lerp(g.y[i], g.y[i + 1], f - i);
  }

  /** Attach the live rut grids (physics getDynamic().ruts: {nz, runs: [{run, d}]}, vertex-major n·nz) to the ground copy. */
  _syncRuts(R) {
    if (!R || R.ver === this.rutVer) return;
    this.rutVer = R.ver;
    for (const r of R.runs) {
      const g = this.ground[r.run];
      if (g && r.d.length === g.base.length * (R.nz || this.trail.nz)) g.d = r.d;
    }
  }

  /** 2D side view: the drawn ground line is the surface under the near (left in 2D) tyres, at trail depth z. */
  _profile2d(z, xl, xr) {
    const T = this.trail, Z = this.dyZones;
    for (const g of this.ground) {
      const b = g.base, p = g.points;
      for (let i = 0; i < b.length; i++) {
        if (b[i][0] < xl - 2 || b[i][0] > xr + 2) continue;
        p[i][1] = b[i][1] + laneOffset(T, Z, b[i][0], b[i][1], z) - (g.d ? rutAt(T, g.d, i, z) : 0);
      }
    }
  }

  setVehicle(shape) {
    this.shape = shape; this._pods = null; this._head = null; this._tail = null;                           // roof pods: re-recorded by _rig (bogger)
    const pt = shape.paint || {};
    this.colors = { ...BODY_COLORS, ...(pt.body ? { hood: pt.body, cab: pt.body } : {}), ...(pt.bed ? { bed: pt.bed } : {}), ...(pt.frame ? { frame: pt.frame } : {}),
      body: pt.body || BODY_COLORS.cab, trim: pt.trim || '#2a2c30', accent: pt.accent || '#16171a' };
    for (const k of ['hood', 'cab', 'bed', 'body']) if (this.colors[k]) this.colors[k] = paintHex(this.colors[k]);
    this.dirt = 0; this._dirtPts = null;                             // fresh paint: glossy, clean
    // render-only `look` (vehicle JSON): decor style, tube-cage panel kinds, tread / rim / coil colours, 4-link
    const lk = shape.look || {};
    this.look = { style: lk.style || '', tube: new Set(lk.tube || []), links: lk.links ?? 4, rim: lk.rim || '#b8bcc4', beadlock: !!lk.beadlock,
      tread: { lugs: 12, depth: 0.05, kind: 'lug', ...(lk.tread || {}) }, coil: { spring: '#f0c020', body: '#2f6fb3', ...(lk.coil || {}) } };
    this.box = {};
    for (const s of shape.shapes) {
      const b = this.box[s.kind] || (this.box[s.kind] = { x0: Infinity, x1: -Infinity, y0: Infinity, y1: -Infinity });
      for (const p of s.points) { b.x0 = Math.min(b.x0, p[0]); b.x1 = Math.max(b.x1, p[0]); b.y0 = Math.min(b.y0, p[1]); b.y1 = Math.max(b.y1, p[1]); }
    }
    this.tracks.last.length = 0;
    this.dims = null;
    const cen = s => s.points.reduce((a, p) => a + p[0], 0) / s.points.length;
    this.frames = shape.shapes.filter(s => s.kind === 'frame');
    this.panels = shape.shapes.filter(s => s.kind !== 'frame' && !this.look.tube.has(s.kind));   // tube kinds draw as cages (_rig)
    this.panelOrder = this.panels.slice().sort((a, b) => cen(b) - cen(a));   // 2.5D sides: front panels first
    const cab = shape.shapes.find(s => s.kind === 'cab');
    this.window = null; this.rearWin = null;
    if (cab && !this.look.tube.has('cab')) {
      let cx = 0, cy = 0;
      for (const p of cab.points) { cx += p[0]; cy += p[1]; }
      cx /= cab.points.length; cy /= cab.points.length;
      this.window = cab.points.map(p => [lerp(cx, p[0], 0.74), lerp(cy + 0.1, p[1], 0.66)]);
      // rear cab face (outward normal ≈ −x) carries the 2.5D rear window
      const q = cab.points, n = q.length, sg = this._area(q) > 0 ? 1 : -1;
      this.rearWin = null;
      for (let i = 0; i < n; i++) {
        const a = q[i], b = q[(i + 1) % n], dx = b[0] - a[0], dy = b[1] - a[1], l = Math.hypot(dx, dy) || 1;
        if (sg * dy / l < -0.6 && l > 0.2) this.rearWin = a[1] < b[1] ? [a, b] : [b, a];
      }
    }
    // exhaust stack: behind the cab, rising above the roof (chassis frame)
    const src = cab || shape.shapes[0];
    let minX = Infinity, maxY = -Infinity, baseY = Infinity;
    for (const p of src.points) { minX = Math.min(minX, p[0]); maxY = Math.max(maxY, p[1]); baseY = Math.min(baseY, p[1]); }
    this.stack = { x: minX - 0.07, y0: baseY + 0.1, y1: maxY + 0.28 };
    // styles without a stack: smoke from a hidden tail / side pipe (st.z = its body depth sign, × B)
    const fb = this.box.frame || { x0: minX, y0: baseY }, cb = this.box.cab || fb, st = this.look.style;
    if (st === 'jeep') this.stack = { x: fb.x0 + 0.05, y0: fb.y0 + 0.06, y1: fb.y0 + 0.06, hidden: true, z: 0.6 };
    else if (st === 'buggy') this.stack = { x: fb.x0 + 0.15, y0: fb.y1 - 0.05, y1: fb.y1 - 0.05, hidden: true, z: 0 };
    else if (st === 'truggy') this.stack = { x: cb.x0 - 0.3, y0: fb.y0 + 0.22, y1: fb.y0 + 0.22, hidden: true, z: 1.1 };
    else if (st === 'wedge') this.stack = { x: fb.x0 + 0.1, y0: fb.y0 + 0.08, y1: fb.y0 + 0.08, hidden: true, z: 0.5 };   // quad tips out the back
    this.rigPts = this._rigPts();
  }

  /** Drawn terrain top at (x, z): max over ground runs. z == null → the 2D side-view profile (points), else the
   *  2.5D surface (base + berms / zone dy − ruts at z). */
  _surfY(x, z) {
    const T = this.trail;
    let y = -Infinity;
    for (const g of this.ground) {
      const b = g.base, n = b.length;
      if (n < 2 || x < b[0][0] || x > b[n - 1][0]) continue;
      let lo = 0, hi = n - 1;
      while (hi - lo > 1) { const m = (lo + hi) >> 1; if (b[m][0] <= x) lo = m; else hi = m; }
      const dx = b[hi][0] - b[lo][0], t = dx > 1e-6 ? (x - b[lo][0]) / dx : 1;
      let v;
      if (z == null) v = lerp(g.points[lo][1], g.points[hi][1], t);
      else {
        const by = lerp(b[lo][1], b[hi][1], t);
        v = by + laneOffset(T, this.dyZones, x, by, z) - (g.d ? lerp(rutAt(T, g.d, lo, z), rutAt(T, g.d, hi, z), t) : 0);
      }
      if (v > y) y = v;
    }
    return y;
  }

  /** Top of the bridge deck at x (intact planks of this frame's dyn), -Infinity off the decks. */
  _deckY(x) {
    let y = -Infinity;
    for (const p of this._planks || []) {
      const c = Math.cos(p.a), u = (x - p.x) / Math.max(0.2, c);
      if (Math.abs(u) > p.hw + 0.02) continue;
      y = Math.max(y, p.y + Math.sin(p.a) * u + p.hh * c);
    }
    return y;
  }

  /** True when x lies over a bridge span (gorge under the deck). */
  _overGorge(x) {
    for (const br of (this.level && this.level.bridges) || []) if (x > br.from[0] - 0.2 && x < br.to[0] + 0.2) return true;
    return false;
  }

  /** Winch rope path (current ctx path): quadratic sag curve m → c → a over u ∈ [u0, u1], rope z = lerp(z0, z1, u)
   *  (2.5D: projected; z0 == null → 2D). Slack rope never dips below the terrain: each sample rests on _surfY + r. */
  _ropePath(mx, my, cx, cy, ax, ay, u0, u1, z0, z1) {
    const ctx = this.ctx, d3 = z0 != null, r = d3 ? 0.05 : 0.03;
    const N = Math.max(4, Math.min(64, Math.ceil(Math.hypot(ax - mx, ay - my) * (u1 - u0) / 0.2)));
    for (let i = 0; i <= N; i++) {
      const u = lerp(u0, u1, i / N), w0 = (1 - u) * (1 - u), w1 = 2 * u * (1 - u), w2 = u * u;
      const x = w0 * mx + w1 * cx + w2 * ax, z = d3 ? lerp(z0, z1, u) : 0;
      let y = w0 * my + w1 * cy + w2 * ay;
      if (u > 0 && u < 1) y = Math.max(y, this._surfY(x, d3 ? z : null) + r);   // ends stay on drum / anchor
      const sx = x + z * C30, sy = y - z * S30;
      i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy);
    }
  }

  /** Highest authored ground at x. Segments pre-sorted by x0 (`_gIdx`, built in setLevel): binary search + a short backward
   *  scan (x0 >= x - longest segment) instead of every segment each call (was ~10 MB/s of iterator garbage + O(segs)). */
  groundY(x) {
    const G = this._gIdx;
    if (!G) return 0;
    let lo = 0, hi = G.n;                                           // first index with x0 > x
    while (lo < hi) { const m = (lo + hi) >> 1; if (G.x0[m] <= x) lo = m + 1; else hi = m; }
    let y = -Infinity;
    for (let i = lo - 1; i >= 0 && G.x0[i] >= x - G.maxLen; i--) {
      if (x > G.x1[i]) continue;
      const v = G.ay[i] + (G.by[i] - G.ay[i]) * (x - G.ax[i]) / (G.bx[i] - G.ax[i]);
      if (v > y) y = v;
    }
    return y === -Infinity ? 0 : y;
  }

  // -- frame -------------------------------------------------------------------------------------
  /** frame = { vehicle, dyn, debug, dt, cam?: '2D' | '2.5D' (default 2D), anchor?: [fx, fy] truck screen spot (default 0.46, 0.5), info: {fps, steps, time} } */
  draw(frame) {
    if (this._auto && this._autoTick()) this.resize();                // AUTO resolution step: re-size before painting this frame
    {                                                                 // lights follow the engine: on while running, ~0.15 s fade
      const fv = frame.vehicle, on = fv && fv.engineOn && !(fv.falling || Math.abs(fv.roll || 0) > 0.15) ? 1 : 0, k = this._lampK ?? on;   // far-side fall: it drops behind the cliff, its lights must not shine through
      this._lampK = k + (on - k) * Math.min(1, clamp(frame.dt || 0, 0, 0.05) * 15);
    }
    this._planks = frame.dyn && frame.dyn.planks;                    // bridge decks this frame (_deckY)
    const SR = this.showroom && frame.showroom;
    this.ppm = SR && SR.ppm > 0 ? SR.ppm : this.ppmBase;
    const { ctx, w, h, ppm } = this, v = frame.vehicle, cam = this.cam;
    if (SR) return this._showroomDraw(frame, SR);
    const z3 = frame.cam === '2.5D' ? v.z || 0 : 0;                 // 2.5D: follow the truck across the trail too
    // framing: truck ~46 % down the screen (more trail under / ahead of it, less sky); slope look-ahead: climbing looks up the
    // hill, descending looks down the drop (chassis pitch, eased by the camera lag below)
    const pitch = Math.max(-0.6, Math.min(0.6, v.angle || 0));
    const tx = v.x + 1.6 + Math.max(-3, Math.min(3, v.speed * 0.35)) + z3 * C30, ty = v.y + 0.3 + 1.6 * Math.sin(pitch) - z3 * S30;
    if (!cam.ready) { cam.x = tx; cam.y = ty; cam.ready = true; this._heave = null; this._brS = null; }
    const k = 1 - Math.exp(-(frame.dt || 0.016) * 5);
    cam.x += (tx - cam.x) * k; cam.y += (ty - cam.y) * k;
    const fdt = clamp(frame.dt || 0, 0, 0.05), d3 = frame.cam === '2.5D';
    this._fdt = fdt;
    // weight: a landing / ledge drop (the chassis' fall stopped hard) throws the camera on down with the truck's momentum,
    // then a damped spring (~1.6 Hz) brings it back -- a heavy thump instead of a toy's dead stop
    const hv = this._heave || (this._heave = { y: 0, v: 0, py: v.y, vy: 0 });
    if (fdt > 0) {
      const vy = (v.y - hv.py) / fdt;
      if (Math.abs(v.y - hv.py) > 2) { hv.y = hv.v = 0; hv.vy = 0; }  // respawn / recover jump
      else {
        const dv = vy - hv.vy;
        if (dv > 1.4 && hv.vy < -0.8) hv.v -= Math.min(1.6, (dv - 0.9) * 0.4) * (this.shakeScale ?? 1);
        hv.vy += (vy - hv.vy) * Math.min(1, fdt * 30);              // light smoothing: one landing = one kick
      }
      hv.py = v.y;
      const w = TAU * 1.6, z = 0.42;
      hv.v += (-w * w * hv.y - 2 * z * w * hv.v) * fdt; hv.y = clamp(hv.y + hv.v * fdt, -0.35, 0.2);
    }
    if (this.level) { this._feel(v, fdt, d3); this._dirtTick(v, fdt); }   // impact / snap detection -> camera trauma + bursts; dirt
    const sk = this._shakeOff(fdt);
    const an = frame.anchor, ox = w * (an ? an[0] : 0.46) - cam.x * ppm + sk[0], oy = h * (an ? an[1] : 0.5) + (cam.y + hv.y) * ppm + sk[1];

    ctx.setTransform(1, 0, 0, 1, 0, 0);
    this._sky(cam);
    ctx.setTransform(ppm, 0, 0, -ppm, ox, oy);
    ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    this._ox = ox; this._oy = oy;
    const L = this.level;
    if (this.shape && !this.dims) this.dims = this._dims(v);
    if (frame.cam === '2.5D' && this.dims && L) {
      if (frame.dyn) this._syncRuts(frame.dyn.ruts);
      const dt = clamp(frame.dt || 0, 0, 0.05);
      this.t += dt;
      if (dt > 0) { this._emit(v, dt); this._stepFx(dt); }
      this._fdt = dt;
      this._draw3d(L, v, frame.dyn || { logs: [], planks: [] });
      this._z(0);
      this._atmoPost(v, true);
      this._feelDraw(v, true);
      this.spotter.draw(this, v, frame.dyn || {}, frame.spot && frame.spot.box, frame.spot ? frame.spot.mode : 'off', dt);
      if (frame.debug) { this._debugWorld(v); ctx.setTransform(1, 0, 0, 1, 0, 0); this._debugText(v, frame.info || {}); }
      return;
    }
    if (L) {
      if (frame.dyn) this._syncRuts(frame.dyn.ruts);
      let zn = 0, n = 0; for (const o of v.wheels) if (o.side === 'L') { zn += o.z || 0; n++; }
      this._profile2d(n ? zn / n : 0, cam.x - w / ppm, cam.x + w / ppm);
      const xl2 = cam.x - w / ppm, xr2 = cam.x + w / ppm;
      for (const f of this.flora) if (f.far && f.x > xl2 - 4 && f.x < xr2 + 4) this._plant(f, f.x + 0.3, this.groundY(f.x) + 0.25 + f.dz * 0.3, true);
      for (const r of this.rivers) this._river2d(r);
      this._ground(L, xl2, xr2);
      this._zones(L);
      this._polygons(L);
      this._tracks(v, cam, w / ppm);
      this._props(L, v);
    }
    const dt = clamp(frame.dt || 0, 0, 0.05);
    this.t += dt;
    if (dt > 0) { this._emit(v, dt); this._stepFx(dt); }
    if (frame.dyn) this._dynamic(frame.dyn, L);
    this._drawFx(0);                                   // far-side spray behind the truck
    this._vehicle(v);
    this._drawFx(1);                                   // near-side spray, exhaust, sparks in front
    this._atmoPost(v, false);
    this._feelDraw(v, false);
    if (L) this.spotter.draw(this, v, frame.dyn || {}, frame.spot && frame.spot.box, frame.spot ? frame.spot.mode : 'off', dt);
    if (frame.debug) {
      this._debugWorld(v);
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      this._debugText(v, frame.info || {});
    }
  }

  // -- start-screen showroom ----------------------------------------------------------------------
  /** Fixed camera on SR {x, y}, warm backdrop, glass platform at y = 0, the 2.5D truck, overhead light. No FX, feel or atmosphere. */
  _showroomDraw(frame, SR) {
    const { ctx, w, h, ppm } = this, v = frame.vehicle, cam = this.cam, an = frame.anchor || [0.5, 0.6];
    cam.x = SR.x; cam.y = SR.y; cam.ready = false;                  // next setLevel / run re-centres its own camera
    const ox = w * an[0] - cam.x * ppm, oy = h * an[1] + cam.y * ppm;
    this._ox = ox; this._oy = oy;
    const PL = SHOW_PL, PW = SHOW_PW, PT = 0.14, px = SR.x;          // one glass size for every truck (no jump on a swap)
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    this._showroomBg(oy - (PW * S30 + 0.3) * ppm, ox + px * ppm, oy, PL * ppm);
    ctx.setTransform(ppm, 0, 0, -ppm, ox, oy);
    ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    this._showroomTrack(-ox / ppm - 3, (w - ox) / ppm + 3, PT, px - PL, px + PL, PW);   // lane drawn up to the glass, glass has no end faces   // drive lane up to the glass edges
    this._platform(px, PL, PW, PT, v);
    if (this.shape && !this.dims) this.dims = this._dims(v);
    if (this.dims && this.level) { this._draw3d(this.level, v, frame.dyn || { logs: [], planks: [] }); this._z(0); }
    this._reflect(px, PL, PW, oy + ((this.dims ? this.dims.h : 1) + (v.z || 0)) * S30 * ppm);
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    this._showroomLight(ox + px * ppm, oy);
    if (this._xfT != null) {                                         // swap: old frame fades out (smoothstep) over the new one
      this._xfT += clamp(frame.dt || 0, 0, 0.05);
      const u = clamp(this._xfT / 0.35, 0, 1);
      if (u >= 1 || this._xf0.width !== w) this._xfT = null;
      else { ctx.globalAlpha = 1 - u * u * (3 - 2 * u); ctx.drawImage(this._xf0, 0, 0); ctx.globalAlpha = 1; }
    }
  }

  /** Warm studio backdrop: wall + floor gradients, dark side pillars, ceiling lamps with soft downward cones and wall washes. */
  _showroomBg(hy, cx, cy, pl) {
    const { ctx, w, h } = this, key = w + '|' + h + '|' + Math.round(hy) + '|' + Math.round(cx);
    let G = this._srG;
    if (!G || G.key !== key) {
      G = this._srG = { key };
      G.wall = ctx.createLinearGradient(0, 0, 0, hy);
      G.wall.addColorStop(0, '#16110d'); G.wall.addColorStop(0.55, '#33281f'); G.wall.addColorStop(1, '#4a3b2e');
      G.floor = ctx.createLinearGradient(0, hy, 0, h);
      G.floor.addColorStop(0, '#5a4837'); G.floor.addColorStop(0.35, '#3e3227'); G.floor.addColorStop(1, '#1a1511');
      G.pool = ctx.createRadialGradient(cx, cy, 0, cx, cy, Math.max(pl * 1.6, w * 0.32));
      G.pool.addColorStop(0, 'rgba(255,214,160,0.30)'); G.pool.addColorStop(0.5, 'rgba(255,200,140,0.10)'); G.pool.addColorStop(1, 'rgba(255,200,140,0)');
      G.side = ctx.createLinearGradient(0, 0, w, 0);
      for (const [t, a] of [[0, 0.62], [0.12, 0.18], [0.2, 0.32], [0.3, 0], [0.7, 0], [0.8, 0.32], [0.88, 0.18], [1, 0.62]]) G.side.addColorStop(t, `rgba(8,6,4,${a})`);
      G.cone = ctx.createLinearGradient(0, 0, 0, hy + (h - hy) * 0.4);
      G.cone.addColorStop(0, 'rgba(255,226,180,0.20)'); G.cone.addColorStop(1, 'rgba(255,214,160,0)');
      G.wash = ctx.createRadialGradient(0, 0, 0, 0, 0, 1);
      G.wash.addColorStop(0, 'rgba(255,214,160,0.26)'); G.wash.addColorStop(1, 'rgba(255,214,160,0)');
      G.lamp = ctx.createRadialGradient(0, 0, 0, 0, 0, 1);
      G.lamp.addColorStop(0, 'rgba(255,246,226,1)'); G.lamp.addColorStop(0.25, 'rgba(255,226,180,0.55)'); G.lamp.addColorStop(1, 'rgba(255,214,160,0)');
    }
    ctx.fillStyle = G.wall; ctx.fillRect(0, 0, w, hy + 1);
    ctx.fillStyle = G.floor; ctx.fillRect(0, hy, w, h - hy);
    ctx.fillStyle = 'rgba(255,230,190,0.10)'; ctx.fillRect(0, hy - 1, w, Math.max(1, h * 0.004));   // wall / floor seam catches light
    for (const f of [0.08, 0.3, 0.7, 0.92]) {                         // wall washes (scalloped light below each lamp)
      ctx.setTransform(h * 0.2, 0, 0, h * 0.42, f * w, hy * 0.08); ctx.fillStyle = G.wash; ctx.beginPath(); ctx.arc(0, 0, 1, 0, TAU); ctx.fill();
    }
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = G.cone;                                          // soft downward cones (haze), the centre key light widest
    for (const [f, a, b] of [[0.2, 0.02, 0.1], [0.8, 0.02, 0.1], [cx / w, 0.03, 0.2]]) {
      const x = f * w; ctx.beginPath(); ctx.moveTo(x - a * w, 0); ctx.lineTo(x + a * w, 0); ctx.lineTo(x + b * w, h); ctx.lineTo(x - b * w, h); ctx.closePath(); ctx.fill();
    }
    ctx.fillStyle = G.pool; ctx.fillRect(0, hy, w, h - hy);
    ctx.fillStyle = G.side; ctx.fillRect(0, 0, w, h);
    for (const f of [0.2, cx / w, 0.8]) {                             // ceiling lamps
      ctx.setTransform(h * 0.07, 0, 0, h * 0.03, f * w, h * 0.004); ctx.fillStyle = G.lamp; ctx.beginPath(); ctx.arc(0, 0, 1, 0, TAU); ctx.fill();
    }
    ctx.setTransform(1, 0, 0, 1, 0, 0);
  }

  /** Dark glass slab, top at y = 0: x px ± PL, z ± PW, thickness PT (world, pre-transform). Lit edges, sheen, contact shadow. */
  _platform(px, PL, PW, PT, v) {
    const ctx = this.ctx, x0 = px - PL, x1 = px + PL;
    const P = (x, y, z) => [x + z * C30, y - z * S30];
    const fl = P(x0, 0, -PW), fr = P(x1, 0, -PW), nr = P(x1, 0, PW), nl = P(x0, 0, PW);
    const poly = (pts, fill) => { ctx.beginPath(); ctx.moveTo(pts[0][0], pts[0][1]); for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i][0], pts[i][1]); ctx.closePath(); if (fill) { ctx.fillStyle = fill; ctx.fill(); } };
    // floor shadow under the slab
    ctx.save(); ctx.transform(PL * 1.25, 0, PW * 1.3 * C30, -PW * 1.3 * S30, px, -PT - 0.02);
    let g = ctx.createRadialGradient(0, 0, 0, 0, 0, 1); g.addColorStop(0, 'rgba(0,0,0,0.5)'); g.addColorStop(1, 'rgba(0,0,0,0)');
    ctx.fillStyle = g; ctx.beginPath(); ctx.arc(0, 0, 1, 0, TAU); ctx.fill(); ctx.restore();
    // left + near faces (thick glass: green-tinted, bright top lip)
    const d = [0, -PT];
    const add = (p, q) => [p[0] + q[0], p[1] + q[1]];
    // thick float glass seen edge-on: bright green-teal edge faces (light travels through the slab)
    g = ctx.createLinearGradient(0, 0, 0, -PT); g.addColorStop(0, 'rgba(190,240,225,0.85)'); g.addColorStop(0.45, 'rgba(90,160,145,0.7)'); g.addColorStop(1, 'rgba(40,90,82,0.8)');
    // (no end faces: the stone lane meets the glass ends flush, one solid surface)
    poly([nl, nr, add(nr, d), add(nl, d)], g);
    // top: lightly smoked glass (grey-black tint, floor still shows through), cooler far edge catching the room light
    poly([fl, fr, nr, nl], 'rgba(16,18,20,0.30)');
    g = ctx.createLinearGradient(0, fl[1], 0, nl[1]); g.addColorStop(0, 'rgba(200,225,220,0.24)'); g.addColorStop(0.5, 'rgba(120,150,148,0.10)'); g.addColorStop(1, 'rgba(40,50,52,0.26)');
    poly([fl, fr, nr, nl], g);
    ctx.save(); poly([fl, fr, nr, nl]); ctx.clip();
    ctx.save(); ctx.transform(PL * 0.9, 0, PW * C30, -PW * S30, px, 0);
    g = ctx.createRadialGradient(0, 0, 0, 0, 0, 1); g.addColorStop(0, 'rgba(255,214,160,0.16)'); g.addColorStop(1, 'rgba(255,214,160,0)');
    ctx.fillStyle = g; ctx.beginPath(); ctx.arc(0, 0, 1, 0, TAU); ctx.fill(); ctx.restore();
    ctx.fillStyle = 'rgba(255,250,240,0.12)';                        // diagonal sheen bands (polished surface)
    for (const [a, b] of [[-0.55, -0.35], [-0.25, -0.18], [0.35, 0.6]]) {
      const ua = px + a * PL * 2, ub = px + b * PL * 2;
      poly([P(ua - 1.2, 0, -PW), P(ub - 1.2, 0, -PW), P(ub + 1.2, 0, PW), P(ua + 1.2, 0, PW)]);
      ctx.fill();
    }
    // contact shadow under the truck (body + tyres), tightens as it settles
    let gap = Infinity; for (const o of v.wheels) gap = Math.min(gap, o.y - (o.radius || 0.4));
    const lift = clamp(gap / 0.8, 0, 1);
    ctx.save(); ctx.transform(PL * 0.62, 0, 1.25 * C30, -1.25 * S30, v.x, 0);
    g = ctx.createRadialGradient(0, 0, 0, 0, 0, 1); g.addColorStop(0, `rgba(0,0,0,${(0.62 - 0.35 * lift).toFixed(3)})`); g.addColorStop(0.7, 'rgba(0,0,0,0.25)'); g.addColorStop(1, 'rgba(0,0,0,0)');
    ctx.fillStyle = g; ctx.beginPath(); ctx.arc(0, 0, 1, 0, TAU); ctx.fill(); ctx.restore();
    ctx.restore();
    // edges: bright near lip, softer far + side edges
    ctx.lineWidth = 0.04; ctx.strokeStyle = 'rgba(235,255,248,0.9)';   // polished near lip catches the light
    ctx.beginPath(); ctx.moveTo(nl[0], nl[1]); ctx.lineTo(nr[0], nr[1]); ctx.stroke();
    ctx.lineWidth = 0.015; ctx.strokeStyle = 'rgba(220,240,232,0.5)';
    ctx.beginPath(); ctx.moveTo(fl[0], fl[1]); ctx.lineTo(fr[0], fr[1]); ctx.stroke();   // far edge only (ends meet the lane)
    ctx.strokeStyle = 'rgba(180,220,210,0.25)';
    ctx.beginPath(); ctx.moveTo(nl[0], nl[1] - PT); ctx.lineTo(nr[0], nr[1] - PT); ctx.stroke();
  }

  /** Showroom drive lane: dark slate pavers at y = 0 (the physics floor), wide enough for every truck, warm grout joints,
   *  a slim brushed-steel kerb on each side and a shallow near face. Flat fills (no gradients), static seeded tones. */
  _showroomTrack(x0, x1, PT, g0, g1, TW) {
    // two runs, [x0, g0] and [g1, x1]: same width + thickness as the glass, so lane and stand meet edge to edge
    const ctx = this.ctx, P = (x, y, z) => [x + z * C30, y - z * S30];
    const quad = (a, b, c, d, fill) => { ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.lineTo(c[0], c[1]); ctx.lineTo(d[0], d[1]); ctx.closePath(); ctx.fillStyle = fill; ctx.fill(); };
    const L = 0.95;
    for (const [ra, rb] of [[x0, g0], [g1, x1]]) {
      if (rb <= ra) continue;
      quad(P(ra, 0, TW), P(rb, 0, TW), P(rb, -PT, TW), P(ra, -PT, TW), '#1c1814');               // near face
      quad(P(ra, 0, -TW), P(rb, 0, -TW), P(rb, 0, TW), P(ra, 0, TW), '#221e1a');                 // grout bed
      for (let k = Math.floor(ra / L) - 1; k * L < rb; k++) for (const [za, zb, off] of [[-TW, 0, 0], [0, TW, 0.5]]) {   // pavers, staggered
        const xa = Math.max(ra, (k + off) * L + 0.02), xb = Math.min(rb, (k + off + 1) * L - 0.02), t = h01(k, za < 0 ? 3 : 7);
        if (xb <= xa) continue;
        quad(P(xa, 0, za + 0.03), P(xb, 0, za + 0.03), P(xb, 0, zb - 0.03), P(xa, 0, zb - 0.03), t < 0.33 ? '#3a342d' : t < 0.66 ? '#433b33' : '#362f29');
      }
      ctx.lineWidth = 0.04; ctx.strokeStyle = '#8d8a84';                                         // steel kerbs on the edges
      for (const z of [-TW, TW]) { const a = P(ra, 0, z), b = P(rb, 0, z); ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); }
    }
  }

  /** Glass top reflection: the frame above the near tyres' contact line Y0 (screen px) mirrored into the slab top, faded out. */
  _reflect(px, PL, PW, Y0) {
    const { ctx, w, h, ppm } = this, H = Math.min(Y0, h - Y0, 3.2 * ppm);
    if (!(H > 4)) return;
    const P = (x, z) => [x + z * C30, -z * S30];
    const q = [P(px - PL, -PW), P(px + PL, -PW), P(px + PL, PW), P(px - PL, PW)];
    ctx.save();
    ctx.beginPath(); for (let i = 0; i < 4; i++) ctx[i ? 'lineTo' : 'moveTo'](q[i][0], q[i][1]); ctx.closePath(); ctx.clip();   // world transform
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.beginPath(); ctx.rect(0, Y0, w, H); ctx.clip();
    ctx.globalAlpha = 0.3; ctx.setTransform(1, 0, 0, -1, 0, 2 * Y0);
    ctx.drawImage(this.canvas, 0, Y0 - H, w, H, 0, Y0 - H, w, H);    // self-blit (copied first per spec)
    ctx.globalAlpha = 1; ctx.setTransform(1, 0, 0, 1, 0, 0);
    const g = ctx.createLinearGradient(0, Y0, 0, Y0 + H * 0.85);
    g.addColorStop(0, 'rgba(14,15,17,0)'); g.addColorStop(1, 'rgba(14,15,17,0.6)');
    ctx.fillStyle = g; ctx.fillRect(0, Y0, w, H);
    ctx.restore();
  }

  /** Overhead showroom light over the finished frame: warm top-down soft-light grade + edge vignette (screen space). */
  _showroomLight(cx, cy) {
    const { ctx, w, h } = this, key = w + '|' + h + '|' + Math.round(cx) + '|' + Math.round(cy);
    let G = this._srL;
    if (!G || G.key !== key) {
      G = this._srL = { key };
      G.top = ctx.createLinearGradient(0, 0, 0, h);
      G.top.addColorStop(0, 'rgba(255,226,180,0.55)'); G.top.addColorStop(0.45, 'rgba(255,214,160,0.18)'); G.top.addColorStop(1, 'rgba(0,0,0,0.35)');
      G.vig = ctx.createRadialGradient(cx, cy - h * 0.1, h * 0.25, cx, cy - h * 0.1, Math.hypot(w, h) * 0.65);
      G.vig.addColorStop(0, 'rgba(0,0,0,0)'); G.vig.addColorStop(1, 'rgba(0,0,0,0.55)');
    }
    ctx.globalCompositeOperation = 'soft-light'; ctx.fillStyle = G.top; ctx.fillRect(0, 0, w, h);
    ctx.globalCompositeOperation = 'source-over'; ctx.fillStyle = G.vig; ctx.fillRect(0, 0, w, h);
  }

  /** Lighting / atmosphere preset by name (ATMOS key); anything else picks one at random (per playthrough). */
  setAtmosphere(name) {
    const keys = Object.keys(ATMOS);
    this.atmoName = keys.includes(name) ? name : keys[Math.floor(Math.random() * keys.length)];
    this.atmo = ATMOS[this.atmoName];
    this._starSeed = Math.random() * 1000;
    return this.atmoName;
  }

  /** Per-level scenery (seeded by level name, stable across restarts): trailside flora, mud puddles, gorge rivers. */
  _decor(level) {
    const R = rngOf(hashStr(level.name || 'level')), T = this.trail, fl = this.flora = [];
    let x0 = Infinity, x1 = -Infinity;
    for (const g of this.ground) { x0 = Math.min(x0, g.base[0][0]); x1 = Math.max(x1, g.base.at(-1)[0]); }
    const pick = a => a[Math.floor(R() * a.length)];
    const S = { pine: 0.3, bush: 0.35, fern: 0.17, gap0: 0.9, gapR: 2.2, bright: 0.32, pal: 'forest', ...(level.scenery || {}) };
    const [DK, BR] = PALETTES[S.pal] || PALETTES.forest, kP = S.pine, kB = kP + S.bush, kF = kB + S.fern;
    for (let x = x0 + 1; x < x1; x += S.gap0 + R() * S.gapR) {      // far edge: pines, bushes, ferns, grass (level mix)
      const k = R(), parrot = R() < S.bright;
      fl.push({ x, far: true, kind: k < kP ? 'pine' : k < kB ? 'bush' : k < kF ? 'fern' : 'grass', s: 0.7 + R() * 0.6,
        col: parrot && k >= kP ? pick(BR) : pick(DK), acc: pick(BR), seed: R() * 100, dz: R() * 0.5 });
      if (k < kP && R() < 0.5) fl.push({ x: x + 0.4, far: true, kind: R() < 0.5 ? 'fern' : 'grass', s: 0.6 + R() * 0.4, col: pick(BR), acc: pick(DK), seed: R() * 100, dz: 0.55 });
    }
    for (let x = x0 + 2; x < x1; x += 1.6 + R() * 3.5 * (S.gapR / 2.2)) {   // near edge: low grass / ferns only (never hides the truck)
      const parrot = R() < 0.55;
      fl.push({ x, far: false, kind: R() < (S.fern > 0.1 ? 0.6 : 0.85) ? 'grass' : 'fern', s: 0.45 + R() * 0.3, col: parrot ? pick(BR) : pick(DK), acc: pick(BR), seed: R() * 100, dz: 0 });
    }
    if (this.q.lite) {                                               // lite: every other far plant (no companions), every third tuft
      let nf = 0, nn = 0;
      this.flora = fl.filter(f => f.far ? f.dz === 0.55 ? false : nf++ % 2 === 0 : nn++ % 3 === 0);
    }
    for (const f of this.flora) { f.lit = mix(f.col, '#ffffff', f.kind === 'pine' ? 0.12 : 0.22); f.base = mix(f.col, '#000000', 0.25); }   // shades once, not per frame
    this.puddles = [];                                               // glossy puddles on soft runs (2.5D tops)
    for (const g of this.ground) if (isWet(g.surface)) {
      const b = g.base;
      for (let x = b[0][0] + 1; x < b.at(-1)[0] - 0.8; x += 1.8 + R() * 2.6)
        this.puddles.push({ x, z: (R() * 2 - 1) * T.edge * 0.85, rx: 0.35 + R() * 0.55, rz: 0.12 + R() * 0.16, ph: R() * TAU });
    }
    this.rivers = [];                                                // gorges under the bridges: a river at the floor
    for (const br of level.bridges || []) {
      let lo = Infinity, xa = br.from[0], xb = br.to[0];
      for (let x = xa; x <= xb; x += 0.1) lo = Math.min(lo, this.groundY(x));
      const deck = Math.min(br.from[1], br.to[1]);
      if (!(deck - lo > 0.8)) continue;
      const wl = lo + Math.min(0.5, (deck - lo) * 0.35);
      let a = xa, b = xb;
      while (a < xb && this.groundY(a) > wl) a += 0.05;
      while (b > a && this.groundY(b) > wl) b -= 0.05;
      if (b - a > 0.3) this.rivers.push({ x0: a - 0.05, x1: b + 0.05, y: wl, lo });
    }
  }

  _sky(cam) {
    const { ctx, w, h } = this, A = this.atmo;
    // gradients depend only on atmosphere + canvas size: built once, reused every frame (haze bands shift by translate)
    const key = this.atmoName + '|' + w + '|' + h;
    let G = this._skyG;
    if (!G || G.key !== key) {
      G = this._skyG = { key, haze: [] };
      G.sky = ctx.createLinearGradient(0, 0, 0, h);
      G.sky.addColorStop(0, A.sky[0]); G.sky.addColorStop(0.62, A.sky[1]); G.sky.addColorStop(1, A.sky[2]);
      if (A.sun) { const [sx, sy, sr, sc] = A.sun, X = sx * w, Y = sy * h, R = sr * h;
        G.halo = ctx.createRadialGradient(X, Y, R * 0.5, X, Y, R * 5); G.halo.addColorStop(0, sc); G.halo.addColorStop(1, 'rgba(255,255,255,0)'); }
      for (const base of SKY_BASES) {
        const hz = ctx.createLinearGradient(0, h * base, 0, h * (base + 0.12));
        hz.addColorStop(0, 'rgba(255,255,255,0)'); hz.addColorStop(1, this.atmoName === 'night' ? 'rgba(40,50,80,0.18)' : 'rgba(235,238,230,0.16)');
        G.haze.push(hz);
      }
    }
    ctx.fillStyle = G.sky; ctx.fillRect(0, 0, w, h);
    if (A.stars) {
      ctx.fillStyle = 'rgba(235,240,255,0.8)';
      for (let i = 0; i < 90; i++) { const sx = h01(i, this._starSeed) * w, sy = h01(i + 500, this._starSeed) * h * 0.5, r = h01(i + 900) < 0.15 ? 1.6 : 0.9; ctx.fillRect(sx, sy, r * this.dpr, r * this.dpr); }
    }
    if (A.sun) {
      const [sx, sy, sr, sc] = A.sun, X = sx * w, Y = sy * h, R = sr * h;
      ctx.globalAlpha = 0.35; ctx.fillStyle = G.halo; ctx.fillRect(X - R * 5, Y - R * 5, R * 10, R * 10); ctx.globalAlpha = 1;
      ctx.fillStyle = sc; ctx.beginPath(); ctx.arc(X, Y, R, 0, TAU); ctx.fill();
    }
    // layered ridges far -> near (parallax), forest fringe on the two nearest, haze band at each foot
    const layers = SKY_LAYERS;
    for (const [par, base, amp, li] of layers) {
      const off = cam.x * this.ppm * par, lift = -cam.y * this.ppm * par * 0.3 + h * 0.02, col = A.ridges[li];
      const yAt = x => { const u = (x + off) / h;
        return h * (base - amp * (Math.sin(u * (2.1 + li * 0.7) + li) * 0.55 + Math.sin(u * (5.3 + li) + 1.7) * 0.3 + Math.sin(u * 13.1 + li * 2) * 0.15)) + lift; };
      ctx.fillStyle = col; ctx.beginPath(); ctx.moveTo(0, h);
      const step = w / (48 + li * 24);
      for (let x = 0; x <= w + step; x += step) ctx.lineTo(x, yAt(x));
      ctx.lineTo(w, h); ctx.closePath(); ctx.fill();
      if (li >= 2 + (this.q.lite ? 1 : 0)) {                         // pine silhouettes along the ridge line (lite: nearest ridge only)
        const sp = (li === 2 ? 0.012 : 0.02) * h, th = (li === 2 ? 0.025 : 0.045) * h;
        ctx.beginPath();                                             // one path for the whole fringe (was a fill per pine)
        for (let k = Math.floor(-off / sp) - 1, n = 0; n < w / sp + 3; k++, n++) {
          const x = k * sp + off - Math.floor(off / sp) * sp - off + ((k * sp) % sp), xs = k * sp + off - off;   // world-anchored spacing
          const X = ((k * sp - (off % sp)) + sp * 4) - sp * 4, r = h01(k, li);
          if (r < 0.25) continue;
          const y = yAt(X), hh = th * (0.6 + r * 0.7);
          ctx.moveTo(X - sp * 0.42, y + 2); ctx.lineTo(X, y - hh); ctx.lineTo(X + sp * 0.42, y + 2); ctx.closePath();
        }
        ctx.fill();
      }
      ctx.translate(0, lift); ctx.fillStyle = G.haze[li]; ctx.fillRect(0, h * base, w, h * 0.12); ctx.translate(0, -lift);
    }
  }

  /** Lighting pass over the finished world (screen space tint) + night headlights from the truck nose. */
  _atmoPost(v, d3) {
    const ctx = this.ctx, A = this.atmo;
    if (!A.tint) return;                                             // bright day: no tint, no headlights
    ctx.save(); ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = A.tint; ctx.fillRect(0, 0, this.w, this.h);
    ctx.restore();
    if (!this.shape) return;
    // headlight strength: atmosphere x engine (fades with _lampK); LOW detail keeps only the full night beams (fill budget)
    const I = (!this.q.detail && (A.lamp ?? 0) < 1 ? 0 : A.lamp ?? 0) * (this._lampK || 0);
    const S = this._lightSpr || (this._lightSpr = { fan: fanSprite(), near: throwSprite(0.5), glow: glowSprite(255, 236, 200, 2.4),
      spill: glowSprite(150, 170, 210, 1.3), pool: glowSprite(255, 200, 140, 1.6), red: glowSprite(255, 44, 30, 2.2),
      rev: glowSprite(255, 236, 214, 1.8) });   // built once
    const pods = this._pods, dark = (A.lamp ?? 0) >= 0.4 ? I : 0;   // roof pods: dusk / night only
    if (!A.lights && I < 0.01) return;
    ctx.save(); ctx.globalCompositeOperation = 'lighter';
    if (A.lights && S.spill) {                                       // ambient spill so the rig stays readable at night (cached sprite)
      if (d3) this._tz(0); else this._z(0);
      ctx.globalAlpha = 0.14; ctx.drawImage(S.spill, v.x - 3.2, v.y - 3.2, 6.4, 6.4);
    }
    const T = I >= 0.01 && S.fan, G = S.glow;
    if (T && G) {
      // ONE continuous side-profile throw: a fan from the near headlamp lens, its top edge just under the horizon (lights rock
      // faces ahead), its bottom edge onto the trail past the front tyre; soft dark start off the lens, every edge feathered to
      // 0, smooth fade with distance. + that lens' bloom. Front tyres clipped out (no haze over them). Additive, cached sprites.
      const wm = this.shape.winchMount, c = Math.cos(v.angle), s = Math.sin(v.angle), yaw = d3 ? v.yaw || 0 : 0, tf = this._T;
      const hd = this._head, D = d3 ? tf : null;
      // lens in truck frame (pre-_tz) -> _z(0) world-screen coords (no edge-roll term: the lights are off the trail then)
      const hlx = hd ? hd[0] : wm[0], hly = hd ? hd[1] : wm[1] + 0.35, hlz = hd ? hd[2] : (this.dims ? this.dims.B * 0.7 : 0);
      const lens = z => {                                            // lens at truck-frame depth z -> [X, Y] in _z(0) coords
        const lx = v.x + c * hlx - s * hly + (D ? z * D.Dx : 0), ly = v.y + s * hlx + c * hly + (D ? z * D.Dy : 0);
        return d3 ? [tf.a * lx + tf.xc * (1 - tf.a) + tf.zc * C30, tf.b * lx + ly - tf.b * tf.xc - tf.zc * S30] : [lx, ly];
      };
      ctx.save();
      { // clip: everything except the front tyres (both, one hull: the body between them takes no light either)
        if (d3) this._tz(0); else this._z(0);
        ctx.beginPath(); ctx.rect(v.x - 60, v.y - 60, 120, 120);
        const fr = (v.wheels || []).filter(o => o.axle === 0);
        if (d3 && this.dims && fr.length) {
          const F = []; for (const o of fr) F.push(...this._tyreOutline(o, this.dims.tw).slice(0, 64));
          hullPath(ctx, F);
        } else for (const o of fr) { const r = (o.radius || 0.5) + 0.06; ctx.moveTo((o.hx ?? o.x) + r, o.hy ?? o.y); ctx.arc(o.hx ?? o.x, o.hy ?? o.y, r, 0, TAU); }
        ctx.clip('evenodd');
      }
      this._z(0);
      let fw = -Infinity; for (const o of v.wheels || []) fw = Math.max(fw, (o.hx ?? o.x) + (o.radius || 0.5));   // front tyres' leading face
      const zc = d3 ? v.z || 0 : 0, cy = Math.cos(yaw), sy = Math.sin(yaw), Lf = 10, hw = Lf * Math.tan(0.24);
      const gd = Math.max(3.2, fw + 4.2 - (v.x + c * hlx));
      let fx = 0, fy = 0, fn = 0; for (const o of v.wheels || []) if (o.axle === 0) { fx += o.hx ?? o.x; fy += (o.hy ?? o.y) - (o.radius || 0.5); fn++; }
      if (fn) { fx /= fn; fy /= fn; } else { fx = v.x; fy = v.y - 0.9; }
      // TWO lamps (2.5D): far one first (dimmer: mostly behind the body), then the near one. Each fan's axis lands ~4 m past
      // the tyre face on its own side of the trail centre (±0.55 m across the heading), so the pair spreads across the trail
      // and overlaps softly in the middle; feathered edges bleed a little past both trail sides. 2D: the near lamp only.
      for (const [lz, side, a, bl] of d3 ? [[-hlz, -1, 0.2, 0.4], [hlz, 1, 0.24, 1]] : [[hlz, 0, 0.34, 1]]) {
        const [LX, LY] = lens(lz), o = 0.55 * side;
        const gx = v.x + c * hlx + gd * cy - o * sy, gz = zc + gd * sy + o * cy;
        // target on the truck's OWN ground plane (front tyre contact line, extended along the chassis pitch), not the terrain
        // ahead: real lamps are bolted to the body, so the throw only moves with the body (terrain targets swung the beams down
        // on crests / landings)
        const GX = gx + (d3 ? gz * C30 : 0), GY = fy + (gx - fx) * Math.tan(v.angle) - (d3 ? gz * S30 : 0);
        const aim = Math.min(v.angle - 0.14, Math.max(v.angle - 0.6, Math.atan2(GY - LY, GX - LX)));
        ctx.save(); ctx.translate(LX, LY); ctx.rotate(aim); ctx.globalAlpha = a * I; ctx.drawImage(T.img, 0, -hw, Lf, 2 * hw); ctx.restore();
        ctx.globalAlpha = 0.35 * bl * I; ctx.drawImage(G, LX - 0.32, LY - 0.32, 0.64, 0.64);   // lens bloom (far lamp: faint, peeks over the bonnet)
        ctx.globalAlpha = 0.8 * bl * I; ctx.drawImage(G, LX - 0.09, LY - 0.09, 0.18, 0.18);
      }
      if (S.near) this._nearPool(v, d3, fw, fx, fy, I, S.near);
      ctx.restore();
    }
    if (pods && dark >= 0.01 && G && S.pool) {
      // roof light pods (only rigs that have them): lit lenses + a subtle wide downward spill on the ground around the nose
      if (d3) this._tz(0); else this._z(0);
      const c = Math.cos(v.angle), s = Math.sin(v.angle), D = d3 ? this._T : null;
      for (const lz of pods.z) {
        if (!d3 && lz !== pods.z[pods.z.length - 1]) continue;
        const X = v.x + c * pods.x - s * pods.y + (D ? lz * D.Dx : 0), Y = v.y + s * pods.x + c * pods.y + (D ? lz * D.Dy : 0);
        ctx.globalAlpha = 0.45 * dark; ctx.drawImage(G, X - 0.18, Y - 0.18, 0.36, 0.36);
      }
      const yaw = d3 ? v.yaw || 0 : 0, gx = v.x + c * pods.x + 1.6 * Math.cos(yaw), gz = d3 ? (v.z || 0) + 1.6 * Math.sin(yaw) : 0;
      this._z(0);
      const g = d3 ? this._surfY(gx, gz) : this.groundY(gx);
      if (Number.isFinite(g)) {
        ctx.save(); ctx.globalAlpha = 0.16 * dark;
        if (d3) ctx.transform(3.6, 0, 2.6 * C30, -2.6 * S30, gx + gz * C30, g - gz * S30); else ctx.transform(3.6, 0, 0, 0.5, gx, g - 0.2);
        ctx.drawImage(S.pool, -1, -1, 2, 2); ctx.restore();
      }
    }
    if (dark >= 0.01 && S.red && S.rev) this._tailLights(v, d3, dark, S);
    ctx.restore(); this._z(0);
  }

  /** One slab's rising step faces (recorded by _terrainSlab), drawn after the zone paint + detail: a contact shadow along the
   *  foot, a light crest line, then the night headlight catch. Rock faces (r15: the ledge ribs read as uniform bevelled
   *  beams) become fractured natural ledges: the contact shadow's width wanders, the crest is chipped (jagged, with dark
   *  notches) and tapered fracture cracks run down the face at irregular spacing. All of it is world-anchored (fixed
   *  FRACT.step depth grid + per-face hash of its world x), so slab bounds moving with the truck never reshuffle it, and it
   *  rides the same two paths as before (one dark fill, one light stroke per slab). Render-only: terrain heights untouched. */
  _faceFx(k0, k1, za, zb) {
    const ctx = this.ctx, F = this._faces, J = FRACT.step, j0 = Math.ceil(za / J - 1e-9), j1 = Math.floor(zb / J + 1e-9);
    // jitter fields: grid values at z = j·J, linear between (continuous across slab bounds)
    const fld = (key, salt, z) => { const t = z / J, j = Math.floor(t), u = t - j; return lerp(h01(key + j * 7.31, salt), h01(key + (j + 1) * 7.31, salt), u); };
    const pt = (r, side, z, o) => { const u = (z - za) / (zb - za || 1); o[0] = (side ? r[1] : r[0]) + z * C30; o[1] = lerp(side ? r[3] : r[2], side ? r[5] : r[4], u) - z * S30; return o; };
    const A = this._pa, B = this._pb;
    ctx.beginPath();                                                 // dark: contact shadow bands + crest notches + cracks
    for (let k = k0; k < k1; k++) {
      const r = F[k], rk = r[8] > 0, key = Math.round(r[0] * 23) * 0.913, h = Math.max(0.05, ((r[3] - r[2]) + (r[5] - r[4])) / 2);
      const wAt = z => rk ? FRACT.foot0 + FRACT.foot1 * fld(key, 3, z) : FACE.foot * 0.6;
      // contact band: on the ground just before the foot (x0 − w) up to a hair onto the face
      pt(r, 0, za, A); ctx.moveTo(A[0] - wAt(za), A[1]);
      for (let j = j0; j <= j1; j++) { const z = j * J; if (z <= za || z >= zb) continue; pt(r, 0, z, A); ctx.lineTo(A[0] - wAt(z), A[1]); }
      pt(r, 0, zb, A); ctx.lineTo(A[0] - wAt(zb), A[1]); ctx.lineTo(A[0] + 0.02, A[1] + 0.03);
      pt(r, 0, za, A); ctx.lineTo(A[0] + 0.02, A[1] + 0.03); ctx.closePath();
      if (!rk) continue;
      for (let j = j0 - 1; j <= j1; j++) {                           // per grid cell: maybe a crest notch, maybe a crack
        const c0 = j * J, n1 = h01(key + j * 3.17, 11), n2 = h01(key + j * 5.03, 12), n3 = h01(key + j * 2.21, 13);
        const zc = c0 + J * (0.2 + 0.6 * n2);
        if (zc < za || zc >= zb) continue;                           // drawn by the slab holding its depth
        if (n1 < FRACT.notch) {                                      // chipped crest: a small dark bite under the lip
          const hw = J * (0.18 + 0.2 * n3), dd = Math.min(h * 0.45, 0.04 + 0.08 * n3);
          pt(r, 1, Math.max(za, zc - hw), A); ctx.moveTo(A[0], A[1] - 0.005);
          pt(r, 1, Math.min(zb, zc + hw), A); ctx.lineTo(A[0], A[1] - 0.005);
          const u = dd / h; pt(r, 1, zc + hw * 0.2, A); pt(r, 0, zc + hw * 0.2, B); ctx.lineTo(lerp(A[0], B[0], u), lerp(A[1], B[1], u)); ctx.closePath();
        }
        if (n2 < FRACT.crack && h > 0.15) {                          // fracture: tapered, kinked, down from the crest
          const len = 0.55 + 0.4 * n1, kz = (n3 - 0.5) * 0.12, tw = FRACT.crackW * (0.7 + 0.6 * n1);
          pt(r, 1, zc - tw, A); ctx.moveTo(A[0], A[1]);
          pt(r, 1, zc + tw, A); ctx.lineTo(A[0], A[1]);
          const um = len * 0.5, zm = zc + kz;
          pt(r, 1, zm + tw * 0.5, A); pt(r, 0, zm + tw * 0.5, B); ctx.lineTo(lerp(A[0], B[0], um), lerp(A[1], B[1], um));
          pt(r, 1, zm - kz * 0.5, A); pt(r, 0, zm - kz * 0.5, B); ctx.lineTo(lerp(A[0], B[0], len), lerp(A[1], B[1], len));
          pt(r, 1, zm - tw * 0.5, A); pt(r, 0, zm - tw * 0.5, B); ctx.lineTo(lerp(A[0], B[0], um), lerp(A[1], B[1], um)); ctx.closePath();
        }
      }
    }
    ctx.fillStyle = 'rgba(0,0,0,0.3)'; ctx.fill();
    ctx.beginPath();                                                 // light crest: jagged on rock (dips by a wandering depth)
    for (let k = k0; k < k1; k++) {
      const r = F[k], rk = r[8] > 0, key = Math.round(r[0] * 23) * 0.913, h = Math.max(0.05, ((r[3] - r[2]) + (r[5] - r[4])) / 2);
      const cr = z => { pt(r, 1, z, A); if (!rk) return A; const u = Math.min(0.4, FRACT.jag * fld(key, 5, z) / h); pt(r, 0, z, B); A[0] = lerp(A[0], B[0], u); A[1] = lerp(A[1], B[1], u); return A; };
      cr(za); ctx.moveTo(A[0], A[1]);
      if (rk) for (let j = j0; j <= j1; j++) { const z = j * J; if (z > za && z < zb) { cr(z); ctx.lineTo(A[0], A[1]); } }
      cr(zb); ctx.lineTo(A[0], A[1]);
    }
    ctx.lineWidth = FACE.crest; ctx.strokeStyle = 'rgba(255,255,255,0.3)'; ctx.stroke();
    if (this._catch && this._catch.on) this._faceCatch(F, k0, k1, za, zb);
  }

  /** Headlight catch on rock step faces (r13: obstacles ahead were unreadable at night: the night tint is a flat overlay, so
   *  a face's darker shade all but vanished, and the additive pools lit faces and tops alike). Real low beams hit a face that
   *  stands across their path head-on: it is the brightest thing ahead. Drawn in depth order by the slab that owns the face
   *  pieces (nearer terrain / the truck still cover it); warm additive fill, ≤ 5 strength buckets; the night tint comes after,
   *  hence the strong alpha. */
  _faceCatch(F, k0, k1, za, zb) {
    // r15: strength is evaluated per WORLD-FIXED depth cell (CATCH.cell) of each face, not per slab strip: slab bounds move
    // with the truck's tyre / body planes, so strip-centre sampling re-cut the lit patch every frame (blocks jumping along a
    // face = 'shading glitch'). Each face piece of this slab is split at the cell lines; a cell's strength comes from its
    // centre only, so it is the same whichever slab draws it. Smooth fall-offs (distance, widening cone, height) + 8 buckets.
    // The first piece starts ~1 px before za like the slab tops (their seam overlap paints over that strip of the farther
    // slab's catch).
    const C = this._catch, ctx = this.ctx, K = CATCH.K, cell = CATCH.cell, zo = za - 1.2 / this.ppm;
    const PC = this._catchP || (this._catchP = new Float64Array(4 * 2048)); let np = 0;
    for (let k = k0; k < k1 && np < 2048; k++) {
      const r = F[k], d = r[0] - C.fw;
      if (d < -0.3 || d > FACE.reach || this._overGorge(r[0])) continue;   // gorge walls under a deck: the planks shade them
      const fd = 1 / (1 + (Math.max(0, d) / CATCH.d0) ** 2) * smooth01((FACE.reach - d) / 2.5);
      const fh = lerp(1, CATCH.top, smooth01((Math.min(r[2], r[4]) - C.gy - CATCH.hi) / CATCH.hiR));   // face foot high above the beams
      const f0 = fd * fh;
      if (f0 < 0.04) continue;
      const zl = C.zc + Math.max(0, d) * C.ty, half = CATCH.w + CATCH.k * Math.max(0, d);
      for (let c = Math.floor(za / cell); c * cell < zb && np < 2048; c++) {
        const a = Math.max(za, c * cell), b = Math.min(zb, (c + 1) * cell);
        if (b - a < 1e-4) continue;
        const f = f0 * smooth01((half - Math.abs((c + 0.5) * cell - zl)) / CATCH.soft + 0.5);
        const lv = Math.round(f * K);
        if (lv < 1) continue;
        const o = 4 * np++; PC[o] = k; PC[o + 1] = a; PC[o + 2] = b; PC[o + 3] = lv;
      }
    }
    if (!np) return;
    ctx.save(); ctx.globalCompositeOperation = 'lighter'; ctx.fillStyle = 'rgb(255,214,160)';
    for (let lvl = 1; lvl <= K; lvl++) {
      ctx.beginPath(); let m = 0;
      for (let q = 0; q < np; q++) {
        const o = 4 * q;
        if (PC[o + 3] !== lvl) continue;
        const r = F[PC[o]], a = PC[o + 1], b = PC[o + 2], ua = (a - za) / (zb - za || 1), ub = (b - za) / (zb - za || 1), az = a === za ? zo : a;
        const y0a = lerp(r[2], r[4], ua), y1a = lerp(r[3], r[5], ua), y0b = lerp(r[2], r[4], ub), y1b = lerp(r[3], r[5], ub);
        ctx.moveTo(r[0] + az * C30, y0a - az * S30); ctx.lineTo(r[1] + az * C30, y1a - az * S30);
        ctx.lineTo(r[1] + b * C30, y1b - b * S30); ctx.lineTo(r[0] + b * C30, y0b - b * S30); ctx.closePath(); m++;
      }
      if (m) { ctx.globalAlpha = FACE.a * C.I * lvl / K; ctx.fill(); }
    }
    ctx.restore();
  }

  /** Near-field light (round 12: an obstacle right in front of the bumper stayed black at night): the low beams' footprint
   *  on the ground from just under the bumper to ~NEAR.len m ahead, across the trail, FOLLOWING THE TERRAIN (step faces /
   *  drops ahead catch it). The cached ground-throw sprite is cut into NEAR.n strips along the heading, each mapped by one
   *  affine onto the ground between two centre-line samples (2D: a band just under the surface line). Additive, inside the
   *  front-tyre clip of _atmoPost; ~7 drawImage, no gradients. */
  _nearPool(v, d3, fw, fx, fy, I, T) {
    const ctx = this.ctx, img = T.img, iw = img.width, ih = img.height, N = NEAR.n, L = NEAR.len;
    const yaw = d3 ? v.yaw || 0 : 0, cy = Math.cos(yaw), sy = Math.sin(yaw), zc = d3 ? v.z || 0 : 0, ta = Math.tan(v.angle);
    const x0 = fw - NEAR.back, W = NEAR.w;
    const P = this._nearP || (this._nearP = new Float64Array(3 * (NEAR.n + 1)));
    for (let k = 0; k <= N; k++) {                                   // centre-line samples: world x, y, z
      const d = (k / N) * L, x = x0 + d * cy, z = zc + d * sy;
      let y = d3 ? this._surfY(x, z) : this.groundY(x);
      y = Math.max(y, this._deckY(x));                               // on / near a bridge: the deck, not the gorge floor (r13: the
      const plane = fy + (x - fx) * ta;                              // pool hung a bright wedge into the gorge at both deck ends)                              // the truck's own ground plane: fallback / clamp
      if (!Number.isFinite(y) || y < plane - 3) y = plane;
      P[3 * k] = x; P[3 * k + 1] = y; P[3 * k + 2] = z;
    }
    this._z(0);
    // across vector (full width W) in screen units: 2.5D = the trail's z direction turned by the heading; 2D = a band downward
    const ax = d3 ? W * (-sy + cy * C30) : 0, ay = d3 ? -W * cy * S30 : -NEAR.band;
    ctx.globalAlpha = NEAR.a * I;
    for (let k = 0; k < N; k++) {
      const X0 = P[3 * k] + P[3 * k + 2] * (d3 ? C30 : 0), Y0 = P[3 * k + 1] - P[3 * k + 2] * (d3 ? S30 : 0);
      const X1 = P[3 * k + 3] + P[3 * k + 5] * (d3 ? C30 : 0), Y1 = P[3 * k + 4] - P[3 * k + 5] * (d3 ? S30 : 0);
      const ox = d3 ? X0 - ax / 2 : X0, oy = d3 ? Y0 - ay / 2 : Y0 + NEAR.lift;
      const sx0 = (k / N) * iw, sw = iw / N;
      ctx.save(); ctx.transform(X1 - X0, Y1 - Y0, ax, ay, ox, oy);
      ctx.drawImage(img, sx0, 0, sw, ih, -0.004, 0, 1.008, 1);       // tiny overlap: no seams between strips
      ctx.restore();
    }
  }

  /** Rear lighting (dusk / night, engine on; additive cached sprites, ~6 drawImage): red tail lens glows (+ brighter on the
   *  brake), a very short feathered red ground spill behind the bumper, and in reverse a short warm-white pool. The rear tyres'
   *  silhouette is clipped out so nothing washes over them. Lamp anchor: the rig's drawn tail lenses (`_tail`), else the
   *  rear-most body box. */
  _tailLights(v, d3, k, S) {
    const ctx = this.ctx, tf = this._T, B = this.box || {}, dm = this.dims;
    let tx = this._tail ? this._tail[0] : Infinity, ty = this._tail ? this._tail[1] : 0, tz = this._tail ? this._tail[2] : (dm ? dm.B * 0.85 : 0.8);
    if (!this._tail) for (const q in B) if (B[q].x0 < tx) { tx = B[q].x0; ty = B[q].y1 - 0.06; }   // no drawn lenses (cage rigs): marker glow on the rear-most box's top corner
    if (!Number.isFinite(tx)) return;
    const brake = clamp(v.brake || 0, 0, 1), rev = !!v.reverse, c = Math.cos(v.angle), s = Math.sin(v.angle), D = d3 ? tf : null;
    // lens glows (truck frame, _tz(0); tiny, unclipped: the pools' tyre hull spans the rear face here): near lamp full,
    // far one dimmer; rear face turned away -> both dimmer
    ctx.save(); if (d3) this._tz(0); else this._z(0);
    const back = !d3 || c * tf.bx - s * S30 > 0.05, g = Math.min(1, (0.62 + 0.38 * brake) * k), sz = 0.24 + 0.12 * brake;   // playtest r10: brighter (was 0.3 / 0.18)
    for (const [z, w] of d3 ? [[-tz, 0.6], [tz, 1]] : [[0, 1]]) {
      const X = v.x + c * tx - s * ty + (D ? z * D.Dx : 0), Y = v.y + s * tx + c * ty + (D ? z * D.Dy : 0);
      ctx.globalAlpha = g * w * (back ? 1 : 0.45); ctx.drawImage(S.red, X - sz, Y - sz, 2 * sz, 2 * sz);
      const cz = sz * 0.38; ctx.globalAlpha = Math.min(1, (0.7 + 0.3 * brake) * k) * w * (back ? 1 : 0.5); ctx.drawImage(S.red, X - cz, Y - cz, 2 * cz, 2 * cz);   // hot lens core
      if (rev) { ctx.globalAlpha = 0.55 * w * k * (back ? 1 : 0.45); ctx.drawImage(S.rev, X - 0.14, Y - 0.24, 0.28, 0.28); }   // reverse lamp just under the tail lens
    }
    ctx.restore();
    ctx.save();
    { // ground pools: clip out the rear tyres (one hull)
      if (d3) this._tz(0); else this._z(0);
      ctx.beginPath(); ctx.rect(v.x - 60, v.y - 60, 120, 120);
      const rr = (v.wheels || []).filter(o => o.axle === 1);
      if (d3 && dm && rr.length) {
        const F = []; for (const o of rr) F.push(...this._tyreOutline(o, dm.tw).slice(0, 64));
        hullPath(ctx, F);
      } else for (const o of rr) { const r = (o.radius || 0.5) + 0.06; ctx.moveTo((o.hx ?? o.x) + r, o.hy ?? o.y); ctx.arc(o.hx ?? o.x, o.hy ?? o.y, r, 0, TAU); }
      ctx.clip('evenodd');
    }
    // ground spills behind the bumper (_z(0) ground plane, oriented with the heading); start behind the rear tyres
    this._z(0);
    let rx = v.x + c * tx;
    for (const o of v.wheels || []) rx = Math.min(rx, (o.hx ?? o.x) - (o.radius || 0.5));
    const yaw = d3 ? v.yaw || 0 : 0, cy = Math.cos(yaw), sy = Math.sin(yaw), zc = d3 ? v.z || 0 : 0;
    const pool = (img, d, ra, rb, a) => {
      const x = rx - d * cy, z = zc - d * sy, gy = d3 ? this._surfY(x, z) : this.groundY(x);
      if (!Number.isFinite(gy)) return;
      ctx.save(); ctx.globalAlpha = a;
      if (d3) ctx.transform(ra * (cy + sy * C30), -ra * sy * S30, rb * (-sy + cy * C30), -rb * cy * S30, x + z * C30, gy - z * S30);
      else ctx.transform(ra, 0, 0, 0.22, x, gy - 0.08);
      ctx.drawImage(img, -1, -1, 2, 2); ctx.restore();
    };
    pool(S.red, 0.45, 0.8, 1.35, (0.2 + 0.14 * brake) * k);         // very short red spill
    if (rev) pool(S.rev, 1.2, 1.35, 1.4, 0.22 * k);                 // short reverse pool
    ctx.restore();
  }

  /** One flora plant at ground (x, y), world units (y up), scale s; t = time (sway). */
  _plant(f, x, y, far) {
    const ctx = this.ctx, s = f.s, sw = Math.sin(this.t * 1.3 + f.seed) * 0.04 * s;
    if (f.kind === 'pine') {
      const hgt = 2.1 * s + 0.9;
      ctx.lineWidth = 0.16 * s; ctx.strokeStyle = '#4a3420'; ctx.beginPath(); ctx.moveTo(x, y - 0.1); ctx.lineTo(x, y + hgt * 0.35); ctx.stroke();
      const dark = f.col, lit = f.lit;
      for (let pass = 0; pass < 2; pass++) {                         // batched: all dark tiers, then all lit flanks (tiers only overlap
        ctx.fillStyle = pass ? lit : dark; ctx.beginPath();          // upward; a lit flank sits inside its own tier)
        for (let i = 0; i < 4; i++) {
          const yb = y + hgt * (0.2 + i * 0.19), wd = (1.05 - i * 0.2) * s, yt = yb + hgt * 0.34, k = sw * (i + 1) * 0.6;
          if (pass) { ctx.moveTo(x + k, yt); ctx.lineTo(x + wd, yb); ctx.lineTo(x + wd * 0.35, yb + 0.05); }
          else { ctx.moveTo(x - wd, yb); ctx.lineTo(x + k, yt); ctx.lineTo(x + wd, yb); }
          ctx.closePath();
        }
        ctx.fill();
      }
      return;
    }
    if (f.kind === 'bush') {
      const r = 0.38 * s, base = f.base, lit = f.lit;
      const blobs = [[-0.9, 0.5, 0.85], [0.8, 0.55, 0.8], [0, 0.95, 1], [-0.4, 1.35, 0.7], [0.5, 1.3, 0.65]];
      ctx.fillStyle = base; ctx.beginPath(); for (const [bx, by, br] of blobs) { const cx = x + bx * r + sw * by; ctx.moveTo(cx + br * r, y + by * r); ctx.arc(cx, y + by * r, br * r, 0, TAU); } ctx.fill();
      ctx.fillStyle = f.col; ctx.beginPath(); for (const [bx, by, br] of blobs) { const cx = x + bx * r - 0.06 + sw * by; ctx.moveTo(cx + br * r * 0.78, y + by * r + 0.07); ctx.arc(cx, y + by * r + 0.07, br * r * 0.78, 0, TAU); } ctx.fill();
      ctx.fillStyle = lit; ctx.beginPath(); ctx.arc(x - 0.25 * r + sw, y + 1.25 * r, 0.32 * r, 0, TAU); ctx.fill();
      if (h01(f.seed) < 0.4) { ctx.fillStyle = f.acc; ctx.beginPath(); ctx.arc(x + 0.7 * r + sw, y + 0.9 * r, 0.3 * r, 0, TAU); ctx.fill(); }
      return;
    }
    ctx.lineCap = 'round';
    if (f.kind === 'fern') {
      ctx.strokeStyle = f.col; ctx.lineWidth = 0.05 * s;
      ctx.beginPath();
      for (let i = 0; i < 7; i++) {
        const a = Math.PI * (0.15 + 0.7 * i / 6), L = (0.55 + 0.35 * Math.sin(i * 1.7 + f.seed)) * s;
        const ex = x + Math.cos(a) * L + sw * 2, ey = y + Math.sin(a) * L * 0.9;
        ctx.moveTo(x, y); ctx.quadraticCurveTo(x + Math.cos(a) * L * 0.5, y + Math.sin(a) * L * 1.05, ex, ey);
      }
      ctx.stroke();
      ctx.strokeStyle = f.acc; ctx.lineWidth = 0.035 * s; ctx.beginPath(); ctx.moveTo(x, y); ctx.quadraticCurveTo(x + 0.1, y + 0.5 * s, x + 0.3 * s + sw * 2, y + 0.7 * s); ctx.stroke();
      return;
    }
    ctx.strokeStyle = f.col; ctx.lineWidth = 0.035;                 // grass tuft
    ctx.beginPath();
    for (let i = 0; i < 7; i++) { const dx = (i - 3) * 0.06 * s, L = (0.3 + 0.25 * h01(f.seed + i)) * s; ctx.moveTo(x + dx, y); ctx.quadraticCurveTo(x + dx * 1.4, y + L * 0.6, x + dx * 2.2 + sw * 3, y + L); }
    ctx.stroke();
    ctx.strokeStyle = f.acc; ctx.beginPath(); ctx.moveTo(x, y); ctx.quadraticCurveTo(x + 0.04, y + 0.3 * s, x + 0.1 * s + sw * 3, y + 0.55 * s); ctx.stroke();
  }

  /** Near-wall detail of a ground run (2D wall / 2.5D last slab): rock strata + joints, dirt pebbles, mud gloss.
   *  top(i) -> [x, y] of profile vertex i (already shifted to the wall plane). */
  _wallDetail(surface, i0, i1, top) {
    if (!this.q.detail) return;
    const ctx = this.ctx;
    if (isRock(surface)) {
      for (let k = 1; k <= 4; k++) {                                 // strata bands following the profile
        const dy = -(0.18 + k * 0.34 + 0.06 * Math.sin(k * 2.3));
        ctx.beginPath();
        for (let i = i0; i <= i1; i++) { const [x, y] = top(i); const yy = y + dy + 0.05 * Math.sin(x * 1.7 + k); i === i0 ? ctx.moveTo(x, yy) : ctx.lineTo(x, yy); }
        ctx.lineWidth = 0.06; ctx.strokeStyle = 'rgba(0,0,0,0.16)'; ctx.stroke();
        ctx.save(); ctx.translate(0, 0.05); ctx.lineWidth = 0.03; ctx.strokeStyle = 'rgba(255,255,255,0.08)'; ctx.stroke(); ctx.restore();
      }
      ctx.beginPath();                                               // vertical joints (blocky ledge faces)
      for (let i = i0; i < i1; i++) {
        const [x, y] = top(i), [x2] = top(i + 1);
        if (x2 - x < 0.05) continue;
        const n = Math.max(1, Math.round((x2 - x) / 1.1));
        for (let j = 0; j < n; j++) {
          const u = (j + 0.3 + 0.4 * h01(i * 7 + j, 3)) / n, xx = lerp(x, x2, u), yy = lerp(y, top(i + 1)[1], u), d = 0.4 + 0.8 * h01(i * 7 + j, 5);
          ctx.moveTo(xx, yy - 0.05); ctx.lineTo(xx + 0.06, yy - d * 0.5); ctx.lineTo(xx - 0.03, yy - d);
        }
      }
      ctx.lineWidth = 0.035; ctx.strokeStyle = 'rgba(0,0,0,0.22)'; ctx.stroke();
    } else if (isWet(surface) || isWater(surface)) {
      ctx.beginPath();
      for (let i = i0; i <= i1; i++) { const [x, y] = top(i); i === i0 ? ctx.moveTo(x, y - 0.06) : ctx.lineTo(x, y - 0.06); }
      ctx.lineWidth = 0.04; ctx.strokeStyle = isWater(surface) ? 'rgba(220,245,250,0.5)' : 'rgba(255,255,255,0.16)'; ctx.stroke();
      if (isWater(surface)) {                                        // drifting ripples on the water line
        ctx.beginPath();
        for (let i = i0; i < i1; i++) {
          const [x, y] = top(i), x2 = top(i + 1)[0];
          for (let k = 0; k < 2; k++) { const xx = lerp(x, x2, (h01(i, k) + this.t * 0.1 * (k + 1)) % 1); ctx.moveTo(xx - 0.15, y - 0.12 - k * 0.1); ctx.quadraticCurveTo(xx, y - 0.09 - k * 0.1, xx + 0.15, y - 0.12 - k * 0.1); }
        }
        ctx.lineWidth = 0.025; ctx.strokeStyle = 'rgba(200,235,240,0.45)'; ctx.stroke();
      }
      if (!isWater(surface)) for (let i = i0; i < i1; i++) {         // wet drips down the cut
        if (h01(i, 9) < 0.55) continue;
        const [x, y] = top(i); ctx.fillStyle = 'rgba(20,12,6,0.35)'; ctx.fillRect(x, y - 0.9 - h01(i, 2), 0.06, 0.8 + h01(i, 2));
      }
    } else {
      for (const dk of [true, false]) {                              // pebbles in the dirt cut (one path per shade)
        ctx.fillStyle = dk ? 'rgba(0,0,0,0.22)' : 'rgba(255,240,220,0.14)'; ctx.beginPath();
        for (let i = i0; i < i1; i++) {
          const [x, y] = top(i), [x2, y2] = top(i + 1);
          for (let j = 0; j < 2; j++) {
            if ((h01(i, j + 41) < 0.5) !== dk) continue;
            const u = h01(i, j + 11), d = 0.25 + 1.6 * h01(i, j + 21), r = 0.04 + 0.07 * h01(i, j + 31), ex = lerp(x, x2, u), ey = lerp(y, y2, u) - d;
            hexDot(ctx, ex, ey, r * 1.4, r);
          }
        }
        ctx.fill();
      }
    }
  }

  _surfColor(name) {                                                // authored colour, colour-graded once (gradeHex), cached
    let c = this._gradeMap.get(name);
    if (!c) { const s = this.level.surfaces[name]; this._gradeMap.set(name, c = gradeHex((s && s.color) || '#777')); }
    return c;
  }

  _ground(L, xl, xr) {
    const ctx = this.ctx, bottom = L.bottom - 40;
    for (const run of this.ground) {
      const p = run.points;
      const path = dy => {
        ctx.beginPath(); ctx.moveTo(p[0][0], bottom);
        for (const q of p) ctx.lineTo(q[0], q[1] + dy);
        ctx.lineTo(p[p.length - 1][0], bottom); ctx.closePath();
      };
      path(0); ctx.fillStyle = this._surfColor(run.surface); ctx.fill();
      path(-0.45); ctx.fillStyle = 'rgba(0,0,0,0.22)'; ctx.fill();
      path(-2.2); ctx.fillStyle = 'rgba(0,0,0,0.18)'; ctx.fill();
      let i0 = 0, i1 = p.length - 1;
      while (i0 < i1 && p[i0 + 1][0] < xl) i0++;
      while (i1 > i0 && p[i1 - 1][0] > xr) i1--;
      this._wallDetail(run.surface, i0, i1, i => p[i]);
      if (run.d) this._churn(run, xl, xr);
      ctx.beginPath(); ctx.moveTo(p[0][0], p[0][1]);
      for (const q of p) ctx.lineTo(q[0], q[1]);
      ctx.lineWidth = 0.09;
      ctx.strokeStyle = run.surface === 'mud' ? '#2c2018' : 'rgba(255,255,255,0.28)';
      ctx.stroke();
    }
  }

  /** Churned soft ground: a dark, wet band under the rut surface, deeper rut = thicker + darker, glossy floor line. */
  _churn(run, xl, xr) {
    const ctx = this.ctx, p = run.points, d = run.d, th = i => 0.1 + 0.8 * d[i];
    ctx.save();
    ctx.fillStyle = run.churn;
    for (let i = 1; i < p.length; i++) {
      const dm = Math.max(d[i - 1], d[i]);
      if (dm < 0.005 || p[i][0] < xl || p[i - 1][0] > xr) continue;
      ctx.globalAlpha = Math.min(0.85, 0.25 + 1.6 * dm);
      ctx.beginPath();
      ctx.moveTo(p[i - 1][0], p[i - 1][1] + 0.01); ctx.lineTo(p[i][0], p[i][1] + 0.01);
      ctx.lineTo(p[i][0], p[i][1] - th(i)); ctx.lineTo(p[i - 1][0], p[i - 1][1] - th(i - 1));
      ctx.closePath(); ctx.fill();
    }
    ctx.globalAlpha = 1; ctx.lineWidth = 0.025; ctx.strokeStyle = 'rgba(255,240,220,0.16)';
    ctx.beginPath();
    for (let i = 1; i < p.length; i++) {
      if (Math.min(d[i - 1], d[i]) < 0.03 || p[i][0] < xl || p[i - 1][0] > xr) continue;
      ctx.moveTo(p[i - 1][0], p[i - 1][1] - 0.07); ctx.lineTo(p[i][0], p[i][1] - 0.07);
    }
    ctx.stroke();
    ctx.restore();
  }

  _zones(L) {
    const ctx = this.ctx;
    for (const z of L.zones) {
      ctx.beginPath();
      let any = false;
      for (const [a, b] of this.segs) {
        const x0 = Math.max(Math.min(a[0], b[0]), z.x0), x1 = Math.min(Math.max(a[0], b[0]), z.x1);
        if (x1 - x0 < 1e-6 || Math.abs(b[0] - a[0]) < 1e-6) continue;
        const y0 = lerp(a[1], b[1], (x0 - a[0]) / (b[0] - a[0])), y1 = lerp(a[1], b[1], (x1 - a[0]) / (b[0] - a[0]));
        ctx.moveTo(x0, y0 - 0.1); ctx.lineTo(x1, y1 - 0.1); any = true;
      }
      if (!any) continue;
      ctx.lineWidth = 0.26; ctx.strokeStyle = this._surfColor(z.surface);
      ctx.lineCap = 'butt';
      ctx.setLineDash(z.side === 'both' ? [] : [0.55, 0.35]);   // dashed = one side of the vehicle only
      ctx.stroke(); ctx.setLineDash([]); ctx.lineCap = 'round';
    }
  }

  _polygons(L) {
    const ctx = this.ctx;
    for (const poly of L.polygons) {
      const p = poly.points;
      ctx.beginPath(); ctx.moveTo(p[0][0], p[0][1]);
      for (let i = 1; i < p.length; i++) ctx.lineTo(p[i][0], p[i][1]);
      ctx.closePath();
      ctx.fillStyle = this._surfColor(poly.surface); ctx.fill();
      ctx.lineWidth = 0.06; ctx.strokeStyle = 'rgba(0,0,0,0.45)'; ctx.stroke();
    }
  }

  /** Wooden stage post (world units, y up): post + sign board: a flag pictogram + the stage number (no words). */
  _stagePost(x, gy, name) {
    const ctx = this.ctx;
    ctx.lineWidth = 0.16; ctx.strokeStyle = '#5b4026';
    ctx.beginPath(); ctx.moveTo(x, gy - 0.1); ctx.lineTo(x, gy + 1.9); ctx.stroke();
    ctx.fillStyle = '#9a7850'; ctx.strokeStyle = '#3d2a17'; ctx.lineWidth = 0.05;
    ctx.fillRect(x - 0.75, gy + 1.45, 1.5, 0.5); ctx.strokeRect(x - 0.75, gy + 1.45, 1.5, 0.5);
    ctx.save(); ctx.translate(x, gy + 1.7); ctx.scale(0.01, -0.01);
    const num = (String(name || '').match(/\d+/) || [''])[0];
    if (num) {
      paintIcon(ctx, 'flag', -52, -19, 38, { color: '#fff2d0', lw: 2.6 });
      ctx.fillStyle = '#fff2d0'; ctx.font = 'bold 34px system-ui, sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(num, 18, 1);
    } else {
      ctx.fillStyle = '#fff2d0'; ctx.font = 'bold 30px system-ui, sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(name, 0, 0);
    }
    ctx.restore();
  }

  _props(L, v) {
    const ctx = this.ctx;
    for (const st of L.stages || []) this._stagePost(st.x, this.groundY(st.x), st.name);
    for (const a of L.anchors) {
      const gy = this.groundY(a.x);
      ctx.lineWidth = 0.22; ctx.strokeStyle = '#5b4026';
      ctx.beginPath(); ctx.moveTo(a.x, gy - 0.1); ctx.lineTo(a.x, a.y + 0.25); ctx.stroke();
      const hot = v.winch.attached && Math.abs(v.winch.ax - a.x) < 0.01 && Math.abs(v.winch.ay - a.y) < 0.01;
      ctx.lineWidth = 0.07; ctx.strokeStyle = hot ? '#ffd23c' : '#e8e2c8';
      ctx.beginPath(); ctx.arc(a.x, a.y, 0.14, 0, TAU); ctx.stroke();
    }
    if (L.finish) {
      const x = L.finish.x, gy = this.groundY(x);
      ctx.lineWidth = 0.08; ctx.strokeStyle = '#f2f2f2';
      ctx.beginPath(); ctx.moveTo(x, gy); ctx.lineTo(x, gy + 2.4); ctx.stroke();
      for (let i = 0; i < 3; i++) for (let j = 0; j < 2; j++) {
        ctx.fillStyle = (i + j) % 2 ? '#111' : '#f2f2f2';
        ctx.fillRect(x + i * 0.3, gy + 1.8 + j * 0.3, 0.3, 0.3);
      }
    }
  }

  _dynamic(dyn, L) {
    const ctx = this.ctx;
    for (const p of dyn.planks) {
      ctx.save(); ctx.translate(p.x, p.y); ctx.rotate(p.a);
      ctx.fillStyle = '#8b6b47'; ctx.fillRect(-p.hw, -p.hh, p.hw * 2, p.hh * 2);
      ctx.lineWidth = 0.03; ctx.strokeStyle = '#4c3924'; ctx.strokeRect(-p.hw + 0.02, -p.hh, p.hw * 2 - 0.04, p.hh * 2);
      ctx.restore();
    }
    const cans = (L && L.cans) || [];
    for (let i = 0; i < cans.length; i++) if (!(dyn.cans && dyn.cans[i])) this._jerryCan(cans[i].x, this.groundY(cans[i].x), i);
    const defs = (L && L.logDefs) || [];                  // level JSON logs (same order): surface 'rock' = loose boulder
    dyn.logs.forEach((l, i) => {
      if (defs[i] && /rock|stone/.test(defs[i].surface || '')) {
        const r = l.r;
        ctx.save(); ctx.translate(l.x, l.y); ctx.rotate(l.a);
        ctx.beginPath();
        for (let k = 0; k < 9; k++) { const a = k * TAU / 9, q = r * (0.9 + 0.1 * Math.sin(k * 2.7 + i)); k ? ctx.lineTo(Math.cos(a) * q, Math.sin(a) * q) : ctx.moveTo(q, 0); }
        ctx.closePath(); ctx.fillStyle = this._surfColor(defs[i].surface); ctx.fill();
        ctx.lineWidth = 0.04; ctx.strokeStyle = 'rgba(0,0,0,0.45)'; ctx.stroke();
        ctx.beginPath(); ctx.moveTo(-r * 0.45, r * 0.15); ctx.lineTo(r * 0.1, -r * 0.2); ctx.lineTo(r * 0.5, r * 0.05);
        ctx.lineWidth = 0.025; ctx.strokeStyle = 'rgba(255,255,255,0.22)'; ctx.stroke();
        ctx.restore();
        return;
      }
      this._logFace(l);
    });
  }

  /** Red jerry can (fuel pickup) standing on (x, y), with a slow gold halo pulse so it reads from afar. */
  _jerryCan(x, y, i = 0) {
    const ctx = this.ctx, k = 0.5 + 0.5 * Math.sin((this.t || 0) * 3.2 + i * 1.7), w = 0.34, h = 0.44, x0 = x - w / 2;
    ctx.globalAlpha = 0.14 + 0.22 * k; ctx.fillStyle = '#ffcd46';
    ctx.beginPath(); ctx.ellipse(x, y + 0.02, 0.36 + 0.06 * k, 0.1 + 0.02 * k, 0, 0, TAU); ctx.fill();
    ctx.globalAlpha = 1;
    ctx.fillStyle = '#b8241b'; ctx.strokeStyle = '#5e120d'; ctx.lineWidth = 0.025;
    ctx.beginPath(); ctx.moveTo(x0 + 0.04, y); ctx.lineTo(x0 + w - 0.04, y); ctx.quadraticCurveTo(x0 + w, y, x0 + w, y + 0.04);
    ctx.lineTo(x0 + w, y + h - 0.07); ctx.lineTo(x0 + w - 0.07, y + h); ctx.lineTo(x0 + 0.04, y + h); ctx.quadraticCurveTo(x0, y + h, x0, y + h - 0.04);
    ctx.lineTo(x0, y + 0.04); ctx.quadraticCurveTo(x0, y, x0 + 0.04, y); ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.strokeStyle = 'rgba(255, 150, 130, 0.55)'; ctx.lineWidth = 0.022;            // pressed X + light rim
    ctx.beginPath(); ctx.moveTo(x0 + 0.06, y + 0.06); ctx.lineTo(x0 + w - 0.06, y + h - 0.14); ctx.moveTo(x0 + w - 0.06, y + 0.06); ctx.lineTo(x0 + 0.06, y + h - 0.14); ctx.stroke();
    ctx.fillStyle = '#8f1b14';                                                          // handle bar (three grips) + spout cap
    ctx.fillRect(x0 + 0.03, y + h - 0.02, 0.2, 0.07);
    ctx.fillStyle = '#e0c25a'; ctx.fillRect(x0 + w - 0.09, y + h - 0.01, 0.07, 0.07);
    ctx.fillStyle = 'rgba(0, 0, 0, 0.35)';
    for (let j = 0; j < 3; j++) ctx.fillRect(x0 + 0.05 + j * 0.06, y + h + 0.005, 0.035, 0.03);
  }

  _logFace(l) {
    const ctx = this.ctx;
    ctx.save(); ctx.translate(l.x, l.y); ctx.rotate(l.a);
    {
      ctx.beginPath(); ctx.arc(0, 0, l.r, 0, TAU); ctx.fillStyle = '#9a7850'; ctx.fill();
      ctx.lineWidth = 0.05; ctx.strokeStyle = '#5c4326'; ctx.stroke();
      ctx.lineWidth = 0.025;
      ctx.beginPath(); ctx.arc(0, 0, l.r * 0.62, 0, TAU); ctx.stroke();
      ctx.beginPath(); ctx.arc(0, 0, l.r * 0.28, 0, TAU); ctx.moveTo(l.r * 0.28, 0); ctx.lineTo(l.r * 0.95, 0); ctx.stroke();
      ctx.restore();
    }
  }

  // -- 2.5D oblique camera ---------------------------------------------------------------------------
  // Depth z across the trail (+z = truck right = near side). Every planar layer at depth z is the 2D drawing shifted
  // by (z·cos30, −z·sin30); a point nearer on a screen ray always has larger z, so drawing z-slabs far -> near is an
  // exact painter's order. Slab bounds (world z): berm rise | trail | berm rise, plus rutted lane centres, zone dy
  // edges and the truck's body / tyre planes. Each slab top is bilinear between the surface profiles at its two bounds
  // (ground + berm + zone dy − lane rut), so ruts, berms and side steps show as real walls.
  // The truck is yawed / offset: its parts are drawn through an affine "truck frame" (_tz) and sorted by local z.
  _area(q) { let a = 0; for (let i = 0; i < q.length; i++) { const p = q[i], n = q[(i + 1) % q.length]; a += p[0] * n[1] - n[0] * p[1]; } return a; }

  _dims(v) {
    const sh = this.shape, W = v.wheels, ca = Math.cos(v.angle), sa = Math.sin(v.angle), r = sh.wheelRadius || W[0].radius, T = this.trail;
    let xf = -Infinity, xr = Infinity, yf = 0;
    for (const o of W) { const lx = ((o.hx ?? o.x) - v.x) * ca + ((o.hy ?? o.y) - v.y) * sa; if (lx > xf) yf = -((o.hx ?? o.x) - v.x) * sa + ((o.hy ?? o.y) - v.y) * ca; xf = Math.max(xf, lx); xr = Math.min(xr, lx); }
    const h = sh.halfTrack || clamp(0.3 * (xf - xr), 0.75, 1.15), tw = sh.tyreWidth || clamp(r * 0.8, 0.32, 0.55);   // half track, tyre width (m)
    const B = h - tw / 2 - 0.1, D = (T.rim ?? T.edge + T.bermW + 0.4) + 0.05;   // D: drawn trail half-width (just past the rim)
    return { h, tw, B, D, sz: h - tw / 2 - 0.05, W: 1.9,   // W: bridge deck half-width (rails at BRIDGE.rail)
      zA: -D + 0.35, zStack: -B * 0.55, front: xf + r, fly: yf - r };   // front, fly: front tyre face / contact (chassis frame)
  }

  _z(z) { this.ctx.setTransform(this.ppm, 0, 0, -this.ppm, this._ox + z * C30 * this.ppm, this._oy + z * S30 * this.ppm); }

  /** Truck frame at local depth lz: pitched chassis coords (world x, y as the 2D view draws them) -> yawed, z-offset screen. */
  _tz(lz) {
    const t = this._T;
    this._z(0);
    if (!t.sr) { this.ctx.transform(t.a, t.b, 0, 1, t.xc * (1 - t.a) + t.zc * C30 + lz * t.bx, -t.b * t.xc - t.zc * S30 + lz * t.by); return; }
    // edge tip-over roll r about the truck's long axis at chassis height yA: y' = yA + (y - yA)cos r - lz sin r, depth' = lz cos r + (y - yA) sin r
    const sr = t.sr, cr = t.cr, yA = t.yA, e0 = t.xc * (1 - t.a) + t.zc * C30, f0 = -t.b * t.xc - t.zc * S30, d = cr + sr * t.by;
    this.ctx.transform(t.a, t.b, sr * t.bx, d, e0 + lz * cr * t.bx - yA * sr * t.bx, f0 + yA - lz * sr + lz * cr * t.by - yA * d);
  }

  /** Rolled frame (t.sr / t.cr set): the extrusion vector must map (through _tz's linear part [[a, C], [b, d]]) onto the
   *  rolled depth axis (cr·bx, −sr + cr·by), else every prism collapses flat and depth-offset decor drifts off its face. */
  _rollDz(t) {
    const C = t.sr * t.bx, d = t.cr + t.sr * t.by, det = t.a * d - t.b * C;
    const X = t.cr * t.bx, Y = -t.sr + t.cr * t.by;
    if (Math.abs(det) > 1e-3) { t.Dx = (d * X - C * Y) / det; t.Dy = (t.a * Y - t.b * X) / det; }
  }

  /** Per frame: affine coefficients of the truck frame from heading yaw and trail z (see _tz). */
  _truckFrame(v) {
    const y = v.yaw || 0, c = Math.cos(y), s = Math.sin(y), t = this._T || (this._T = {});
    t.a = c + s * C30; t.b = -s * S30; t.bx = C30 * c - s; t.by = -c * S30; t.xc = v.x; t.zc = v.z || 0;
    t.Dx = t.bx / t.a; t.Dy = t.by - t.b * t.Dx;                     // local depth vector in pre-transform coords
    t.cy = c; t.sy = s; t.ang = v.angle;
    const r = v.roll || 0; t.sr = Math.abs(r) > 1e-4 ? Math.sin(r) : 0; t.cr = Math.cos(r); t.yA = v.y;   // edge tip-over roll (_tz)
    if (t.sr) this._rollDz(t);
    // body roll (render only: physics is planar): solid axles tilt with the lane heights under their tyres (left vs right
    // hub height); the sprung body follows BODY_ROLL of the mean axle tilt. Far side (lz < 0) higher = positive roll.
    let n = 0, d = 0;
    for (const ax of [0, 1]) {
      let yl = 0, yr = 0, hl = 0, nl = 0, nr = 0;
      for (const o of v.wheels || []) if (o.axle === ax) { if (o.lz < 0) { yl += o.hy ?? o.y; nl++; hl = -o.lz; } else { yr += o.hy ?? o.y; nr++; hl = hl || o.lz; } }
      if (nl && nr && hl > 0) { d += Math.atan2(yl / nl - yr / nr, 2 * hl); n++; }
    }
    const brT = n && !t.sr ? clamp(BODY_ROLL * d / n, -0.35, 0.35) : 0;
    // the sprung body lags the axles and sways past them before it settles (~1.4 Hz, under-damped): mass, not a rigid toy
    const S = this._brS || (this._brS = { x: brT, v: 0 }), dt = this._fdt || 0;
    if (t.sr || this.showroom) { S.x = brT; S.v = 0; }
    else if (dt > 0) { const w = TAU * 1.4; S.v += (w * w * (brT - S.x) - 2 * 0.32 * w * S.v) * dt; S.x = clamp(S.x + S.v * dt, -0.4, 0.4); }
    t.br = S.x;
  }

  _shades(col) {
    let s = this._shadeMap.get(col);
    if (!s) this._shadeMap.set(col, s = { cap: col, top: mix(col, '#ffffff', 0.2), side: mix(col, '#000000', 0.15), rear: mix(col, '#000000', 0.32),
      edge: mix(col, '#000000', 0.45) });   // opaque edge: props are re-stroked every slab, alpha would stack to black
    return s;
  }

  /** Rock lump mesh (cached per key; rebuilt only when its log moves): rings x longitudes of an irregular ellipsoid
   *  (rx = ry·1.08 = r, rz) around (l.x, l.y, cz), rotated with the log, every vertex kept no lower than the drawn
   *  terrain and the bottom rings pulled down onto it (seated: never floats over a dip, never shows below the surface). */
  _lumpMesh(key, l, cz, r, rz, surface, dx = 0, box = false) {
    const C = this._lumps || (this._lumps = new Map());
    let m = C.get(key);
    const cx = l.x + dx;
    if (m && Math.abs(m.x - cx) < 0.004 && Math.abs(m.y - l.y) < 0.004 && Math.abs(m.a - l.a) < 0.004 && m.cz === cz && m.lv === this.level) return m;
    const NR = 6, NL = 10, V = new Float64Array((NR + 1) * NL * 3), G = new Uint8Array((NR + 1) * NL), ca = Math.cos(l.a), sa = Math.sin(l.a);
    // box: a fractured ledge block instead of a cobble -- superellipse (flat top, steep sides), stronger facet jitter
    const ex = box ? 0.55 : 1, sp = (t, e) => Math.sign(t) * Math.abs(t) ** e, j0 = box ? 0.78 : 0.84, j1 = box ? 0.3 : 0.24;
    const rn = (j, k) => { const h = Math.sin(key * 7.13 + j * 3.71 + k * 1.93 + (j === 0 || j === NR ? 99 * j : 0) * (k === 0 ? 1 : 0)) * 15731.7; return h - Math.floor(h); };
    for (let j = 0; j <= NR; j++) {
      const th = -Math.PI / 2 + Math.PI * j / NR, ct = Math.cos(th), st = Math.sin(th);
      for (let k = 0; k < NL; k++) {
        const ph = TAU * (k + (j & 1) * 0.5) / NL, q = j === 0 || j === NR ? 0.92 + 0.1 * rn(j, 0) : Math.min(box && j >= NR - 1 ? 1 : 9, j0 + j1 * rn(j, k));
        const ce = sp(ct, ex), u = r * ce * sp(Math.cos(ph), ex) * q, v = r * 0.92 * sp(st, ex) * (j === NR ? (box ? 0.97 : 0.8) : 1) * q, w = rz * ce * sp(Math.sin(ph), ex) * q;
        const o = 3 * (j * NL + k);
        V[o] = cx + ca * u - sa * v; V[o + 1] = l.y + sa * u + ca * v; V[o + 2] = cz + w;
        const gy = this._surfY(V[o], V[o + 2]);
        if (Number.isFinite(gy)) {
          if (V[o + 1] < gy - 0.04) { V[o + 1] = gy - 0.04; G[j * NL + k] = 1; }   // buried part: flattened onto the surface
          else if (j <= 1 && V[o + 1] > gy - 0.04 && V[o + 1] - gy < r) { V[o + 1] = gy - 0.04; G[j * NL + k] = 1; }   // hanging bottom: down to it
        }
      }
    }
    m = { x: cx, y: l.y, a: l.a, cz, lv: this.level, V, G, NR, NL, P: new Float64Array((NR + 1) * NL * 2), fz: new Float64Array(NR * NL),
      vis: new Int8Array(NR * NL), lev: new Int8Array(NR * NL), pal: this._lumpPal(this._surfColor(surface)) };
    const L0 = -0.38, L1 = 0.82, L2 = 0.43, Ln = Math.hypot(L0, L1, L2);
    for (let j = 0; j < NR; j++) for (let k = 0; k < NL; k++) {      // face (j,k): ring j..j+1, longitude k..k+1 (fixed shading)
      const f = j * NL + k, k1 = (k + 1) % NL, i0 = 3 * (j * NL + k), i1 = 3 * (j * NL + k1), i2 = 3 * ((j + 1) * NL + k1), i3 = 3 * ((j + 1) * NL + k);
      const ax = V[i2] - V[i0], ay = V[i2 + 1] - V[i0 + 1], az = V[i2 + 2] - V[i0 + 2], bx = V[i3] - V[i1], by = V[i3 + 1] - V[i1 + 1], bz = V[i3 + 2] - V[i1 + 2];
      let nx = ay * bz - az * by, ny = az * bx - ax * bz, nz = ax * by - ay * bx;
      const cxm = (V[i0] + V[i2]) / 2 - cx, cym = (V[i0 + 1] + V[i2 + 1]) / 2 - l.y, czm = (V[i0 + 2] + V[i2 + 2]) / 2 - cz;
      if (nx * cxm + ny * cym + nz * czm < 0) { nx = -nx; ny = -ny; nz = -nz; }   // outward
      const nl = Math.hypot(nx, ny, nz) || 1;
      m.fz[f] = (V[i0 + 2] + V[i1 + 2] + V[i2 + 2] + V[i3 + 2]) / 4;
      const vis = (-nx * C30 + ny * S30 + nz) / nl;                  // view axis (-cos30, sin30, 1): faces toward the camera
      const gn = G[i0 / 3] + G[i1 / 3] + G[i2 / 3] + G[i3 / 3];
      // r15: a face lying flat in the ground (all 4 corners flattened onto it) is not drawn: its outline was the 'chain'
      // around half-buried cobbles; faces touching the ground are a shade darker (contact / dirt line)
      m.vis[f] = vis > 1e-3 && gn < 4 ? 1 : 0;
      const lit = (nx * L0 + ny * L1 + nz * L2) / (nl * Ln);
      m.lev[f] = Math.max(0, Math.min(7, Math.round(3.4 + lit * 3.6) - (gn >= 2 ? 2 : gn ? 1 : 0)));
    }
    C.set(key, m);
    return m;
  }

  _lumpPal(col) {
    const M = this._lumpPals || (this._lumpPals = new Map());
    let p = M.get(col);
    if (!p) { p = []; for (let q = 0; q < 8; q++) p.push(q < 4 ? mix(col, '#000000', 0.42 - q * 0.1) : mix(col, '#ffffff', (q - 3) * 0.06)); p.edge = mix(col, '#000000', 0.5); M.set(col, p); }
    return p;
  }

  /** Draw lump m: front faces with centroid depth in [a, b] (whole range at the item's ends), batched per shade level,
   *  then the silhouette (front/back face boundaries) as a thin dark outline. */
  _rockLump(m, a, b, it) {
    if (!m) return;
    const ctx = this.ctx, V = m.V, P = m.P, G = m.G, NR = m.NR, NL = m.NL, lo = a <= it.za + 1e-9 ? -Infinity : a, hi = b >= it.zb - 1e-9 ? Infinity : b;
    for (let i = 0, n = (NR + 1) * NL; i < n; i++) { P[2 * i] = V[3 * i] + V[3 * i + 2] * C30; P[2 * i + 1] = V[3 * i + 1] - V[3 * i + 2] * S30; }
    const on = f => m.vis[f] && m.fz[f] >= lo && m.fz[f] <= hi;
    const quad = (j, k) => { const k1 = (k + 1) % NL, q0 = 2 * (j * NL + k), q1 = 2 * (j * NL + k1), q2 = 2 * ((j + 1) * NL + k1), q3 = 2 * ((j + 1) * NL + k);
      ctx.moveTo(P[q0], P[q0 + 1]); ctx.lineTo(P[q1], P[q1 + 1]); ctx.lineTo(P[q2], P[q2 + 1]); ctx.lineTo(P[q3], P[q3 + 1]); ctx.closePath(); };
    // one mid-shade underlay of every face in range (fills the AA seams between faces: no per-shade seam strokes),
    // then one fill per shade level: <= 9 fills + 1 outline per pass (was 8 fills + 8 strokes + 1)
    ctx.beginPath(); let anyF = false;
    for (let j = 0; j < NR; j++) for (let k = 0; k < NL; k++) { const f = j * NL + k; if (on(f)) { quad(j, k); anyF = true; } }
    if (!anyF) return;
    ctx.fillStyle = m.pal[3]; ctx.fill();
    for (let q = 0; q < 8; q++) {
      if (q === 3) continue;
      let any = false;
      ctx.beginPath();
      for (let j = 0; j < NR; j++) for (let k = 0; k < NL; k++) { const f = j * NL + k; if (m.lev[f] === q && on(f)) { quad(j, k); any = true; } }
      if (any) { ctx.fillStyle = m.pal[q]; ctx.fill(); }
    }
    ctx.beginPath();
    for (let j = 0; j < NR; j++) for (let k = 0; k < NL; k++) {
      const f = j * NL + k, k1 = (k + 1) % NL, fr = j * NL + k1, fu = (j + 1) * NL + k;
      // silhouette edges only above ground: an edge whose both ends sit flattened on the surface is the contact line (the
      // darker contact faces carry it), not an outline
      if (m.vis[f] !== m.vis[fr] && (on(f) || on(fr))) { const a = j * NL + k1, b = (j + 1) * NL + k1; if (!(G[a] && G[b])) { ctx.moveTo(P[2 * a], P[2 * a + 1]); ctx.lineTo(P[2 * b], P[2 * b + 1]); } }
      if (j < NR - 1 && m.vis[f] !== m.vis[fu] && (on(f) || on(fu))) { const a = (j + 1) * NL + k, b = (j + 1) * NL + k1; if (!(G[a] && G[b])) { ctx.moveTo(P[2 * a], P[2 * a + 1]); ctx.lineTo(P[2 * b], P[2 * b + 1]); } }
    }
    ctx.lineWidth = 0.025; ctx.strokeStyle = m.pal.edge; ctx.stroke();
  }

  /** Extruded polygon (flat xy array P, n verts) between depths za..zb along this._dz: front faces (mode & 1), near cap (mode & 2);
   *  mode & 4 skips the up-facing faces (something opaque always covers them, e.g. a bridge plank's logs). */
  _prism(P, n, za, zb, sh, mode = 3, edge = 'rgba(0,0,0,0.4)') {
    const ctx = this.ctx, Dx = this._dz[0], Dy = this._dz[1], ax = za * Dx, ay = za * Dy, bx = zb * Dx, by = zb * Dy;
    let area = 0;
    for (let i = 0; i < n; i++) { const j = (i + 1) % n; area += P[2 * i] * P[2 * j + 1] - P[2 * j] * P[2 * i + 1]; }
    const sg = area > 0 ? 1 : -1;
    ctx.lineWidth = 0.02; ctx.strokeStyle = edge;
    if (mode & 1) for (let i = 0; i < n; i++) {
      const j = (i + 1) % n, x0 = P[2 * i], y0 = P[2 * i + 1], x1 = P[2 * j], y1 = P[2 * j + 1], dx = x1 - x0, dy = y1 - y0;
      if (sg * (dx * Dy - dy * Dx) <= 0) continue;                   // faces away from the camera
      const l = Math.hypot(dx, dy) || 1, wy = -sg * dx / l, wx = sg * dy / l;
      const ny = this._prismRot ? wy * this._prismRot[0] - wx * this._prismRot[1] : wy, nx = this._prismRot ? wx * this._prismRot[0] + wy * this._prismRot[1] : wx;
      if (mode & 4 && ny > 0.55) continue;
      ctx.beginPath(); ctx.moveTo(x0 + ax, y0 + ay); ctx.lineTo(x1 + ax, y1 + ay); ctx.lineTo(x1 + bx, y1 + by); ctx.lineTo(x0 + bx, y0 + by); ctx.closePath();
      ctx.fillStyle = ny > 0.55 ? sh.top : nx < -0.55 ? sh.rear : sh.side; ctx.fill();
      ctx.beginPath(); ctx.moveTo(x0 + ax, y0 + ay); ctx.lineTo(x0 + bx, y0 + by); ctx.moveTo(x1 + ax, y1 + ay); ctx.lineTo(x1 + bx, y1 + by);
      ctx.stroke();                                                  // depth edges only: no seams at slab boundaries
    }
    if (mode & 2) {
      ctx.beginPath(); ctx.moveTo(P[0] + bx, P[1] + by);
      for (let i = 1; i < n; i++) ctx.lineTo(P[2 * i] + bx, P[2 * i + 1] + by);
      ctx.closePath(); ctx.fillStyle = sh.cap; ctx.fill(); ctx.stroke();
    }
  }

  /** local points -> this._P (world), rigid transform (x, y, a). */
  _xf(pts, x, y, a) {
    const P = this._P, c = Math.cos(a), s = Math.sin(a);
    for (let i = 0; i < pts.length; i++) { const p = pts[i]; P[2 * i] = x + c * p[0] - s * p[1]; P[2 * i + 1] = y + s * p[0] + c * p[1]; }
    return P;
  }

  /** Surface profile of run g at trail depth z into out[i] (visible vertices only): ground + berm + zone dy − rut. */
  _prof(g, z, out) {
    const T = this.trail, Z = this.dyZones, b = g.base, d = g.d;
    for (let i = g.i0; i <= g.i1; i++) {
      const p = b[i];
      out[i] = p[1] + laneOffset(T, Z, p[0], p[1], z) - (d ? rutAt(T, d, i, z) : 0);
    }
    return out;
  }

  /** Slab bounds (world z, far -> near) for this frame. */
  _bounds(v, dm, xl, xr) {
    const T = this.trail, D = dm.D, e = T.edge, zs = this._zsBuf;
    zs.length = 0;
    zs.push(-D, -(e + T.bermW), -(e + T.bermW * 0.5), -(e + T.bermW * 0.3), -e, e, e + T.bermW * 0.3, e + T.bermW * 0.5, e + T.bermW, D);   // ±(e + 0.3 bermW): where zone paint stops (_zones3d / _slabDetail test slab centres against it)
    if (T.lipZ) for (const sd of [-1, 1]) for (const u of [-1, -0.5, 0, 0.5, 1]) { const z = sd * (T.lipZ + u * T.lipW); if (Math.abs(z) < D) zs.push(z); }   // edge-lip ridge
    if (this.level && this.level.bridges && this.level.bridges.length) zs.push(-dm.W, dm.W);   // deck edges: rail sort planes (_items3d)
    // truck bounds (body planes, tyre edges + planes) move with the rig. r15 #4 (parked, front wheels steered: the near
    // rear tyre's bottom lugs vanished into the dirt, back when rolling re-centred the wheels): the lugs reach dT below the
    // contact, so a terrain slab starting < 2·dT in front of a tyre plane (the steered front tyre's plane, a few cm ahead)
    // painted over them. Truck bounds inside a tyre's lug margin (chained across tyres) are dropped: that terrain draws
    // before the tyres (static bounds stay: terrain shape never changes)
    const c = this._T, B = dm.B * c.cy, tr = this.look ? this.look.tread : { depth: 0.05 }, TP = this._tpBuf || (this._tpBuf = []), DB = this._dbBuf || (this._dbBuf = []);
    TP.length = 0; DB.length = 0;
    DB.push(c.zc - B, c.zc + B);                                     // body plane between the two tyre groups
    for (const o of v.wheels) { const p = this._tyrePlane(v, o, dm); DB.push(o.z - dm.tw / 2, o.z + dm.tw / 2, p); TP.push(p, p + 2 * treadDepth(o.radius, tr) + 0.04); }
    for (const z of DB) {                                            // (chained: a dropped tyre plane keeps its own margin)
      let skip = false;
      for (let i = 0; i < TP.length; i += 2) if (z > TP[i] + 1e-9 && z <= TP[i + 1]) { skip = true; break; }
      if (!skip) zs.push(z);
    }
    for (const q of this.dyZones) if (q.x1 >= xl && q.x0 <= xr) {
      const lo = q.side === 'R' ? 0 : q.z0, hi = q.side === 'L' ? 0 : q.z1;
      for (const zz of [lo, hi]) if (Math.abs(zz) < D) zs.push(zz - T.zRamp / 2, zz + T.zRamp / 2);
    }
    for (const q of this.zZones) if (q.x1 >= xl && q.x0 <= xr) {     // painted lane zones (mud strip, gravel lane): their z
      const lo = q.side === 'R' ? 0 : q.z0, hi = q.side === 'L' ? 0 : q.z1; // edges must be bounds, else the strip they paint
      for (const zz of [lo, hi]) if (Math.abs(zz) < D) zs.push(zz);  // follows whichever slab centre the tyre planes leave
    }
    const nz = T.nz, used = this._laneUse;                           // rutted lanes in view: their centres become bounds
    used.fill(0);
    for (const g of this.ground) {
      if (!g.vis || !g.d) continue;
      for (let i = g.i0, b = g.i0 * nz; i <= g.i1; i++) for (let l = 0; l < nz; l++, b++) if (g.d[b] > 0.004) used[l] = 1;
    }
    for (let l = 0; l < nz; l++) if (used[l] || (used[l - 1] || used[l + 1])) zs.push(laneZ(T, l));
    zs.sort((a, b) => a - b);
    let k = 0;
    const W = dm.W;                                                  // deck edges stay exact: a bound within 1.5 cm yields to them
    for (let i = 0; i < zs.length; i++) {
      const z = clamp(zs[i], -D, D);
      if (!k || z - zs[k - 1] > 0.015) zs[k++] = z; else if (Math.abs(Math.abs(z) - W) < 1e-9) zs[k - 1] = z;
    }
    zs.length = k;
    return zs;
  }

  /** Depth plane a tyre is painted at: its near face as yawed + steered (half width·|cos h| + radius·|sin h|), so terrain /
   *  prop slabs inside its own footprint never paint over it (a yawed tyre in a sunken lane or perched on a ledge vanished
   *  into the nearer bank); far tyres stay behind the body plane, near ones wait for it. */
  _tyrePlane(v, o, dm) {
    const h = (v.yaw || 0) + (o.steer || 0), zn = o.z + dm.tw / 2 * Math.abs(Math.cos(h)) + o.radius * Math.abs(Math.sin(h));
    const zBody = Math.min(this._T.zc + dm.B * this._T.cy, dm.D);
    return Math.min(o.lz < 0 ? Math.min(zn, zBody) : Math.max(zn, zBody), dm.D);
  }

  _draw3d(L, v, dyn) {
    const dm = this.dims;
    this._truckFrame(v);
    const xl = -this._ox / this.ppm - dm.D * C30 - 1, xr = (this.w - this._ox) / this.ppm + dm.D * C30 + 1;
    for (const g of this.ground) {
      const p = g.base; let i0 = 0, i1 = p.length - 1;
      while (i0 < i1 && p[i0 + 1][0] < xl) i0++;
      while (i1 > i0 && p[i1 - 1][0] > xr) i1--;
      g.i0 = i0; g.i1 = i1; g.vis = !(p[i1][0] < xl || p[i0][0] > xr);
    }
    const zs = this._bounds(v, dm, xl, xr), nS = zs.length - 1;
    this._nFaces = 0;                                                // rising step faces recorded by _terrainSlab
    { // night headlight catch params (see _faceCatch). r15 ('moves too fast', 'reads as a shading glitch'): the beam origin
      // is the CHASSIS front (rigid, no per-frame suspension / hub bounce), heading + lateral centre are low-passed (~0.2 s),
      // strength eases; only where it is dark enough for low beams to matter (dusk dim, night full; overcast / day: none)
      const A = this.atmo, lamp = A && A.tint && this.shape && !this.showroom ? (!this.q.detail && (A.lamp ?? 0) < 1 ? 0 : A.lamp ?? 0) : 0;
      const C = this._catch || (this._catch = { on: false, fw: 0, zc: 0, ty: 0, I: 0, gy: 0, init: false });
      const It = clamp((lamp - CATCH.lamp0) / (1 - CATCH.lamp0), 0, 1) * (this._lampK || 0) * (!v.falling && Math.abs(v.roll || 0) < 0.15 ? 1 : 0);
      const dt = clamp(this._fdt || 0, 0, 0.05), e = dt > 0 ? 1 - Math.exp(-dt / CATCH.tau) : 1;
      const ca = Math.cos(v.angle || 0), sa = Math.sin(v.angle || 0), fx = dm.front, fy = dm.fly;
      const fw = v.x + ca * fx - sa * fy, gy = v.y + sa * fx + ca * fy, zc = v.z || 0, ty = Math.tan(v.yaw || 0);
      if (!C.init) { C.zc = zc; C.ty = ty; C.I = It; C.init = true; }
      C.fw = fw; C.gy = gy;                                           // position: rigid with the chassis (a lag would trail at speed)
      C.zc += (zc - C.zc) * e; C.ty += (ty - C.ty) * e; C.I += (It - C.I) * Math.min(1, e * 1.5);
      C.on = C.I >= 0.01;
    }
    for (const g of this.ground) if (g.vis) {                        // each run's profile once per bound (slabs share them)
      const pb = g.pb || (g.pb = []), n = g.base.length;
      for (let k = 0; k <= nS; k++) this._prof(g, zs[k], pb[k] || (pb[k] = new Float64Array(n)));
    }
    const items = this._items3d(L, v, dyn, xl, xr), lists = this._slabs || (this._slabs = []), bottom = L.bottom - 40, ctx = this.ctx;
    for (let k = 0; k < nS; k++) {                                   // each slab's items in draw order (sorted up front: prop redraws look ahead)
      const za = zs[k], zb = zs[k + 1], zlo = k ? za : -Infinity, zhi = k === nS - 1 ? Infinity : zb, list = lists[k] || (lists[k] = []);
      list.length = 0;
      for (const it of items) if (it.za === it.zb ? it.za >= zlo && it.za < zhi : it.zb > za && it.za < zb) list.push(it);
      list.sort((a, b) => cmpSlab(a, b, za));
    }
    // Props (rock polygons, planks, logs, boulders) rest partly below the bilinear slab tops (embedded logs, slab bases
    // under berms, bridge ends at an abutment), so a nearer terrain slab would slice them. Every slab redraws each prop
    // from its far end up to the slab's near bound, keeping it on top of every terrain ribbon it spans. A truck part
    // already drawn (tyre, body) is in front of the prop's depths behind it: those depth segments are redrawn with the
    // part's silhouette clipped out, so the prop never repaints over it, and is never sliced around it either.
    const done = this._done || (this._done = []);                    // truck parts drawn so far ({it, z: in front of depths < z})
    done.length = 0;
    // A prop redraw repaints the footprint of every earlier one (opaque base, depth range only grows), except inside
    // the silhouettes of truck parts drawn before it. So the near-end redraw is the only full one; an earlier redraw
    // only shows inside the silhouette of a part it may touch (padded test) drawn in the same slab (before the prop:
    // the band painted over the part; after it: the last redraw before the part) or in the next slab before the prop
    // (the last redraw before the part). Other redraws are skipped, the needed ones scissored to those parts' boxes
    // (it.ks parts, it.ka: drawn before the prop in their slab). Props away from the truck (most bridge planks, gorge
    // river, logs, rocks) draw once per frame.
    const parts = this._parts || (this._parts = []);
    parts.length = 0;
    for (const it of items) if (it.ord != null && it.hull) {
      let k = 0; while (k < nS - 1 && zs[k + 1] <= it.za) k++;
      it.slab = k; parts.push(it);
    }
    for (const it of items) if (it.prop) {
      const ks = it.ks || (it.ks = []), ka = it.ka || (it.ka = []); ks.length = 0; ka.length = 0;
      for (const t of parts) if (overlap(it, it.za - 0.4, it.zb + 0.4, t)) {
        const l = lists[t.slab], ip = l.indexOf(it);
        ks.push(t); ka.push(ip < 0 || ip > l.indexOf(t));
      }
    }
    const sc = this._sc || (this._sc = [0, 0, 0, 0]);
    for (let k = 0; k < nS; k++) {
      const za = zs[k], zb = zs[k + 1], last = k === nS - 1, zlo = k ? za : -Infinity, zhi = last ? Infinity : zb;
      this._z(0);
      if (!this.showroom) {                                           // showroom: the glass platform (drawn first) is the floor
        const f0 = this._nFaces;
        for (const g of this.runOrder) if (g.vis) {
          g.ya = g.pb[k]; g.yb = g.pb[k + 1];
          this._terrainSlab(g, g.ya, g.yb, za, zb, last, bottom, k === 0);
        }
        this._zones3d(L, za, zb, xl, xr, k === 0);
        if (this.q.detail) for (const g of this.runOrder) if (g.vis) this._slabDetail(g, g.ya, g.yb, za, zb);
        if (this._nFaces > f0) this._faceFx(f0, this._nFaces, za, zb);
        this._tracks3d(zlo, zhi, xl, xr);
        if (L.finish && L.finish.x > xl && L.finish.x < xr) this._finishStrip(L.finish.x, za, zb, dm.D);
      }
      const list = lists[k], n0 = done.length;
      for (const it of list) {
        const b = Math.min(it.zb, zb);
        if (it.prop) {
          // once-props (bridge span planks: nothing slices them) draw a single full-depth pass, at the first slab holding a
          // truck part they may touch (before the parts sorted after it; earlier-drawn parts are clipped out), else at their
          // near end. The parts drawn later sit on the deck and paint over it. Saves ~6 clipped redraws per plank under the truck.
          if (it.once) {
            if (it.drawn) continue;
            let k0 = Infinity; for (const t of it.ks) k0 = Math.min(k0, t.slab);
            if (k >= k0 || last || b >= it.zb) { it.drawn = true; this._propRedraw(it, it.zb, done); }
            continue;
          }
          if (last || b >= it.zb) { this._propRedraw(it, b, done); continue; }
          // first slab with a part to come that it may touch: one full-depth redraw now (scissored to all those parts), so
          // every later part already has the prop behind it; later slabs only add the band painted over parts drawn first
          let a = it.za, e = b;
          if (!it.early && scissor(it, k, sc, 'before')) { it.early = true; scissor(it, k, sc, 'all'); e = it.zb; }
          else if (scissor(it, k, sc, 'band')) a = za;
          else continue;
          sc[0] -= 0.05; sc[1] -= 0.05; sc[2] += 0.05; sc[3] += 0.05;
          ctx.save(); this._z(0); ctx.beginPath(); ctx.rect(sc[0], sc[1], sc[2] - sc[0], sc[3] - sc[1]); ctx.clip();
          this._cull = sc;                                           // prop draws may skip pieces outside the scissor
          this._propRedraw(it, e, done, a);
          this._cull = null;
          ctx.restore();
          continue;
        }
        this._z(0);
        it.draw(Math.max(it.za, za), b, it);
        if (it.ord != null && it.hull) done.push({ it, z: za });     // same slab: props drawn later keep this slab's piece
      }
      for (let i = n0; i < done.length; i++) done[i].z = zb;         // later slabs: start past this one
      this._z(0);
      this._drawFx(0, zlo, zhi);
    }
  }


  /** 2.5D slab-top surface detail between depths za..zb: rock block joints, mud sheen + puddles, water ripples + bank foam. */
  _slabDetail(g, ya, yb, za, zb) {
    const ctx = this.ctx, p = g.base, i0 = g.i0, i1 = g.i1, T = this.trail, zm = (za + zb) / 2, dz = zb - za;
    if (dz < 1e-4) return;
    const onTrail = Math.abs(zm) <= T.edge + 0.3 * T.bermW, Z = this.level.zones;
    const surfAt = (x, y, z) => {                                    // painted surface: first matching zone, else the run's
      if (Math.abs(z) <= T.edge + 0.3 * T.bermW) for (const q of Z) if (zoneAtZ(q, z) && x >= q.x0 && x <= q.x1 && y >= q.y0 && y <= q.y1) return q.surface;
      return g.surface;
    };
    // per segment: 0 plain 1 rock 2 wet 3 water 4 gravel -- run class + static zone candidates (setLevel), only zoneAtZ per slab
    const cls = this._cls || (this._cls = new Uint8Array(4096)), sc = g.segCls, szc = g.segZones;
    let any = 0, has = 0;                                            // has: bit per class present (passes with nothing to draw are skipped)
    for (let i = i0; i < i1 && i - i0 < 4096; i++) {
      let c = sc[i];
      const cand = onTrail && szc[i];
      if (cand) for (const q of cand) if (zoneAtZ(q, zm)) { c = surfCls(q.surface); break; }
      any |= cls[i - i0] = c; has |= 1 << c;
    }
    const P = (x, i, z, o) => {                                      // surface point at x in segment i, depth z -> screen (pre-transform), into o
      const t = (x - p[i][0]) / Math.max(1e-6, p[i + 1][0] - p[i][0]), u = (z - za) / dz;
      o[0] = x + z * C30; o[1] = lerp(lerp(ya[i], ya[i + 1], t), lerp(yb[i], yb[i + 1], t), u) - z * S30; return o;
    };
    const PA = this._pa, PB = this._pb;
    if (T.lipZ) for (const sd of [-1, 1]) {                         // edge lip crest strips: packed stones + a dark kerb line
      const zc = sd * T.lipZ, row = sd > 0 ? 7 : 3, hw = T.lipW * 0.25;
      if (zb <= zc - hw || za > zc + hw) continue;                   // world-anchored: each stone / the kerb drawn by the strip holding its z
      if (zc >= za && zc < zb) {
        ctx.beginPath();
        for (let i = i0; i < i1; i++) { const a = P(p[i][0], i, zc, PA); if (i === i0) ctx.moveTo(a[0], a[1]); else ctx.lineTo(a[0], a[1]); }
        { const a = P(p[i1][0], i1 - 1, zc, PA); ctx.lineTo(a[0], a[1]); }
        ctx.lineWidth = 0.05; ctx.strokeStyle = 'rgba(0,0,0,0.28)'; ctx.stroke();
      }
      for (const [col, k0] of LIP_DOTS) {
        ctx.fillStyle = col; ctx.beginPath();
        for (let i = i0; i < i1; i++) {
          const x0 = p[i][0], x1 = p[i + 1][0], n = Math.max(1, Math.round((x1 - x0) / 0.32));
          for (let j = 0; j < n; j++) {
            const z = zc + (h01(i * 17 + j, row + 5) - 0.5) * T.lipW * 0.5;
            if (z < za || z >= zb) continue;
            const hh = h01(i * 31 + j, row), x = x0 + (x1 - x0) * (j + hh) / n;
            const a = P(x, i, z, PA), r0 = 0.05 + 0.05 * h01(i * 7 + j, row + 2), r = k0 > 0 ? r0 * 0.45 : r0, y = a[1] + k0 * 0.02 * (k0 < 0 ? 1 : 1.5);
            hexDot(ctx, a[0] - (k0 > 0 ? r0 * 0.25 : 0), y, r, r * 0.6);
          }
        }
        ctx.fill();
      }
    }
    if (!any) return;
    if (has & 2) {
    ctx.beginPath();
    {                                                                // rock: block joints in fixed 0.6 m depth courses (world-anchored:
      const c0 = Math.floor((za + 5) / 0.6), c1 = Math.ceil((zb + 5) / 0.6);   // slab bounds that move with the truck never reshuffle them)
      for (let i = i0; i < i1; i++) {
        const x0 = p[i][0], x1 = p[i + 1][0];
        if (cls[i - i0] !== 1 || x1 - x0 < 0.2) continue;
        const n = Math.max(1, Math.round((x1 - x0) / 0.9));
        for (let c = c0; c < c1; c++) {
          const ca = Math.max(za, c * 0.6 - 5), cb = Math.min(zb, (c + 1) * 0.6 - 5);
          if (cb <= ca) continue;
          for (let j = 0; j < n; j++) {
            const x = x0 + (x1 - x0) * (j + 0.25 + 0.5 * h01(i * 13 + j, c)) / n, a = P(x, i, ca, PA), b = P(x, i, cb, PB);
            ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]);
          }
        }
      }
      ctx.lineWidth = 0.025; ctx.strokeStyle = 'rgba(0,0,0,0.2)'; ctx.stroke();
    }
    }
    if (has & 16) {                                                  // gravel / scree: loose stones (light + shadow dots), 3 per
      const c0 = Math.floor((za + 5) / 0.4), c1 = Math.ceil((zb + 5) / 0.4);   // fixed 0.4 m depth cell, drawn by the strip holding their z
      for (const [col, k0] of GRAVEL_DOTS) {
        ctx.fillStyle = col; ctx.beginPath();
        for (let i = i0; i < i1; i++) {
          if (cls[i - i0] !== 4) continue;
          for (let c = c0; c < c1; c++) for (let j = 0; j < 3; j++) {
            const z = (c + h01(i * 5 + j + k0, c * 3 + 1)) * 0.4 - 5;
            if (z < za || z >= zb) continue;
            const u = h01(i * 5 + j + k0, c), pt = P(lerp(p[i][0], p[i + 1][0], u), i, z, PA), r = 0.025 + 0.03 * h01(j + k0 + c * 7, i);
            hexDot(ctx, pt[0], pt[1] + 0.01, r * 1.3, r * 0.7);
          }
        }
        ctx.fill();
      }
    }
    {                                                                // wet: sheen streaks along the strip
      if (has & 4) {
      ctx.beginPath();
      for (let row = Math.ceil((za + 5) * 4 - 0.5); (row + 0.5) / 4 - 5 < zb; row++) {   // fixed 0.25 m rows (world-anchored)
        const zr = (row + 0.5) / 4 - 5;
        if (zr < za) continue;
        for (let i = i0; i < i1; i++) {
          if (cls[i - i0] !== 2 || h01(i, row) < 0.55) continue;
          const x0 = p[i][0], x1 = p[i + 1][0], u = h01(i, row + 7) * 0.6, a = P(lerp(x0, x1, u), i, zr, PA), b = P(lerp(x0, x1, Math.min(1, u + 0.4)), i, zr, PB);
          ctx.moveTo(a[0], a[1] + 0.006); ctx.lineTo(b[0], b[1] + 0.006);
        }
      }
      ctx.lineWidth = 0.03; ctx.strokeStyle = 'rgba(225,235,245,0.16)'; ctx.stroke();
      }
      // puddle = the surface it lies on (zone or segment) darkened, a hint of sky (bright sky-grey puddles read as white patches)
      const PM = this._pudM && this._pudA === this.atmoName ? this._pudM : (this._pudA = this.atmoName, this._pudM = new Map());
      const pudCol = sn => { let c = PM.get(sn); if (!c) PM.set(sn, c = mix(mixHex(this._surfColor(sn), '#000000', 0.38), this.atmo.sky[1], 0.22)); return c; };
      for (const q of onTrail ? this.puddles : []) {
        if (q.x + q.rx < p[i0][0] || q.x - q.rx > p[i1][0] || q.z + q.rz < za || q.z - q.rz > zb) continue;
        const sn = surfAt(q.x, this._gy(q.x), q.z);
        if (!isWet(sn)) continue;                                    // a rock lane zone here: no puddle
        const pc = pudCol(sn);
        ctx.beginPath(); let k = 0;
        for (let m = 0; m < 16; m++) {
          const t = m * TAU / 16, x = q.x + Math.cos(t) * q.rx * (1 + 0.15 * Math.sin(3 * t + q.ph)), z = clamp(q.z + Math.sin(t) * q.rz, za, zb);
          let i = i0; while (i < i1 - 1 && p[i + 1][0] < x) i++;
          if (x < p[i0][0] || x > p[i1][0]) continue;
          const pt = P(x, i, z, PA); k++ ? ctx.lineTo(pt[0], pt[1] + 0.005) : ctx.moveTo(pt[0], pt[1] + 0.005);
        }
        if (k < 3) continue;
        ctx.closePath(); ctx.fillStyle = pc; ctx.fill();
        if (q.z >= za && q.z < zb) {                                 // glint once, in the strip holding the centre
          let i = i0; while (i < i1 - 1 && p[i + 1][0] < q.x) i++;
          const a = P(q.x - q.rx * 0.4, i, q.z, PA), b = P(q.x + q.rx * 0.2, i, q.z, PB);
          ctx.beginPath(); ctx.moveTo(a[0], a[1] + 0.01); ctx.lineTo(b[0], b[1] + 0.01);
          ctx.lineWidth = 0.03; ctx.strokeStyle = 'rgba(255,255,255,0.22)'; ctx.stroke();
        }
      }
    }
    if (!isWater(g.surface)) return;
    const t = this.t;                                                // water: drifting ripples on fixed 1/3 m rows, foam on the banks
    ctx.beginPath();
    for (let row = Math.ceil((za + 5) * 3 - 0.5); (row + 0.5) / 3 - 5 < zb; row++) {
      const zr = (row + 0.5) / 3 - 5;
      if (zr < za) continue;
      for (let i = i0; i < i1; i++) {
        const x0 = p[i][0], x1 = p[i + 1][0];
        for (let k = 0; k < 2; k++) {
          const u = (h01(i * 3 + k, row) + t * 0.06 * (k + 1)) % 1, x = lerp(x0, x1, u), a = P(Math.max(x0, x - 0.18), i, zr, PA), b = P(Math.min(x1, x + 0.18), i, zr, PB);
          ctx.moveTo(a[0], a[1] + 0.01); ctx.quadraticCurveTo((a[0] + b[0]) / 2, (a[1] + b[1]) / 2 + 0.035, b[0], b[1] + 0.01);
        }
      }
    }
    ctx.lineWidth = 0.028; ctx.strokeStyle = 'rgba(225,245,250,0.55)'; ctx.stroke();
    ctx.fillStyle = 'rgba(240,250,250,0.8)';                         // bank foam: both banks in one fill (they never overlap)
    const n = p.length - 1;
    ctx.beginPath();
    for (let e = 0; e < 2; e++) {
      const i = e ? n - 1 : 0, x = e ? p[n][0] - 0.12 : p[0][0] + 0.12;
      if (i < i0 || i >= i1) continue;
      for (let row = Math.ceil((za + 5) / 0.3 - 0.5); (row + 0.5) * 0.3 - 5 < zb; row++) {   // fixed 0.3 m rows
        const zr = (row + 0.5) * 0.3 - 5;
        if (zr < za) continue;
        for (let k = 0; k < 3; k++) {
          const pt = P(x + (k - 1) * 0.12, i, zr, PA), r = Math.max(0.02, 0.05 + 0.04 * Math.sin(t * 3 + k * 2 + zr * 5));
          ctx.moveTo(pt[0] + r, pt[1] + 0.02); ctx.arc(pt[0], pt[1] + 0.02, r, 0, TAU);
        }
      }
    }
    ctx.fill();
  }

  /** 2D river in a gorge under a bridge (drawn before the ground, which hides what lies below the terrain). */
  _river2d(r) {
    const ctx = this.ctx;
    ctx.fillStyle = RIVER; ctx.fillRect(r.x0, r.y - 6, r.x1 - r.x0, 6);
    ctx.fillStyle = '#235d6b'; ctx.fillRect(r.x0, r.y - 6, r.x1 - r.x0, 5.7);
    this._ripples(r, 0, 0.1, 0);
  }
  /** River surface ripples + shore foam, rows at trail depths z = z0 + 0.45 k inside [a, b) (2D: one row at z 0):
   *  every row in one batched stroke + one foam fill, offsets applied per point (current transform = _z(0) or 2D). */
  _ripples(r, a, b, z0) {
    const ctx = this.ctx, t = this.t;
    ctx.beginPath();
    const q = this._cull;
    for (let k = 0, z = z0; z < b; z += 0.45, k++) {
      if (z < a) continue;
      const ox = z * C30, y = r.y - z * S30;
      if (q && (y + 0.1 < q[1] || y - 0.1 > q[3])) continue;
      for (let x = r.x0 + 0.2; x < r.x1 - 0.2; x += 0.55) {
        const u = (h01(Math.floor(x * 3), k) * 0.4 + t * 0.35) % 0.55, xx = Math.min(r.x1 - 0.2, x + u) + ox;
        ctx.moveTo(xx - 0.16, y); ctx.quadraticCurveTo(xx, y + 0.04, xx + 0.16, y);
      }
    }
    ctx.lineWidth = 0.03; ctx.strokeStyle = '#a9d5da'; ctx.stroke();
    ctx.fillStyle = '#eef7f6'; ctx.beginPath();                      // shore foam
    for (let k = 0, z = z0; z < b; z += 0.45, k++) {
      if (z < a) continue;
      const ox = z * C30, y = r.y - z * S30 + 0.02;
      if (q && (y + 0.1 < q[1] || y - 0.1 > q[3])) continue;
      for (let e = 0; e < 2; e++) for (let j = 0; j < 3; j++) {
        const xx = (e ? r.x1 - 0.1 : r.x0 + 0.1) + (j - 1) * 0.1 + ox, rr = 0.05 + 0.03 * Math.abs(Math.sin(t * 2.5 + j + z));
        ctx.moveTo(xx + rr, y); ctx.arc(xx, y, rr, 0, TAU);
      }
    }
    ctx.fill();
  }

  /** Prop redraw za..b in depth segments split at the drawn truck parts it overlaps; each segment clips out the
   *  silhouettes of the parts in front of it (part z >= segment end). cut is in draw order (z ascending), so segment k
   *  lies behind parts k..n-1: the clips are pushed back to front, one save each (same-z parts with disjoint boxes
   *  share one evenodd path), then unwound while drawing far -> near, so each hull is clipped once, not once per
   *  segment. Clip rect = the prop's padded screen box (mask cost ~ its area). */
  _propRedraw(it, b, done, a0 = it.za) {
    const ctx = this.ctx, cut = this._cut || (this._cut = []), lo = this._cutLo || (this._cutLo = []), bxs = this._cutBx || (this._cutBx = []);
    cut.length = 0;
    for (const d of done) if (d.z > a0 && d.z < b && overlap(it, a0, b, d.it)) cut.push(d);
    const n = cut.length;
    this._z(0);
    if (n) {
      const m = 0.6, bx0 = it.x0 + a0 * C30 - m, bx1 = it.x1 + b * C30 + m, by0 = it.y0 - b * S30 - m, by1 = it.y1 - a0 * S30 + m;
      lo.length = 0; bxs.length = 0;
      let cur = -1;                                                  // lowest cut index in the open clip path
      for (let j = n - 1; j >= 0; j--) {
        const H = cut[j].it.hull, hb = cut[j].it.hb;
        if (cur >= 0) {
          let sep = cut[j + 1].z - cut[j].z >= 1e-6;                 // segment j+1 is drawn: needs its own clip state
          if (!sep) for (const q of bxs) if (hb[0] < q[2] && hb[2] > q[0] && hb[1] < q[3] && hb[3] > q[1]) { sep = true; break; }
          if (sep) { ctx.clip('evenodd'); lo[cur] = 1; cur = -1; bxs.length = 0; }
        }
        if (cur < 0) { ctx.save(); ctx.beginPath(); ctx.rect(bx0, by0, bx1 - bx0, by1 - by0); }
        ctx.moveTo(H[0], H[1]); for (let i = 2; i < H.length; i += 2) ctx.lineTo(H[i], H[i + 1]); ctx.closePath();
        bxs.push(hb); cur = j;
      }
      ctx.clip('evenodd'); lo[cur] = 1;
    }
    let s0 = a0;
    for (let k = 0; k <= n; k++) {
      const s1 = k < n ? cut[k].z : b;
      if (k === n || s1 - s0 >= 1e-6) {
        this._z(0);
        it.mid = k < n;                                              // inner segment: no near cap (the next one covers it)
        it.draw(s0, Math.max(s0, s1), it);
        it.mid = false;
        s0 = s1;
      }
      if (k < n && lo[k]) ctx.restore();
    }
  }

  /** One terrain slab of a ground run: top faces bilinear between profiles ya (depth za) and yb (zb), churned rut floor, near wall. */
  _terrainSlab(g, ya, yb, za, zb, last, bottom, farEdge) {
    const ctx = this.ctx, i0 = g.i0, i1 = g.i1, p = g.base, ax = za * C30, ay = -za * S30, bx = zb * C30, by = -zb * S30, dz = zb - za;
    // opaque top faces reach ~1 px back under the farther slab: adjacent strips would otherwise leave antialiased
    // hairline seams (the farther content showing through) at every slab bound
    const ov = farEdge ? 0 : 1.2 / this.ppm, fx = (za - ov) * C30, fy = -(za - ov) * S30;
    const quad = (i, qx = ax, qy = ay) => {
      const x0 = p[i][0], x1 = p[i + 1][0];
      ctx.moveTo(x0 + qx, ya[i] + qy); ctx.lineTo(x1 + qx, ya[i + 1] + qy); ctx.lineTo(x1 + bx, yb[i + 1] + by); ctx.lineTo(x0 + bx, yb[i] + by); ctx.closePath();
    };
    // visible top faces far (+x) -> near, batched per shade run: 0 flat, 1 riser, 2 cross wall facing the camera (drops
    // toward +z: far rut wall, far berm), 3 cross wall facing away (rises toward +z: near rut wall, near berm)
    const cols = [g.top, g.riser, g.lit, g.riserL], F = this._faces || (this._faces = []), nf0 = this._nFaces | 0;
    let cls = -1, nf = nf0;
    ctx.beginPath();
    for (let i = i1 - 1; i >= i0; i--) {
      const x0 = p[i][0], x1 = p[i + 1][0], dx = x1 - x0;
      // screen-space winding of the quad: visible when it turns clockwise (y-up world frame)
      const Ax = x0 + ax, Ay = ya[i] + ay, Bx = x1 + ax, By = ya[i + 1] + ay, Cx = x1 + bx, Cy = yb[i + 1] + by, Dx = x0 + bx, Dy = yb[i] + by;
      if ((Cx - Ax) * (Dy - By) - (Cy - Ay) * (Dx - Bx) >= 0) continue;
      // riser = steep authored segment: decided once per segment (g.rise), never per slab, so a step that sits on the
      // threshold cannot flicker between shades from one depth slab to the next
      const sz = ((yb[i] + yb[i + 1]) - (ya[i] + ya[i + 1])) * 0.5 / dz;
      // cross walls only where the trail truly steps (shelves, deep rut walls: slope > 0.6). Berm and lip slopes (≤ ~0.55 per
      // fixed slab) stay plain: tyre planes splitting them would flip a thin strip's shade = a line that follows the truck
      // + zone step faces (shelves, steps, blocks: their faces come from laneOffset, not authored segments): same shade.
      // Rising faces (the ones facing the camera / the truck) also get a crest highlight + foot shadow line below and are
      // recorded for the headlight catch (_faceCatch). r13: rock steps read as flat ground, worst at night.
      const rise = ((ya[i + 1] - ya[i]) + (yb[i + 1] - yb[i])) * 0.5, steep = rise > 0.1 && rise >= FACE.steep * dx;
      const c = g.rise[i] || steep ? 1 : sz < -0.6 ? 2 : sz > 0.6 ? 3 : 0;
      if (c !== cls) { if (cls >= 0) { ctx.fillStyle = cols[cls]; ctx.fill(); ctx.beginPath(); } cls = c; }
      quad(i, fx, fy);
      if (c === 1 && rise >= FACE.min && nf < FACE.max) {
        const r = F[nf] || (F[nf] = new Float64Array(9));
        r[0] = x0; r[1] = x1; r[2] = ya[i]; r[3] = ya[i + 1]; r[4] = yb[i]; r[5] = yb[i + 1]; r[6] = za; r[7] = zb;
        let rk = g.segCls[i] === 1;                                  // rock face (fractured ledge detail in _faceFx): rock run or rock zone
        if (!rk && g.segZones[i]) { const zm = (za + zb) / 2; for (const q of g.segZones[i]) if (zoneAtZ(q, zm)) { rk = surfCls(q.surface) === 1; break; } }
        r[8] = rk ? 1 : 0; nf++;
      }
    }
    if (cls >= 0) { ctx.fillStyle = cols[cls]; ctx.fill(); }
    this._nFaces = nf;                                               // lines + headlight catch: _faceFx, after the zone paint
    const d = g.d;
    if (d) {                                                         // churned rut floor: darker with depth (slab centre)
      // sampled mid-way between the two rut-lane centres around the slab (those are bounds): sub-slabs cut by tyre planes
      // share one shade instead of re-banding as the truck moves
      const T = this.trail, zm = laneZ(T, Math.floor(((za + zb) / 2) / T.lane + T.nz / 2 - 0.5)) + T.lane / 2;
      let any = false;
      for (let i = i0; i <= i1 && !any; i++) any = rutAt(T, d, i, zm) > 0.005;
      if (any) {
        ctx.fillStyle = g.churn;
        for (const [lo, hi, al] of [[0.005, 0.05, 0.35], [0.05, 0.15, 0.6], [0.15, 9, 0.85]]) {
          ctx.beginPath(); let n = 0;
          for (let i = i0; i < i1; i++) {
            const m = Math.max(rutAt(T, d, i, zm), rutAt(T, d, i + 1, zm));
            if (m < lo || m >= hi || p[i + 1][0] - p[i][0] < 1e-6) continue;
            quad(i); n++;
          }
          if (n) { ctx.globalAlpha = al; ctx.fill(); }
        }
        ctx.globalAlpha = 1;
        ctx.beginPath();                                             // wet sheen along the rut floors: on fixed lane centres (world-anchored)
        for (let l = 0; l < T.nz; l++) {
          const zl = laneZ(T, l);
          if (zl < za || zl >= zb) continue;
          const ox = zl * C30, oy = -zl * S30, u = (zl - za) / dz;
          for (let i = i0; i < i1; i++) {
            if (Math.min(rutAt(T, d, i, zl), rutAt(T, d, i + 1, zl)) < 0.04) continue;
            ctx.moveTo(p[i][0] + ox, lerp(ya[i], yb[i], u) + oy + 0.008); ctx.lineTo(p[i + 1][0] + ox, lerp(ya[i + 1], yb[i + 1], u) + oy + 0.008);
          }
        }
        ctx.lineWidth = 0.035; ctx.strokeStyle = 'rgba(210,225,235,0.28)'; ctx.stroke();
      }
    }
    if (farEdge) {                                                   // horizon lip of the ribbon
      ctx.beginPath(); ctx.moveTo(p[i0][0] + ax, ya[i0] + ay);
      for (let i = i0 + 1; i <= i1; i++) ctx.lineTo(p[i][0] + ax, ya[i] + ay);
      ctx.lineWidth = 0.05; ctx.strokeStyle = 'rgba(255,255,255,0.22)'; ctx.stroke();
    }
    if (!last) return;
    const path = dy => {
      ctx.beginPath(); ctx.moveTo(p[i0][0] + bx, bottom + by);
      for (let i = i0; i <= i1; i++) ctx.lineTo(p[i][0] + bx, yb[i] + by + dy);
      ctx.lineTo(p[i1][0] + bx, bottom + by); ctx.closePath();
    };
    path(0); ctx.fillStyle = g.col || this._surfColor(g.surface); ctx.fill();   // near wall = the classic 2D ground look
    path(-0.45); ctx.fillStyle = 'rgba(0,0,0,0.22)'; ctx.fill();
    path(-2.2); ctx.fillStyle = 'rgba(0,0,0,0.18)'; ctx.fill();
    const tp = this._tp || (this._tp = [0, 0]);
    this._wallDetail(g.surface, i0, i1, i => { tp[0] = p[i][0] + bx; tp[1] = yb[i] + by; return tp; });
    ctx.beginPath(); ctx.moveTo(p[i0][0] + bx, yb[i0] + by);
    for (let i = i0 + 1; i <= i1; i++) ctx.lineTo(p[i][0] + bx, yb[i] + by);
    ctx.lineWidth = 0.07; ctx.strokeStyle = g.surface === 'mud' ? '#2c2018' : 'rgba(255,255,255,0.28)'; ctx.stroke();
  }

  /** Surface zones painted on the slab tops whose centre lies in the zone's z range (side L = z < 0, R = z >= 0, z0 / z1). */
  _zones3d(L, za, zb, xl, xr, farEdge) {
    const ctx = this.ctx, T = this.trail, zm = (za + zb) / 2, bx = zb * C30, by = -zb * S30;
    const ov = farEdge ? 0 : 1.2 / this.ppm, ax = (za - ov) * C30, ay = -(za - ov) * S30;   // same seam overlap as the tops
    if (Math.abs(zm) > T.edge + 0.3 * T.bermW) return;              // the berms are their own surface
    for (const z of L.zones) {
      if (!zoneAtZ(z, zm)) continue;
      const x0 = Math.max(z.x0, xl), x1 = Math.min(z.x1, xr);
      if (x1 <= x0) continue;
      for (const g of this.ground) {
        if (!g.vis) continue;
        ctx.beginPath(); let any = false, nSteep = 0;
        const p = g.base, ya = g.ya, yb = g.yb;                      // this slab's cached profiles (_draw3d)
        const st = this._zSteep || (this._zSteep = []);
        for (let i = g.i0; i < g.i1; i++) {
          const a = p[i], b = p[i + 1], dx = b[0] - a[0];
          if (dx < 1e-6 || b[0] <= x0 || a[0] >= x1 || C30 * (b[1] - a[1]) + S30 * dx <= 0) continue;
          if ((a[1] + b[1]) / 2 < z.y0 || (a[1] + b[1]) / 2 > z.y1) continue;
          const u0 = Math.max(0, (x0 - a[0]) / dx), u1 = Math.min(1, (x1 - a[0]) / dx);
          const px0 = lerp(a[0], b[0], u0), px1 = lerp(a[0], b[0], u1);
          const rise = ((lerp(ya[i], ya[i + 1], u1) - lerp(ya[i], ya[i + 1], u0)) + (lerp(yb[i], yb[i + 1], u1) - lerp(yb[i], yb[i + 1], u0))) * 0.5;
          if (rise > 0.1 && rise >= FACE.steep * (px1 - px0)) { st[nSteep++] = i; continue; }   // step face: own (riser) shade below
          ctx.moveTo(px0 + ax, lerp(ya[i], ya[i + 1], u0) + ay); ctx.lineTo(px1 + ax, lerp(ya[i], ya[i + 1], u1) + ay);
          ctx.lineTo(px1 + bx, lerp(yb[i], yb[i + 1], u1) + by); ctx.lineTo(px0 + bx, lerp(yb[i], yb[i + 1], u0) + by); ctx.closePath();
          any = true;
        }
        if (!any && !nSteep) continue;
        // opaque premix of the old 85 % overlay on the run's top shade: alpha strips would seam at every slab bound
        let zc = this._zcol.get(z); if (!zc) this._zcol.set(z, zc = new Map());   // object-keyed: no per-frame string keys
        let col = zc.get(g);
        // r15 'stone shade changing': lifted like the run tops (g.top = colour + 12 % white). The raw surface colour made a
        // rock shelf on a rock run a flat dark parallelogram floating on the hill (read as a moving light / shade glitch);
        // the shelf now reads through its faces (riser shade, crest + fracture detail in _faceFx), like every other step
        // (all hex: mix() returns rgb() strings and silently reads non-hex input as grey 120 -- the old mix(colour, g.top)
        // and the riser mix of that result were the flat grey 'pipe / beam' faces of rock shelves)
        if (!col) {
          const gt = /^#/.test(g.top) ? g.top : mixHex(this._surfColor(g.surface), '#ffffff', 0.12);
          zc.set(g, col = mixHex(mixHex(this._surfColor(z.surface), '#ffffff', 0.12), gt, 0.15));
        }
        if (any) { ctx.fillStyle = col; ctx.fill(); }
        if (nSteep) {                                                // step faces (r13: were repainted in the flat zone colour)
          const RC = this._zRiser || (this._zRiser = new Map()); let cr = RC.get(col); if (!cr) RC.set(col, cr = mixHex(col, '#000000', 0.24));
          ctx.beginPath();
          for (let j = 0; j < nSteep; j++) {
            const i = st[j], a = p[i], b = p[i + 1], dx = b[0] - a[0];
            const u0 = Math.max(0, (x0 - a[0]) / dx), u1 = Math.min(1, (x1 - a[0]) / dx), px0 = lerp(a[0], b[0], u0), px1 = lerp(a[0], b[0], u1);
            ctx.moveTo(px0 + ax, lerp(ya[i], ya[i + 1], u0) + ay); ctx.lineTo(px1 + ax, lerp(ya[i], ya[i + 1], u1) + ay);
            ctx.lineTo(px1 + bx, lerp(yb[i], yb[i + 1], u1) + by); ctx.lineTo(px0 + bx, lerp(yb[i], yb[i + 1], u0) + by); ctx.closePath();
          }
          ctx.fillStyle = cr; ctx.fill();
        }
      }
    }
  }

  /** Skid marks whose trail z lies in [z0, z1). */
  _tracks3d(z0, z1, xl, xr) {
    const ctx = this.ctx, T = this.tracks, D = this.dims;
    ctx.fillStyle = '#140e0a';
    for (let j = 0; j < TRACK_MAX; j++) {
      const age = this.t - T.born[j], zc = T.z[j];
      if (zc < z0 || zc >= z1 || age > TRACK_LIFE || age < 0 || T.x1[j] < xl || T.x0[j] > xr) continue;
      const hw = D.tw * (T.w[j] > 0.1 ? 0.45 : 0.36), a = zc - hw, b = zc + hw;
      ctx.globalAlpha = T.a[j] * (1 - age / TRACK_LIFE) * 0.8;
      ctx.beginPath();
      ctx.moveTo(T.x0[j] + a * C30, T.y0[j] + 0.01 - a * S30); ctx.lineTo(T.x1[j] + a * C30, T.y1[j] + 0.01 - a * S30);
      ctx.lineTo(T.x1[j] + b * C30, T.y1[j] + 0.01 - b * S30); ctx.lineTo(T.x0[j] + b * C30, T.y0[j] + 0.01 - b * S30);
      ctx.closePath(); ctx.fill();
    }
    ctx.globalAlpha = 1;
  }

  _finishStrip(fx, za, zb, D) {
    const ctx = this.ctx, cell = 0.3;
    for (let i = 0; i < 2; i++) {
      const x0 = fx + i * cell, x1 = x0 + cell, y0 = this._gy(x0) + 0.012, y1 = this._gy(x1) + 0.012;
      for (let c = Math.floor((za + D) / cell); c * cell - D < zb; c++) {
        const a = Math.max(za, c * cell - D), b = Math.min(zb, (c + 1) * cell - D);
        if (b <= a) continue;
        const oa = laneOffset(this.trail, this.dyZones, x0, y0, a), ob = laneOffset(this.trail, this.dyZones, x0, y0, b);
        ctx.fillStyle = (i + c) % 2 ? '#111' : '#f2f2f2';
        ctx.beginPath(); ctx.moveTo(x0 + a * C30, y0 + oa - a * S30); ctx.lineTo(x1 + a * C30, y1 + oa - a * S30);
        ctx.lineTo(x1 + b * C30, y1 + ob - b * S30); ctx.lineTo(x0 + b * C30, y0 + ob - b * S30); ctx.closePath(); ctx.fill();
      }
    }
  }

  /** Everything that is not terrain, as depth-ranged items {za, zb, x0, x1, ym, ord?, draw(z0, z1, it)} (za === zb: a plane). */
  _items3d(L, v, dyn, xl, xr) {
    const dm = this.dims, items = [], ctx = this.ctx, defs = L.logDefs || [], T = this.trail;
    const add = (za, zb, x0, x1, ym, draw, ord, prop = false, y0 = ym - 1, y1 = ym + 1) => {
      if (x1 < xl || x0 > xr) return null;
      const it = { za, zb, x0, x1, ym, y0, y1, draw, ord, prop };
      items.push(it); return it;
    };
    // level rock polygons: full ribbon width
    L.polygons.forEach((q, i) => {
      const b = this.polyBox[i], P = this._P, n = q.points.length;
      add(-dm.D, dm.D, b.x0, b.x1, b.ym, (a, z1, it) => { for (let j = 0; j < n; j++) { P[2 * j] = q.points[j][0]; P[2 * j + 1] = q.points[j][1]; }
        const sh = this._shades(this._surfColor(q.surface)); this._prism(P, n, a, z1, sh, it.mid ? 1 : 3, sh.edge); }, undefined, true, b.y0, b.y1);
    });
    // gorge rivers: water in the gorge floor flowing across the trail, sliced per slab like terrain (not a prop), so the
    // nearer banks hide its edges and the truck / deck sort over it; the near end shows a darker water face down to the floor
    for (const r of this.rivers) add(-dm.D, dm.D, r.x0, r.x1, r.y - 0.3, (a, b, it) => {
      ctx.fillStyle = RIVER; ctx.beginPath();
      ctx.moveTo(r.x0 + a * C30, r.y - a * S30); ctx.lineTo(r.x1 + a * C30, r.y - a * S30); ctx.lineTo(r.x1 + b * C30, r.y - b * S30); ctx.lineTo(r.x0 + b * C30, r.y - b * S30);
      ctx.closePath(); ctx.fill();
      this._ripples(r, a, b, -dm.D + 0.3);
      if (b >= it.zb - 1e-9) {
        const X = b * C30, Y = -b * S30;
        ctx.fillStyle = '#235d6b'; ctx.beginPath();
        ctx.moveTo(r.x0 + X, r.y + Y); ctx.lineTo(r.x1 + X, r.y + Y); ctx.lineTo(r.x1 + X, r.lo + Y); ctx.lineTo(r.x0 + X, r.lo + Y);
        ctx.closePath(); ctx.fill();
      }
    }, undefined, false, r.y - 1, r.y + 0.1);
    // bridge planks: dark stringer plank + round logs laid across the trail (end grain on the near end)
    let tyreLow = Infinity;
    for (const o of v.wheels || []) tyreLow = Math.min(tyreLow, o.y - o.radius);
    const pk = this._plankFoot || (this._plankFoot = []), plk = this._plankIt || (this._plankIt = []);
    plk.length = 0;
    // deck joints: where two planks of a bridge meet, both use the midpoint of their touching top ends, so the deck is one
    // continuous polyline (logs evenly spaced along it): no plank end pokes above / tucks under its neighbour at the sag
    const PL = dyn.planks, JX = this._jx || (this._jx = []);
    JX.length = 0;
    for (let pi = 0; pi < PL.length; pi++) {
      const p = PL[pi], c = Math.cos(p.a), sn = Math.sin(p.a), w = p.hh * 0.2;
      JX.push(p.x - c * p.hw - sn * w, p.y - sn * p.hw + c * w, p.x + c * p.hw - sn * w, p.y + sn * p.hw + c * w);
    }
    for (let pi = 0; pi + 1 < PL.length; pi++) {
      if (PL[pi].b !== PL[pi + 1].b) continue;
      const o = 4 * pi, q = o + 4;
      if (Math.hypot(JX[o + 2] - JX[q], JX[o + 3] - JX[q + 1]) > 0.25) continue;   // snapped joint: free ends
      const mx = (JX[o + 2] + JX[q]) / 2, my = (JX[o + 3] + JX[q + 1]) / 2;
      JX[o + 2] = JX[q] = mx; JX[o + 3] = JX[q + 1] = my;
    }
    for (let pi = 0; pi < dyn.planks.length; pi++) {
      const p = dyn.planks[pi];
      const lr = Math.min(0.11, p.hw / 3);
      const n = Math.max(1, Math.round(p.hw / lr)), c = Math.cos(p.a), sn = Math.sin(p.a), ac = Math.abs(c), as = Math.abs(sn);
      const ey0 = as * p.hw + ac * p.hh + 0.05, ey1 = as * p.hw + ac * (0.2 * p.hh + 1.8 * lr) + 0.05;   // rotated plank + logs (x0 / x1 stay ±hw: draw order)
      const pl = add(-dm.W, dm.W, p.x - p.hw, p.x + p.hw, p.y, (a, b, it) => {
        const o = 4 * pi, ax0 = JX[o], ay0 = JX[o + 1], ax1 = JX[o + 2], ay1 = JX[o + 3], len = Math.hypot(ax1 - ax0, ay1 - ay0) || 1e-6;
        const tx = (ax1 - ax0) / len, ty = (ay1 - ay0) / len, nx = -ty, ny = tx, th = p.hh * 1.2;
        // stringer under the logs along the joint-to-joint segment (its top face is covered by the logs)
        const shP = this._plankSh || (this._plankSh = { ...this._shades('#3a2a1a'), rear: '#55391f' }), P = this._P;
        P[0] = ax0 - nx * th; P[1] = ay0 - ny * th; P[2] = ax1 - nx * th; P[3] = ay1 - ny * th; P[4] = ax1; P[5] = ay1; P[6] = ax0; P[7] = ay0;
        this._prism(P, 4, a, b, shP, (it.mid ? 1 : 3) | 4, shP.rear);   // edge = face shade: no joint lines
        ctx.lineCap = 'butt';
        // logs batched into few paths (each draw used to cost ~4 canvas calls per log): even logs, odd logs on top (the
        // alternate shade overlaps both neighbours), one bark-highlight path, then the near end grain discs + rings
        const near = b >= it.zb - 1e-9 && !it.mid, q = this._cull, L = this._logX || (this._logX = []);
        L.length = 0;
        for (let k = 0; k < n; k++) {
          const u = (k + 0.5) * len / n, x = ax0 + tx * u + nx * lr * 0.8, y = ay0 + ty * u + ny * lr * 0.8;
          const X0 = x + a * C30, Y0 = y - a * S30, X = x + b * C30, Y = y - b * S30;
          if (q && (X + lr < q[0] || X0 - lr > q[2] || Y0 + lr < q[1] || Y - lr > q[3])) continue;   // outside the scissor
          L.push(k, X0, Y0, X, Y);
        }
        if (L.length) {
          ctx.lineWidth = lr * 2.2;                                  // a touch fat: neighbours (and the next plank's end log) close up
          for (let odd = 0; odd < 2; odd++) {
            ctx.strokeStyle = odd ? '#55391f' : '#614426'; ctx.beginPath();
            for (let i = 0; i < L.length; i += 5) if ((L[i] & 1) === odd) { ctx.moveTo(L[i + 1], L[i + 2]); ctx.lineTo(L[i + 3], L[i + 4]); }
            ctx.stroke();
            if (a <= it.za + 1e-9) {                                 // rounded far ends: a scalloped far edge, not a sawtooth
              ctx.fillStyle = ctx.strokeStyle; ctx.beginPath();
              for (let i = 0; i < L.length; i += 5) if ((L[i] & 1) === odd) { ctx.moveTo(L[i + 1] + lr * 1.1, L[i + 2]); ctx.arc(L[i + 1], L[i + 2], lr * 1.1, 0, TAU); }
              ctx.fill();
            }
          }
          ctx.lineWidth = lr * 0.5; ctx.strokeStyle = '#6f5234'; ctx.beginPath();   // opaque: a plank redrawn per slab pass never stacks brighter
          for (let i = 0; i < L.length; i += 5) { ctx.moveTo(L[i + 1], L[i + 2] + lr * 0.45); ctx.lineTo(L[i + 3], L[i + 4] + lr * 0.45); }
          ctx.stroke();
          if (near) {
            ctx.fillStyle = '#8a6844'; ctx.beginPath();
            for (let i = 0; i < L.length; i += 5) { ctx.moveTo(L[i + 3] + lr * 0.95, L[i + 4]); ctx.arc(L[i + 3], L[i + 4], lr * 0.95, 0, TAU); }
            ctx.fill();
            ctx.lineWidth = 0.012; ctx.strokeStyle = '#4e3520'; ctx.beginPath();
            for (let i = 0; i < L.length; i += 5) { ctx.moveTo(L[i + 3] + lr * 0.55, L[i + 4]); ctx.arc(L[i + 3], L[i + 4], lr * 0.55, 0, TAU); }
            ctx.stroke();
          }
        }
        ctx.lineCap = 'round';
      }, undefined, true, p.y - ey0, p.y + ey1);
      // span plank: the gorge floor is well below both ends, so no terrain slab can slice it -> drawn once (see _draw3d)
      // and the truck rides on top of the deck (its tyres are never behind / under the span: a truck down in the gorge keeps
      // the full prop treatment, so the deck still covers it)
      // Terrain at depth z shows over deck depth z' > z when it sits (z' - z)·cos30 to the right and (z' - z)·sin30 lower, so
      // the ground must stay under the deck's lowest edge minus that slope across the whole depth sweep to the right.
      // (terrain is static and planks barely move: the clearance test is cached per plank until it shifts 5 cm)
      if (!pl) continue;
      let f = pk[pi];
      if (!f || Math.abs(f.x - p.x) > 0.05 || Math.abs(f.y - p.y) > 0.05 || f.W !== dm.W) {
        const x0 = p.x - p.hw, x1 = p.x + p.hw, sweep = 2 * dm.W * C30 + 0.5, yd = p.y - ey0 - GORGE;
        let ok = true;
        for (let x = x0; x <= x1 + sweep && ok; x += 0.25) ok = this.groundY(x) < yd - (S30 / C30) * Math.max(0, x - x1);
        f = pk[pi] = { x: p.x, y: p.y, W: dm.W, ok };
      }
      pl.once = f.ok && tyreLow > p.y - 0.25;
      {                                                              // swept footprint = the joint-to-joint deck segment it draws
        const o = 4 * pi, x0 = JX[o], y0 = JX[o + 1], x1 = JX[o + 2], y1 = JX[o + 3], len = Math.hypot(x1 - x0, y1 - y0) || 1e-6;
        const tx = (x1 - x0) / len, ty = (y1 - y0) / len, nx = -ty, ny = tx, lo = p.hh * 1.2 + 0.02, up = 2 * lr, e = 0.01;
        pl.foot = [[x0 - tx * e - nx * lo, y0 - ty * e - ny * lo], [x1 + tx * e - nx * lo, y1 + ty * e - ny * lo], [x1 + tx * e + nx * up, y1 + ty * e + ny * up], [x0 - tx * e + nx * up, y0 - ty * e + ny * up]];
      }
      plk.push(pl);
    }
    // A multi-pass plank overlapping a draw-once plank at lower x (cmpItem: lower x draws last = on top) would now paint
    // over it: its draws clip out that plank's swept screen footprint (z = 0 frame), keeping the original stacking.
    for (const P of plk) {
      if (P.once) continue;
      const hide = plk.filter(Q => Q.once && Q.x0 < P.x0 && Q.x1 > P.x0);
      if (!hide.length) continue;
      const draw = P.draw;
      P.draw = (a, b, it) => {
        ctx.save(); ctx.beginPath(); ctx.rect(it.x0 - 9, it.y0 - 9, it.x1 - it.x0 + 18, it.y1 - it.y0 + 18);
        for (const Q of hide) {
          const F = [];
          for (const [x, y] of Q.foot) F.push(x + Q.za * C30, y - Q.za * S30, x + Q.zb * C30, y - Q.zb * S30);
          hullPath(ctx, F);
        }
        ctx.clip('evenodd'); draw(a, b, it); ctx.restore();
      };
    }
    // bridge posts + sagging rope rails on both edges. Bank posts stand on the ground; the middle posts ride their plank
    // (the deck sags and bounces, the rails go with it). A rope span stretched far past its rest length has snapped
    // (collapsed bridge): not drawn; posts on planks that fell away go with them.
    (L.bridges || []).forEach((br, bi) => {
      const P = this._brPl || (this._brPl = []); P.length = 0;
      for (const p of dyn.planks) if (p.b === bi) P.push(p);
      const xa = br.from[0] + 0.15, xb = br.to[0] - 0.15, ya = br.from[1], yb = br.to[1];
      const nm = Math.max(1, Math.round((xb - xa) / 3.5)), sp = (xb - xa) / nm, posts = [[xa, ya]];
      // each middle post is bound ONCE to the plank under its rest station (index + local offset along it), then rides that
      // plank: re-picking the nearest plank every frame made posts jump a plank length as the deck shifted under the truck
      const PB = this._postBind || (this._postBind = new Map());
      let bind = PB.get(br);
      if (!bind || bind.n !== P.length) {
        bind = { n: P.length, k: [] };
        for (let k = 1; k < nm; k++) {
          const x = xa + k * sp; let qi = -1, d = Infinity;
          for (let i = 0; i < P.length; i++) { const e = Math.abs(P[i].x - x); if (e < d) { d = e; qi = i; } }
          if (qi >= 0) bind.k.push([qi, clamp((x - P[qi].x) / Math.max(1e-6, Math.cos(P[qi].a)), -P[qi].hw, P[qi].hw)]);
        }
        PB.set(br, bind);
      }
      for (const [qi, u] of bind.k) { const q = P[qi]; if (q) posts.push([q.x + Math.cos(q.a) * u, q.y + Math.sin(q.a) * u + q.hh * Math.cos(q.a) - 0.05]); }
      posts.push([xb, yb]);
      let lo = Infinity, hi = -Infinity; for (const p of posts) { lo = Math.min(lo, p[1]); hi = Math.max(hi, p[1]); }
      // Sort depth vs draw depth (r15 'pole glitch'): the rails stand 5 cm inside the deck edge (z = ±(W − 0.05)), but they
      // are SORTED as planes exactly on the deck edges ±W (always slab bounds, see _bounds). Near rail: every plank pass ends
      // by the slab that closes at +W, the rail draws in the one opening there -> always over the deck. Sorted at its own
      // 1.85, a truck tyre plane landing between 1.85 and 1.9 (wide Bogger, slight yaw) split the slab and the deck's
      // near-end pass painted over the whole near rail for those frames. Far rail: a plane on its slab's far bound
      // (atFar) -> always before the plank passes (the deck hides the post foot), never a comparator coin-flip.
      for (const side of [-1, 1]) {
        const z = side * (dm.W - 0.05), zs = side * dm.W;
        add(zs, zs, xa - 0.2, xb + 0.2, hi, () => {
          this._z(z);
          ctx.lineCap = 'round';
          // far rail: the deck hides the post's foot, so it starts at the deck top (its below-deck stub used to paint over the
          // end logs = a pole stabbing through the deck, playtest r12); near rail: the stub hangs down the deck's open side
          const foot = side < 0 ? 0.03 : 0.6;
          for (const [x, y] of posts) {
            ctx.lineWidth = 0.17; ctx.strokeStyle = '#4a3420'; ctx.beginPath(); ctx.moveTo(x, y - foot); ctx.lineTo(x, y + 1.0); ctx.stroke();
            ctx.fillStyle = '#8a6844'; ctx.beginPath(); ctx.arc(x, y + 1.0, 0.085, 0, TAU); ctx.fill();
          }
          ctx.lineWidth = 0.035; ctx.strokeStyle = '#b8966a'; ctx.beginPath();
          for (let k = 1; k < posts.length; k++) {
            const [x0, y0] = posts[k - 1], [x1, y1] = posts[k];
            if (Math.hypot(x1 - x0, y1 - y0) > sp * 1.35) continue;     // snapped span
            ctx.moveTo(x0, y0 + 0.85); ctx.quadraticCurveTo((x0 + x1) / 2, (y0 + y1) / 2 + 0.5, x1, y1 + 0.85);
          }
          ctx.stroke();
          this._z(0);
        }, undefined, false, lo - (side < 0 ? 0.05 : 0.6), hi + 1.1);
      }
    });
    // trailside flora: far edge (pines / bushes / ferns / grass), near edge (low tufts only)
    for (const f of this.flora) {
      if (f.x < xl - 3 || f.x > xr + 3) continue;
      if (!f.far && this.rivers.some(r => f.x > r.x0 - 0.3 && f.x < r.x1 + 0.3)) continue;   // no tufts growing out of a gorge river
      const z = f.far ? -dm.D + 0.08 + f.dz * 0.5 : dm.D - 0.12, y = this._surfY(f.x, z);
      if (!Number.isFinite(y)) continue;
      add(z, z, f.x - 1.2, f.x + 1.2, y, () => { this._z(z); this._plant(f, f.x, y - 0.02, f.far); this._z(0); });
    }
    // logs (cylinders across the trail) and loose boulders (extruded chunks)
    const LW = T.edge + T.bermW * 0.5;
    dyn.logs.forEach((l, i) => {
      const d = defs[i] || {}, part = d.z0 != null || d.z1 != null;   // partial width: spans [z0, z1] only (physics zs)
      if (/rock|stone/.test(d.surface || '')) {
        // partial boulders end at the edge lip (never hang out over the drop); full-width rock ridges span the trail
        const RW = T.lipZ ? T.lipZ - T.lipW * 0.4 : LW;
        const za = part ? Math.max(-RW, d.z0 ?? -RW) : -LW * 0.8, zb = part ? Math.min(RW, d.z1 ?? RW) : LW * 0.8;
        // boulders draw as rounded low-poly rock lumps (3D meshes, faceted + lit, bottoms seated on the drawn terrain so none
        // floats or sinks): a partial boulder = a big lump at its trail-side end + smaller ones toward the edge (a rock pile);
        // a full-width low ridge = a row of cobbles. Each lump is its own depth item; front faces of a convex lump never
        // overlap, so a pass only needs the faces whose centroid lies in its depth range.
        // full-width ridges (r15: a row of equal round cobbles read as a chain): a fractured ledge of irregular BLOCKS,
        // widths 0.55..1.45x the mean with thin fracture gaps, boxy + jagged meshes (_lumpMesh box), per-block x offset /
        // height jitter kept within the physics log (the top never rises above l.y + r: tyres still meet what they see)
        const span = zb - za, n = part ? Math.max(1, Math.round(span / (0.95 * l.r))) : Math.max(3, Math.round(span / (1.8 * l.r)));
        const inner = part ? (Math.abs(za) < Math.abs(zb) ? 0 : n - 1) : -1, WZ = this._ridgeW || (this._ridgeW = []);
        let wsum = 0;
        for (let c = 0; c < n; c++) { WZ[c] = part ? 1 : 0.55 + 0.9 * h01(i * 7 + c, 41); wsum += WZ[c]; }
        let z0 = za;
        for (let c = 0; c < n; c++) {
          const h = Math.sin(i * 12.9898 + c * 78.233) * 43758.5453, rnd = h - Math.floor(h), dz = span * WZ[c] / wsum;
          const sc = c === inner ? 1 : part ? 0.78 + 0.12 * rnd : 0.84 + 0.16 * rnd;
          const cz = z0 + dz / 2, rz = part ? Math.min(l.r * sc * 1.1, dz * 0.62) : dz * 0.5 - 0.025;
          z0 += dz;
          const it = add(cz - rz, cz + rz, l.x - l.r, l.x + l.r, l.y, (a, b, it) => this._rockLump(it.lump, a, b, it), undefined, true, l.y - l.r - 0.1, l.y + l.r);
          if (it) it.lump = this._lumpMesh(i * 31 + c, l, cz, l.r * sc, rz, d.surface, part ? 0 : 0.16 * l.r * (rnd - 0.5), !part);
        }
        return;
      }
      const za = part ? Math.max(-LW, d.z0 ?? -LW) : -LW, zb = part ? Math.min(LW, d.z1 ?? LW) : LW;
      add(za, zb, l.x - l.r, l.x + l.r, l.y, (a, b, it) => {
        ctx.lineCap = 'round'; ctx.lineWidth = l.r * 2; ctx.strokeStyle = '#6e5235';
        ctx.beginPath(); ctx.moveTo(l.x + a * C30, l.y - a * S30); ctx.lineTo(l.x + b * C30, l.y - b * S30); ctx.stroke();
        const ox = S30 * l.r * 0.45, oy = C30 * l.r * 0.45;           // bark highlight on the upper flank
        ctx.lineWidth = l.r * 0.45; ctx.strokeStyle = '#856949';   // opaque: redrawn per slab pass
        ctx.beginPath(); ctx.moveTo(l.x + a * C30 + ox, l.y - a * S30 + oy); ctx.lineTo(l.x + b * C30 + ox, l.y - b * S30 + oy); ctx.stroke();
        if (b >= it.zb - 1e-9) { this._z(b); this._logFace(l); }      // end grain on the near end only
      }, undefined, true, l.y - l.r, l.y + l.r);
    });
    // winch anchors: posts along the far edge
    const wn = v.winch;
    for (const a of L.anchors) add(dm.zA, dm.zA, a.x - 0.2, a.x + 0.2, a.y, () => {
      this._z(dm.zA);
      const gy = this.groundY(a.x) + laneOffset(T, this.dyZones, a.x, a.y, dm.zA);
      ctx.lineWidth = 0.22; ctx.strokeStyle = '#5b4026';
      ctx.beginPath(); ctx.moveTo(a.x, gy - 0.1); ctx.lineTo(a.x, a.y + 0.25); ctx.stroke();
      const hot = wn.attached && Math.abs(wn.ax - a.x) < 0.01 && Math.abs(wn.ay - a.y) < 0.01;
      ctx.lineWidth = 0.07; ctx.strokeStyle = hot ? '#ffd23c' : '#e8e2c8';
      ctx.beginPath(); ctx.arc(a.x, a.y, 0.14, 0, TAU); ctx.stroke();
    });
    // jerry cans (fuel pickups) standing in their lane until collected
    const cans = L.cans || [], taken = dyn.cans || [];
    for (let i = 0; i < cans.length; i++) if (!taken[i]) {
      const c = cans[i], gy0 = this.groundY(c.x), gy = gy0 + laneOffset(T, this.dyZones, c.x, gy0, c.z);
      add(c.z, c.z, c.x - 0.3, c.x + 0.3, gy + 0.2, () => { this._z(c.z); this._jerryCan(c.x, gy, i); this._z(0); });
    }
    // stage posts: wooden signs on the far edge
    for (const st of L.stages || []) add(dm.zA, dm.zA, st.x - 0.8, st.x + 0.8, this.groundY(st.x), () => {
      this._z(dm.zA);
      this._stagePost(st.x, this.groundY(st.x) + laneOffset(T, this.dyZones, st.x, this.groundY(st.x), dm.zA), st.name);
    });
    // finish gate: a pole on each edge
    if (L.finish) for (const z of [-dm.D + 0.25, dm.D - 0.25]) {
      const x = L.finish.x, gy = this.groundY(x) + T.bermH;
      add(z, z, x - 0.1, x + 0.9, gy, () => {
        this._z(z);
        ctx.lineWidth = 0.08; ctx.strokeStyle = '#f2f2f2';
        ctx.beginPath(); ctx.moveTo(x, gy); ctx.lineTo(x, gy + 2.4); ctx.stroke();
        for (let i = 0; i < 3; i++) for (let j = 0; j < 2; j++) { ctx.fillStyle = (i + j) % 2 ? '#111' : '#f2f2f2'; ctx.fillRect(x + i * 0.3, gy + 1.8 + j * 0.3, 0.3, 0.3); }
      });
    }
    if (!this.shape) return items;
    // -- truck (yawed truck frame, painter's order by local z): each tyre + its coil-over at the tyre's trail z,
    //    axles + body at the truck's z. Far tyre, far coil-over | axles, body | near coil-over, near tyre.
    const W = v.wheels, tf = this._T, zc = tf.zc;
    let bx0 = Infinity, bx1 = -Infinity;
    for (const s of this.shape.shapes) { const P = this._xf(s.points, v.x, v.y, v.angle); for (let i = 0; i < s.points.length; i++) { bx0 = Math.min(bx0, P[2 * i]); bx1 = Math.max(bx1, P[2 * i]); } }
    const sw = Math.abs(tf.sy) * (dm.h + dm.tw);
    bx0 -= sw; bx1 += sw;
    let ay = 0; for (const o of W) ay += o.y; ay /= W.length;
    const D0 = this._dz;
    // the body box spans zc ± B; it is drawn as one plane on its near face, so the terrain under it (centre / far
    // ribbon, a crest it is high-centred on) is all painted first. Near tyres yawed inside that face wait for the body.
    const zBody = Math.min(zc + dm.B * tf.cy, dm.D);
    // silhouette (z = 0 frame) of a truck part from pre-transform truck-frame points (see _tz(0)), + its box
    const e = tf.xc * (1 - tf.a) + tf.zc * C30, f = -tf.b * tf.xc - tf.zc * S30;
    const sil = (it, F) => {
      if (!it) return;
      for (let i = 0; i < F.length; i += 2) { const u = F[i], w = F[i + 1]; F[i] = tf.a * u + e; F[i + 1] = tf.b * u + w + f; }
      const H = it.hull = hull(F), hb = it.hb = [Infinity, Infinity, -Infinity, -Infinity];
      for (let i = 0; i < H.length; i += 2) { hb[0] = Math.min(hb[0], H[i]); hb[1] = Math.min(hb[1], H[i + 1]); hb[2] = Math.max(hb[2], H[i]); hb[3] = Math.max(hb[3], H[i + 1]); }
    };
    for (const o of W) {
      const far = o.lz < 0, zw = this._tyrePlane(v, o, dm);
      const it = add(zw, zw, o.x - o.radius, o.x + o.radius, o.y, () => {
        this._dz = [tf.Dx, tf.Dy];
        if (far) { this._wheel3(o, dm.tw, true); this._coil(o, -dm.sz); } else { this._coil(o, dm.sz); this._wheel3(o, dm.tw, false); }
        this._dz = D0; this._z(0);
      }, o.lz);
      sil(it, this._tyreOutline(o, dm.tw));
    }
    const body = add(zBody, zBody, bx0, bx1, Math.max(v.y, ay + 0.01), () => {
      this._dz = [tf.Dx, tf.Dy];
      this._tz(0);
      const a = -dm.h + dm.tw / 2, b = dm.h - dm.tw / 2;
      for (const ax of [0, 1]) {                                     // axle housings (+ diff pumpkin at the centre line)
        let x = 0, y = 0, n = 0; for (const o of W) if (o.axle === ax) { x += o.hx ?? o.x; y += o.hy ?? o.y; n++; }
        if (!n) continue; x /= n; y /= n;
        ctx.lineCap = 'butt'; ctx.lineWidth = 0.15; ctx.strokeStyle = '#2b2d31';
        ctx.beginPath(); ctx.moveTo(x + a * tf.Dx, y + a * tf.Dy); ctx.lineTo(x + b * tf.Dx, y + b * tf.Dy); ctx.stroke(); ctx.lineCap = 'round';
        ctx.fillStyle = '#3a3c41'; ctx.beginPath(); ctx.arc(x, y, 0.17, 0, TAU); ctx.fill();
      }
      const sr0 = tf.sr, cr0 = tf.cr, Dx0 = tf.Dx, Dy0 = tf.Dy;      // body roll: the sprung parts tilt, hubs / axles don't
      if (tf.br) { tf.sr = Math.sin(tf.br); tf.cr = Math.cos(tf.br); tf.Dx = Dx0; tf.Dy = Dy0; this._rollDz(tf); this._dz = [tf.Dx, tf.Dy]; }
      this._body3(v);                                                // (rolled depth vector: caps, rig decor and the window agree)
      tf.sr = sr0; tf.cr = cr0; tf.Dx = Dx0; tf.Dy = Dy0;
      this._dz = D0; this._z(0);
    }, 0);
    if (body) {                                                      // body: chassis panels at ±B, axle ends, stack tip
      const F = [], B = dm.B, c = Math.cos(v.angle), s2 = Math.sin(v.angle);
      for (const sh of this.shape.shapes) {
        const P = this._xf(sh.points, v.x, v.y, v.angle);
        for (let i = 0; i < sh.points.length; i++) for (const lz of [-B, B]) F.push(P[2 * i] + lz * tf.Dx, P[2 * i + 1] + lz * tf.Dy);
      }
      const st = this.stack;
      if (st) F.push(v.x + c * st.x - s2 * st.y1 + dm.zStack * tf.Dx, v.y + s2 * st.x + c * st.y1 + dm.zStack * tf.Dy);
      for (const p of this.rigPts) for (const lz of [-B * 1.2, B * 1.2]) F.push(v.x + c * p[0] - s2 * p[1] + lz * tf.Dx, v.y + s2 * p[0] + c * p[1] + lz * tf.Dy);
      sil(body, F);
    }
    if (wn.attached) {
      // drum: truck frame (lz 0) -> equivalent world point at the truck's z; rope runs z = zc (drum) -> zA (anchor)
      const sx = tf.a * wn.mx + tf.xc * (1 - tf.a) + zc * C30, sy = wn.my + tf.b * (wn.mx - tf.xc) - zc * S30;
      const mx = sx - zc * C30, my = sy + zc * S30, zA = dm.zA;
      const d = Math.hypot(wn.ax - mx, wn.ay - my), sag = Math.sqrt(Math.max(0, wn.length - d) * Math.max(d, 0.01)) * 0.45;
      const cx = (mx + wn.ax) / 2, cy = (my + wn.ay) / 2 - sag * 2, t = Math.min(1, wn.tension / wn.breakTension);
      add(Math.min(zA, zc), Math.max(zA, zc) + 0.001, Math.min(mx, wn.ax), Math.max(mx, wn.ax), Math.max(my, wn.ay), (a, b) => {
        const ua = clamp((a - zc) / (zA - zc), 0, 1), ub = clamp((b - zc) / (zA - zc), 0, 1), u0 = Math.min(ua, ub), u1 = Math.max(ua, ub);
        if (u1 <= u0) return;
        ctx.lineWidth = 0.045 + 0.03 * t;
        ctx.strokeStyle = `rgb(${Math.round(lerp(70, 235, t))},${Math.round(lerp(70, 50, t))},${Math.round(lerp(75, 40, t))})`;
        ctx.beginPath();
        this._ropePath(mx, my, cx, cy + this._ropeHum(t), wn.ax, wn.ay, u0, u1, zc, zA);
        ctx.stroke();
        if (u1 >= 1) { ctx.fillStyle = '#ffd23c'; ctx.beginPath(); ctx.arc(wn.ax + zA * C30, wn.ay - zA * S30, 0.08, 0, TAU); ctx.fill(); }
      });
    }
    return items;
  }

  /** Coil-over + link of one corner in the truck frame plane lz (frame mount -> hub). */
  _coil(o, lz) {
    const ctx = this.ctx, hx = o.hx ?? o.x, hy = o.hy ?? o.y;
    this._tz(lz);
    const q = this._coilTmp || (this._coilTmp = {});
    const dy = this._T.br ? -lz * Math.sin(this._T.br) : 0;         // body roll moves the frame-side mounts, not the hub
    q.x = hx; q.y = hy; q.mountX = o.mountX; q.mountY = o.mountY + dy;
    if (dy) { const L = this._linkTmp || (this._linkTmp = {}); L.linkX = o.linkX; L.linkY = o.linkY + dy; this._links(L, hx, hy, this._T.ang || 0); }
    else this._links(o, hx, hy, this._T.ang || 0);
    this._shock(q, o.damage || 0);
  }

  /** Pre-transform truck-frame outline points of a corner as _wheel3 + _coil draw it: both tread rims + the
   *  coil-over / link top (silhouette for clipping props, see _propRedraw). */
  _tyreOutline(o, tw) {
    const tf = this._T, r = o.radius + 0.06, st = o.steer || 0, cs = Math.cos(st), ss = Math.sin(st), F = [];
    const hx = (o.hx ?? o.x) + o.lz * tf.Dx, hy = (o.hy ?? o.y) + o.lz * tf.Dy;
    const Ax = -ss + cs * tf.Dx, Ay = cs * tf.Dy, fa = cs + ss * tf.Dx, fb = ss * tf.Dy;
    for (const w of [-tw / 2, tw / 2]) for (let k = 0; k < 16; k++) {
      const t = k * TAU / 16, u = Math.cos(t) * r, q = Math.sin(t) * r;
      F.push(hx + w * Ax + fa * u, hy + w * Ay + q + fb * u);
    }
    const lz = (o.lz < 0 ? -1 : 1) * this.dims.sz;
    F.push(o.mountX + lz * tf.Dx, o.mountY + lz * tf.Dy, o.linkX + lz * tf.Dx, o.linkY + lz * tf.Dy);
    return F;
  }

  /**
   * Tyre as a cylinder in the truck frame, turned by its steer angle: sheared tread discs along the axle with
   * rolling lugs, then the camera-facing sidewall. Heading of the tyre = yaw + steer.
   */
  _wheel3(o, tw, far) {
    const ctx = this.ctx, tf = this._T, r = o.radius, st = o.steer || 0, cs = Math.cos(st), ss = Math.sin(st);
    const hx = (o.hx ?? o.x) + o.lz * tf.Dx, hy = (o.hy ?? o.y) + o.lz * tf.Dy;   // hub centre (pre-transform)
    const Ax = -ss + cs * tf.Dx, Ay = cs * tf.Dy;                   // axle direction per m (pre-transform)
    const fa = cs + ss * tf.Dx, fb = ss * tf.Dy;                    // sidewall shear: (u, v) -> (fa·u, v + fb·u)
    const th = Math.atan2(tf.sy, tf.cy) + st, ct = Math.cos(th), sn = Math.sin(th);
    const out = C30 * sn + ct >= 0 ? 1 : -1;                        // which sidewall faces the camera
    this._tz(0);
    // tread lugs = solid blocks ON the tread (each: base angle ±d0 at the carcass, top ±d1 at r + depth, across the axle),
    // projected and filled as the hull of their corners: the half turned away is drawn first (only its knobs past the
    // carcass silhouette stay visible), the facing half after the carcass. A lug rolling over the edge just turns into
    // view -- nothing pops in. Angles match the sidewall lugs of _wheelFace, so the blocks join them.
    const tr = this.look ? this.look.tread : { lugs: 12, depth: 0.05, kind: 'lug' }, nL = tr.lugs, pad = tr.kind === 'paddle', blk = tr.kind === 'block';
    const dT = treadDepth(r, tr), lw = clamp(TAU * r / nL * (pad ? 0.62 : 0.5), 0.07, 0.26), vx = sn - C30 * ct;
    const a0 = -tw / 2 + 0.02, a1 = tw / 2 - 0.02, G = LUG_G;
    const lugs = side => {
      ctx.beginPath();
      for (let k = 0; k < nL; k++) {
        const t = o.angle + (k + 0.5) * TAU / nL, vis = Math.cos(t) * vx + S30 * Math.sin(t);
        if (side ? vis < 0 : vis >= 0) continue;
        const dd = blk && k & 1 ? dT * 0.65 : dT, R1 = r + dd, d0 = 0.58 * lw / r, d1 = 0.36 * lw / R1, f = pad ? 0.09 / R1 : 0;
        const segs = lugSegs(SEG_TMP, pad, blk, k, a0, a1, f), nSeg = pad ? 8 : 4;
        for (let sI = 0; sI < nSeg; sI += 4) {
          let gi = 0;
          for (let e = 0; e < 2; e++) {
            const w = segs[sI + e], off = segs[sI + 2 + e];
            for (let ri = 0; ri < 2; ri++) {
              const rr = ri ? R1 : r - 0.02, dd2 = ri ? d1 : d0;
              for (let sg = -1; sg <= 1; sg += 2) {
                const tt = t + off + sg * dd2, u = Math.cos(tt) * rr, q = Math.sin(tt) * rr;
                G[gi++] = hx + fa * u + w * Ax; G[gi++] = hy + q + fb * u + w * Ay;
              }
            }
          }
          hullPath(ctx, G, gi);
        }
      }
      ctx.fillStyle = far ? '#17181a' : '#26272b'; ctx.fill();
      if (!side) return;
      ctx.beginPath();                                               // lit lug tops (the faces turned most to the camera)
      for (let k = 0; k < nL; k++) {
        const t = o.angle + (k + 0.5) * TAU / nL, vis = Math.cos(t) * vx + S30 * Math.sin(t);
        if (vis < 0.25) continue;
        const dd = blk && k & 1 ? dT * 0.65 : dT, R1 = r + dd, d1 = 0.3 * lw / R1, f = pad ? 0.09 / R1 : 0;
        const segs = lugSegs(SEG_TMP, pad, blk, k, a0, a1, f), nSeg = pad ? 8 : 4;
        for (let sI = 0; sI < nSeg; sI += 4) {
          for (let c = 0; c < 4; c++) {                              // corners p0 p1 p2 p3: (w, off, sg)
            const e = c === 1 || c === 2 ? 1 : 0, w = segs[sI + e] + (e ? -0.02 : 0.02), off = segs[sI + 2 + e], sg = c < 2 ? -1 : 1;
            const tt = t + off + sg * d1, u = Math.cos(tt) * R1, q = Math.sin(tt) * R1, X = hx + fa * u + w * Ax, Y = hy + q + fb * u + w * Ay;
            c ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y);
          }
          ctx.closePath();
        }
      }
      ctx.fillStyle = far ? '#1d1e21' : '#33353a'; ctx.fill();
    };
    lugs(0);
    ctx.fillStyle = far ? '#121315' : '#1b1c1f';
    {                                                                // tread: the sidewall disc swept along the axle = convex hull of
      const F = TREAD_F;                                             // both sidewall ellipses (discs stepped along it left see-through
      let fi = 0;                                                    // gaps when a steered tyre turns edge-on)
      for (let wi = 0; wi < 2; wi++) for (let k = 0; k < 24; k++) {
        const w = wi ? tw / 2 : -tw / 2, t = k * TAU / 24, u = Math.cos(t) * r, q = Math.sin(t) * r;
        F[fi++] = hx + w * Ax + fa * u; F[fi++] = hy + w * Ay + q + fb * u;
      }
      ctx.beginPath(); hullPath(ctx, F, fi); ctx.fill();
    }
    lugs(1);
    ctx.save(); ctx.translate(hx + out * tw / 2 * Ax, hy + out * tw / 2 * Ay); ctx.transform(fa, fb, 0, 1, 0, 0);
    this._wheelFace(0, 0, o.angle, r, far);
    ctx.restore();
  }

  /** Chassis panels extruded across the body width (truck frame); caps (and the side window, dents) on the near (right) side. */
  _body3(v) {
    const ctx = this.ctx, sh = this.shape, dm = this.dims, B = dm.B, tf = this._T;
    const wm = sh.winchMount, c = Math.cos(v.angle), s = Math.sin(v.angle);
    const wx = v.x + c * (wm[0] - 0.09) - s * wm[1], wy = v.y + s * (wm[0] - 0.09) + c * wm[1];
    this._tz(0);
    ctx.lineCap = 'butt'; ctx.lineWidth = 0.24; ctx.strokeStyle = '#15161a';           // winch drum
    ctx.beginPath(); ctx.moveTo(wx - 0.3 * tf.Dx, wy - 0.3 * tf.Dy); ctx.lineTo(wx + 0.3 * tf.Dx, wy + 0.3 * tf.Dy); ctx.stroke();
    // body faces shade by their TRUCK-frame normal (_prismRot): world normals flipped a face lying near the top / rear threshold
    // (Buggy rear chamfer, ny 0.555) between shades whenever the truck pitched a degree (gas, bumps)
    const bodyRot = [c, s];
    this._prismRot = bodyRot;
    for (const f of this.frames) this._prism(this._xf(f.points, v.x, v.y, v.angle), f.points.length, -B * 0.8, B * 0.8, this._shades(this.colors.frame), 3, 'rgba(0,0,0,0.5)');
    this._prismRot = null;
    this._rig(v, 0, true); this._tz(0);
    for (const p of this.panelOrder) {
      this._prismRot = bodyRot;
      this._prism(this._xf(p.points, v.x, v.y, v.angle), p.points.length, -B, B, this._shades(this.colors[p.kind] || '#c8572a'), 1, 'rgba(0,0,0,0.5)');
      this._prismRot = null;
      if (p.kind === 'cab' && this.rearWin) {                        // rear window on the cab's back face
        const [lo, hi] = this.rearWin, P = this._xf([[lerp(lo[0], hi[0], 0.42), lerp(lo[1], hi[1], 0.42)], [lerp(lo[0], hi[0], 0.88), lerp(lo[1], hi[1], 0.88)]], v.x, v.y, v.angle);
        if (c * tf.bx - s * S30 > 0.1) {                              // back face (local −x) turned toward the camera
          const za = -B * 0.72, zb = B * 0.72;
          ctx.beginPath(); ctx.moveTo(P[0] + za * tf.Dx, P[1] + za * tf.Dy); ctx.lineTo(P[2] + za * tf.Dx, P[3] + za * tf.Dy);
          ctx.lineTo(P[2] + zb * tf.Dx, P[3] + zb * tf.Dy); ctx.lineTo(P[0] + zb * tf.Dx, P[1] + zb * tf.Dy); ctx.closePath();
          ctx.fillStyle = 'rgba(150,190,210,0.85)'; ctx.fill();
        }
      }
    }
    const st = this.stack;                                           // exhaust stack sits between the panels' sides and caps
    if (!st.hidden) {
      this._tz(dm.zStack); ctx.save(); ctx.translate(v.x, v.y); ctx.rotate(v.angle);
      ctx.lineWidth = 0.075; ctx.strokeStyle = '#2b2c30'; ctx.lineCap = 'butt';
      ctx.beginPath(); ctx.moveTo(st.x, st.y0); ctx.lineTo(st.x, st.y1); ctx.stroke(); ctx.lineCap = 'round';
      ctx.restore();
    }
    this._tz(0);
    this._prismRot = bodyRot;
    for (const p of this.panels) this._prism(this._xf(p.points, v.x, v.y, v.angle), p.points.length, -B, B, this._shades(this.colors[p.kind] || '#c8572a'), 2, 'rgba(0,0,0,0.5)');
    this._prismRot = null;
    this._tz(B); ctx.save(); ctx.translate(v.x, v.y); ctx.rotate(v.angle);
    this._paintFx(v);                                                // gloss + dirt under the window glass
    if (this.window && this.look.style !== 'jeep') {
      const p = this.window;
      ctx.beginPath(); ctx.moveTo(p[0][0], p[0][1]);
      for (let i = 1; i < p.length; i++) ctx.lineTo(p[i][0], p[i][1]);
      ctx.closePath(); ctx.fillStyle = 'rgba(190,225,240,0.9)'; ctx.fill();
    }
    const hc = v.hardcore;
    if (hc && hc.body > 0.03) this._dents(sh, hc.body);
    ctx.restore();
    this._tz(0); this._rig(v, 1, true); this._z(0);
    if (hc && (hc.engineHot || hc.stalled)) { this._tz(0); this._steam(v, hc); this._z(0); }
  }


  // -- rig decor (vehicle JSON `look`) -------------------------------------------------------------
  // Drawn in world coords of the current frame (2.5D: _tz(0) pre-transform, 2D: _z(0)) from chassis-local
  // points (lx forward, ly up) at body depth lz (2D: depth collapses). pass 0 = before the panels, 1 = after.

  /** 4-link: lower link frame pivot -> hub, upper link + axle bracket above it (local up of chassis angle a). */
  _links(o, hx, hy, a) {
    const ctx = this.ctx, ux = -Math.sin(a), uy = Math.cos(a);
    ctx.lineWidth = 0.1; ctx.strokeStyle = '#191a1d';
    ctx.beginPath(); ctx.moveTo(o.linkX, o.linkY); ctx.lineTo(hx, hy);
    if (this.look && this.look.links >= 4) {
      ctx.stroke(); ctx.lineWidth = 0.065; ctx.strokeStyle = '#2a2c31'; ctx.beginPath();
      ctx.moveTo(o.linkX + ux * 0.24, o.linkY + uy * 0.24); ctx.lineTo(hx + ux * 0.18, hy + uy * 0.18);
      ctx.moveTo(hx, hy); ctx.lineTo(hx + ux * 0.2, hy + uy * 0.2);
    }
    ctx.stroke();
  }

  /** Chassis-local decor extents (silhouette for prop clipping). */
  _rigPts() {
    const b = this.box, st = this.look.style, r = this.shape.wheelRadius || 0.5, P = [];
    const fr = b.frame, cb = b.cab, bd = b.bed;
    if (st === 'jeep' && bd) P.push([bd.x0 - 0.3, bd.y0 + 0.1], [bd.x0 - 0.3, bd.y1 + 0.35]);
    if (st === 'bogger' && cb && fr) P.push([cb.x1 - 0.4, cb.y1 + 0.3], [fr.x1 + 0.12, 0.66], [fr.x1 + 0.12, 0.12]);
    if (st === 'buggy' && cb) P.push([cb.x0 + 1.2, cb.y1 + 0.15]);
    if (st === 'truggy' && bd && fr) P.push([bd.x0 + 0.6, bd.y0 + 0.1 + 2 * r * 0.78], [fr.x1 + 0.12, 0.3], [fr.x0 - 0.12, 0.3]);
    return P;
  }

  _rigFrame(v, d3) {
    const r = this._rT || (this._rT = {}), tf = this._T;
    r.x = v.x; r.y = v.y; r.c = Math.cos(v.angle); r.s = Math.sin(v.angle); r.a = v.angle; r.v = v;
    r.d3 = d3; r.Dx = d3 ? tf.Dx : 0; r.Dy = d3 ? tf.Dy : 0;
    r.rear = d3 && r.c * tf.bx - r.s * S30 > 0.05;                // local −x faces turned toward the camera
    return r;
  }
  _rx(lx, ly, lz) { const r = this._rT; return r.x + r.c * lx - r.s * ly + lz * r.Dx; }
  _ry(lx, ly, lz) { const r = this._rT; return r.y + r.s * lx + r.c * ly + lz * r.Dy; }
  /** Path through local points [lx, ly, lz?] (lz defaults to z). */
  _rpath(pts, z, close) {
    const ctx = this.ctx; ctx.beginPath();
    for (let i = 0; i < pts.length; i++) { const p = pts[i], lz = p[2] ?? z, X = this._rx(p[0], p[1], lz), Y = this._ry(p[0], p[1], lz); i ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y); }
    if (close) ctx.closePath();
  }
  /** Round tube: dark body + light highlight. */
  _tube(pts, z, w, col) {
    const ctx = this.ctx; this._rpath(pts, z);
    ctx.lineWidth = w; ctx.strokeStyle = mix(col, '#000000', 0.35); ctx.stroke();
    ctx.lineWidth = w * 0.4; ctx.strokeStyle = mix(col, '#ffffff', 0.35); ctx.stroke();
  }
  _rfill(pts, z, col, edge) { const ctx = this.ctx; this._rpath(pts, z, true); ctx.fillStyle = col; ctx.fill(); if (edge) { ctx.lineWidth = 0.02; ctx.strokeStyle = edge; ctx.stroke(); } }
  /** Solid extruded part between depths z0..z1 (hex colour). */
  _rslab(pts, z0, z1, col) {
    // face shades from the TRUCK-frame normals (rigid decor: a slanted face must not flip top/side/rear shade as the truck
    // pitches = the Truggy hood scoop changing colour with the slope)
    const r = this._rT, D0 = this._dz; this._dz = [r.Dx, r.Dy]; this._prismRot = [Math.cos(r.a), Math.sin(r.a)];
    this._prism(this._xf(pts, r.x, r.y, r.a), pts.length, z0, z1, this._shades(col), 3, this._shades(col).edge);
    this._dz = D0; this._prismRot = null;
  }
  _disc(lx, ly, lz, rad, col) { const ctx = this.ctx; ctx.beginPath(); ctx.arc(this._rx(lx, ly, lz), this._ry(lx, ly, lz), rad, 0, TAU); ctx.fillStyle = col; ctx.fill(); }
  /** Near-side headlamp: drawn like _lamp + its truck-frame lens position kept for the headlight pass (_atmoPost beam origin). */
  _headLamp(lx, ly, lz, rad, lens) { (this._head || (this._head = [0, 0, 0])).splice(0, 3, lx, ly, lz); this._lamp(lx, ly, lz, rad, lens); }
  // lens lit only while the engine runs (_lampK)
  _lamp(lx, ly, lz, rad, lens = (this._lampK || 0) > 0.5 ? '#fff2c4' : '#8f8e88') { this._disc(lx, ly, lz, rad, '#1a1b1e'); this._disc(lx, ly, lz, rad * 0.72, this.colors.trim); this._disc(lx, ly, lz, rad * 0.5, lens); }
  /** Polygon clipped to y0 <= ly <= y1 (Sutherland–Hodgman, two horizontal planes). */
  _clipY(pts, y0, y1) {
    const cut = (P, y, keepAbove) => {
      const o = [];
      for (let i = 0; i < P.length; i++) {
        const a = P[i], b = P[(i + 1) % P.length], ia = keepAbove ? a[1] >= y : a[1] <= y, ib = keepAbove ? b[1] >= y : b[1] <= y;
        if (ia) o.push(a);
        if (ia !== ib) { const t = (y - a[1]) / (b[1] - a[1]); o.push([lerp(a[0], b[0], t), y]); }
      }
      return o;
    };
    return cut(cut(pts, y0, true), y1, false);
  }
  /** Tyre standing in the side plane, tread swept z0 -> z1 (z1 = camera face, rim drawn there). */
  _spareSide(lx, ly, z0, z1, rad) {
    const ctx = this.ctx, r = this._rT;
    if (r.d3) for (let k = 0; k <= 4; k++) this._disc(lx, ly, lerp(z0, z1, k / 4), rad, k < 4 ? '#141517' : '#1b1c1f');
    this._wheelFace(this._rx(lx, ly, z1), this._ry(lx, ly, z1), r.a + 0.4, rad, false);
  }
  /** Tyre hung on the rear face (plane x = lx), sticking out depth `w` backwards. 2D: side profile. */
  _spareRear(lx, ly, lz, rad, w) {
    const ctx = this.ctx, r = this._rT;
    if (!r.d3) { this._rfill([[lx, ly - rad], [lx - w, ly - rad], [lx - w, ly + rad], [lx, ly + rad]], 0, '#1b1c1f', '#000'); return; }
    for (let k = 0; k <= 5; k++) {
      const x = lx - w * k / 5, outer = k === 5;
      ctx.save(); ctx.translate(this._rx(x, ly, lz), this._ry(x, ly, lz)); ctx.transform(r.Dx, r.Dy, -r.s, r.c, 0, 0);
      ctx.beginPath(); ctx.arc(0, 0, rad, 0, TAU); ctx.fillStyle = outer && r.rear ? '#202124' : '#141517'; ctx.fill();
      if (outer && r.rear) {
        ctx.beginPath(); ctx.arc(0, 0, rad * 0.55, 0, TAU); ctx.fillStyle = this.look.rim; ctx.fill();
        ctx.beginPath(); ctx.arc(0, 0, rad * 0.15, 0, TAU); ctx.fillStyle = '#2b2c30'; ctx.fill();
      }
      ctx.restore();
    }
  }
  /** Cage rails along a tube-kind panel outline (bottom edges skipped) at depth z. */
  _cageRail(q, y0, z, w, col) {
    const n = q.length;
    for (let i = 0; i < n; i++) {
      const a = q[i], b = q[(i + 1) % n];
      if (a[1] < y0 + 0.02 && b[1] < y0 + 0.02) continue;
      this._tube([a, b], z, w, col);
    }
  }
  /** Mud spatter clipped to the near caps of the solid panels (deterministic layout). */
  _spatter(B, y1, n, col) {
    const ctx = this.ctx, fr = this.box.frame; ctx.save(); ctx.beginPath();
    for (const p of [...this.frames, ...this.panels]) {
      const q = p.points; ctx.moveTo(this._rx(q[0][0], q[0][1], B), this._ry(q[0][0], q[0][1], B));
      for (let i = 1; i < q.length; i++) ctx.lineTo(this._rx(q[i][0], q[i][1], B), this._ry(q[i][0], q[i][1], B));
      ctx.closePath();
    }
    ctx.clip('nonzero'); ctx.fillStyle = col;
    let seed = 7;
    const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
    for (let i = 0; i < n; i++) {
      const x = lerp(fr.x0, fr.x1, rnd()), y = lerp(fr.y0, y1, rnd() ** 1.6), rad = 0.025 + 0.09 * rnd() * (1 - (y - fr.y0) / (y1 - fr.y0 + 1e-6));
      this._disc(x, y, B, rad, col);
    }
    ctx.restore();
  }

  _rig(v, pass, d3) {
    const st = this.look && this.look.style;
    if (!st || !this.dims) return;
    const ctx = this.ctx, r = this._rigFrame(v, d3), B = this.dims.B, b = this.box, C = this.colors;
    const fr = b.frame, hd = b.hood, cb = b.cab, bd = b.bed, sh = this.shape, R = sh.wheelRadius || 0.5;
    const tube = this.look.tube, poly = k => (sh.shapes.find(s => s.kind === k) || {}).points;
    ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    const zs = d3 ? [-1, 1] : [1];                                     // far -> near sides
    // front bumpers / guards sit ahead of the nose, i.e. behind the hood for this camera: far side + cross tubes
    // before the panels (pass 0) so the hood hides them, near side after (pass 1)
    const front = (sideFn, crossFn) => { if (pass === 0) { if (d3) sideFn(-1); crossFn(); } else sideFn(1); };
    if (st === 'jeep' && sh.winchMount) { const wm = sh.winchMount;
      front(z => this._tube([[wm[0] - 0.05, wm[1] - 0.15, z * (B + 0.1)], [wm[0] - 0.3, wm[1] - 0.15, z * (B + 0.12)]], 0, 0.08, C.trim),
        () => this._tube([[wm[0] - 0.05, wm[1] - 0.15, -B - 0.1], [wm[0] - 0.05, wm[1] - 0.15, B + 0.1]], 0, 0.11, C.trim)); }
    if (st === 'bogger' && fr) { const gx = fr.x1 + 0.06;
      front(z => this._tube([[gx - 0.35, 0.18], [gx, 0.18], [gx, 0.62], [gx - 0.18, 0.66]], z * B * 1.04, 0.08, C.trim),
        () => { for (const y of [0.18, 0.42, 0.62]) this._tube([[gx, y, -B * 1.04], [gx, y, B * 1.04]], 0, 0.08, C.trim); }); }
    if (st === 'truggy' && fr) { const bx = fr.x1 + 0.04;
      front(z => this._tube([[bx, 0.26], [bx - 0.32, 0.26]], z * (B + 0.06), 0.1, C.trim),
        () => this._tube([[bx, 0.26, -B - 0.06], [bx, 0.26, B + 0.06]], 0, 0.12, C.trim)); }
    if (st === 'jeep') {
      if (pass !== 1 || !cb) return;
      const yt = cb.y0 + 0.6 * (cb.y1 - cb.y0), canvas = '#b3a274', q = poly('cab');
      if (this.window) this._rfill(this._clipY(this.window, -9, yt), B, '#2a2a22');   // open side: dark tub interior
      this._rslab([[cb.x0 + 0.42, cb.y0 + 0.22], [cb.x0 + 0.72, cb.y0 + 0.22], [cb.x0 + 0.72, cb.y0 + 0.3], [cb.x0 + 0.42, cb.y0 + 0.3]], B * 0.1, B * 0.7, '#57583a');
      this._rslab([[cb.x0 + 0.34, cb.y0 + 0.22], [cb.x0 + 0.44, cb.y0 + 0.22], [cb.x0 + 0.38, yt - 0.02], [cb.x0 + 0.28, yt - 0.02]], B * 0.1, B * 0.7, '#57583a');
      this._rslab(this._clipY(q, yt, 9), -B * 1.03, B * 1.03, canvas);                   // canvas soft top
      ctx.lineWidth = 0.03; ctx.strokeStyle = mix(canvas, '#000000', 0.35);
      for (let i = 1; i <= 3; i++) { const x = lerp(cb.x0 + 0.15, cb.x1 - 0.35, i / 4); this._rpath([[x, yt + 0.02], [x, cb.y1 - 0.01]], B * 1.03); ctx.stroke(); }
      if (bd) {
        const xr = Math.min(bd.x0, fr.x0);
        this._rslab([[xr - 0.16, bd.y0 + 0.02], [xr, bd.y0 + 0.02], [xr, bd.y0 + 0.42], [xr - 0.16, bd.y0 + 0.42]], -B * 0.85, -B * 0.4, '#4b5b2f');   // jerry can
        this._tail = [xr, bd.y1 - 0.09, B * 0.9];                                         // tail lamps -> _atmoPost glow
        if (r.rear) for (const z of [-B * 0.9, B * 0.9]) this._rfill([[xr, bd.y1 - 0.14, z - 0.06], [xr, bd.y1 - 0.14, z + 0.06], [xr, bd.y1 - 0.04, z + 0.06], [xr, bd.y1 - 0.04, z - 0.06]], 0, '#b0201c');
        this._spareRear(xr, (bd.y0 + bd.y1) / 2 + 0.08, B * 0.3, R * 0.82, 0.24);
      }
      if (hd) this._headLamp(hd.x1 - 0.08, hd.y1 - 0.12, B + 0.02, 0.08);
    } else if (st === 'bogger') {
      if (pass !== 1 || !cb) return;
      this._spatter(B, Math.min(0.75, this.window ? Math.min(...this.window.map(p => p[1])) - 0.04 : 0.75), 70, 'rgba(86,58,34,0.6)');
      ctx.lineWidth = 0.02; ctx.strokeStyle = 'rgba(0,0,0,0.35)';                       // door seam
      const wb = Math.min(cb.y0 + 0.62, this.window ? Math.min(...this.window.map(p => p[1])) - 0.03 : 9);   // stops at the sill: never across the glass
      this._rpath([[cb.x1 - 0.1, wb], [cb.x1 - 0.1, cb.y0 + 0.08], [cb.x0 + 0.12, cb.y0 + 0.08], [cb.x0 + 0.12, wb]], B); ctx.stroke();
      this._rslab([[cb.x1 - 0.02, cb.y0 + 0.6], [cb.x1 + 0.1, cb.y0 + 0.6], [cb.x1 + 0.1, cb.y0 + 0.74], [cb.x1 - 0.02, cb.y0 + 0.74]], B, B + 0.14, '#1d1e22');   // mirror
      if (bd) {                                                                          // bed roll bar + braces
        const xb = bd.x1 - 0.22, yt = cb.y1 - 0.08;
        for (const z of zs) { this._tube([[xb, bd.y1], [xb, yt], [bd.x0 + 0.5, bd.y1]], z * B * 0.82, 0.08, C.frame); if (z < 0) this._tube([[xb, yt, -B * 0.82], [xb, yt, B * 0.82]], 0, 0.08, C.frame); }
        this._tail = [bd.x0, bd.y1 - 0.125, B * 0.9];
        if (r.rear) for (const z of [-B * 0.9, B * 0.9]) this._rfill([[bd.x0, bd.y1 - 0.2, z - 0.07], [bd.x0, bd.y1 - 0.2, z + 0.07], [bd.x0, bd.y1 - 0.05, z + 0.07], [bd.x0, bd.y1 - 0.05, z - 0.07]], 0, '#b0201c');
      }
      const cx = poly('cab').filter(p => p[1] > cb.y1 - 0.01).reduce((m, p) => Math.max(m, p[0]), cb.x0) - 0.08, ly = cb.y1 + 0.07;   // roof light bar + 4 pods
      this._tube([[cx, ly, -B * 0.8], [cx, ly, B * 0.8]], 0, 0.06, C.frame);
      for (let i = 0; i < 4; i++) { const z = lerp(-0.6, 0.6, i / 3) * B; if (!d3 && i < 3) continue; this._tube([[cx, ly], [cx, ly + 0.1]], z, 0.035, C.frame); this._lamp(cx + 0.02, ly + 0.15, z, 0.08); }
      this._pods = { x: cx + 0.04, y: ly + 0.15, z: [0, 1, 2, 3].map(i => lerp(-0.6, 0.6, i / 3) * B) };   // roof pods -> _atmoPost glow + spill
      if (hd) this._headLamp(hd.x1 - 0.06, hd.y1 - 0.16, B + 0.02, 0.07, '#ffb347');
    } else if (st === 'wedge') {                                     // lifted wedge supercar: side intake, rear wing, slit lamps
      if (pass !== 1 || !cb) return;
      const ix0 = cb.x0 + 0.08, ix1 = cb.x0 + 0.55, iy0 = cb.y0 + 0.1, iy1 = cb.y0 + 0.34;
      this._rslab([[ix0, iy0], [ix1, iy0], [ix1 - 0.1, iy1], [ix0, iy1 - 0.04]], B, B + 0.015, '#141417');
      ctx.lineWidth = 0.018; ctx.strokeStyle = mix(C.body, '#000000', 0.25);
      for (let i = 1; i < 4; i++) { const y = lerp(iy0, iy1 - 0.04, i / 4); this._rpath([[ix0 + 0.03, y], [ix1 - 0.08 - 0.1 * i / 4, y]], B + 0.02); ctx.stroke(); }
      ctx.lineWidth = 0.025; ctx.strokeStyle = 'rgba(0,0,0,0.45)';                     // door shut line + sill crease
      this._rpath([[cb.x1 - 0.05, cb.y0 + 0.02], [cb.x1 - 0.2, cb.y0 + 0.38]], B + 0.005); ctx.stroke();
      if (fr) { this._rpath([[fr.x0 + 0.35, fr.y1 + 0.06], [fr.x1 - 0.4, fr.y1 + 0.06]], B + 0.005); ctx.stroke(); }
      if (bd) {                                                                          // wing on two struts
        const sx = bd.x0 + 0.35, wy = bd.y1 + 0.24, wx0 = bd.x0 + 0.02, wx1 = bd.x0 + 0.62;
        for (const z of zs) this._tube([[sx, bd.y1 - 0.05], [sx + 0.05, wy]], z * B * 0.6, 0.045, C.frame);
        this._rslab([[wx0, wy], [wx1, wy], [wx1 - 0.06, wy + 0.06], [wx0 - 0.03, wy + 0.08]], -B * 0.98, B * 0.98, C.bed || C.body);
        this._tail = [bd.x0, bd.y0 + 0.19, B * 0.8];
        if (r.rear) for (const z of [-B * 0.8, B * 0.8]) this._rfill([[bd.x0, bd.y0 + 0.16, z - 0.12], [bd.x0, bd.y0 + 0.16, z + 0.12], [bd.x0, bd.y0 + 0.22, z + 0.12], [bd.x0, bd.y0 + 0.22, z - 0.12]], 0, '#c01a1a');
      }
      if (hd) this._headLamp(hd.x1 - 0.14, hd.y0 + 0.07, B + 0.02, 0.05);
    } else if (st === 'buggy') {
      const q = poly('cab'), tw = 0.075, tc = C.frame;
      if (!q) return;
      if (pass === 0) { if (d3) this._tube([[cb.x1, cb.y0 + 0.38], [fr.x1 - 0.05, fr.y1 - 0.02]], -B, tw, tc); return; }
      if (tube.has('cab')) {
        this._rslab(this._clipY(q, -9, cb.y0 + 0.22), -B, B, C.body.startsWith('#') ? C.body : '#2d5fb5');   // low tub panels
        if (d3) { this._cageRail(q, cb.y0, -B, tw, tc); this._tube([[cb.x0 + 0.2, cb.y1], [cb.x0 + 0.85, cb.y0 + 0.22]], -B, tw, tc); }
        for (const p of q) if (p[1] > cb.y0 + 0.02) this._tube([[p[0], p[1], -B], [p[0], p[1], B]], 0, tw, tc);
        for (const z of zs) {                                                            // bucket seats
          const sx = cb.x0 + 0.75, sz = z * B * 0.45, y = cb.y0 + 0.22;
          this._rslab([[sx - 0.3, y], [sx + 0.2, y], [sx + 0.2, y + 0.12], [sx - 0.3, y + 0.12]], sz - 0.17, sz + 0.17, '#25272c');
          this._rslab([[sx - 0.42, y + 0.08], [sx - 0.26, y + 0.08], [sx - 0.12, y + 0.62], [sx - 0.28, y + 0.66]], sz - 0.17, sz + 0.17, '#2e3036');
        }
        this._cageRail(q, cb.y0, B, tw, tc);
        this._tube([[cb.x0 + 0.2, cb.y1], [cb.x0 + 0.85, cb.y0 + 0.22]], B, tw, tc);
      }
      for (const z of zs) this._tube([[cb.x0 + 0.1, cb.y0 + 0.55], [fr.x0 + 0.05, fr.y1 - 0.08]], z * B, tw, tc);   // rear braces
      this._tube([[cb.x1, cb.y0 + 0.38], [fr.x1 - 0.05, fr.y1 - 0.02]], B, tw, tc);                                   // near front brace
      if (hd) this._rslab([[hd.x0 + 0.35, hd.y1 - 0.02], [hd.x0 + 0.7, hd.y1 - 0.06], [hd.x0 + 0.68, hd.y1 + 0.02], [hd.x0 + 0.38, hd.y1 + 0.04]], -B * 0.3, B * 0.3, '#1c2f57');   // hood vent
      const top = q.filter(p => p[1] > cb.y1 - 0.01).reduce((m, p) => Math.max(m, p[0]), cb.x0);
      for (const z of zs) this._lamp(top, cb.y1 + 0.08, z * B * 0.5, 0.065);
    } else if (st === 'truggy') {
      if (pass !== 1) return;
      const tw = 0.07, tc = C.frame, chrome = C.trim;
      if (hd) {                                                                          // muscle hood: scoop + crease
        const yh = x => { const q = poly('hood'); let y = hd.y0; for (let i = 0; i < q.length; i++) { const a = q[i], c = q[(i + 1) % q.length];
          if ((a[0] - x) * (c[0] - x) <= 0 && a[0] !== c[0]) y = Math.max(y, lerp(a[1], c[1], (x - a[0]) / (c[0] - a[0]))); } return y; };
        const x0 = hd.x0 + 0.5, x1 = hd.x0 + 1.05;
        this._rslab([[x0, yh(x0) - 0.01], [x1, yh(x1) - 0.01], [x1 - 0.06, yh(x1) + 0.07], [x0 + 0.08, yh(x0) + 0.08]], -B * 0.32, B * 0.32, C.hood.startsWith('#') ? C.hood : '#c3262a');
        this._rfill([[x1 - 0.05, yh(x1) + 0.0, B * 0.32], [x1 - 0.1, yh(x1) + 0.06, B * 0.32], [x1 - 0.1, yh(x1) + 0.06, -B * 0.32], [x1 - 0.05, yh(x1) + 0.0, -B * 0.32]], 0, C.accent);
        ctx.lineWidth = 0.025; ctx.strokeStyle = 'rgba(0,0,0,0.3)';
        this._rpath([[hd.x1 - 0.05, hd.y0 + 0.2], [cb ? cb.x0 + 0.1 : hd.x0, hd.y0 + 0.24]], B); ctx.stroke();
        this._headLamp(hd.x1 - 0.1, hd.y0 + 0.2, B + 0.01, 0.065);
        for (let i = 0; i < 4; i++) { ctx.fillStyle = '#121316'; this._rpath([[hd.x1 - 0.02, hd.y0 + 0.06 + i * 0.045], [hd.x1 - 0.32, hd.y0 + 0.06 + i * 0.045]], B + 0.005); ctx.lineWidth = 0.02; ctx.strokeStyle = '#121316'; ctx.stroke(); }
      }
      if (cb) {
        ctx.lineWidth = 0.02; ctx.strokeStyle = 'rgba(0,0,0,0.35)';
        this._rpath([[cb.x1 - 0.05, cb.y0 + 0.05], [cb.x0 + 0.1, cb.y0 + 0.05], [cb.x0 + 0.1, cb.y0 + 0.42]], B); ctx.stroke();
        this._rslab([[cb.x1 - 0.02, cb.y0 + 0.44], [cb.x1 + 0.1, cb.y0 + 0.44], [cb.x1 + 0.1, cb.y0 + 0.56], [cb.x1 - 0.02, cb.y0 + 0.56]], B, B + 0.13, '#1d1e22');
      }
      const rx = fr.x0 - 0.04;                                                           // rear tube bumper
      if (bd && tube.has('bed')) {                                                       // tube rear cage, deck, upright spare
        const q = poly('bed'), dy = bd.y0 + 0.08;
        this._rslab([[bd.x0, bd.y0], [bd.x1, bd.y0], [bd.x1, dy], [bd.x0, dy]], -B, B, '#26272b');
        if (d3) { this._cageRail(q, bd.y0, -B, tw, tc); this._tube([[bd.x1, bd.y1 - 0.05], [bd.x0 + 0.1, dy]], -B, tw, tc); }
        for (const p of q) if (p[1] > bd.y0 + 0.02) this._tube([[p[0], p[1], -B], [p[0], p[1], B]], 0, tw, tc);
        const sr = R * 0.78, sx = bd.x0 + 0.6;
        this._spareSide(sx, dy + sr, -0.2, 0.2, sr);
        this._tube([[sx, dy + 2 * sr - 0.02, -0.25], [sx, dy + 2 * sr - 0.02, 0.25]], 0, 0.035, '#d0a020');   // strap
        this._cageRail(q, bd.y0, B, tw, tc);
        this._tube([[bd.x1, bd.y1 - 0.05], [bd.x0 + 0.1, dy]], B, tw, tc);
      }
      for (const z of zs) this._tube([[rx + 0.3, 0.3], [rx, 0.3]], z * (B + 0.04), 0.08, tc);
      this._tube([[rx, 0.3, -B - 0.04], [rx, 0.3, B + 0.04]], 0, 0.09, tc);
      if (cb) {                                                                          // side exhaust + heat shield
        const y = fr.y0 + 0.22, z = B + 0.08;
        this._tube([[cb.x1 - 0.05, y], [cb.x0 - 0.3, y]], z, 0.09, chrome);
        ctx.fillStyle = '#5a5d63';
        for (let i = 0; i < 9; i++) this._disc(lerp(cb.x1 - 0.2, cb.x0 - 0.1, i / 8), y, z + 0.01, 0.012, '#4a4d52');
        this._disc(cb.x0 - 0.3, y, z, 0.035, '#111');
      }
    }
  }

  // -- vehicle -----------------------------------------------------------------------------------
  _wheel(o, far) { this._wheelFace(o.x + (far ? 0.08 : 0), o.y + (far ? 0.06 : 0), o.angle, o.radius, far); }

  _wheelFace(x, y, angle, r, far) {
    const ctx = this.ctx;
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(angle);
    const lk = this.look || { tread: { lugs: 12, depth: 0.05, kind: 'lug' }, rim: '#b8bcc4' }, tr = lk.tread, n = tr.lugs, d = treadDepth(r, tr);
    const lw = clamp(TAU * r / n * (tr.kind === 'paddle' ? 0.62 : 0.5), 0.07, 0.26);
    // tread lugs: chamfered blocks standing proud of the carcass (depth scales with tyre size + tread depth), a sipe
    // across each, block treads alternate long / short; then sidewall shoulder lugs on the carcass ring
    const lug = far ? '#17181a' : '#26272b', top = far ? '#1d1e21' : '#33353a', sipe = far ? '#0e0f10' : '#141518';
    // batched: all lug blocks = one fill, all lit top edges = one stroke, all sipes = one stroke (lugs never overlap)
    const LR = LUG_LR; LR[0] = LR[10] = r - 0.03;                  // (radius, width·lw) per corner; 0 = t0 (−0.012 on the chamfers)
    for (let pass = 0; pass < 3; pass++) {
      ctx.beginPath();
      for (let i = 0; i < n; i++) {
        const a = (i + 0.5) * TAU / n, c = Math.cos(a), sn = Math.sin(a), dd = tr.kind === 'block' && i & 1 ? d * 0.65 : d, t0 = r + dd;
        if (pass === 0) {
          for (let k = 0; k < 6; k++) {
            const rr = LR[2 * k] || (k === 1 || k === 4 ? t0 - 0.012 : t0), w = LR[2 * k + 1] * lw, X = c * rr - sn * w, Y = sn * rr + c * w;
            k ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y);
          }
          ctx.closePath();
        } else {
          const rr = pass === 1 ? t0 - 0.006 : r + dd * 0.45, w = (pass === 1 ? 0.34 : 0.4) * lw;   // worn top edge catches light / sipe
          ctx.moveTo(c * rr + sn * w, sn * rr - c * w); ctx.lineTo(c * rr - sn * w, sn * rr + c * w);
        }
      }
      if (pass === 0) { ctx.fillStyle = lug; ctx.fill(); }
      else { ctx.lineWidth = pass === 1 ? 0.014 : 0.012; ctx.strokeStyle = pass === 1 ? top : sipe; ctx.stroke(); }
    }
    ctx.beginPath(); ctx.arc(0, 0, r - 0.015, 0, TAU); ctx.fillStyle = far ? '#101113' : '#1a1b1e'; ctx.fill();
    ctx.fillStyle = far ? '#151618' : '#222327';                    // sidewall shoulder lugs
    for (let i = 0; i < n; i++) {
      const a = (i + 0.5 + (i & 1 ? 0.12 : -0.12)) * TAU / n, c = Math.cos(a), s = Math.sin(a), w = lw * 0.28, r0 = r - 0.02 - d * 0.9, r1 = r - 0.02;
      if (!i) ctx.beginPath();                                       // one path, one fill for every shoulder lug
      ctx.moveTo(c * r0 - s * w, s * r0 + c * w); ctx.lineTo(c * r1 - s * w * 1.3, s * r1 + c * w * 1.3); ctx.lineTo(c * r1 + s * w * 1.3, s * r1 - c * w * 1.3); ctx.lineTo(c * r0 + s * w, s * r0 - c * w); ctx.closePath();
    }
    ctx.fill();
    ctx.lineWidth = 0.025; ctx.strokeStyle = far ? '#1c1d20' : '#34363b';
    ctx.beginPath(); ctx.arc(0, 0, r * 0.78, 0, TAU); ctx.stroke();
    ctx.beginPath(); ctx.arc(0, 0, r * 0.54, 0, TAU); ctx.fillStyle = far ? (this._rimFar?.k === lk.rim ? this._rimFar.c : (this._rimFar = { k: lk.rim, c: mix(lk.rim, '#000000', 0.5) }).c) : lk.rim; ctx.fill();
    if (lk.beadlock) {                        // beadlock ring + bolts
      ctx.lineWidth = 0.04; ctx.strokeStyle = far ? '#202124' : '#3c3e44'; ctx.beginPath(); ctx.arc(0, 0, r * 0.5, 0, TAU); ctx.stroke();
      ctx.fillStyle = far ? '#55585e' : '#c8ccd2';
      for (let i = 0; i < 10; i++) { const a = i * TAU / 10; ctx.fillRect(Math.cos(a) * r * 0.5 - 0.012, Math.sin(a) * r * 0.5 - 0.012, 0.024, 0.024); }
    }
    ctx.lineWidth = 0.05; ctx.strokeStyle = far ? '#3a3c40' : '#5d6068';
    ctx.beginPath();
    for (let i = 0; i < 5; i++) { const a = i * TAU / 5; ctx.moveTo(0, 0); ctx.lineTo(Math.cos(a) * r * 0.5, Math.sin(a) * r * 0.5); }
    ctx.stroke();
    ctx.beginPath(); ctx.arc(0, 0, r * 0.13, 0, TAU); ctx.fillStyle = far ? '#2a2b2e' : '#3b3d42'; ctx.fill();
    ctx.fillStyle = far ? '#7a2f1a' : '#e0492a';   // one coloured lug nut makes rotation obvious
    ctx.beginPath(); ctx.arc(r * 0.36, 0, r * 0.07, 0, TAU); ctx.fill();
    ctx.restore();
  }

  _shock(o, dmg = 0) {
    // coil-over from the chassis mount to the hub; the coil visibly shortens as the wheel compresses.
    // dmg 0..1 (hardcore shock damage): the shaft bends and the coil turns rust-red.
    const ctx = this.ctx, dx = o.x - o.mountX, dy = o.y - o.mountY, len = Math.hypot(dx, dy) || 1;
    const ux = dx / len, uy = dy / len, nx = -uy, ny = ux, kink = dmg * 0.16;
    ctx.lineWidth = 0.05; ctx.strokeStyle = dmg > 0.5 ? '#9a8f88' : '#c9ccd2';
    ctx.beginPath(); ctx.moveTo(o.mountX, o.mountY);
    if (kink > 0.005) ctx.lineTo(o.mountX + ux * len * 0.55 + nx * kink, o.mountY + uy * len * 0.55 + ny * kink);
    ctx.lineTo(o.x, o.y); ctx.stroke();
    const body = Math.min(0.3, len * 0.45);
    const cc = this.look ? this.look.coil : { spring: '#f0c020', body: '#2f6fb3' };
    ctx.lineWidth = 0.13; ctx.strokeStyle = cc.body; ctx.lineCap = 'butt';
    ctx.beginPath(); ctx.moveTo(o.mountX, o.mountY); ctx.lineTo(o.mountX + ux * body, o.mountY + uy * body); ctx.stroke();
    ctx.lineCap = 'round';
    const a0 = 0.06, a1 = len - 0.1, coils = 7, amp = 0.085;
    ctx.lineWidth = 0.04;
    ctx.strokeStyle = dmg > 0.01 ? mix(cc.spring, '#c83c1e', dmg) : cc.spring;
    ctx.beginPath(); ctx.moveTo(o.mountX + ux * a0, o.mountY + uy * a0);
    for (let i = 1; i < coils * 2; i++) {
      const t = lerp(a0, a1, i / (coils * 2)), s = (i % 2 ? 1 : -1) * amp;
      ctx.lineTo(o.mountX + ux * t + nx * s, o.mountY + uy * t + ny * s);
    }
    ctx.lineTo(o.mountX + ux * a1, o.mountY + uy * a1); ctx.stroke();
  }

  _vehicle(v) {
    const ctx = this.ctx, sh = this.shape, W = v.wheels;
    if (!sh) return;
    for (const o of W) if (o.side === 'R') this._wheel(o, true);           // far side
    for (const o of W) if (o.side === 'L') this._links(o, o.x, o.y, v.angle);
    this._rig(v, 0, false);
    ctx.save(); ctx.translate(v.x, v.y); ctx.rotate(v.angle);
    for (const s of sh.shapes) {
      if (this.look.tube.has(s.kind)) continue;                          // tube cage: _rig
      const p = s.points;
      ctx.beginPath(); ctx.moveTo(p[0][0], p[0][1]);
      for (let i = 1; i < p.length; i++) ctx.lineTo(p[i][0], p[i][1]);
      ctx.closePath();
      ctx.fillStyle = this.colors[s.kind] || '#c8572a'; ctx.fill();
      ctx.lineWidth = 0.035; ctx.strokeStyle = 'rgba(0,0,0,0.5)'; ctx.stroke();
    }
    if (this.window && this.look.style !== 'jeep') {
      const p = this.window;
      ctx.beginPath(); ctx.moveTo(p[0][0], p[0][1]);
      for (let i = 1; i < p.length; i++) ctx.lineTo(p[i][0], p[i][1]);
      ctx.closePath(); ctx.fillStyle = 'rgba(190,225,240,0.9)'; ctx.fill();
    }
    const st = this.stack;
    if (!st.hidden) {
      ctx.lineWidth = 0.075; ctx.strokeStyle = '#2b2c30'; ctx.lineCap = 'butt';
      ctx.beginPath(); ctx.moveTo(st.x, st.y0); ctx.lineTo(st.x, st.y1); ctx.stroke(); ctx.lineCap = 'round';
    }
    const wm = sh.winchMount;
    ctx.fillStyle = '#15161a'; ctx.fillRect(wm[0] - 0.22, wm[1] - 0.12, 0.26, 0.24);   // winch drum
    const hc = v.hardcore;
    if (hc && hc.body > 0.03) this._dents(sh, hc.body);
    ctx.restore();
    this._rig(v, 1, false);
    if (hc && (hc.engineHot || hc.stalled)) this._steam(v, hc);
    const wn = v.winch;
    if (wn.attached) {
      const d = Math.hypot(wn.ax - wn.mx, wn.ay - wn.my), slack = Math.max(0, wn.length - d);
      const sag = Math.sqrt(slack * Math.max(d, 0.01)) * 0.45, t = Math.min(1, wn.tension / wn.breakTension);
      ctx.lineWidth = 0.045 + 0.03 * t;
      ctx.strokeStyle = `rgb(${Math.round(lerp(70, 235, t))},${Math.round(lerp(70, 50, t))},${Math.round(lerp(75, 40, t))})`;
      ctx.beginPath();
      this._ropePath(wn.mx, wn.my, (wn.mx + wn.ax) / 2, (wn.my + wn.ay) / 2 - sag * 2 + this._ropeHum(t), wn.ax, wn.ay, 0, 1, null);
      ctx.stroke();
      ctx.fillStyle = '#ffd23c'; ctx.beginPath(); ctx.arc(wn.ax, wn.ay, 0.08, 0, TAU); ctx.fill();
    }
    for (const o of W) if (o.side === 'L') this._wheel(o, false);          // near side
    for (const o of W) if (o.side === 'L') {
      const pair = W.find(q => q.axle === o.axle && q !== o);
      this._shock(o, Math.max(o.damage || 0, pair ? pair.damage || 0 : 0));
    }
  }

  /** Body damage: dark scuffs / creases on the chassis panels (deterministic layout, chassis frame). */
  /** Near-face paint: clear-coat gloss streaks (fade as it gets dirty) + dirt: a mud wash rising from the sills and
   *  flecks, growing with this.dirt. Chassis-local coords (inside _body3's near-face transform). Flat fills, no gradients. */
  _paintFx(v) {
    const ctx = this.ctx, d = this.dirt || 0, gl = (1 - d) * (this.showroom ? 1.15 : 1);
    if (!this._dirtPts) {                                             // per-truck fleck pattern (seeded, stable)
      const R = rngOf(hashStr(String(this.shape.name || '') + this.panels.length)), pts = this._dirtPts = [];
      for (let i = 0; i < 90; i++) pts.push([R(), R(), 0.012 + R() * 0.035, R()]);
    }
    for (const p of this.panels) {
      const b = this.box[p.kind]; if (!b) continue;
      const w = b.x1 - b.x0, h = b.y1 - b.y0;
      ctx.save(); ctx.beginPath(); ctx.moveTo(p.points[0][0], p.points[0][1]);
      for (let i = 1; i < p.points.length; i++) ctx.lineTo(p.points[i][0], p.points[i][1]);
      ctx.closePath(); ctx.clip();
      if (gl > 0.02) {                                                // gloss: bright thin streak + soft wide sheen, slanted
        ctx.globalAlpha = 0.3 * gl; ctx.fillStyle = '#ffffff';
        ctx.beginPath(); ctx.moveTo(b.x0, b.y1 - h * 0.18); ctx.lineTo(b.x1, b.y1 - h * 0.08); ctx.lineTo(b.x1, b.y1 - h * 0.2); ctx.lineTo(b.x0, b.y1 - h * 0.3); ctx.fill();
        ctx.globalAlpha = 0.13 * gl;
        ctx.beginPath(); ctx.moveTo(b.x0, b.y1 - h * 0.34); ctx.lineTo(b.x1, b.y1 - h * 0.24); ctx.lineTo(b.x1, b.y1 - h * 0.55); ctx.lineTo(b.x0, b.y1 - h * 0.65); ctx.fill();
        ctx.globalAlpha = 0.35 * gl;                                  // crisp highlight line along the top edge
        ctx.fillRect(b.x0 + w * 0.06, b.y1 - h * 0.06, w * 0.88, Math.min(0.025, h * 0.04));
      }
      if (d > 0.02) {                                                 // mud wash from the bottom up + flecks
        ctx.globalAlpha = 0.55 * d; ctx.fillStyle = DIRT.col;
        const top = b.y0 + h * (0.25 + 0.5 * d);
        ctx.beginPath(); ctx.moveTo(b.x0, b.y0); ctx.lineTo(b.x1, b.y0);
        for (let k = 8; k >= 0; k--) { const x = b.x0 + w * k / 8; ctx.lineTo(x, top + Math.sin(k * 2.3 + w) * h * 0.06); }
        ctx.closePath(); ctx.fill();
        ctx.fillStyle = DIRT.fleck;
        const n = Math.round(this._dirtPts.length * d);
        ctx.globalAlpha = 0.6 * Math.min(1, d * 1.5); ctx.beginPath();
        for (let i = 0; i < n; i++) { const q = this._dirtPts[i], x = b.x0 + q[0] * w, y = b.y0 + Math.pow(q[1], 1.6) * h * (0.4 + 0.6 * d);
          ctx.moveTo(x + q[2], y); ctx.arc(x, y, q[2], 0, TAU); }
        ctx.fill();
      }
      ctx.restore();
    }
    ctx.globalAlpha = 1;
  }

  /** Dirt build-up from distance driven by surface under the tyres + wheelspin (render state; setLevel = clean). */
  _dirtTick(v, dt) {
    if (!(dt > 0) || !v.wheels) return;
    let k = 0, sp = 0;
    for (const o of v.wheels) {
      if (!o.grounded || !o.surface) continue;
      const n = o.surface;
      k += /mud|bog|clay|swamp/.test(n) ? DIRT.mud : /ford|water|river/.test(n) ? DIRT.water : /rock|stone|boulder|wood|plank/.test(n) ? DIRT.rock : DIRT.dirt;
      sp += /mud|bog|clay|dirt|berm/.test(n) ? Math.abs(o.slip || 0) : 0;
    }
    this.dirt = clamp((this.dirt || 0) + (k / 4) * Math.abs(v.speed || 0) * dt + DIRT.spin * (sp / 4) * dt, 0, 1);
  }

  _dents(sh, dmg) {
    const ctx = this.ctx, n = Math.ceil(dmg * 14);
    ctx.strokeStyle = `rgba(20,16,14,${(0.35 + 0.4 * dmg).toFixed(2)})`; ctx.lineWidth = 0.03;
    ctx.beginPath();
    for (let i = 0; i < n; i++) {
      const s = sh.shapes[i % sh.shapes.length].points, a = s[i % s.length], b = s[(i + 2) % s.length];
      const t = ((i * 0.618) % 1) * 0.6 + 0.2, x = lerp(a[0], b[0], t), y = lerp(a[1], b[1], t);
      ctx.moveTo(x - 0.14, y + 0.05); ctx.lineTo(x, y - 0.03); ctx.lineTo(x + 0.12, y + 0.04);
    }
    ctx.stroke();
  }

  /** Steam from an overheated / dead engine (world frame, above the hood). */
  _steam(v, hc) {
    const ctx = this.ctx, t = (this._steamT = (this._steamT || 0) + 0.016);
    const hb = this.box && (this.box.hood || this.box.frame), lx = hb ? lerp(hb.x0, hb.x1, 0.6) : 1.3, ly = hb ? hb.y1 - 0.02 : 0.75;   // from the bonnet
    const c = Math.cos(v.angle), s = Math.sin(v.angle), hx = v.x + c * lx - s * ly, hy = v.y + s * lx + c * ly;
    ctx.fillStyle = hc.engineHot ? 'rgba(235,235,235,0.35)' : 'rgba(60,60,60,0.25)';
    for (let i = 0; i < 4; i++) {
      const k = (t * 0.8 + i / 4) % 1;
      ctx.beginPath(); ctx.arc(hx - k * 0.25, hy + 0.1 + k * 1.1, 0.1 + k * 0.25, 0, TAU); ctx.fill();
    }
  }

  // -- particles ---------------------------------------------------------------------------------
  _spawn(kind, layer, col, x, y, vx, vy, life, size, grow, alpha, z = 0, vz = 0) {
    const f = this.fx, i = f.n;
    if (i >= this.q.fx) return;
    f.n++;
    f.kind[i] = kind; f.layer[i] = layer; f.col[i] = col; f.stuck[i] = 0;
    f.x[i] = x; f.y[i] = y; f.vx[i] = vx; f.vy[i] = vy; f.z[i] = z; f.vz[i] = vz;
    f.life[i] = f.max[i] = life; f.size[i] = size; f.grow[i] = grow; f.alpha[i] = alpha;
  }

  /** Emission from plain vehicle state: tyre roost, exhaust smoke, winch sparks. */
  _emit(v, dt) {
    const f = this.fx, R = Math.random, ca = Math.cos(v.angle), sa = Math.sin(v.angle);
    // tyre roost: scaled by slip speed (m/s) and wheel spin (rad/s * r)
    v.wheels.forEach((o, i) => {
      if (!o.grounded || !o.contacts.length) { f.acc[i] = 0; return; }
      const slip = Math.abs(o.slip), spin = Math.abs(o.omega) * o.radius, ex = slip - 0.45;
      if (ex <= 0) { f.acc[i] = 0; return; }
      const sf = this.surfFx[o.surface] || this.surfFx.dirt || { kind: K.DUST, col: 0 };
      f.acc[i] += dt * Math.min(42, ex * 9 + spin * 1.6);
      const c = o.contacts[0], dir = o.slip > 0 ? -1 : 1;              // spinning forward throws material back
      let nx = c.nx, ny = c.ny; if (ny < 0) { nx = -nx; ny = -ny; }
      const layer = o.side === 'L' ? 1 : 0, sp = Math.min(slip, 10), D = this.dims;
      const zl = o.z ?? (D ? (o.side === 'R' ? D.h : -D.h) : 0), zw = D ? D.tw : 0, zo = o.side === 'R' ? 1 : -1;   // 2.5D lane
      while (f.acc[i] >= 1) {
        f.acc[i] -= 1;
        if (f.n >= this.q.fx * 0.83) continue;
        const back = (1.2 + sp * 0.55) * (0.6 + R() * 0.7), up = (0.8 + R() * 2.2 + spin * 0.12);
        let vx = dir * ca * back + nx * up, vy = dir * sa * back + ny * up;
        const px = c.x + dir * ca * 0.12 + (R() - 0.5) * 0.15, py = c.y + 0.04 + R() * 0.06;
        const col = sf.col + (R() < 0.35 ? 1 : 0), pz = zl + (R() - 0.5) * zw, vz = zo * R() * 0.9 + (R() - 0.5) * 0.6;
        switch (sf.kind) {
          case K.BLOB: this._spawn(K.BLOB, layer, col, px, py, vx, vy, 0.9 + R() * 0.7, 0.05 + R() * 0.07, 0, 0.95, pz, vz); break;
          case K.CHIP: this._spawn(K.CHIP, layer, col, px, py, vx * 1.2, vy * 1.1, 0.6 + R() * 0.5, 0.03 + R() * 0.04, 0, 1, pz, vz * 1.2); break;
          case K.DROP: this._spawn(K.DROP, layer, 4, px, py, vx * 0.9, vy, 0.5 + R() * 0.4, 0.025 + R() * 0.03, 0, 0.8, pz, vz); break;
          default: this._spawn(K.DUST, layer, col, px, py, vx * 0.45, vy * 0.35, 0.9 + R() * 0.8, 0.1 + R() * 0.08, 0.55, 0.32, pz, vz * 0.5);
        }
      }
    });
    // exhaust: puffs from the stack tip with throttle, lugging, overheat or cranking
    const hc = v.hardcore || {}, st = this.stack, zs = (v.z || 0) + (this.dims ? (st && st.z != null ? st.z * this.dims.B : this.dims.zStack) : 0);
    if (st && (v.engineOn || hc.cranking)) {
      const thr = v.throttle || 0, lug = Math.min(1, Number.isFinite(hc.lug) ? hc.lug : 0);
      const rate = 0.8 + thr * 7 + lug * 12 + (hc.engineHot ? 5 : 0) + (hc.cranking ? 20 : 0);
      f.acc[4] += dt * rate;
      const tx = v.x + ca * st.x - sa * st.y1, ty = v.y + sa * st.x + ca * st.y1;
      const dark = thr > 0.6 || lug > 0.2 || hc.cranking;
      while (f.acc[4] >= 1) {
        f.acc[4] -= 1;
        this._spawn(K.SMOKE, 1, dark ? 0 : 1, tx, ty, v.vx * 0.4 - ca * 0.3 + (R() - 0.5) * 0.25, v.vy * 0.4 + 0.7 + R() * 0.5,
          0.9 + R() * 0.6, 0.05 + R() * 0.03, 0.22 + thr * 0.12, (0.05 + thr * 0.09 + lug * 0.14) * (hc.engineHot ? 1.3 : 1), zs, (R() - 0.5) * 0.3);
      }
    } else f.acc[4] = 0;
    // misfire: backfire pop (dark puff burst + a spit of flame) on each new misfire slot
    if (st && hc.misfire && !this._misPrev) {
      const tx = v.x + ca * st.x - sa * st.y1, ty = v.y + sa * st.x + ca * st.y1;
      for (let k = 0; k < 4; k++) this._spawn(K.SMOKE, 1, 0, tx, ty, v.vx * 0.4 - ca * 0.6 + (R() - 0.5) * 0.8, v.vy * 0.4 + 0.9 + R() * 0.9,
        0.8 + R() * 0.5, 0.08 + R() * 0.05, 0.5, 0.42, zs);
      for (let k = 0; k < 3; k++) this._spawn(K.SPARK, 1, 2 + (k & 1), tx, ty, -ca * (1 + R() * 2) + (R() - 0.5), 1.5 + R() * 2, 0.12 + R() * 0.1, 0.05, 0, 1, zs);
    }
    this._misPrev = !!hc.misfire;
    // winch sparks: cable tension above 75 % of the break limit
    const wn = v.winch, t = wn.attached && wn.breakTension > 0 ? wn.tension / wn.breakTension : 0;
    if (t > 0.75) {
      const k = Math.min(1, (t - 0.75) / 0.25);
      f.acc[5] += dt * (18 + 90 * k);
      const d = Math.hypot(wn.ax - wn.mx, wn.ay - wn.my), sag = Math.sqrt(Math.max(0, wn.length - d) * Math.max(d, 0.01)) * 0.45;
      const cx = (wn.mx + wn.ax) / 2, cy = (wn.my + wn.ay) / 2 - sag * 2;
      while (f.acc[5] >= 1) {
        f.acc[5] -= 1;
        const r = R(), u = r < 0.5 ? 1 : r < 0.75 ? 0 : R(), w0 = (1 - u) * (1 - u), w1 = 2 * u * (1 - u), w2 = u * u;
        const px = w0 * wn.mx + w1 * cx + w2 * wn.ax, py = w0 * wn.my + w1 * cy + w2 * wn.ay;
        const a = R() * TAU, s = 1.5 + R() * 3.5 * (0.6 + k);
        this._spawn(K.SPARK, 1, R() < 0.4 ? 3 : 2, px, py, Math.cos(a) * s, Math.sin(a) * s + 1, 0.18 + R() * 0.3, 0.035, 0, 1,
          lerp(v.z || 0, this.dims ? this.dims.zA : 0, u), (R() - 0.5) * s);   // cable runs z drum -> zA (anchor), linear in u
      }
    } else f.acc[5] = 0;
    this._trackEmit(v);
  }

  _stepFx(dt) {
    const f = this.fx;
    for (let i = 0; i < f.n; i++) {
      f.life[i] -= dt;
      if (f.life[i] <= 0) { this._kill(i--); continue; }
      const k = f.kind[i];
      if (f.stuck[i]) continue;
      const g = k === K.SMOKE ? 0.35 : k === K.DUST ? 0.6 : k === K.SPARK ? -6 : -9.81;
      const drag = k === K.SMOKE ? 1.6 : k === K.DUST ? 2.4 : k === K.SPARK ? 0.6 : 0.35;
      const dd = Math.exp(-drag * dt);
      f.vx[i] *= dd; f.vy[i] = f.vy[i] * dd + g * dt;
      f.vz[i] *= dd; f.x[i] += f.vx[i] * dt; f.y[i] += f.vy[i] * dt; f.z[i] += f.vz[i] * dt;
      f.size[i] += f.grow[i] * dt;
      if (k === K.SMOKE || k === K.DUST) continue;
      const gy = this._gy(f.x[i]);
      if (f.y[i] < gy) {
        if (k === K.BLOB) { f.stuck[i] = 1; f.y[i] = gy + f.size[i] * 0.25; f.life[i] = Math.min(f.life[i], 0.9); }
        else if (k === K.DROP) this._kill(i--);
        else { f.y[i] = gy; f.vy[i] = Math.abs(f.vy[i]) * 0.3; f.vx[i] *= 0.5; }
      }
    }
  }

  _kill(i) {
    const f = this.fx, j = --f.n;
    if (i === j) return;
    f.x[i] = f.x[j]; f.y[i] = f.y[j]; f.vx[i] = f.vx[j]; f.vy[i] = f.vy[j]; f.life[i] = f.life[j]; f.max[i] = f.max[j];
    f.size[i] = f.size[j]; f.grow[i] = f.grow[j]; f.alpha[i] = f.alpha[j]; f.kind[i] = f.kind[j]; f.layer[i] = f.layer[j];
    f.col[i] = f.col[j]; f.stuck[i] = f.stuck[j]; f.z[i] = f.z[j]; f.vz[i] = f.vz[j];
  }

  /** 2D: draw one layer (0 far, 1 near). 2.5D: pass z0..z1 -> particles whose depth lies in that slab, projected. */
  _drawFx(layer, z0, z1) {
    const ctx = this.ctx, f = this.fx, P = this.palette, d3 = z0 !== undefined;
    if (!f.n) return;
    ctx.save();
    for (let i = 0; i < f.n; i++) {
      if (d3 ? f.z[i] < z0 || f.z[i] >= z1 : f.layer[i] !== layer) continue;
      const k = f.kind[i], u = f.life[i] / f.max[i], s = f.size[i];
      const X = d3 ? f.x[i] + f.z[i] * C30 : f.x[i], Y = d3 ? f.y[i] - f.z[i] * S30 : f.y[i];
      let a = f.alpha[i] * (k === K.SMOKE || k === K.DUST ? Math.min(1, u * 1.6) * Math.min(1, (1 - u) * 8 + 0.3) : Math.min(1, u * 3));
      if (d3 ? f.z[i] < -0.3 : !layer) a *= 0.7;                    // far side reads a little dimmer
      if (a < 0.01) continue;
      ctx.globalAlpha = a;
      if (k === K.SPARK) {
        ctx.strokeStyle = P[f.col[i]]; ctx.lineWidth = s;
        ctx.beginPath(); ctx.moveTo(X, Y); ctx.lineTo(X - (f.vx[i] + (d3 ? f.vz[i] * C30 : 0)) * 0.04, Y - (f.vy[i] - (d3 ? f.vz[i] * S30 : 0)) * 0.04); ctx.stroke();
        continue;
      }
      ctx.fillStyle = P[f.col[i]];
      if (k === K.CHIP) { ctx.fillRect(X - s, Y - s, s * 2, s * 1.4); continue; }
      ctx.beginPath();
      if (k === K.BLOB && !f.stuck[i]) ctx.ellipse(X, Y, s * 1.25, s * 0.85, Math.atan2(f.vy[i], f.vx[i]), 0, TAU);
      else ctx.arc(X, Y, s, 0, TAU);
      ctx.fill();
    }
    ctx.restore();
  }

  // -- game feel: camera trauma, impact bursts, slip / stress / rope-snap cues (pure visuals from getState()) -------
  _kick(k) { const F = this.feel; if (k > 0) { F.trauma = Math.min(1, Math.max(F.trauma, k) + k * 0.25); try { this.onKick?.(k); } catch (_) { /* haptics optional */ } } }
  _flash(a, col) { const F = this.feel; if (a > F.flash) { F.flash = Math.min(0.6, a); F.flashCol = col; } }

  /** Screen-px camera offset: trauma² × amplitude × smooth two-sine noise (4-8 Hz); trauma decays ~0.55 s from full. */
  _shakeOff(dt) {
    const F = this.feel, o = this._sk || (this._sk = [0, 0]);
    F.t += dt; F.trauma = Math.max(0, F.trauma - dt * 1.8);
    // 4-8 Hz, <= 1.3 % of the short side: a heavy thump. (Was 7-15 Hz at 2.2 %: sampled at 60 fps that read as frame jitter.)
    const a = F.trauma * F.trauma * Math.min(this.w, this.h) * 0.013 * this.shakeScale, t = F.t;
    o[0] = a * (0.7 * Math.sin(t * 29.3 + F.ph[0]) + 0.3 * Math.sin(t * 47.9 + F.ph[1]));
    o[1] = a * (0.7 * Math.sin(t * 33.1 + F.ph[1]) + 0.3 * Math.sin(t * 51.7 + F.ph[0]));
    return o;
  }

  /** Rope vibration (m, added to the sag control point) once the cable passes 65 % of its break load. */
  _ropeHum(t) { return t > 0.65 ? Math.sin(this.feel.t * 95) * 0.06 * (t - 0.65) / 0.35 : 0; }

  /** Burst of surface flecks (and sparks on rock / metal) at a contact, count ∝ closing speed. */
  _burst(x, y, z, nx, ny, surf, vn, sparks) {
    const R = Math.random, sf = this.surfFx[surf] || { kind: K.DUST, col: 0 }, n = Math.min(12, Math.round(vn * 2.2));
    if (ny < 0) { nx = -nx; ny = -ny; }
    for (let k = 0; k < n && this.fx.n < this.q.fx - 8; k++) {
      const s = (0.8 + R() * 1.6) * Math.min(vn, 8) * 0.5, a = (R() - 0.5) * 2.2, cx = Math.cos(a), sx = Math.sin(a);
      const vx = (nx * cx - ny * sx) * s, vy = (nx * sx + ny * cx) * s, col = sf.col + (R() < 0.4 ? 1 : 0), vz = (R() - 0.5) * 1.5;
      if (sf.kind === K.DUST) this._spawn(K.DUST, 1, col, x, y + 0.05, vx * 0.5, vy * 0.4, 0.7 + R() * 0.6, 0.09 + R() * 0.08, 0.7, 0.4, z, vz);
      else this._spawn(sf.kind === K.DROP ? K.DROP : sf.kind, 1, sf.kind === K.DROP ? 4 : col, x, y + 0.05, vx, vy, 0.5 + R() * 0.5, 0.03 + R() * 0.04, 0, 1, z, vz);
    }
    if (sparks) for (let k = 0; k < Math.min(8, n); k++) {
      const a = R() * Math.PI, s = 2 + R() * 4;
      this._spawn(K.SPARK, 1, 2 + (k & 1), x, y + 0.05, Math.cos(a) * s, Math.sin(a) * s, 0.15 + R() * 0.2, 0.035, 0, 1, z, (R() - 0.5) * s);
    }
  }

  /** Edge-detect heavy hits, bottom-outs, axle / rope snaps, rollover and new damage from plain state -> trauma, flash, bursts. */
  _feel(v, dt, d3) {
    const F = this.feel, C = F.cool, hc = v.hardcore || {}, D = this.dims, R = Math.random;
    for (let i = 0; i < C.length; i++) C[i] = Math.max(0, C[i] - dt);
    if (!dt) return;
    const zOf = o => (d3 ? o.z ?? v.z ?? 0 : 0);
    // chassis strike (bottoming the frame, nosing a ledge): vn > 1.6 m/s
    let cv = 0, cc = null;
    for (const c of v.chassisContacts || []) if (c.vn > cv) { cv = c.vn; cc = c; }
    if (cc && cv > 1.6 && C[4] <= 0) {
      C[4] = 0.15; this._kick((cv - 1.2) / 5);
      this._burst(cc.x, cc.y, d3 ? v.z || 0 : 0, cc.nx, cc.ny, cc.surface, cv, /rock|stone|boulder|gravel/.test(cc.surface || '') || cv > 4);
    }
    v.wheels.forEach((o, i) => {
      // tyre landing / rock strike: vn > 2.4 m/s
      let wv = 0, wc = null;
      for (const c of o.contacts) if (c.vn > wv) { wv = c.vn; wc = c; }
      if (wc && wv > 2.4 && C[i] <= 0) { C[i] = 0.18; this._kick((wv - 2) / 9); this._burst(wc.x, wc.y, zOf(o), wc.nx, wc.ny, wc.surface, wv * 0.8, false); }
      // bottom-out: suspension slams into the bump stop (rising edge, fast compression)
      const cp = o.compression ?? 0;
      if (cp >= 0.96 && F.comp[i] < 0.96 && Math.abs(o.susSpeed || 0) > 0.6 && C[i] <= 0.1) { this._kick(0.1 + Math.min(0.3, Math.abs(o.susSpeed) * 0.08)); C[i] = 0.18; }
      F.comp[i] = cp;
      // digging a rut: dark clods pop up slowly around the sinking tyre
      const sf = this.surfFx[o.surface];
      if (o.grounded && sf && sf.dig > 0 && Math.abs(o.slip) > 1 && R() < dt * 14 * sf.dig && this.fx.n < this.q.fx * 0.83 && o.contacts[0]) {
        const c = o.contacts[0], zw = D ? D.tw : 0;
        this._spawn(K.BLOB, 1, sf.col + 1, c.x + (R() - 0.5) * 0.4, c.y + 0.03, (R() - 0.5) * 0.8, 0.6 + R() * 1.1, 0.7 + R() * 0.4,
          0.04 + R() * 0.05, 0, 1, zOf(o) + (R() - 0.5) * zw, (R() - 0.5) * 0.4);
      }
    });
    // axle shaft snap: hard jolt, white-hot flash, spark shower at the axle
    const ab = hc.axleBroken || [];
    for (let a = 0; a < 2; a++) {
      if (ab[a] && !F.axle[a]) {
        this._kick(0.8); this._flash(0.45, '255,200,120');
        const W = v.wheels.filter(o => o.axle === a);
        if (W.length) { const x = W.reduce((s, o) => s + o.x, 0) / W.length, y = W.reduce((s, o) => s + o.y, 0) / W.length;
          this._burst(x, y - 0.1, d3 ? v.z || 0 : 0, 0, 1, 'rock', 7, true); }
      }
      F.axle[a] = !!ab[a];
    }
    // winch cable snap: jolt + both rope ends whip back
    const wn = v.winch;
    if (wn.broken && !F.wBroken && F.wPrev) {
      this._kick(0.6); this._flash(0.25, '255,230,150');
      F.whip = { ...F.wPrev, p: 0 };
      this._burst(F.wPrev.ax, F.wPrev.ay, F.wPrev.za, 0, 1, 'rock', 5, true);
    }
    F.wBroken = !!wn.broken;
    if (wn.attached) { const w0 = F.wPrev || (F.wPrev = {}); w0.mx = wn.mx; w0.my = wn.my; w0.ax = wn.ax; w0.ay = wn.ay;
      w0.zm = d3 ? v.z || 0 : 0; w0.za = d3 && D ? D.zA : 0; }
    else if (!wn.broken) F.wPrev = null;
    if (F.whip && (F.whip.p += dt / 0.5) >= 1) F.whip = null;
    // rollover onto the roof
    if (v.upsideDown && !F.upside) this._kick(0.5);
    F.upside = !!v.upsideDown;
    // new body / shock damage: red edge flash ∝ the damage just taken
    const dmg = (hc.body || 0) + (hc.shock ? hc.shock.reduce((s, x) => s + x, 0) : 0);
    if (F.dmg >= 0 && dmg > F.dmg + 0.004) { this._flash(Math.min(0.5, 0.12 + (dmg - F.dmg) * 5), '255,60,40'); this._kick(Math.min(0.35, (dmg - F.dmg) * 4)); }
    F.dmg = dmg;
    F.flash = Math.max(0, F.flash - dt * 2.2);
  }

  /** World-space overlays (wheel-spin arcs, axle strain glow, rope whip) + screen-space flash / stress vignette. */
  _feelDraw(v, d3) {
    const ctx = this.ctx, F = this.feel, hc = v.hardcore || {}, t = F.t, D = this.dims, tf = this._T;
    ctx.save();
    ctx.lineCap = 'round';
    // wheel spin / lock-up cues on the camera-side tyres (2D: L = near; 2.5D: +lz = near)
    for (const o of v.wheels) {
      if (d3 ? !(o.lz > 0) : o.side !== 'L') continue;
      const s = o.slip || 0, a = clamp((Math.abs(s) - 1.1) / 3, 0, 1);
      if (!o.grounded || a <= 0) continue;
      ctx.save();
      if (d3 && D && tf) {
        const st = o.steer || 0, cs = Math.cos(st), ss = Math.sin(st), Ax = -ss + cs * tf.Dx, Ay = cs * tf.Dy;
        this._tz(0);
        ctx.translate((o.hx ?? o.x) + o.lz * tf.Dx + D.tw / 2 * Ax, (o.hy ?? o.y) + o.lz * tf.Dy + D.tw / 2 * Ay);
        ctx.transform(cs + ss * tf.Dx, ss * tf.Dy, 0, 1, 0, 0);
      } else ctx.translate(o.x, o.y);
      const r = o.radius * 1.16;
      if (s > 0) {                                           // wheelspin: amber arcs whirling with the tyre
        ctx.strokeStyle = `rgba(255,190,60,${(0.35 + 0.5 * a).toFixed(3)})`; ctx.lineWidth = 0.035 + 0.03 * a;
        for (let k = 0; k < 3; k++) { const b = o.angle + k * TAU / 3; ctx.beginPath(); ctx.arc(0, 0, r, b, b + 0.5 + 0.7 * a); ctx.stroke(); }
      } else {                                               // lock-up / slide: pale streaks trailing at the contact patch
        ctx.strokeStyle = `rgba(230,240,255,${(0.3 + 0.45 * a).toFixed(3)})`; ctx.lineWidth = 0.03;
        const dir = v.speed >= 0 ? -1 : 1, L = 0.25 + 0.5 * a;
        for (let k = 0; k < 3; k++) { const y = -o.radius + 0.04 + k * 0.07; ctx.beginPath(); ctx.moveTo(dir * 0.1, y); ctx.lineTo(dir * (0.1 + L * (1 - k * 0.25)), y); ctx.stroke(); }
      }
      ctx.restore();
    }
    if (d3) this._z(0);
    // axle strain: pulsing red glow at the diff, faster as it nears the snap; snapped = static crack mark
    const as = hc.axleStress || [], ab = hc.axleBroken || [];
    for (let a = 0; a < 2; a++) {
      const W = v.wheels.filter(o => o.axle === a); if (!W.length) continue;
      const x = W.reduce((s, o) => s + o.x, 0) / W.length, y = W.reduce((s, o) => s + o.y, 0) / W.length;
      const z = d3 ? W.reduce((s, o) => s + (o.z ?? 0), 0) / W.length : 0, X = x + z * C30, Y = y - z * S30;
      if (ab[a]) {
        ctx.strokeStyle = 'rgba(255,70,50,0.9)'; ctx.lineWidth = 0.05; ctx.beginPath();
        ctx.moveTo(X - 0.18, Y + 0.12); ctx.lineTo(X - 0.05, Y - 0.02); ctx.lineTo(X + 0.04, Y + 0.08); ctx.lineTo(X + 0.18, Y - 0.08); ctx.stroke();
      } else if (as[a] > 0.55) {
        const k = (as[a] - 0.55) / 0.45, p = 0.5 + 0.5 * Math.sin(t * (7 + 16 * k)), rr = 0.25 + 0.25 * k;
        const gr = ctx.createRadialGradient(X, Y, 0, X, Y, rr);
        gr.addColorStop(0, `rgba(255,${Math.round(150 - 110 * k)},40,${(0.35 + 0.45 * k * p).toFixed(3)})`); gr.addColorStop(1, 'rgba(255,60,30,0)');
        ctx.fillStyle = gr; ctx.beginPath(); ctx.arc(X, Y, rr, 0, TAU); ctx.fill();
      }
    }
    // snapped rope: both ends recoil along a decaying wave
    const wp = F.whip;
    if (wp) {
      const e = 1 - (1 - wp.p) * (1 - wp.p), fade = 1 - wp.p;
      const ends = [[wp.mx, wp.my, wp.zm, wp.ax, wp.ay, wp.za], [wp.ax, wp.ay, wp.za, wp.mx, wp.my, wp.zm]];
      ctx.strokeStyle = `rgba(235,80,50,${fade.toFixed(3)})`; ctx.lineWidth = 0.05;
      for (const [x0, y0, z0, x1, y1, z1] of ends) {
        const L = (1 - e) * 0.5;                                // each half shrinks back toward its own end
        ctx.beginPath();
        for (let k = 0; k <= 12; k++) {
          const u = (k / 12) * L, w = Math.sin(k * 1.3 + wp.p * 30) * 0.25 * fade * (k / 12);
          const x = lerp(x0, x1, u), y = lerp(y0, y1, u) + w, z = d3 ? lerp(z0, z1, u) : 0;
          k ? ctx.lineTo(x + z * C30, y - z * S30) : ctx.moveTo(x + z * C30, y - z * S30);
        }
        ctx.stroke();
      }
    }
    ctx.restore();
    // screen space: hit flash + amber stress pulse at the screen edge when any drivetrain part is near failure
    const stress = Math.max(as[0] || 0, as[1] || 0, hc.xferHeat || 0, hc.transTemp || 0, hc.engineHeat || 0);
    const sp = stress > 0.8 ? (stress - 0.8) / 0.2 * (0.5 + 0.5 * Math.sin(t * 9)) * 0.16 : 0;
    if (F.flash > 0.01 || sp > 0.01) {
      ctx.save(); ctx.setTransform(1, 0, 0, 1, 0, 0);
      const { w, h } = this, gr = ctx.createRadialGradient(w / 2, h / 2, Math.min(w, h) * 0.35, w / 2, h / 2, Math.hypot(w, h) * 0.55);
      const [col, a] = F.flash > sp ? [F.flashCol, F.flash] : ['255,150,30', sp];
      gr.addColorStop(0, `rgba(${col},0)`); gr.addColorStop(1, `rgba(${col},${a.toFixed(3)})`);
      ctx.fillStyle = gr; ctx.fillRect(0, 0, w, h);
      ctx.restore();
    }
  }

  // -- tyre tracks / skid marks --------------------------------------------------------------------
  _trackEmit(v) {
    const T = this.tracks;
    v.wheels.forEach((o, i) => {
      const last = T.last[i] || (T.last[i] = { x: 0, y: 0, ok: false });
      const soft = /mud|clay|bog|dirt/.test(o.surface || '');
      const slip = Math.abs(o.slip);
      if (!o.grounded || !o.contacts.length || slip < (soft ? 0.3 : 0.7)) { last.ok = false; return; }
      let c = null;                                                  // terrain contacts only (no logs / planks / rocks)
      for (const q of o.contacts) if ((q.terrain ?? Math.abs(q.y - this._gy(q.x)) < 0.08) && (!c || q.force > c.force)) c = q;
      if (!c) { last.ok = false; return; }
      const d = Math.hypot(c.x - last.x, c.y - last.y);
      if (last.ok && d < 0.1) return;
      if (last.ok && d < 0.8) {
        const j = T.head; T.head = (j + 1) % TRACK_MAX;
        T.x0[j] = last.x; T.y0[j] = last.y; T.x1[j] = c.x; T.y1[j] = c.y; T.born[j] = this.t;
        T.a[j] = clamp(0.22 + slip * 0.06, 0.22, 0.6) * (o.side === 'L' ? 1 : 0.7);
        T.w[j] = (soft ? 0.12 : 0.07) * (o.side === 'L' ? 1 : 0.8); T.s[j] = o.side === 'R' ? 1 : 0; T.z[j] = o.z || 0;
      }
      last.x = c.x; last.y = c.y; last.ok = true;
    });
  }

  _tracks(v, cam, viewW) {
    const ctx = this.ctx, T = this.tracks, xl = cam.x - viewW, xr = cam.x + viewW;
    ctx.save(); ctx.strokeStyle = '#140e0a'; ctx.lineCap = 'butt';
    for (let j = 0; j < TRACK_MAX; j++) {
      const age = this.t - T.born[j];
      if (age > TRACK_LIFE || age < 0) continue;
      if (T.x1[j] < xl || T.x0[j] > xr) continue;
      ctx.globalAlpha = T.a[j] * (1 - age / TRACK_LIFE);
      ctx.lineWidth = T.w[j];
      ctx.beginPath(); ctx.moveTo(T.x0[j], T.y0[j] - 0.015); ctx.lineTo(T.x1[j], T.y1[j] - 0.015); ctx.stroke();
    }
    ctx.restore();
  }

  // -- debug overlay -----------------------------------------------------------------------------
  _debugWorld(v) {
    const ctx = this.ctx, N = 1 / 6000;           // metres of arrow per newton
    const arrows = (list, col) => {
      ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = 0.035;
      for (const c of list) {
        ctx.beginPath(); ctx.arc(c.x, c.y, 0.06, 0, TAU); ctx.fill();
        ctx.beginPath(); ctx.moveTo(c.x, c.y); ctx.lineTo(c.x + c.nx * c.force * N, c.y + c.ny * c.force * N); ctx.stroke();
      }
    };
    for (const o of v.wheels) arrows(o.contacts, o.side === 'L' ? '#35f08a' : '#19a8ff');   // wheel contacts + normal force
    arrows(v.chassisContacts, '#ff4040');                                                    // body / roof contacts
    for (const o of v.wheels) {                    // suspension travel bars (L = left bar, R = right bar)
      const bx = o.x + (o.axle === 0 ? 0.62 : -0.74) + (o.side === 'R' ? 0.1 : 0), by = o.y - 0.3, c = Math.max(0, Math.min(1, o.compression));
      ctx.fillStyle = 'rgba(0,0,0,0.55)'; ctx.fillRect(bx, by, 0.08, 0.6);
      ctx.fillStyle = c > 0.9 ? '#ff4040' : (o.side === 'L' ? '#35f08a' : '#19a8ff'); ctx.fillRect(bx, by, 0.08, 0.6 * c);
    }
    const r = 0.13;                                // centre of mass
    ctx.lineWidth = 0.03; ctx.strokeStyle = '#000'; ctx.fillStyle = '#ffe14a';
    ctx.beginPath(); ctx.arc(v.com.x, v.com.y, r, 0, TAU); ctx.fill(); ctx.stroke();
    ctx.fillStyle = '#000';
    ctx.beginPath(); ctx.moveTo(v.com.x, v.com.y); ctx.arc(v.com.x, v.com.y, r, 0, Math.PI / 2); ctx.closePath(); ctx.fill();
    ctx.beginPath(); ctx.moveTo(v.com.x, v.com.y); ctx.arc(v.com.x, v.com.y, r, Math.PI, Math.PI * 1.5); ctx.closePath(); ctx.fill();
  }

  _debugText(v, info) {
    const ctx = this.ctx, d = this.dpr, fs = 7.5 * d, lh = fs * 1.08;
    const fin = n => (Number.isFinite(n) ? n : 0), f0 = n => fin(n).toFixed(0), f1 = n => fin(n).toFixed(1), pad = (s, n) => String(s).padStart(n);
    const lines = [
      `RPM ${pad(f0(v.rpm), 4)} Te ${pad(f0(v.engineTorque), 3)} Tw ${pad(f0(v.wheelTorque), 5)} N·m ${v.gear} ${v.fourWD ? '4WD' : '2WD'}${v.lockF ? ' FL' : ''}${v.lockR ? ' RL' : ''}`,
      `v ${pad(f1(v.speed), 5)} m/s pitch ${pad(f0(v.angle * 57.2958), 4)}° CoM ${f1(v.com.x)},${f1(v.com.y)}`,
      `z ${pad((v.z || 0).toFixed(2), 5)} m yaw ${pad(f0((v.yaw || 0) * 57.2958), 3)}° steer ${pad(f0((v.steer || 0) * 57.2958), 3)}°`,
      'whl  w r/s   T Nm  N kN slip susp surf',
    ];
    for (const o of v.wheels) {
      lines.push(`${o.name.replace('front', 'F').replace('rear', 'R').padEnd(3)} ${pad(f1(o.omega), 6)} ${pad(f0(o.driveTorque), 6)} ${pad(f1(o.load / 1000), 5)} ${pad(f1(o.slip), 4)} ${pad(f0(o.compression * 100), 3)}% ${o.surface || 'air'} z${(o.z || 0).toFixed(2)}`);
    }
    const h = v.hardcore;
    if (h) lines.push(`AX F ${f0(h.axleStress[0] * 100)}%${h.axleBroken[0] ? ' SNAP' : ''} R ${f0(h.axleStress[1] * 100)}%${h.axleBroken[1] ? ' SNAP' : ''} XF ${f0(h.xferHeat * 100)}%${h.xferHot ? ' HOT' : ''} ENG ${f0(h.engineHeat * 100)}%${h.engineHot ? ' HOT' : ''}${h.stalled ? ' STALL' : ''} lug ${f0(h.lug * 100)}%`,
      `BODY ${f0(h.body * 100)}% shk ${h.shock.map(d => f0(d * 100)).join('/')}% pwr ${f0(h.power * 100)}%`);
    const wn = v.winch;
    lines.push(wn.attached
      ? `WINCH ${f1(wn.length)}m T ${f1(wn.tension / 1000)} pk ${f1(wn.peakTension / 1000)} stall ${f0(wn.maxPull / 1000)} brk ${f0(wn.breakTension / 1000)} kN`
      : `WINCH ${wn.broken ? 'ROPE BROKEN' : 'free'} (max ${f0(wn.maxLength)}m)`);
    lines.push(`fps ${f0(info.fps || 0)} fx ${this.fx.n}/${FX_MAX} steps ${info.steps ?? 0} t ${f1(info.time || 0)}s`);
    ctx.font = `${fs}px ui-monospace, Menlo, Consolas, monospace`;
    ctx.textBaseline = 'top';
    let wmax = 0;
    for (const l of lines) wmax = Math.max(wmax, ctx.measureText(l).width);
    // tucked under the top-left MENU / RESET bar; if it would run into the bottom-left thumb cluster (steering over BRAKE · WINCH · REV)
    // it moves right of that column. info.dodge = hud.dodge {top, right} (CSS px, live control sizes); fallback mirrors style.css bases.
    const W = this.w / d, H = this.h / d, vmin = Math.min(W, H) / 100, boxH = lines.length * lh / d + 4, top = 56;
    const dg = info.dodge, stackTop = dg ? dg.top : H - (10 + clamp(22 * vmin, 74, 128) + 10 + clamp(32 * vmin, 104, 176));
    const colRight = dg ? dg.right : 10 + clamp(22 * vmin, 74, 128) * 0.8 + 2 * clamp(14.5 * vmin, 50, 72) + 20;
    const x = (top + boxH + 6 > stackTop ? colRight + 12 : 8) * d, y = top * d;
    ctx.fillStyle = 'rgba(8,10,14,0.55)'; ctx.fillRect(x - 3 * d, y - 2 * d, wmax + 6 * d, lines.length * lh + 4 * d);
    ctx.fillStyle = '#d7f7e2';
    lines.forEach((l, i) => ctx.fillText(l, x, y + i * lh));
  }
}
