// engine/renderer.js — 2D side-view / 2.5D oblique canvas renderer + debug overlay.
// Consumes ONLY plain data: PhysicsWorld.getStatic()/getDynamic() and Vehicle.getShape()/getState().
// It never imports or calls the physics engine (trail.js is shared pure geometry: berms, zone dy, rut-grid lanes).
import { TRAIL, laneOffset, rutAt, laneZ, zoneAtZ } from './trail.js';

const TAU = Math.PI * 2;
const BODY_COLORS = { frame: '#2a2d33', hood: '#d4622b', cab: '#d4622b', bed: '#b9531f' };
const lerp = (a, b, t) => a + (b - a) * t;
const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
// 2.5D camera: 3/4 rear-right oblique, depth z = vehicle right (+z = near side, R wheels), screen = (x + z·cos30°, y − z·sin30°)
const C30 = Math.cos(Math.PI / 6), S30 = 0.5;
const cmpItem = (a, b) => (a.x1 <= b.x0 ? 1 : b.x1 <= a.x0 ? -1 : a.ym - b.ym);   // same slab: nearer (−x) / higher last
// slab draw order: truck parts by local z; a plane lying on the slab's far bound (za) is behind every ranged piece of it
const atFar = (it, za) => it.za === it.zb && it.za - za < 0.02;
// screen boxes (z = 0 frame: x + z·cos30, y − z·sin30): prop p's box swept over depths a..b (a hexagon) vs truck part
// t's silhouette box hb -- separating axes x, y and the sweep normal (sin30, cos30), on which the sweep adds nothing
const overlap = (p, a, b, t) => { const h = t.hb;
  return p.x1 + b * C30 > h[0] && p.x0 + a * C30 < h[2] && p.y1 - a * S30 > h[1] && p.y0 - b * S30 < h[3]
    && p.x1 * S30 + p.y1 * C30 > h[0] * S30 + h[1] * C30 && p.x0 * S30 + p.y0 * C30 < h[2] * S30 + h[3] * C30; };
/** Convex hull of a flat [x, y, ...] point list (monotone chain) -> flat CCW list. */
function hull(F) {
  const n = F.length / 2, idx = Array.from({ length: n }, (_, i) => i).sort((i, j) => F[2 * i] - F[2 * j] || F[2 * i + 1] - F[2 * j + 1]);
  const cr = (o, a, b) => (F[2 * a] - F[2 * o]) * (F[2 * b + 1] - F[2 * o + 1]) - (F[2 * a + 1] - F[2 * o + 1]) * (F[2 * b] - F[2 * o]);
  const lo = [], up = [];
  for (const i of idx) { while (lo.length > 1 && cr(lo[lo.length - 2], lo[lo.length - 1], i) <= 0) lo.pop(); lo.push(i); }
  for (let k = n - 1; k >= 0; k--) { const i = idx[k]; while (up.length > 1 && cr(up[up.length - 2], up[up.length - 1], i) <= 0) up.pop(); up.push(i); }
  const out = [];
  for (const i of lo.slice(0, -1).concat(up.slice(0, -1))) out.push(F[2 * i], F[2 * i + 1]);
  return out;
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
const FX_MAX = 180;                 // hard cap on live particles (mobile budget)
const FX_ROOST_MAX = 150;           // roost may not starve exhaust / sparks
const TRACK_MAX = 360;              // skid-mark segments in the ring buffer
// atmosphere presets (renderer.setAtmosphere; random per playthrough): sky gradient, 4 parallax ridges far->near,
// screen tint over the world, sun / moon, stars + headlights at night
export const ATMOS = {
  day:      { sky: ['#82b1da', '#dde8d6', '#d3c79f'], ridges: ['#adc3cf', '#93afab', '#6f957a', '#4a7450'], tint: null, sun: [0.8, 0.14, 0.045, '#fff7dc'] },
  dusk:     { sky: ['#3a4985', '#e48a62', '#f6c686'], ridges: ['#9a84a6', '#74658e', '#524d70', '#38404e'], tint: 'rgba(255,130,60,0.13)', sun: [0.18, 0.5, 0.07, '#ffd08a'] },
  overcast: { sky: ['#8f9ba6', '#c3cac9', '#b9b6a6'], ridges: ['#a8b2b6', '#8f9e9a', '#738a7f', '#557062'], tint: 'rgba(110,122,132,0.16)', sun: null },
  night:    { sky: ['#060a1a', '#18223e', '#2a3146'], ridges: ['#28304a', '#1f273d', '#171d2d', '#11161f'], tint: 'rgba(6,12,36,0.52)', sun: [0.74, 0.16, 0.035, '#e8eef8'],
              stars: true, lights: true },
};
// foliage palettes: dark forest greens + bright parrot-green accents (so the vegetation reads at a glance)
const FOREST = ['#1f4219', '#295221', '#33652c', '#193815', '#40733a'], PARROT = ['#66cc30', '#7fdc42', '#55bd2b', '#93e352'];
const hashStr = t => { let h = 2166136261; for (let i = 0; i < t.length; i++) { h ^= t.charCodeAt(i); h = Math.imul(h, 16777619); } return h >>> 0; };
const rngOf = seed => { let a = seed || 1; return () => { a = (a + 0x6D2B79F5) | 0; let t = Math.imul(a ^ (a >>> 15), 1 | a); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; };
const h01 = (i, k = 0) => { const v = Math.sin(i * 127.1 + k * 311.7) * 43758.5453; return v - Math.floor(v); };   // stable per-index noise
const CLS = new Map();   // surface name -> slab-detail class 0 plain 1 rock 2 wet 3 water 4 gravel (regexes once per name)
const surfCls = n => { let c = CLS.get(n); if (c === undefined) CLS.set(n, c = isWater(n) ? 3 : isWet(n) ? 2 : isRock(n) ? 1 : isGravel(n) ? 4 : 0); return c; };
const isRock = n => /rock|stone|boulder/.test(n), isGravel = n => /gravel|scree/.test(n), isWet = n => /mud|bog|clay|slick|swamp/.test(n), isWater = n => /ford|water|river/.test(n);

const RIVER = '#357f8c';                                           // gorge river surface
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
const mixHex = (h, to, t) => { const a = hexRgb(h), b = hexRgb(to);   // hex result (safe to feed _shades)
  return '#' + a.map((v, i) => Math.round(lerp(v, b[i], t)).toString(16).padStart(2, '0')).join(''); };

export class Renderer {
  constructor(canvas, opts = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d', { alpha: false });
    this.viewHeight = opts.viewHeight || 9.5;   // metres visible vertically (landscape)
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
    this.resize();
  }

  resize() {
    const c = this.canvas, dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.max(1, Math.round(c.clientWidth * dpr)), h = Math.max(1, Math.round(c.clientHeight * dpr));
    if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
    this.w = w; this.h = h; this.dpr = dpr;
    this.ppm = Math.min(h / this.viewHeight, w / this.minViewWidth);
  }

  setLevel(level) {
    this.level = level;
    this._gradeMap = new Map();
    this.segs = [];
    this.trail = level.trail || TRAIL;
    this.dims = null;
    this.dyZones = (level.zones || []).filter(z => z.dy);
    this._laneUse = new Uint8Array(this.trail.nz);
    // own copy of the ground: base = authored run (resampled), points = 2D side-view profile (under the near tyres,
    // re-shaped from dyn.ruts + lane offsets), ya / yb = 2.5D slab-bound profiles (segs share the points arrays)
    this.ground = level.ground.map(r => ({ surface: r.surface, points: r.points.map(p => [p[0], p[1]]), d: null,
      churn: mix((level.surfaces[r.surface] && level.surfaces[r.surface].color) || '#777', '#0b0604', 0.6),
      base: r.points.map(p => [p[0], p[1]]), ya: new Float64Array(r.points.length), yb: new Float64Array(r.points.length),
      i0: 0, i1: 0, vis: false }));
    for (const g of this.ground) { const c = this._surfColor(g.surface); g.top = mix(c, '#ffffff', 0.12); g.lane = mix(c, '#000000', 0.1); g.cap = mix(c, '#000000', 0.45);
      g.riser = mix(c, '#000000', 0.22); g.riserL = mix(c, '#000000', 0.3); g.lit = mix(c, '#ffffff', 0.24);
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
    this.shape = shape;
    const pt = shape.paint || {};
    this.colors = { ...BODY_COLORS, ...(pt.body ? { hood: pt.body, cab: pt.body } : {}), ...(pt.bed ? { bed: pt.bed } : {}), ...(pt.frame ? { frame: pt.frame } : {}),
      body: pt.body || BODY_COLORS.cab, trim: pt.trim || '#2a2c30', accent: pt.accent || '#16171a' };
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

  groundY(x) {
    let y = -Infinity;
    for (const [a, b] of this.segs) {
      const x0 = Math.min(a[0], b[0]), x1 = Math.max(a[0], b[0]);
      if (x < x0 || x > x1 || x1 - x0 < 1e-6) continue;
      const v = lerp(a[1], b[1], (x - a[0]) / (b[0] - a[0]));
      if (v > y) y = v;
    }
    return y === -Infinity ? 0 : y;
  }

  // -- frame -------------------------------------------------------------------------------------
  /** frame = { vehicle, dyn, debug, dt, cam?: '2D' | '2.5D' (default 2D), info: {fps, steps, time} } */
  draw(frame) {
    const { ctx, w, h, ppm } = this, v = frame.vehicle, cam = this.cam;
    const z3 = frame.cam === '2.5D' ? v.z || 0 : 0;                 // 2.5D: follow the truck across the trail too
    const tx = v.x + 1.6 + Math.max(-3, Math.min(3, v.speed * 0.35)) + z3 * C30, ty = v.y + 0.8 - z3 * S30;
    if (!cam.ready) { cam.x = tx; cam.y = ty; cam.ready = true; }
    const k = 1 - Math.exp(-(frame.dt || 0.016) * 5);
    cam.x += (tx - cam.x) * k; cam.y += (ty - cam.y) * k;
    const fdt = clamp(frame.dt || 0, 0, 0.05), d3 = frame.cam === '2.5D';
    if (this.level) this._feel(v, fdt, d3);                          // impact / snap detection -> camera trauma + bursts
    const sk = this._shakeOff(fdt);
    const ox = w * 0.46 - cam.x * ppm + sk[0], oy = h * 0.6 + cam.y * ppm + sk[1];

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
      this._draw3d(L, v, frame.dyn || { logs: [], planks: [] });
      this._z(0);
      this._atmoPost(v, true);
      this._feelDraw(v, true);
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
    if (frame.debug) {
      this._debugWorld(v);
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      this._debugText(v, frame.info || {});
    }
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
    for (let x = x0 + 1; x < x1; x += 0.9 + R() * 2.2) {           // far edge: pines, bushes, ferns, grass
      const k = R(), parrot = R() < 0.32;
      fl.push({ x, far: true, kind: k < 0.3 ? 'pine' : k < 0.65 ? 'bush' : k < 0.82 ? 'fern' : 'grass', s: 0.7 + R() * 0.6,
        col: parrot && k >= 0.3 ? pick(PARROT) : pick(FOREST), acc: pick(PARROT), seed: R() * 100, dz: R() * 0.5 });
      if (k < 0.3 && R() < 0.5) fl.push({ x: x + 0.4, far: true, kind: R() < 0.5 ? 'fern' : 'grass', s: 0.6 + R() * 0.4, col: pick(PARROT), acc: pick(FOREST), seed: R() * 100, dz: 0.55 });
    }
    for (let x = x0 + 2; x < x1; x += 1.6 + R() * 3.5) {           // near edge: low grass / ferns only (never hides the truck)
      const parrot = R() < 0.55;
      fl.push({ x, far: false, kind: R() < 0.6 ? 'grass' : 'fern', s: 0.45 + R() * 0.3, col: parrot ? pick(PARROT) : pick(FOREST), acc: pick(PARROT), seed: R() * 100, dz: 0 });
    }
    for (const f of fl) { f.lit = mix(f.col, '#ffffff', f.kind === 'pine' ? 0.12 : 0.22); f.base = mix(f.col, '#000000', 0.25); }   // shades once, not per frame
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
      if (b - a > 0.3) this.rivers.push({ x0: a - 0.05, x1: b + 0.05, y: wl });
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
      if (li >= 2) {                                                 // pine silhouettes along the ridge line
        const sp = (li === 2 ? 0.012 : 0.02) * h, th = (li === 2 ? 0.025 : 0.045) * h;
        for (let k = Math.floor(-off / sp) - 1, n = 0; n < w / sp + 3; k++, n++) {
          const x = k * sp + off - Math.floor(off / sp) * sp - off + ((k * sp) % sp), xs = k * sp + off - off;   // world-anchored spacing
          const X = ((k * sp - (off % sp)) + sp * 4) - sp * 4, r = h01(k, li);
          if (r < 0.25) continue;
          const y = yAt(X), hh = th * (0.6 + r * 0.7);
          ctx.beginPath(); ctx.moveTo(X - sp * 0.42, y + 2); ctx.lineTo(X, y - hh); ctx.lineTo(X + sp * 0.42, y + 2); ctx.closePath(); ctx.fill();
        }
      }
      ctx.translate(0, lift); ctx.fillStyle = G.haze[li]; ctx.fillRect(0, h * base, w, h * 0.12); ctx.translate(0, -lift);
    }
  }

  /** Lighting pass over the finished world (screen space tint) + night headlights from the truck nose. */
  _atmoPost(v, d3) {
    const ctx = this.ctx, A = this.atmo;
    if (!A.tint) return;
    ctx.save(); ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = A.tint; ctx.fillRect(0, 0, this.w, this.h);
    ctx.restore();
    if (!A.lights || !this.shape) return;
    if (d3) this._tz(0); else this._z(0);
    const wm = this.shape.winchMount, c = Math.cos(v.angle), s = Math.sin(v.angle), on = v.engineOn !== false;
    ctx.save(); ctx.globalCompositeOperation = 'lighter';
    const zs = d3 && this.dims ? [-this.dims.B * 0.7, this.dims.B * 0.7] : [0];
    for (const lz of zs) {
      const D = d3 ? this._T : null, lx = v.x + c * wm[0] - s * (wm[1] + 0.35) + (D ? lz * D.Dx : 0), ly = v.y + s * wm[0] + c * (wm[1] + 0.35) + (D ? lz * D.Dy : 0);
      const L = 10, sp = 0.28, g = ctx.createRadialGradient(lx, ly, 0.1, lx, ly, L);
      g.addColorStop(0, on ? 'rgba(255,240,200,0.32)' : 'rgba(255,240,200,0.05)'); g.addColorStop(1, 'rgba(255,240,200,0)');
      ctx.fillStyle = g; ctx.beginPath(); ctx.moveTo(lx, ly);
      ctx.lineTo(lx + L * Math.cos(v.angle - sp), ly + L * Math.sin(v.angle - sp)); ctx.lineTo(lx + L * Math.cos(v.angle + sp * 0.5), ly + L * Math.sin(v.angle + sp * 0.5)); ctx.closePath(); ctx.fill();
    }
    const glow = ctx.createRadialGradient(v.x, v.y, 0.3, v.x, v.y, 3.2);                // ambient spill so the rig stays readable
    glow.addColorStop(0, 'rgba(150,170,210,0.14)'); glow.addColorStop(1, 'rgba(150,170,210,0)');
    ctx.fillStyle = glow; ctx.fillRect(v.x - 3.2, v.y - 3.2, 6.4, 6.4);
    ctx.restore(); this._z(0);
  }

  /** One flora plant at ground (x, y), world units (y up), scale s; t = time (sway). */
  _plant(f, x, y, far) {
    const ctx = this.ctx, s = f.s, sw = Math.sin(this.t * 1.3 + f.seed) * 0.04 * s;
    if (f.kind === 'pine') {
      const hgt = 2.1 * s + 0.9;
      ctx.lineWidth = 0.16 * s; ctx.strokeStyle = '#4a3420'; ctx.beginPath(); ctx.moveTo(x, y - 0.1); ctx.lineTo(x, y + hgt * 0.35); ctx.stroke();
      const dark = f.col, lit = f.lit;
      for (let i = 0; i < 4; i++) {
        const yb = y + hgt * (0.2 + i * 0.19), wd = (1.05 - i * 0.2) * s, yt = yb + hgt * 0.34, k = sw * (i + 1) * 0.6;
        ctx.fillStyle = dark; ctx.beginPath(); ctx.moveTo(x - wd, yb); ctx.lineTo(x + k, yt); ctx.lineTo(x + wd, yb); ctx.closePath(); ctx.fill();
        ctx.fillStyle = lit; ctx.beginPath(); ctx.moveTo(x + k, yt); ctx.lineTo(x + wd, yb); ctx.lineTo(x + wd * 0.35, yb + 0.05); ctx.closePath(); ctx.fill();
      }
      return;
    }
    if (f.kind === 'bush') {
      const r = 0.38 * s, base = f.base, lit = f.lit;
      const blobs = [[-0.9, 0.5, 0.85], [0.8, 0.55, 0.8], [0, 0.95, 1], [-0.4, 1.35, 0.7], [0.5, 1.3, 0.65]];
      ctx.fillStyle = base; for (const [bx, by, br] of blobs) { ctx.beginPath(); ctx.arc(x + bx * r + sw * by, y + by * r, br * r, 0, TAU); ctx.fill(); }
      ctx.fillStyle = f.col; for (const [bx, by, br] of blobs) { ctx.beginPath(); ctx.arc(x + bx * r - 0.06 + sw * by, y + by * r + 0.07, br * r * 0.78, 0, TAU); ctx.fill(); }
      ctx.fillStyle = lit; ctx.beginPath(); ctx.arc(x - 0.25 * r + sw, y + 1.25 * r, 0.32 * r, 0, TAU); ctx.fill();
      if (h01(f.seed) < 0.4) { ctx.fillStyle = f.acc; ctx.beginPath(); ctx.arc(x + 0.7 * r + sw, y + 0.9 * r, 0.3 * r, 0, TAU); ctx.fill(); }
      return;
    }
    ctx.lineCap = 'round';
    if (f.kind === 'fern') {
      ctx.strokeStyle = f.col; ctx.lineWidth = 0.05 * s;
      for (let i = 0; i < 7; i++) {
        const a = Math.PI * (0.15 + 0.7 * i / 6), L = (0.55 + 0.35 * Math.sin(i * 1.7 + f.seed)) * s;
        const ex = x + Math.cos(a) * L + sw * 2, ey = y + Math.sin(a) * L * 0.9;
        ctx.beginPath(); ctx.moveTo(x, y); ctx.quadraticCurveTo(x + Math.cos(a) * L * 0.5, y + Math.sin(a) * L * 1.05, ex, ey); ctx.stroke();
      }
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
      for (let i = i0; i < i1; i++) {                                // pebbles in the dirt cut
        const [x, y] = top(i), [x2, y2] = top(i + 1);
        for (let j = 0; j < 2; j++) {
          const u = h01(i, j + 11), d = 0.25 + 1.6 * h01(i, j + 21), r = 0.04 + 0.07 * h01(i, j + 31);
          ctx.fillStyle = h01(i, j + 41) < 0.5 ? 'rgba(0,0,0,0.22)' : 'rgba(255,240,220,0.14)';
          ctx.beginPath(); ctx.ellipse(lerp(x, x2, u), lerp(y, y2, u) - d, r * 1.4, r, 0, 0, TAU); ctx.fill();
        }
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

  /** Wooden stage post (world units, y up): post + sign board reading the stage name. */
  _stagePost(x, gy, name) {
    const ctx = this.ctx;
    ctx.lineWidth = 0.16; ctx.strokeStyle = '#5b4026';
    ctx.beginPath(); ctx.moveTo(x, gy - 0.1); ctx.lineTo(x, gy + 1.9); ctx.stroke();
    ctx.fillStyle = '#9a7850'; ctx.strokeStyle = '#3d2a17'; ctx.lineWidth = 0.05;
    ctx.fillRect(x - 0.75, gy + 1.45, 1.5, 0.5); ctx.strokeRect(x - 0.75, gy + 1.45, 1.5, 0.5);
    ctx.save(); ctx.translate(x, gy + 1.7); ctx.scale(0.01, -0.01);
    ctx.fillStyle = '#fff2d0'; ctx.font = 'bold 30px system-ui, sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(name, 0, 0); ctx.restore();
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
    let xf = -Infinity, xr = Infinity;
    for (const o of W) { const lx = ((o.hx ?? o.x) - v.x) * ca + ((o.hy ?? o.y) - v.y) * sa; xf = Math.max(xf, lx); xr = Math.min(xr, lx); }
    const h = sh.halfTrack || clamp(0.3 * (xf - xr), 0.75, 1.15), tw = sh.tyreWidth || clamp(r * 0.8, 0.32, 0.55);   // half track, tyre width (m)
    const B = h - tw / 2 - 0.1, D = T.edge + T.bermW + 0.45;
    return { h, tw, B, D, sz: h - tw / 2 - 0.05, W: T.edge + 0.3, zA: -D + 0.35, zStack: -B * 0.55 };
  }

  _z(z) { this.ctx.setTransform(this.ppm, 0, 0, -this.ppm, this._ox + z * C30 * this.ppm, this._oy + z * S30 * this.ppm); }

  /** Truck frame at local depth lz: pitched chassis coords (world x, y as the 2D view draws them) -> yawed, z-offset screen. */
  _tz(lz) {
    const t = this._T;
    this._z(0);
    this.ctx.transform(t.a, t.b, 0, 1, t.xc * (1 - t.a) + t.zc * C30 + lz * t.bx, -t.b * t.xc - t.zc * S30 + lz * t.by);
  }

  /** Per frame: affine coefficients of the truck frame from heading yaw and trail z (see _tz). */
  _truckFrame(v) {
    const y = v.yaw || 0, c = Math.cos(y), s = Math.sin(y), t = this._T || (this._T = {});
    t.a = c + s * C30; t.b = -s * S30; t.bx = C30 * c - s; t.by = -c * S30; t.xc = v.x; t.zc = v.z || 0;
    t.Dx = t.bx / t.a; t.Dy = t.by - t.b * t.Dx;                     // local depth vector in pre-transform coords
    t.cy = c; t.sy = s; t.ang = v.angle;
  }

  _shades(col) {
    let s = this._shadeMap.get(col);
    if (!s) this._shadeMap.set(col, s = { cap: col, top: mix(col, '#ffffff', 0.2), side: mix(col, '#000000', 0.15), rear: mix(col, '#000000', 0.32),
      edge: mix(col, '#000000', 0.45) });   // opaque edge: props are re-stroked every slab, alpha would stack to black
    return s;
  }

  /** Extruded polygon (flat xy array P, n verts) between depths za..zb along this._dz: front faces (mode & 1), near cap (mode & 2). */
  _prism(P, n, za, zb, sh, mode = 3, edge = 'rgba(0,0,0,0.4)') {
    const ctx = this.ctx, Dx = this._dz[0], Dy = this._dz[1], ax = za * Dx, ay = za * Dy, bx = zb * Dx, by = zb * Dy;
    let area = 0;
    for (let i = 0; i < n; i++) { const j = (i + 1) % n; area += P[2 * i] * P[2 * j + 1] - P[2 * j] * P[2 * i + 1]; }
    const sg = area > 0 ? 1 : -1;
    ctx.lineWidth = 0.02; ctx.strokeStyle = edge;
    if (mode & 1) for (let i = 0; i < n; i++) {
      const j = (i + 1) % n, x0 = P[2 * i], y0 = P[2 * i + 1], x1 = P[2 * j], y1 = P[2 * j + 1], dx = x1 - x0, dy = y1 - y0;
      if (sg * (dx * Dy - dy * Dx) <= 0) continue;                   // faces away from the camera
      const l = Math.hypot(dx, dy) || 1, ny = -sg * dx / l, nx = sg * dy / l;
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
    zs.push(-D, -(e + T.bermW), -(e + T.bermW * 0.5), -e, e, e + T.bermW * 0.5, e + T.bermW, D);
    const c = this._T, B = dm.B * c.cy;
    zs.push(c.zc - B, c.zc + B);                                     // body plane between the two tyre groups
    for (const o of v.wheels) zs.push(o.z - dm.tw / 2, o.z + dm.tw / 2);
    for (const q of this.dyZones) if (q.x1 >= xl && q.x0 <= xr) {
      const lo = q.side === 'R' ? 0 : q.z0, hi = q.side === 'L' ? 0 : q.z1;
      for (const zz of [lo, hi]) if (Math.abs(zz) < D) zs.push(zz - T.zRamp / 2, zz + T.zRamp / 2);
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
    for (let i = 0; i < zs.length; i++) { const z = clamp(zs[i], -D, D); if (!k || z - zs[k - 1] > 0.015) zs[k++] = z; }
    zs.length = k;
    return zs;
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
      for (const g of this.runOrder) if (g.vis) {
        g.ya = g.pb[k]; g.yb = g.pb[k + 1];
        this._terrainSlab(g, g.ya, g.yb, za, zb, last, bottom, k === 0);
      }
      this._zones3d(L, za, zb, xl, xr, k === 0);
      for (const g of this.runOrder) if (g.vis) this._slabDetail(g, g.ya, g.yb, za, zb);
      this._tracks3d(zlo, zhi, xl, xr);
      if (L.finish && L.finish.x > xl && L.finish.x < xr) this._finishStrip(L.finish.x, za, zb, dm.D);
      const list = lists[k], n0 = done.length;
      for (const it of list) {
        const b = Math.min(it.zb, zb);
        if (it.prop) {
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
    let any = 0;
    for (let i = i0; i < i1 && i - i0 < 4096; i++) {
      let c = sc[i];
      const cand = onTrail && szc[i];
      if (cand) for (const q of cand) if (zoneAtZ(q, zm)) { c = surfCls(q.surface); break; }
      any |= cls[i - i0] = c;
    }
    if (!any) return;
    const P = (x, i, z, o) => {                                      // surface point at x in segment i, depth z -> screen (pre-transform), into o
      const t = (x - p[i][0]) / Math.max(1e-6, p[i + 1][0] - p[i][0]), u = (z - za) / dz;
      o[0] = x + z * C30; o[1] = lerp(lerp(ya[i], ya[i + 1], t), lerp(yb[i], yb[i + 1], t), u) - z * S30; return o;
    };
    const PA = this._pa, PB = this._pb;
    ctx.beginPath();
    {                                                                // rock: block joints across the strip, offset per strip
      const row = Math.floor((zm + 5) * 1.6);
      for (let i = i0; i < i1; i++) {
        const x0 = p[i][0], x1 = p[i + 1][0];
        if (cls[i - i0] !== 1 || x1 - x0 < 0.2) continue;
        const n = Math.max(1, Math.round((x1 - x0) / 0.9));
        for (let j = 0; j < n; j++) {
          const x = x0 + (x1 - x0) * (j + 0.25 + 0.5 * h01(i * 13 + j, row)) / n, a = P(x, i, za, PA), b = P(x, i, zb, PB);
          ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]);
        }
      }
      ctx.lineWidth = 0.025; ctx.strokeStyle = 'rgba(0,0,0,0.2)'; ctx.stroke();
    }
    {                                                                // gravel / scree: loose stones (light + shadow dots)
      const row = Math.floor((zm + 5) * 5), ns = Math.min(3, Math.round(3 * dz / 0.4));   // stones ∝ strip width (thin tyre strips: few)
      if (ns > 0) for (const [col, k0] of GRAVEL_DOTS) {
        ctx.fillStyle = col; ctx.beginPath();
        for (let i = i0; i < i1; i++) {
          if (cls[i - i0] !== 4) continue;
          for (let j = 0; j < ns; j++) {
            const u = h01(i * 5 + j + k0, row), z = lerp(za, zb, h01(i * 5 + j + k0, row + 3)), pt = P(lerp(p[i][0], p[i + 1][0], u), i, z, PA), r = 0.025 + 0.03 * h01(j + k0, i);
            ctx.moveTo(pt[0] + r, pt[1] + 0.01); ctx.ellipse(pt[0], pt[1] + 0.01, r * 1.3, r * 0.7, 0, 0, TAU);
          }
        }
        ctx.fill();
      }
    }
    {                                                                // wet: sheen streaks along the strip
      ctx.beginPath();
      const row = Math.floor((zm + 5) * 4);
      for (let i = i0; i < i1; i++) {
        if (cls[i - i0] !== 2 || h01(i, row) < 0.55) continue;
        const x0 = p[i][0], x1 = p[i + 1][0], u = h01(i, row + 7) * 0.6, a = P(lerp(x0, x1, u), i, zm, PA), b = P(lerp(x0, x1, Math.min(1, u + 0.4)), i, zm, PB);
        ctx.moveTo(a[0], a[1] + 0.006); ctx.lineTo(b[0], b[1] + 0.006);
      }
      ctx.lineWidth = 0.03; ctx.strokeStyle = 'rgba(225,235,245,0.16)'; ctx.stroke();
      if (g.pudA !== this.atmoName) { g.pud = mix(g.top, this.atmo.sky[1], 0.5); g.pudA = this.atmoName; }
      const pc = g.pud;
      for (const q of onTrail ? this.puddles : []) {
        if (q.x + q.rx < p[i0][0] || q.x - q.rx > p[i1][0] || q.z + q.rz < za || q.z - q.rz > zb) continue;
        if (!isWet(surfAt(q.x, this._gy(q.x), q.z))) continue;      // a rock lane zone here: no puddle
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
          ctx.lineWidth = 0.03; ctx.strokeStyle = 'rgba(255,255,255,0.4)'; ctx.stroke();
        }
      }
    }
    if (!isWater(g.surface)) return;
    const t = this.t, row = Math.floor((zm + 5) * 3);                // water: drifting ripples, foam on the banks
    ctx.beginPath();
    for (let i = i0; i < i1; i++) {
      const x0 = p[i][0], x1 = p[i + 1][0];
      for (let k = 0; k < 2; k++) {
        const u = (h01(i * 3 + k, row) + t * 0.06 * (k + 1)) % 1, x = lerp(x0, x1, u), a = P(Math.max(x0, x - 0.18), i, zm, PA), b = P(Math.min(x1, x + 0.18), i, zm, PB);
        ctx.moveTo(a[0], a[1] + 0.01); ctx.quadraticCurveTo((a[0] + b[0]) / 2, (a[1] + b[1]) / 2 + 0.035, b[0], b[1] + 0.01);
      }
    }
    ctx.lineWidth = 0.028; ctx.strokeStyle = 'rgba(225,245,250,0.55)'; ctx.stroke();
    ctx.fillStyle = 'rgba(240,250,250,0.8)';                         // bank foam: both banks in one fill (they never overlap)
    const n = p.length - 1;
    ctx.beginPath();
    for (let e = 0; e < 2; e++) {
      const i = e ? n - 1 : 0, x = e ? p[n][0] - 0.12 : p[0][0] + 0.12;
      if (i < i0 || i >= i1) continue;
      for (let k = 0; k < 3; k++) {
        const pt = P(x + (k - 1) * 0.12, i, zm, PA), r = Math.max(0.02, 0.05 + 0.04 * Math.sin(t * 3 + k * 2 + zm * 5));
        ctx.moveTo(pt[0] + r, pt[1] + 0.02); ctx.arc(pt[0], pt[1] + 0.02, r, 0, TAU);
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
    const cols = [g.top, g.riser, g.lit, g.riserL];
    let cls = -1;
    ctx.beginPath();
    for (let i = i1 - 1; i >= i0; i--) {
      const x0 = p[i][0], x1 = p[i + 1][0], dx = x1 - x0;
      // screen-space winding of the quad: visible when it turns clockwise (y-up world frame)
      const Ax = x0 + ax, Ay = ya[i] + ay, Bx = x1 + ax, By = ya[i + 1] + ay, Cx = x1 + bx, Cy = yb[i + 1] + by, Dx = x0 + bx, Dy = yb[i] + by;
      if ((Cx - Ax) * (Dy - By) - (Cy - Ay) * (Dx - Bx) >= 0) continue;
      // riser = steep authored segment: decided once per segment (g.rise), never per slab, so a step that sits on the
      // threshold cannot flicker between shades from one depth slab to the next
      const sz = ((yb[i] + yb[i + 1]) - (ya[i] + ya[i + 1])) * 0.5 / dz;
      const c = g.rise[i] ? 1 : sz < -0.35 ? 2 : sz > 0.35 ? 3 : 0;
      if (c !== cls) { if (cls >= 0) { ctx.fillStyle = cols[cls]; ctx.fill(); ctx.beginPath(); } cls = c; }
      quad(i, fx, fy);
    }
    if (cls >= 0) { ctx.fillStyle = cols[cls]; ctx.fill(); }
    const d = g.d;
    if (d) {                                                         // churned rut floor: darker with depth (slab centre)
      const zm = (za + zb) / 2, T = this.trail;
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
        ctx.beginPath();                                             // wet sheen along the rut floors
        const ox = zm * C30, oy = -zm * S30;
        for (let i = i0; i < i1; i++) {
          if (Math.min(rutAt(T, d, i, zm), rutAt(T, d, i + 1, zm)) < 0.04) continue;
          ctx.moveTo(p[i][0] + ox, (ya[i] + yb[i]) / 2 + oy + 0.008); ctx.lineTo(p[i + 1][0] + ox, (ya[i + 1] + yb[i + 1]) / 2 + oy + 0.008);
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
        ctx.beginPath(); let any = false;
        const p = g.base, ya = g.ya, yb = g.yb;                      // this slab's cached profiles (_draw3d)
        for (let i = g.i0; i < g.i1; i++) {
          const a = p[i], b = p[i + 1], dx = b[0] - a[0];
          if (dx < 1e-6 || b[0] <= x0 || a[0] >= x1 || C30 * (b[1] - a[1]) + S30 * dx <= 0) continue;
          if ((a[1] + b[1]) / 2 < z.y0 || (a[1] + b[1]) / 2 > z.y1) continue;
          const u0 = Math.max(0, (x0 - a[0]) / dx), u1 = Math.min(1, (x1 - a[0]) / dx);
          const px0 = lerp(a[0], b[0], u0), px1 = lerp(a[0], b[0], u1);
          ctx.moveTo(px0 + ax, lerp(ya[i], ya[i + 1], u0) + ay); ctx.lineTo(px1 + ax, lerp(ya[i], ya[i + 1], u1) + ay);
          ctx.lineTo(px1 + bx, lerp(yb[i], yb[i + 1], u1) + by); ctx.lineTo(px0 + bx, lerp(yb[i], yb[i + 1], u0) + by); ctx.closePath();
          any = true;
        }
        if (!any) continue;
        // opaque premix of the old 85 % overlay on the run's top shade: alpha strips would seam at every slab bound
        let zc = this._zcol.get(z); if (!zc) this._zcol.set(z, zc = new Map());   // object-keyed: no per-frame string keys
        let col = zc.get(g);
        if (!col) zc.set(g, col = mix(this._surfColor(z.surface), g.top, 0.15));
        ctx.fillStyle = col; ctx.fill();
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
    // gorge rivers (opaque: props redraw every slab), drawn under the bridge deck
    for (const r of this.rivers) add(-dm.D, dm.D, r.x0, r.x1, r.y - 0.3, (a, b) => {
      ctx.fillStyle = RIVER; ctx.beginPath();
      ctx.moveTo(r.x0 + a * C30, r.y - a * S30); ctx.lineTo(r.x1 + a * C30, r.y - a * S30); ctx.lineTo(r.x1 + b * C30, r.y - b * S30); ctx.lineTo(r.x0 + b * C30, r.y - b * S30);
      ctx.closePath(); ctx.fill();
      this._ripples(r, a, b, -dm.D + 0.3);
    }, undefined, true, r.y - 1, r.y + 0.1);
    // bridge planks: dark stringer plank + round logs laid across the trail (end grain on the near end)
    for (const p of dyn.planks) {
      const pts = [[-p.hw, -p.hh], [p.hw, -p.hh], [p.hw, p.hh * 0.2], [-p.hw, p.hh * 0.2]], lr = Math.min(0.11, p.hw / 3);
      const n = Math.max(1, Math.round(p.hw / lr)), c = Math.cos(p.a), sn = Math.sin(p.a), ac = Math.abs(c), as = Math.abs(sn);
      const ey0 = as * p.hw + ac * p.hh + 0.05, ey1 = as * p.hw + ac * (0.2 * p.hh + 1.8 * lr) + 0.05;   // rotated plank + logs (x0 / x1 stay ±hw: draw order)
      add(-dm.W, dm.W, p.x - p.hw, p.x + p.hw, p.y, (a, b, it) => {
        this._prism(this._xf(pts, p.x, p.y, p.a), 4, a, b, this._shades('#4f3a26'), it.mid ? 1 : 3, '#2e2116');
        ctx.lineCap = 'butt';
        const near = b >= it.zb - 1e-9 && !it.mid, q = this._cull;
        for (let k = 0; k < n; k++) {                                // per log (the next log overlaps it): body, bark highlight, end grain
          const u = -p.hw + lr + k * (2 * p.hw - 2 * lr) / Math.max(1, n - 1), x = p.x + c * u - sn * (p.hh * 0.2 + lr * 0.8), y = p.y + sn * u + c * (p.hh * 0.2 + lr * 0.8);
          const X0 = x + a * C30, Y0 = y - a * S30, X = x + b * C30, Y = y - b * S30;
          if (q && (X + lr < q[0] || X0 - lr > q[2] || Y0 + lr < q[1] || Y - lr > q[3])) continue;   // outside the scissor
          ctx.lineWidth = lr * 2; ctx.strokeStyle = k & 1 ? '#7a5a38' : '#86643f';
          ctx.beginPath(); ctx.moveTo(X0, Y0); ctx.lineTo(X, Y); ctx.stroke();
          ctx.lineWidth = lr * 0.5; ctx.strokeStyle = 'rgba(255,225,180,0.18)';
          ctx.beginPath(); ctx.moveTo(X0, Y0 + lr * 0.45); ctx.lineTo(X, Y + lr * 0.45); ctx.stroke();
          if (near) {
            ctx.fillStyle = '#b08a5a'; ctx.beginPath(); ctx.arc(X, Y, lr * 0.95, 0, TAU); ctx.fill();
            ctx.lineWidth = 0.012; ctx.strokeStyle = '#6b4c2c'; ctx.beginPath(); ctx.arc(X, Y, lr * 0.55, 0, TAU); ctx.stroke();
          }
        }
        ctx.lineCap = 'round';
      }, undefined, true, p.y - ey0, p.y + ey1);
    }
    // bridge posts + sagging rope rails on both edges
    for (const br of L.bridges || []) for (const side of [-1, 1]) {
      const z = side * (dm.W - 0.05), xa = br.from[0] + 0.15, xb = br.to[0] - 0.15, ya = br.from[1], yb = br.to[1];
      const posts = [[xa, ya]]; const nm = Math.floor((xb - xa) / 4);
      for (let k = 1; k <= nm; k++) { const u = k / (nm + 1); posts.push([lerp(xa, xb, u), lerp(ya, yb, u) - 0.15]); }
      posts.push([xb, yb]);
      add(z, z, xa - 0.2, xb + 0.2, Math.max(ya, yb), () => {
        this._z(z);
        ctx.lineCap = 'round';
        for (const [x, y] of posts) {
          ctx.lineWidth = 0.17; ctx.strokeStyle = '#5c4128'; ctx.beginPath(); ctx.moveTo(x, y - 0.6); ctx.lineTo(x, y + 1.0); ctx.stroke();
          ctx.fillStyle = '#a5804f'; ctx.beginPath(); ctx.arc(x, y + 1.0, 0.085, 0, TAU); ctx.fill();
        }
        ctx.lineWidth = 0.035; ctx.strokeStyle = '#c2a26e'; ctx.beginPath();
        for (let k = 1; k < posts.length; k++) {
          const [x0, y0] = posts[k - 1], [x1, y1] = posts[k];
          ctx.moveTo(x0, y0 + 0.85); ctx.quadraticCurveTo((x0 + x1) / 2, (y0 + y1) / 2 + 0.5, x1, y1 + 0.85);
        }
        ctx.stroke();
        this._z(0);
      });
    }
    // trailside flora: far edge (pines / bushes / ferns / grass), near edge (low tufts only)
    for (const f of this.flora) {
      if (f.x < xl - 3 || f.x > xr + 3) continue;
      const z = f.far ? -dm.D + 0.08 + f.dz * 0.5 : dm.D - 0.12, y = this._surfY(f.x, z);
      if (!Number.isFinite(y)) continue;
      add(z, z, f.x - 1.2, f.x + 1.2, y, () => { this._z(z); this._plant(f, f.x, y - 0.02, f.far); this._z(0); });
    }
    // logs (cylinders across the trail) and loose boulders (extruded chunks)
    const LW = T.edge + T.bermW * 0.5;
    dyn.logs.forEach((l, i) => {
      if (defs[i] && /rock|stone/.test(defs[i].surface || '')) {
        const pts = []; for (let k = 0; k < 9; k++) { const a = k * TAU / 9, q = l.r * (0.9 + 0.1 * Math.sin(k * 2.7 + i)); pts.push([Math.cos(a) * q, Math.sin(a) * q]); }
        add(-LW * 0.8, LW * 0.8, l.x - l.r, l.x + l.r, l.y, (a, b, it) => { const sh = this._shades(this._surfColor(defs[i].surface)); this._prism(this._xf(pts, l.x, l.y, l.a), 9, a, b, sh, it.mid ? 1 : 3, sh.edge); }, undefined, true, l.y - l.r, l.y + l.r);
        return;
      }
      add(-LW, LW, l.x - l.r, l.x + l.r, l.y, (a, b, it) => {
        ctx.lineCap = 'round'; ctx.lineWidth = l.r * 2; ctx.strokeStyle = '#6e5235';
        ctx.beginPath(); ctx.moveTo(l.x + a * C30, l.y - a * S30); ctx.lineTo(l.x + b * C30, l.y - b * S30); ctx.stroke();
        const ox = S30 * l.r * 0.45, oy = C30 * l.r * 0.45;           // bark highlight on the upper flank
        ctx.lineWidth = l.r * 0.45; ctx.strokeStyle = 'rgba(255,225,180,0.16)';
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
      const far = o.lz < 0, zw = far ? o.z : Math.max(o.z, zBody);
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
      this._body3(v);
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
    this._links(o, hx, hy, this._T.ang || 0);
    const q = this._coilTmp || (this._coilTmp = {});
    q.x = hx; q.y = hy; q.mountX = o.mountX; q.mountY = o.mountY;
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
    ctx.fillStyle = far ? '#121315' : '#1b1c1f';
    for (let k = 0; k <= 5; k++) {                                   // tread: discs swept along the axle (far -> camera side)
      const w = out * (k / 5 - 0.5) * tw;
      ctx.save(); ctx.translate(hx + w * Ax, hy + w * Ay); ctx.transform(fa, fb, 0, 1, 0, 0);
      ctx.beginPath(); ctx.arc(0, 0, r, 0, TAU); ctx.fill(); ctx.restore();
    }
    const tr = this.look ? this.look.tread : { lugs: 12, depth: 0.05, kind: 'lug' }, nL = tr.lugs, pad = tr.kind === 'paddle', blk = tr.kind === 'block';
    ctx.lineCap = 'butt'; ctx.lineWidth = clamp(0.075 * Math.sqrt(tr.depth / 0.05) * 12 / nL, 0.05, 0.11); ctx.strokeStyle = far ? '#26272a' : '#3a3c41';
    ctx.beginPath();
    const a0 = -tw / 2 + 0.03, a1 = tw / 2 - 0.03;
    for (let k = 0; k < nL; k++) {
      const t = o.angle + k * TAU / nL, cx = Math.cos(t), sy = Math.sin(t);
      if (cx * (sn - C30 * ct) + S30 * sy < 0.12) continue;          // lug on the hidden half of the tread
      const u = cx * (r - 0.012), v = sy * (r - 0.012), px = hx + fa * u, py = hy + v + fb * u;
      if (pad) {                                                     // paddle chevron: V pointing along the roll
        const ru = -sy * 0.09, rv = cx * 0.09, mx = px + fa * ru, my = py + rv + fb * ru;
        ctx.moveTo(px + a0 * Ax, py + a0 * Ay); ctx.lineTo(mx, my); ctx.lineTo(px + a1 * Ax, py + a1 * Ay);
      } else if (blk) { const b0 = k & 1 ? a0 : -0.04, b1 = k & 1 ? 0.04 : a1;   // staggered blocks
        ctx.moveTo(px + b0 * Ax, py + b0 * Ay); ctx.lineTo(px + b1 * Ax, py + b1 * Ay);
      } else { ctx.moveTo(px + a0 * Ax, py + a0 * Ay); ctx.lineTo(px + a1 * Ax, py + a1 * Ay); }
    }
    ctx.stroke(); ctx.lineCap = 'round';
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
    for (const f of this.frames) this._prism(this._xf(f.points, v.x, v.y, v.angle), f.points.length, -B * 0.8, B * 0.8, this._shades(this.colors.frame), 3, 'rgba(0,0,0,0.5)');
    this._rig(v, 0, true); this._tz(0);
    for (const p of this.panelOrder) {
      this._prism(this._xf(p.points, v.x, v.y, v.angle), p.points.length, -B, B, this._shades(this.colors[p.kind] || '#c8572a'), 1, 'rgba(0,0,0,0.5)');
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
    for (const p of this.panels) this._prism(this._xf(p.points, v.x, v.y, v.angle), p.points.length, -B, B, this._shades(this.colors[p.kind] || '#c8572a'), 2, 'rgba(0,0,0,0.5)');
    this._tz(B); ctx.save(); ctx.translate(v.x, v.y); ctx.rotate(v.angle);
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
    const r = this._rT, D0 = this._dz; this._dz = [r.Dx, r.Dy];
    this._prism(this._xf(pts, r.x, r.y, r.a), pts.length, z0, z1, this._shades(col), 3, this._shades(col).edge);
    this._dz = D0;
  }
  _disc(lx, ly, lz, rad, col) { const ctx = this.ctx; ctx.beginPath(); ctx.arc(this._rx(lx, ly, lz), this._ry(lx, ly, lz), rad, 0, TAU); ctx.fillStyle = col; ctx.fill(); }
  _lamp(lx, ly, lz, rad, lens = '#fff2c4') { this._disc(lx, ly, lz, rad, '#1a1b1e'); this._disc(lx, ly, lz, rad * 0.72, this.colors.trim); this._disc(lx, ly, lz, rad * 0.5, lens); }
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
        if (r.rear) for (const z of [-B * 0.9, B * 0.9]) this._rfill([[xr, bd.y1 - 0.14, z - 0.06], [xr, bd.y1 - 0.14, z + 0.06], [xr, bd.y1 - 0.04, z + 0.06], [xr, bd.y1 - 0.04, z - 0.06]], 0, '#b0201c');
        this._spareRear(xr, (bd.y0 + bd.y1) / 2 + 0.08, B * 0.3, R * 0.82, 0.24);
      }
      if (hd) this._lamp(hd.x1 - 0.08, hd.y1 - 0.12, B + 0.02, 0.08);
    } else if (st === 'bogger') {
      if (pass !== 1 || !cb) return;
      this._spatter(B, Math.min(0.75, this.window ? Math.min(...this.window.map(p => p[1])) - 0.04 : 0.75), 70, 'rgba(86,58,34,0.6)');
      ctx.lineWidth = 0.02; ctx.strokeStyle = 'rgba(0,0,0,0.35)';                       // door seam
      this._rpath([[cb.x1 - 0.1, cb.y0 + 0.08], [cb.x0 + 0.12, cb.y0 + 0.08], [cb.x0 + 0.12, cb.y0 + 0.62], [cb.x1 - 0.1, cb.y0 + 0.62]], B, true); ctx.stroke();
      this._rslab([[cb.x1 - 0.02, cb.y0 + 0.6], [cb.x1 + 0.1, cb.y0 + 0.6], [cb.x1 + 0.1, cb.y0 + 0.74], [cb.x1 - 0.02, cb.y0 + 0.74]], B, B + 0.14, '#1d1e22');   // mirror
      if (bd) {                                                                          // bed roll bar + braces
        const xb = bd.x1 - 0.22, yt = cb.y1 - 0.08;
        for (const z of zs) { this._tube([[xb, bd.y1], [xb, yt], [bd.x0 + 0.5, bd.y1]], z * B * 0.82, 0.08, C.frame); if (z < 0) this._tube([[xb, yt, -B * 0.82], [xb, yt, B * 0.82]], 0, 0.08, C.frame); }
        if (r.rear) for (const z of [-B * 0.9, B * 0.9]) this._rfill([[bd.x0, bd.y1 - 0.2, z - 0.07], [bd.x0, bd.y1 - 0.2, z + 0.07], [bd.x0, bd.y1 - 0.05, z + 0.07], [bd.x0, bd.y1 - 0.05, z - 0.07]], 0, '#b0201c');
      }
      const cx = poly('cab').filter(p => p[1] > cb.y1 - 0.01).reduce((m, p) => Math.max(m, p[0]), cb.x0) - 0.08, ly = cb.y1 + 0.07;   // roof light bar + 4 pods
      this._tube([[cx, ly, -B * 0.8], [cx, ly, B * 0.8]], 0, 0.06, C.frame);
      for (let i = 0; i < 4; i++) { const z = lerp(-0.6, 0.6, i / 3) * B; if (!d3 && i < 3) continue; this._tube([[cx, ly], [cx, ly + 0.1]], z, 0.035, C.frame); this._lamp(cx + 0.02, ly + 0.15, z, 0.08); }
      if (hd) this._lamp(hd.x1 - 0.06, hd.y1 - 0.16, B + 0.02, 0.07, '#ffb347');
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
        this._lamp(hd.x1 - 0.1, hd.y0 + 0.2, B + 0.01, 0.065);
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
    const lk = this.look || { tread: { lugs: 12, depth: 0.05, kind: 'lug' }, rim: '#b8bcc4' }, tr = lk.tread, n = tr.lugs, d = tr.depth;
    const lw = clamp(TAU * r / n * (tr.kind === 'paddle' ? 0.62 : 0.48), 0.06, 0.24);
    ctx.fillStyle = far ? '#17181a' : '#232427';
    for (let i = 0; i < n; i++) {             // tread lugs (block: alternating long / short)
      ctx.rotate(TAU / n);
      const dd = tr.kind === 'block' && i & 1 ? d * 0.6 : d;
      ctx.fillRect(r - dd, -lw / 2, 2 * dd, lw);
    }
    ctx.beginPath(); ctx.arc(0, 0, r - 0.015, 0, TAU); ctx.fillStyle = far ? '#101113' : '#1a1b1e'; ctx.fill();
    ctx.lineWidth = 0.025; ctx.strokeStyle = far ? '#1c1d20' : '#34363b';
    ctx.beginPath(); ctx.arc(0, 0, r * 0.78, 0, TAU); ctx.stroke();
    ctx.beginPath(); ctx.arc(0, 0, r * 0.54, 0, TAU); ctx.fillStyle = far ? mix(lk.rim, '#000000', 0.5) : lk.rim; ctx.fill();
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
    const c = Math.cos(v.angle), s = Math.sin(v.angle), hx = v.x + c * 1.3 - s * 0.75, hy = v.y + s * 1.3 + c * 0.75;
    ctx.fillStyle = hc.engineHot ? 'rgba(235,235,235,0.35)' : 'rgba(60,60,60,0.25)';
    for (let i = 0; i < 4; i++) {
      const k = (t * 0.8 + i / 4) % 1;
      ctx.beginPath(); ctx.arc(hx + k * 0.4, hy + k * 1.1, 0.12 + k * 0.25, 0, TAU); ctx.fill();
    }
  }

  // -- particles ---------------------------------------------------------------------------------
  _spawn(kind, layer, col, x, y, vx, vy, life, size, grow, alpha, z = 0, vz = 0) {
    const f = this.fx, i = f.n;
    if (i >= FX_MAX) return;
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
        if (f.n >= FX_ROOST_MAX) continue;
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
  _kick(k) { const F = this.feel; if (k > 0) F.trauma = Math.min(1, Math.max(F.trauma, k) + k * 0.25); }
  _flash(a, col) { const F = this.feel; if (a > F.flash) { F.flash = Math.min(0.6, a); F.flashCol = col; } }

  /** Screen-px camera offset: trauma² × amplitude × smooth two-sine noise; trauma decays ~0.55 s from full. */
  _shakeOff(dt) {
    const F = this.feel, o = this._sk || (this._sk = [0, 0]);
    F.t += dt; F.trauma = Math.max(0, F.trauma - dt * 1.8);
    const a = F.trauma * F.trauma * Math.min(this.w, this.h) * 0.022 * this.shakeScale, t = F.t;
    o[0] = a * (0.62 * Math.sin(t * 47.3 + F.ph[0]) + 0.38 * Math.sin(t * 91.7 + F.ph[1]));
    o[1] = a * (0.62 * Math.sin(t * 53.9 + F.ph[1]) + 0.38 * Math.sin(t * 79.1 + F.ph[0]));
    return o;
  }

  /** Rope vibration (m, added to the sag control point) once the cable passes 65 % of its break load. */
  _ropeHum(t) { return t > 0.65 ? Math.sin(this.feel.t * 95) * 0.06 * (t - 0.65) / 0.35 : 0; }

  /** Burst of surface flecks (and sparks on rock / metal) at a contact, count ∝ closing speed. */
  _burst(x, y, z, nx, ny, surf, vn, sparks) {
    const R = Math.random, sf = this.surfFx[surf] || { kind: K.DUST, col: 0 }, n = Math.min(12, Math.round(vn * 2.2));
    if (ny < 0) { nx = -nx; ny = -ny; }
    for (let k = 0; k < n && this.fx.n < FX_MAX - 8; k++) {
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
      if (o.grounded && sf && sf.dig > 0 && Math.abs(o.slip) > 1 && R() < dt * 14 * sf.dig && this.fx.n < FX_ROOST_MAX && o.contacts[0]) {
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
