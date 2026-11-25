// engine/physics.js — Offroad2D physics world. NO rendering / DOM code in this file.
// Units: metres, kilograms, seconds, radians. +x = forward (right), +y = up.
import * as planck from './vendor/planck.mjs';   // vendored esm.sh build of planck@1.5.1 (works offline)
import { TRAIL, zoneAtZ, laneOffset, laneZ, rutAt, rutUnder, edgeDrop, lipAt, lipUnder, isWall } from './trail.js';
export { planck, TRAIL };

const Vec2 = planck.Vec2;

export const FIXED_DT = 1 / 60;      // the only timestep the simulation ever uses
export const MAX_FRAME_DT = 0.1;     // frame-time clamp (max 6 fixed steps per rendered frame)
export const VEL_ITERS = 10;
export const POS_ITERS = 4;
// TERRAIN = rock polygons, GROUND = shared ground chains (chassis, logs), LANE << k = per-wheel lane colliders (8 bits)
export const CAT = { TERRAIN: 1, VEHICLE: 2, PLANK: 4, LOG: 8, GROUND: 16, LANE: 32 };
export const LANES = 0x7ff * CAT.LANE;   // 11 lane bits (32 .. 32768): 4 tyres + 1 belly per truck

// mu = tyre friction coefficient, rr = rolling-resistance coefficient (fraction of wheel load).
export const DEFAULT_SURFACES = {
  dirt:    { mu: 0.85, rr: 0.030, color: '#7b5d3c' },
  rock:    { mu: 1.05, rr: 0.015, color: '#6e7379' },
  mud:     { mu: 0.26, rr: 0.110, color: '#463325' },
  wetwood: { mu: 0.40, rr: 0.020, color: '#8b6b47' },
  wood:    { mu: 0.60, rr: 0.020, color: '#9a7850' },
  ice:     { mu: 0.08, rr: 0.010, color: '#bcd9e6' },
  berm:    { mu: 0.72, rr: 0.085, color: '#8a6a43', dig: 0 },   // soft dirt trail-edge berms (TRAIL)
};

const INF = 1e9;

// Soft-terrain rutting: tyres spinning on surfaces with `dig` > 0 sink a per-run rut grid (TRAIL.nz lanes across z,
// one row per lane-collider vertex); each wheel's lane collider follows the mean depth under its own footprint.
//   res       (unused: soft flats resample at TRAIL.res)  maxDepth  deepest rut (m)
//   slip      contact-patch slip speed (m/s) below which nothing digs (gentle crawl stays on top)
//   rate      m of depth per s per (m/s of excess slip) at loadRef, per lane   width  half-width of the dig kernel along x (m)
//   maxStep   max sink per vertex per fixed step (m)     wall      max rut wall slope (dy/dx); also ramps depth to 0 at rigid edges
//   bank      dig rate fades linearly to 0 at this base slope (dy/dx): pit floors rut, banks / ledge faces shed the spoil
export const RUT = { res: 0.25, maxDepth: 0.35, slip: 1.0, rate: 0.1, width: 0.45, maxStep: 0.003, wall: 0.6, loadRef: 5000, bank: 0.25 };
/** Rut-rate multiplier of a surface: explicit `dig`, else mud/bog 1, clay 0.6, anything else 0 (rigid). */
// Log tether: every log / boulder is bedded at its settled pose by a gravity-compensated spring-damper
// (linear + angular), so props stay in their corduroy / pile slots but micro-rock, twist and sink under load.
// Rock logs (boulders, rock ridges) are frozen static at their bed pose once sunk: stone doesn't rock.
//   settle   max steps a fresh log falls free (x held) before its bed pose is captured (earlier once at rest)
//   hz/vhz/zeta  horizontal / vertical (bed) spring frequency (Hz) / damping ratio (k = m·(2π·hz)², scaled by mass -> same feel for any size)
//   hover    bed sits this far above the resting contact: wheel load sinks the log up to it, then the ground takes over
//   slack    |offset| beyond which the linear spring stiffens (×1 + gain·excess/slack, ≤ maxGain)
//   vslack   sag below the bed beyond which the bed spring stiffens (≤ vmaxGain): a firm bottom-out, not soft ground
//   twistHz/twistZeta  angular spring; twist  free twist (rad), beyond it stiffens (gain) and the bed angle creeps <= creep rad/s
//   embed    fraction of the diameter a terrain-resting log is bedded into the ground once everything has settled
//            (per-log `embed` overrides; terrain collision then drops, the bed spring carries it). Stacked logs sink with
//            the deepest log they rest on, so piles keep their shape.
//   free     per-log `free: true` in level JSON disables the tether (loose prop)
//   glance / glanceBody  a tyre (chassis) whose z span reaches less than this far into a partial boulder's span only
//            brushes its rounded end: no contact; the tyre is nudged out sideways (tire.glance -> vehicle.js), the chassis ignores it
//   faceMu / faceNy  tyre friction cap on a partial-width boulder (rock log with z0 / z1) where the contact normal is
//            steeper than |ny| < faceNy: its flank cannot be climbed or launched off, only driven around
// timber bridges: sag x span (slack chain), rail = |z| of the rope rails (tyre walls), plank angular damping, breaking
export const BRIDGE = { sag: 0.03, rail: 1.85, angDamp: 0.25, breakSpeed: 9.0, fastTime: 0.6, breakVn: 4.5 };   // sturdy (2026-11-24 r3): only a hard slam (4.5 m/s) or a near-flat-out run (32 km/h held 0.6 s) breaks it
// tyre bite vs wheelspin: planck friction is Coulomb (the same mu sticking or sliding), so a spinning tyre pulled as hard as
// a gripping one and feathering the gas earned nothing. Real tread grips best near zero slip; a spinning carcass polishes
// rock / rolls loose stones and loses bite. Tyre mu x (1 − loss · smoothstep((slip − s0) / (s1 − s0))), slip = the tyre's
// low-passed contact-patch sliding speed (m/s, time constant tau: no stick/slip chatter); loss = surface `spin` if set, else
// by name (mud / bog / clay: lugs clear and dig, less; gravel / scree: rolling stones, more; ice most), else `spin`.
export const TYRE = { s0: 0.35, s1: 3, spin: 0.07, tau: 0.08 };   // mild: 0.1+ stalled open-diff trucks on marginal L03 steps
const spinOf = (name, s) => (s && s.spin != null ? s.spin : /mud|bog|clay/.test(name) ? 0.04 : /gravel|scree/.test(name) ? 0.1 : /ice/.test(name) ? 0.15 : TYRE.spin);
export const LOG = { glance: 0.16, glanceBody: 0.15, faceMu: 0.12, wallMu: 0.02, faceNy: 0.7, settle: 90, embed: 0.25, hz: 3.2, vhz: 8, zeta: 0.85, hover: 0.006, slack: 0.12, gain: 2, maxGain: 5, vslack: 0.015, vmaxGain: 2.5,
  twistHz: 4.5, twistZeta: 1, twist: 0.25, creep: 0.12, angularDamping: 2.5, linearDamping: 0.3 };
const digOf = (name, s) => (s && s.dig != null ? s.dig : /mud|bog/.test(name) ? 1 : /clay/.test(name) ? 0.6 : 0);
/** Split segments wider than `res` (in x) for which need(a, b) holds into equal pieces; steps stay whole. */
function resample(pts, res, need) {
  const out = [pts[0]];
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1], b = pts[i], m = need(a, b) ? Math.max(1, Math.ceil((b[0] - a[0]) / res - 1e-9)) : 1;
    for (let k = 1; k < m; k++) out.push([a[0] + (b[0] - a[0]) * k / m, a[1] + (b[1] - a[1]) * k / m]);
    out.push(b);
  }
  return out;
}

// ---------------------------------------------------------------------------------------------
// z span [lo, hi] of a fixture's owner across the trail: partial logs carry zs, the chassis zs (vehicle.js), tyres tire.z ± w/2
const zLo = u => (u.zs ? u.zs.lo : u.tire ? u.tire.z - u.tire.w / 2 : null), zHi = u => (u.zs ? u.zs.hi : u.tire ? u.tire.z + u.tire.w / 2 : null);
const _tz = { zs: { lo: 0, hi: 0 } };   // reused: a steered tyre's z span at the contact x
/** True when two fixtures' z spans are both known and disjoint (they pass beside each other on the trail). */
function _zMiss(a, b) {
  const al = zLo(a), bl = zLo(b);
  return al != null && bl != null && (zHi(a) < bl || zHi(b) < al);
}

// Winch: one-sided spring-damper rope between a body-mounted drum and an anchor.
//   maxLength   rope on the drum (m)      maxPull       motor stall tension (N)
//   breakTension rope snaps above this (N) reelSpeed    no-load line speed (m/s)
// ---------------------------------------------------------------------------------------------
export class Winch {
  constructor(phys, body, cfg) {
    this.phys = phys;
    this.body = body;
    this.mount = Vec2(cfg.mount[0], cfg.mount[1]);
    this.maxLength = cfg.maxLength;
    this.minLength = cfg.minLength ?? 0.8;
    this.reelSpeed = cfg.reelSpeed;
    this.maxPull = cfg.maxPull;
    this.breakTension = cfg.breakTension;
    this.k = cfg.stiffness;
    this.c = cfg.damping;
    this.attached = false;
    this.broken = false;
    this.length = 0;        // paid-out rope length
    this.tension = 0;       // N, last step
    this.peakTension = 0;   // N, since attach
    this.reel = 0;          // +1 reel in, -1 pay out, 0 hold
    this.target = null;
  }

  mountWorld() { return this.body.getWorldPoint(this.mount); }

  anchorWorld() {
    const t = this.target;
    if (!t) return null;
    return t.body ? t.body.getWorldPoint(t.local) : t.point;
  }

  /** Attach to {x, y, body?}. Fails (returns false) if the anchor is beyond maxLength. */
  attach(target) {
    const m = this.mountWorld();
    const d = Math.hypot(target.x - m.x, target.y - m.y);
    if (d > this.maxLength) return false;
    const p = Vec2(target.x, target.y);
    this.target = target.body ? { body: target.body, local: target.body.getLocalPoint(p) } : { point: p };
    this.length = Math.max(d, this.minLength);
    this.attached = true;
    this.broken = false;
    this.tension = 0;
    this.peakTension = 0;
    return true;
  }

  /** Attach to the nearest in-range anchor, preferring anchors in front (fx, fy = forward unit vector). */
  attachNearest(anchors, fx = 1, fy = 0) {
    const m = this.mountWorld();
    let best = null, bestScore = Infinity;
    for (const a of anchors) {
      const dx = a.x - m.x, dy = a.y - m.y, d = Math.hypot(dx, dy);
      if (d > this.maxLength) continue;
      const score = d + (dx * fx + dy * fy < 0 ? this.maxLength : 0);
      if (score < bestScore) { bestScore = score; best = a; }
    }
    return best ? this.attach(best) : false;
  }

  detach() {
    this.attached = false;
    this.target = null;
    this.tension = 0;
    this.reel = 0;
  }

  step(dt) {
    if (!this.attached) { this.tension = 0; return; }
    if (this.reel > 0) {
      const sp = this.reelSpeed * Math.max(0, 1 - this.tension / this.maxPull); // stalls at maxPull
      this.length = Math.max(this.minLength, this.length - sp * dt);
    } else if (this.reel < 0) {
      this.length = Math.min(this.maxLength, this.length + this.reelSpeed * dt);
    }
    const m = this.mountWorld(), a = this.anchorWorld();
    const dx = a.x - m.x, dy = a.y - m.y;
    const dist = Math.hypot(dx, dy);
    const stretch = dist - this.length;
    let T = 0;
    if (stretch > 0 && dist > 1e-6) {
      const ux = dx / dist, uy = dy / dist;
      const vm = this.body.getLinearVelocityFromWorldPoint(m);
      let rvx = -vm.x, rvy = -vm.y;
      const tb = this.target.body;
      if (tb) { const va = tb.getLinearVelocityFromWorldPoint(a); rvx += va.x; rvy += va.y; }
      T = this.k * stretch + this.c * (rvx * ux + rvy * uy);
      if (T < 0) T = 0;
      if (T > this.breakTension) {
        this.peakTension = T;
        this.detach();
        this.broken = true;
        this.phys.events.push({ type: 'winch-break', step: this.phys.stepCount, tension: T });
        return;
      }
      if (T > 0) {
        this.body.applyForce(Vec2(ux * T, uy * T), m, true);
        if (tb) tb.applyForce(Vec2(-ux * T, -uy * T), a, true);
      }
    }
    this.tension = T;
    if (T > this.peakTension) this.peakTension = T;
  }
}

// ---------------------------------------------------------------------------------------------
// PhysicsWorld: fixed-step planck world built from a level description (see CLAUDE.md schema).
// ---------------------------------------------------------------------------------------------
export class PhysicsWorld {
  constructor(level = {}) {
    this.level = level;
    const g = level.gravity || [0, -9.81];
    this.gravity = { x: g[0], y: g[1] };
    this.world = new planck.World({ gravity: Vec2(g[0], g[1]), allowSleep: true });
    this.surfaces = {};
    for (const k in DEFAULT_SURFACES) this.surfaces[k] = { ...DEFAULT_SURFACES[k] };
    for (const k in (level.surfaces || {})) this.surfaces[k] = { ...this.surfaces[k], ...level.surfaces[k] };
    this.zones = (level.zones || []).map(z => ({
      x0: z.x0 ?? -INF, x1: z.x1 ?? INF, y0: z.y0 ?? -INF, y1: z.y1 ?? INF, z0: z.z0 ?? -INF, z1: z.z1 ?? INF,
      surface: z.surface, side: z.side || 'both', dy: z.dy || 0,
    }));
    this.dyZones = this.zones.filter(z => z.dy);
    this.anchors = (level.anchors || []).map(a => ({ x: a.x, y: a.y, name: a.name || '' }));
    // jerry cans: {x, y, z, fuel (tank fraction)}; the vehicle collects one it drives over (getDynamic().cans = taken flags)
    this.cans = (level.cans || []).map(c => ({ x: c.x, y: c.y ?? 0, z: c.z ?? 0, fuel: c.fuel ?? 0.2, taken: false }));
    this.tank = Math.min(1, Math.max(0.05, level.tank ?? 1));   // starting fill of every truck's tank (fraction)
    this.preStep = [];     // fn(dt) called before each fixed step
    this.postStep = [];    // fn(dt) called after each fixed step
    this.tracked = [];     // bodies whose userData.contacts[] is refreshed every step
    this.winches = [];
    this.events = [];      // drained by the game loop: {type, step, ...}
    this.logs = [];
    this.zLogs = [];       // partial-width logs (z0 / z1)
    this.planks = [];
    this.bridges = [];     // addBridge records {x0, x1, y, rail, planks, joints, intact, ...}
    this.accumulator = 0;
    this.stepCount = 0;
    this.time = 0;
    this.groundBody = this.world.createBody();
    this.ruts = [];        // rut-grid patches of deformable runs (see _softRun / _rut)
    this.rutVersion = 0;   // bumped whenever any rut depth changes
    this.lanes = [];       // per-wheel lane colliders (addLane)
    this._dig = [];        // per-step [patch, wheelBody, contactRec, tire] quads from _preSolve
    this._digOf = {};      // surface name -> dig factor
    this._touched = []; this._cp = Vec2(0, 0); this._zero = Vec2(0, 0);
    this._buildGround(level.ground || []);
    this._buildPolygons(level.polygons || []);
    for (const l of (level.logs || [])) this.addLog(l);
    for (const b of (level.bridges || [])) this.addBridge(b);
    this.world.on('pre-solve', c => this._preSolve(c));
    this.world.on('post-solve', (c, imp) => this._postSolve(c, imp));
  }

  surface(name) { return this.surfaces[name] || this.surfaces.dirt; }

  // -- terrain ---------------------------------------------------------------------------------
  _buildGround(runs) {
    // Runs are consecutive polylines (each run's first point == previous run's last point). The shared chain (chassis,
    // logs) keeps the authored points and never deforms; wheels ride their own lane colliders (addLane), built on a
    // TRAIL.res resample of the segments that can change shape along x (diggable flats, zone dy ramps).
    const T = TRAIL, grade = (a, b) => (b[0] - a[0] > 1e-6 ? Math.abs((b[1] - a[1]) / (b[0] - a[0])) : INF);
    const need = base => (a, b) => this.dyZones.some(q => b[0] >= q.x0 - T.ramp && a[0] <= q.x1 + T.ramp)
      || (grade(a, b) < RUT.bank && this._digAt(base, (a[0] + b[0]) / 2, (a[1] + b[1]) / 2) > 0);
    const lay = this._lay = runs.map(r => (r.points && r.points.length >= 2 ? resample(r.points, T.res, need(r.surface || 'dirt')) : r.points || []));
    // wall shelves (isWall): lane vertices just either side of each x face, so it is a real near-vertical face
    const cut = []; for (const q of this.dyZones) if (isWall(T, q)) cut.push(q.x0 - T.wallCh, q.x0 - 0.005, q.x0 + 0.005, q.x1 - 0.005, q.x1 + 0.005, q.x1 + T.wallCh);
    if (cut.length) for (const pts of lay) for (const x of cut) {
      if (pts.length < 2 || x <= pts[0][0] || x >= pts[pts.length - 1][0]) continue;
      let i = 1; while (pts[i][0] < x) i++;
      const a = pts[i - 1], b = pts[i];
      if (Math.abs(b[0] - x) < 1e-3 || Math.abs(a[0] - x) < 1e-3 || b[0] - a[0] < 1e-6) continue;
      pts.splice(i, 0, [x, a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0])]);
    }
    this._patchOf = [];
    for (let i = 0; i < runs.length; i++) {
      const pts = runs[i].points;
      if (!pts || pts.length < 2) continue;
      const shape = new planck.ChainShape(pts.map(p => Vec2(p[0], p[1])), false);
      const prev = runs[i - 1] && runs[i - 1].points, next = runs[i + 1] && runs[i + 1].points;
      if (prev && prev.length > 1) { const p = prev[prev.length - 2]; shape.setPrevVertex(Vec2(p[0], p[1])); }
      if (next && next.length > 1) { const p = next[1]; shape.setNextVertex(Vec2(p[0], p[1])); }
      const surface = runs[i].surface || 'dirt';
      this.groundBody.createFixture({ shape, friction: 0.8, restitution: 0, filterCategoryBits: CAT.GROUND, userData: { surface, terrain: true } });
      const sr = this._softRun(lay[i], surface);
      if (!sr) continue;
      const n = lay[i].length;
      const patch = { run: i, n, x: lay[i].map(p => p[0]), k: sr.k, cap: sr.cap, i0: sr.i0, i1: sr.i1,
        d: new Float64Array(n * T.nz), df: new Float32Array(n * T.nz), lo: Infinity, hi: -1, llo: T.nz, lhi: -1, dirty: false };
      this._patchOf[i] = patch;
      this.ruts.push(patch);
    }
    this._rutView = { ver: 0, nz: T.nz, lane: T.lane, runs: this.ruts.map(p => ({ run: p.run, d: p.df })) };
  }

  /** Highest dig factor over the rut-grid lanes at (x, y): lane-specific surfaces come from side / z zones. */
  _digAt(base, x, y) {
    let m = 0;
    for (let l = 0; l < TRAIL.nz; l++) m = Math.max(m, this.digOf(this.surfaceAt(base, x, y, laneZ(TRAIL, l), true)));
    return m;
  }

  /** Geometric dig factor per vertex of a resampled run (grade fade); null if no lane of any vertex can deform. */
  _softRun(out, base) {
    const R = RUT, digAt = (x, y) => this._digAt(base, x, y);
    const grade = (a, b) => (b[0] - a[0] > 1e-6 ? Math.abs((b[1] - a[1]) / (b[0] - a[0])) : INF);
    const n = out.length, k = new Float64Array(n), cap = new Float64Array(n);
    let i0 = n, i1 = -1;
    for (let i = 1; i < n - 1; i++) {   // run endpoints stay fixed (continuity with the neighbouring runs)
      const g = Math.max(grade(out[i - 1], out[i]), grade(out[i], out[i + 1]));
      if (!(g < R.bank) || !(digAt(out[i][0], out[i][1]) > 0)) continue;
      k[i] = 1 - g / R.bank;
      if (k[i] > 0) { i0 = Math.min(i0, i); i1 = Math.max(i1, i); }
    }
    if (i1 < 0) return null;
    // depth cap: maxDepth, ramped down at wall slope towards every rigid vertex (no sharp steps at patch edges)
    let last = -INF;
    for (let i = 0; i < n; i++) { if (!(k[i] > 0)) last = out[i][0]; cap[i] = Math.min(R.maxDepth, R.wall * (out[i][0] - last)); }
    last = INF;
    for (let i = n - 1; i >= 0; i--) { if (!(k[i] > 0)) last = out[i][0]; cap[i] = Math.max(0, Math.min(cap[i], R.wall * (last - out[i][0]))); }
    return { k, cap, i0, i1 };
  }

  /** Rut-rate multiplier of a surface name (see digOf). */
  digOf(name) { const c = this._digOf; return c[name] ?? (c[name] = digOf(name, this.surfaces[name])); }

  // -- lane colliders (2.5D) --------------------------------------------------------------------
  /**
   * A private copy of the ground for one wheel (collision category CAT.LANE << k, collides with VEHICLE only).
   * Its vertices follow H(x) = ground + laneOffset(x, z) - rut depth under the tyre footprint at the lane's z; the
   * window lane.x ± TRAIL.win is refreshed before every step. Set lane.x / lane.z / lane.w from the vehicle's preStep.
   */
  addLane(z = 0, w = 0.4) {
    let lane = this.lanes.find(l => l.free);
    if (!lane) {
      const k = this.lanes.length;
      if (k >= 11) throw new Error('physics: out of lane colliders (11)');
      lane = { bit: CAT.LANE << k, z, w, x: 0, free: false, chains: [] };
      const lay = this._lay;
      for (let i = 0; i < lay.length; i++) {
        const pts = lay[i];
        if (!pts || pts.length < 2) continue;
        const shape = new planck.ChainShape(pts.map(p => Vec2(p[0], p[1])), false);
        const prev = lay[i - 1], next = lay[i + 1];
        if (prev && prev.length > 1) shape.setPrevVertex(Vec2(prev[prev.length - 2][0], prev[prev.length - 2][1]));
        if (next && next.length > 1) shape.setNextVertex(Vec2(next[1][0], next[1][1]));
        const surface = (this.level.ground[i].surface) || 'dirt', patch = this._patchOf[i] || null;
        const fixture = this.groundBody.createFixture({ shape, friction: 0.8, restitution: 0, filterCategoryBits: lane.bit,
          filterMaskBits: CAT.VEHICLE, userData: { surface, terrain: true, lane, patch } });
        lane.chains[i] = { fixture, shape, x: pts.map(p => p[0]), y0: pts.map(p => p[1]), patch };
      }
      // ghost vertices reference the neighbouring chain's live vertices, so smoothing across run joints follows the lane
      for (let i = 0; i < lane.chains.length; i++) {
        const c = lane.chains[i];
        if (!c) continue;
        const p = lane.chains[i - 1], q = lane.chains[i + 1];
        if (p) c.shape.m_prevVertex = p.shape.m_vertices[p.shape.m_vertices.length - 2];
        if (q) c.shape.m_nextVertex = q.shape.m_vertices[1];
      }
      this.lanes.push(lane);
    }
    lane.free = false; lane.z = z; lane.w = w;
    this._laneFill(lane, -INF, INF);
    return lane;
  }

  releaseLane(lane) { if (lane) lane.free = true; }

  /** Authored ground height at x (highest run; trail centre, no berm / zone / drop): edge-fall depth + recover height. */
  groundAt(x) {
    let y = -Infinity;
    for (const p of this._lay) {
      if (!p || p.length < 2 || x < p[0][0] || x > p[p.length - 1][0]) continue;
      let i = 0, hi = p.length - 1;
      while (hi - i > 1) { const m = (i + hi) >> 1; if (p[m][0] <= x) i = m; else hi = m; }
      const a = p[i], b = p[hi], v = b[0] > a[0] ? a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0]) : Math.max(a[1], b[1]);
      if (v > y) y = v;
    }
    return y;
  }

  /** Move lane-collider vertices with x in [xl, xr] to the lane's current surface; refresh the moved edges' AABBs. */
  _laneFill(lane, xl, xr) {
    const T = TRAIL, Z = this.dyZones, z = lane.z, w = lane.w, xf = this.groundBody.getTransform(), bp = this.world.m_broadPhase;
    // a yawed / steered tyre meets a shelf or step with its footprint line: zones sampled at z + (x − hub x)·tan heading, |dx| ≤ R
    const th = lane.h ? Math.tan(lane.h) : 0, lx = lane.x, lr = lane.r || 0;
    const lipF = lane.foot === false ? 0 : lipUnder(T, z, w) - lipAt(T, z);
    const hwT = Math.max(0, w / 2 - T.treadIn);                      // tread edges sample raised zones (see laneOffset)                    // edge lip under the whole tread, not just its centre line
    for (let r = 0; r < lane.chains.length; r++) {
      const c = lane.chains[r];
      if (!c) continue;
      const x = c.x, n = x.length;
      if (x[n - 1] < xl || x[0] > xr) continue;
      let i = 0, hi = n;
      while (i < hi) { const m = (i + hi) >> 1; if (x[m] < xl) i = m + 1; else hi = m; }
      const V = c.shape.m_vertices, P = c.patch;
      let lo = n, top = -1;
      for (i = Math.max(0, i - 1); i < n; i++) {   // + the vertex on each side: a long edge through the window moves whole
        const zq = th ? z + Math.max(-lr, Math.min(lr, x[i] - lx)) * th : z;
        if (lane.belly) {                                            // chassis belly: highest surface across its width (no ruts / drop)
          let m = 0;
          for (let k = -2; k <= 2; k++) m = Math.max(m, laneOffset(T, Z, x[i], c.y0[i], zq + k * lane.belly / 2));
          const y = c.y0[i] + m;
          if (Math.abs(V[i].y - y) > 1e-7) { V[i].y = y; if (i < lo) lo = i; top = i; }
          if (x[i] > xr) break;
          continue;
        }
        const y = c.y0[i] + laneOffset(T, Z, x[i], c.y0[i], z, zq, hwT) + lipF - edgeDrop(T, z) - (P && P.cap[i] > 0 ? rutUnder(T, P.d, i, z, w) : 0);
        if (Math.abs(V[i].y - y) > 1e-7) { V[i].y = y; if (i < lo) lo = i; top = i; }
        if (x[i] > xr) break;
      }
      if (top < 0) continue;
      const px = c.fixture.m_proxies;
      for (let e = Math.max(0, lo - 1), e1 = Math.min(n - 2, top); e <= e1; e++) {
        const pr = px[e];
        c.shape.computeAABB(pr.aabb, xf, e);
        bp.moveProxy(pr.proxyId, pr.aabb, this._zero);
      }
    }
  }

  _buildPolygons(polys) {
    // Static rocks / ledges: any simple polygon (closed chain loop).
    for (const poly of polys) {
      const shape = new planck.ChainShape(poly.points.map(p => Vec2(p[0], p[1])), true);
      this.groundBody.createFixture({
        shape, friction: 0.8, restitution: 0,
        filterCategoryBits: CAT.TERRAIN,
        userData: { surface: poly.surface || 'rock', terrain: true },
      });
    }
  }

  // -- dynamic obstacles -----------------------------------------------------------------------
  addLog(def) {
    const r = def.r ?? 0.3;
    // partial-width log / boulder: spans world z [z0, z1] across the trail (default: the full width). Wheels, the
    // chassis and other logs collide with it only while their own z span overlaps (see _zMiss); vehicle.js keeps
    // tyres / chassis from sliding sideways into one they already overlap in x-y (a side wall).
    const zs = def.z0 != null || def.z1 != null ? { lo: def.z0 ?? -INF, hi: def.z1 ?? INF } : null;
    const mass = def.mass ?? Math.PI * r * r * (def.length ?? 2.2) * (def.density ?? 550);
    const body = this.world.createBody({
      type: 'dynamic', position: Vec2(def.x, def.y), angle: def.angle ?? 0,
      angularDamping: def.free ? 0.4 : LOG.angularDamping, linearDamping: def.free ? 0.02 : LOG.linearDamping,
    });
    body.createFixture({
      shape: new planck.CircleShape(r), density: 0, friction: 0.6, restitution: 0.05,
      filterCategoryBits: CAT.LOG, filterMaskBits: 0xffff & ~LANES,
      userData: { surface: def.surface || 'wood', zs, rock: /rock|stone|boulder/.test(def.surface || '') },
    });
    body.setMassData({ mass, center: Vec2(0, 0), I: 0.5 * mass * r * r });
    const I = 0.5 * mass * r * r, w = 2 * Math.PI * LOG.hz, wv = 2 * Math.PI * LOG.vhz, wt = 2 * Math.PI * LOG.twistHz;
    const tether = def.free ? null : {
      x: def.x, y: def.y, a: def.angle ?? 0, age: 0, bedded: false, sunk: false, m: mass, embed: def.embed ?? LOG.embed,
      k: mass * w * w, c: 2 * LOG.zeta * mass * w, kv: mass * wv * wv, cv: 2 * LOG.zeta * mass * wv, kt: I * wt * wt, ct: 2 * LOG.twistZeta * I * wt,
    };
    body.setUserData({ kind: 'log', r, tether, zs, rock: /rock|stone|boulder/.test(def.surface || '') });
    if (zs) this.zLogs.push(body);
    this.logs.push(body);
    return body;
  }

  addBridge(def) {
    // Timber bridge: n planks hinged end-to-end, pinned to the ground at both ends. The chain is laid on a shallow parabola
    // (sag def.sag m, default BRIDGE.sag x span) so it is slack: it sags and sways under a truck instead of acting like a
    // rigid beam. It breaks (one hinge lets go near the hit) when a truck crosses faster than breakSpeed m/s (held fastTime s), or anything
    // lands on it with a closing speed over breakVn m/s -> event 'bridge-break'.
    const ax = def.from[0], ay = def.from[1], bx = def.to[0], by = def.to[1];
    const n = def.planks ?? Math.max(6, Math.round(Math.hypot(bx - ax, by - ay) / 0.9)), t = def.thickness ?? 0.14;
    const span = Math.hypot(bx - ax, by - ay), sag = def.sag ?? BRIDGE.sag * span;
    const ux = (bx - ax) / span, uy = (by - ay) / span;
    const mass = def.plankMass ?? 150;
    const P = k => { const u = k / n; return [ax + (bx - ax) * u + uy * 4 * sag * u * (1 - u), ay + (by - ay) * u - ux * 4 * sag * u * (1 - u)]; };
    const br = { def, x0: Math.min(ax, bx), x1: Math.max(ax, bx), y: Math.min(ay, by), rail: BRIDGE.rail, planks: [], joints: [], intact: true,
      breakSpeed: def.breakSpeed ?? BRIDGE.breakSpeed, breakVn: def.breakVn ?? BRIDGE.breakVn, hitVn: 0, hitX: 0, touch: false, touchV: 0, idle: 99, fast: 0 };
    let prev = this.groundBody;
    for (let i = 0; i < n; i++) {
      const [x0, y0] = P(i), [x1, y1] = P(i + 1), len = Math.hypot(x1 - x0, y1 - y0), ang = Math.atan2(y1 - y0, x1 - x0);
      const nx = -Math.sin(ang), ny = Math.cos(ang);
      const body = this.world.createBody({
        type: 'dynamic', position: Vec2((x0 + x1) / 2 - nx * t / 2, (y0 + y1) / 2 - ny * t / 2), angle: ang,
        angularDamping: BRIDGE.angDamp, linearDamping: 0.05,
      });
      body.createFixture({
        shape: new planck.BoxShape(len / 2, t / 2), density: 0, friction: 0.5, restitution: 0,
        filterCategoryBits: CAT.PLANK, filterMaskBits: CAT.VEHICLE | CAT.LOG,
        userData: { surface: def.surface || 'wetwood', bridge: br },
      });
      body.setMassData({ mass, center: Vec2(0, 0), I: mass * (len * len + t * t) / 12 });
      body.setUserData({ kind: 'plank', hw: len / 2, hh: t / 2, bridge: this.bridges.length });
      br.joints.push(this.world.createJoint(new planck.RevoluteJoint({ collideConnected: false }, prev, body, Vec2(x0, y0))));
      br.planks.push(body);
      this.planks.push(body);
      prev = body;
    }
    br.joints.push(this.world.createJoint(new planck.RevoluteJoint({ collideConnected: false }, prev, this.groundBody, Vec2(bx, by))));
    // entry / exit posts: side walls just outside the rails (zs), so a tyre lined up outside the rails hits a post instead
    // of rolling onto the deck beside them (the chassis and the tyres between the rails pass)
    for (const [px, py] of [[ax, ay], [bx, by]]) for (const sd of [-1, 1]) {
      const zs = sd > 0 ? { lo: br.rail - 0.02, hi: 9 } : { lo: -9, hi: -(br.rail - 0.02) };
      this.groundBody.createFixture({ shape: new planck.BoxShape(0.09, 0.7, Vec2(px + (px === ax ? 0.15 : -0.15), py + 0.55), 0), friction: 0.6,
        filterCategoryBits: CAT.LOG, filterMaskBits: CAT.VEHICLE, userData: { surface: 'wood', zs } });
    }
    this.bridges.push(br);
    return br.planks;
  }

  /** After a step: a bridge first touched by a vehicle moving faster than breakSpeed, or hit harder than breakVn, breaks. */
  _bridgeCheck(dt) {
    for (let k = 0; k < this.bridges.length; k++) {
      const br = this.bridges[k];
      if (br.intact && br.touch) {
        br.fast = br.touchV > br.breakSpeed ? br.fast + dt : Math.max(0, br.fast - 2 * dt);   // driven across too fast (held, not a blip)
        const arrive = br.fast > BRIDGE.fastTime, slam = br.hitVn > br.breakVn;
        if (arrive || slam) this.breakBridge(k, br.hitX, arrive ? 'speed' : 'impact', arrive ? br.touchV : br.hitVn);
      }
      if (!br.touch) br.fast = Math.max(0, br.fast - 2 * dt);
      br.idle = br.touch ? 0 : br.idle + dt;
      br.touch = false; br.touchV = 0; br.hitVn = 0;
    }
  }

  /** Break bridge k at x: the hinge nearest x and the next one let go (that plank drops), the halves hang from the banks. */
  breakBridge(k, x, cause = 'load', v = 0) {
    const br = this.bridges[k];
    if (!br || !br.intact) return;
    br.intact = false;
    let j = 0, d = Infinity;
    for (let i = 0; i < br.joints.length; i++) { const a = br.joints[i].getAnchorA(), e = Math.abs(a.x - x); if (e < d) { d = e; j = i; } }
    j = Math.min(Math.max(1, j), br.joints.length - 2);                // never the bank pins: the deck splits mid-span
    for (const i of [j, j + 1]) if (br.joints[i]) { this.world.destroyJoint(br.joints[i]); br.joints[i] = null; }
    this.events.push({ type: 'bridge-break', bridge: k, x, y: br.y, cause, v });
  }

  createWinch(body, cfg) {
    const w = new Winch(this, body, cfg);
    this.winches.push(w);
    return w;
  }

  /** Register a body whose userData.contacts array should be filled each step. */
  track(body) { this.tracked.push(body); }

  // -- surfaces / friction ---------------------------------------------------------------------
  /**
   * Effective surface name at a world point for a tyre at trail z (null = no lane: chassis, logs -> only zones without
   * side / z bounds apply). ground = a ground-chain contact: past TRAIL.edge the soft dirt berm takes over.
   */
  surfaceAt(base, x, y, z = null, ground = false) {
    if (ground && z != null && Math.abs(z) > TRAIL.edge + 0.3 * TRAIL.bermW) return TRAIL.bermSurface;
    const Z = this.zones;
    for (let i = 0; i < Z.length; i++) {
      const q = Z[i];
      if (x >= q.x0 && x <= q.x1 && y >= q.y0 && y <= q.y1 && zoneAtZ(q, z)) return q.surface;
    }
    return base;
  }

  _preSolve(contact) {
    contact._o2d = null;
    const fA = contact.getFixtureA(), fB = contact.getFixtureB();
    const uA = fA.getUserData(), uB = fB.getUserData();
    if (!uA || !uB) return;
    if (uA.zs || uB.zs) {
      // a yawed chassis spans a different z at every x: take its span at the contact point (zsAt), not at its centre
      // a steered / yawed tyre: its plan-view footprint is a rotated w × 2R box, so at the contact x its z span is offset
      let a = uA, b = uB;
      if ((uA.zsAt && uB.zs) || (uB.zsAt && uA.zs)) {
        const m = contact.getWorldManifold(null), o = uA.zsAt ? fB : fA;
        const x = m && m.pointCount ? m.points[0].x : o.getBody().getPosition().x;
        if (uA.zsAt) a = uA.zsAt(x); else b = uB.zsAt(x);
      } else if ((uA.tire && uA.tire.h && uB.zs) || (uB.tire && uB.tire.h && uA.zs)) {
        const f = uA.tire ? fA : fB, t = (uA.tire || uB.tire), m = contact.getWorldManifold(null);
        const wx = f.getBody().getPosition().x, R = f.getShape().getRadius ? f.getShape().getRadius() : 0.5;
        const dx = Math.max(-R, Math.min(R, (m && m.pointCount ? m.points[0].x : wx) - wx));
        const c = Math.max(0.5, Math.cos(t.h)), zc = t.z + dx * Math.tan(t.h), hw = t.w / 2 / c;
        _tz.zs.lo = zc - hw; _tz.zs.hi = zc + hw;
        if (uA.tire) a = _tz; else b = _tz;
      }
      // lenient both ways: a miss at the centre OR along the heading (turned away = clear; turned in = still slides along)
      if (_zMiss(uA, uB) || ((a !== uA || b !== uB) && _zMiss(a, b))) { contact.setEnabled(false); return; }
      // rounded boulder ends (the drawn rock lumps taper there): a shallow z overlap only brushes it -> glance off sideways
      const rk = uA.rock && uA.zs ? uA : uB.rock && uB.zs ? uB : null, ou = rk === uA ? uB : uA, os = rk === uA ? b : a;
      if (rk && !ou.rock && (ou.tire || ou.zsAt)) {
        const lo = zLo(os), hi = zHi(os), ov = Math.min(hi, rk.zs.hi) - Math.max(lo, rk.zs.lo);
        if (ov < (ou.tire ? LOG.glance : LOG.glanceBody)) {
          contact.setEnabled(false);
          if (ou.tire) {
            const g = ((lo + hi) / 2 < (rk.zs.lo + rk.zs.hi) / 2 ? -1 : 1) * (ov + 0.01);
            if (Math.abs(g) > Math.abs(ou.tire.glance || 0)) ou.tire.glance = g;
          }
          return;
        }
      }
    }
    const sA = uA.surface, sB = uB.surface;
    if (!sA && !sB) return;
    const wm = contact.getWorldManifold(null);
    if (!wm || !wm.pointCount) return;
    const px = wm.points[0].x, py = wm.points[0].y;
    if (sA && sB) {
      const nA = uA.terrain ? this.surfaceAt(sA, px, py, null) : sA;
      const nB = uB.terrain ? this.surfaceAt(sB, px, py, null) : sB;
      contact.setFriction(Math.sqrt(this.surface(nA).mu * this.surface(nB).mu) * 0.8);
      return;
    }
    const s = sA ? uA : uB, o = sA ? uB : uA;
    if (s.bridge) {                                                    // a vehicle part on a bridge plank: arrival speed / impact
      const ob = (sA ? fB : fA).getBody(), pb = (sA ? fA : fB).getBody(), cp = Vec2(px, py);
      if (ob.getUserData() && ob.getUserData().contacts) {
        const vv = ob.getLinearVelocity(), vT = ob.getLinearVelocityFromWorldPoint(cp), vP = pb.getLinearVelocityFromWorldPoint(cp);
        const n = wm.normal, sg = sA ? 1 : -1, vn = Math.abs((vT.x - vP.x) * n.x * sg + (vT.y - vP.y) * n.y * sg);
        const br = s.bridge; br.touch = true; br.touchV = Math.max(br.touchV, Math.abs(vv.x));
        if (vn > br.hitVn) { br.hitVn = vn; br.hitX = px; } else if (!br.hitX) br.hitX = px;
      }
    }
    const z = o.tire ? o.tire.z : null;
    const name = s.terrain ? this.surfaceAt(s.surface, px, py, z, !!s.lane) : s.surface;
    let mu = this.surface(name).mu * (o.tire ? o.tire.grip : (o.bodyMu ?? 0.5));
    if (o.tire) {                                                      // wheelspin loses bite (TYRE): per-tyre slip, once per step
      const t = o.tire;
      if (t._ss !== this.stepCount) {
        const tb = (sA ? fB : fA).getBody(), cp = Vec2(px, py), vT = tb.getLinearVelocityFromWorldPoint(cp), vO = (sA ? fA : fB).getBody().getLinearVelocityFromWorldPoint(cp);
        const n = wm.normal, sl = Math.abs((vT.x - vO.x) * n.y - (vT.y - vO.y) * n.x);
        t._ss = this.stepCount; t.slip = (t.slip || 0) + (sl - (t.slip || 0)) * (1 - Math.exp(-FIXED_DT / TYRE.tau));
        const k = Math.min(1, Math.max(0, (t.slip - TYRE.s0) / (TYRE.s1 - TYRE.s0)));
        t.bite = 1 - spinOf(name, this.surface(name)) * k * k * (3 - 2 * k);
      }
      mu *= t.bite;
    }
    // a tyre finds no purchase on the steep flank of a partial-width boulder (rock log with zs): no wheelie up the side
    if (s.zs && o.tire && s.rock && Math.abs(wm.normal.y) < LOG.faceNy) mu = Math.min(mu, LOG.faceMu);
    // ...nor on the face of a tall side shelf (isWall): a tyre that runs into it stops / slides, it can't crawl up a 0.9 m wall
    else if (s.lane && o.tire && Math.abs(wm.normal.y) < LOG.faceNy && this._wallFace(px, o.tire.z)) mu = Math.min(mu, LOG.wallMu);   // ~0: the huge impact normal force x any grip threw the tyre up the face
    contact.setFriction(mu);
    const ob = (sA ? fB : fA).getBody(), ud = ob.getUserData();
    if (ud && ud.contacts) {
      const sgn = sA ? 1 : -1; // manifold normal points A -> B; store it pointing into the tracked body
      const nx = wm.normal.x * sgn, ny = wm.normal.y * sgn, cp = Vec2(px, py);
      // closing speed (m/s, + = approaching) before this step's solve: impact-damage input
      const vT = ob.getLinearVelocityFromWorldPoint(cp), vO = (sA ? fA : fB).getBody().getLinearVelocityFromWorldPoint(cp);
      const vn = -((vT.x - vO.x) * nx + (vT.y - vO.y) * ny);
      const rec = { x: px, y: py, nx, ny, surface: name, mu, impulse: 0, vn, terrain: !!s.terrain };
      ud.contacts.push(rec);
      contact._o2d = rec;
      if (s.patch && o.tire && contact._digStep !== this.stepCount) { contact._digStep = this.stepCount; this._dig.push(s.patch, ob, rec, o.tire); }
    }
  }

  _postSolve(contact, impulse) {
    const rec = contact._o2d;
    if (!rec) return;
    const ni = impulse.normalImpulses, n = contact.getManifold().pointCount;
    let sum = 0;
    for (let i = 0; i < n; i++) sum += ni[i];
    if (sum > rec.impulse) rec.impulse = sum;
  }

  // -- stepping --------------------------------------------------------------------------------
  /** Advance by a real frame time (clamped); runs 0..n fixed 1/60 s steps. Returns steps taken. */
  step(frameDt) {
    if (!(frameDt > 0)) return 0;
    this.accumulator += Math.min(frameDt, MAX_FRAME_DT);
    let n = 0;
    while (this.accumulator >= FIXED_DT - 1e-9) {
      this.stepFixed();
      this.accumulator -= FIXED_DT;
      n++;
    }
    if (this.accumulator < 0) this.accumulator = 0;
    return n;
  }

  /** Exactly one deterministic 1/60 s step. */
  stepFixed() {
    const dt = FIXED_DT;
    for (let i = 0; i < this.preStep.length; i++) this.preStep[i](dt);
    for (let i = 0; i < this.lanes.length; i++) { const l = this.lanes[i]; if (!l.free) this._laneFill(l, l.x - (l.win || TRAIL.win), l.x + (l.win || TRAIL.win)); }
    for (let i = 0; i < this.winches.length; i++) this.winches[i].step(dt);
    for (let i = 0; i < this.tracked.length; i++) this.tracked[i].getUserData().contacts.length = 0;
    if (this.logs.length) this._tetherLogs();
    this.world.step(dt, VEL_ITERS, POS_ITERS);
    if (this._dig.length) this._rut(dt);
    if (this.bridges.length) this._bridgeCheck(dt);
    this.stepCount++;
    this.time = this.stepCount * dt;
    for (let i = 0; i < this.postStep.length; i++) this.postStep[i](dt);
  }

  /** Is (x, tyre z) on an x face of a tall side shelf (trail.js isWall)? */
  _wallFace(x, z) {
    const T = TRAIL, W = this._walls || (this._walls = this.dyZones.filter(q => isWall(T, q)));
    for (const q of W) {
      if (Math.min(Math.abs(x - q.x0), Math.abs(x - q.x1)) > 0.15) continue;   // the vertical part (the chamfer below is climbable)
      const lo = q.side === 'R' ? 0 : q.z0 ?? -INF, hi = q.side === 'L' ? 0 : q.z1 ?? INF;
      if (z > lo - 0.4 && z < hi + 0.4) return true;
    }
    return false;
  }

  /** Once every tethered log has a bed pose: sink them into the ground (see LOG.embed) and drop their terrain collision. */
  _sinkLogs() {
    const T = this.logs.map(b => b.getUserData().tether);
    if (T.some(t => t && !t.bedded)) return;
    this._logSink = false;
    const todo = this.logs.filter((b, i) => T[i] && !T[i].sunk);
    const sink = new Map();
    for (const b of todo) { const t = b.getUserData().tether; sink.set(b, t.terr ? 2 * b.getUserData().r * t.embed : 0); }
    for (let pass = 0; pass < 4; pass++) for (const b of todo) {   // stacks follow the deepest log they rest on
      const t = b.getUserData().tether;
      if (t.terr) continue;
      for (const o of t.under) sink.set(b, Math.max(sink.get(b), sink.get(o) ?? 0));
    }
    for (const b of todo) {
      const t = b.getUserData().tether, d = sink.get(b);
      t.sunk = true;
      if (d > 0) {
        t.y -= d;
        for (let f = b.getFixtureList(); f; f = f.getNext()) f.setFilterData({ groupIndex: 0, categoryBits: CAT.LOG, maskBits: 0xffff & ~(CAT.TERRAIN | CAT.GROUND | LANES) });
        b.setAwake(true);
      }
      // rock (boulders, rock ridges): tonnes of stone don't rock under a truck -> frozen at the bed pose (static body)
      if (b.getUserData().rock) { b.setTransform(Vec2(t.x, t.y - LOG.hover), t.a); b.setStatic(); t.frozen = true; }
    }
  }

  /** Bed every tethered log at its settled pose (see LOG). Sleeping logs stay asleep (no wake). */
  _tetherLogs() {
    const L = LOG, g = this.gravity, F = this._cp;
    if (this._logSink) this._sinkLogs();
    for (let i = 0; i < this.logs.length; i++) {
      const b = this.logs[i], t = b.getUserData().tether;
      if (!t || t.frozen || !b.isAwake()) continue;
      const p = b.getPosition(), v = b.getLinearVelocity();
      let dx = p.x - t.x, dy = 0, gx = 0, gy = 0;
      if (!t.bedded) {
        // settling: falls onto its bed under gravity (x held); capture pose once at rest or after `settle` steps
        t.age++;
        if (t.age >= L.settle || (t.age > 8 && Math.hypot(v.x, v.y) < 0.02 && Math.abs(b.getAngularVelocity()) < 0.05)) {
          t.bedded = true; t.y = p.y + L.hover; t.a = b.getAngle(); this._logSink = true;
          t.terr = false; t.under = [];                                  // what it rests on (for _sinkLogs)
          for (let ce = b.getContactList(); ce; ce = ce.next) {
            if (!ce.contact.isTouching()) continue;
            const o = ce.other, od = o.getUserData();
            if (o === this.groundBody) t.terr = true;
            else if (od && od.kind === 'log' && o.getPosition().y < p.y) t.under.push(o);
          }
        }
      }
      if (t.bedded) { dy = p.y - t.y; gx = -t.m * g.x; gy = -t.m * g.y; }   // weight carried by the bed spring
      const d = Math.hypot(dx, dy), ex = d > L.slack ? Math.min(L.maxGain, 1 + L.gain * (d - L.slack) / L.slack) : 1;
      F.x = gx - t.k * ex * dx - t.c * v.x;
      const ev = dy < -L.vslack ? Math.min(L.vmaxGain, 1 + L.gain * (-dy - L.vslack) / L.vslack) : 1;
      F.y = t.bedded ? gy - t.kv * ev * dy - t.cv * v.y : 0;
      b.applyForceToCenter(F, false);
      if (!t.bedded) continue;
      let da = b.getAngle() - t.a;
      // past `twist` the bed stiffens and the bed angle creeps along at <= `creep` rad/s: a slow turn in place, never a free roller
      const ov = Math.abs(da) - L.twist;
      if (ov > 0) { const s = Math.min(ov, L.creep * FIXED_DT) * Math.sign(da); t.a += s; da -= s; }
      const xt = Math.abs(da) > L.twist ? Math.min(L.maxGain, 1 + L.gain * (Math.abs(da) - L.twist) / L.twist) : 1;
      b.applyTorque(-t.kt * xt * da - t.ct * b.getAngularVelocity(), false);
    }
  }

  // -- soft terrain ----------------------------------------------------------------------------
  /** Dig ruts under slipping tyres into the lanes their footprint covers (this step's contacts, post-solve velocities). */
  _rut(dt) {
    const R = RUT, T = TRAIL, nz = T.nz, LW = T.lane, D = this._dig, w = R.width, cp = this._cp, touched = this._touched;
    for (let j = 0; j < D.length; j += 4) {
      const P = D[j], body = D[j + 1], rec = D[j + 2], tire = D[j + 3];
      const N = rec.impulse / dt, dg = this.digOf(rec.surface);
      if (!(N > 0) || !(dg > 0)) continue;
      cp.x = rec.x; cp.y = rec.y;
      const v = body.getLinearVelocityFromWorldPoint(cp);
      const ex = Math.abs(v.x * rec.ny - v.y * rec.nx) - R.slip;   // contact-patch sliding speed above the threshold
      if (ex <= 0) continue;
      const base = R.rate * ex * Math.min(N / R.loadRef, 1.5) * dg * dt;
      if (!(base > 0)) continue;
      // lanes under the tyre footprint, weighted by covered fraction
      const za = tire.z - tire.w / 2, zb = tire.z + tire.w / 2;
      const l0 = Math.max(0, Math.floor(za / LW + nz / 2)), l1 = Math.min(nz - 1, Math.floor(zb / LW + nz / 2));
      if (l1 < l0) continue;
      const x = P.x, d = P.d, cap = P.cap, xr = rec.x + w, xl = rec.x - w;
      let i = P.i0, hi = P.i1 + 1;
      while (i < hi) { const m = (i + hi) >> 1; if (x[m] <= xl) i = m + 1; else hi = m; }
      for (; i <= P.i1 && x[i] < xr; i++) {
        const u = (x[i] - rec.x) / w;
        if (u <= -1 || !(cap[i] > 0)) continue;
        const kx = base * P.k[i] * 0.5 * (1 + Math.cos(Math.PI * u));
        if (!(kx > 0)) continue;
        for (let l = l0; l <= l1; l++) {
          const lo = (l - nz / 2) * LW, cov = (Math.min(zb, lo + LW) - Math.max(za, lo)) / LW, q = i * nz + l;
          const dd = Math.min(R.maxStep, kx * cov * (1 - d[q] / cap[i]));
          if (!(dd > 0)) continue;
          d[q] = Math.min(cap[i], d[q] + dd);
          if (i < P.lo) P.lo = i;
          if (i > P.hi) P.hi = i;
          if (l < P.llo) P.llo = l;
          if (l > P.lhi) P.lhi = l;
          if (!P.dirty) { P.dirty = true; touched.push(P); }
        }
      }
    }
    D.length = 0;
    if (!touched.length) return;
    for (const P of touched) {
      const { d, cap, x } = P;
      let lo = P.lo, hi = P.hi;
      // slump (per lane, along x): rut walls never steeper than RUT.wall -- the deep floor drags its neighbours down
      for (let l = P.llo; l <= P.lhi; l++) {
        let a = P.lo, b = P.hi;
        for (let i = a + 1; i <= P.i1; i++) {
          const q = i * nz + l, v = Math.min(cap[i], Math.max(d[q], d[q - nz] - R.wall * (x[i] - x[i - 1])));
          if (v !== d[q]) { d[q] = v; if (i > b) b = i; } else if (i > b) break;
        }
        for (let i = b - 1; i >= P.i0; i--) {
          const q = i * nz + l, v = Math.min(cap[i], Math.max(d[q], d[q + nz] - R.wall * (x[i + 1] - x[i])));
          if (v !== d[q]) { d[q] = v; if (i < a) a = i; } else if (i < a) break;
        }
        lo = Math.min(lo, a); hi = Math.max(hi, b);
      }
      for (let q = lo * nz, q1 = (hi + 1) * nz; q < q1; q++) P.df[q] = d[q];
      P.lo = Infinity; P.hi = -1; P.llo = nz; P.lhi = -1; P.dirty = false;
    }
    touched.length = 0;
    this.rutVersion++;
  }

  /** Rut depth (m, >= 0) at world x: at trail z (lanes interpolated), or the deepest lane when z is omitted. */
  rutDepthAt(x, z = null) {
    let best = 0;
    const T = TRAIL, nz = T.nz;
    for (const P of this.ruts) {
      for (let i = 1; i < P.n; i++) {
        const a = P.x[i - 1], b = P.x[i];
        if (x < a || x > b || b - a < 1e-9) continue;
        const t = (x - a) / (b - a);
        if (z != null) { best = Math.max(best, rutAt(T, P.d, i - 1, z) * (1 - t) + rutAt(T, P.d, i, z) * t); continue; }
        for (let l = 0; l < nz; l++) best = Math.max(best, P.d[(i - 1) * nz + l] * (1 - t) + P.d[i * nz + l] * t);
      }
    }
    return best;
  }

  /** Deepest rut anywhere in the level (m). */
  maxRutDepth() {
    let m = 0;
    for (const P of this.ruts) for (let q = 0; q < P.d.length; q++) if (P.d[q] > m) m = P.d[q];
    return m;
  }

  // -- plain-data views (the renderer consumes only these) -------------------------------------
  getStatic() {
    const L = this.level;
    return {
      name: L.name || '', scenery: L.scenery || null,
      ground: (L.ground || []).map((r, i) => ({ surface: r.surface || 'dirt', points: this._lay[i] || r.points })),   // TRAIL.res resample
      polygons: (L.polygons || []).map(p => ({ surface: p.surface || 'rock', points: p.points })),
      zones: this.zones.map(z => ({ ...z })),
      trail: { ...TRAIL },
      anchors: this.anchors.map(a => ({ ...a })),
      cans: this.cans.map(c => ({ x: c.x, y: c.y, z: c.z, fuel: c.fuel })),
      stages: (L.stages || []).map((st, i) => ({ x: st.x, name: st.name || `STAGE ${i + 1}` })),   // trailside posts (renderer)
      surfaces: this.surfaces,
      finish: L.finish || null,
      bottom: L.bottom ?? -30,
    };
  }

  getDynamic(out = { logs: [], planks: [] }) {
    out.logs.length = 0;
    out.planks.length = 0;
    for (const b of this.logs) {
      const p = b.getPosition();
      out.logs.push({ x: p.x, y: p.y, a: b.getAngle(), r: b.getUserData().r });
    }
    for (const b of this.planks) {
      const p = b.getPosition(), u = b.getUserData();
      out.planks.push({ x: p.x, y: p.y, a: b.getAngle(), hw: u.hw, hh: u.hh, b: u.bridge });
    }
    const cn = out.cans || (out.cans = []);
    cn.length = 0;
    for (const c of this.cans) cn.push(c.taken);
    this._rutView.ver = this.rutVersion;
    out.ruts = this._rutView;   // {ver, nz, lane, runs: [{run, d}]}: live Float32 rut grids (vertex-major n·nz, read-only)
    return out;
  }

  /** FNV-1a hash over every body's exact float state (determinism checks). */
  hash() {
    let h = 0x811c9dc5;
    const f = new Float64Array(1), u = new Uint8Array(f.buffer);
    const add = v => { f[0] = v; for (let i = 0; i < 8; i++) { h ^= u[i]; h = Math.imul(h, 0x01000193); } };
    for (let b = this.world.getBodyList(); b; b = b.getNext()) {
      const p = b.getPosition(), v = b.getLinearVelocity();
      add(p.x); add(p.y); add(b.getAngle()); add(v.x); add(v.y); add(b.getAngularVelocity());
    }
    for (const P of this.ruts) for (let q = P.i0 * TRAIL.nz; q < (P.i1 + 1) * TRAIL.nz; q++) if (P.d[q]) add(P.d[q]);
    return (h >>> 0).toString(16).padStart(8, '0');
  }
}
