// SPOTTER: small top-down view of the trail around the truck (x right = forward, z down = truck right / near side), drawn by the
// renderer in screen space into a HUD-measured box. Shapes only -- no path line, numbers or guides: it shows where
// things are, the player still judges the line. AUTO fades it in near steering-critical hazards (spotHazards) at crawl speed.
import { TRAIL, zoneAtZ } from './trail.js';
import { throwSprite, glowSprite } from './lights.js';

export const SPOT = { span: 9.6, lead: 1.1, leadRate: 2.5, margin: 0.55, follow: 3, near: 4.5, past: 2.2, edge: 0.45, speed: 5, wall: 0.25, bigLog: 0.45, drop: 0.45, fadeIn: 4, fadeOut: 1.8 };
export const SPOT_MODES = ['auto', 'on', 'off'];

/** Steering-critical hazards of a level, sorted by x: [{x0, x1, kind}]. Partial-width logs / boulders (side walls), big
 *  full-width logs (high-centre), partial shelves / ledges (zone dy >= SPOT.wall on one side), bridges (narrow deck over a
 *  gorge) and sharp drop-offs in the ground (>= SPOT.drop m down at > 45°). Small drive-over logs, mud / bog, low lanes skip. */
export function spotHazards(level) {
  const out = [], logs = level.logs || level.logDefs || [];
  for (const l of logs) {
    const part = l.z0 != null || l.z1 != null, r = l.r || 0.3;
    if (part || r >= SPOT.bigLog) out.push({ x0: l.x - r, x1: l.x + r, kind: part ? 'wall' : 'log' });
  }
  for (const z of level.zones || []) {
    const part = z.z0 != null || z.z1 != null || z.side === 'L' || z.side === 'R';
    if (part && (z.dy || 0) >= SPOT.wall && Number.isFinite(z.x0) && Number.isFinite(z.x1)) out.push({ x0: z.x0, x1: z.x1, kind: 'shelf' });
  }
  for (const b of level.bridges || []) out.push({ x0: Math.min(b.from[0], b.to[0]) - 0.5, x1: Math.max(b.from[0], b.to[0]) + 0.5, kind: 'bridge' });
  for (const g of level.ground || []) {
    const p = g.points || g.base || [];
    for (let i = 0; i + 1 < p.length; i++) {
      let j = i, d = 0;
      while (j + 1 < p.length && p[j + 1][0] > p[j][0] && (p[j][1] - p[j + 1][1]) > (p[j + 1][0] - p[j][0])) { d += p[j][1] - p[j + 1][1]; j++; }
      if (d >= SPOT.drop) out.push({ x0: p[i][0] - 0.5, x1: p[j][0] + 0.5, kind: 'drop' });
      i = Math.max(i, j);
    }
  }
  return out.sort((a, b) => a.x0 - b.x0);
}

/** AUTO trigger: a tyre's outer edge within SPOT.edge of the rim (falling risk, any speed: edgeGap = vehicle state), or a
 *  hazard overlapping [x - SPOT.past (rear clear), x + SPOT.near] while crawling (|speed| < SPOT.speed m/s). */
export function spotWanted(haz, x, speed, edgeGap = Infinity) {
  if (edgeGap < SPOT.edge) return true;
  if (Math.abs(speed || 0) >= SPOT.speed) return false;
  const a = x - SPOT.past, b = x + SPOT.near;
  for (const h of haz) { if (h.x0 > b) break; if (h.x1 >= a) return true; }
  return false;
}

const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
const h01 = (i, k = 0) => { const v = Math.sin(i * 127.1 + k * 311.7) * 43758.5453; return v - Math.floor(v); };
const TAU = Math.PI * 2;
const isRock = n => /rock|stone|boulder/.test(n || ''), isWet = n => /mud|bog|clay|slick|swamp/.test(n || ''), isWater = n => /ford|water|river/.test(n || '');

export class Spotter {
  constructor() { this.a = 0; this.haz = []; this.level = null; }

  setLevel(level) { this.level = level; this.haz = level ? spotHazards(level) : []; this.a = 0; this.zc = null; }

  /** r = Renderer (ctx, dpr, ground, colors, box, shape, atmo, _lampK, puddles, rivers, _surfColor, groundY); box = CSS px
   *  {x, y, w, h}; mode auto | on | off. */
  draw(r, v, dyn, box, mode, dt) {
    const want = !!box && mode !== 'off' && !!this.level && (mode === 'on' || spotWanted(this.haz, v.x, v.speed, v.edgeGap));
    this.a = Math.min(1, Math.max(0, this.a + (want ? SPOT.fadeIn : -SPOT.fadeOut) * (dt || 0)));
    if (this.a < 0.01 || !box || !(box.w > 40 && box.h > 30)) return;
    const ctx = r.ctx, d = r.dpr, X = box.x * d, Y = box.y * d, W = box.w * d, H = box.h * d, rad = Math.min(18 * d, H * 0.18);
    // framing (r14): centred on the truck's footprint (bumpers + tyres, yawed) with a lead toward travel (rearward in R /
    // rolling back), zoomed so the whole rig + SPOT.margin always fits (was nose-anchored: long rigs / yaw clipped the tail)
    const F = this._foot(r, v), dir = Math.abs(v.speed || 0) > 0.4 ? Math.sign(v.speed) : v.reverse ? -1 : 1;
    this.lead = this.lead == null || !(dt > 0) ? dir * SPOT.lead : this.lead + (dir * SPOT.lead - this.lead) * Math.min(1, SPOT.leadRate * dt);
    const m = SPOT.margin, span = Math.max(SPOT.span, 2 * (F.ex + Math.abs(this.lead) + m), 2 * (F.ez + m) * W / H);
    const s = W / span, x0 = F.cx + this.lead - span / 2, x1 = x0 + span, zh = H / 2 / s;
    const T = r.trail || TRAIL, edge = T.edge, berm = T.edge + T.bermW;
    // the view follows the truck across the trail (eased), clamped so it never looks past the rim, then hard-clamped so the
    // rig's footprint stays inside with the margin
    const zm = Math.max(0, (T.rim ?? berm) + 0.25 - zh), zt = Math.max(-zm, Math.min(zm, F.cz));
    this.zc = this.zc == null || !(dt > 0) ? zt : this.zc + (zt - this.zc) * Math.min(1, SPOT.follow * dt);
    const zr = Math.max(0, zh - F.ez - m); this.zc = Math.max(F.cz - zr, Math.min(F.cz + zr, this.zc));
    this._px = 1 / s;                                                  // one device px in metres (crisp outlines)
    const zc = this.zc, zf = zh + Math.abs(zc) + 1;
    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.globalAlpha = this.a;
    ctx.beginPath(); ctx.roundRect(X, Y, W, H, rad); ctx.clip();
    ctx.setTransform(s, 0, 0, s, X - x0 * s, Y + H / 2 - zc * s);     // world (x, z) metres, z down
    ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    ctx.fillStyle = '#2a2620'; ctx.fillRect(x0 - 1, -zf, x1 - x0 + 2, 2 * zf);   // under the berms (only seen at run ends)
    this._trail(r, ctx, x0, x1, edge, berm);
    this._borders(ctx, x0, x1, berm);
    this._zones(r, ctx, x0, x1, edge);
    this._puddles(r, ctx, x0, x1);
    this._bridges(r, ctx, dyn, x0, x1, zf);
    this._logs(r, ctx, dyn, x0, x1, edge + T.bermW * 0.5);
    this._truck(r, ctx, v, this._wedge(v, dt));
    // lighting: the scene's tint, then the truck's headlight cones (night) and a soft edge vignette
    const A = r.atmo || {};
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    if (A.tint) { ctx.fillStyle = A.tint; ctx.fillRect(X, Y, W, H); }
    const I = (A.lamp ?? 0) * (r._lampK || 0);                         // same strength as the main view's beams
    if (I > 0.01) this._lights(r, ctx, v, s, X - x0 * s, Y + H / 2 - zc * s, I);
    if (!this._vg || this._vgW !== W || this._vgH !== H) {            // edge vignette, cached per box size
      this._vgW = W; this._vgH = H; this._vg = ctx.createRadialGradient(0, 0, Math.min(W, H) * 0.35, 0, 0, Math.hypot(W, H) * 0.56);
      this._vg.addColorStop(0, 'rgba(0,0,0,0)'); this._vg.addColorStop(1, 'rgba(0,0,0,0.45)');
    }
    const vg = this._vg;
    ctx.translate(X + W / 2, Y + H / 2); ctx.fillStyle = vg; ctx.fillRect(-W / 2, -H / 2, W, H); ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.restore();
    ctx.save(); ctx.globalAlpha = this.a;
    ctx.beginPath(); ctx.roundRect(X + 0.5 * d, Y + 0.5 * d, W - d, H - d, rad);
    ctx.lineWidth = 1.5 * d; ctx.strokeStyle = 'rgba(255,255,255,0.2)'; ctx.stroke();
    if (this._fd !== d) { this._fd = d; this._font = `italic 700 ${Math.round(10 * d)}px "Exo 2", system-ui, sans-serif`; }
    ctx.font = this._font; ctx.textBaseline = 'top';
    ctx.fillStyle = 'rgba(238,242,246,0.55)'; ctx.fillText('SPOTTER', X + 10 * d, Y + 7 * d);
    ctx.restore();
  }

  /** The rig's yawed footprint (chassis boxes = bumpers, tyres +-R / +-(halfTrack + w/2)) -> world centre + half extents. */
  _foot(r, v) {
    const B = r.box || {}, sh = r.shape || {}, ht = sh.halfTrack || 0.9, tw = sh.tyreWidth || 0.45;
    let a = Infinity, b = -Infinity;
    for (const k in B) { a = Math.min(a, B[k].x0); b = Math.max(b, B[k].x1); }
    for (const o of v.wheels || []) { const lx = (o.hx ?? v.x) - v.x, R = o.radius || 0.5; a = Math.min(a, lx - R); b = Math.max(b, lx + R); }
    if (!Number.isFinite(a)) { a = -2.2; b = 2.2; }
    const yaw = v.yaw || 0, c = Math.abs(Math.cos(yaw)), sn = Math.abs(Math.sin(yaw)), cl = (a + b) / 2, hl = (b - a) / 2, hz = ht + tw / 2;
    const F = this._F || (this._F = {});
    F.cx = v.x + Math.cos(yaw) * cl; F.cz = (v.z || 0) + Math.sin(yaw) * cl; F.ex = hl * c + hz * sn; F.ez = hl * sn + hz * c;
    return F;
  }

  /** Wedged tyres: under throttle with the truck barely moving, a tyre pressing on a steep face (contact normal across the
   *  chassis, |n.up| < 0.6) -> per-wheel 0..1 glow (eased: no flicker), side = +1 face ahead of the hub / -1 behind. */
  _wedge(v, dt) {
    const W = this.wg || (this.wg = [0, 0, 0, 0]), S = this.wgS || (this.wgS = [1, 1, 1, 1]);
    const a = v.angle || 0, fx = Math.cos(a), fy = Math.sin(a), ux = -fy, uy = fx;
    const push = (v.throttle || 0) > 0.15 && v.engineOn !== false && Math.abs(v.speed || 0) < 0.6;
    (v.wheels || []).forEach((o, i) => {
      let hit = 0;
      if (push) for (const c of o.contacts || []) {
        if (!(c.force > 50) || Math.abs(c.nx * ux + c.ny * uy) >= 0.6) continue;
        hit = 1; S[i] = (c.x - (o.hx ?? o.x)) * fx + (c.y - (o.hy ?? o.y)) * fy >= 0 ? 1 : -1;
      }
      W[i] = Math.max(0, Math.min(1, W[i] + (hit ? 5 : -2.5) * (dt > 0 ? dt : 0.016)));
    });
    return W;
  }

  _trail(r, ctx, x0, x1, edge, berm) {                              // ground runs: surface colour, berm bands, slope relief
    const bc = r.level.surfaces[TRAIL.bermSurface] ? r._surfColor(TRAIL.bermSurface) : null;
    for (const g of r.ground) {
      const p = g.base;
      if (p.at(-1)[0] < x0 || p[0][0] > x1) continue;
      ctx.fillStyle = bc || g.cap || g.top; ctx.fillRect(Math.max(p[0][0], x0 - 1), -berm, Math.min(p.at(-1)[0], x1 + 1) - Math.max(p[0][0], x0 - 1), 2 * berm);
      ctx.fillStyle = g.top; ctx.fillRect(Math.max(p[0][0], x0 - 1), -edge, Math.min(p.at(-1)[0], x1 + 1) - Math.max(p[0][0], x0 - 1), 2 * edge);
      for (let i = 0; i + 1 < p.length; i++) {                        // relief: climbs catch light, descents fall into shade
        const a = p[i], b = p[i + 1];
        if (b[0] < x0 - 1 || a[0] > x1 + 1 || b[0] - a[0] < 1e-6) continue;
        const sl = (b[1] - a[1]) / (b[0] - a[0]), k = Math.min(0.32, Math.abs(sl) * 0.35);
        if (Math.abs(sl) > 0.9 && Math.abs(b[1] - a[1]) >= 0.2) {      // rock step / drop-off across the trail: crisp edge lines
          const up = sl > 0, xc = up ? b[0] : a[0], xf = up ? a[0] : b[0], px = this._px || 0.02;   // crest / foot
          ctx.globalAlpha *= 0.3; ctx.fillStyle = '#ffffff'; ctx.fillRect(xc - (up ? 1.5 * px : 0), -edge, 1.5 * px, 2 * edge); ctx.globalAlpha /= 0.3;
          ctx.globalAlpha *= 0.6; ctx.fillStyle = '#000000';
          ctx.fillRect(up ? xc : xc - 1.6 * px, -edge, 1.6 * px, 2 * edge);       // shadow line on the low side of the crest
          ctx.fillRect(xf - 0.8 * px, -edge, 1.6 * px, 2 * edge); ctx.globalAlpha /= 0.6;
        }
        if (k < 0.02) continue;
        ctx.globalAlpha *= k; ctx.fillStyle = sl > 0 ? '#fff4dc' : '#000000';
        ctx.fillRect(a[0], -berm, b[0] - a[0], 2 * berm); ctx.globalAlpha /= k;
      }
      const rock = isRock(g.surface), wet = isWet(g.surface);         // organic surface flecks (world-anchored)
      // batched: one path per fleck colour (was a fill per fleck)
      const i0f = Math.floor(Math.max(x0, p[0][0]) / 0.45), xe = Math.min(x1, p.at(-1)[0]);
      ctx.globalAlpha *= wet ? 0.175 : 0.35;
      for (const col of wet ? ['#e8eef0'] : ['#000000', '#ffffff']) {
        ctx.fillStyle = col; ctx.beginPath();
        for (let i = i0f; i * 0.45 < xe; i++) for (let k = 0; k < 3; k++) {
          if (!wet && (h01(i, k + 80) < 0.5) !== (col === '#000000')) continue;
          const x = i * 0.45 + h01(i, k + 60) * 0.45, z = (h01(i, k + 70) * 2 - 1) * edge;
          if (rock) { const rx = 0.12 + h01(i, k) * 0.12, a = h01(i, k + 2) * 3; ctx.moveTo(x + rx * Math.cos(a), z + rx * Math.sin(a)); ctx.ellipse(x, z, rx, 0.08 + h01(i, k + 1) * 0.08, a, 0, TAU); }
          else if (wet) { const rx = 0.3 + h01(i, k) * 0.3; ctx.moveTo(x + rx, z); ctx.ellipse(x, z, rx, 0.06, 0, 0, TAU); }
          else ctx.rect(x, z, 0.06, 0.06);
        }
        ctx.fill();
      }
      ctx.globalAlpha /= wet ? 0.175 : 0.35;
    }
  }

  _borders(ctx, x0, x1, berm) {                                    // trail drop-off: light crest, crisp dark edge, shadow beyond
    const px = this._px || 0.02, xa = x0 - 1, w = x1 - x0 + 2;
    for (const sg of [-1, 1]) {
      const e = sg * berm;
      ctx.globalAlpha *= 0.22; ctx.fillStyle = '#ffffff'; ctx.fillRect(xa, sg > 0 ? e - 3 * px : e + 1.5 * px, w, 1.5 * px); ctx.globalAlpha /= 0.22;
      ctx.globalAlpha *= 0.3; ctx.fillStyle = '#000000'; ctx.fillRect(xa, sg > 0 ? e : e - 0.22, w, 0.22);
      ctx.globalAlpha /= 0.3; ctx.globalAlpha *= 0.7; ctx.fillRect(xa, e - 0.9 * px, w, 1.8 * px); ctx.globalAlpha /= 0.7;
    }
  }

  _zones(r, ctx, x0, x1, edge) {                                    // surface lanes / shelves: own colour, raised = lit + shadow lip
    for (const q of r.level.zones || []) {
      if (!q.surface || q.x1 < x0 || q.x0 > x1) continue;
      const za = Math.max(-edge, q.z0 ?? (q.side === 'R' ? 0 : -edge)), zb = Math.min(edge, q.z1 ?? (q.side === 'L' ? 0 : edge));
      if (zb <= za || !zoneAtZ(q, (za + zb) / 2)) continue;
      const gy = r.groundY((q.x0 + q.x1) / 2);
      if ((q.y0 != null && gy < q.y0) || (q.y1 != null && gy > q.y1)) continue;
      const xa = Math.max(q.x0, x0 - 1), xb = Math.min(q.x1, x1 + 1), dy = q.dy || 0;
      ctx.fillStyle = r._surfColor(q.surface); ctx.fillRect(xa, za, xb - xa, zb - za);
      if (Math.abs(dy) >= 0.05) {
        const inner = Math.abs(za) < Math.abs(zb) ? za : zb, dir = inner === za ? -1 : 1, w = Math.min(0.45, 0.12 + Math.abs(dy) * 0.4);
        ctx.globalAlpha *= 0.5; ctx.fillStyle = dy > 0 ? '#000000' : '#000000';
        if (dy > 0) ctx.fillRect(xa, inner + (dir < 0 ? -w : 0), xb - xa, w);   // raised: shadow cast onto the trail beside it
        else ctx.fillRect(xa, inner + (dir < 0 ? 0 : -w), xb - xa, w);          // sunken: its own wall shades the hollow
        ctx.globalAlpha /= 0.5;
        if (dy > 0) { ctx.globalAlpha *= 0.18; ctx.fillStyle = '#ffffff'; ctx.fillRect(xa, za, xb - xa, zb - za); ctx.globalAlpha /= 0.18; }
        if (Math.abs(dy) >= 0.12) {                                   // step / shelf border: crisp dark outline, raised = lit inner rim
          const px = this._px || 0.02;
          ctx.globalAlpha *= 0.65; ctx.strokeStyle = '#000000'; ctx.lineWidth = 1.6 * px; ctx.strokeRect(xa, za, xb - xa, zb - za); ctx.globalAlpha /= 0.65;
          if (dy > 0) { ctx.globalAlpha *= 0.3; ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 1.2 * px; ctx.strokeRect(xa + 2.2 * px, za + 2.2 * px, xb - xa - 4.4 * px, zb - za - 4.4 * px); ctx.globalAlpha /= 0.3; }
        }
      }
      if (isWet(q.surface) || isWater(q.surface)) {                  // wet sheen streaks
        ctx.globalAlpha *= 0.22; ctx.fillStyle = isWater(q.surface) ? '#cfe6f2' : '#f0ece2';
        for (let i = Math.floor(xa / 0.7); i * 0.7 < xb; i++) { const x = i * 0.7 + h01(i, 3) * 0.6, z = za + h01(i, 4) * (zb - za);
          if (x > xa && x < xb) { ctx.beginPath(); ctx.ellipse(x, z, 0.35, 0.05 + h01(i, 5) * 0.05, 0, 0, TAU); ctx.fill(); } }
        ctx.globalAlpha /= 0.22;
      }
    }
  }

  _puddles(r, ctx, x0, x1) {
    for (const p of r.puddles || []) {
      if (p.x + p.rx < x0 || p.x - p.rx > x1) continue;
      ctx.globalAlpha *= 0.55; ctx.fillStyle = '#2c3a40'; ctx.beginPath(); ctx.ellipse(p.x, p.z, p.rx, p.rz * 1.6, 0, 0, TAU); ctx.fill();
      ctx.globalAlpha *= 0.5; ctx.fillStyle = '#b8d0dc'; ctx.beginPath(); ctx.ellipse(p.x - p.rx * 0.2, p.z - p.rz * 0.4, p.rx * 0.5, p.rz * 0.35, 0, 0, TAU); ctx.fill();
      ctx.globalAlpha /= 0.275;
    }
  }

  _bridges(r, ctx, dyn, x0, x1, zh) {                               // gorge shadow + river under the deck, plank logs, rope rails
    const bs = r.level.bridges || [];
    if (!bs.length) return;
    for (const b of bs) {
      const a = Math.min(b.from[0], b.to[0]), c = Math.max(b.from[0], b.to[0]);
      if (c < x0 || a > x1) continue;
      ctx.fillStyle = '#14130f'; ctx.fillRect(a, -zh - 1, c - a, 2 * zh + 2);
      for (const rv of r.rivers || []) if (rv.x1 > a && rv.x0 < c) { ctx.fillStyle = '#1d3a48'; ctx.fillRect(Math.max(a, rv.x0) + 0.3, -zh - 1, Math.min(c, rv.x1) - Math.max(a, rv.x0) - 0.6, 2 * zh + 2); }
    }
    const hw = TRAIL.half + 0.3;
    for (const p of dyn.planks || []) {
      if (p.x + p.hw < x0 || p.x - p.hw > x1) continue;
      ctx.fillStyle = '#4e3520'; ctx.beginPath(); ctx.roundRect(p.x - p.hw * 0.92, -hw, p.hw * 1.84, 2 * hw, p.hw * 0.6); ctx.fill();
      ctx.globalAlpha *= 0.2; ctx.fillStyle = '#e7c99a'; ctx.fillRect(p.x - p.hw * 0.5, -hw + 0.05, p.hw * 0.35, 2 * hw - 0.1); ctx.globalAlpha /= 0.2;
    }
    ctx.strokeStyle = '#8a6a44'; ctx.lineWidth = 0.05;
    for (const b of bs) { const a = Math.min(b.from[0], b.to[0]), c = Math.max(b.from[0], b.to[0]);
      if (c < x0 || a > x1) continue;
      for (const z of [-hw - 0.08, hw + 0.08]) { ctx.beginPath(); ctx.moveTo(a, z); ctx.quadraticCurveTo((a + c) / 2, z + 0.06 * Math.sign(z), c, z); ctx.stroke(); } }
  }

  _logs(r, ctx, dyn, x0, x1, LW) {                                  // boulders (chunk clusters) and trunks, soft drop shadows
    const defs = r.level.logDefs || [];
    (dyn.logs || []).forEach((l, i) => {
      if (l.x + l.r < x0 - 0.5 || l.x - l.r > x1 + 0.5) return;
      const dd = defs[i] || {}, part = dd.z0 != null || dd.z1 != null;
      const za = part ? Math.max(-LW, dd.z0 ?? -LW) : -LW * 0.85, zb = part ? Math.min(LW, dd.z1 ?? LW) : LW * 0.85;
      if (isRock(dd.surface) && !part && l.r < SPOT.bigLog) {         // low full-width rock ridge (main view: a slab across the trail)
        const w = l.r * 1.6, col = r._surfColor(dd.surface);
        ctx.globalAlpha *= 0.35; ctx.fillStyle = '#000000'; ctx.beginPath(); ctx.roundRect(l.x - w / 2 + 0.08, za + 0.06, w, zb - za, w * 0.3); ctx.fill(); ctx.globalAlpha /= 0.35;
        ctx.fillStyle = col; ctx.beginPath(); ctx.roundRect(l.x - w / 2, za, w, zb - za, w * 0.3); ctx.fill();
        ctx.globalAlpha *= 0.7; ctx.strokeStyle = '#0e0e0e'; ctx.lineWidth = 1.5 * (this._px || 0.02); ctx.stroke(); ctx.globalAlpha /= 0.7;
        ctx.globalAlpha *= 0.3; ctx.fillStyle = '#ffffff'; ctx.fillRect(l.x - w / 2 + w * 0.12, za + 0.08, w * 0.22, zb - za - 0.16);
        ctx.fillStyle = '#000000'; ctx.fillRect(l.x + w * 0.18, za + 0.08, w * 0.22, zb - za - 0.16);
        ctx.strokeStyle = '#000000'; ctx.lineWidth = 0.03;                 // joints across the slab
        for (let k = 1; k < 5; k++) { const z = za + (zb - za) * (k + (h01(i, k) - 0.5) * 0.5) / 5; ctx.beginPath(); ctx.moveTo(l.x - w / 2, z); ctx.lineTo(l.x + w / 2, z + 0.05); ctx.stroke(); }
        ctx.globalAlpha /= 0.3;
      } else if (isRock(dd.surface)) {
        const n = part ? Math.max(2, Math.round((zb - za) / (0.9 * l.r))) : Math.max(3, Math.round((zb - za) / 0.5)), dz = (zb - za) / n;
        const big = part || l.r > 0.4, col = r._surfColor(dd.surface);
        for (let pass = 0; pass < 2; pass++) for (let c = 0; c < n; c++) {
          const inner = Math.abs(za) < Math.abs(zb) ? 0 : n - 1, sc = big ? (c === inner ? 1 : 0.8 + 0.12 * Math.sin(i * 1.7 + c * 2.3)) : 0.7 + 0.3 * h01(i, c);
          const rx = l.r * sc, rz = Math.min(dz * 0.82, rx), cx = l.x + (big ? 0.1 * l.r * Math.sin(c * 2.1 + i) : 0), cz = za + (c + 0.5) * dz;
          ctx.beginPath();
          for (let k = 0; k < 9; k++) { const t = k * TAU / 9, q = 0.84 + 0.16 * h01(i * 13 + c, k);
            const px = cx + Math.cos(t) * rx * q + (pass ? 0 : 0.1), pz = cz + Math.sin(t) * rz * q + (pass ? 0 : 0.12); k ? ctx.lineTo(px, pz) : ctx.moveTo(px, pz); }
          ctx.closePath();
          if (!pass) { ctx.globalAlpha *= 0.4; ctx.fillStyle = '#000000'; ctx.fill(); ctx.globalAlpha /= 0.4; continue; }
          ctx.fillStyle = col; ctx.fill(); ctx.globalAlpha *= 0.8; ctx.strokeStyle = '#0e0e0e'; ctx.lineWidth = Math.max(0.03, 1.6 * (this._px || 0.02)); ctx.stroke(); ctx.globalAlpha /= 0.8;
          ctx.globalAlpha *= 0.28; ctx.fillStyle = '#ffffff'; ctx.beginPath(); ctx.ellipse(cx - rx * 0.25, cz - rz * 0.3, rx * 0.45, rz * 0.35, -0.4, 0, TAU); ctx.fill();
          ctx.fillStyle = '#000000'; ctx.beginPath(); ctx.ellipse(cx + rx * 0.3, cz + rz * 0.35, rx * 0.5, rz * 0.3, -0.4, 0, TAU); ctx.fill(); ctx.globalAlpha /= 0.28;
        }
      } else {                                                       // trunk: bark capsule along z, end rings
        const w = l.r;
        ctx.globalAlpha *= 0.4; ctx.fillStyle = '#000000'; ctx.beginPath(); ctx.roundRect(l.x - w + 0.08, za + 0.1, 2 * w, zb - za, w); ctx.fill(); ctx.globalAlpha /= 0.4;
        ctx.fillStyle = '#5e4128'; ctx.beginPath(); ctx.roundRect(l.x - w, za, 2 * w, zb - za, w); ctx.fill();
        ctx.globalAlpha *= 0.7; ctx.strokeStyle = '#1a1008'; ctx.lineWidth = 1.5 * (this._px || 0.02); ctx.stroke(); ctx.globalAlpha /= 0.7;
        ctx.strokeStyle = '#3f2a18'; ctx.lineWidth = 0.03;
        for (let k = -1; k <= 1; k++) { ctx.beginPath(); ctx.moveTo(l.x + k * w * 0.45, za + w * 0.6); ctx.lineTo(l.x + k * w * 0.45 + 0.02, zb - w * 0.6); ctx.stroke(); }
        ctx.fillStyle = '#b48a5a'; for (const z of [za + w * 0.5, zb - w * 0.5]) { ctx.beginPath(); ctx.ellipse(l.x, z, w * 0.8, w * 0.35, 0, 0, TAU); ctx.fill(); }
      }
    });
  }

  _truck(r, ctx, v, wg) {                                               // top-down rig: shadow, tyres (fronts steered), hood / cab / bed
    const sh = r.shape;
    if (!sh) return;
    const B = r.box || {}, ht = sh.halfTrack || 0.9, tw = sh.tyreWidth || 0.45, bw = ht * 0.86, C = r.colors || {};
    let xa = Infinity, xb = -Infinity;
    for (const k in B) { xa = Math.min(xa, B[k].x0); xb = Math.max(xb, B[k].x1); }
    if (!Number.isFinite(xa)) { xa = -2; xb = 2; }
    ctx.save(); ctx.translate(v.x, v.z || 0); ctx.rotate(v.yaw || 0);
    ctx.globalAlpha *= 0.4; ctx.fillStyle = '#000000'; ctx.beginPath(); ctx.roundRect(xa + 0.12, -ht - tw / 2 + 0.15, xb - xa, 2 * ht + tw, 0.3); ctx.fill(); ctx.globalAlpha /= 0.4;
    for (const o of v.wheels || []) {
      const lx = (o.hx ?? v.x) - v.x, lz = o.lz ?? (o.side === 'L' ? -ht : ht), R = o.radius || 0.5;
      ctx.save(); ctx.translate(lx, lz); if (lx > 0) ctx.rotate(v.steer || 0);
      ctx.fillStyle = '#141518'; ctx.beginPath(); ctx.roundRect(-R, -tw / 2, 2 * R, tw, tw * 0.3); ctx.fill();
      ctx.strokeStyle = '#2c2e33'; ctx.lineWidth = 0.035;
      ctx.beginPath(); for (let k = -3; k <= 3; k++) { ctx.moveTo(k * R * 0.28, -tw * 0.42); ctx.lineTo(k * R * 0.28 + 0.05, tw * 0.42); } ctx.stroke();
      ctx.restore();
    }
    ctx.fillStyle = C.frame || '#2a2d33'; ctx.fillRect(xa + 0.1, -ht * 0.35, xb - xa - 0.2, ht * 0.7);
    for (const k of ['bed', 'hood', 'cab', 'frame']) {
      const b = B[k];
      if (!b || k === 'frame') continue;
      const w = k === 'cab' ? bw * 0.92 : bw;
      ctx.fillStyle = C[k] || C.body || '#d4622b'; ctx.beginPath(); ctx.roundRect(b.x0, -w, b.x1 - b.x0, 2 * w, 0.14); ctx.fill();
      if (k === 'cab') {
        ctx.globalAlpha *= 0.3; ctx.fillStyle = '#000000'; ctx.beginPath(); ctx.roundRect(b.x0 + 0.12, -w + 0.12, b.x1 - b.x0 - 0.3, 2 * w - 0.24, 0.1); ctx.fill(); ctx.globalAlpha /= 0.3;
        ctx.fillStyle = '#9fb8cc'; ctx.globalAlpha *= 0.7; ctx.beginPath(); ctx.roundRect(b.x1 - 0.22, -w * 0.82, 0.14, w * 1.64, 0.05); ctx.fill(); ctx.globalAlpha /= 0.7;
      }
    }
    ctx.globalAlpha *= 0.25; ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 0.03;
    ctx.beginPath(); ctx.roundRect(xa, -bw, xb - xa, 2 * bw, 0.14); ctx.stroke(); ctx.globalAlpha /= 0.25;
    // wedged tyre: a warm contact arc hugging the tread on the side pressing into the face (soft halo + crisp line)
    (v.wheels || []).forEach((o, i) => {
      const k = wg ? wg[i] : 0;
      if (!(k > 0.02)) return;
      const lx = (o.hx ?? v.x) - v.x, lz = o.lz ?? (o.side === 'L' ? -ht : ht), R = o.radius || 0.5, sd = this.wgS ? this.wgS[i] : 1;
      const a0 = sd > 0 ? -1.05 : Math.PI - 1.05, a1 = a0 + 2.1, px = this._px || 0.02;
      ctx.save(); ctx.translate(lx, lz); if (lx > 0) ctx.rotate(v.steer || 0);
      ctx.beginPath(); ctx.ellipse(0, 0, R + 0.1, tw / 2 + 0.1, 0, a0, a1);
      ctx.globalAlpha *= 0.28 * k; ctx.strokeStyle = '#ff9a3c'; ctx.lineWidth = 0.17; ctx.stroke(); ctx.globalAlpha /= 0.28 * k;
      ctx.globalAlpha *= 0.9 * k; ctx.strokeStyle = '#ffc46a'; ctx.lineWidth = 2.4 * px; ctx.stroke(); ctx.globalAlpha /= 0.9 * k;
      ctx.restore();
    });
    ctx.restore();
  }

  _lights(r, ctx, v, s, ox, oz, I) {                                // ONE merged, low-glare ground throw (cached sprite), additive
    const T = this._throw || (this._throw = throwSprite(0.42)), Bx = r.box || {};   // built once (shared cache with the main view)
    if (!T) return;
    let xb = -Infinity; for (const k in Bx) xb = Math.max(xb, Bx[k].x1);
    if (!Number.isFinite(xb)) xb = 2;
    const yaw = v.yaw || 0, c = Math.cos(yaw), sn = Math.sin(yaw), L = 6 * s, hw = 2.1 * s;
    ctx.save(); ctx.globalCompositeOperation = 'lighter'; ctx.globalAlpha = 0.16 * I;
    const px = ox + (v.x + c * (xb + 0.1)) * s, pz = oz + ((v.z || 0) + sn * (xb + 0.1)) * s;
    ctx.setTransform(c, sn, -sn, c, px, pz); ctx.drawImage(T.img, 0, -hw, L, 2 * hw);
    // rear (dusk / night): two small red tail glows (brighter braking), a short red spill, reverse = short warm-white pool
    if ((r.atmo?.lamp ?? 0) >= 0.4) {
      const R = this._red || (this._red = glowSprite(255, 44, 30, 2.2)), W = this._rev || (this._rev = glowSprite(255, 236, 214, 1.8));
      let xr = Infinity; for (const k in Bx) xr = Math.min(xr, Bx[k].x0);
      if (!Number.isFinite(xr)) xr = -2;
      const br = Math.min(1, Math.max(0, v.brake || 0)), ht = (r.shape.halfTrack || 0.9) * 0.8;
      ctx.setTransform(c * s, sn * s, -sn * s, c * s, ox + (v.x + c * xr) * s, oz + ((v.z || 0) + sn * xr) * s);   // metres, rear centre
      ctx.globalAlpha = (0.08 + 0.06 * br) * I; ctx.drawImage(R, -1.0, -1.0, 1.0, 2.0);
      if (v.reverse) { ctx.globalAlpha = 0.12 * I; ctx.drawImage(W, -2.4, -1.2, 2.2, 2.4); }
      ctx.globalAlpha = (0.3 + 0.25 * br) * I;
      for (const z of [-ht, ht]) ctx.drawImage(R, -0.18, z - 0.18, 0.36, 0.36);
    }
    ctx.restore();
  }

}
