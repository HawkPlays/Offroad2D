// engine/trail.js — lateral trail geometry for the 2.5D layer. Pure math (no planck, no DOM): shared by
// physics.js (per-wheel lane colliders, rut grid) and renderer.js (terrain slabs), so both see the same surface.
// World z runs across the trail: +z = truck right = near side of the 2.5D camera, -z = far / left. Units m.
//   half / soft   where the berm push on the nose is full / starts (vehicle.js; not a clamp: a held steer beats it)
//   rim / dropW / dropH  wheel |z| where the trail top ends (renderer slab edge) / drop-off blend width / depth (edgeDrop)
//   edge / bermW / bermH  soft dirt berms: wheel |z| where they start rising, rise width, height
//   bermSurface   surface of the berm slopes (|z| > edge + 0.3·bermW)
//   res           lane-collider resample spacing (m)        win   half-width of the per-step lane refresh window (m)
//   lane / nz     rut-grid lane width (m) / lane count, grid centred on z = 0
//   ramp / zRamp  zone `dy` (height offset) blend width across x0/x1 and across z bounds (m); wallDy / wallCh: tall partial
//                 shelves (isWall) get a chamfer + vertical x face instead
//   lipZ / lipW / lipH   edge lip: a raised ridge on top of each berm, crest at |z| = lipZ, half-width lipW, height lipH. It holds
//                 the tyres on the trail (vehicle.js: a tyre's outer edge can't cross the crest outward) unless steering is held hard
//                 into it; its outer foot (lipZ + lipW) is the rim: the lip sits right at the real edge
export const TRAIL = { half: 1.15, soft: 1.0, edge: 1.9, bermW: 0.6, bermH: 0.22, lipZ: 2.65, lipW: 0.2, lipH: 0.14, rim: 2.85, dropW: 0.3, dropH: 8,
  bermSurface: 'berm', res: 0.25, win: 1.6, lane: 0.2, nz: 26, ramp: 0.5, zRamp: 0.15, treadIn: 0.1, wallDy: 0.6, wallCh: 0.06 };
/** Tall partial-width shelf (dy >= wallDy, side / z bounds, not a `climb` formation): a rock WALL. Its x faces = a 45° chamfer
 *  wallCh tall at the foot (cushions a head-on hit over ~0.3 m) then vertical (physics inserts lane vertices at the corners;
 *  tyre grip on the face capped at LOG.faceMu): a tyre meets a face it can't climb. A 0.5 m ramp let a strong truck charge
 *  up and launch; a bare vertical face stopped it dead and grip let the tyre crawl straight up it (r7/r8 'jolts'). */
export const isWall = (T, q) => q.dy >= (T.wallDy ?? 9) && !q.climb && (q.side === 'L' || q.side === 'R' || q.z0 != null || q.z1 != null);

const smooth = t => (t <= 0 ? 0 : t >= 1 ? 1 : t * t * (3 - 2 * t));
const BIG = 1e8;

/** Does zone q apply at world z? side L = far half (z < 0), R = near half (z >= 0), optional z0 / z1.
 *  z == null (chassis, logs: no lane) matches only zones without any z restriction. */
export function zoneAtZ(q, z) {
  const zs = q.side === 'L' || q.side === 'R' || q.z0 > -BIG || q.z1 < BIG;
  if (z == null) return !zs;
  if ((q.side === 'L' && z >= 0) || (q.side === 'R' && z < 0)) return false;
  return z >= q.z0 && z <= q.z1;
}

/** Edge-lip height (m) at z: a cosine ridge, crest lipH at |z| = lipZ, 0 beyond lipZ ± lipW. */
export function lipAt(T, z) {
  const u = (Math.abs(z) - (T.lipZ ?? 99)) / (T.lipW || 0.25);
  return u > -1 && u < 1 ? (T.lipH || 0) * 0.5 * (1 + Math.cos(Math.PI * u)) : 0;
}

/** Highest lip under a tyre footprint [|z| - w/2, |z| + w/2]: a tyre whose edge reaches the ridge rides up onto it (a speed
 *  bump), it never sinks its outer half through it. */
export function lipUnder(T, z, w) {
  const a = Math.abs(z) - w / 2, b = Math.abs(z) + w / 2, L = T.lipZ ?? 99;
  if (Math.abs(z) > L) return lipAt(T, z);                          // centre over the crest: it rolls off down the outside
  return b >= L ? (T.lipH || 0) : Math.max(lipAt(T, a), lipAt(T, b));
}

/** Trail height offset (m) at (x, z) on top of the ground run (base height y): berm rise + every zone's `dy`, blended. */
export function laneOffset(T, zones, x, y, z, zq = z, hw = 0) {   // zq: z the zones (shelves, steps) are sampled at (steered footprint)
  // hw (physics tyre lanes): raised zones are also sampled at the tread edges z ± hw (hw = half width − T.treadIn): a tyre whose
  // tread overlaps a shelf / step by more than ~treadIn rides on it or meets its face (r13: centre-line-only let a tyre drive
  // along 0.2 m INSIDE a 0.9 m wall shelf; the renderer, rightly, hid it behind the rock = 'transparent boulder clipping')
  let off = T.bermH * smooth((Math.abs(z) - T.edge) / T.bermW) + lipAt(T, z);
  for (let i = 0; i < zones.length; i++) {
    const q = zones[i];
    if (!q.dy || y < q.y0 || y > q.y1) continue;
    let wx;
    if (isWall(T, q)) {                                // wall shelf: 45° chamfer (wallCh m tall) at the foot, then a vertical face
      const d = Math.min(x - q.x0, q.x1 - x), c = T.wallCh;
      wx = d > 0 ? 1 : d <= -c ? 0 : Math.min(1, (d + c) / q.dy);
    } else wx = smooth((x - q.x0) / T.ramp + 0.5) * smooth((q.x1 - x) / T.ramp + 0.5);
    if (!(wx > 0)) continue;
    const lo = q.side === 'R' ? 0 : q.z0, hi = q.side === 'L' ? 0 : q.z1;
    const wzAt = (u) => smooth((u - lo) / T.zRamp + 0.5) * smooth((hi - u) / T.zRamp + 0.5);
    // r16: a WALL shelf is never sampled along the heading (a yawed + steered tyre, e.g. creeping back in reverse turned, had its
    // footprint swung ~0.5 m off the shelf = the tyre drove straight through the wall); steps / ledges stay lenient
    const zz = zq !== z && isWall(T, q) ? z : zq;
    let wz;
    if (hw > 0 && isWall(T, q)) {                      // physics tyre vs wall shelf: a solid block, not a ramp. Any tread edge inside its z span
      wz = wzAt(z - hw) > 0.5 || wzAt(z) > 0.5 || wzAt(z + hw) > 0.5 ? 1 : 0;   // = the face is there (head-on, alongside, reversing: no creeping up it
    } else wz = zz === z ? wzAt(z) : Math.min(wzAt(z), wzAt(zz));   // from the side, no passing through); steps stay lenient: least of centre / footprint
    if (hw > 0 && q.dy > 0 && wz < 1 && !isWall(T, q)) for (const s of [-hw, hw]) wz = Math.max(wz, zz === z ? wzAt(z + s) : Math.min(wzAt(z + s), wzAt(zz + s)));
    off += q.dy * wx * wz;
  }
  return off;
}

/** Depth (m, >= 0) of the edge drop-off under a wheel at z: 0 on the trail, dropH past rim + dropW (smooth lip). Physics lanes
 *  only (renderer slabs stop at the rim): a tyre rolling past the rim loses its ground and falls -- no wall, no clamp. */
export const edgeDrop = (T, z) => (T.dropH || 0) * smooth((Math.abs(z) - (T.rim ?? T.edge + T.bermW + 0.4)) / (T.dropW || 0.3));

/** Centre z of rut-grid lane l. */
export const laneZ = (T, l) => (l + 0.5 - T.nz / 2) * T.lane;

/** Rut depth at vertex i, linear across lane centres at z (d: vertex-major n·nz array). */
export function rutAt(T, d, i, z) {
  const f = z / T.lane + T.nz / 2 - 0.5, l = Math.floor(f), t = f - l, nz = T.nz, b = i * nz;
  const a0 = l >= 0 && l < nz ? d[b + l] : 0, a1 = l + 1 >= 0 && l + 1 < nz ? d[b + l + 1] : 0;
  return a0 + (a1 - a0) * t;
}

/** Mean rut depth under a tyre footprint [z - w/2, z + w/2] at vertex i (lane overlap weighted): what the tyre rests on. */
export function rutUnder(T, d, i, z, w) {
  const L = T.lane, nz = T.nz, h = nz / 2, a = z - w / 2, b = z + w / 2, b0 = i * nz;
  let s = 0;
  for (let l = Math.max(0, Math.floor(a / L + h)), l1 = Math.min(nz - 1, Math.floor(b / L + h)); l <= l1; l++) {
    const lo = (l - h) * L, ov = Math.min(b, lo + L) - Math.max(a, lo);
    if (ov > 0) s += ov * d[b0 + l];
  }
  return s / w;
}
