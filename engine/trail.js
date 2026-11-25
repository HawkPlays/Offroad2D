// engine/trail.js — lateral trail geometry for the 2.5D layer. Pure math (no planck, no DOM): shared by
// physics.js (per-wheel lane colliders, rut grid) and renderer.js (terrain slabs), so both see the same surface.
// World z runs across the trail: +z = truck right = near side of the 2.5D camera, -z = far / left. Units m.
//   half / soft   truck-centre |z| hard limit / where the berms start turning the nose back (vehicle.js)
//   edge / bermW / bermH  soft dirt berms: wheel |z| where they start rising, rise width, height
//   bermSurface   surface of the berm slopes (|z| > edge + 0.3·bermW)
//   res           lane-collider resample spacing (m)        win   half-width of the per-step lane refresh window (m)
//   lane / nz     rut-grid lane width (m) / lane count, grid centred on z = 0
//   ramp / zRamp  zone `dy` (height offset) blend width across x0/x1 and across z bounds (m)
export const TRAIL = { half: 1.15, soft: 0.85, edge: 1.6, bermW: 0.7, bermH: 0.22, bermSurface: 'berm', res: 0.25, win: 1.6,
  lane: 0.2, nz: 26, ramp: 0.5, zRamp: 0.15 };

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

/** Trail height offset (m) at (x, z) on top of the ground run (base height y): berm rise + every zone's `dy`, blended. */
export function laneOffset(T, zones, x, y, z) {
  let off = T.bermH * smooth((Math.abs(z) - T.edge) / T.bermW);
  for (let i = 0; i < zones.length; i++) {
    const q = zones[i];
    if (!q.dy || y < q.y0 || y > q.y1) continue;
    const wx = smooth((x - q.x0) / T.ramp + 0.5) * smooth((q.x1 - x) / T.ramp + 0.5);
    if (!(wx > 0)) continue;
    const lo = q.side === 'R' ? 0 : q.z0, hi = q.side === 'L' ? 0 : q.z1;
    off += q.dy * wx * smooth((z - lo) / T.zRamp + 0.5) * smooth((hi - z) / T.zRamp + 0.5);
  }
  return off;
}

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
