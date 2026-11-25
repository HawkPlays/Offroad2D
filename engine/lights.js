// engine/lights.js — headlight sprites, built once (pixel-exact feathering, no banding) and reused every frame: the renderer and
// the SPOTTER draw a beam or glow with ONE drawImage each (additive 'lighter'), never a per-frame gradient, filter or shadowBlur.
// Beam sprite: cone along +x from the left-centre (the lens), width = 2·tan(spread) of its length. Two-stage angular falloff
// (tight warm halogen core + wide soft spill) × distance falloff (soft start off the lens, smooth fade to nothing at the tip).
const cache = new Map();

const smooth = t => (t <= 0 ? 0 : t >= 1 ? 1 : t * t * (3 - 2 * t));
const mk = (w, h) => {
  if (typeof OffscreenCanvas === 'function') return new OffscreenCanvas(w, h);
  if (typeof document !== 'undefined') { const c = document.createElement('canvas'); c.width = w; c.height = h; return c; }
  return null;
};

/** Beam sprite {img, spread}: spread = half-angle (rad) the sprite covers at its edge; core / spill = angular widths (share of
 *  spread); reach = where along the length the light has faded out (0..1). Cached per parameter set. */
export function beamSprite(spread = 0.32, core = 0.34, spill = 0.85, reach = 1, w = 256, h = 128) {
  const key = `b${spread}|${core}|${spill}|${reach}|${w}|${h}`;
  if (cache.has(key)) return cache.get(key);
  const c = mk(w, h);
  if (!c) { cache.set(key, null); return null; }
  const g = c.getContext('2d'), im = g.createImageData(w, h), d = im.data;
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    const u = (x + 0.5) / w, v = ((y + 0.5) / h) * 2 - 1, t = Math.abs(v) / Math.max(u, 0.02);   // t = 1 at the cone edge
    const ang = 0.8 * Math.exp(-((t / core) ** 2) * 2.2) + 0.5 * Math.exp(-((t / spill) ** 2) * 2.4);   // core + spill
    const dist = smooth(u / 0.22) * (1 - smooth((u - 0.2) / (reach - 0.2))) ** 1.15;              // soft start off the lens -> fade
    const a = Math.min(1, ang * dist * (1 - smooth((t - 0.92) / 0.08)) / 1.15);                     // soft clip at the edge
    const hot = 0.6 * Math.exp(-((t / core) ** 2) * 3) * smooth(u / 0.3) * (1 - u);                  // whiter at the hot core
    const i = (y * w + x) * 4;
    d[i] = 255; d[i + 1] = Math.round(206 + 38 * hot); d[i + 2] = Math.round(150 + 80 * hot); d[i + 3] = Math.round(a * 255);
  }
  g.putImageData(im, 0, 0);
  const s = { img: c, spread };
  cache.set(key, s);
  return s;
}

/** Round soft glow sprite (lens bloom / ground pool / ambient spill): rgb + falloff power, unit radius = half the size. */
export function glowSprite(r = 255, gg = 232, b = 190, pow = 2, w = 64) {
  const key = `g${r}|${gg}|${b}|${pow}|${w}`;
  if (cache.has(key)) return cache.get(key);
  const c = mk(w, w);
  if (!c) { cache.set(key, null); return null; }
  const g = c.getContext('2d'), im = g.createImageData(w, w), d = im.data, h = w / 2;
  for (let y = 0; y < w; y++) for (let x = 0; x < w; x++) {
    const q = Math.hypot(x + 0.5 - h, y + 0.5 - h) / h, a = q >= 1 ? 0 : (1 - smooth(q)) ** pow, i = (y * w + x) * 4;
    d[i] = r; d[i + 1] = gg; d[i + 2] = b; d[i + 3] = Math.round(a * 255);
  }
  g.putImageData(im, 0, 0);
  cache.set(key, c);
  return c;
}

/** Ground throw sprite {img, w0}: the headlights' footprint on the ground, length along +x (u 0 = bumper, 1 = far tip), width
 *  across y (full height = full far width). Starts as wide as the lamp pair (w0 of the far width: no V-gap at the bumper),
 *  flat-topped across (no hotspot where two beams would overlap), feathered edges, soft near-bumper spill, smooth distance fade. */
export function throwSprite(w0 = 0.42, w = 256, h = 128) {
  const key = `t${w0}|${w}|${h}`;
  if (cache.has(key)) return cache.get(key);
  const c = mk(w, h);
  if (!c) { cache.set(key, null); return null; }
  const g = c.getContext('2d'), im = g.createImageData(w, h), d = im.data;
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    const u = (x + 0.5) / w, v = Math.abs(((y + 0.5) / h) * 2 - 1);
    const hw = w0 + (1 - w0) * smooth(u / 0.75), q = v / hw;
    const lat = 1 - smooth((q - 0.2) / 0.72);                                           // flat top, long feather, 0 before the edge
    const dist = smooth((u + 0.05) / 0.16) * (1 - smooth((u - 0.16) / 0.84)) ** 1.5;   // soft spill off the bumper -> fade
    const a = Math.min(1, lat * dist), warm = 1 - u;                                     // a touch whiter near the truck
    const i = (y * w + x) * 4;
    d[i] = 255; d[i + 1] = Math.round(176 + 24 * warm); d[i + 2] = Math.round(96 + 36 * warm); d[i + 3] = Math.round(a * 255);
  }
  g.putImageData(im, 0, 0);
  const s = { img: c, w0 };
  cache.set(key, s);
  return s;
}

/** Side-profile headlight fan {img}: apex at the left-centre (the lens), axis +x, rows span the fan's full angle (t −1..1 at
 *  every u; drawn into a rect of half-height L·tan(angle) so one sprite fits any angle). Flat-ish across with long feathers to
 *  0 before both edges, a soft dark start off the lens (no glare blob) and a smooth fade to 0 at the tip: no hard edge anywhere. */
export function fanSprite(w = 256, h = 160) {
  const key = `f${w}|${h}`;
  if (cache.has(key)) return cache.get(key);
  const c = mk(w, h);
  if (!c) { cache.set(key, null); return null; }
  const g = c.getContext('2d'), im = g.createImageData(w, h), d = im.data;
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    const u = (x + 0.5) / w, t = Math.abs(((y + 0.5) / h) * 2 - 1) / Math.max(u, 0.02);
    const lat = 1 - smooth((t - 0.12) / 0.8);                                            // 0 from t 0.92 on
    const dist = smooth((u - 0.03) / 0.2) * (1 - smooth((u - 0.2) / 0.8)) ** 1.6;
    const a = Math.min(1, lat * dist), warm = 1 - u;
    const i = (y * w + x) * 4;
    d[i] = 255; d[i + 1] = Math.round(176 + 24 * warm); d[i + 2] = Math.round(96 + 36 * warm); d[i + 3] = Math.round(a * 255);
  }
  g.putImageData(im, 0, 0);
  const s = { img: c };
  cache.set(key, s);
  return s;
}
