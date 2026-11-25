// engine/tilt.js — phone-tilt steering math (pure, no DOM; hud.js wires the sensors). The steering angle is the roll of the
// screen in its own plane, read off the gravity vector: devicemotion accelerationIncludingGravity (preferred) or the up
// vector rebuilt from deviceorientation beta / gamma (fallback). Both are device-frame vectors, rotated into screen axes by
// the screen orientation angle, so either landscape side reads the same and how far the phone is reclined does not move
// the centre: level screen = straight, no calibration.

/** Tunables: dead (°) soft deadzone · knee (0..1) width of its soft edge · expo response curve (0 = linear, 1 = cubic) ·
 *  1€ filter on the vector (minHz cutoff at rest, beta Hz per unit/s of vector speed, dHz low-pass on the signed velocity:
 *  zero-mean hand jitter cancels there, a real turn does not) · outTau (s) light
 *  low-pass on the steer value · flat0 / flat1: screen-up share of gravity where the exact in-plane roll blends to the
 *  edge-tilt reading of a phone lying flat · flipOn / flipTau: sign detector (iOS reports gravity, not its reaction). */
export const TILT = { dead: 2.5, knee: 0.08, expo: 0.25, minHz: 1.2, beta: 1, dHz: 1, outTau: 0.03,
  flat0: 0.12, flat1: 0.3, flipOn: 0.35, flipTau: 0.5 };

const R = Math.PI / 180, clamp = (v, a, b) => Math.max(a, Math.min(b, v));

/** Device-frame up vector from DeviceOrientation beta / gamma (°). */
export function upFromEuler(beta, gamma) {
  const b = beta * R, c = gamma * R;
  return [-Math.sin(c) * Math.cos(b), Math.sin(b), Math.cos(c) * Math.cos(b)];
}

/** Device-frame vector -> screen axes [sx right, sy up, sz out of the screen] for screen angle (°, 0 / 90 / 180 / 270). */
export function toScreen(ux, uy, uz, angle = 0) {
  const a = angle * R, ca = Math.cos(a), sa = Math.sin(a);
  return [ux * ca - uy * sa, ux * sa + uy * ca, uz];
}

/** Screen roll (°, + = clockwise = steer right) of a device-frame up vector (any length). Held up: the exact in-plane roll
 *  atan2(−sx, sy), the same however far the phone is reclined; lying flat (sy -> 0, where atan2 turns to noise): asin(−sx),
 *  i.e. tilting the edges; smoothstep blend over sy = flat0 .. flat1. */
export function screenRoll(ux, uy, uz, angle = 0) {
  const n = Math.hypot(ux, uy, uz);
  if (!(n > 1e-9)) return 0;
  const [sx, sy] = toScreen(ux / n, uy / n, uz / n, angle);
  const flat = Math.asin(clamp(-sx, -1, 1)), k = clamp((sy - TILT.flat0) / (TILT.flat1 - TILT.flat0), 0, 1), w = k * k * (3 - 2 * k);
  return (w > 0 ? w * Math.atan2(-sx, sy) + (1 - w) * flat : flat) / R;
}

/** Legacy helper: screen roll (°) straight from DeviceOrientation beta / gamma. */
export const tiltRoll = (beta, gamma, angle = 0) => screenRoll(...upFromEuler(beta, gamma), angle);

/** Roll (°) -> steer −1..1: soft deadzone (no kink at its edge), full lock at maxDeg, expo curve (fine control near centre). */
export function tiltSteer(roll, maxDeg = 25, o = TILT) {
  const m = Math.abs(roll);
  if (!(m > o.dead)) return 0;
  const x = Math.min(1, (m - o.dead) / Math.max(1, maxDeg - o.dead)), k = o.knee;
  const s = (x < k ? x * x / (2 * k) : x - k / 2) / (1 - k / 2);      // quadratic knee: slope 0 -> 1 over the first k
  return Math.sign(roll) * ((1 - o.expo) * s + o.expo * s * s * s);
}

/** Smooths device-frame gravity samples (1€ filter: steady at rest, quick on a real turn) and returns the screen roll (°).
 *  push(x, y, z, tSec, angle, detectFlip): detectFlip for devicemotion: a phone held to play has gravity's reaction pointing
 *  up the screen and / or out of it (sy + sz > 0); a source that reports gravity itself (iOS) reads negative there and is
 *  flipped (EMA + hysteresis, so a sideways moment never flips it). */
export class TiltFilter {
  constructor(o = {}) { this.o = { ...TILT, ...o }; this.reset(); }
  reset() { this.v = null; this.p = null; this.dv = [0, 0, 0]; this.t = null; this.inv = 1; this.ie = null; this.roll = 0; }
  push(x, y, z, t, angle = 0, detectFlip = false) {
    const n = Math.hypot(x, y, z), o = this.o;
    if (!(n > 1e-6) || !Number.isFinite(n) || !Number.isFinite(t)) return this.roll;
    x /= n; y /= n; z /= n;
    const dt = this.t == null ? 0 : clamp(t - this.t, 0, 0.25);
    if (this.t == null || t > this.t) this.t = t;
    if (detectFlip) {
      const [, sy, sz] = toScreen(x, y, z, angle), s = sy + sz;
      this.ie = this.ie == null ? s : this.ie + (s - this.ie) * (1 - Math.exp(-dt / o.flipTau));
      const inv = this.ie < -o.flipOn ? -1 : this.ie > o.flipOn ? 1 : this.inv;
      if (inv !== this.inv) { this.inv = inv; this.v = null; }       // sign settled the other way: restart the smoothing
    }
    if (this.inv < 0) { x = -x; y = -y; z = -z; }
    if (!this.v) { this.v = [x, y, z]; this.p = [x, y, z]; this.dv = [0, 0, 0]; }
    else if (dt > 0) {
      const v = this.v, p = this.p, dv = this.dv, ad = 1 - Math.exp(-2 * Math.PI * o.dHz * dt);
      dv[0] += ((x - p[0]) / dt - dv[0]) * ad; dv[1] += ((y - p[1]) / dt - dv[1]) * ad; dv[2] += ((z - p[2]) / dt - dv[2]) * ad;
      p[0] = x; p[1] = y; p[2] = z;
      const a = 1 - Math.exp(-2 * Math.PI * (o.minHz + o.beta * Math.hypot(dv[0], dv[1], dv[2])) * dt);
      v[0] += (x - v[0]) * a; v[1] += (y - v[1]) * a; v[2] += (z - v[2]) * a;
    }
    this.roll = screenRoll(this.v[0], this.v[1], this.v[2], angle);
    return this.roll;
  }
}
