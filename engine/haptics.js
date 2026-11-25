// engine/haptics.js — phone vibration cues (navigator.vibrate: Android Chrome / Brave / Samsung; iOS has none -> silent no-op).
// The web API only sets on/off durations (no strength, no Android haptic primitives), so crispness = SHORT pulses: on phones
// with a good linear motor a 7–12 ms pulse feels like a crisp tick; budget motors barely spin up in that time, so STRONG
// doubles every duration (they feel it too). Levels: 'off' | 'light' | 'strong' (HUD ui.haptics).
// Only events the player should notice: controls (tick / toggle / gear engaged), engine catch / failed crank / stall, hard
// hits + landings (renderer trauma kicks), crash stall, axle / rope snap, bridge planks breaking, finish stars.
// A higher-priority pattern replaces a running one; a lower one never cuts it short; per-kind cooldowns stop buzzing.

// Steering (r15): s = strong pattern (else light ×2); k = load 0..1 (stopped / dry-steer / F-LOCK = firmer, rolling = lighter).
const KINDS = {   // pattern ms (light) · priority · cooldown s; thump / crash scale with k 0..1
  tick:     { p: [7], pri: 0, cool: 0.03 },
  detent:   { p: [5], pri: 0, cool: 0.25 },
  steer:      { p: k => [Math.round(4 + k)], s: k => [Math.round(7 + 2 * k)], pri: 0, cool: 0.055 },   // WHEEL rim micro-tick per ~22°
  steerCenter: { p: k => [Math.round(5 + k)], s: k => [Math.round(8 + 2 * k)], pri: 1, cool: 0.15, tactile: true },   // 0° notch
  steerLock:  { p: [10], s: [16], pri: 1, cool: 0.25, tactile: true },   // bump stop at full lock
  steerResist: { p: k => [Math.round(4 + k)], s: k => [Math.round(7 + 2 * k)], pri: 0, cool: 0.22 },   // TILT dry-steer / F-LOCK
  toggle:   { p: [10], pri: 1, cool: 0.05 },
  commit:   { p: [16], pri: 2, cool: 0.12 },
  catch:    { p: [14], pri: 2, cool: 0.3 },
  cough:    { p: [9, 45, 9, 45, 9], pri: 2, cool: 0.3 },
  star:     { p: [14], pri: 2, cool: 0.1 },
  thump:    { p: k => [Math.round(9 + 26 * k)], pri: 3, cool: 0.22 },
  crash:    { p: k => [Math.round(22 + 18 * k)], pri: 4, cool: 0.3 },
  rope:     { p: [24, 40, 12], pri: 4, cool: 0.5 },
  bridge:   { p: [14, 45, 14, 45, 26], pri: 4, cool: 0.6 },
  stall:    { p: [28, 70, 16, 90, 9], pri: 4, cool: 0.8 },
  snap:     { p: [20, 35, 20], pri: 5, cool: 0.5 },
  crashStall: { p: [32, 55, 40], pri: 5, cool: 0.8 },
};
export const HAPTIC_LEVELS = ['off', 'light', 'strong'];
export const hapticsSupported = () => typeof navigator !== 'undefined' && typeof navigator.vibrate === 'function';

/** Native wrapper hook (e.g. a WebView / TWA app): window.OffroadNativeHaptics = { play(kind, { k, level, pattern }) } or a
 *  function(kind, opts). When present it gets every cue (after cooldown / priority) and the browser paths stay silent. */
const nativeHook = () => {
  const h = typeof window !== 'undefined' && window.OffroadNativeHaptics;
  return typeof h === 'function' ? h : h && typeof h.play === 'function' ? (kind, o) => h.play(kind, o) : null;
};

/** level 'off' | 'light' | 'strong'; opts.tactile(kind, k, level) = Web Audio click fallback for `tactile` kinds (steering
 *  centre notch / full lock): Android Brave drops navigator.vibrate() while the ringer is on Silent, a quiet click still reads. */
export function createHaptics(level = 'light', opts = {}) {
  const ok = hapticsSupported(), last = {};
  let busyUntil = 0, busyPri = -1;
  const now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now()) / 1000;
  return {
    get supported() { return ok || !!nativeHook(); },
    get level() { return level; },
    set level(v) { level = HAPTIC_LEVELS.includes(v) ? v : 'light'; if (level === 'off' && ok && !nativeHook()) try { navigator.vibrate(0); } catch (_) { /* */ } },
    /** Play a cue: kind (KINDS) + strength / load k 0..1. Returns the pattern sent (or null). */
    play(kind, k = 1) {
      const K = KINDS[kind], nat = nativeHook();
      if (!K || level === 'off' || !(ok || nat || (K.tactile && opts.tactile))) return null;
      const t = now();
      if (t < (last[kind] || 0) + K.cool) return null;
      if (t < busyUntil && K.pri < busyPri) return null;
      k = Math.max(0, Math.min(1, k));
      const pat = P => (typeof P === 'function' ? P(k) : P.slice());
      let p = level === 'strong' && K.s ? pat(K.s) : pat(K.p);
      if (level === 'strong' && !K.s) p = p.map((d, i) => (i % 2 ? d : Math.round(d * 2)));
      last[kind] = t; busyPri = K.pri; busyUntil = t + p.reduce((a, b) => a + b, 0) / 1000;
      if (nat) { try { nat(kind, { k, level, pattern: p }); } catch (_) { return null; } return p; }
      if (K.tactile && opts.tactile) try { opts.tactile(kind, k, level); } catch (_) { /* audio optional */ }
      if (ok) try { navigator.vibrate(p); } catch (_) { return null; }
      return p;
    },
  };
}
