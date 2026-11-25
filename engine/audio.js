// engine/audio.js — Web Audio sound: recorded .mp3 samples from ./sfx/ (fetched + decoded on unlock()) layered over a
// procedural synth that is ALSO the fallback for any sample that is missing / fails to decode. Standalone: imports
// nothing and consumes ONLY the plain data returned by Vehicle.getState(). No physics / canvas / HUD code.
//
//   engine : per-truck recordings (ENGINE_PROFILES: clean idle + rev takes, split files, looped in place with loopStart /
//            loopEnd, zero-crossing snapped + baked pre-roll crossfade) on an engine sub-bus: idle <-> rev equal-power blend by
//            smoothed load (throttle + rpm), rev pitch-matched to idle (no beating), rate + exp lowpass sweep by rpm / throttle,
//            flare one-shot only on a throttle stab; else the shared engine_idle loop; synth (sawtooth + triangle -> lowpass
//            110..360 Hz + sine sub) underneath / alone as the fallback. Silent while OFF; 150 ms equal-power crank -> idle.
//   starter: crank file's starter section looped while cranking, its catch / fire-up tail when the engine starts (CRANKS);
//            clicking_eng_cooldown ticks after a warm / hot shutdown or stall (level + length by heat), and as heat-soak
//            clinks while running hot at low rev / light gas (faded out once revved). Synth fallbacks.
//   tyres  : per-surface loops (stones / gravel / squelch / wood / shallow + deep wading) whose rate, filter, gain follow
//            wheel tread speed + slip; synth pink/brown slip scrub stays underneath (full level as fallback)
//   impacts: synth tyreThud (suspension spike; impact_thud.mp3 unused: sub + click = tinny), metal_thud_1/2 (chassis bottom-out), heavy_boulder_thud (boulder / hard
//            rock strike), water splash 1/2 (wheel / chassis entering water), metal_scrape loop (chassis sliding)
//   winch  : synth ratchet click per CLICK_M reeled + stick-slip cable creak (always synth); winch_snap on break
//   failure: engine_misfire on overheat / stall / misfire coughs, axle_break on axle snap, metal crunch when
//            hardcore.body jumps; synth starter whirr / cough / pop / snap when the samples are missing
//   shifts : gear_shift_1 = lever notch (HUD lever steps, 4WD / F-LOCK / R-LOCK / REV toggles); gear_shift_fail = grind when a
//            range is requested while moving > 0.3 m/s (pendingRange) or re-requested while pending; gear_shift_2 = standard
//            range commit (into H / up toward L+); gear_shift_3 = heavy commit (H -> low, into L / L-, or a pending shift
//            completing at <= 0.3 m/s). Synth notch / grind / double clunk when the samples are missing.
//   chassis: susp_squeak on high compression / articulation while the suspension works (sooner + more with worn shocks),
//            metallic_groan on heavy bottom-outs / chassis slams, metal_spring on fast worn-shock rebound. All cooled down.
// Water = any surface named /water|river|ford|stream|wade|puddle/ (deep: /deep/ in the name, or the chassis touching it).

const ENGINE_ORDER = 3;       // engine note = crank Hz * order  (800 rpm -> 40 Hz, 5000 rpm -> 250 Hz)
const LP_MIN = 110, LP_MAX = 360;
const SLIP_DEADBAND = 0.3;    // m/s of wheel slip before any scrub is heard
const SLIP_FULL = 4.5;        // m/s of slip (above the deadband) for full level / top of the frequency sweep
const HIT_RATE = 3.0;         // suspension compression rate (fraction of travel per second) that counts as a hit
const HIT_SPAN = 9.0;         // rate above HIT_RATE that maps to a full-strength thud
// impact gating: weaker hits are felt (suspension) but not heard, so rock gardens / ripples don't machine-gun thuds
const HIT_MIN = 0.14;         // min wheel-hit strength 0..1 (≈ rate 4.3 travel/s or tyre closing speed 1.2 m/s)
const BODY_VN = 1.4, BODY_F = 9000;  // chassis bottom-out: min closing speed (m/s) or contact force (N)
const ROCK_VN = 1.6;          // loose boulder strike: min closing speed (m/s)
const HIT_COOL = { thud: 0.15, metal: 0.18, boulder: 0.2 };   // s between one-shots of one impact type
const BUSY_TAU = 0.6;         // s: hits inside this window pile up -> each further hit a little quieter + duller
const hitGain = (k, lo, hi) => lo + (hi - lo) * Math.pow(k, 1.5);   // perceptual: soft hits stay soft
const CLICK_M = 0.05;       // metres of rope per ratchet click
const SHIFT_RATE = { H: 1.08, 'L+': 1.0, L: 0.93, 'L-': 0.86 };   // gear_shift_2 playbackRate by the range engaged
const SHIFT_COOL = 0.08;      // s between lever notches (rapid HUD taps stay crisp, not a buzz)
const STRESS_COOL = { grind: 0.5, squeak: 0.9, groan: 1.2, spring: 1.8, rev: 3.0, tick: 2.0 };   // s between stress one-shots
const SQUEAK_AT = [0.7, 1.45, 2.2, 3.45, 4.2, 4.95];   // s: squeak onsets in susp_squeak.mp3 (one ~0.45 s slice per event)
const BROWN_MAKEUP = 3.0;     // brown noise carries little energy in the slip band; lift it to match pink
const CREAK_GAIN = 2.0;       // the narrow creak bandpass passes only a sliver of the sawtooth

// Tyre-slip voicing: bandpass centre sweeps lo -> hi (Hz) with slip; pink = bright scrub, brown = low wet hiss.
const SLIP = {
  rock:    { lo: 1300, hi: 2700, q: 1.7, pink: 1.0, brown: 0.0, gain: 1.10 },
  dirt:    { lo: 420,  hi: 950,  q: 0.8, pink: 0.4, brown: 0.8, gain: 0.50 },
  mud:     { lo: 240,  hi: 620,  q: 0.6, pink: 0.2, brown: 1.0, gain: 0.50 },
  wood:    { lo: 800,  hi: 1600, q: 2.2, pink: 0.8, brown: 0.2, gain: 0.80 },
  wetwood: { lo: 700,  hi: 1400, q: 2.6, pink: 0.8, brown: 0.3, gain: 0.80 },
  ice:     { lo: 2200, hi: 3800, q: 3.0, pink: 1.0, brown: 0.0, gain: 0.50 },
  slickmud:{ lo: 200,  hi: 540,  q: 0.6, pink: 0.15, brown: 1.0, gain: 0.55 },
  clay:    { lo: 320,  hi: 760,  q: 0.9, pink: 0.3, brown: 0.9, gain: 0.50 },
  slickclay:{ lo: 260, hi: 640,  q: 0.7, pink: 0.2, brown: 1.0, gain: 0.55 },
  scree:   { lo: 900,  hi: 2000, q: 1.1, pink: 0.9, brown: 0.3, gain: 0.85 },
  berm:    { lo: 360,  hi: 820,  q: 0.7, pink: 0.35, brown: 0.9, gain: 0.55 },   // soft dirt trail-edge berms
};
const SLIP_KEYS = ['lo', 'hi', 'q', 'pink', 'brown', 'gain'];
SLIP.water = { lo: 500, hi: 1300, q: 0.5, pink: 0.6, brown: 0.9, gain: 0.45 };

// ---- recorded samples ---------------------------------------------------------------------------
const SFX = {
  crank: 'engine_crank.mp3', idle: 'engine_idle.mp3', misfire: 'engine_misfire.mp3', axle: 'axle_break.mp3',
  winchSnap: 'winch_snap.mp3', stones: 'tyre_stones_roll.mp3', gravel: 'tyre_gravel_roll.mp3', mud: 'wet_squelch.mp3',
  wood: 'tyre_wood.mp3', shallow: 'river_shallow_wading.mp3', deep: 'river_deep_wading.mp3',
  splashBig: 'water-splash_1.mp3', splashSmall: 'water_splash_2.mp3',
  gear1: 'gear_shift_1.mp3', gear2: 'gear_shift_2.mp3', gear3: 'gear_shift_3.mp3', gearFail: 'gear_shift_fail.mp3',
  cooldown: 'clicking_eng_cooldown.mp3', squeak: 'susp_squeak.mp3', groan: 'metallic_groan.mp3', spring: 'metal_spring.mp3',
  boulder: 'heavy_boulder_thud.mp3', metal1: 'metal_thud_1.mp3', metal2: 'metal_thud_2.mp3', scrape: 'metal_scrape.mp3',
};
// continuous loops: filter type, cutoff f0 -> f1 (Hz), q, gain, `full` tread m/s for top level / rate, rate r0 + rk * k
const LOOP = {
  idle:    { type: 'lowpass',  f0: 700, f1: 7000, q: 0.7, gain: 0.55 },
  rev:     { type: 'lowpass',  f0: 700, f1: 9000, q: 0.7, gain: 0.6 },      // profile rev / load loop (engine voice 2)
  stones:  { type: 'lowpass',  f0: 900, f1: 9000, q: 0.7, gain: 0.75, full: 10, r0: 0.8,  rk: 0.45 },
  gravel:  { type: 'lowpass',  f0: 700, f1: 7500, q: 0.7, gain: 0.70, full: 10, r0: 0.8,  rk: 0.45 },
  mud:     { type: 'bandpass', f0: 280, f1: 1700, q: 0.8, gain: 1.00, full: 6,  r0: 0.75, rk: 0.5 },
  wood:    { type: 'lowpass',  f0: 800, f1: 8000, q: 0.7, gain: 0.70, full: 8,  r0: 0.85, rk: 0.35 },
  shallow: { type: 'lowpass',  f0: 900, f1: 6500, q: 0.6, gain: 0.85, full: 6,  r0: 0.85, rk: 0.35 },
  deep:    { type: 'lowpass',  f0: 400, f1: 3200, q: 0.6, gain: 0.95, full: 5,  r0: 0.85, rk: 0.3 },
  scrape:  { type: 'bandpass', f0: 900, f1: 4200, q: 0.6, gain: 1.10, full: 4,  r0: 0.85, rk: 0.35 },
};
const SURF_CATS = ['stones', 'gravel', 'mud', 'wood'];
const ENG_LOOPS = ['idle', 'rev'];   // engine recordings: routed through the engine sample sub-bus (g.engSmp)
const CLINK_HEAT = 0.5, CLINK_RN = 0.22;   // running heat-soak clinks: min heat 0..1, max normalised rpm (low rev only)
const CATCH_XF = 0.15;        // s: equal-power crank -> idle crossfade as the engine catches (also any engine-on edge)
const RPM_TAU = 0.08, THR_UP = 0.05, THR_DN = 0.12, LOAD_UP = 0.07, LOAD_DN = 0.18;   // s: JS-side smoothing (no warble)
// held gas: physics rpm swings ~1 Hz (wheel hop spins up, tyre catch drags down: 2400 <-> 5800 on the V8s); heard raw that is
// a rhythmic wah. Audible rpm falls slowly while gas is held (RPM_FALL_GAS) and leans on a decaying peak (PEAK_TAU, PEAK_HOLD).
const RPM_FALL_GAS = 0.55, PEAK_TAU = 1.4, PEAK_HOLD = 0.7;
const EP_IN = new Float32Array(32).map((_, i) => Math.sin(i / 31 * Math.PI / 2)), EP_OUT = EP_IN.slice().reverse();
/** Soft ceiling: linear up to 75 % of m, then eases into m (no flat plateau / kink at the rate cap). */
const softCap = (x, m) => { const k = 0.75 * m; return x <= k ? x : k + (m - k) * (1 - Math.exp(-(x - k) / (m - k))); };
/** Exponential (perceptual) sweep f0 -> f1 by k 0..1: equal steps of k = equal musical steps of cutoff. */
const expLerp = (f0, f1, k) => f0 * Math.pow(f1 / f0, k < 0 ? 0 : k > 1 ? 1 : k);
// Per-truck engine voicing over the SHARED engine_* samples (+ synth). rate = idle playbackRate, rpmExp / maxRate = pitch
// rise to redline, lp = idle-loop cutoff x, q, gain = sample level x, synth = synth level x under the sample, order =
// synth firing order (Hz = rpm/60 x order), synthLp / sub = synth lowpass x / sub-octave level, crank / misfire = rates.
// A vehicle JSON may add `sound {}` with any of these (+ `files {idle, crank, misfire}` relative to sfx/); otherwise
// sfx/<vehicleId>/engine_idle.mp3 etc. are used when present (then rate 1: that truck's own recording).
const ENGINE_SFX = ['idle', 'crank', 'misfire'];
const VOICE_DEFAULT = { rate: 1, rpmExp: 0.55, maxRate: 2.8, lp: 1, q: 0.7, gain: 1, synth: 1, order: 3, synthLp: 1, sub: 1, crank: 1, misfire: 1 };
export const VOICES = {
  crawler01: { rate: 1.0,  rpmExp: 0.55, maxRate: 2.8, lp: 1.0,  q: 1.2, gain: 1.05, synth: 1.0, order: 3, synthLp: 1.0,  sub: 1.0, crank: 1.0,  misfire: 1.0 },   // punchy petrol six
  crawler02: { rate: 0.72, rpmExp: 0.5,  maxRate: 1.9, lp: 0.55, q: 1.6, gain: 1.2,  synth: 1.6, order: 3, synthLp: 0.75, sub: 1.8, crank: 0.8,  misfire: 0.78 },  // heavy diesel bogger
  crawler03: { rate: 1.22, rpmExp: 0.62, maxRate: 3.4, lp: 1.6,  q: 0.8, gain: 0.9,  synth: 0.7, order: 4, synthLp: 1.4,  sub: 0.5, crank: 1.2,  misfire: 1.25 },  // light high-rev V8 buggy
};
/** Voice for a truck: built-in preset by id, else picked from its engine / mass; vehicle JSON `sound {}` wins. */
export function voiceFor(id, cfg = {}) {
  const e = cfg.engine || {}, mass = (cfg.chassis && cfg.chassis.mass) || 1400;
  const base = VOICES[id] || ((e.redlineRpm ?? 5000) >= 6000 ? VOICES.crawler03
    : (mass >= 2000 || (e.redlineRpm ?? 5000) <= 4500) ? VOICES.crawler02 : VOICES.crawler01);
  return { ...VOICE_DEFAULT, ...base, ...(cfg.sound || {}) };
}
// Recorded engine profiles (split takes in sfx/): each section = [file, a, b] s, looped in place (loopStart / loopEnd).
//   idle  = steady idle loop (rate 1 at idleRpm)   rev = steady on-throttle loop; revPitch = its fundamental / the idle take's
//           (measured), so rev rate = idle rate / revPitch and the two never beat while blended
//   flare = free-rev blip, a one-shot only on a fast throttle stab   crank = starter file (CRANKS: loop + catch tail)
//   voice = VOICE keys applied while the recording plays (the shared-sample pitch / filter presets don't fit a real take).
// A vehicle picks one by id (VEHICLE_PROFILE) or with `sound { profile: 'name' | {inline} | null }` in its JSON (legacy
// inline `{file, idle: [a, b], drive?, driveRef?, rev?}` still works).
export const CRANKS = {
  'engine_crank.mp3': { loop: [0.08, 1.7], catch: [1.72, 3.26] },
  'v8_plymouth_barcuda_crank.mp3': { loop: [0.1, 1.68], catch: [1.7, 2.35] },
};
export const ENGINE_PROFILES = {
  jeep:      { idle: ['thar_jeep_driving.mp3', 16.4, 19.8], rev: ['thar_jeep_driving.mp3', 3.0, 6.2], revPitch: 1.4,
               crank: 'engine_crank.mp3', voice: { rate: 1, rpmExp: 0.5, maxRate: 2.2, lp: 1.1, q: 0.8, gain: 1.15, synth: 0.6 } },  // utility jeep
  charger:   { idle: ['v8_dodge_chrgr_idle.mp3', 5.6, 9.4], rev: ['v8_dodge_chrgr_rev.mp3', 0.15, 1.45], revPitch: 2.15,  // 48 / 103 Hz
               flare: ['v8_dodge_chrgr_rev.mp3', 1.5, 2.45], crank: 'v8_plymouth_barcuda_crank.mp3',
               voice: { rate: 1, rpmExp: 0.6, maxRate: 2.6, lp: 0.9, q: 1.0, gain: 0.95, synth: 0.8 } },           // heavy V8 bogger
  barracuda: { idle: ['v8_plymth_brcuda_idle.mp3', 0.5, 7.5], rev: ['v8_plymth_brcuda_rev.mp3', 0.1, 2.4], revPitch: 1.0,  // 74 / 73 Hz
               flare: ['v8_plymth_brcuda_rev.mp3', 2.5, 4.1], crank: 'v8_plymouth_barcuda_crank.mp3',
               voice: { rate: 1, rpmExp: 0.55, maxRate: 2.3, lp: 1.0, q: 0.9, gain: 0.95, synth: 0.7 } },
};
/** Legacy single-file inline profile {file, idle: [a, b], drive?, driveRef?, rev?} -> split-section form. */
function normProfile(p) {
  if (!p || !p.file || Array.isArray(p.idle) && typeof p.idle[0] === 'string') return p;
  const f = p.file, sec = s => (s ? [f, s[0], s[1]] : null);
  return { ...p, idle: sec(p.idle), rev: sec(p.drive), revPitch: p.revPitch ?? 1 + (p.driveRef ?? 0.35) * 2, flare: sec(p.rev) };
}
// crawler03 (light V8 buggy) keeps the shared engine_idle voice; barracuda is the ready profile for a heavy V8 rig (crawler04).
export const VEHICLE_PROFILE = { crawler01: 'jeep', crawler02: 'charger', crawler04: 'barracuda' };
VOICES.crawler04 = { rate: 0.85, rpmExp: 0.5, maxRate: 2.2, lp: 0.8, q: 1.3, gain: 1.1, synth: 1.2, order: 4, synthLp: 0.85, sub: 1.5, crank: 0.9, misfire: 0.85 };
/** Engine profile for a truck: sound.profile (name / inline object / null = none) wins over the per-id default. */
export function profileFor(id, cfg = {}) {
  const p = cfg.sound && 'profile' in cfg.sound ? cfg.sound.profile : VEHICLE_PROFILE[id];
  return typeof p === 'string' ? ENGINE_PROFILES[p] || null : p && typeof p === 'object' && (p.file || p.idle) ? normProfile(p) : null;
}

/** Zero-crossing (rising) nearest sample i within +-r samples of the mono mix. */
function zeroCross(chs, i, r) {
  const at = k => { let v = 0; for (const c of chs) v += c[k]; return v; };
  for (let o = 0; o <= r; o++) for (const k of [i + o, i - o]) if (k > 0 && k < chs[0].length && at(k - 1) < 0 && at(k) >= 0) return k;
  return i;
}
/**
 * Loop slice [a, b] s of buf for loopStart / loopEnd playback: ends snapped to rising zero crossings, copied with xfadeS of
 * pre-roll, whose audio is crossfaded (equal power) into the slice tail, so the wrap loopEnd -> loopStart continues the
 * recording seamlessly (no rough tail cut / click). Returns { b, loopStart, loopEnd } (s, in the new buffer) or null.
 */
export function sliceLoop(ctx, buf, a, b, xfadeS = 0.08) {
  const sr = buf.sampleRate, nch = buf.numberOfChannels, chs = [];
  for (let c = 0; c < nch; c++) chs.push(buf.getChannelData(c));
  const r = Math.round(sr * 0.01);
  let s0 = zeroCross(chs, Math.round(a * sr), r), s1 = zeroCross(chs, Math.min(buf.length - 1, Math.round(b * sr)), r);
  if (!(s1 - s0 > sr * 0.2)) return null;
  const X = Math.min(Math.round(sr * xfadeS), s0, Math.floor((s1 - s0) * 0.25)), L = s1 - s0 + X;
  const out = ctx.createBuffer(nch, L, sr);
  for (let c = 0; c < nch; c++) {
    const d = chs[c], o = out.getChannelData(c);
    for (let i = 0; i < L; i++) o[i] = d[s0 - X + i];
    for (let i = 0; i < X; i++) {                 // tail -> pre-roll: at the wrap the next sample is d[s0], right after d[s0 - 1]
      const k = (i + 0.5) / X * Math.PI / 2, j = L - X + i;
      o[j] = o[j] * Math.cos(k) + o[i] * Math.sin(k);
    }
  }
  return { b: out, loopStart: X / sr, loopEnd: L / sr };
}

const SLIP_OF_CAT = { stones: 'rock', gravel: 'dirt', mud: 'mud', wood: 'wood', water: 'water' };
const SYNTH_UNDER = 0.35;     // synth level kept under a playing sample (engine body / tyre scrub)
const MAX_SHOTS = 14;         // concurrent one-shot samples (extra ones are dropped)
const STONE_RE = /rock|stone|scree|boulder|granite|slab/;

/** Sound category of a surface name: water | mud | wood | stones | gravel (dirt, berms, unknown). */
const catCache = {};
export function surfaceCat(name) {
  if (!name) return null;
  let c = catCache[name];
  if (c) return c;
  c = /water|river|ford|stream|wade|puddle/.test(name) ? 'water'
    : /mud|bog|clay|swamp|marsh/.test(name) ? 'mud'
    : /wood|log|bridge|plank|timber/.test(name) ? 'wood'
    : STONE_RE.test(name) ? 'stones' : 'gravel';
  return (catCache[name] = c);
}

/**
 * Seamless loop: trim the quiet lead-in / fade-out tail (20 ms RMS below `thr` x median), then bake an equal-power
 * crossfade of the tail into the head so the buffer loops with no dip or click. Returns a new AudioBuffer.
 */
export function seamlessLoop(ctx, buf, thr = 0.5, xfadeS = 0.3) {
  const sr = buf.sampleRate, nch = buf.numberOfChannels, n = buf.length, W = Math.max(1, Math.round(sr * 0.02));
  const ch = []; for (let c = 0; c < nch; c++) ch.push(buf.getChannelData(c));
  const nw = Math.floor(n / W);
  if (nw < 10) return buf;
  const rms = new Float32Array(nw);
  for (let w = 0; w < nw; w++) {
    let e = 0;
    for (let c = 0; c < nch; c++) { const d = ch[c]; for (let i = w * W, j = i + W; i < j; i++) e += d[i] * d[i]; }
    rms[w] = Math.sqrt(e / (W * nch));
  }
  const sorted = Array.from(rms).sort((a, b) => a - b), lim = sorted[nw >> 1] * thr;
  let w0 = 0, w1 = nw - 1;
  while (w0 < nw - 1 && rms[w0] < lim) w0++;
  while (w1 > w0 && rms[w1] < lim) w1--;
  const s0 = w0 * W, len = (w1 + 1) * W - s0;
  const X = Math.min(Math.round(sr * xfadeS), Math.floor(len * 0.2));
  if (len < sr * 0.2 || X < 16) return buf;
  const out = ctx.createBuffer(nch, len - X, sr);
  for (let c = 0; c < nch; c++) {
    const d = ch[c], o = out.getChannelData(c);
    for (let i = 0; i < len - X; i++) o[i] = d[s0 + i];
    for (let i = 0; i < X; i++) {                    // head fades in while the tail (which precedes the head) fades out
      const a = (i + 0.5) / X;
      o[i] = d[s0 + i] * Math.sin(a * Math.PI / 2) + d[s0 + len - X + i] * Math.cos(a * Math.PI / 2);
    }
  }
  return out;
}

const clamp01 = v => (v < 0 ? 0 : v > 1 ? 1 : v);
const lerp = (a, b, t) => a + (b - a) * t;

/** 2 s loopable mono noise, peak-normalised. kind: 'pink' (Kellet filter) | 'brown' (leaky integrator). */
function noiseBuffer(ctx, kind) {
  const n = Math.floor(ctx.sampleRate * 2), buf = ctx.createBuffer(1, n, ctx.sampleRate), d = buf.getChannelData(0);
  let b0 = 0, b1 = 0, b2 = 0, last = 0;
  for (let i = 0; i < n; i++) {
    const w = Math.random() * 2 - 1;
    if (kind === 'pink') {
      b0 = 0.99765 * b0 + w * 0.0990460; b1 = 0.96300 * b1 + w * 0.2965164; b2 = 0.57000 * b2 + w * 1.0526913;
      d[i] = b0 + b1 + b2 + w * 0.1848;
    } else {
      last = (last + 0.02 * w) / 1.02; d[i] = last;
    }
  }
  const drift = d[n - 1] - d[0];                 // remove the end-to-start step so the loop point is click-free
  let peak = 1e-9;
  for (let i = 0; i < n; i++) { d[i] -= drift * i / (n - 1); peak = Math.max(peak, Math.abs(d[i])); }
  for (let i = 0; i < n; i++) d[i] /= peak;
  return buf;
}

/**
 * @param opts { volume = 0.8, muted = false, context?, sfxBase?, samples = true }   context = inject an (Offline)
 *   AudioContext for tests; sfxBase = URL of the sample folder (default ../sfx/ next to engine/); samples false = synth only
 * @returns { unlock(), update(vehicleState, dt, engineCfg), shift(kind, info?), reset(), setMuted(bool), muted, ctx, samples, destroy() }
 *   setVehicle(id, vehicleCfg)  per-truck engine voice (VOICES preset / auto / cfg.sound {}) + sfx/<id>/ overrides
 *   samples   { loaded: [names], failed: [names], total }  (loading starts on the first unlock()); voices = debug gains
 *   unlock()  call from a user gesture (tap / key): creates + resumes the AudioContext. Safe to call repeatedly.
 *   update()  once per rendered frame with Vehicle.getState(); engineCfg = { idleRpm, redlineRpm }.
 *   reset()   call after teleports (restart / recover / respawn) so the jump is not heard as an impact.
 */
export function createAudio(opts = {}) {
  const volume = opts.volume ?? 0.8, injected = !!opts.context;
  let ctx = opts.context || null, g = null, muted = !!opts.muted, stale = true;
  const st = { wheels: [], bodyHit: false, bodyCool: 0, len: null, pay: 0, broken: false, creakT: 0, creakP: 1, creakA: 1,
              stalled: false, misfire: false, axle: [false, false], body: null };
  const mix = { lo: 0, hi: 0, q: 0, pink: 0, brown: 0, gain: 0 };
  // samples: buf[name] = decoded AudioBuffer (loops already seamless), vox[name] = running loop voice {src, f, g}
  const buf = {}, vox = {}, smp = { loaded: [], failed: [], total: Object.keys(SFX).length };
  const cool = { metal: 0, boulder: 0, splash: 0, thud: 0, misfire: 0, notch: 0, grind: 0, squeak: 0, groan: 0, spring: 0, rev: 0, tick: 0, clink: 0 };
  let gear = null;                                         // last {fourWD, lockF, lockR, rev, range, pend}; null = prime silently
  const hitFx = { busy: 0, metal: '' };                    // recent-hit density, last metal clip (alternate takes)
  const acc = {};                                          // per-category {n, drive, slip} scratch
  for (const c of [...SURF_CATS, 'water']) acc[c] = { n: 0, drive: 0, slip: 0 };
  let loading = false, shots = 0, crankSrc = null;
  let vid = null, voice = { ...VOICE_DEFAULT }, own = {};          // current truck id, voice, explicit sound {} keys
  const ov = {};                                                  // ov[vehicleId][idle|crank|misfire] = override buffer
  let prof = null;                                                // current truck's ENGINE_PROFILES entry (or null)
  const files = {}, slices = {};                                  // profile files: decoded buffer | 'loading' | 'failed'; loop slices
  const sx = { wet: [], cranking: false, hot: false, scrape: 0, on: false, warm: 0, thrLow: 0, susp: [], cooldownSrc: null };
  const sfxBase = opts.sfxBase ?? (() => { try { return new URL('../sfx/', import.meta.url).href; } catch (_) { return './sfx/'; } })();

  const onVisibility = () => {
    if (!ctx || injected || ctx.state === 'closed') return;
    (document.hidden ? ctx.suspend() : ctx.resume()).catch(() => {});
  };

  function build() {
    const gain = (v, to) => { const n = ctx.createGain(); n.gain.value = v; if (to) n.connect(to); return n; };
    const filter = (type, f, q, to) => {
      const n = ctx.createBiquadFilter(); n.type = type; n.frequency.value = f; n.Q.value = q; if (to) n.connect(to); return n;
    };
    const osc = (type, f, cents, to) => {
      const n = ctx.createOscillator(); n.type = type; n.frequency.value = f; n.detune.value = cents; n.connect(to); n.start(); return n;
    };
    const loop = (buffer, to) => {
      const n = ctx.createBufferSource(); n.buffer = buffer; n.loop = true; n.connect(to); n.start(); return n;
    };

    // bus -> limiter (safety only: voices are mixed to stay under it, so impacts keep their punch) -> master -> speakers
    const master = gain(muted ? 0 : volume, ctx.destination);
    const limiter = ctx.createDynamicsCompressor();
    limiter.threshold.value = -4; limiter.knee.value = 4; limiter.ratio.value = 12;
    limiter.attack.value = 0.002; limiter.release.value = 0.15;
    limiter.connect(master);
    const bus = gain(1, limiter);
    const pink = noiseBuffer(ctx, 'pink'), brown = noiseBuffer(ctx, 'brown');

    // engine: both oscillators go through the lowpass (never straight to the bus); every voice starts silent
    const f0 = 800 / 60 * ENGINE_ORDER;
    const engGain = gain(0, bus);
    const engLP = filter('lowpass', LP_MIN, 1.3, engGain);
    const oscA = osc('sawtooth', f0, -14, gain(0.5, engLP));
    const oscB = osc('triangle', f0, 17, gain(0.65, engLP));
    const subGain = gain(0.28, engGain), sub = osc('sine', f0 / 2, 0, subGain);
    const engSmp = gain(0, bus);                                   // recorded idle / rev loops: on/off + crank -> idle fade
    sx.engOn = false;                                              // fresh gate is closed: next running frame fades it in

    // tyre slip
    const slipGain = gain(0, bus);
    const slipBP = filter('bandpass', 600, 1, slipGain);
    const pinkMix = gain(0, slipBP), brownMix = gain(0, slipBP);
    const noiseSrc = [loop(pink, pinkMix), loop(brown, brownMix)];

    // winch cable creak: sawtooth "stick-slip" source ringing a narrow bandpass
    const creakGain = gain(0, bus);
    const creakBP = filter('bandpass', 700, 10, creakGain);
    const creakOsc = osc('sawtooth', 60, 0, creakBP);

    // starter motor: buzzy sawtooth chopped by an 11 Hz square LFO ("rur-rur-rur") -> lowpass; silent unless cranking
    const crankGain = gain(0, bus);
    const crankAM = gain(0.5, filter('lowpass', 900, 0.9, crankGain));
    const crankOsc = osc('sawtooth', 95, 0, crankAM);
    const crankLfo = osc('square', 11, 0, gain(0.5, crankAM.gain));

    g = { master, bus, pink, brown, engGain, engSmp, engLP, oscA, oscB, sub, subGain, slipGain, slipBP, pinkMix, brownMix, creakGain, creakBP, creakOsc,
          crankGain, crankOsc, sources: [oscA, oscB, sub, creakOsc, crankOsc, crankLfo, ...noiseSrc] };
  }

  // ---- samples --------------------------------------------------------------------------------
  function decode(ab) {
    return new Promise((res, rej) => {
      try { const p = ctx.decodeAudioData(ab, res, rej); if (p && p.then) p.then(res, rej); } catch (e) { rej(e); }
    });
  }
  function loadSamples() {
    if (loading || opts.samples === false || typeof fetch !== 'function') return;
    loading = true;
    for (const name in SFX) {
      fetch(sfxBase + SFX[name])
        .then(r => (r.ok ? r.arrayBuffer() : Promise.reject(new Error(r.status))))
        .then(decode)
        .then(b => {
          if (!g) return;
          buf[name] = LOOP[name] ? seamlessLoop(ctx, b, name === 'stones' ? 0.6 : 0.5) : b;
          if (LOOP[name]) startLoop(name);
          smp.loaded.push(name);
        })
        .catch(() => { smp.failed.push(name); });
    }
  }
  /** Per-truck engine sample overrides: sound.files {name: file} or sfx/<id>/engine_*.mp3 (missing = shared sample). */
  function loadOverrides(id) {
    if (!id || ov[id] || opts.samples === false || typeof fetch !== 'function') return;
    const o = ov[id] = {}, files = voice.files || {};
    for (const name of ENGINE_SFX) {
      const url = sfxBase + (files[name] || `${id}/${SFX[name]}`);
      fetch(url).then(r => (r.ok ? r.arrayBuffer() : Promise.reject(new Error(r.status)))).then(decode)
        .then(b => { if (g) o[name] = name === 'idle' ? seamlessLoop(ctx, b) : b; })
        .catch(() => {});
    }
  }
  /** Profile / crank recording by file name (fetched + decoded once, shared between trucks). */
  function loadFile(f) {
    if (!f || files[f] || opts.samples === false || typeof fetch !== 'function') return;
    files[f] = 'loading';
    fetch(sfxBase + f).then(r => (r.ok ? r.arrayBuffer() : Promise.reject(new Error(r.status)))).then(decode)
      .then(b => { files[f] = g ? b : undefined; if (g) smp.loaded.push(f); })
      .catch(() => { files[f] = 'failed'; smp.failed.push(f); });
  }
  const fileBuf = f => (f && files[f] && typeof files[f] === 'object' ? files[f] : null);
  /** Cached loop slice of a profile section [file, a, b] s (null until decoded). */
  function slice(sec) {
    const b = sec && fileBuf(sec[0]);
    if (!b) return null;
    const k = sec.join('|');
    return k in slices ? slices[k] : (slices[k] = sliceLoop(ctx, b, sec[1], sec[2], 0.22));   // long wrap xfade: no loop 'tick'
  }
  function loadProfile() { if (prof) for (const k of ['idle', 'rev', 'flare']) if (prof[k]) loadFile(prof[k][0]); if (prof) loadFile(prof.crank); }
  /** Profile voicing while its recording plays (sound {} keys still win). */
  const profVoice = () => (prof && prof.idle && fileBuf(prof.idle[0]) ? { ...voice, ...(prof.voice || {}), ...own } : voice);
  /** Engine sample for the current truck: its override if loaded, else the shared one. */
  const engBuf = name => (vid && ov[vid] && ov[vid][name]) || buf[name];
  /** Starter: { b, loop?: slice, catch?: [a, b] } -- override > profile crank file > shared engine_crank. */
  function crankBuf() {
    if (vid && ov[vid] && ov[vid].crank) return { b: ov[vid].crank };
    const f = prof && fileBuf(prof.crank) ? prof.crank : buf.crank ? SFX.crank : null;
    if (!f) return null;
    const b = f === SFX.crank && !fileBuf(f) ? buf.crank : fileBuf(f), c = CRANKS[f];
    if (!c) return { b };
    const k = f + '|crank';
    if (!(k in slices)) slices[k] = sliceLoop(ctx, b, c.loop[0], c.loop[1], 0.05);
    return { b, loop: slices[k], catch: c.catch };
  }
  /** Rate multiplier: the voice's preset, or 1 when playing the truck's own recording (unless sound {} sets it). */
  const engRate = (name, key) => (vid && ov[vid] && ov[vid][name] && !(key in own) ? 1 : voice[key]);
  /** Silent looping voice: src -> filter -> gain -> bus; random start offset so loops don't phase-lock. b replaces it.
   *  sl = { loopStart, loopEnd } plays only that section of b in place (sliceLoop: zero-crossing + crossfaded wrap). */
  function stopLoop(name) {
    const old = vox[name];
    if (!old) return;
    try { old.src.stop(); } catch (_) {}
    old.g.disconnect();
    const i = g.sources.indexOf(old.src); if (i >= 0) g.sources.splice(i, 1);
    delete vox[name];
  }
  function startLoop(name, b = buf[name], sl = null) {
    stopLoop(name);
    const c = LOOP[name], src = ctx.createBufferSource(), f = ctx.createBiquadFilter(), gn = ctx.createGain();
    src.buffer = b; src.loop = true;
    if (sl) { src.loopStart = sl.loopStart; src.loopEnd = sl.loopEnd; }
    f.type = c.type; f.frequency.value = c.f0; f.Q.value = c.q; gn.gain.value = 0;
    src.connect(f); f.connect(gn); gn.connect(ENG_LOOPS.includes(name) ? g.engSmp : g.bus);   // engine takes: sub-bus gate
    src.start(0, sl ? sl.loopStart + Math.random() * (sl.loopEnd - sl.loopStart) : Math.random() * b.duration);
    vox[name] = { src, f, g: gn, b };
    g.sources.push(src);
  }
  /** Drive a loop voice: k = 0..1 level / brightness, extra = 0..1 slip (brighter + a little faster), lvl = gain. */
  function setLoop(name, lvl, k, extra, t) {
    const v = vox[name]; if (!v) return;
    const c = LOOP[name];
    v.g.gain.setTargetAtTime(Math.max(0, lvl), t, lvl > 0 ? 0.06 : 0.12);
    if (c.r0 !== undefined) v.src.playbackRate.setTargetAtTime(c.r0 + c.rk * k + 0.12 * extra, t, 0.08);
    v.f.frequency.setTargetAtTime(lerp(c.f0, c.f1, clamp01(0.75 * k + 0.45 * extra)), t, 0.08);
  }
  /** One-shot sample; returns the source or null (missing / too many playing). dur = play only a faded slice. */
  function shot(name, peak, { rate = 1, offset = 0, dur = 0, loop = false, lp = 0, delay = 0, sl = null, attack = 0 } = {}) {
    const b = typeof name !== 'string' ? name : ENGINE_SFX.includes(name) ? engBuf(name) : buf[name];
    if (!b || shots >= MAX_SHOTS) return null;
    const t = ctx.currentTime + delay, src = ctx.createBufferSource(), gn = ctx.createGain();
    src.buffer = b; src.playbackRate.value = rate; src.loop = loop;
    if (loop && sl) { src.loopStart = sl.loopStart; src.loopEnd = sl.loopEnd; if (!offset) offset = sl.loopStart; }
    gn.gain.value = peak;
    if (attack > 0) { gn.gain.setValueAtTime(0, t); gn.gain.linearRampToValueAtTime(peak, t + attack); }
    if (dur > 0) { gn.gain.setValueAtTime(peak, t + Math.max(0, dur - 0.12)); gn.gain.linearRampToValueAtTime(0.0001, t + dur); }
    let f = null;                                          // lp = lowpass Hz: soft hits sound duller, not just quieter
    if (lp > 0) { f = ctx.createBiquadFilter(); f.type = 'lowpass'; f.frequency.value = lp; f.Q.value = 0.5; src.connect(f); f.connect(gn); }
    else src.connect(gn);
    gn.connect(g.bus);
    shots++;
    src.onended = () => { shots--; gn.disconnect(); if (f) f.disconnect(); };
    src.start(t, offset, dur > 0 ? dur * rate : undefined);
    src._gain = gn;
    return src;
  }
  const jitter = (a = 0.06) => 1 + (Math.random() * 2 - 1) * a;

  // ---- one-shots ------------------------------------------------------------------------------
  const env = (param, t, peak, attack, decay) => {
    param.setValueAtTime(0.0001, t);
    param.linearRampToValueAtTime(peak, t + attack);
    param.exponentialRampToValueAtTime(0.0001, t + attack + decay);
  };
  /** Short slice of a noise buffer -> filters (in parallel) -> envelope -> bus. */
  function noiseBurst(buffer, t, dur, peak, filters) {
    const src = ctx.createBufferSource(), out = ctx.createGain();
    src.buffer = buffer;
    for (const [type, f, q] of filters) {
      const n = ctx.createBiquadFilter(); n.type = type; n.frequency.value = f; n.Q.value = q;
      src.connect(n); n.connect(out);
    }
    env(out.gain, t, peak, 0.002, dur);
    out.connect(g.bus);
    src.start(t, Math.random() * (buffer.duration - dur - 0.1), dur + 0.05);
    src.onended = () => out.disconnect();
  }
  /** Low pitch-drop thud. k = 0..1 strength; body = chassis / roof hit (a little higher and crunchier). */
  function thud(k, body = false) {
    // soft hits: slower attack, lower + duller, shorter; pitch / length jittered so repeats don't sound sampled
    const t = ctx.currentTime, o = ctx.createOscillator(), a = ctx.createGain(), lp = ctx.createBiquadFilter();
    const len = (0.18 + 0.24 * k) * jitter(0.12), f0 = ((body ? 120 : 82) + 60 * k) * jitter(0.1);
    o.type = 'triangle';
    o.frequency.setValueAtTime(f0, t);
    o.frequency.exponentialRampToValueAtTime(Math.min(f0 * 0.9, 38 + 8 * Math.random()), t + 0.45 * len);
    lp.type = 'lowpass'; lp.frequency.value = 160 + 260 * k; lp.Q.value = 0.4;
    env(a.gain, t, hitGain(k, 0.12, 0.85), 0.012 - 0.008 * k, len);
    o.connect(lp); lp.connect(a); a.connect(g.bus);
    o.start(t); o.stop(t + len + 0.05);
    o.onended = () => { a.disconnect(); lp.disconnect(); };
    noiseBurst(g.brown, t, 0.45 * len, (body ? 0.8 : 0.45) * hitGain(k, 0.2, 1), [['lowpass', (body ? 260 : 160) + 260 * k, 0.6]]);
  }
  /** Tyre landing on dirt: sine body + 2nd harmonic + brown-noise thump, all < ~1 kHz (no click, audible on small
   *  speakers). Replaces impact_thud.mp3 (sub boom + 5 kHz click = hollow tin can on phones). gain = busy duck. */
  function tyreThud(k, gain = 1) {
    const t = ctx.currentTime, len = (0.16 + 0.22 * k) * jitter(0.1), f0 = (75 + 40 * k) * jitter(0.1);
    const peak = hitGain(k, 0.1, 0.8) * gain, att = 0.014 - 0.008 * k;
    const o = ctx.createOscillator(), a = ctx.createGain();
    o.type = 'sine';
    o.frequency.setValueAtTime(f0, t);
    o.frequency.exponentialRampToValueAtTime(45, t + 0.6 * len);
    env(a.gain, t, peak, att, len);
    o.connect(a); a.connect(g.bus);
    const h = ctx.createOscillator(), hg = ctx.createGain(), lp = ctx.createBiquadFilter();
    h.type = 'triangle';
    h.frequency.setValueAtTime(2 * f0, t);
    h.frequency.exponentialRampToValueAtTime(90, t + 0.5 * len);
    lp.type = 'lowpass'; lp.frequency.value = 300 + 300 * k; lp.Q.value = 0.4;
    env(hg.gain, t, 0.35 * peak, att, 0.7 * len);
    h.connect(lp); lp.connect(hg); hg.connect(g.bus);
    for (const n of [o, h]) { n.start(t); n.stop(t + len + 0.05); }
    o.onended = () => { a.disconnect(); hg.disconnect(); lp.disconnect(); };
    noiseBurst(g.brown, t, 0.35 * len, peak * (0.6 + 0.4 * k), [['lowpass', (350 + 450 * k) * jitter(0.1), 0.6]]);
  }
  /** Metallic ratchet pawl: noise ping through two inharmonic resonances; k = tension / maxPull. */
  function click(k) {
    const f = 2300 + 1200 * k + Math.random() * 250;
    noiseBurst(g.pink, ctx.currentTime, 0.028, 0.8 + 1.4 * k, [['bandpass', f, 14], ['bandpass', f * 1.58, 18]]);
  }
  function snap() {
    noiseBurst(g.pink, ctx.currentTime, 0.11, 0.9, [['highpass', 1800, 0.7]]);
    thud(0.55, true);
  }

  /** Dying-engine cough: three decaying low pops + a falling thud. */
  function cough() {
    const t = ctx.currentTime;
    for (let i = 0; i < 3; i++) noiseBurst(g.brown, t + i * 0.09 + Math.random() * 0.02, 0.07, 1.1 * (1 - i * 0.28), [['lowpass', 300 - i * 50, 0.9]]);
    thud(0.35);
  }
  /** Misfire / backfire: one short exhaust pop. */
  function pop() {
    noiseBurst(g.brown, ctx.currentTime, 0.05, 0.9, [['lowpass', 380, 1.2], ['bandpass', 140, 2]]);
  }
  /** Axle shaft snap: sharp crack, inharmonic metal ring, heavy driveline clunk. */
  function axleSnap() {
    const t = ctx.currentTime;
    noiseBurst(g.pink, t, 0.06, 1.4, [['highpass', 2500, 0.7]]);
    noiseBurst(g.pink, t, 0.45, 1.6, [['bandpass', 2600, 25], ['bandpass', 4100, 25], ['bandpass', 5700, 25]]);
    thud(0.7, true);
  }
  /** Body crunch: grinding bandpassed noise stutter + low clank ring; k = 0..1. */
  function crunch(k) {
    const t = ctx.currentTime;
    for (let i = 0; i < 3; i++) noiseBurst(g.pink, t + i * 0.035, 0.05, (0.6 + 0.8 * k) * (1 - i * 0.2), [['bandpass', 900 + Math.random() * 300, 1.4]]);
    noiseBurst(g.pink, t + 0.01, 0.35, 0.8 + 0.8 * k, [['bandpass', 420, 18], ['bandpass', 690, 18]]);
    thud(0.5 + 0.5 * k, true);
  }
  /** Lever notch: crisp detent tick (bright metal ping + short low knock). */
  function notch(k = 1) {
    const t = ctx.currentTime, f = 2600 * jitter(0.08);
    noiseBurst(g.pink, t, 0.022, 0.9 * k, [['bandpass', f, 9], ['bandpass', f * 1.47, 12]]);
    noiseBurst(g.brown, t + 0.004, 0.05, 0.9 * k, [['lowpass', 420, 0.8]]);
  }
  /** Transfer-case engage: two heavy driveline clunks ~0.12 s apart (dog teeth slam, then the case settles). */
  function clunk(k = 1, rate = 1) {
    const t0 = ctx.currentTime;
    for (const [dt, a] of [[0, 1], [0.12 / rate, 0.7]]) {
      const t = t0 + dt, o = ctx.createOscillator(), og = ctx.createGain(), lp = ctx.createBiquadFilter();
      const f0 = 95 * rate * jitter(0.06), len = 0.2 * jitter(0.1);
      o.type = 'triangle';
      o.frequency.setValueAtTime(f0, t);
      o.frequency.exponentialRampToValueAtTime(f0 * 0.55, t + 0.6 * len);
      lp.type = 'lowpass'; lp.frequency.value = 380; lp.Q.value = 0.5;
      env(og.gain, t, 0.75 * k * a, 0.004, len);
      o.connect(lp); lp.connect(og); og.connect(g.bus);
      o.start(t); o.stop(t + len + 0.05);
      o.onended = () => { og.disconnect(); lp.disconnect(); };
      noiseBurst(g.brown, t, 0.08, 1.0 * k * a, [['lowpass', 520 * rate, 0.7]]);
      noiseBurst(g.pink, t, 0.16, 0.5 * k * a, [['bandpass', 1150 * rate, 16], ['bandpass', 1870 * rate, 18]]);
    }
  }
  /** Gear grind (dog teeth skating on a moving shift): rough bandpassed noise stutter + low rattle. */
  function grind(k = 1) {
    const t = ctx.currentTime;
    for (let i = 0; i < 5; i++) noiseBurst(g.pink, t + i * 0.045 * jitter(0.2), 0.04, 0.7 * k * (1 - i * 0.12), [['bandpass', 1700 * jitter(0.15), 3], ['bandpass', 3100, 6]]);
    noiseBurst(g.brown, t, 0.25, 0.6 * k, [['lowpass', 300, 0.8]]);
  }
  /** Tone helper: osc type f0 -> f1 Hz over len s (exp), through a filter, attack / decay envelope. */
  function tone(type, f0, f1, len, peak, filt = ['lowpass', 2000, 0.7], t = ctx.currentTime, vib = 0) {
    const o = ctx.createOscillator(), a = ctx.createGain(), f = ctx.createBiquadFilter();
    o.type = type; o.frequency.setValueAtTime(f0, t); o.frequency.exponentialRampToValueAtTime(Math.max(20, f1), t + len);
    f.type = filt[0]; f.frequency.value = filt[1]; f.Q.value = filt[2];
    env(a.gain, t, peak, 0.01, len);
    o.connect(f); f.connect(a); a.connect(g.bus);
    let lfo = null;
    if (vib) { lfo = ctx.createOscillator(); const lg = ctx.createGain(); lfo.frequency.value = vib; lg.gain.value = f0 * 0.04; lfo.connect(lg); lg.connect(o.frequency); lfo.start(t); lfo.stop(t + len + 0.05); }
    o.start(t); o.stop(t + len + 0.05);
    o.onended = () => { a.disconnect(); f.disconnect(); };
  }
  const squeakSyn = k => tone('sine', 950 * jitter(0.1), 1450 * jitter(0.1), 0.18 + 0.1 * k, 0.12 + 0.2 * k, ['bandpass', 1200, 2], undefined, 23);
  const groanSyn = k => tone('sawtooth', 85 * jitter(0.08), 48, 0.55 + 0.3 * k, 0.25 + 0.4 * k, ['bandpass', 320, 3]);
  const springSyn = k => { tone('sine', 260 * jitter(0.1), 180, 0.6, 0.2 + 0.35 * k, ['lowpass', 1400, 0.7], undefined, 14);
    noiseBurst(g.pink, ctx.currentTime, 0.4, 0.4 * k, [['bandpass', 2300, 30], ['bandpass', 3700, 30]]); };
  /** Hot exhaust / block contracting: sparse metallic ticks over len s (fallback for clicking_eng_cooldown). */
  function ticks(k, len) {
    const t = ctx.currentTime, n = Math.round(4 + 10 * k);
    for (let i = 0; i < n; i++) {
      const f = 3200 + Math.random() * 2400;
      noiseBurst(g.pink, t + 0.3 + Math.random() * len, 0.02, (0.25 + 0.35 * k) * (1 - i / (n * 1.4)), [['bandpass', f, 22], ['bandpass', f * 1.37, 24]]);
    }
  }
  /** Drivetrain controls (state edges): grind on a moving (pending) range request, gear2 / gear3 on commit, gear1 on 4WD / locker / REV. */
  function shifts(v, dt, play) {
    cool.notch -= dt; cool.grind -= dt;
    const now = { fourWD: !!v.fourWD, lockF: !!v.lockF, lockR: !!v.lockR, rev: v.gear === 'R',
                  range: v.range || 'H', pend: v.pendingRange || null };
    const was = gear; gear = now;
    if (!was || !play) return;
    if (now.range !== was.range) {                          // committed (stopped tap, or pending shift completing)
      const pending = was.pend === now.range, low = now.range === 'L' || now.range === 'L-' || (was.range === 'H' && now.range !== 'H');
      const heavy = pending || low, rate = (SHIFT_RATE[now.range] || 1) * jitter(0.03), k = heavy ? 1 : 0.85;
      if (!shot(heavy ? 'gear3' : 'gear2', 0.95 * k, { rate }) && !(heavy && shot('gear2', 0.95, { rate: rate * 0.92 }))) clunk(k, rate);
      cool.notch = Math.max(cool.notch, 0.2);                // the clunk already carries the lever
      return;
    }
    if (now.pend && now.pend !== was.pend) {                 // requested while moving > SHIFT_SPEED (or re-requested): grind
      if (cool.grind <= 0) {
        const k = clamp01(0.55 + Math.abs(v.speed || 0) / 8);
        if (!shot('gearFail', 0.75 * k, { rate: jitter(0.05) })) grind(k);
        cool.grind = STRESS_COOL.grind; cool.notch = Math.max(cool.notch, 0.15);
      }
      return;
    }
    let lever = now.pend !== was.pend ? 0.7 : 0;            // pending request cancelled (lever back to the engaged range)
    for (const n of ['fourWD', 'lockF', 'lockR']) if (now[n] !== was[n]) lever = 1;
    if (now.rev !== was.rev) lever = Math.max(lever, 0.7);
    if (!lever || cool.notch > 0) return;
    cool.notch = SHIFT_COOL;
    if (!shot('gear1', 0.85 * lever, { rate: jitter(0.05) })) notch(lever);
  }
  /** Transfer-case lever: 'notch' = detent clack as it drops into a gate, 'commit' = gears mesh (clunk + dog ring), 'drive' = 4WD / locker
   *  actuator (air-solenoid tick + small clunk). All synth, short, through the hit bus. */
  function lever(kind, info = {}) {
    const t = ctx.currentTime;
    if (kind === 'commit') {
      noiseBurst(g.pink, t, 0.04, 1.1, [['bandpass', 1500, 6], ['bandpass', 2300, 9]]);
      noiseBurst(g.pink, t + 0.03, 0.22, 0.7, [['bandpass', 880, 20], ['bandpass', 1370, 22]]);
      thud(info.range && info.range !== 'H' ? 0.45 : 0.35, true);
    } else if (kind === 'drive') {
      noiseBurst(g.pink, t, 0.05, 0.6, [['highpass', 3200, 0.7]]);
      noiseBurst(g.brown, t + 0.04, 0.06, info.on ? 0.9 : 0.6, [['lowpass', 420, 1], ['bandpass', 170, 2]]);
    } else {
      const f = 2600 + Math.random() * 300;
      noiseBurst(g.pink, t, 0.022, 0.9, [['bandpass', f, 10], ['bandpass', f * 1.47, 12]]);
      noiseBurst(g.brown, t + 0.008, 0.04, 0.7, [['lowpass', 520, 1]]);
    }
  }
  function failures(v, t, dt) {
    const h = v.hardcore;
    if (!h) return;
    cool.misfire -= dt; cool.tick -= dt;
    // starter: crank file's starter section looped from its start on each crank; when the engine catches its fire-up tail
    // plays (CRANKS catch) while the idle loop fades in; gives up silently (dry tank). Synth whirr / cough without samples.
    const ownCrank = vid && ov[vid] && ov[vid].crank, cr = h.cranking && !sx.cranking ? crankBuf() : null;
    if (cr) {
      if (sx.cooldownSrc) { stopShot(sx.cooldownSrc, t); sx.cooldownSrc = null; }
      const rate = ownCrank || (prof && fileBuf(prof.crank)) ? 1 : engRate('crank', 'crank');
      crankSrc = shot(cr.b, 0.8 * voice.gain, { loop: true, rate, sl: cr.loop || null });
      sx.catch = cr.catch ? { b: cr.b, sec: cr.catch, rate } : null;
    }
    if (!h.cranking && sx.cranking) {
      const caught = v.engineOn !== false;
      if (crankSrc) {                                              // caught: starter fades out equal-power over CATCH_XF while the
        if (caught) { epRamp(crankSrc._gain.gain, t, false); try { crankSrc.stop(t + CATCH_XF + 0.05); } catch (_) {} }   // idle bus
        else stopShot(crankSrc, t);                                // fades in (engine()); gave up: plain stop
        crankSrc = null;
      }
      if (caught) {                                                // fire-up tail (soft 60 ms attack, <= 0.9 s), else a synth cough-pop
        const c = sx.catch, len = c ? Math.min(0.9, (c.sec[1] - c.sec[0]) / c.rate) : 0;
        if (!(c && shot(c.b, 0.6 * voice.gain, { offset: c.sec[0], dur: len, rate: c.rate, attack: 0.06 }))) { pop(); thud(0.25); }
      }
      sx.catch = null;
    }
    sx.cranking = h.cranking;
    g.crankGain.gain.setTargetAtTime(h.cranking && !crankSrc ? 0.35 : 0, t, 0.03);
    // engine warmth (state-only): builds while running (faster on load), eases off when stopped; max with engineHeat
    const running = v.engineOn !== false;
    sx.warm = clamp01(sx.warm + (running ? dt * (0.004 + 0.02 * clamp01(v.throttle || 0) * clamp01(v.rpm / 4000)) : -dt * 0.004));
    if (sx.on && !running && !h.cranking) {                      // just shut off / stalled: cooldown ticking, level + length by heat
      const heat = Math.max(sx.warm, h.engineHeat || 0);
      if (heat > 0.12 && cool.tick <= 0) {
        const len = 3 + 9 * heat;
        sx.cooldownSrc = shot('cooldown', 0.2 + 0.55 * heat, { offset: Math.random() * 3, dur: len, lp: 3000 + 5000 * heat });
        if (!sx.cooldownSrc) ticks(heat, len);
        cool.tick = STRESS_COOL.tick;
      }
    }
    if (running && sx.cooldownSrc) { stopShot(sx.cooldownSrc, t); sx.cooldownSrc = null; }
    // heat-soak clinks while RUNNING hot at low rev / light gas (the idle doesn't mask them): intermittent faded cooldown
    // slices, level + brightness by heat; fade out once revved (> CLINK_RN), off gas, cooled or stopped (off path takes over)
    const heatR = Math.max(h.engineHeat || 0, h.engineHot ? 1 : 0, 0.85 * sx.warm), kH = clamp01((heatR - CLINK_HEAT) / (1 - CLINK_HEAT));
    const clinkOk = running && heatR > CLINK_HEAT && (sx.rn ?? 1) < CLINK_RN && (sx.thrS ?? 1) < 0.3;
    cool.clink -= dt;
    if (clinkOk && !sx.clinkSrc && cool.clink <= 0) {
      const len = 2.5 + 2 * Math.random(), src = shot('cooldown', 0.22 + 0.33 * kH, { offset: Math.random() * 3, dur: len, lp: 3500 + 4500 * kH, attack: 0.2 });
      if (src) { sx.clinkSrc = src; src.addEventListener('ended', () => { if (sx.clinkSrc === src) sx.clinkSrc = null; }); }
      else ticks(0.4 + 0.4 * kH, len);
      cool.clink = len + 0.6 + 1.4 * Math.random() * (1 - kH);          // hotter = denser clinking
    }
    if (!clinkOk && sx.clinkSrc) { stopShot(sx.clinkSrc, t); sx.clinkSrc = null; }
    sx.on = running;
    const hot = !!h.engineHot;
    if ((hot && !sx.hot) || (h.stalled && !st.stalled)) {        // overheat / stall: the full coughing misfire take
      if (shot('misfire', 0.9 * voice.gain, { rate: engRate('misfire', 'misfire') })) cool.misfire = 2.5; else cough();
    } else if (h.misfire && !st.misfire) {                         // misfire slot: a short faded slice, synth pop between
      if (cool.misfire <= 0 && shot('misfire', 0.75 * voice.gain, { offset: Math.random() * 1.5, dur: 0.9, rate: jitter(0.05) * engRate('misfire', 'misfire') })) cool.misfire = 1.4;
      else pop();
    }
    sx.hot = hot;
    for (let i = 0; i < 2; i++) { if (h.axleBroken[i] && !st.axle[i] && !shot('axle', 1.0)) axleSnap(); st.axle[i] = h.axleBroken[i]; }
    if (st.body !== null && h.body - st.body >= 0.02) {
      const k = clamp01((h.body - st.body) / 0.25);
      if (!buf.metal2) crunch(k);
      else if (cool.metal <= 0) { shot('metal2', 0.6 + 0.4 * k, { rate: jitter() }); cool.metal = 0.25; }
    }
    st.stalled = h.stalled; st.misfire = h.misfire; st.body = h.body;
  }
  function stopShot(src, t) {
    try { src._gain.gain.setTargetAtTime(0, t, 0.04); src.stop(t + 0.25); } catch (_) {}
  }

  // ---- per-frame voices -----------------------------------------------------------------------
  /** Equal-power gate ramp of a gain param over CATCH_XF (in: current -> peak, out: current -> 0); holds prior automation. */
  function epRamp(param, t, into, peak = 1) {
    if (param.cancelAndHoldAtTime) param.cancelAndHoldAtTime(t); else { param.cancelScheduledValues(t); param.setValueAtTime(param.value, t); }
    const from = param.value, curve = (into ? EP_IN : EP_OUT).map(k => (into ? from + (peak - from) * k : from * k));
    try { param.setValueCurveAtTime(curve, t + 0.001, CATCH_XF); } catch (_) { param.setTargetAtTime(into ? peak : 0, t, CATCH_XF / 3); }
  }
  function engine(v, eng, t) {
    const idle = eng.idleRpm ?? 800, red = eng.redlineRpm ?? 5000;
    const dtE = Math.min(0.1, Math.max(0, t - (sx.tLast ?? t)));
    sx.tLast = t;
    // JS-side smoothing: wheel-hop rpm jitter / finger noise is filtered BEFORE it reaches playbackRate, filter and blend,
    // so loops glide instead of warbling; engine catch glides 0.6 x idle -> idle (no pitch jump)
    const rpmRaw = Math.max(0.6 * idle, v.rpm || 0.6 * idle), thrRaw = clamp01(v.throttle || 0);
    const ease = (s, x, up, dn) => (s == null ? x : s + (x - s) * (1 - Math.exp(-dtE / (x > s ? up : dn))));
    const thr = sx.thrS = ease(sx.thrS, thrRaw, THR_UP, THR_DN);
    sx.rpmS = ease(sx.rpmS, rpmRaw, RPM_TAU, lerp(RPM_TAU, RPM_FALL_GAS, thr * thr));
    sx.rpmPk = Math.max(sx.rpmS, sx.rpmPk == null ? sx.rpmS : sx.rpmPk + (sx.rpmS - sx.rpmPk) * (1 - Math.exp(-dtE / PEAK_TAU)));
    const rpm = sx.rpmS + PEAK_HOLD * thr * thr * (sx.rpmPk - sx.rpmS);   // audible rpm: steady roar under held gas
    const V = voice, rn = clamp01((rpm - idle) / (red - idle)), f = rpm / 60 * (V.order || ENGINE_ORDER);
    sx.rn = rn;
    sx.load = ease(sx.load, clamp01(0.65 * thr + 0.6 * rn), LOAD_UP, LOAD_DN);   // gas-led: held full gas = rev loop, no see-saw
    g.oscA.frequency.setTargetAtTime(f, t, 0.03);
    g.oscB.frequency.setTargetAtTime(f, t, 0.03);
    g.sub.frequency.setTargetAtTime(f / 2, t, 0.03);
    g.subGain.gain.setTargetAtTime(0.28 * V.sub, t, 0.1);
    g.engLP.frequency.setTargetAtTime(V.synthLp * lerp(LP_MIN, LP_MAX, clamp01(0.65 * rn + 0.45 * thr)), t, 0.05);
    // recording: truck override > profile idle / rev slices (in place, loopStart / loopEnd) > shared engine_idle
    const over = vid && ov[vid] && ov[vid].idle, pIdle = !over && prof ? slice(prof.idle) : null;
    const pRev = pIdle && prof.rev ? slice(prof.rev) : null;
    const want = over || (pIdle ? pIdle.b : buf.idle), wantR = pRev ? pRev.b : null;
    if (want && (!vox.idle || vox.idle.b !== want)) startLoop('idle', want, over ? null : pIdle);   // truck change / sample decoded
    if (wantR ? !vox.rev || vox.rev.b !== wantR : vox.rev) wantR ? startLoop('rev', wantR, pRev) : stopLoop('rev');
    const P = pIdle ? profVoice() : V;
    // engine OFF (spawn / key off / stall) is silent; cranking: the synth starter only (v.engineOn false while off / cranking)
    const on = v.engineOn !== false ? 1 : (v.hardcore && v.hardcore.cranking && !crankSrc ? 0.5 : 0);
    const idleV = vox.idle, revV = vox.rev, running = v.engineOn !== false;
    g.engGain.gain.setTargetAtTime(on * (idleV ? Math.min(1, SYNTH_UNDER * 1.3 * P.synth) : 1) * (0.2 + 0.18 * thr + 0.08 * rn), t, on ? 0.06 : 0.12);
    // sub-bus gate: equal-power CATCH_XF fade in on every engine-on edge (crank catch / startEngine), out on stall / key off
    if (running !== !!sx.engOn) { epRamp(g.engSmp.gain, t, running); sx.engOn = running; if (running) sx.rpmS = sx.rpmPk = Math.min(sx.rpmS, 0.6 * idle + 0.4 * rpmRaw); }
    if (idleV) {                                          // recorded idle: pitch with rpm, opens up + louder with throttle
      const c = LOOP.idle, misfire = v.hardcore && v.hardcore.misfire ? 0.55 : 1;
      const base = pIdle ? P.rate : engRate('idle', 'rate'), lvl = misfire * c.gain * P.gain * (0.55 + 0.3 * thr + 0.25 * rn);
      // idle <-> rev: equal-power crossfade by smoothed load (smoothstep: no lurch at either end)
      const L = sx.load, x = revV ? L * L * (3 - 2 * L) : 0, gi = Math.cos(x * Math.PI / 2), gr = Math.sin(x * Math.PI / 2);
      // rate: rpm (soft-capped below maxRate, no plateau kink) + a touch of throttle so feathering is heard even when bogged
      const rate = Math.max(0.4, softCap(base * Math.pow(rpm / idle, P.rpmExp) * (1 + 0.04 * thr), P.maxRate));
      idleV.src.playbackRate.setTargetAtTime(rate, t, 0.05);
      idleV.f.frequency.setTargetAtTime(P.lp * expLerp(c.f0, c.f1, 0.35 * thr + 0.55 * rn + 0.15 * thr * rn), t, 0.06);
      idleV.f.Q.setTargetAtTime(P.q, t, 0.1);
      idleV.g.gain.setTargetAtTime(lvl * gi, t, 0.06);
      if (revV) {                                         // pitch-matched to the idle loop (rate / revPitch): no beating mid-blend
        const cr = LOOP.rev;
        revV.src.playbackRate.setTargetAtTime(Math.max(0.4, rate / (prof.revPitch || 1)), t, 0.05);
        revV.f.frequency.setTargetAtTime(P.lp * expLerp(cr.f0, cr.f1, 0.25 + 0.4 * thr + 0.45 * rn), t, 0.06);   // full gas + high rpm = wide open
        revV.f.Q.setTargetAtTime(Math.min(P.q, 0.9), t, 0.1);
        revV.g.gain.setTargetAtTime(lvl * gr * cr.gain / c.gain, t, 0.06);
      }
    }
    // flare: profile free-rev blip only on a genuine stab (< 40 % -> > 85 % within 0.5 s, below 50 % rpm, one at a time) -- never repeated
    // while held at high rpm (that re-triggering is what pulsed); pitched with the loops, quick attack
    sx.thrLow = thrRaw < 0.4 ? 0.5 : Math.max(0, sx.thrLow - dtE);
    cool.rev -= dtE;
    if (pIdle && prof.flare && running && cool.rev <= 0 && thrRaw > 0.85 && sx.thrLow > 0 && rn < 0.5 && !sx.flareSrc) {
      const r = prof.flare, rate = Math.min(1.3, Math.max(0.85, idleV ? idleV.src.playbackRate.value / (prof.revPitch || 1) : 1));
      const src = fileBuf(r[0]) && shot(fileBuf(r[0]), 0.4 * P.gain, { offset: r[1], dur: (r[2] - r[1]) / rate, rate, lp: 9000, attack: 0.03 });
      if (src) { cool.rev = STRESS_COOL.rev * jitter(0.2); sx.flareSrc = src; src.addEventListener('ended', () => { if (sx.flareSrc === src) sx.flareSrc = null; }); }
      sx.thrLow = 0;
    }
  }

  /** Surface + water loops from each wheel's tread speed / slip; splashes on wheels (or the chassis) entering water. */
  function surfaces(v, t, dt) {
    for (const c in acc) { const a = acc[c]; a.n = 0; a.drive = 0; a.slip = 0; }
    const ws = v.wheels, nw = ws.length || 1, spd = Math.abs(v.speed || 0), hv = Math.hypot(v.vx || 0, v.vy || 0);
    let deep = 0, entry = 0;
    cool.splash -= dt;
    for (let i = 0; i < ws.length; i++) {
      const w = ws[i], cat = w.grounded ? surfaceCat(w.surface) : null;
      const wet = cat === 'water';
      if (wet && !sx.wet[i]) entry = Math.max(entry, hv);
      sx.wet[i] = wet;
      if (!cat) continue;
      const a = acc[cat], slip = Math.abs(w.slip || 0);
      a.n += 1 / nw; a.drive += Math.max(spd, Math.abs(w.surfaceSpeed || 0)) / nw; a.slip += slip / nw;
      if (wet && /deep/.test(w.surface)) deep += 1 / nw;
    }
    const cc = v.chassisContacts || [];
    let bodyWet = false;
    for (const c of cc) if (surfaceCat(c.surface) === 'water') { bodyWet = true; break; }
    if (bodyWet) { if (!sx.bodyWet) entry = Math.max(entry, hv * 1.3); deep = 1; }
    sx.bodyWet = bodyWet;
    for (const cat of SURF_CATS) {
      const a = acc[cat], c = LOOP[cat];
      if (!vox[cat]) continue;
      const drive = a.n > 0 ? a.drive / a.n : 0, k = clamp01(drive / c.full), sk = clamp01((a.n > 0 ? a.slip / a.n : 0) / 4);
      const lvl = drive < 0.15 ? 0 : c.gain * a.n * Math.pow(k, 0.6) * (1 + 0.4 * sk);
      setLoop(cat, lvl, k, sk, t);
    }
    const wa = acc.water, wd = wa.n > 0 ? wa.drive / wa.n : 0, wet = Math.max(wa.n, bodyWet ? 1 : 0);
    const kw = clamp01(Math.max(wd, hv) / LOOP.shallow.full);
    setLoop('shallow', LOOP.shallow.gain * wet * (1 - deep) * (0.25 + 0.75 * kw), kw, 0, t);
    setLoop('deep', LOOP.deep.gain * deep * (0.3 + 0.7 * kw), kw, 0, t);
    if (entry > 0.6 && cool.splash <= 0) {                // both wheels of an axle land in the water as one splash
      const big = entry > 3.5;
      if (shot(big ? 'splashBig' : 'splashSmall', clamp01(0.3 + entry / (big ? 9 : 4)), { rate: jitter() })) cool.splash = 0.7;
    }
  }

  function tyres(v, t) {
    let sum = 0, peak = 0;
    for (const k of SLIP_KEYS) mix[k] = 0;
    let under = 0;
    for (const w of v.wheels) {
      const s = w.grounded ? Math.abs(w.slip) - SLIP_DEADBAND : 0;
      if (!(s > 0)) continue;
      const cat = surfaceCat(w.surface), p = SLIP[w.surface] || SLIP[SLIP_OF_CAT[cat]] || SLIP.dirt;
      for (const k of SLIP_KEYS) mix[k] += p[k] * s;       // slip-weighted blend (split-mu = two surfaces at once)
      under += s * (vox[cat === 'water' ? 'shallow' : cat] ? SYNTH_UNDER : 1);   // sample playing -> synth sits under it
      sum += s; if (s > peak) peak = s;
    }
    if (sum <= 0) { g.slipGain.gain.setTargetAtTime(0, t, 0.08); return; }
    for (const k of SLIP_KEYS) mix[k] /= sum;
    const k = clamp01(peak / SLIP_FULL);
    g.slipBP.frequency.setTargetAtTime(lerp(mix.lo, mix.hi, k), t, 0.06);
    g.slipBP.Q.setTargetAtTime(mix.q, t, 0.06);
    g.pinkMix.gain.setTargetAtTime(mix.pink, t, 0.06);
    g.brownMix.gain.setTargetAtTime(mix.brown * BROWN_MAKEUP, t, 0.06);
    g.slipGain.gain.setTargetAtTime(mix.gain * Math.pow(k, 0.7) * (under / sum), t, 0.05);
  }

  /** Impact one-shot level: k^1.5 curve, ducked while hits pile up (rock garden), lowpass opens with strength. */
  function hitShot(name, k, lo, hi, opt = {}) {
    const duck = 1 / (1 + 0.3 * hitFx.busy);
    return shot(name, hitGain(k, lo, hi) * duck, { rate: jitter(0.07), lp: lerp(900, 9000, k * k) * (0.6 + 0.4 * duck), ...opt });
  }
  /** Chassis metal take: light -> metal1, hard -> metal2, in between alternate so repeats vary. */
  function metalTake(k) {
    if (!buf.metal2) return 'metal1';
    if (!buf.metal1 || k > 0.7) return 'metal2';
    if (k < 0.35) return 'metal1';
    return hitFx.metal === 'metal1' ? 'metal2' : 'metal1';
  }
  function impacts(v, dt) {
    let hit = 0, bottomed = false;
    hitFx.busy *= Math.exp(-dt / BUSY_TAU);
    for (let i = 0; i < v.wheels.length; i++) {
      const w = v.wheels[i], s = st.wheels[i] || (st.wheels[i] = { c: null, t: 0, cool: 0, peak: 0, vn: 0 });
      s.cool -= dt; s.t += dt;
      if (s.c === null) { s.c = w.compression; s.t = 0; continue; }
      if (w.compression === s.c) continue;                 // no physics step happened this frame
      const rate = (w.compression - s.c) / Math.min(0.1, Math.max(s.t, 1 / 240));
      s.c = w.compression; s.t = 0;
      let wvn = 0;
      for (const c of w.contacts || []) if (c.vn > wvn) wvn = c.vn;
      if (s.peak > 0) {                                    // inside a spike: fire once the rate stops rising
        if (wvn > s.vn) s.vn = wvn;
        if (rate > s.peak) { s.peak = rate; continue; }
        const bot = w.compression > 0.93;
        const k = Math.max(clamp01((s.peak - HIT_RATE) / HIT_SPAN), clamp01((s.vn - 0.6) / 4.5)) + (bot ? 0.2 : 0);
        if (k >= HIT_MIN) { hit = Math.max(hit, k); bottomed = bottomed || bot; }   // minor bumps: silent
        s.peak = 0; s.vn = 0; s.cool = 0.22;
      } else if (rate > HIT_RATE && s.cool <= 0) { s.peak = rate; s.vn = wvn; }
    }
    cool.thud -= dt; cool.metal -= dt; cool.boulder -= dt;
    sx.bottomK = bottomed ? Math.min(1, hit) : 0;           // -> chassisStress(): metallic groan on heavy bottom-outs
    if (hit > 0 && cool.thud <= 0) {                       // left + right wheel of an axle land as one thud
      const k = Math.min(1, hit);
      tyreThud(Math.min(1, k + (bottomed ? 0.1 : 0)), 1 / (1 + 0.3 * hitFx.busy));
      cool.thud = HIT_COOL.thud * jitter(0.15);
      hitFx.busy += 0.5 + k;
    }
    // boulder / loose rock strikes: a wheel or the chassis closing fast on a non-terrain stone (logs with surface rock)
    let rockVn = 0;
    for (const w of v.wheels) for (const c of w.contacts || []) if (!c.terrain && c.vn > rockVn && STONE_RE.test(c.surface)) rockVn = c.vn;

    st.bodyCool -= dt;
    const cc = v.chassisContacts || [], touching = cc.length > 0;
    let f = 0, vn = 0, slide = 0, stone = false;
    for (const c of cc) {
      if (c.force > f) f = c.force;
      if ((c.vn || 0) > vn) { vn = c.vn; stone = STONE_RE.test(c.surface || ''); }
      if (!c.terrain && STONE_RE.test(c.surface) && c.vn > rockVn) rockVn = c.vn;
      if (surfaceCat(c.surface) === 'water') continue;
      const s = Math.abs(-(v.vx || 0) * c.ny + (v.vy || 0) * c.nx);   // chassis speed along the contact tangent
      if (s > slide) slide = s;
    }
    sx.slamK = 0;
    if (touching && !st.bodyHit && st.bodyCool <= 0 && (f > BODY_F || vn > BODY_VN)) {
      const k = Math.max(clamp01((f - BODY_F) / 45000), clamp01((vn - BODY_VN + 0.3) / 4.5));
      sx.slamK = k;
      if (buf.metal1 || buf.metal2) {                      // bottom-out: light / medium vs hard metal hit
        const take = metalTake(k);
        if (cool.metal <= 0 && hitShot(take, k, 0.12, 0.95)) { hitFx.metal = take; cool.metal = HIT_COOL.metal * jitter(0.15); }
        if (stone && vn > 3 && cool.boulder <= 0 && hitShot('boulder', clamp01((vn - 2) / 5), 0.2, 0.9)) cool.boulder = HIT_COOL.boulder;
      } else if (cool.metal <= 0) { thud(k, true); cool.metal = HIT_COOL.metal; }
      hitFx.busy += 0.5 + k;
      st.bodyCool = 0.3;
    }
    st.bodyHit = touching;
    if (rockVn > ROCK_VN && cool.boulder <= 0 && hitShot('boulder', clamp01((rockVn - ROCK_VN + 0.4) / 5), 0.12, 0.9)) {
      cool.boulder = HIT_COOL.boulder * jitter(0.15); hitFx.busy += 0.5;
    }
    // undercarriage dragging: metal_scrape loop by sliding speed (fades quickly once the chassis lifts off)
    sx.scrape = touching ? clamp01((slide - 0.25) / LOOP.scrape.full) : 0;
    const fk = clamp01(f / 20000);
    setLoop('scrape', sx.scrape > 0 ? LOOP.scrape.gain * Math.pow(sx.scrape, 0.7) * (0.5 + 0.5 * fk) : 0, sx.scrape, fk, ctx.currentTime);
  }

  /** Suspension / frame stress: squeaks while the springs work hard (high compression or cross-axle articulation, earlier
   *  and more often with worn shocks), metallic groan on a heavy bottom-out / chassis slam, metal spring on fast worn rebound. */
  function chassisStress(v, dt) {
    cool.squeak -= dt; cool.groan -= dt; cool.spring -= dt;
    const ws = v.wheels || [];
    let comp = 0, art = 0, motion = 0, wear = 0, rebound = 0;
    for (let i = 0; i < ws.length; i++) {
      const w = ws[i], c = w.compression || 0, d = w.damage || 0, p = sx.susp[i];
      const rate = p === undefined || dt <= 0 ? 0 : (c - p) / Math.max(dt, 1 / 240);   // travel fractions / s
      sx.susp[i] = c;
      comp = Math.max(comp, clamp01((c - 0.7) / 0.3));
      motion = Math.max(motion, clamp01(Math.abs(rate) / 1.2));
      wear += d / (ws.length || 1);
      if (d > 0.25 && w.grounded && rate < -2.5) rebound = Math.max(rebound, d * clamp01((-rate - 2.5) / 4));
    }
    for (let a = 0; a + 1 < ws.length; a += 2) art = Math.max(art, clamp01((Math.abs((ws[a].compression || 0) - (ws[a + 1].compression || 0)) - 0.35) / 0.4));
    const drive = Math.max(comp, art) * motion * (1 + 1.5 * wear);
    if (drive > 0.35 - 0.15 * wear && cool.squeak <= 0) {
      const k = clamp01(drive), at = SQUEAK_AT[(Math.random() * SQUEAK_AT.length) | 0];
      if (!shot('squeak', 0.12 + 0.3 * k + 0.15 * wear, { offset: at, dur: 0.45, rate: jitter(0.08) })) squeakSyn(k);
      cool.squeak = STRESS_COOL.squeak * (1.3 - 0.7 * wear) * jitter(0.25);
    }
    const heavy = Math.max(sx.bottomK >= 0.4 ? sx.bottomK : 0, sx.slamK >= 0.45 ? sx.slamK : 0);
    if (heavy > 0 && cool.groan <= 0) {
      if (!shot('groan', 0.25 + 0.55 * heavy, { rate: jitter(0.08) * (1.05 - 0.15 * heavy), lp: lerp(1500, 8000, heavy) })) groanSyn(heavy);
      cool.groan = STRESS_COOL.groan * jitter(0.2);
    }
    if (rebound > 0.05 && cool.spring <= 0) {
      if (!shot('spring', 0.18 + 0.5 * rebound, { offset: Math.random() * 3.5, dur: 0.9 + 0.6 * rebound, rate: jitter(0.1) })) springSyn(rebound);
      cool.spring = STRESS_COOL.spring * jitter(0.2);
    }
    sx.bottomK = 0; sx.slamK = 0;
  }

  function winch(w, dt, t) {
    if (w.broken && !st.broken && !shot('winchSnap', 1.0)) snap();
    st.broken = !!w.broken;
    if (!w.attached) {
      st.len = null; st.pay = 0;
      g.creakGain.gain.setTargetAtTime(0, t, 0.04);
      return;
    }
    const load = w.tension / (w.maxPull || 1);             // 1 = motor stalled; rope breaks further up
    if (st.len !== null && w.reeling) {
      st.pay += Math.max(0, st.len - w.length);
      if (st.pay >= CLICK_M) { st.pay %= CLICK_M; click(clamp01(load)); }
    }
    st.len = w.length;
    const strain = clamp01((load - 0.2) / 0.9);
    st.creakT -= dt;
    if (st.creakT <= 0) {                                  // stick-slip: new pitch + grab/let-go every 50..210 ms
      st.creakT = 0.05 + Math.random() * 0.16;
      st.creakP = 0.8 + Math.random() * 0.5;
      st.creakA = Math.random() < 0.65 ? 1 : 0.15;
    }
    g.creakOsc.frequency.setTargetAtTime((55 + 70 * strain) * st.creakP, t, 0.02);
    g.creakBP.frequency.setTargetAtTime((650 + 900 * strain) * (0.9 + 0.2 * st.creakP), t, 0.03);
    g.creakGain.gain.setTargetAtTime(CREAK_GAIN * Math.pow(strain, 1.3) * st.creakA, t, 0.025);
  }

  const api = {
    get ctx() { return ctx; },
    get muted() { return muted; },
    get samples() { return smp; },
    get vehicle() { return { id: vid, voice: profVoice(), overrides: Object.keys((vid && ov[vid]) || {}),
      profile: prof ? { idle: prof.idle && prof.idle[0], rev: prof.rev && prof.rev[0], ready: !!(prof.idle && fileBuf(prof.idle[0])),
        revReady: !!(prof.rev && fileBuf(prof.rev[0])), crank: prof.crank, crankReady: !!fileBuf(prof.crank) } : null }; },
    /** Select the truck's engine voice (call per run): id = content.json vehicle id, cfg = its vehicle JSON. */
    setVehicle(id, cfg = {}) {
      vid = id || null; own = { ...(cfg.sound || {}) }; delete own.profile; delete own.files;
      voice = voiceFor(id, cfg); prof = profileFor(id, cfg);
      if (g) { loadOverrides(vid); loadProfile(); }
    },
    /** Debug: current gain of every running sample loop + synth engine / slip (console: offroad.audio.voices). */
    get voices() {
      const o = {}; for (const n in vox) o[n] = +vox[n].g.gain.value.toFixed(3);
      if (vox.idle) { o.idleRate = +vox.idle.src.playbackRate.value.toFixed(2); o.idleLp = Math.round(vox.idle.f.frequency.value); o.idleLoop = [+vox.idle.src.loopStart.toFixed(3), +vox.idle.src.loopEnd.toFixed(3)]; }
      if (vox.rev) { o.revRate = +vox.rev.src.playbackRate.value.toFixed(2); o.revLp = Math.round(vox.rev.f.frequency.value); }
      if (g) o.engSmp = +g.engSmp.gain.value.toFixed(3);
      o.clink = !!sx.clinkSrc;                                     // running heat-soak clinks playing
      if (g) { o.synthEngine = +g.engGain.gain.value.toFixed(3); o.synthSlip = +g.slipGain.gain.value.toFixed(3); }
      return o;
    },
    unlock() {
      if (!ctx) {
        const AC = globalThis.AudioContext || globalThis.webkitAudioContext;
        if (!AC) return false;
        try { ctx = new AC({ latencyHint: 'interactive' }); } catch (_) { return false; }
        if (typeof document !== 'undefined') document.addEventListener('visibilitychange', onVisibility);
      }
      if (!g) build();
      for (const n in buf) if (LOOP[n] && !vox[n]) startLoop(n);   // rebuilt after destroy(): restart decoded loops
      loadSamples();
      loadOverrides(vid);
      loadProfile();
      if (!injected && ctx.state === 'suspended') ctx.resume().catch(() => {});
      return true;
    },
    update(v, dt = 1 / 60, eng = {}) {
      if (!g || !(injected || ctx.state === 'running')) { stale = true; return; }
      if (stale) { api.reset(); stale = false; }
      const t = ctx.currentTime;
      engine(v, eng, t);
      tyres(v, t);
      shifts(v, dt, !muted);
      if (muted) { for (const n in vox) if (!ENG_LOOPS.includes(n)) vox[n].g.gain.setTargetAtTime(0, t, 0.05); return; }   // one-shots not worth scheduling
      surfaces(v, t, dt);
      impacts(v, dt);
      chassisStress(v, dt);
      winch(v.winch, dt, t);
      failures(v, t, dt);
    },
    /** HUD shifter hook: kind 'notch' (lever passes a gate) | 'commit' (range / drivetrain engaged) | 'drive' (4WD / locker toggle). */
    shift(kind = 'notch', info = {}) {
      if (!g || muted || !(injected || ctx.state === 'running')) return false;
      if (kind === 'drive' || kind === 'commit') return true;       // played from the vehicle state edges (shifts()), no double
      if (kind === 'key') { notch(0.45); return true; }             // ignition key: small detent tick (the starter follows)
      if (cool.notch > 0) return true;
      cool.notch = SHIFT_COOL;
      if (!shot('gear1', 0.7, { rate: jitter(0.06) })) lever(kind, info);
      return true;
    },
    reset() {
      for (const s of st.wheels) { s.c = null; s.peak = 0; s.vn = 0; s.cool = 0.15; }
      hitFx.busy = 0;
      st.len = null; st.pay = 0; st.bodyHit = true; st.bodyCool = 0.15;
      st.body = null; st.stalled = false; st.misfire = false; st.axle = [false, false];
      sx.wet.length = 0; sx.bodyWet = false; sx.hot = false;            // a teleport into water is not an entry splash
      for (const k in cool) cool[k] = Math.max(cool[k], 0.15);
      if (ctx && crankSrc) { stopShot(crankSrc, ctx.currentTime); crankSrc = null; }
      if (ctx && sx.cooldownSrc) { stopShot(sx.cooldownSrc, ctx.currentTime); sx.cooldownSrc = null; }
      if (ctx && sx.clinkSrc) { stopShot(sx.clinkSrc, ctx.currentTime); sx.clinkSrc = null; }
      sx.cranking = false; sx.catch = null; sx.on = false; sx.rpmS = null; sx.rpmPk = null; sx.thrS = null; sx.load = null; sx.susp.length = 0; sx.bottomK = 0; sx.slamK = 0; sx.thrLow = 0;
      gear = null;                                                      // new run / teleport: re-prime, no shift sounds
    },
    setMuted(m) {
      m = !!m;
      if (m === muted) return;
      muted = m;
      if (!g) return;
      g.master.gain.setTargetAtTime(muted ? 0 : volume, ctx.currentTime, 0.015);
      if (muted) { g.creakGain.gain.setTargetAtTime(0, ctx.currentTime, 0.02); g.crankGain.gain.setTargetAtTime(0, ctx.currentTime, 0.02); api.reset(); }
      if (muted) for (const n in vox) vox[n].g.gain.setTargetAtTime(0, ctx.currentTime, 0.02);
    },
    destroy() {
      if (typeof document !== 'undefined') document.removeEventListener('visibilitychange', onVisibility);
      if (g) for (const s of g.sources) { try { s.stop(); } catch (_) {} }
      if (crankSrc) { try { crankSrc.stop(); } catch (_) {} crankSrc = null; }
      for (const n in vox) delete vox[n];
      if (ctx && !injected && ctx.state !== 'closed') ctx.close().catch(() => {});
      g = null;
    },
  };
  return api;
}
