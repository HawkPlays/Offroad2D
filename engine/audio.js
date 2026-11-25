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
const HIT_MIN = 0.1;          // min wheel-hit strength 0..1 (≈ rate 3.9 travel/s or tyre closing speed 1.05 m/s)
// tyre STRIKE: a wheel contact's closing speed jumping (log / ledge face, rock lip) -- caught even when the shock barely moves
const STRIKE_VN = 1.2, STRIKE_RISE = 0.8, STRIKE_SPAN = 4, STRIKE_COOL = 0.3;   // m/s min, m/s rise vs last frame, m/s to full, s/wheel
const BODY_VN = 1.4, BODY_F = 9000;  // chassis bottom-out: min closing speed (m/s) or contact force (N)
const ROCK_VN = 1.6;          // loose boulder strike: min closing speed (m/s)
const HIT_COOL = { thud: 0.2, metal: 0.18, boulder: 0.2 };   // s between one-shots of one impact type
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
  fail1: '90s_laugh_fail.mp3', fail2: 'laugh_fail_a.mp3', fail3: 'laugh_fail_b.mp3', finish: 'level_finish.mp3',   // jingles (api.jingle)
};
const JINGLES = { fail: ['fail1', 'fail2', 'fail3'], finish: ['finish'] };
// continuous loops: filter type, cutoff f0 -> f1 (Hz), q, gain, `full` tread m/s for top level / rate, rate r0 + rk * k
const LOOP = {
  idle:    { type: 'lowpass',  f0: 700, f1: 7000, q: 0.7, gain: 0.55 },
  rev:     { type: 'lowpass',  f0: 700, f1: 9000, q: 0.7, gain: 0.6 },      // profile rev / load loop (engine voice 2)
  roar:    { type: 'lowpass',  f0: 1200, f1: 9500, q: 0.7, gain: 0.62 },    // profile held-gas roar loop (profile.roar)
  bank:    { type: 'lowpass',  f0: 600, f1: 9500, q: 0.7, gain: 0.6 },      // loop-bank voices (eb0..)
  stones:  { type: 'lowpass',  f0: 900, f1: 9000, q: 0.7, gain: 0.75, full: 10, r0: 0.8,  rk: 0.45 },
  gravel:  { type: 'lowpass',  f0: 700, f1: 7500, q: 0.7, gain: 0.70, full: 10, r0: 0.8,  rk: 0.45 },
  mud:     { type: 'bandpass', f0: 280, f1: 1700, q: 0.8, gain: 1.00, full: 6,  r0: 0.75, rk: 0.5 },
  wood:    { type: 'lowpass',  f0: 900, f1: 9000, q: 0.7, gain: 1.25, full: 6,  r0: 0.85, rk: 0.35 },   // bridge planks: loud (engine ducks)
  // wading takes are ~10 dB quieter than the gravel / wood loops (-29 dB mean): round 12 'can't hear the water' (Bogger) -> x2+
  shallow: { type: 'lowpass',  f0: 1400, f1: 7000, q: 0.6, gain: 1.9, full: 6,  r0: 0.85, rk: 0.35 },
  deep:    { type: 'lowpass',  f0: 700, f1: 3600, q: 0.6, gain: 2.3, full: 5,  r0: 0.85, rk: 0.3 },
  scrape:  { type: 'bandpass', f0: 900, f1: 4200, q: 0.6, gain: 1.10, full: 4,  r0: 0.85, rk: 0.35 },
};
const SURF_CATS = ['stones', 'gravel', 'mud', 'wood'];
const WOOD_DUCK = 0.45, WATER_DUCK = 0.3;   // engine steps back on bridge planks / while wading so the surface carries
const WET_HOLD = 0.6, WET_DRY = 1.0;   // tyre water hysteresis (s): stays 'in' through hops / stones; splash only after being dry this long
const ENG_LOOPS = ['idle', 'rev', 'roar'];   // engine recordings: routed through the engine sample sub-bus (g.engSmp)
const BANK_MAX = 6;                  // loop-bank voices eb0..eb5 (also engine recordings)
for (let i = 0; i < BANK_MAX; i++) { ENG_LOOPS.push('eb' + i); }
// engine warmth 0..1 (state-only): idle WARM_IDLE /s + WARM_WORK /s x gas x rpm (a hard drive ~45 s to 0.6), cools WARM_COOL /s off.
// Running heat-soak clinks need warmth > CLINK_HEAT (or a real overheat) and CLINK_RUN s since the engine caught; low rev only.
// Shutdown / stall ticking needs warmth > TICK_HEAT (a fresh crank-and-stop stays quiet).
const WARM_IDLE = 0.0015, WARM_WORK = 0.016, WARM_COOL = 0.004;
const CLINK_HEAT = 0.6, CLINK_RN = 0.22, CLINK_RUN = 40, TICK_HEAT = 0.25;
const CATCH_XF = 0.15;
// engine run-down (no more 4 s misfire take on a stall): on the running -> off edge the loops keep playing while the
// audible rpm sags to DIE_RPM x idle and the level fades; a stall sputters (a few chopped dips + exhaust pops) over
// DIE_STALL s, a key-off winds down cleanly over DIE_KEY s
const DIE_STALL = 1.5, DIE_KEY = 0.75, DIE_RPM = 0.3;        // s: equal-power crank -> idle crossfade as the engine catches (also any engine-on edge)
const RPM_TAU = 0.08, THR_UP = 0.05, THR_DN = 0.12, LOAD_UP = 0.07, LOAD_DN = 0.18;   // s: JS-side smoothing (no warble)
// held gas: physics rpm swings ~1 Hz (wheel hop spins up, tyre catch drags down: 2400 <-> 5800 on the V8s); heard raw that is
// a rhythmic wah. Audible rpm falls slowly while gas is held (RPM_FALL_GAS) and leans on a decaying peak (PEAK_TAU, PEAK_HOLD).
// Pedal HELD steady (pedal activity = |gas - its PEDAL_TAU average|, x4) the roar is near-constant: rpm rises over HELD_UP, falls
// over HELD_DN, the peak decays over HELD_PK_TAU and weighs HELD_PK -- only moving the pedal (or lifting) revs it quickly.
const RPM_FALL_GAS = 0.55, PEAK_TAU = 1.4, PEAK_HOLD = 0.7;
// pedal intent (sound only): physics rpm is tied to wheel speed once rolling, so the pedal barely moved the pitch (only the
// filter opened). Audible rpm keeps INTENT_LIFT of the above-idle rpm off gas and adds up to INTENT_PUSH x (redline - idle)
// x gas of headroom (converter slip under load): sliding the pedal sweeps the note up / down like a real engine.
const INTENT_LIFT = 0.6, INTENT_PUSH = 0.28;
const FLARE_FLOOR = 0.55;      // audible rpm >= idle + FLARE_FLOOR x gas^2 x (redline - idle) while running
const PEDAL_TAU = 0.5, HELD_UP = 0.6, HELD_DN = 1.6, HELD_PK_TAU = 3, HELD_PK = 0.95;
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
  // v2: starter engages 0.08, pulses every ~0.1 s (loop trough -> trough), fires 1.97; the tail is still loud at the file end
  // so it fades over `release` s under the incoming idle loop. start = first-play offset (before the loop section)
  'v8_plymouth_barcuda_crank_v2.mp3': { start: 0.08, loop: [0.38, 1.84], catch: [1.95, 2.77], release: 0.4 },
};
// Backfire / overrun pops (new_rev_bf.mp3): [file, [[a, b] s, ...]] -- on a lift-off from high rpm, 1-3 random takes
const BF_POPS = ['new_rev_bf.mp3', [[1.84, 2.0], [2.03, 2.2], [2.76, 2.93], [3.03, 3.2], [4.78, 4.96], [5.35, 5.55]]];
export const ENGINE_PROFILES = {
  // rpm-indexed loop bank (no baked-in revs repeat): each entry [file, a, b, f0 Hz (, f1 Hz: swept -> pitch-flattened)];
  // the audible rpm picks the two entries around its note and crossfades them (equal power, log pitch), each played at
  // note / f0. new_rev.mp3: idle 0-0.75 s, then rev sweeps -- only the turning points (steady pitch) + one flattened stretch
  rally:     { bank: [['new_rev.mp3', 0.08, 0.7, 39.6], ['new_rev.mp3', 8.8, 9.05, 78.5], ['new_rev.mp3', 3.2, 3.44, 90.5],
                      ['new_rev.mp3', 4.9, 5.2, 149, 135], ['new_rev.mp3', 4.1, 4.33, 196]],
               pitchExp: 0.87, pops: BF_POPS, crank: 'engine_crank.mp3',
               voice: { rate: 1, rpmExp: 0.87, maxRate: 1.5, lp: 1.2, q: 0.8, gain: 1.25, synth: 0.45, sub: 1.2, floor: 0.98, lpIdle: 0.4 } },   // sport 4-cyl buggy: idle floor 0.5 -> 0.98 (~+6 dB) + brighter idle filter (lpIdle), roar unchanged
  jeep:      { idle: ['thar_jeep_driving.mp3', 16.4, 19.8], rev: ['thar_jeep_driving.mp3', 3.0, 6.2], revPitch: 1.4,
               crank: 'engine_crank.mp3', voice: { rate: 1, rpmExp: 0.5, maxRate: 2.2, lp: 1.1, q: 0.8, gain: 1.15, synth: 0.6 } },  // utility jeep
  charger:   { idle: ['v8_dodge_chrgr_idle.mp3', 5.6, 9.4], rev: ['v8_plymth_brcuda_rev.mp3', 0.1, 2.4], revPitch: 1.52, revGain: 0.7, revLoop: false,  // 48 / 73 Hz
               // (its own rev file is a string of blips -- looped under held gas it pulsed; it stays as the stab flare)
               flare: ['v8_dodge_chrgr_rev.mp3', 1.5, 2.45], crank: 'v8_plymouth_barcuda_crank_v2.mp3', pops: BF_POPS,
               // held full gas: the V8 roar loop pitched DOWN (big-block, slower firing) and pushed hard -- the idle-solo alone
               // sounded thin for a 2.6 t mud truck at full rpm (playtest r7)
               roar: ['v8_plymth_brcuda_roar.mp3', 0.03, 0.5], roarRate: [0.8, 0.9], roarXf: 0.07, roarGain: 1.5,
               voice: { rate: 1, rpmExp: 0.6, maxRate: 2.6, lp: 0.9, q: 1.0, gain: 1.05, synth: 0.9, sub: 1.6 } },  // heavy V8 bogger
  barracuda: { idle: ['v8_plymth_brcuda_idle.mp3', 0.5, 7.5], rev: ['v8_plymth_brcuda_rev.mp3', 0.1, 2.4], revPitch: 1.0, revLoop: false,  // 74 / 73 Hz
               // roar: floored gas at high rpm loops the bright top of the rev take (2.92–3.58 s: centroid ~1 kHz vs ~300 Hz in
               // the 0.1–2.4 s stretch -- the 'highest' note, ~3rd–4th second) at ~native rate, so it holds while the pedal does
               // roar: held floored gas loops v8_plymth_brcuda_roar.mp3 = the two loud, steady stretches of the player-cut
               // rev2 take (0.63-0.92 + 1.40-1.67 s, the -20 dB flutter at ~1.3 s skipped, crossfaded + levelled), ~native rate.
               // A gas stab plays rev2's build-up (0-0.66 s) as the flare intro; the loop takes over as it ends (roarIntro)
               roar: ['v8_plymth_brcuda_roar.mp3', 0.03, 0.5], roarRate: [0.97, 1.03], roarXf: 0.07, roarIntro: true,
               flare: ['v8_plymth_brcuda_rev2.mp3', 0.0, 0.66], crank: 'v8_plymouth_barcuda_crank_v2.mp3', pops: BF_POPS,
               voice: { rate: 1, rpmExp: 0.55, maxRate: 2.3, lp: 1.0, q: 0.9, gain: 0.95, synth: 0.7 } },
};
/** Legacy single-file inline profile {file, idle: [a, b], drive?, driveRef?, rev?} -> split-section form. */
function normProfile(p) {
  if (!p || !p.file || Array.isArray(p.idle) && typeof p.idle[0] === 'string') return p;
  const f = p.file, sec = s => (s ? [f, s[0], s[1]] : null);
  return { ...p, idle: sec(p.idle), rev: sec(p.drive), revPitch: p.revPitch ?? 1 + (p.driveRef ?? 0.35) * 2, flare: sec(p.rev) };
}
// crawler03 (sport buggy) = the rally loop bank; barracuda = the heavy V8 Truggy (crawler04).
export const VEHICLE_PROFILE = { crawler01: 'jeep', crawler02: 'charger', crawler03: 'rally', crawler04: 'barracuda' };
VOICES.crawler04 = { rate: 0.85, rpmExp: 0.5, maxRate: 2.2, lp: 0.8, q: 1.3, gain: 1.1, synth: 1.2, order: 4, synthLp: 0.85, sub: 1.5, crank: 0.9, misfire: 0.85 };
/** Engine profile for a truck: sound.profile (name / inline object / null = none) wins over the per-id default. */
export function profileFor(id, cfg = {}) {
  const p = cfg.sound && 'profile' in cfg.sound ? cfg.sound.profile : VEHICLE_PROFILE[id];
  return typeof p === 'string' ? ENGINE_PROFILES[p] || null : p && typeof p === 'object' && (p.file || p.idle || p.bank) ? normProfile(p) : null;
}

/**
 * Constant-pitch copy of a swept section [a, b] s of buf whose fundamental glides fa -> fb Hz (linear): resampled with a
 * time-varying read speed (fc / f(t), fc = sqrt(fa fb)) so the whole slice sits at fc. Lets a short stretch of a rev
 * sweep loop without a periodic pitch dip. Returns { b: AudioBuffer, f: fc }.
 */
export function flattenSweep(ctx, buf, a, b, fa, fb) {
  const sr = buf.sampleRate, nch = buf.numberOfChannels, s0 = Math.round(a * sr), s1 = Math.min(buf.length - 1, Math.round(b * sr));
  const fc = Math.sqrt(fa * fb), L = s1 - s0, pos = [];
  for (let p = 0; p < L - 1;) { pos.push(p); const f = fa + (fb - fa) * (p / L); p += fc / f; }
  const out = ctx.createBuffer(nch, pos.length, sr);
  for (let c = 0; c < nch; c++) {
    const d = buf.getChannelData(c), o = out.getChannelData(c);
    for (let i = 0; i < pos.length; i++) { const p = s0 + pos[i], k = Math.floor(p), t = p - k; o[i] = d[k] * (1 - t) + d[k + 1] * t; }
  }
  return { b: out, f: fc };
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

/**
 * Texture loop: a short steady engine stretch [a, b] s (pitch f0 Hz) re-stitched into a `dur` s loop of random grains, each
 * starting on a firing-period boundary (a + k/f0: phase-aligned joins) and crossfaded (equal power, >= one period) into the
 * next, gain jittered +-1.5 dB, wrapped seamlessly. A 0.25 s slice looped as-is repeats ~4x a second (the rally bank 'rev
 * that keeps repeating', most audible while held on part gas); this has no period shorter than `dur`. Deterministic (seed).
 */
export function textureLoop(ctx, buf, a, b, f0, dur = 3, seed = 1) {
  const sr = buf.sampleRate, nch = buf.numberOfChannels, chs = [];
  for (let c = 0; c < nch; c++) chs.push(buf.getChannelData(c));
  const P = sr / f0, s0 = Math.round(a * sr), s1 = Math.min(buf.length, Math.round(b * sr)), span = s1 - s0;
  if (!(span > sr * 0.12) || !(P > 1)) return null;
  const X = Math.max(Math.round(sr * 0.018), Math.round(P)), N = Math.round(dur * sr);
  let r = seed * 9301 + 49297;
  const rnd = () => { r = (r * 9301 + 49297) % 233280; return r / 233280; };
  const out = ctx.createBuffer(nch, N, sr), O = []; for (let c = 0; c < nch; c++) O.push(out.getChannelData(c));
  let pos = 0;
  while (pos < N) {
    const L = Math.min(Math.max(2 * X, Math.round(P * (3 + Math.floor(rnd() * 6)))), Math.round(span - X - P));
    if (L < X + 2) return null;
    const st = s0 + Math.round(P * Math.floor(rnd() * Math.max(1, Math.floor((span - L - X) / P)))), gn = Math.pow(10, (rnd() - 0.5) * 3 / 20);
    for (let i = 0; i < L + X; i++) {             // fade in over X, hold, fade out over the X that overlaps the next grain
      const w = i < X ? Math.sin((i + 0.5) / X * Math.PI / 2) : i >= L ? Math.cos((i - L + 0.5) / X * Math.PI / 2) : 1, o = (pos + i) % N;
      for (let c = 0; c < nch; c++) O[c][o] += chs[c][st + i] * w * gn;
    }
    pos += L;
  }
  return { b: out, loopStart: 0, loopEnd: N / sr };
}

const SLIP_OF_CAT = { stones: 'rock', gravel: 'dirt', mud: 'mud', wood: 'wood', water: 'water' };
const SOLO_MAX = 2.0, SOLO_SYNTH = 1.7, BLIP_COOL = 0.6;   // rev loop off: idle pitch cap, synth growl x; s between stab blips
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

/** HORN (held): per-rig synth voice (`HORNS`, picked by setVehicle from cfg.sound.horn || cfg.look.style): reeds (`f` Hz,
 *  `type` wave, `mix` per-reed gains, default equal) -> soft clip (tanh `drive`) -> band (highpass `hp`, peaking `pk` [Hz, dB, Q?],
 *  lowpass `lp`); pitch settles from `flat` below in `tau` s (diaphragm / air-valve onset), attack `att` s to `gain`, slight sag to
 *  `hold`·gain; optional `rasp` [depth, Hz] = amplitude flutter (a blatting air reed: roughness, not a note), `wob` [cents, Hz]
 *  = slow pitch drift, `air` [level, Hz, Q] = valve hiss (band-passed noise); release fades in ~25 ms. World bus.
 *  Round 12: Bogger was a major triad (read as an instrument) -> two out-of-tune low reeds + rasp + air; Buggy too thin ->
 *  lower, buzzier disc horn with a body peak. Round 13: that Bogger was 'boring' (no note) -> a real dual-trumpet air horn:
 *  E3 + G3 (minor third: a tone, not a chord) + a slightly sharp E chorus reed, less rasp / drift / hiss; same phone level. */
export const HORNS = {
  jeep:   { f: [392, 494], type: 'square', drive: 1.6, hp: 300, pk: [1700, 4], lp: 2900, gain: 0.14, hold: 0.85, att: 0.01, flat: 0.05, tau: 0.012 },   // honest utility twin, a little nasal
  bogger: { f: [165, 196, 166.5], mix: [0.5, 0.45, 0.25], type: 'sawtooth', drive: 3, hp: 120, pk: [720, 5, 0.9], lp: 2800, gain: 0.2, hold: 0.88,
            att: 0.05, flat: 0.08, tau: 0.05, rasp: [0.2, 31], wob: [6, 0.5], air: [0.03, 1600, 0.9] },   // dual-trumpet truck air horn: minor third + a chorus reed, light grit
  buggy:  { f: [415, 440], mix: [0.6, 0.45], type: 'sawtooth', drive: 3.2, hp: 170, pk: [900, 5, 0.9], lp: 3800, gain: 0.16, hold: 0.82,
            att: 0.006, flat: 0.03, tau: 0.008, rasp: [0.15, 52] },   // buzzy little disc horn: 'meep', with body
  truggy: { f: [345, 432], type: 'sawtooth', drive: 2.2, hp: 230, pk: [1400, 5], lp: 3200, gain: 0.2, hold: 0.8, att: 0.012, flat: 0.06, tau: 0.015 },   // brassy American twin
  wedge:  { f: [415, 523], type: 'sawtooth', drive: 1.1, hp: 260, pk: [2100, 3], lp: 4600, gain: 0.15, hold: 0.85, att: 0.01, flat: 0.025, tau: 0.01 },   // refined Euro twin, clean
};
const NOISE = new WeakMap();
/** Builds one held horn voice on `ac` into `dest`; returns {out, stop(now)}. Pure Web Audio (also runs in an OfflineAudioContext). */
export function hornGraph(ac, H, dest, d = 1) {
  const t = ac.currentTime, out = ac.createGain(), sh = ac.createWaveShaper(), hp = ac.createBiquadFilter(), pk = ac.createBiquadFilter(), lp = ac.createBiquadFilter();
  const mixG = ac.createGain();
  const curve = new Float32Array(1024);
  for (let i = 0; i < curve.length; i++) curve[i] = Math.tanh(H.drive * (i / 511.5 - 1)) / Math.tanh(H.drive);
  sh.curve = curve;
  hp.type = 'highpass'; hp.frequency.value = H.hp; hp.Q.value = 0.6;
  pk.type = 'peaking'; pk.frequency.value = H.pk[0]; pk.Q.value = H.pk[2] || 1; pk.gain.value = H.pk[1];
  lp.type = 'lowpass'; lp.frequency.value = H.lp; lp.Q.value = 0.7;
  const srcs = [], nodes = [sh, hp, pk, lp, out, mixG];
  let wob = null;
  if (H.wob) {                                          // slow pitch drift shared by the reeds (cents)
    wob = ac.createOscillator(); const wg = ac.createGain();
    wob.frequency.value = H.wob[1]; wg.gain.value = H.wob[0]; wob.connect(wg); wob.start(t); srcs.push(wob); nodes.push(wg); wob = wg;
  }
  H.f.forEach((f0, i) => {
    const f = f0 * d, o = ac.createOscillator(), og = ac.createGain();
    o.type = H.type; o.frequency.setValueAtTime(f * (1 - H.flat), t); o.frequency.setTargetAtTime(f, t, H.tau);
    if (wob) wob.connect(o.detune);
    og.gain.value = H.mix ? H.mix[i] : 1 / H.f.length; o.connect(og); og.connect(mixG); o.start(t);
    srcs.push(o); nodes.push(og);
  });
  mixG.connect(sh);
  let tail = sh;
  if (H.rasp) {                                         // amplitude flutter after the clipper: 1 - depth .. 1
    const am = ac.createGain(), lfo = ac.createOscillator(), lg = ac.createGain();
    am.gain.value = 1 - H.rasp[0] / 2; lfo.type = 'triangle'; lfo.frequency.value = H.rasp[1] * d; lg.gain.value = H.rasp[0] / 2;
    lfo.connect(lg); lg.connect(am.gain); lfo.start(t); sh.connect(am); tail = am; srcs.push(lfo); nodes.push(am, lg);
  }
  tail.connect(hp); hp.connect(pk); pk.connect(lp); lp.connect(out); out.connect(dest);
  if (H.air) {                                          // valve hiss, band-passed noise straight into the band filters
    let buf = NOISE.get(ac);
    if (!buf) { buf = ac.createBuffer(1, ac.sampleRate, ac.sampleRate); const c = buf.getChannelData(0); for (let i = 0; i < c.length; i++) c[i] = Math.random() * 2 - 1; NOISE.set(ac, buf); }
    const n = ac.createBufferSource(), bp = ac.createBiquadFilter(), ng = ac.createGain();
    n.buffer = buf; n.loop = true; bp.type = 'bandpass'; bp.frequency.value = H.air[1]; bp.Q.value = H.air[2]; ng.gain.value = H.air[0];
    n.connect(bp); bp.connect(ng); ng.connect(pk); n.start(t); srcs.push(n); nodes.push(bp, ng);
  }
  out.gain.setValueAtTime(0, t); out.gain.linearRampToValueAtTime(H.gain, t + H.att); out.gain.setTargetAtTime(H.gain * H.hold, t + H.att + 0.02, 0.12);
  return {
    out,
    stop(now = false, fast = false) {                     // fast: a swipe that started on the cap -> near-silent cut (~20 ms)
      const t1 = ac.currentTime, end = now ? t1 : t1 + 0.2;
      out.gain.cancelScheduledValues(t1); out.gain.setValueAtTime(out.gain.value, t1); out.gain.setTargetAtTime(0, t1, fast ? 0.006 : 0.025);
      srcs.forEach((o, i) => { try { o.stop(end); } catch (_) { /* already stopped */ } if (!i) o.onended = () => { for (const x of nodes) x.disconnect(); }; });
    },
  };
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
  const vol = { master: 1, engine: 1, world: 1, ui: 1 };                 // MENU -> SOUND sliders (0..1), setVolumes()
  /** Route the one-shots fn() starts to a volume group ('busEng' | 'busUi'; default world). */
  const via = (bus, fn) => { if (!g) return fn(); const o = g.bus; g.bus = g[bus] || g.busWorld; try { return fn(); } finally { g.bus = o; } };
  let ctx = opts.context || null, g = null, muted = !!opts.muted, stale = true;
  const st = { wheels: [], bodyHit: false, bodyCool: 0, len: null, pay: 0, broken: false, creakT: 0, creakP: 1, creakA: 1,
              stalled: false, misfire: false, axle: [false, false], body: null };
  const mix = { lo: 0, hi: 0, q: 0, pink: 0, brown: 0, gain: 0 };
  // samples: buf[name] = decoded AudioBuffer (loops already seamless), vox[name] = running loop voice {src, f, g}
  const buf = {}, vox = {}, smp = { loaded: [], failed: [], total: Object.keys(SFX).length };
  const cool = { metal: 0, boulder: 0, splash: 0, thud: 0, misfire: 0, notch: 0, grind: 0, tick2: 0, squeak: 0, groan: 0, spring: 0, rev: 0, tick: 0, clink: 0, pop: 0 };
  let gear = null;                                         // last {fourWD, lockF, lockR, rev, range, pend}; null = prime silently
  const hitFx = { busy: 0, metal: '' };                    // recent-hit density, last metal clip (alternate takes)
  const acc = {};                                          // per-category {n, drive, slip} scratch
  for (const c of [...SURF_CATS, 'water']) acc[c] = { n: 0, drive: 0, slip: 0 };
  let loading = false, shots = 0, crankSrc = null;
  let vid = null, voice = { ...VOICE_DEFAULT }, own = {};          // current truck id, voice, explicit sound {} keys
  const ov = {};                                                  // ov[vehicleId][idle|crank|misfire] = override buffer
  let prof = null;                                                // current truck's ENGINE_PROFILES entry (or null)
  const files = {}, slices = {};                                  // profile files: decoded buffer | 'loading' | 'failed'; loop slices
  let eo = { rev: true, blip: true };                             // engine options: rev loop (profile revLoop, set per truck), stab blip
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
    const master = gain(muted ? 0 : volume * vol.master, ctx.destination);
    const limiter = ctx.createDynamicsCompressor();
    limiter.threshold.value = -4; limiter.knee.value = 4; limiter.ratio.value = 12;
    limiter.attack.value = 0.002; limiter.release.value = 0.15;
    limiter.connect(master);
    const stage = gain(1, limiter);                                // showroom parade: distance fade of the whole mix (stageFade)
    const bus = gain(1, stage);
    // MENU -> SOUND volume groups (setVolumes): engine (engine voices, starter, pops / coughs), world (terrain, water, impacts,
    // chassis, winch: the default route), ui (shifter, button ticks). One-shots go to g.bus = whichever group via() selected.
    const busEng = gain(vol.engine, bus), busWorld = gain(vol.world, bus), busUi = gain(vol.ui, bus);
    const pink = noiseBuffer(ctx, 'pink'), brown = noiseBuffer(ctx, 'brown');

    // engine: both oscillators go through the lowpass (never straight to the bus); every voice starts silent
    const f0 = 800 / 60 * ENGINE_ORDER;
    // engDuck: the whole engine (synth + recorded loops) dips while the tyres roll over bridge planks, so the wood is heard
    const engDuck = gain(1, busEng);
    const engDie = gain(1, engDuck);                               // run-down fade when the engine dies / is keyed off (DIE_*)
    const engGain = gain(0, engDie);
    const engLP = filter('lowpass', LP_MIN, 1.3, engGain);
    const oscA = osc('sawtooth', f0, -14, gain(0.5, engLP));
    const oscB = osc('triangle', f0, 17, gain(0.65, engLP));
    const subGain = gain(0.28, engGain), sub = osc('sine', f0 / 2, 0, subGain);
    const engSmp = gain(0, engDie);                               // recorded idle / rev loops: on/off + crank -> idle fade
    sx.engOn = false;                                              // fresh gate is closed: next running frame fades it in

    // tyre slip
    const slipGain = gain(0, busWorld);
    const slipBP = filter('bandpass', 600, 1, slipGain);
    const pinkMix = gain(0, slipBP), brownMix = gain(0, slipBP);
    const noiseSrc = [loop(pink, pinkMix), loop(brown, brownMix)];

    // winch cable creak: sawtooth "stick-slip" source ringing a narrow bandpass
    const creakGain = gain(0, busWorld);
    const creakBP = filter('bandpass', 700, 10, creakGain);
    const creakOsc = osc('sawtooth', 60, 0, creakBP);

    // starter motor: buzzy sawtooth chopped by an 11 Hz square LFO ("rur-rur-rur") -> lowpass; silent unless cranking
    const crankGain = gain(0, busEng);
    const crankAM = gain(0.5, filter('lowpass', 900, 0.9, crankGain));
    const crankOsc = osc('sawtooth', 95, 0, crankAM);
    const crankLfo = osc('square', 11, 0, gain(0.5, crankAM.gain));

    g = { master, limiter, stage, mix: bus, bus: busWorld, busEng, busWorld, busUi, pink, brown, engDuck, engDie, engGain, engSmp, engLP, oscA, oscB, sub, subGain, slipGain, slipBP, pinkMix, brownMix, creakGain, creakBP, creakOsc,
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
  function slice(sec, xf = 0.22) {
    const b = sec && fileBuf(sec[0]);
    if (!b) return null;
    const k = sec.join('|') + '|' + xf;
    return k in slices ? slices[k] : (slices[k] = sliceLoop(ctx, b, sec[1], sec[2], xf));   // long wrap xfade: no loop 'tick'
  }
  function loadProfile() {
    if (!prof) return;
    for (const k of ['idle', 'rev', 'flare', 'roar']) if (prof[k]) loadFile(prof[k][0]);
    for (const e of prof.bank || []) loadFile(e[0]);
    if (prof.pops) loadFile(prof.pops[0]);
    loadFile(prof.crank);
  }
  /** Loop bank of the profile: [{ b, sl, f }] per entry (swept entries pitch-flattened first), null until every file decoded. */
  function bankSlices() {
    const B = prof && prof.bank;
    if (!B) return null;
    const k = 'bank|' + B.map(e => e.join(',')).join('|');
    if (k in slices) return slices[k];
    if (!B.every(e => fileBuf(e[0]))) return null;
    // each entry -> a 3 s grain-stitched texture loop (textureLoop): the raw 0.23-0.6 s slices repeated audibly ~2-4x a second
    return (slices[k] = B.map(([f, a, b, f0, f1], n) => {
      if (f1 == null) return { ...(textureLoop(ctx, fileBuf(f), a, b, f0, 3, n + 1) || sliceLoop(ctx, fileBuf(f), a, b, 0.06)), f: f0 };
      const fl = flattenSweep(ctx, fileBuf(f), a, b, f0, f1), sl = textureLoop(ctx, fl.b, 0, fl.b.duration, fl.f, 3, n + 1) || sliceLoop(ctx, fl.b, 0, fl.b.duration, 0.06);
      return { ...sl, f: fl.f };
    }).filter(e => e && e.b).sort((x, y) => x.f - y.f));
  }
  /** Profile voicing while its recording plays (sound {} keys still win). */
  const profVoice = () => (prof && (prof.idle ? fileBuf(prof.idle[0]) : prof.bank && bankSlices()) ? { ...voice, ...(prof.voice || {}), ...own } : voice);
  /** Engine sample for the current truck: its override if loaded, else the shared one. */
  const engBuf = name => (vid && ov[vid] && ov[vid][name]) || buf[name];
  /** Starter: { b, loop?: slice, start?, catch?: [a, b], release? } -- override > profile crank file > shared engine_crank. */
  function crankBuf() {
    if (vid && ov[vid] && ov[vid].crank) return { b: ov[vid].crank };
    const f = prof && fileBuf(prof.crank) ? prof.crank : buf.crank ? SFX.crank : null;
    if (!f) return null;
    const b = f === SFX.crank && !fileBuf(f) ? buf.crank : fileBuf(f), c = CRANKS[f];
    if (!c) return { b };
    const k = f + '|crank';
    if (!(k in slices)) slices[k] = sliceLoop(ctx, b, c.loop[0], c.loop[1], 0.05);
    return { b, loop: slices[k], start: c.start || 0, catch: c.catch, release: c.release };
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
    const c = LOOP[name] || LOOP.bank, src = ctx.createBufferSource(), f = ctx.createBiquadFilter(), gn = ctx.createGain();
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
  function shot(name, peak, { rate = 1, offset = 0, dur = 0, loop = false, lp = 0, delay = 0, sl = null, attack = 0, release = 0.12 } = {}) {
    const b = typeof name !== 'string' ? name : ENGINE_SFX.includes(name) ? engBuf(name) : buf[name];
    if (!b || shots >= MAX_SHOTS) return null;
    const t = ctx.currentTime + delay, src = ctx.createBufferSource(), gn = ctx.createGain();
    src.buffer = b; src.playbackRate.value = rate; src.loop = loop;
    if (loop && sl) { src.loopStart = sl.loopStart; src.loopEnd = sl.loopEnd; if (!offset) offset = sl.loopStart; }
    gn.gain.value = peak;
    if (attack > 0) { gn.gain.setValueAtTime(0, t); gn.gain.linearRampToValueAtTime(peak, t + attack); }
    if (dur > 0) {                                         // end fade: linear 120 ms, or a smooth exponential tail over a longer release
      const r = Math.min(release, dur), t0 = t + Math.max(attack, dur - r);
      gn.gain.setValueAtTime(peak, t0);
      if (r > 0.12) gn.gain.setTargetAtTime(0.0001, t0, r / 4); else gn.gain.linearRampToValueAtTime(0.0001, t + dur);
    }
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
    const peak = hitGain(k, 0.16, 0.85) * gain, att = 0.014 - 0.008 * k;
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
  /** Tyre hit by material (over tyreThud's rubber body): wood = hollow knock, stone = hard thock + grit tick, gravel = crunch,
   *  mud = wet squelch (duller), dirt = soft puff. k 0..1 = hardness / closing speed; gain = busy duck. Water: splash elsewhere. */
  function tyreHit(k, surface, gain = 1) {
    const sc = surfaceCat(surface), cat = !sc || (sc === 'gravel' && !/gravel|scree|grit|shingle/.test(surface)) ? 'dirt' : sc, t = ctx.currentTime, pk = hitGain(k, 0.12, 0.75) * gain;
    if (cat === 'water') return;
    tyreThud(cat === 'mud' ? 0.8 * k : k, gain * (cat === 'mud' ? 0.8 : 1));
    if (cat === 'wood') {                                  // log: two hollow resonances + a short triangle knock
      const f = (170 + 70 * k) * jitter(0.08), o = ctx.createOscillator(), a = ctx.createGain(), len = 0.09 + 0.07 * k;
      o.type = 'triangle'; o.frequency.setValueAtTime(f, t); o.frequency.exponentialRampToValueAtTime(f * 0.8, t + len);
      env(a.gain, t, 0.85 * pk, 0.003, len); o.connect(a); a.connect(g.bus); o.start(t); o.stop(t + len + 0.05);
      o.onended = () => a.disconnect();
      noiseBurst(g.pink, t, 0.07 + 0.05 * k, 2.3 * pk, [['bandpass', f * 2.3, 6], ['lowpass', 1400, 0.5]]);
    } else if (cat === 'stones') {                         // rock: hard low thock + a bright grit tick; boulder take when hard
      noiseBurst(g.brown, t, 0.06 + 0.04 * k, 0.9 * pk, [['bandpass', (300 + 200 * k) * jitter(0.1), 1.6]]);
      noiseBurst(g.pink, t + 0.004, 0.03 + 0.02 * k, 0.9 * pk, [['bandpass', (1500 + 1200 * k) * jitter(0.15), 2.5]]);
      if (k > 0.55 && cool.boulder <= 0 && hitShot('boulder', clamp01((k - 0.45) * 1.4), 0.08, 0.5)) cool.boulder = HIT_COOL.boulder * jitter(0.15);
    } else if (cat === 'gravel') {                         // gravel: short crunchy spray of stones
      noiseBurst(g.pink, t, 0.08 + 0.06 * k, 1.1 * pk, [['bandpass', (1800 + 1400 * k) * jitter(0.15), 1.1], ['highpass', 700, 0.5]]);
    } else if (cat === 'mud') {                            // mud: dull wet slap, a falling low squelch
      noiseBurst(g.brown, t, 0.12 + 0.08 * k, 1.1 * pk, [['lowpass', (420 + 250 * k) * jitter(0.1), 1.2]]);
      noiseBurst(g.pink, t + 0.02, 0.06, 0.35 * pk, [['bandpass', (700 + 300 * k) * jitter(0.12), 3]]);
    } else {                                               // dirt: soft dusty puff
      noiseBurst(g.brown, t, 0.08 + 0.05 * k, 0.7 * pk, [['lowpass', (600 + 400 * k) * jitter(0.1), 0.7]]);
    }
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
  let hornV = null, hornCfg = HORNS.truggy;
  function hornOn() {
    if (hornV) return;
    hornV = hornGraph(ctx, hornCfg, g.busWorld, jitter(0.01));
  }
  function hornOff(now = false, fast = false) {
    const h = hornV;
    if (!h) return;
    hornV = null;
    h.stop(now, fast);
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
  /** Drivetrain controls (state edges, ONE sound per change, never per HUD slot): gear engaged (gate / range / direction) =
   *  gear2 (into H / up) or gear3 (heavy: H -> low, into L / L-, R, or a waiting shift completing); in / out of N only = light
   *  gear1; a slot requested while moving (pendingRange) = one grind per request; 4WD / locks = gear1. A new shift sound stops the
   *  previous one (fast re-shifts never stack). HUD drags add only quiet ticks (shift('tick')). */
  function shifts(v, dt, play) {
    cool.notch -= dt; cool.grind -= dt; cool.tick2 -= dt;
    const gate = v.gate || (v.gear === 'R' ? 'R' : v.range || 'H');
    const now = { fourWD: !!v.fourWD, lockF: !!v.lockF, lockR: !!v.lockR, gate, rev: gate === 'R',
                  range: v.range || 'H', pend: v.pendingRange || null };
    const was = gear; gear = now;
    if (!was || !play) return;
    const one = (name, peak, o) => { if (sx.shiftSrc) stopShot(sx.shiftSrc, ctx.currentTime); sx.shiftSrc = shot(name, peak, o); return sx.shiftSrc; };
    if (now.gate !== was.gate) {
      const neutralOnly = (now.gate === 'N' || was.gate === 'N') && now.range === was.range;
      if (neutralOnly) { if (!one('gear1', 0.75, { rate: jitter(0.05) })) notch(0.8); cool.notch = Math.max(cool.notch, 0.15); return; }
      const pending = was.pend === now.gate, low = now.range === 'L' || now.range === 'L-' || now.rev || (was.range === 'H' && now.range !== 'H');
      const heavy = pending || low, rate = (SHIFT_RATE[now.range] || 1) * jitter(0.03), k = heavy ? 1 : 0.85;
      if (!one(heavy ? 'gear3' : 'gear2', 0.95 * k, { rate }) && !(heavy && one('gear2', 0.95, { rate: rate * 0.92 }))) clunk(k, rate);
      cool.notch = Math.max(cool.notch, 0.2);                // the clunk already carries the lever
      return;
    }
    if (now.pend && now.pend !== was.pend) {                 // requested while moving > SHIFT_SPEED (or re-requested): one grind
      if (cool.grind <= 0) {
        const k = clamp01(0.55 + Math.abs(v.speed || 0) / 8);
        if (!one('gearFail', 0.75 * k, { rate: jitter(0.05) })) grind(k);
        cool.grind = STRESS_COOL.grind; cool.notch = Math.max(cool.notch, 0.15);
      }
      return;
    }
    let lever = 0;
    for (const n of ['fourWD', 'lockF', 'lockR']) if (now[n] !== was[n]) lever = 1;
    if (!lever || cool.notch > 0) return;
    cool.notch = SHIFT_COOL;
    if (!one('gear1', 0.85 * lever, { rate: jitter(0.05) })) notch(lever);
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
    } else {                                                 // detent tick: ~1/4 of a clunk, short, pitch varied (never machine-gunned)
      const f = (2600 + Math.random() * 500), k = kind === 'tick' ? 0.28 : 0.9;
      noiseBurst(g.pink, t, 0.018, k, [['bandpass', f, 10], ['bandpass', f * 1.47, 12]]);
      noiseBurst(g.brown, t + 0.006, 0.03, 0.75 * k, [['lowpass', 520, 1]]);
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
      crankSrc = shot(cr.b, 0.8 * voice.gain, { loop: true, rate, sl: cr.loop || null, offset: cr.loop && cr.start || 0 });
      sx.catch = cr.catch ? { b: cr.b, sec: cr.catch, rate, release: cr.release } : null;
    }
    if (!h.cranking && sx.cranking) {
      const caught = v.engineOn !== false;
      if (crankSrc) {                                              // caught: starter fades out equal-power over CATCH_XF while the
        if (caught) { epRamp(crankSrc._gain.gain, t, false); try { crankSrc.stop(t + CATCH_XF + 0.05); } catch (_) {} }   // idle bus
        else stopShot(crankSrc, t);                                // fades in (engine()); gave up: plain stop
        crankSrc = null;
      }
      if (caught) {                                                // fire-up tail (soft 60 ms attack, <= 1 s, faded over its release), else a synth cough-pop
        const c = sx.catch, len = c ? Math.min(1, (c.sec[1] - c.sec[0]) / c.rate) : 0;
        if (!(c && shot(c.b, 0.6 * voice.gain, { offset: c.sec[0], dur: len, rate: c.rate, attack: 0.06, release: c.release }))) { pop(); thud(0.25); }
      }
      sx.catch = null;
    }
    // hard start (damaged / hot engine): each stumble = a short failed-fire cough over the still-turning starter
    if (h.cranking && (h.crankCoughs || 0) > (sx.coughs || 0)) {
      if (!shot('misfire', 0.55 * voice.gain, { offset: 0.2 + Math.random() * 1.2, dur: 0.32, rate: jitter(0.06) * engRate('misfire', 'misfire'), attack: 0.02, release: 0.12 })) pop();
      if (crankSrc && crankSrc._gain) { const gg = crankSrc._gain.gain, v0 = gg.value; gg.cancelScheduledValues(t); gg.setValueAtTime(v0, t); gg.linearRampToValueAtTime(v0 * 0.55, t + 0.06); gg.linearRampToValueAtTime(v0, t + 0.35); }
    }
    sx.coughs = h.cranking ? h.crankCoughs || 0 : 0;
    sx.cranking = h.cranking;
    g.crankGain.gain.setTargetAtTime(h.cranking && !crankSrc ? 0.35 : 0, t, 0.03);
    // engine warmth (state-only): builds while running (faster on load), eases off when stopped; max with engineHeat
    const running = v.engineOn !== false;
    sx.warm = clamp01(sx.warm + (running ? dt * (WARM_IDLE + WARM_WORK * clamp01(v.throttle || 0) * clamp01(v.rpm / 4000)) : -dt * WARM_COOL));
    sx.runT = running ? (sx.runT || 0) + dt : 0;                 // s since the engine caught
    if (sx.on && !running && !h.cranking) {                      // just shut off / stalled: cooldown ticking, level + length by heat
      const heat = Math.max(sx.warm, h.engineHeat || 0);
      if (heat > TICK_HEAT && cool.tick <= 0) {
        const len = 3 + 9 * heat;
        sx.cooldownSrc = shot('cooldown', 0.12 + 0.38 * heat, { offset: Math.random() * 3, dur: len, lp: 3000 + 5000 * heat });
        if (!sx.cooldownSrc) ticks(heat, len);
        cool.tick = STRESS_COOL.tick;
      }
    }
    if (running && sx.cooldownSrc) { stopShot(sx.cooldownSrc, t); sx.cooldownSrc = null; }
    // heat-soak clinks while RUNNING hot at low rev / light gas (the idle doesn't mask them): intermittent faded cooldown
    // slices, level + brightness by heat; fade out once revved (> CLINK_RN), off gas, cooled or stopped (off path takes over)
    const heatR = Math.max(h.engineHot ? 1 : 0, sx.warm), kH = clamp01((heatR - CLINK_HEAT) / (1 - CLINK_HEAT));
    const clinkOk = running && sx.runT > CLINK_RUN && heatR > CLINK_HEAT && (sx.rn ?? 1) < CLINK_RN && (sx.thrS ?? 1) < 0.3;
    cool.clink -= dt;
    if (clinkOk && !sx.clinkSrc && cool.clink <= 0) {
      const len = 2.5 + 2 * Math.random(), src = shot('cooldown', 0.13 + 0.2 * kH, { offset: Math.random() * 3, dur: len, lp: 3500 + 4500 * kH, attack: 0.2 });
      if (src) { sx.clinkSrc = src; src.addEventListener('ended', () => { if (sx.clinkSrc === src) sx.clinkSrc = null; }); }
      else ticks(0.4 + 0.4 * kH, len);
      cool.clink = len + 0.6 + 1.4 * Math.random() * (1 - kH);          // hotter = denser clinking
    }
    if (!clinkOk && sx.clinkSrc) { stopShot(sx.clinkSrc, t); sx.clinkSrc = null; }
    sx.on = running;
    const hot = !!h.engineHot;
    if (hot && !sx.hot) {                                          // overheat: the full coughing misfire take (a stall = dieView run-down)
      if (shot('misfire', 0.9 * voice.gain, { rate: engRate('misfire', 'misfire') })) cool.misfire = 2.5; else cough();
    } else if (h.misfire && !st.misfire) {                         // misfire slot: a short faded slice, synth pop between
      if (cool.misfire <= 0 && shot('misfire', 0.75 * voice.gain, { offset: Math.random() * 1.5, dur: 0.9, rate: jitter(0.05) * engRate('misfire', 'misfire') })) cool.misfire = 1.4;
      else pop();
    }
    sx.hot = hot;
    for (let i = 0; i < 2; i++) { if (h.axleBroken[i] && !st.axle[i] && !via('busWorld', () => shot('axle', 1.0))) via('busWorld', axleSnap); st.axle[i] = h.axleBroken[i]; }
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
  /** Engine run-down: while it lasts the engine is drawn as running with rpm falling to DIE_RPM x idle, off gas;
   *  engDie carries the fade + stall sputter. Returns v unchanged otherwise. */
  function dieView(v, t, idle) {
    const off = v.engineOn === false, h = v.hardcore || {};
    if (!off || h.cranking) { if (sx.die || sx.dieDown) { sx.die = null; sx.dieDown = false; g.engDie.gain.cancelScheduledValues(t); g.engDie.gain.setTargetAtTime(1, t, 0.02); } return v; }
    if (sx.engOn && !sx.die && !h.cranking) {
      const stall = !!h.stalled, dur = stall ? DIE_STALL : DIE_KEY;
      sx.die = { t0: t, dur, rpm0: Math.max(idle, sx.rpmS || idle), stall };
      const p = g.engDie.gain; p.cancelScheduledValues(t); p.setValueAtTime(1, t);
      if (stall) {                                       // sputter: 3-4 dips, each recovering less, then the last gasp
        const n = 3 + (Math.random() < 0.5 ? 1 : 0); let tt = t + 0.12;
        for (let k = 0; k < n; k++) {
          const lv = 1 - (k + 1) / (n + 1), d = (dur - 0.25) / n * (0.7 + 0.5 * Math.random());
          p.linearRampToValueAtTime(lv * 0.25, tt + 0.05); p.linearRampToValueAtTime(lv, tt + 0.05 + 0.45 * d);
          if (k < 3) setTimeout(() => { if (sx.die) pop(); }, (tt - t + 0.04) * 1000);
          tt += d;
        }
        p.linearRampToValueAtTime(0, t + dur);
      } else p.setValueCurveAtTime(new Float32Array([1, 0.85, 0.6, 0.35, 0.15, 0.04, 0]), t + 0.001, dur);
    }
    const d = sx.die;
    if (!d) return v;
    const k = (t - d.t0) / d.dur;
    if (k >= 1) { sx.die = null; sx.dieDown = true; g.engDie.gain.cancelScheduledValues(t); g.engDie.gain.setValueAtTime(0, t); return v; }   // gate closes (epRamp), engDie restored on next run
    const rpm = d.rpm0 + (DIE_RPM * idle - d.rpm0) * Math.sqrt(k);
    sx.rpmS = sx.rpmPk = rpm;                            // no smoothing lag: the note drops with the engine
    return { ...v, engineOn: true, rpm, throttle: 0, hardcore: { ...h, misfire: d.stall && k > 0.3 } };
  }
  function engine(v, eng, t) {
    const idle = eng.idleRpm ?? 800, red = eng.redlineRpm ?? 5000;
    v = dieView(v, t, idle);
    const dtE = Math.min(0.1, Math.max(0, t - (sx.tLast ?? t)));
    sx.tLast = t;
    // JS-side smoothing: wheel-hop rpm jitter / finger noise is filtered BEFORE it reaches playbackRate, filter and blend,
    // so loops glide instead of warbling; engine catch glides 0.6 x idle -> idle (no pitch jump)
    const fl = sx.die ? 0.25 : 0.6, thrRaw = clamp01(v.throttle || 0), rpmPhys = Math.max(fl * idle, v.rpm || fl * idle);   // run-down sags below the usual floor
    const rpmInt = v.engineOn === false || rpmPhys <= idle ? rpmPhys
      : Math.min(red, idle + (rpmPhys - idle) * lerp(INTENT_LIFT, 1, thrRaw) + INTENT_PUSH * thrRaw * (red - idle) * (1 - clamp01((rpmPhys - idle) / (red - idle))));
    // converter flare: with the gas well down an automatic sits in its power band whatever the gear / wheel speed -- the
    // roar holds instead of sagging to a low note when the truck hooks up or upshifts
    const rpmRaw = v.engineOn === false ? rpmInt : Math.max(rpmInt, idle + FLARE_FLOOR * thrRaw * thrRaw * (red - idle));
    const ease = (s, x, up, dn) => (s == null ? x : s + (x - s) * (1 - Math.exp(-dtE / (x > s ? up : dn))));
    const thr = sx.thrS = ease(sx.thrS, thrRaw, THR_UP, THR_DN);
    sx.thrAvg = ease(sx.thrAvg, thrRaw, PEDAL_TAU, PEDAL_TAU);
    const held = thr * thr * (1 - clamp01(4 * Math.abs(thrRaw - sx.thrAvg)));   // 1 = gas held steady, 0 = pedal moving / off
    sx.rpmS = ease(sx.rpmS, rpmRaw, lerp(RPM_TAU, HELD_UP, held), lerp(lerp(RPM_TAU, RPM_FALL_GAS, thr * thr), HELD_DN, held));
    sx.rpmPk = Math.max(sx.rpmS, sx.rpmPk == null ? sx.rpmS : sx.rpmPk + (sx.rpmS - sx.rpmPk) * (1 - Math.exp(-dtE / lerp(PEAK_TAU, HELD_PK_TAU, held))));
    const rpm = sx.rpmS + lerp(PEAK_HOLD * thr * thr, HELD_PK, held) * (sx.rpmPk - sx.rpmS);   // audible rpm: steady roar under held gas
    const V = voice, rn = clamp01((rpm - idle) / (red - idle)), f = rpm / 60 * (V.order || ENGINE_ORDER);
    sx.rn = rn;
    sx.load = ease(sx.load, clamp01(0.65 * thr + 0.6 * rn), LOAD_UP, LOAD_DN);   // gas-led: held full gas = rev loop, no see-saw
    g.oscA.frequency.setTargetAtTime(f, t, 0.03);
    g.oscB.frequency.setTargetAtTime(f, t, 0.03);
    g.sub.frequency.setTargetAtTime(f / 2, t, 0.03);
    g.subGain.gain.setTargetAtTime(0.28 * V.sub, t, 0.1);
    g.engLP.frequency.setTargetAtTime(V.synthLp * lerp(LP_MIN, LP_MAX, clamp01(0.75 * rn + 0.25 * thr)), t, 0.05);
    // recording: truck override > profile idle / rev slices (in place, loopStart / loopEnd) > shared engine_idle
    const over = vid && ov[vid] && ov[vid].idle, bank = !over && prof && prof.bank ? bankSlices() : null;
    if (prof && prof.bank && !over) { bankEngine(v, eng, t, bank, rpm, thr, rn, idle); popCheck(thrRaw, rn, t, dtE); return; }
    for (let i = 0; i < BANK_MAX; i++) if (vox['eb' + i]) stopLoop('eb' + i);   // truck change away from a bank profile
    const pIdle = !over && prof ? slice(prof.idle) : null;
    const pRev = pIdle && prof.rev && eo.rev ? slice(prof.rev) : null;
    const solo = !!pIdle && !pRev;                       // idle-only (Buggy-style): capped pitch + more synth growl, opens up
    const want = over || (pIdle ? pIdle.b : buf.idle), wantR = pRev ? pRev.b : null;
    if (want && (!vox.idle || vox.idle.b !== want)) startLoop('idle', want, over ? null : pIdle);   // truck change / sample decoded
    if (wantR ? !vox.rev || vox.rev.b !== wantR : vox.rev) wantR ? startLoop('rev', wantR, pRev) : stopLoop('rev');
    const pRoar = pIdle && prof.roar ? slice(prof.roar, prof.roarXf ?? 0.22) : null;     // held-gas roar loop (profile.roar)
    if (pRoar ? !vox.roar || vox.roar.b !== pRoar.b : vox.roar) pRoar ? startLoop('roar', pRoar.b, pRoar) : stopLoop('roar');
    const P = pIdle ? profVoice() : V;
    // engine OFF (spawn / key off / stall) is silent; cranking: the synth starter only (v.engineOn false while off / cranking)
    const on = v.engineOn !== false ? 1 : (v.hardcore && v.hardcore.cranking && !crankSrc ? 0.5 : 0);
    const idleV = vox.idle, revV = vox.rev, running = v.engineOn !== false;
    g.engGain.gain.setTargetAtTime(on * (idleV ? Math.min(1, SYNTH_UNDER * 1.3 * P.synth * (solo ? SOLO_SYNTH : 1)) : 1) * (0.2 + 0.18 * thr + 0.08 * rn), t, on ? 0.06 : 0.12);
    // sub-bus gate: equal-power CATCH_XF fade in on every engine-on edge (crank catch / startEngine), out on stall / key off
    if (running !== !!sx.engOn) { epRamp(g.engSmp.gain, t, running); sx.engOn = running; if (running) sx.rpmS = sx.rpmPk = Math.min(sx.rpmS, 0.6 * idle + 0.4 * rpmRaw); }
    if (idleV) {                                          // recorded idle: pitch with rpm, opens up + louder with throttle
      const c = LOOP.idle, misfire = v.hardcore && v.hardcore.misfire ? 0.55 : 1;
      const base = pIdle ? P.rate : engRate('idle', 'rate'), lvl = misfire * c.gain * P.gain * (0.55 + 0.3 * thr + 0.25 * rn);
      // idle <-> rev: equal-power crossfade by smoothed load (smoothstep: no lurch at either end)
      const L = sx.load, x = revV ? L * L * (3 - 2 * L) : 0, gr = Math.sin(x * Math.PI / 2);
      // roar: gas floored (smoothed > ~0.7) with the rpm up -> equal-power handoff idle -> roar loop, held as long as the pedal
      const sm = u => (u <= 0 ? 0 : u >= 1 ? 1 : u * u * (3 - 2 * u));
      // roarIntro: while the stab's build-up plays, the loop waits; it comes in fast just before the build-up ends
      const intro = prof.roarIntro && sx.flareSrc, early = intro && t - sx.flareT < (sx.flareLen || 0) - 0.15;
      const hgT = vox.roar && running && !early ? sm((thr - 0.7) / 0.25) * sm((rn - 0.3) / 0.3) : 0;
      const hg = sx.roarG = ease(sx.roarG ?? 0, hgT, intro ? 0.06 : 0.3, 0.16), gi = Math.cos(x * Math.PI / 2) * Math.cos(hg * Math.PI / 2);
      if (vox.roar) {
        const rr = prof.roarRate || [1, 1.3], cR = LOOP.roar;
        vox.roar.src.playbackRate.setTargetAtTime(lerp(rr[0], rr[1], rn), t, 0.08);
        vox.roar.f.frequency.setTargetAtTime(P.lp * expLerp(cR.f0, cR.f1, 0.45 + 0.55 * rn), t, 0.06);
        vox.roar.f.Q.setTargetAtTime(Math.min(P.q, 0.9), t, 0.1);
        vox.roar.g.gain.setTargetAtTime(lvl * Math.sin(hg * Math.PI / 2) * cR.gain / c.gain * (prof.roarGain ?? 1), t, 0.05);
        // the stab flare hands over to the loop once it has had its say (same take: no seam in timbre)
        if (sx.flareSrc && !prof.roarIntro && hg > 0.6 && t - (sx.flareT ?? t) > 0.8) { stopShot(sx.flareSrc, t); sx.flareSrc = null; }
      }
      // rate: rpm (soft-capped below maxRate, no plateau kink) + a touch of throttle so feathering is heard even when bogged
      const rate = Math.max(0.4, softCap(base * Math.pow(rpm / idle, P.rpmExp) * (1 + 0.04 * thr), solo ? Math.min(P.maxRate, SOLO_MAX) : P.maxRate));
      idleV.src.playbackRate.setTargetAtTime(rate, t, 0.05);
      idleV.f.frequency.setTargetAtTime(P.lp * expLerp(c.f0, c.f1, 0.2 * thr + 0.65 * rn + 0.15 * thr * rn + (solo ? 0.15 * rn : 0)), t, 0.06);
      idleV.f.Q.setTargetAtTime(P.q, t, 0.1);
      idleV.g.gain.setTargetAtTime(lvl * gi, t, 0.06);
      if (revV) {                                         // pitch-matched to the idle loop (rate / revPitch): no beating mid-blend
        const cr = LOOP.rev;
        revV.src.playbackRate.setTargetAtTime(Math.max(0.4, rate / (prof.revPitch || 1)), t, 0.05);
        revV.f.frequency.setTargetAtTime(P.lp * expLerp(cr.f0, cr.f1, 0.25 + 0.4 * thr + 0.45 * rn), t, 0.06);   // full gas + high rpm = wide open
        revV.f.Q.setTargetAtTime(Math.min(P.q, 0.9), t, 0.1);
        revV.g.gain.setTargetAtTime(lvl * gr * cr.gain / c.gain * (prof.revGain ?? 1), t, 0.06);
      }
    }
    // flare: profile free-rev blip only on a genuine stab (< 40 % -> > 85 % within 0.5 s, below 50 % rpm, jabs ~BLIP_COOL apart, a new jab cuts the last) -- never repeated
    // while held at high rpm (that re-triggering is what pulsed); pitched with the loops, quick attack
    sx.thrLow = thrRaw < 0.4 ? 0.5 : Math.max(0, sx.thrLow - dtE);
    cool.rev -= dtE;
    if (pIdle && prof.flare && eo.blip && running && cool.rev <= 0 && thrRaw > 0.85 && sx.thrLow > 0 && rn < 0.5) {
      if (sx.flareSrc) { stopShot(sx.flareSrc, t); sx.flareSrc = null; }   // pumping: each jab cuts the last blip
      const r = prof.flare, rate = Math.min(1.3, Math.max(0.85, idleV ? idleV.src.playbackRate.value / (prof.revPitch || 1) : 1));
      const src = fileBuf(r[0]) && shot(fileBuf(r[0]), 0.4 * P.gain, { offset: r[1], dur: (r[2] - r[1]) / rate, rate, lp: 9000, attack: 0.03 });
      if (src) { cool.rev = BLIP_COOL * jitter(0.2); sx.flareSrc = src; sx.flareT = t; sx.flareLen = (r[2] - r[1]) / rate; src.addEventListener('ended', () => { if (sx.flareSrc === src) sx.flareSrc = null; }); }
      sx.thrLow = 0;
    }
    if (running) popCheck(thrRaw, rn, t, dtE);
  }

  /** Loop-bank engine (profile.bank): the note F = f_idle (rpm / idle)^pitchExp picks the two bank entries around it,
   *  equal-power crossfade on log pitch, each at rate F / f0 (<= maxRate on the top one). Synth growl sits under it. */
  function bankEngine(v, eng, t, bank, rpm, thr, rn, idle) {
    const running = v.engineOn !== false, on = running ? 1 : (v.hardcore && v.hardcore.cranking && !crankSrc ? 0.5 : 0);
    const P = profVoice();
    g.engGain.gain.setTargetAtTime(on * (bank ? Math.min(1, SYNTH_UNDER * 1.3 * P.synth) : 1) * (0.2 + 0.18 * thr + 0.08 * rn), t, on ? 0.06 : 0.12);
    if (running !== !!sx.engOn) { epRamp(g.engSmp.gain, t, running); sx.engOn = running; if (running) sx.rpmS = sx.rpmPk = Math.min(sx.rpmS, idle); }
    for (const n of ['idle', 'rev', 'roar']) if (vox[n]) stopLoop(n);
    if (!bank || !bank.length) return;
    for (let i = 0; i < bank.length && i < BANK_MAX; i++) if (!vox['eb' + i] || vox['eb' + i].b !== bank[i].b) startLoop('eb' + i, bank[i].b, bank[i]);
    const F = bank[0].f * (P.rate || 1) * Math.pow(Math.max(0.5, rpm / idle), prof.pitchExp ?? P.rpmExp ?? 0.8);   // sound.rate: a higher-strung engine
    let i = 0; while (i + 1 < bank.length && bank[i + 1].f <= F) i++;
    const j = Math.min(bank.length - 1, i + 1), x = j === i ? 0 : clamp01(Math.log(F / bank[i].f) / Math.log(bank[j].f / bank[i].f));
    const misfire = v.hardcore && v.hardcore.misfire ? 0.55 : 1, c = LOOP.bank;
    const f0 = P.floor ?? 0.5, lvl = misfire * c.gain * P.gain * (f0 + (0.85 - f0) * thr + 0.25 * rn);   // floor = idle level (full gas unchanged)
    const l0 = P.lpIdle ?? 0.25, lp = P.lp * expLerp(c.f0, c.f1, l0 + (0.65 - l0) * thr + 0.35 * rn);   // off-gas at high rpm = duller overrun; lpIdle opens the idle
    for (let k = 0; k < bank.length && k < BANK_MAX; k++) {
      const V = vox['eb' + k]; if (!V) continue;
      const gk = k === i ? Math.cos(x * Math.PI / 2) : k === j && j !== i ? Math.sin(x * Math.PI / 2) : 0;
      const rate = Math.max(0.5, Math.min(k === bank.length - 1 ? P.maxRate || 1.5 : 1.6, F / bank[k].f));
      if (gk > 0.001) V.src.playbackRate.setTargetAtTime(rate, t, 0.04);
      V.f.frequency.setTargetAtTime(lp, t, 0.06); V.f.Q.setTargetAtTime(Math.min(P.q, 0.9), t, 0.1);
      V.g.gain.setTargetAtTime(lvl * gk, t, 0.05);
    }
  }

  /** Backfire / overrun pops (profile.pops): lifting off from high revs (gas > 60 % within the last 0.4 s, now < 15 %,
   *  rpm > 40 % of the band) fires 1-3 random takes 70-220 ms apart -- chance grows with rpm, never twice within ~3 s. */
  function popCheck(thrRaw, rn, t, dt) {
    const pp = prof && prof.pops, b = pp && fileBuf(pp[0]);
    sx.gasHi = thrRaw > 0.6 ? 0.4 : Math.max(0, (sx.gasHi || 0) - dt);
    cool.pop = (cool.pop || 0) - dt;
    if (!b || muted || cool.pop > 0 || thrRaw > 0.15 || !(sx.gasHi > 0) || rn < 0.4) return;
    sx.gasHi = 0;
    cool.pop = 2.4 + 1.6 * Math.random();
    if (Math.random() > 0.3 + 0.6 * rn) return;                       // sometimes it just burbles down
    const n = 1 + Math.floor(Math.random() * 3), P = profVoice();
    let d = 0.06 + 0.06 * Math.random();
    for (let k = 0; k < n; k++) {
      const r = pp[1][Math.floor(Math.random() * pp[1].length)], rate = jitter(0.07);
      shot(b, (0.45 + 0.4 * rn) * (k ? 0.75 : 1) * (P.gain || 1), { offset: r[0], dur: (r[1] - r[0]) / rate, rate, delay: d, attack: 0.004, release: 0.06 });
      d += 0.07 + 0.15 * Math.random();
    }
  }

  /** Surface + water loops from each wheel's tread speed / slip; splashes on wheels (or the chassis) entering water. */
  function surfaces(v, t, dt) {
    for (const c in acc) { const a = acc[c]; a.n = 0; a.drive = 0; a.slip = 0; }
    const ws = v.wheels, nw = ws.length || 1, spd = Math.abs(v.speed || 0), hv = Math.hypot(v.vx || 0, v.vy || 0);
    let deep = 0, entry = 0, inW = 0;
    cool.splash -= dt;
    // water per tyre with hysteresis: a tyre hopping in a ford (grounded flicker) or briefly touching a stone stays 'in the
    // water' (WET_HOLD); a SPLASH only when that tyre enters after being dry > WET_DRY s (each tyre its own splash: two or
    // more entering overlap naturally, slightly staggered). Wading = the loops, never a string of repeated splashes.
    for (let i = 0; i < ws.length; i++) {
      const w = ws[i], cat = w.grounded ? surfaceCat(w.surface) : null;
      const wet = cat === 'water';
      const W = sx.wet[i] || (sx.wet[i] = { hold: 0, dry: 0, cool: 0 });
      W.cool -= dt;
      if (wet) {
        if (W.dry > WET_DRY && W.cool <= 0 && hv > 0.6) {
          const k = clamp01((hv - 0.6) / 4), big = hv > 3.5 && buf.splashBig && Math.random() < 0.5;
          shot(big ? 'splashBig' : 'splashSmall', (big ? 0.35 : 0.3) + 0.45 * k, { rate: jitter(0.12), delay: Math.random() * 0.05,
            dur: big ? 1.4 : 1.2, release: 0.5, lp: lerp(2500, 12000, k) });
          W.cool = 1.5;
        }
        W.hold = WET_HOLD; W.dry = 0;
      } else {
        W.hold -= cat ? 3 * dt : dt;                                  // on another surface: lets go faster than in the air
        if (W.hold <= 0) W.dry += dt;
      }
      if (wet || W.hold > 0) inW += 1 / nw;
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
    // on the planks the engine steps back (up to WOOD_DUCK at full wood share while rolling) so the deck rumble carries
    const woodK = acc.wood.n * clamp01((acc.wood.n > 0 ? acc.wood.drive / acc.wood.n : 0) / 1.5);
    const wa = acc.water, wd = wa.n > 0 ? wa.drive / wa.n : 0, wet = Math.max(inW, bodyWet ? 1 : 0);
    const kw = clamp01(Math.max(wd, hv) / LOOP.shallow.full);
    const waterK = Math.sqrt(wet) * clamp01(Math.max(wd, hv) / 1.2);
    g.engDuck.gain.setTargetAtTime(1 - Math.min(0.5, WOOD_DUCK * woodK + WATER_DUCK * waterK), t, 0.25);
    // wading: the loop carries the water (scaled by the share of tyres in it, rate / filter by speed); quiet when parked
    // a crawl (1-2 m/s) is most wading: sqrt curve, so it is clearly there at walking pace, not only at speed
    setLoop('shallow', LOOP.shallow.gain * Math.sqrt(wet) * (1 - deep) * (0.22 + 0.78 * Math.sqrt(kw)), kw, 0, t);
    setLoop('deep', LOOP.deep.gain * deep * (0.35 + 0.65 * Math.sqrt(kw)), kw, 0, t);
    if (entry > 1 && cool.splash <= 0) {                  // chassis dropping into the water: one big splash
      if (shot(buf.splashBig ? 'splashBig' : 'splashSmall', clamp01(0.35 + entry / 8), { rate: jitter(), dur: 1.8, release: 0.6 })) cool.splash = 2;
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
    let hit = 0, bottomed = false, hitSurf = null;
    hitFx.busy *= Math.exp(-dt / BUSY_TAU);
    for (let i = 0; i < v.wheels.length; i++) {
      const w = v.wheels[i], s = st.wheels[i] || (st.wheels[i] = { c: null, t: 0, cool: 0, peak: 0, vn: 0 });
      s.cool -= dt; s.t += dt; s.scool = (s.scool || 0) - dt;
      let wvn = 0, surf = w.surface;
      for (const c of w.contacts || []) if (c.vn > wvn) { wvn = c.vn; surf = c.surface || surf; }
      // strike: closing speed jumps (tyre into a log / ledge face) -- independent of the shock's compression rate
      const prevVn = s.cvn || 0; s.cvn = wvn;
      if (wvn > STRIKE_VN && wvn - prevVn > STRIKE_RISE && s.scool <= 0) {
        const k = clamp01((wvn - STRIKE_VN + 0.3) / STRIKE_SPAN);
        if (k > hit) { hit = k; hitSurf = surf; }
        s.scool = STRIKE_COOL; s.cool = Math.max(s.cool, 0.15);   // the shock spike of the same hit stays quiet
      }
      if (s.c === null) { s.c = w.compression; s.t = 0; continue; }
      if (w.compression === s.c) continue;                 // no physics step happened this frame
      const rate = (w.compression - s.c) / Math.min(0.1, Math.max(s.t, 1 / 240));
      s.c = w.compression; s.t = 0;
      if (surf) s.surf = surf;
      if (s.peak > 0) {                                    // inside a spike: fire once the rate stops rising
        if (wvn > s.vn) s.vn = wvn;
        if (rate > s.peak) { s.peak = rate; continue; }
        const bot = w.compression > 0.93;
        const k = Math.max(clamp01((s.peak - HIT_RATE) / HIT_SPAN), clamp01((s.vn - 0.6) / 4.5)) + (bot ? 0.2 : 0);
        if (k >= HIT_MIN) { if (k > hit) { hit = k; hitSurf = s.surf || w.surface; } bottomed = bottomed || bot; }   // minor bumps: silent
        s.peak = 0; s.vn = 0; s.cool = 0.22;
      } else if (rate > HIT_RATE && s.cool <= 0) { s.peak = rate; s.vn = wvn; }
    }
    cool.thud -= dt; cool.metal -= dt; cool.boulder -= dt;
    sx.bottomK = bottomed ? Math.min(1, hit) : 0;           // -> chassisStress(): metallic groan on heavy bottom-outs
    if (hit > 0 && cool.thud <= 0) {                       // left + right wheel of an axle land as one thud
      const k = Math.min(1, hit);
      tyreHit(Math.min(1, k + (bottomed ? 0.1 : 0)), hitSurf, 1 / (1 + 0.3 * hitFx.busy));   // material by the hit wheel's surface
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
        revReady: !!(prof.rev && fileBuf(prof.rev[0])), crank: prof.crank, crankReady: !!fileBuf(prof.crank), hasRev: !!prof.rev, hasBlip: !!prof.flare } : null }; },
    get engineOpts() { return { ...eo }; },
    /** A/B: {rev?: bool (rev loop; off = idle-only Buggy-style), blip?: bool (stab blip)}. */
    setEngineOpts(o = {}) { eo = { rev: o.rev ?? (prof ? prof.revLoop !== false : true), blip: o.blip !== false }; },   // unset rev = profile revLoop default
    /** Select the truck's engine voice (call per run): id = content.json vehicle id, cfg = its vehicle JSON. */
    setVehicle(id, cfg = {}) {
      vid = id || null; sx.warm = 0; sx.runT = 0; own = { ...(cfg.sound || {}) }; delete own.profile; delete own.files;   // new run = cold engine
      voice = voiceFor(id, cfg); prof = profileFor(id, cfg); api.setEngineOpts();
      hornCfg = HORNS[cfg.sound?.horn || cfg.look?.style] || HORNS.truggy;   // per-rig horn voice   // profile defaults: V8s idle-only + blip, Jeep rev loop
      if (g) { loadOverrides(vid); loadProfile(); }
    },
    /** Debug: current gain of every running sample loop + synth engine / slip (console: offroad.audio.voices). */
    get voices() {
      const o = {}; for (const n in vox) o[n] = +vox[n].g.gain.value.toFixed(3);
      if (vox.idle) { o.idleRate = +vox.idle.src.playbackRate.value.toFixed(2); o.idleLp = Math.round(vox.idle.f.frequency.value); o.idleLoop = [+vox.idle.src.loopStart.toFixed(3), +vox.idle.src.loopEnd.toFixed(3)]; }
      if (vox.rev) { o.revRate = +vox.rev.src.playbackRate.value.toFixed(2); o.revLp = Math.round(vox.rev.f.frequency.value); }
      if (vox.roar) o.roarRate = +vox.roar.src.playbackRate.value.toFixed(2);
      if (g) { o.stage = +g.stage.gain.value.toFixed(3); o.engSmp = +g.engSmp.gain.value.toFixed(3); o.engDie = +g.engDie.gain.value.toFixed(3); o.engDuck = +g.engDuck.gain.value.toFixed(3); }
      o.clink = !!sx.clinkSrc;                                     // running heat-soak clinks playing
      if (g) { o.synthEngine = +g.engGain.gain.value.toFixed(3); o.synthSlip = +g.slipGain.gain.value.toFixed(3); }
      return o;
    },
    /** Page load, before any gesture: create the (suspended) context and fetch + decode every sample now, so the first
     *  tap only has to resume it (loading at the first tap = 2-3 s of silence on phones). `id, cfg` = the truck shown first:
     *  its engine recording is fetched too (setVehicle reuses the decoded files). */
    preload(id, cfg = {}) {
      if (!api.unlock(false)) return false;
      const p = profileFor(id, cfg); if (p) {
        for (const k of ['idle', 'rev', 'flare', 'roar']) if (p[k]) loadFile(p[k][0]);
        for (const e of p.bank || []) loadFile(e[0]);
        if (p.pops) loadFile(p.pops[0]);
        loadFile(p.crank);
      }
      return true;
    },
    unlock(resume = true) {
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
      if (resume && !injected && ctx.state === 'suspended') ctx.resume().catch(() => {});
      return true;
    },
    update(v, dt = 1 / 60, eng = {}) {
      if (!g || !(injected || ctx.state === 'running')) { stale = true; return; }
      if (stale) { api.reset(); stale = false; }
      const t = ctx.currentTime;
      via('busEng', () => engine(v, eng, t));
      tyres(v, t);
      via('busUi', () => shifts(v, dt, !muted));
      if (muted) { for (const n in vox) if (!ENG_LOOPS.includes(n)) vox[n].g.gain.setTargetAtTime(0, t, 0.05); return; }   // one-shots not worth scheduling
      surfaces(v, t, dt);
      impacts(v, dt);
      chassisStress(v, dt);
      winch(v.winch, dt, t);
      via('busEng', () => failures(v, t, dt));
    },
    /** Result jingle ('fail' = random laugh take, 'finish'): own path to the speakers, so it plays while the game bus is muted
     *  behind the LEVEL FAILED / FINISH panel; `userMute` (MENU MUTE) silences it. Returns the source or null. */
    jingle(kind, userMute = false) {
      if (!g || userMute || !(injected || ctx.state === 'running')) return null;
      const names = (JINGLES[kind] || []).filter(n => buf[n]);
      if (!names.length) return null;
      const src = ctx.createBufferSource(), gn = ctx.createGain();
      src.buffer = buf[names[Math.floor(Math.random() * names.length)]];
      gn.gain.value = volume * 0.9 * vol.master * vol.ui; src.connect(gn); gn.connect(ctx.destination);
      src.onended = () => gn.disconnect();
      src.start();
      return src;
    },
    /** HUD shifter hook: kind 'notch' (lever passes a gate) | 'commit' (range / drivetrain engaged) | 'drive' (4WD / locker toggle). */
    shift(kind = 'tick', info = {}) {
      if (!g || muted || !(injected || ctx.state === 'running')) return false;
      if (kind === 'drive' || kind === 'commit') return true;       // played from the vehicle state edges (shifts()), no double
      if (kind === 'key') { via('busUi', () => notch(0.45)); return true; }   // ignition key: small detent tick (the starter follows)
      // 'tick' (and legacy 'notch'): the lever passing a slot on a slow HUD drag: a short quiet synth detent, at most one at a time
      if (cool.tick2 > 0) return true;
      cool.tick2 = 0.06;
      via('busUi', () => lever('tick', info));
      return true;
    },
    /** r15 steering tactile fallback (haptics `tactile` kinds, paired with the vibration: Android Brave mutes vibrate() on a
     *  Silent ringer): kind 'steerCenter' = a tiny high detent click, 'steerLock' = a soft low bump-stop thump; very quiet,
     *  UI bus (MUTE / INTERFACE volume apply), level 'light' | 'strong' scales it, k = steering load (firmer). */
    steerClick(kind = 'steerCenter', k = 0, level = 'light') {
      if (!g || muted || level === 'off' || !(injected || ctx.state === 'running')) return false;
      const a = (level === 'strong' ? 0.2 : 0.12) * (0.8 + 0.3 * Math.max(0, Math.min(1, k)));
      via('busUi', () => {
        const t = ctx.currentTime;
        if (kind === 'steerLock') {
          noiseBurst(g.brown, t, 0.045, 1.4 * a, [['lowpass', 260, 0.7]]);
          noiseBurst(g.pink, t, 0.012, 0.5 * a, [['bandpass', 1400 * jitter(0.06), 6]]);
        } else noiseBurst(g.pink, t, 0.01, a, [['bandpass', 3200 * jitter(0.06), 10]]);
      });
      return true;
    },
    /** Horn held (true) / released (false): HUD TILT wheel centre cap or H. Muted / suspended: nothing starts. */
    horn(on, cut = false) {                             // cut: fast fade (HUD: the cap touch turned out to be a swipe)
      if (!g) return false;
      if (!on) { hornOff(false, cut); return true; }
      if (muted || !(injected || ctx.state === 'running')) return false;
      hornOn(); return true;
    },
    get horning() { return !!hornV; },
    /** MENU -> SOUND: { master, engine, world, ui } each 0..1 (missing keys unchanged). */
    setVolumes(v = {}) {
      for (const k in vol) if (Number.isFinite(+v[k])) vol[k] = Math.max(0, Math.min(1, +v[k]));
      if (!g) return;
      const t = ctx.currentTime;
      g.master.gain.setTargetAtTime(muted ? 0 : volume * vol.master, t, 0.03);
      g.busEng.gain.setTargetAtTime(vol.engine, t, 0.03); g.busWorld.gain.setTargetAtTime(vol.world, t, 0.03); g.busUi.gain.setTargetAtTime(vol.ui, t, 0.03);
    },
    get volumes() { return { ...vol }; },
    /** Showroom parade (r13): whole-mix gain 0..1 by the truck's distance from the stage centre, so a truck swap is heard as
     *  the old one driving away and the new one arriving (was a hard cut between engine voices). Runs reset it to 1. */
    stageFade(k = 1, tau = 0.08) {
      if (!g) return;
      g.stage.gain.setTargetAtTime(Math.max(0, Math.min(1, k)), ctx.currentTime, tau);
    },
    /** Showroom parade (r14): hand the running engine loops over to a detached GHOST that keeps playing (frozen at the
     *  leaving truck's last pitch / filter) and fades out over `fade` s, while the next truck's voices build fresh on the
     *  normal path = a real crossfade between the two engines (r13 faded the old one to ~silence before the swap: abrupt). */
    handoff(fade = 1.8) {
      if (!g) return;
      const t = ctx.currentTime;
      const lvl = g.stage.gain.value * g.engSmp.gain.value * g.engDie.gain.value * g.engDuck.gain.value * g.busEng.gain.value;
      const ghost = ctx.createGain(); ghost.connect(g.limiter);
      ghost.gain.setValueAtTime(lvl, t);
      ghost.gain.setTargetAtTime(0, t, fade / 4);                // ~98 % gone after `fade`
      for (const n of ENG_LOOPS) {
        const v = vox[n]; if (!v) continue;
        v.g.disconnect(); v.g.connect(ghost);
        try { v.src.stop(t + fade + 0.2); } catch (_) {}
        const i = g.sources.indexOf(v.src); if (i >= 0) g.sources.splice(i, 1);
        delete vox[n];
      }
      setTimeout(() => { try { ghost.disconnect(); } catch (_) {} }, (fade + 0.4) * 1000);
    },
    reset() {
      for (const s of st.wheels) { s.c = null; s.peak = 0; s.vn = 0; s.cool = 0.15; }
      hitFx.busy = 0;
      st.len = null; st.pay = 0; st.bodyHit = true; st.bodyCool = 0.15;
      st.body = null; st.stalled = false; st.misfire = false; st.axle = [false, false];
      sx.wet.length = 0; sx.bodyWet = false; sx.hot = false; // a teleport into water is not an entry splash
      for (const k in cool) cool[k] = Math.max(cool[k], 0.15);
      if (ctx && crankSrc) { stopShot(crankSrc, ctx.currentTime); crankSrc = null; }
      if (ctx && sx.cooldownSrc) { stopShot(sx.cooldownSrc, ctx.currentTime); sx.cooldownSrc = null; }
      if (ctx && sx.clinkSrc) { stopShot(sx.clinkSrc, ctx.currentTime); sx.clinkSrc = null; }
      sx.cranking = false; sx.catch = null; sx.on = false; sx.die = null; sx.dieDown = false; if (g) { g.engDie.gain.cancelScheduledValues(ctx.currentTime); g.engDie.gain.setValueAtTime(1, ctx.currentTime); } sx.rpmS = null; sx.rpmPk = null; sx.thrS = null; sx.thrAvg = null; sx.load = null; sx.susp.length = 0; sx.bottomK = 0; sx.slamK = 0; sx.thrLow = 0;
      gear = null;                                                      // new run / teleport: re-prime, no shift sounds
      if (ctx && sx.shiftSrc) { stopShot(sx.shiftSrc, ctx.currentTime); } sx.shiftSrc = null;
    },
    setMuted(m) {
      m = !!m;
      if (m === muted) return;
      muted = m;
      if (!g) return;
      g.master.gain.setTargetAtTime(muted ? 0 : volume * vol.master, ctx.currentTime, 0.015);
      if (muted) { g.creakGain.gain.setTargetAtTime(0, ctx.currentTime, 0.02); g.crankGain.gain.setTargetAtTime(0, ctx.currentTime, 0.02); api.reset(); hornOff(); }
      if (muted) for (const n in vox) vox[n].g.gain.setTargetAtTime(0, ctx.currentTime, 0.02);
    },
    destroy() {
      if (typeof document !== 'undefined') document.removeEventListener('visibilitychange', onVisibility);
      if (g) for (const s of g.sources) { try { s.stop(); } catch (_) {} }
      if (crankSrc) { try { crankSrc.stop(); } catch (_) {} crankSrc = null; }
      if (ctx && g) hornOff(true);
      for (const n in vox) delete vox[n];
      if (ctx && !injected && ctx.state !== 'closed') ctx.close().catch(() => {});
      g = null;
    },
  };
  return api;
}
