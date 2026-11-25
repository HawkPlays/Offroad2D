// engine/hud.js — mobile landscape touch controls + gauges (DOM only; no physics / canvas code).
// Every control captures its own pointer, so Gas + Steer + Brake + Winch + toggles work simultaneously.
// Two-thumb layout: left = steering (wheel or ◄ ►) + WINCH; right = H-gate shifter (L+ L L- · N · H R) · BRAKE · GAS; bottom-centre =
// KEY 4WD F-LOCK R-LOCK; top bar = MENU (pause panel) + RESET; top-right = telemetry cluster. Icon-first: pictograms, no words
// where an icon reads (engine/icons.js); phone haptics (engine/haptics.js).
import { svg, paintIcon } from './icons.js';
import { createHaptics, hapticsSupported, HAPTIC_LEVELS } from './haptics.js';
import { wheelSvg } from './wheels.js';
import { TILT, TiltFilter, upFromEuler, tiltSteer } from './tilt.js';
export { tiltRoll, screenRoll, tiltSteer } from './tilt.js';

const MOMENTARY = [
  { id: 'brake', icon: 'brakePedal', label: 'brake', cls: 'pedal brake', keys: ['ArrowDown', 'KeyS', 'Space'] },
  { id: 'gas',   icon: 'pedal', label: 'gas', cls: 'pedal gas',   keys: ['ArrowUp', 'KeyW'] },
  { id: 'winch', icon: 'hook', label: 'winch', cls: 'btn round winch', keys: ['KeyE'] },
  { id: 'left',  label: '◄',     cls: 'btn steer',   keys: ['ArrowLeft', 'KeyA'] },    // steer: state.steer = clamp(wheel + right - left)
  { id: 'right', label: '►',     cls: 'btn steer',   keys: ['ArrowRight', 'KeyD'] },
];
// forward ranges (keys: 4 / L cycle, Shift = back, Q up toward H, Z down toward L-; V = R toggle, N = neutral toggle)
const RANGES = ['H', 'L+', 'L', 'L-'];
// H-GATE: left column L+ · L · L- (top -> bottom), N mid-rail, right column H (top) · R (bottom). [col, row]; J = right-column
// junction on the rail (no slot). Rail: L - N - J.
const GATE_POS = { 'L+': [0, 0], L: [0, 1], 'L-': [0, 2], N: [1, 1], J: [2, 1], H: [2, 0], R: [2, 2] };
const GATE_SEGS = [['L+', 'L'], ['L', 'L-'], ['L', 'N'], ['N', 'J'], ['J', 'H'], ['J', 'R']];
const SLOTS = ['H', 'L+', 'L', 'L-', 'N', 'R'];
const GATE_FX = [0.3, 0.5, 0.7], GATE_FY = [0.2, 0.5, 0.8];   // node position: fractions of the shifter box
const TICK_SPEED = 0.45;                             // px/ms: a slower drag ticks each slot it passes (fast swipes stay quiet)
const DRIVE = ['fourWD', 'lockF', 'lockR'];          // drivetrain toggles that fire onShift('drive')
const TOGGLES = [                                    // reset to off on every run (resetControls)
  { id: 'fourWD',  label: '4WD', cls: 'btn tog',  key: 'Digit1' },
];
const OPTIONS = [                                    // pause-panel switches; persist across runs
  { id: 'debug', label: 'DEBUG', key: null },
  { id: 'mute',  label: 'MUTE',  key: 'KeyM' },
];
/** Saved HUD prefs (o2d.ui): steer mode + tilt range (MENU -> STEERING), control size scales (MENU -> RESIZE HUD), graphics quality
 *  (MENU -> GRAPHICS: auto | low | med | high). Tilt has no calibration: gravity roll, level screen = straight. */
export const UI_DEFAULTS = { steer: 'wheel', wheel: 1, gas: 1, brake: 1, tiltMax: 25, gfx: 'auto', spot: 'auto', wheelSens: 1,
  volMaster: 1, volEngine: 1, volWorld: 1, volUi: 1, haptics: 'light' };   // MENU -> SOUND: volumes 0..1 + vibration off | light | strong
export const SPOT_UI = ['auto', 'on', 'off'];   // SPOTTER inset: AUTO near steering-critical hazards | always | never
export const GFX_MODES = ['auto', 'low', 'med', 'high'];
export const STEER_MODES = ['buttons', 'wheel', 'tilt'];
const UI_RANGE = { wheel: [0.7, 1.4], gas: [0.7, 1.4], brake: [0.7, 1.4], tiltMax: [12, 40], wheelSens: [0.5, 2],
  volMaster: [0, 1], volEngine: [0, 1], volWorld: [0, 1], volUi: [0, 1] };
const WHEEL_MAX = 2.36;                              // wheel lock-to-centre angle (135°) = full steer at sensitivity 100 %
const wheelLock = ui => WHEEL_MAX / (ui.wheelSens || 1);   // WHEEL SENSITIVITY: 200 % = full lock at 68°, 50 % = at 270°
const TILT_HINT = 1500, TILT_WAIT = 6000;            // ms of visible time without tilt data: "waiting" toast, then fall back to WHEEL
const HORN_HUB = 0.55;                               // TILT mode: wheel centre cap (radius share of the wheel) = horn button
const HORN_CAP = 0.34, HORN_SLIDE = 0.5;              // WHEEL mode: cap (drawn r 0.28) tap zone; a honking thumb past HORN_SLIDE starts steering
// WHEEL mode (r13; the r12 110 ms still-hold arm felt laggy and killed rapid fun taps): a cap touch honks after HORN_ARM ms (a
// tiny guard: most swipes have moved by then); moving > HORN_MOVE x radius within HORN_SWIPE ms of the touch = a swipe that
// started on the cap -> the horn is cut with a near-silent fade (audio horn(false, cut)) and the thumb steers from there; a cap
// touch within HORN_RECENT ms of letting go of the rim = the next swipe of a steering hand -> steering, no horn; every honk
// lasts >= HORN_TAP ms (quick taps still beep)
const HORN_ARM = 40, HORN_MOVE = 0.12, HORN_SWIPE = 220, HORN_RECENT = 350, HORN_TAP = 90;

/** Overall damage 0..1 for the single DMG bar: body, worst shock, front / rear axle (shaft stress, snapped = 1).
 *  The worst part dominates and the rest add a quarter, so one failure reads critical. Heat is in the status line instead. */
export function overallDamage(h) {   // permanent damage only (axle STRESS comes and goes: hidden; a snapped axle counts)
  const parts = [h.body || 0, Math.max(0, ...(h.shock || [0])), h.axleBroken[0] ? 1 : 0, h.axleBroken[1] ? 1 : 0];
  const worst = Math.max(...parts), sum = parts.reduce((a, b) => a + b, 0);
  return Math.min(1, worst + 0.25 * (sum - worst));
}

/** Run score out of 1000 (finish screen): TIME 400 (par / time, full at or under par) + FUEL 300 (fuel left / starting tank)
 *  + CONDITION 300 (1 - overallDamage). Stars: >= 850 three, >= 650 two, else one. All inputs plain numbers (s, fractions). */
/** Stars for a score total (/1000): ≥850 3, ≥650 2, else 1 (a finished run always earns one). */
export const starsOf = total => total >= 850 ? 3 : total >= 650 ? 2 : 1;

export function scoreRun({ time, par, fuel, tank = 1, damage = 0 }) {
  const c01 = v => Math.max(0, Math.min(1, v));
  const t = Math.round(400 * c01(par > 0 && time > 0 ? par / time : 1)), f = Math.round(300 * c01(fuel / (tank || 1))), d = Math.round(300 * c01(1 - damage));
  const total = t + f + d;
  return { total, time: t, fuel: f, condition: d, stars: starsOf(total) };
}

/**
 * @param root     container element (positioned over the canvas)
 * @param handlers { onRecover(), onRestart(), onPlayAgain(), onMenu() (level select), onPause?(paused), active?() (false: keydowns ignored), cam? '2.5D'|'2D', onCam?(mode),
 *                   ui? {steer, wheel, gas, brake} (saved sizes), onUi?(ui), onVolume?(vols), onShift?(kind 'tick'|'drive'|'key', info) }   (onPlayAgain falls back to onRestart)
 * @returns { state, resetControls(), update(vehicleState), toast(text), hint(text, ms?), showFinish(run), hideFinish(),
 *            pause(open?), paused, ui, dodge {top, right} (left-cluster box, CSS px, for the debug overlay), destroy() }
 *   state = { gas, brake (0..1 analog: touch slide / key = 1), winch, left, right (0|1 momentary), steer (-1..1: wheel angle + right - left), reverse, fourWD, lockF, lockR, range, low,
 *             ignition (key-turn counter: tap KEY / key K; each change = one turn -> vehicle cranks an off engine / shuts a running one),
 *             debug, mute (pause-panel switches, keys G / M), cam ('2.5D'|'2D', key C; steering switches to 2.5D) }
 */
export function createHud(root, handlers = {}) {
  const state = { gas: 0, brake: 0, winch: 0, left: 0, right: 0, steer: 0 };
  const els = {};
  const offs = [];
  const on = (target, type, fn, opts) => { target.addEventListener(type, fn, opts); offs.push(() => target.removeEventListener(type, fn, opts)); };
  const mk = (cls, text, parent, tag = 'div') => {
    const el = document.createElement(tag);
    el.className = cls; if (text) el.textContent = text;
    parent.appendChild(el); return el;
  };
  const ui = { ...UI_DEFAULTS };
  for (const k in UI_DEFAULTS) if (handlers.ui && handlers.ui[k] != null) ui[k] = handlers.ui[k];
  if (!STEER_MODES.includes(ui.steer)) ui.steer = 'wheel';
  if (!HAPTIC_LEVELS.includes(ui.haptics)) ui.haptics = 'light';
  const haptics = createHaptics(ui.haptics, { tactile: (kind, k, lvl) => { try { handlers.onTactile?.(kind, k, lvl); } catch (_) { /* audio optional */ } } });
  const buzz = (kind, k) => { try { return haptics.play(kind, k); } catch (_) { return null; } };
  for (const k in UI_RANGE) ui[k] = Math.min(UI_RANGE[k][1], Math.max(UI_RANGE[k][0], Number.isFinite(+ui[k]) ? +ui[k] : UI_DEFAULTS[k]));
  let paused = false;

  root.classList.add('hud');
  const zones = {};
  for (const z of ['tl', 'tr', 'bl', 'br', 'bc']) zones[z] = mk('hud-zone ' + z, '', root);
  // left thumb: steering at the bottom-left corner, WINCH beside it; right thumb: H-gate shifter · BRAKE · GAS
  // (brake next to gas like real pedals: one thumb rocks between them while the other steers)
  const steerBox = mk('steer-box', '', zones.bl), thumbRow = mk('thumb-row', '', zones.bl);
  const steerRow = mk('steer-row', '', steerBox);
  const place = { brake: zones.br, winch: thumbRow, gas: zones.br, fourWD: zones.bc, left: steerRow, right: steerRow };
  const iconBtn = (el, name, label) => { el.insertAdjacentHTML('beforeend', svg(name)); el.setAttribute('aria-label', label); el.title = label; return el; };
  const menuBtn = iconBtn(mk('btn small menu', '', zones.tl), 'menu', 'menu (Esc)');

  // momentary buttons: pointer capture keeps the press alive even if the finger slides off
  const holdButton = (el, set) => {
    let pid = null;
    const up = e => { if (e.pointerId !== pid) return; pid = null; el.classList.remove('on'); set(false); };
    on(el, 'pointerdown', e => {
      e.preventDefault();
      if (pid !== null) return;
      pid = e.pointerId;
      try { el.setPointerCapture(pid); } catch (_) { /* synthetic pointers */ }
      el.classList.add('on'); set(true);
    });
    on(el, 'pointerup', up); on(el, 'pointercancel', up); on(el, 'lostpointercapture', up);
  };

  // TUTORIAL GLOW (glow(ids)): a soft gold breathing ring on the controls the player needs now (tutorial level only, chosen by the
  // glue); each control stops glowing once used (tutUsed) or after TUT_MS. ids: key fourWD lockF lockR winch shifter wheel gas brake.
  const TUT_MS = 16000, tut = new Set();
  let tutT = null;
  const tutEl = id => id === 'wheel' ? (ui.steer === 'buttons' ? [els.left, els.right] : [wheelEl]) : id === 'key' ? [els.ignition] : id === 'shifter' ? [els.low] : [els[id]];
  const glow = (ids = []) => {
    for (const id of tut) for (const el of tutEl(id)) el?.classList.remove('tut');
    tut.clear(); clearTimeout(tutT);
    for (const id of ids) { tut.add(id); for (const el of tutEl(id)) el?.classList.add('tut'); }
    if (tut.size) tutT = setTimeout(() => glow([]), TUT_MS);
  };
  function tutUsed(id) {
    if (id === 'left' || id === 'right') id = 'wheel';
    if (!tut.has(id)) return;
    tut.delete(id); for (const el of tutEl(id)) el?.classList.remove('tut');
  }
  // reset: tap = recover in place, hold 0.8 s = restart level
  const reset = iconBtn(mk('btn small reset', '', zones.tl), 'reset', 'recover (tap) · restart (hold)');
  let resetTimer = null, resetFired = false;
  holdButton(reset, down => {
    if (down) {
      resetFired = false;
      resetTimer = setTimeout(() => { resetFired = true; reset.classList.remove('on'); handlers.onRestart && handlers.onRestart(); }, 800);
    } else {
      clearTimeout(resetTimer);
      if (!resetFired) handlers.onRecover && handlers.onRecover();
    }
  });

  on(menuBtn, 'pointerup', e => { e.preventDefault(); setPause(!paused); });
  for (const t of TOGGLES) {
    state[t.id] = false;
    const el = els[t.id] = mk(t.cls + (t.icon ? ' ico' : ''), t.icon ? '' : t.label, place[t.id]);
    if (t.icon) iconBtn(el, t.icon, t.label);
    on(el, 'pointerdown', e => { e.preventDefault(); state[t.id] = !state[t.id]; el.classList.toggle('on', state[t.id]); buzz('toggle'); tutUsed(t.id); if (t.id === 'fourWD' && !state.fourWD && state.lockF) { state.lockF = false; syncLocks(); } });
  }
  // AXLE LOCKS: one slim vertical button = a top-down truck (front up, like the gauge rig); tap cycles open -> REAR -> FRONT -> FRONT
  // + REAR -> open, the locked axle's wheels + bar glow amber, padlock in the body lights once any axle is locked. The front locker
  // needs 4WD: in 2WD the FRONT steps are skipped (front wheels blink red, back to open); 4WD off releases it.
  state.lockF = false; state.lockR = false;
  const locks = els.locks = els.lockF = els.lockR = mk('btn locks', '', zones.bc);
  locks.setAttribute('aria-label', 'axle lockers (tap: rear, front, front + rear, open) · keys 2 / 3'); locks.title = 'axle lockers';
  locks.innerHTML = '<span class="lk-body"></span><span class="lk-ax f"></span><span class="lk-ax r"></span>'
    + '<span class="lk-w wf fl"></span><span class="lk-w wf fr"></span><span class="lk-w wr rl"></span><span class="lk-w wr rr"></span>' + svg('padlock', 'lk-pad');
  function syncLocks() { locks.classList.toggle('lf', !!state.lockF); locks.classList.toggle('lr', !!state.lockR); locks.classList.toggle('on', !!(state.lockF || state.lockR)); }
  let lockBlinkT = null;
  // cycle (r13: an F-only step): open -> R -> F -> F+R -> open; 2WD: open -> R -> (front blinks red) open
  const lockStep = () => {
    const F = !!state.lockF, R = !!state.lockR;
    if (!F && !R) state.lockR = true;
    else if (R && !F) {
      if (state.fourWD) { state.lockF = true; state.lockR = false; }
      else { locks.classList.remove('nof'); void locks.offsetWidth; locks.classList.add('nof'); clearTimeout(lockBlinkT); lockBlinkT = setTimeout(() => locks.classList.remove('nof'), 700); state.lockR = false; }
    } else if (F && !R) state.lockR = true;
    else state.lockF = state.lockR = false;
    syncLocks(); buzz('toggle'); tutUsed('lockF'); tutUsed('lockR');
    shiftHook('drive', { id: 'locks', on: !!(state.lockF || state.lockR) });
  };
  on(locks, 'pointerdown', e => { e.preventDefault(); lockStep(); });
  // H-GATE SHIFTER: state.gate = requested slot ('H' 'L+' 'L' 'L-' 'N' 'R'; state.range / state.reverse mirror it for older glue).
  // The vehicle commits a slot with another range / direction only when stopped (v.pendingRange = the waiting slot -> the gate
  // pulses red); N is immediate. Drag the knob along the gate (it follows the slots, pointer capture) and let go = the nearest
  // slot; tap anywhere = jump to the slot nearest the tap. Sound / haptics: a slow drag ticks each slot it passes (quiet, at most
  // one at a time; fast swipes stay silent), and the gear engaging / waiting / neutral clunks come from the vehicle state edges
  // (audio shifts()), once per change — never one per slot.
  const shiftHook = (kind, info) => { try { handlers.onShift?.(kind, info); } catch (_) { /* audio is optional */ } };
  for (const t of TOGGLES) if (DRIVE.includes(t.id)) on(els[t.id], 'pointerdown', () => shiftHook('drive', { id: t.id, on: state[t.id] }));
  state.gate = 'H'; state.range = 'H'; state.low = false; state.reverse = false;
  const shifter = els.low = mk('shifter hgate', '', zones.br);
  shifter.setAttribute('role', 'slider'); shifter.setAttribute('aria-label', 'gear: L+ L L- N H R');
  const gateSvg = (() => {                          // slot path in a 100x100 box (stretched; strokes keep their width)
    const P = k => `${GATE_FX[GATE_POS[k][0]] * 100} ${GATE_FY[GATE_POS[k][1]] * 100}`;
    const d = GATE_SEGS.map(([a2, b2]) => `M${P(a2)}L${P(b2)}`).join('');
    return `<svg class="sh-gate" viewBox="0 0 100 100" preserveAspectRatio="none" aria-hidden="true"><path class="sh-slot-h" d="${d}"/><path class="sh-slot-o" d="${d}"/><path class="sh-slot-i" d="${d}"/></svg>`;
  })();
  shifter.insertAdjacentHTML('beforeend', gateSvg);
  const gateLbl = {};
  for (const k of SLOTS) {
    const [c, r] = GATE_POS[k], l = gateLbl[k] = mk('sh-lbl c' + c, k, shifter);
    l.style.left = `${GATE_FX[c] * 100}%`; l.style.top = `${GATE_FY[r] * 100}%`;
  }
  const knob = mk('sh-knob', '', shifter);
  mk('sh-pulse', '', shifter);                        // pending (moving) shift: slight red pulse (opacity only: compositor)
  let gW = 0, gH = 0;
  const gateBox = () => { gW = shifter.clientWidth || 100; gH = shifter.clientHeight || 90; };
  const nodeXY = k => [GATE_FX[GATE_POS[k][0]] * gW, GATE_FY[GATE_POS[k][1]] * gH];
  let knobXY = [0, 0];
  // tilt (deg): while dragged the knob leans into the push like a real lever (horizontal moves; vertical ones squash it a
  // touch), released = upright again via the snap transition
  let knobTilt = 0, knobSq = 0;
  const placeKnob = (x, y, lean = false) => {
    if (lean) { knobTilt = knobTilt * 0.55 + Math.max(-10, Math.min(10, (x - knobXY[0]) * 3)) * 0.45; knobSq = knobSq * 0.55 + Math.max(-0.08, Math.min(0.08, (y - knobXY[1]) * 0.025)) * 0.45; }
    else { knobTilt = 0; knobSq = 0; }
    knobXY = [x, y];
    knob.style.transform = `translate3d(${x.toFixed(1)}px, ${y.toFixed(1)}px, 0) rotate(${knobTilt.toFixed(1)}deg) scale(${(1 + Math.abs(knobSq) * 0.4).toFixed(3)}, ${(1 - Math.abs(knobSq)).toFixed(3)})`;
  };
  const knobTo = k => { gateBox(); const [x, y] = nodeXY(k); placeKnob(x, y); };
  let lastDrive = 'H', lastFwd = 'H';             // keys: N returns to the last drive slot, V from R to the last forward one
  const setGate = (g, quiet = false) => {
    if (!SLOTS.includes(g)) return;
    state.gate = g; state.reverse = g === 'R';
    if (g !== 'N') lastDrive = g; if (RANGES.includes(g)) lastFwd = g;
    if (g !== 'N') state.range = g === 'R' ? 'L+' : g;
    state.low = state.range !== 'H';
    if (!drag) knobTo(g);
    shifter.setAttribute('aria-valuetext', g);
    shifter.classList.toggle('low', state.low && g !== 'N' && g !== 'R');
    for (const k of SLOTS) gateLbl[k].classList.toggle('req', k === g);
    if (!quiet) tutUsed('shifter');
  };
  const stepRange = (d, wrap = false) => {
    const i = RANGES.indexOf(state.gate) < 0 ? RANGES.indexOf(state.range) : RANGES.indexOf(state.gate), n = RANGES.length, j = i + d;
    if (wrap || (j >= 0 && j < n)) setGate(RANGES[(j + n) % n]);
  };
  const cycleRange = (d = 1) => stepRange(d, true);
  // drag along the gate graph: the knob sits on one segment (si, t 0..1); each finger sub-step projects onto that segment and,
  // near an end, onto the segments sharing that end -> it can only turn at the junctions, like a real gate
  const segXY = i => [nodeXY(GATE_SEGS[i][0]), nodeXY(GATE_SEGS[i][1])];
  const project = (i, x, y) => {
    const [[ax, ay], [bx, by]] = segXY(i), dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy || 1;
    const t = Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / L2)), px = ax + dx * t, py = ay + dy * t;
    return { i, t, x: px, y: py, d: Math.hypot(x - px, y - py) };
  };
  let drag = null;                                    // { pid, si, t, fx, fy, moved, last: slot passed, lt: ms of the last move }
  const startDrag = (pid, x, y) => {
    gateBox();
    const si = GATE_SEGS.findIndex(sg => sg.includes(state.gate));
    drag = { pid, si, t: GATE_SEGS[si][0] === state.gate ? 0 : 1, fx: x, fy: y, x0: x, y0: y, moved: false, last: state.gate, lt: performance.now() };
  };
  const dragTo = (x, y) => {
    const D = drag, n = Math.max(1, Math.ceil(Math.hypot(x - D.fx, y - D.fy) / 5)), now = performance.now();
    const speed = Math.hypot(x - D.fx, y - D.fy) / Math.max(1, now - D.lt);
    for (let k = 1; k <= n; k++) {
      const qx = D.fx + (x - D.fx) * k / n, qy = D.fy + (y - D.fy) * k / n;
      const seg = GATE_SEGS[D.si], ends = [];
      if (D.t < 0.25) ends.push(seg[0]);
      if (D.t > 0.75) ends.push(seg[1]);
      let best = project(D.si, qx, qy);
      GATE_SEGS.forEach((sg, i) => { if (i !== D.si && ends.some(e => sg.includes(e))) { const p2 = project(i, qx, qy); if (p2.d < best.d - 0.5) best = p2; } });
      D.si = best.i; D.t = best.t;
      placeKnob(best.x, best.y, true);
      for (const sl of SLOTS) {                       // passing a slot: quiet tick + crisp haptic (slow, deliberate drags only)
        const [nx, ny] = nodeXY(sl);
        if (sl !== D.last && Math.hypot(best.x - nx, best.y - ny) < 3.5) {
          D.last = sl;
          if (speed < TICK_SPEED) { shiftHook('tick', { slot: sl }); buzz('tick'); }
        }
      }
    }
    D.fx = x; D.fy = y; D.lt = now;
  };
  const nearestSlot = (x, y) => { let b = null, bd = 1e9; for (const sl of SLOTS) { const [nx, ny] = nodeXY(sl), d = Math.hypot(x - nx, y - ny); if (d < bd) { bd = d; b = sl; } } return [b, bd]; };
  const local = e => { const r = shifter.getBoundingClientRect(); return [e.clientX - r.left - shifter.clientLeft, e.clientY - r.top - shifter.clientTop]; };
  on(shifter, 'pointerdown', e => {
    e.preventDefault();
    if (drag) return;
    const [x, y] = local(e);
    startDrag(e.pointerId, x, y);
    try { shifter.setPointerCapture(e.pointerId); } catch (_) { /* synthetic pointers */ }
    shifter.classList.add('grab');
  });
  on(shifter, 'pointermove', e => {
    if (!drag || e.pointerId !== drag.pid) return;
    e.preventDefault();
    const [x, y] = local(e);
    if (!drag.moved) {
      if (Math.hypot(x - drag.x0, y - drag.y0) < 6) return;
      const [kx, ky] = knobXY;                        // a drag starts on the knob (or near it); elsewhere a slide = a tap where it ends
      if (Math.hypot(drag.x0 - kx, drag.y0 - ky) > Math.max(22, gW * 0.22)) return;
      drag.moved = true; drag.fx = kx; drag.fy = ky; shifter.classList.add('drag');
    }
    dragTo(x, y);
  });
  const shUp = e => {
    if (!drag || e.pointerId !== drag.pid) return;
    const D = drag; drag = null;
    shifter.classList.remove('grab', 'drag');
    if (e.type === 'pointercancel' || e.type === 'lostpointercapture') { knobTo(state.gate); return; }
    if (D.moved) { const [sl] = nearestSlot(...knobXY); setGate(sl); knobTo(sl); return; }
    const [x, y] = local(e), [sl, d] = nearestSlot(x, y);   // tap: jump to the slot nearest the tap
    if (d < Math.max(gW, gH) * 0.45 && sl !== state.gate) setGate(sl); else knobTo(state.gate);
  };
  on(shifter, 'pointerup', shUp); on(shifter, 'pointercancel', shUp); on(shifter, 'lostpointercapture', shUp);
  on(window, 'resize', () => requestAnimationFrame(() => { if (!drag) knobTo(state.gate); }));   // vmin-sized gate: re-measure
  let committed = null;                                   // last v.gate seen by update() (null = fresh run: no commit haptic)
  // IGNITION KEY (left of 4WD): tap / key K = one key turn (state.ignition++): engine off or stalled -> crank, running -> shut off.
  // update() sets .off (pulsing amber: engine off / stalled), .crank (key turned, starter running) or .run (green); icon only.
  const keyBtn = els.ignition = mk('btn key off', '', zones.bc);
  zones.bc.insertBefore(keyBtn, zones.bc.firstChild);
  keyBtn.setAttribute('role', 'button'); keyBtn.setAttribute('aria-label', 'ignition key');
  // ignition switch seen face-on: fixed barrel ring with OFF / ON / START marks, the key head turns about the centre
  // (off upright, run +60°, crank +90° wiggling) -> centred in every position
  keyBtn.innerHTML = '<svg class="key-sw" viewBox="0 0 40 40" aria-hidden="true"><circle class="ks-ring" cx="20" cy="20" r="15.5"/>'
    + '<circle class="ks-m m0" cx="20" cy="2.6" r="1.5"/><circle class="ks-m m1" cx="35.1" cy="11.3" r="1.5"/><circle class="ks-m m2" cx="37.4" cy="20" r="1.5"/>'
    // slim key: round-cornered bow with a ring hole, shoulder stops, a long blade with 3 cut teeth + groove (centred on 20 20)
    + '<g class="key-ico"><path d="M18.3 7.4h3.4a3.1 3.1 0 0 1 3.1 3.1v3.8a3.1 3.1 0 0 1-3.1 3.1v1.3h.9v1.3h-.9v1.5h-1.2l1.2 1.4v1.5h-1.2l1.2 1.4v1.5h-1.2l1.2 1.4v1.8l-1.7 2.1-1.7-1.5V20h-.9v-1.3h.9v-1.3a3.1 3.1 0 0 1-3.1-3.1v-3.8a3.1 3.1 0 0 1 3.1-3.1z"/>'
    + '<circle class="key-hole" cx="20" cy="11.4" r="1.7"/><path class="key-grip" d="M19.6 21.2v8.6"/></g></svg>';
  let keyMode = 'off';
  const turnKey = () => {
    state.ignition = (state.ignition | 0) + 1;
    keyBtn.classList.remove('turn'); void keyBtn.offsetWidth; keyBtn.classList.add('turn');
    shiftHook('key', { n: state.ignition, mode: keyMode });
    buzz('toggle'); tutUsed('key');
  };
  on(keyBtn, 'pointerdown', e => { e.preventDefault(); turnKey(); });
  // fresh run / new truck: 2WD, lockers open, range H, engine-off key, pause panel closed; DOM classes synced immediately (sync ring cleared).
  // DEBUG / MUTE / CAM / sizes are settings and survive runs.
  const resetControls = () => {
    state.ignition = 0;
    for (const t of TOGGLES) { state[t.id] = false; els[t.id].classList.remove('on', 'pending', 'blink'); }
    state.lockF = state.lockR = false; syncLocks(); locks.classList.remove('nof');
    shifter.classList.remove('sync');
    for (const k of SLOTS) gateLbl[k].classList.remove('cur');
    committed = null;
    setGate('H', true);
    glow([]);
    if (paused) setPause(false);
  };
  // SHARED STEERING PIPELINE: state.steer = clamp(analog source of the active mode + right − left). Sources: steerIn.wheel (virtual
  // wheel), steerIn.tilt (phone tilt); BUTTONS mode has no analog source (◄ ► / A D ← → only). Keys work in every mode.
  // Steering also flips the camera to 2.5D (the lateral trail is only visible there); CAM: 2D stays a manual toggle.
  const steerIn = { wheel: 0, tilt: 0 };
  let wheelEl = null, wheelRot = null, wheelShown = 0, wheelBack = 1, wheelVis = 0;   // wheelBack: rim follows the tyres (0 = grabbed); wheelVis: drawn-only rim offset
  const syncSteer = () => {
    const analog = ui.steer === 'buttons' ? 0 : steerIn[ui.steer] || 0;
    state.steer = Math.max(-1, Math.min(1, analog + state.right - state.left));
    if (Math.abs(state.steer) > 0.05 && state.cam !== '2.5D') setCam('2.5D');
    const a = Math.round(Math.max(-1, Math.min(1, state.steer + wheelVis)) * wheelLock(ui) * 57.2958);
    if (wheelRot && a !== wheelShown) { steerTick(a - wheelShown, follow); wheelShown = a; wheelRot.style.transform = `rotate(${a}deg)`; }
  };
  // r15 steering feel (WHEEL only; BUTTONS = none, TILT = tiltFeel in onSensor): a faint micro-tick every STEER_DETENT deg the
  // player turns the rim (rate follows the turn speed; haptics cool 55 ms = ≤ 18 Hz, never a buzz), one notch crossing 0°,
  // a firmer bump stop at full lock. Reversing restarts the tick count; the rim easing back by itself (update() follow) is silent.
  // Pulses get a little firmer with load (steerLoad: stopped / dry-steer, F-LOCK), lighter while rolling.
  let follow = false, tickAcc = 0, tickDir = 0, rimSide = 0, rimLock = false, steerLoad = 1;
  const STEER_DETENT = 22;
  const steerTick = (d, quiet) => {
    const s = state.steer + wheelVis, m = Math.abs(s), side = m < 0.04 ? 0 : Math.sign(s);
    const live = !quiet && !paused && d && ui.steer === 'wheel';
    const centre = rimSide && side !== rimSide, lock = m >= 0.995 && !rimLock;
    if (centre || side) rimSide = side;
    if (m >= 0.995) rimLock = true; else if (m < 0.9) rimLock = false;
    if (!live) { tickAcc = 0; return; }
    const dir = Math.sign(d);
    if (dir !== tickDir) { tickDir = dir; tickAcc = 0; }
    tickAcc += Math.abs(d);
    if (centre) { buzz('steerCenter', steerLoad); tickAcc = 0; return; }
    if (lock) { buzz('steerLock', steerLoad); tickAcc = 0; return; }
    if (tickAcc < STEER_DETENT) return;
    tickAcc %= STEER_DETENT;
    buzz('steer', steerLoad);
  };
  // momentary buttons: touch and keyboard tracked apart (state = either, never while paused), so letting go of one source
  // while the other still holds keeps the control down; the .on class follows the combined state
  const src = {};
  const press = (id, down, from = 'touch') => {
    const s = src[id] || (src[id] = { touch: false, key: false });
    s[from] = !!down;
    state[id] = (s.touch || s.key) && !paused ? 1 : 0;
    els[id]?.classList.toggle('on', state[id] > 0);
    if (id === 'left' || id === 'right') syncSteer();
  };
  // analog pedals (GAS / BRAKE): vertical finger slide, bottom edge = PEDAL_MIN (light feather) -> top = 1 (floored);
  // touch level and keyboard (0|1) are tracked separately, state[id] = max of both; fill bar drawn in update()
  const PEDAL_MIN = 0.18, PEDAL_EDGE = 0.08;          // bottom/top 8 % of the pad clamp to MIN / 1 (easy full press)
  const pedals = { gas: { touch: 0, key: 0 }, brake: { touch: 0, key: 0 } }, pedalUi = {};
  const setPedal = (id, src, val) => {
    const p = pedals[id]; p[src] = paused ? 0 : val;
    state[id] = Math.max(p.touch, p.key);
    els[id].classList.toggle('on', state[id] > 0);
  };
  const pedalAt = (el, y) => {
    const r = el.getBoundingClientRect();
    const f = Math.min(1, Math.max(0, ((r.bottom - y) / (r.height || 1) - PEDAL_EDGE) / (1 - 2 * PEDAL_EDGE)));
    return PEDAL_MIN + (1 - PEDAL_MIN) * f;
  };
  const slidePedal = (el, id) => {
    let pid = null;
    const move = e => { if (e.pointerId === pid) { e.preventDefault(); setPedal(id, 'touch', pedalAt(el, e.clientY)); } };
    const up = e => { if (e.pointerId !== pid) return; pid = null; setPedal(id, 'touch', 0); };
    on(el, 'pointerdown', e => {
      e.preventDefault();
      if (pid !== null) return;
      pid = e.pointerId;
      try { el.setPointerCapture(pid); } catch (_) { /* synthetic pointers */ }
      setPedal(id, 'touch', pedalAt(el, e.clientY));
    });
    on(el, 'pointermove', move);
    on(el, 'pointerup', up); on(el, 'pointercancel', up); on(el, 'lostpointercapture', up);
  };
  for (const m of MOMENTARY) {
    const el = els[m.id] = mk(m.cls, m.icon ? '' : m.label, place[m.id]);
    if (m.id === 'left' || m.id === 'right') el.setAttribute('aria-label', m.id === 'left' ? 'steer left' : 'steer right');
    if (pedals[m.id]) {
      pedalUi[m.id] = { fill: mk('pedal-fill', '', el), lbl: mk('pedal-lbl', '', el), v: -1 };
      el.setAttribute('aria-label', m.label); el.title = m.label;   // no pictogram: green / red tinted pads + tread ribs read as the pedals
      slidePedal(el, m.id);
    } else {
      if (m.icon) iconBtn(el, m.icon, m.label);
      holdButton(el, down => { press(m.id, down, 'touch'); if (down) { tutUsed(m.id); if (m.id === 'winch') buzz('toggle'); } });
    }
  }
  // VIRTUAL STEERING WHEEL (MENU -> SETTINGS: wheel | ◄ ► buttons): grab anywhere on the rim and turn; angle accumulates across the
  // ±180° seam, clamps at ±WHEEL_MAX (= full lock, analog steer −1..1) and springs back to centre on release. Keys still add ±1.
  wheelEl = mk('steer-wheel', '', steerBox);
  wheelEl.setAttribute('role', 'slider'); wheelEl.setAttribute('aria-label', 'steering wheel');
  // per-rig wheel art (engine/wheels.js): rotating body + cap (own layer: horn press scales it) and a fixed gloss overlay
  wheelEl.innerHTML = '<div class="sw-rot"></div><div class="sw-gloss-l"></div><div class="sw-pulse"></div><div class="sw-badge">' + svg('tilt') + '</div>';
  let rigKey = '';
  const setRig = (style, paint) => {
    const k = (style || 'jeep') + '|' + (paint || '');
    if (k === rigKey) return;
    rigKey = k;
    const w = wheelSvg(style, paint);
    wheelEl.firstChild.innerHTML = w.body + w.cap;
    wheelEl.children[1].innerHTML = w.gloss;
    wheelEl.dataset.rig = style || 'jeep';
  };
  setRig('jeep', '#5f6e3d');
  wheelRot = wheelEl.firstChild;
  {
    let pid = null, prev = 0, ang = 0;
    const polar = e => {
      const r = wheelEl.getBoundingClientRect(), dx = e.clientX - (r.left + r.width / 2), dy = e.clientY - (r.top + r.height / 2);
      return { a: Math.atan2(dy, dx), k: Math.hypot(dx, dy) / (r.width / 2 || 1) };
    };
    let arm = null, honkAt = 0, rimOff = -1e9;            // arm: WHEEL cap touch {x, y, t0, t (guard timer)}; honkAt: honk start
    const disarm = () => { if (arm) { clearTimeout(arm.t); arm = null; } };
    const honk = () => { honkAt = performance.now(); setHorn('touch', true); };
    const release = e => {
      if (e.pointerId === hornPid) {
        hornPid = null;
        const up = e.type === 'pointerup', pend = !!arm;
        disarm();
        if (up && pend) honk();                           // released inside the guard: still a beep
        const left = HORN_TAP - (performance.now() - honkAt);
        clearTimeout(tapT);
        if (up && state.horn && left > 0) tapT = setTimeout(() => setHorn('touch', false), left);   // quick taps: a short beep
        else setHorn('touch', false);
        return;
      }
      if (e.pointerId !== pid) return;
      rimOff = performance.now();
      pid = null; ang = 0; wheelVis = steerIn.wheel; steerIn.wheel = 0; wheelBack = performance.now(); wheelEl.classList.remove('grab'); syncSteer();
      // input lets go at once (a parked truck keeps its wheel angle); the drawn rim follows the tyres' angle, in update()
    };
    let swipeFrom = null;                                 // cap touch origin for the swipe test {x, y, t0}
    on(wheelEl, 'pointerdown', e => {
      e.preventDefault();
      // centre cap = HORN in every mode (TILT: the wheel is only a live indicator, a bigger cap zone). A cap press never
      // touches the steer input; dragging out past HORN_SLIDE turns it into a normal rim grab from that point (no jump)
      const p0 = polar(e);
      if (hornPid === null && !paused && p0.k <= (ui.steer === 'tilt' ? HORN_HUB : HORN_CAP)
          && !(ui.steer !== 'tilt' && pid === null && performance.now() - rimOff < HORN_RECENT)) {   // just let go of the rim: next swipe
        hornPid = e.pointerId; disarm(); clearTimeout(tapT);
        if (ui.steer === 'tilt' || pid !== null) honk();  // TILT (no steering here) / 2nd finger while steering: honk at once
        else {
          const a = arm = { x: e.clientX, y: e.clientY, t0: performance.now(), t: 0 };
          a.t = setTimeout(() => { if (arm === a) { arm = null; if (hornPid !== null) honk(); } }, HORN_ARM);
        }
        try { wheelEl.setPointerCapture(hornPid); } catch (_) { /* synthetic pointers */ }
        if (ui.steer !== 'tilt') swipeFrom = { x: e.clientX, y: e.clientY, t0: performance.now() };
        return;
      }
      if (ui.steer === 'tilt') return;
      if (pid !== null || paused) return;
      grab(e, p0);
    });
    const grab = (e, p) => {
      pid = e.pointerId; prev = p.k < 0.25 ? NaN : p.a; steerIn.wheel = wheelVis; wheelVis = 0; ang = steerIn.wheel * wheelLock(ui); wheelBack = 0; tutUsed('wheel');
      try { wheelEl.setPointerCapture(pid); } catch (_) { /* synthetic pointers */ }
      wheelEl.classList.add('grab');
    };
    on(wheelEl, 'pointermove', e => {
      if (e.pointerId === hornPid) {                      // honking thumb slides off the cap onto the rim: hand over to steering
        if (ui.steer === 'tilt' || pid !== null || paused) return;
        const p = polar(e), sf = swipeFrom;
        // the thumb moved soon after landing = a swipe that started on the cap: cut the horn (near-silent) and steer from here
        const swipe = sf && performance.now() - sf.t0 < HORN_SWIPE && Math.hypot(e.clientX - sf.x, e.clientY - sf.y) > HORN_MOVE * (wheelEl.offsetWidth / 2);
        if (swipe || p.k > HORN_SLIDE) {
          disarm(); hornPid = null; swipeFrom = null; clearTimeout(tapT);
          if (swipe && state.horn) { hornSrc.touch = false; cutHorn(); } else setHorn('touch', false);
          grab(e, p);
        }
        return;
      }
      if (e.pointerId !== pid) return;
      e.preventDefault();
      const p = polar(e);
      if (p.k < 0.2) return;                              // too close to the hub: angle is noise
      if (Number.isNaN(prev)) { prev = p.a; return; }     // grabbed near the hub (swipe off the cap): first usable angle
      let d = p.a - prev; prev = p.a;
      if (d > Math.PI) d -= 2 * Math.PI; else if (d < -Math.PI) d += 2 * Math.PI;
      const L = wheelLock(ui);
      ang = Math.max(-L, Math.min(L, ang + d));
      steerIn.wheel = paused ? 0 : ang / L; syncSteer();
    });
    on(wheelEl, 'pointerup', release); on(wheelEl, 'pointercancel', release); on(wheelEl, 'lostpointercapture', release);
  }
  // PHONE TILT = gravity-vector roll (engine/tilt.js): devicemotion accelerationIncludingGravity (preferred; iOS's inverted
  // sign is detected) or, where that brings no data, the up vector of deviceorientation(absolute) beta / gamma (first of the two
  // that delivers; motion takes over as soon as it delivers). Device frame -> screen axes by the screen angle (either landscape
  // side), roll in the screen plane = the same however upright / reclined the phone is held: level screen = straight, no
  // re-center. 1€-filtered vector -> soft deadzone + expo curve (tiltSteer, full lock at ui.tiltMax°) -> light low-pass ->
  // steerIn.tilt. requestPermission (iOS 13+, recent Chrome) is asked at once: Chrome grants without a prompt, iOS rejects
  // without a gesture -> asked again from the next tap's touchend / click. Falls back to WHEEL (saved, toast) only on genuine
  // errors: no motion / orientation API, permission denied / blocked, or within TILT_WAIT ms of *visible* time every source
  // the browser has reporting empty samples (no sensor) or http (needs HTTPS). A slow sensor gets a "waiting" toast; after a
  // plain timeout the listener keeps probing (WHEEL meanwhile) and late data switches TILT back.
  const TILT_EVENTS = ['devicemotion', 'deviceorientation', 'deviceorientationabsolute'];
  const tilt = { on: false, live: false, f: 0, t: null, timers: [], waitTap: false, from: null, nulls: new Set(), probe: false, rejects: 0,
    filt: new TiltFilter(), dz: 0, peak: 0 };
  const screenAngle = () => { try { const a = screen.orientation?.angle ?? window.orientation ?? 0; return ((+a || 0) % 360 + 360) % 360; } catch (_) { return 0; } };
  const tiltClear = () => { tilt.timers.forEach(clearTimeout); tilt.timers.length = 0; };
  const fin = v => v != null && Number.isFinite(+v);
  const onSensor = e => {
    let x, y, z;
    if (e.type === 'devicemotion') {
      const g2 = e.accelerationIncludingGravity;
      if (!g2 || !fin(g2.x) || !fin(g2.y) || !fin(g2.z) || Math.hypot(+g2.x, +g2.y, +g2.z) < 1) { tilt.nulls.add(e.type); return; }
      x = +g2.x; y = +g2.y; z = +g2.z;
      if (tilt.from !== e.type) for (const t of TILT_EVENTS) if (t !== e.type) window.removeEventListener(t, onSensor);   // motion wins
    } else {
      if (tilt.from && e.type !== tilt.from) return;             // motion, or the first orientation source that delivered
      if (!fin(e.beta) || !fin(e.gamma)) { tilt.nulls.add(e.type); return; }
      [x, y, z] = upFromEuler(+e.beta, +e.gamma);
    }
    tilt.from = e.type;
    const now = performance.now() / 1000, dt = tilt.t == null ? 0 : Math.min(0.25, Math.max(0, now - tilt.t));
    tilt.t = now;
    const roll = tilt.filt.push(x, y, z, now, screenAngle(), e.type === 'devicemotion');
    if (!tilt.live) {
      tilt.live = true; tiltClear();
      if (tilt.probe) { tilt.probe = false; ui.steer = 'tilt'; applyUi(true); toast('TILT ACTIVE', 1600); }
    }
    if (ui.steer !== 'tilt') return;
    const target = tiltSteer(roll, ui.tiltMax);
    tilt.f = dt > 0 ? tilt.f + (target - tilt.f) * (1 - Math.exp(-dt / TILT.outTau)) : target;
    if (target === 0 && Math.abs(tilt.f) < 0.01) tilt.f = 0;
    if (Math.abs(target) > 0.3) tutUsed('wheel');               // tutorial "steer" glow: tilting counts
    // TILT feel (r15, no rim ticks: holding an angle stays quiet): one notch crossing 0° (settling into the deadzone after a
    // real turn, or swinging straight through), a bump stop at full lock (re-armed below 0.9), and while dry-steering
    // (|v| < 0.3 m/s) or with F-LOCK a faint resistance pulse per 0.25 of steer travel. Never while paused.
    const m = Math.abs(target), dz = m === 0 ? 0 : m >= 0.995 || (tilt.dz === 2 && m > 0.9) ? 2 : 1, sd = Math.sign(target);
    if (!paused) {
      if ((dz === 0 && tilt.dz !== 0 && tilt.peak >= 0.25) || (sd && tilt.sd && sd !== tilt.sd)) buzz('steerCenter', steerLoad);
      if (dz === 2 && tilt.dz !== 2) buzz('steerLock', steerLoad);
      if (steerLoad >= 0.7) {
        tilt.acc = (tilt.acc || 0) + Math.abs(target - (tilt.lastT ?? target));
        if (tilt.acc >= 0.25 && dz !== 2) { tilt.acc = 0; buzz('steerResist', steerLoad); }
      } else tilt.acc = 0;
    }
    tilt.lastT = target;
    if (sd) tilt.sd = sd;
    if (dz !== tilt.dz) { if (dz === 0) { tilt.peak = 0; tilt.sd = 0; } tilt.dz = dz; }
    tilt.peak = Math.max(tilt.peak || 0, m);
    const out = paused ? 0 : Math.round(tilt.f * 1000) / 1000;
    if (out !== steerIn.tilt) { steerIn.tilt = out; syncSteer(); }
  };
  const stopTilt = () => {
    if (tilt.on) for (const t of TILT_EVENTS) window.removeEventListener(t, onSensor);
    tiltClear();
    Object.assign(tilt, { on: false, live: false, f: 0, t: null, waitTap: false, from: null, probe: false, dz: 0, peak: 0 });
    tilt.nulls.clear(); tilt.filt.reset();
    steerIn.tilt = 0; syncSteer();
  };
  const failTilt = msg => { stopTilt(); toast(`${msg} · using WHEEL`, 3200); ui.steer = 'wheel'; applyUi(true); };
  const giveUp = () => {                                         // TILT_WAIT ms visible, no gravity / beta / gamma yet
    if (tilt.live || !tilt.on) return;
    if (!window.isSecureContext) return failTilt('TILT NEEDS HTTPS');
    const srcs = TILT_EVENTS.filter(t => ('on' + t) in window);
    if (srcs.length && srcs.every(t => tilt.nulls.has(t))) return failTilt('NO TILT SENSOR');   // every source fires, all empty
    tilt.probe = true; tiltClear();                              // silent so far: maybe slow; keep listening, WHEEL meanwhile
    toast('NO TILT DATA YET · using WHEEL (switches back when it arrives)', 3200);
    ui.steer = 'wheel'; applyUi(true);
  };
  const armTilt = () => {                                        // watchdog counts visible time only (background tab: no events)
    tiltClear();
    if (!tilt.on || tilt.live || tilt.probe || document.hidden) return;
    tilt.timers.push(setTimeout(() => { if (tilt.on && !tilt.live) toast('TILT · waiting for sensor…', 2000); }, TILT_HINT),
      setTimeout(giveUp, TILT_WAIT));
  };
  const goTilt = () => {
    if (ui.steer !== 'tilt' || tilt.on) return;
    for (const t of TILT_EVENTS) window.addEventListener(t, onSensor);
    Object.assign(tilt, { on: true, live: false, from: null, probe: false, t: null, f: 0 });
    tilt.nulls.clear(); tilt.filt.reset();
    armTilt();
  };
  // permission: motion first (the gravity source), then orientation (the fallback) chained after a grant: iOS shows ONE
  // Motion & Orientation prompt, and a second request in the same gesture may be refused
  const permApis = () => [window.DeviceMotionEvent, window.DeviceOrientationEvent].filter(C => C && typeof C.requestPermission === 'function');
  const askPerm = C => { try { return Promise.resolve(C.requestPermission()); } catch (err) { return Promise.reject(err); } };
  const askTilt = gesture => {                                   // gesture: from a touchend / click (iOS prompts only then)
    tilt.waitTap = false;
    const [first, second] = permApis();
    askPerm(first).then(r => {
      if (ui.steer !== 'tilt') return;
      if (r !== 'granted') return failTilt('TILT DENIED · allow Motion & Orientation access');
      tilt.rejects = 0; goTilt();
      if (second) askPerm(second).catch(() => {});              // orientation fallback: best effort
    }, () => {
      if (ui.steer !== 'tilt' || tilt.on) return;
      if (gesture && ++tilt.rejects >= 3) return failTilt(window.isSecureContext ? 'TILT BLOCKED' : 'TILT NEEDS HTTPS');
      tilt.waitTap = true; toast('TAP THE SCREEN TO ENABLE TILT', 2800);
    });
  };
  const startTilt = () => {
    if (tilt.on) return;
    if (typeof window.DeviceMotionEvent === 'undefined' && typeof window.DeviceOrientationEvent === 'undefined')
      return failTilt(window.isSecureContext ? 'TILT NOT SUPPORTED' : 'TILT NEEDS HTTPS');
    if (permApis().length) askTilt(false); else goTilt();
  };
  // HORN: TILT wheel centre cap (touch) or H (key), tracked apart like the momentary buttons; never while paused
  let hornPid = null, tapT = 0;
  const hornSrc = { touch: false, key: false };
  state.horn = false;
  const setHorn = (from, down) => {
    hornSrc[from] = !!down;
    const h = (hornSrc.touch || hornSrc.key) && !paused;
    if (h === state.horn) return;
    state.horn = h; wheelEl.classList.toggle('horn', h);
    if (h) buzz('tick');
    try { handlers.onHorn?.(h); } catch (_) { /* audio optional */ }
  };
  // swipe off the cap: horn off with the fast fade (unless the H key still holds it)
  const cutHorn = () => {
    if (hornSrc.key || !state.horn) return setHorn('touch', false);
    state.horn = false; wheelEl.classList.remove('horn');
    try { handlers.onHorn?.(false, true); } catch (_) { /* audio optional */ }
  };
  for (const t of ['touchend', 'click']) on(window, t, () => { if (tilt.waitTap && ui.steer === 'tilt') askTilt(true); }, true);
  on(document, 'visibilitychange', () => { if (document.hidden) { tiltClear(); releaseAll(); } else armTilt(); });
  const setSteerMode = m => {
    if (!STEER_MODES.includes(m) || (m === ui.steer && !tilt.probe)) return;
    if (ui.steer === 'tilt' || tilt.on || tilt.waitTap) stopTilt();   // an explicit pick also ends a background probe
    if (hornPid !== null) { hornPid = null; setHorn('touch', false); }
    steerIn.wheel = 0; wheelVis = 0; ui.steer = m; applyUi(true); syncSteer();
    if (m === 'tilt') { tilt.rejects = 0; startTilt(); }      // gravity roll: level screen = straight, nothing to calibrate
  };
  // keyboard routes pedals through setPedal (full press), everything else through press(.., 'key'). held[id] = codes down
  // for that control (A and ← both steer left: letting go of one keeps steering)
  const held = {};
  for (const m of MOMENTARY) held[m.id] = new Set();
  const keyPress = (id, down) => { if (pedals[id]) setPedal(id, 'key', down ? 1 : 0); else press(id, down, 'key'); };
  const keyHold = (id, code, down) => { const h = held[id]; down ? h.add(code) : h.delete(code); keyPress(id, h.size > 0); };
  const releaseKeys = () => { for (const m of MOMENTARY) { held[m.id].clear(); keyPress(m.id, false); } setHorn('key', false); };
  const drawPedal = (id, val) => {
    const u = pedalUi[id], x = Math.round(Math.min(1, Math.max(0, +val || 0)) * 100) / 100;
    if (x !== u.v) { u.v = x; u.fill.style.transform = `scaleY(${x})`; }
  };
  const releaseAll = () => {
    releaseKeys();
    for (const m of MOMENTARY) pedals[m.id] ? setPedal(m.id, 'touch', 0) : press(m.id, false, 'touch');
    hornPid = null; setHorn('touch', false);
    steerIn.wheel = 0; wheelVis = 0; steerIn.tilt = 0; syncSteer();
  };

  // PAUSE PANEL (MENU / Esc): sim paused via handlers.onPause. Icon-first, no words: main view = big ▶ RESUME (✕ on the start
  // screen) + map LEVELS, then a tile grid: STEERING › (badge = mode) · RESIZE HUD › (dot = custom) · GRAPHICS (tap cycles AUTO ->
  // LOW -> MED -> HIGH: 1-3 dots, auto badge) · SOUND › (speaker / muted) · CAM (cube 2.5D / square 2D) · SPOTTER (eye: AUTO badge
  // -> ON -> OFF) · DEBUG (bug). Sub-pages: ‹ back + page icon; sliders carry an icon label and a numeric value.
  const pauseEl = mk('pause', '', root), card = mk('pause-card', '', pauseEl);
  const view = k => mk('p-view ' + k, '', card);
  const pMain = view('main'), pSteer = view('steer'), pSize = view('size'), pSound = view('sound');
  const setView = k => { card.dataset.view = k; };
  setView('main');
  const pBtn = (parent, cls, label, fn, icon) => {
    const b2 = mk('btn p-btn ' + cls, icon ? '' : label, parent);
    if (icon) { b2.innerHTML = svg(icon); b2.setAttribute('aria-label', label); b2.title = label; }
    on(b2, 'pointerup', e => { e.preventDefault(); fn(b2); }); return b2;
  };
  const setIcon = (el, name) => { if (el._ic !== name) { el._ic = name; const old = el.querySelector(':scope > svg.ic'); if (old) old.outerHTML = svg(name); else el.insertAdjacentHTML('afterbegin', svg(name)); } };
  const row1 = mk('p-row top', '', pMain);
  const resumeBtn = pBtn(row1, 'resume', 'resume (Enter)', () => setPause(false), 'play');
  const levelsBtn = pBtn(row1, 'levels', 'levels', () => { setPause(false); handlers.onMenu?.(); }, 'map');
  const grid = mk('p-grid', '', pMain);
  const tile = (cls, label, icon, fn) => { const t = pBtn(grid, 'tile ' + cls, label, fn, icon); return { el: t, badge: mk('p-badge', '', t, 'span') }; };
  const steerT = tile('steer', 'steering', 'wheel', () => setView('steer'));
  const sizeT = tile('size', 'resize controls', 'resize', () => setView('size'));
  const gfxT = tile('gfx', 'graphics quality', 'gfx', () => setUi('gfx', GFX_MODES[(GFX_MODES.indexOf(ui.gfx) + 1) % GFX_MODES.length]));
  const soundT = tile('sound', 'sound', 'speaker', () => setView('sound'));
  // camera toggle: icon shows the live mode (2.5D oblique cube <-> flat 2D square)
  state.cam = '2.5D';
  const camT = tile('cam', 'camera 2.5D / 2D (C)', 'cam3', () => setCam(state.cam === '2D' ? '2.5D' : '2D'));
  const setCam = m => { m = '2.5D'; state.cam = m; setIcon(camT.el, m === '2D' ? 'cam2' : 'cam3'); camT.el.classList.toggle('on', m === '2.5D'); handlers.onCam?.(m); };
  setCam(state.cam); camT.el.style.display = 'none';
  const nextSpot = () => setUi('spot', SPOT_UI[(SPOT_UI.indexOf(ui.spot) + 1) % SPOT_UI.length]);
  const spotT = tile('spot', 'spotter view: auto / on / off (O)', 'eye', nextSpot);
  const setOption = (id, v) => {
    state[id] = v; els[id]?.classList.toggle('on', v);
    if (id === 'mute') { setIcon(soundT.el, v ? 'mute' : 'speaker'); soundT.el.classList.toggle('muted', v); }
  };
  els.debug = tile('debug', 'debug overlay (G)', 'bug', () => setOption('debug', !state.debug)).el; els.debug.style.display = 'none';
  const subHead = (parent, label, icon) => {
    const r = mk('p-head', '', parent);
    pBtn(r, 'back', 'back', () => setView('main'), 'back');
    const t2 = mk('p-title', '', r); t2.innerHTML = svg(icon); t2.setAttribute('aria-label', label); return r;
  };
  const slider = (parent, key, label, step, fmt, icon) => {
    const row = mk('set-row', '', parent), [lo, hi] = UI_RANGE[key];
    const lb = mk('set-lbl', '', row); lb.innerHTML = svg(icon); lb.title = label; row.setAttribute('aria-label', label);
    const track = mk('set-track', '', row); mk('set-fill', '', track); mk('set-thumb', '', track);
    const val = mk('set-val', '', row);
    sliders[key] = () => { track.style.setProperty('--f', ((ui[key] - lo) / (hi - lo)).toFixed(3)); val.textContent = fmt ? fmt(ui[key]) : Math.round(ui[key] * 100) + '%'; };
    let pid = null;
    const at = e => { const r = track.getBoundingClientRect(), f = Math.min(1, Math.max(0, (e.clientX - r.left) / (r.width || 1))); setUi(key, Math.round((lo + f * (hi - lo)) / step) * step); };
    on(track, 'pointerdown', e => { e.preventDefault(); pid = e.pointerId; try { track.setPointerCapture(pid); } catch (_) { /* synthetic */ } at(e); });
    on(track, 'pointermove', e => { if (e.pointerId === pid) at(e); });
    const up = e => { if (e.pointerId === pid) pid = null; };
    on(track, 'pointerup', up); on(track, 'pointercancel', up);
    return row;
  };
  const sliders = {};
  // STEERING: mode (◄ ► | wheel | phone tilt) + wheel sensitivity (WHEEL) / tilt range (TILT)
  subHead(pSteer, 'steering', 'wheel');
  const steerSeg = mk('p-row seg', '', pSteer);
  const segBtns = {
    buttons: pBtn(steerSeg, 'seg-b', 'arrow buttons', () => setSteerMode('buttons'), 'btns'),
    wheel: pBtn(steerSeg, 'seg-b', 'steering wheel', () => setSteerMode('wheel'), 'wheel'),
    tilt: pBtn(steerSeg, 'seg-b', 'tilt the phone', () => setSteerMode('tilt'), 'tilt'),
  };
  const sensRow = slider(pSteer, 'wheelSens', 'wheel sensitivity', 0.05, null, 'wheel');     // WHEEL mode only
  const tiltRow = slider(pSteer, 'tiltMax', 'tilt range', 1, v => `±${v}°`, 'tilt');           // TILT mode only
  // RESIZE HUD: on-screen control sizes
  subHead(pSize, 'resize controls', 'resize');
  slider(pSize, 'wheel', 'wheel size', 0.05, null, 'wheel'); slider(pSize, 'gas', 'gas pedal size', 0.05, null, 'pedal'); slider(pSize, 'brake', 'brake pedal size', 0.05, null, 'brakePedal');
  pSize.querySelectorAll('.set-row')[1].classList.add('gas'); pSize.querySelectorAll('.set-row')[2].classList.add('brake');
  pBtn(mk('p-row', '', pSize), 'defaults', 'reset sizes', () => { for (const k of ['wheel', 'gas', 'brake']) ui[k] = UI_DEFAULTS[k]; applyUi(true); }, 'reset');
  // SOUND: mute + master / engine / world / interface volumes + phone vibration (off · light · strong; hidden without vibrate())
  subHead(pSound, 'sound', 'speaker');
  const sTop = mk('p-row seg snd', '', pSound);
  els.mute = pBtn(sTop, 'seg-b mute-b', 'mute (M)', () => setOption('mute', !state.mute), 'mute');
  slider(pSound, 'volMaster', 'master volume', 0.05, null, 'speaker');
  slider(pSound, 'volEngine', 'engine volume', 0.05, null, 'engine');
  slider(pSound, 'volWorld', 'world sounds: terrain, water, impacts', 0.05, null, 'world');
  slider(pSound, 'volUi', 'interface sounds: shifter, buttons, jingles', 0.05, null, 'bell');
  const hapRow = mk('p-row seg hap', '', pSound);
  const hapIco = mk('set-lbl', '', hapRow); hapIco.innerHTML = svg('vibe'); hapIco.title = 'vibration';
  const hapBtns = {
    off: pBtn(hapRow, 'seg-b', 'vibration off', () => setUi('haptics', 'off'), 'vibeOff'),
    light: pBtn(hapRow, 'seg-b', 'vibration light (crisp)', () => { setUi('haptics', 'light'); buzz('commit'); }, 'vibe1'),
    strong: pBtn(hapRow, 'seg-b', 'vibration strong', () => { setUi('haptics', 'strong'); buzz('commit'); }, 'vibe'),
  };
  if (!hapticsSupported()) hapRow.classList.add('gone');
  for (const o of OPTIONS) setOption(o.id, false);
  // left-cluster box for the canvas debug overlay (renderer dodges it); re-measured on resize / size change
  let dodge = null, spotBox = null;
  // the bottom-centre tray stays screen-centred when it fits between the two thumb clusters, else slides / shrinks into the gap
  const measure = () => {
    const l = zones.bl.getBoundingClientRect(), r = zones.br.getBoundingClientRect(), row = thumbRow.getBoundingClientRect();
    dodge = l.width ? { top: l.top, right: l.right } : null;
    // SPOTTER box (CSS px): left side, under MENU / RESET and above the steering + BRAKE cluster, gauge width, 0.46 aspect (trail only)
    const gr = g.getBoundingClientRect(), tl = zones.tl.getBoundingClientRect(), top = tl.bottom + 8, room = (l.top || innerHeight) - 8 - top;
    const sw = gr.width, shh = Math.min(sw * 0.52, room);              // 8.5 m long x 4.4 m (follows the truck's z)
    spotBox = gr.width && shh >= 50 ? { x: tl.left, y: top, w: sw, h: shh } : null;
    const a2 = Math.max(row.right, l.right) + 8, b2 = r.left - 8, tw = zones.bc.offsetWidth;
    if (!tw || !r.width || b2 <= a2) return;
    const k = Math.min(1, (b2 - a2) / tw), half = tw * k / 2, cx = Math.min(Math.max(innerWidth / 2, a2 + half), b2 - half);
    zones.bc.style.left = `${cx.toFixed(1)}px`; zones.bc.style.transform = `translateX(-50%) scale(${k.toFixed(3)})`;
  };
  const dots = (el, n) => { el.textContent = ''; for (let i = 0; i < 3; i++) mk('pd' + (i < n ? ' on' : ''), '', el, 'i'); };
  const applyUi = save => {
    root.style.setProperty('--wheel-s', ui.wheel); root.style.setProperty('--gas-s', ui.gas); root.style.setProperty('--brake-s', ui.brake);
    root.classList.toggle('steer-btns', ui.steer === 'buttons'); root.classList.toggle('steer-tilt', ui.steer === 'tilt');
    wheelEl.setAttribute('aria-label', ui.steer === 'tilt' ? 'tilt steering indicator · centre cap = horn (H)' : 'steering wheel · centre cap = horn (H)');
    wheelEl.title = 'horn (H)';
    for (const k in segBtns) segBtns[k].classList.toggle('on', ui.steer === k);
    sensRow.classList.toggle('gone', ui.steer !== 'wheel'); tiltRow.classList.toggle('gone', ui.steer !== 'tilt');
    for (const k in sliders) sliders[k]();
    if (!GFX_MODES.includes(ui.gfx)) ui.gfx = 'auto';
    steerT.badge.innerHTML = svg(ui.steer === 'buttons' ? 'btns' : ui.steer === 'tilt' ? 'tilt' : 'wheel');
    sizeT.el.classList.toggle('mod', !['wheel', 'gas', 'brake'].every(k => ui[k] === UI_DEFAULTS[k]));
    if (ui.gfx === 'auto') gfxT.badge.innerHTML = svg('auto'); else dots(gfxT.badge, GFX_MODES.indexOf(ui.gfx));
    gfxT.el.title = 'graphics: ' + ui.gfx;
    if (!SPOT_UI.includes(ui.spot)) ui.spot = 'auto';
    setIcon(spotT.el, ui.spot === 'off' ? 'eyeOff' : 'eye'); spotT.badge.innerHTML = ui.spot === 'auto' ? svg('auto') : '';
    spotT.el.classList.toggle('on', ui.spot !== 'off');
    if (!HAPTIC_LEVELS.includes(ui.haptics)) ui.haptics = 'light';
    haptics.level = ui.haptics;
    for (const k in hapBtns) hapBtns[k].classList.toggle('on', ui.haptics === k);
    if (ui.gfx !== lastGfx) { lastGfx = ui.gfx; try { handlers.onGfx?.(ui.gfx); } catch (_) { /* glue optional */ } }
    try { handlers.onVolume?.({ master: ui.volMaster, engine: ui.volEngine, world: ui.volWorld, ui: ui.volUi }); } catch (_) { /* audio optional */ }
    if (save) { try { handlers.onUi?.({ ...ui }); } catch (_) { /* storage optional */ } }
    requestAnimationFrame(measure);
  };
  let lastGfx = null;
  const setUi = (k, v) => { if (ui[k] === v) return; ui[k] = v; applyUi(true); };
  applyUi(false);
  on(window, 'resize', () => requestAnimationFrame(measure));
  function setPause(p) {
    p = !!p;
    if (p === paused) return;
    if (p) releaseAll();
    paused = p;
    setView('main');
    const idle = !!handlers.active && !handlers.active();             // opened from the start screen: settings only (no run to resume)
    setIcon(resumeBtn, idle ? 'close' : 'play'); resumeBtn.setAttribute('aria-label', idle ? 'close' : 'resume'); levelsBtn.classList.toggle('gone', idle);
    pauseEl.classList.toggle('show', p); root.classList.toggle('paused', p); menuBtn.classList.toggle('on', p);
    try { handlers.onPause?.(p); } catch (_) { /* glue optional */ }
  }
  resetControls();

  // gauges
  const g = mk('gauges', '', zones.tr);
  const gear = mk('gear', 'L', g), speed = mk('speed', '0', g);
  // bar fills: write only on a visible change (~0.4 % steps), so a steady gauge never dirties the glass HUD
  const scaleX = (el, v) => { const q = Math.round(Math.max(0, Math.min(1, v)) * 250); if (el._q !== q) { el._q = q; el.style.transform = `scaleX(${q / 250})`; } };
  const rpmBar = mk('bar-fill', '', mk('bar rpm', '', g));
  // dash warning lamps (beside the rpm bar): all lit while cranking (bulb check), battery + oil while stalled, oil red when the
  // engine boils (limp), check-engine amber with heavy body damage / misfires. Classes change only on a state change.
  const lampBox = mk('lamps', '', g), lamp = {};
  for (const [k, ic, lbl] of [['bat', 'battery', 'charge'], ['oil', 'oil', 'oil pressure / temperature'], ['eng', 'engine', 'check engine']]) {
    lamp[k] = mk('lamp ' + k, '', lampBox); lamp[k].innerHTML = svg(ic); lamp[k].title = lbl;
  }
  let lampKey = '';
  // engine-catch SWEEP (like a real dash self-test): rpm / speed digits / bars run up to full and back once (~0.9 s)
  let sweepT0 = -1;
  const SWEEP_MS = 900, SWEEP_KMH = 140;
  const winchRow = mk('winch-row', '', g), winchTxt = mk('winch-txt', '', winchRow);   // hook icon (rope snapped: red rope icon) + load bar
  winchTxt.innerHTML = svg('hook');
  const winchTrack = mk('bar tension', '', winchRow), winchBar = mk('bar-fill', '', winchTrack);
  const winchStall = mk('bar-mark', '', winchTrack);              // tick at the drum's stall pull (fraction of break load)
  // rig glyph (top-down, front → right; L row on top): wheel dots spin (amber) / lock (white) / air (dim), red ring = shock damage;
  // axle bars glow green → amber → red with shaft stress, dashed red when snapped. Warnings live on the drawing (no text line):
  // engine block at the nose glows orange -> red pulse when hot (limp), gearbox box mid-body glows with transmission heat (bogging
  // = wheels pulse amber, box warms with transTemp; boiled = red pulse) or amber when the transfer case is hot (2WD only)
  const rig = mk('rig', '', g), rigW = [], rigA = [];
  mk('rig-body', '', rig);
  const rigEng = mk('rig-eng', '', rig), rigBox = mk('rig-box', '', rig);
  for (let a = 0; a < 2; a++) rigA.push(mk('rig-axle ' + (a ? 'r' : 'f'), '', rig));
  for (const pos of ['fl', 'fr', 'rl', 'rr']) rigW.push(mk('rig-w ' + pos, '', rig));
  let lastDmg = -1, hitT = null;
  // vertical side meters: icon over a bar that fills from the bottom (no number: the bar is the reading). FUEL left (amber pulse
  // < 20 %, red pulse < 10 %), DMG right = overallDamage() (green -> amber -> red, pulses > 80 %)
  const meter = (cls, label, icon) => {
    const row = mk('meter ' + cls, '', g);
    row.title = label; row.setAttribute('aria-label', label);
    row.insertAdjacentHTML('beforeend', svg(icon, 'm-ico'));
    return { row, fill: mk('bar-fill', '', mk('m-bar', '', row)) };
  };
  const scaleY = (el, v) => { const q = Math.round(Math.max(0, Math.min(1, v)) * 250); if (el._q !== q) { el._q = q; el.style.transform = `scaleY(${q / 250})`; } };
  // bar colour runs green -> yellow -> red with the reading (hue 130 -> 0; good = full fuel / no damage), written per 4° step only
  const hueBar = (el, good) => { const h = Math.round(Math.max(0, Math.min(1, good)) * 130 / 4) * 4; if (el._h !== h) { el._h = h; el.style.setProperty('--h', h); } };
  const fuelM = meter('fuel', 'fuel', 'pump'), dmgM = meter('dmg', 'damage', 'wrench');
  const hintEl = mk('hint', '', root);          // tutorial / stage banner (hint(text) / hint(''))
  const toastEl = mk('toast', '', root);
  // run info: tiny translucent "Level · Stage n · Truck" line top-centre (screenshots say where they were taken)
  const runInfo = mk('runinfo', '', root);
  let toastTimer = null;
  /** toast(text, ms) or toast({ icon, badge?, val?, tone? 'bad' | 'warn' | 'good' | 'cool', label? (aria / title) }, ms): icon toasts carry
   *  no words: a pictogram (+ a small corner badge pictogram, + an optional number such as a countdown). */
  function toast(t, ms = 2200) {
    if (t && typeof t === 'object') {
      toastEl.className = 'toast ico ' + (t.tone || '');
      toastEl.innerHTML = `<span class="t-main">${svg(t.icon)}${t.badge ? `<span class="t-badge">${svg(t.badge)}</span>` : ''}</span>${t.val != null ? `<span class="t-val">${t.val}</span>` : ''}`;
      toastEl.setAttribute('aria-label', t.label || t.icon); toastEl.title = t.label || '';
    } else { toastEl.className = 'toast'; toastEl.textContent = t; toastEl.removeAttribute('aria-label'); toastEl.title = ''; }
    void toastEl.offsetWidth; toastEl.classList.add('show');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => toastEl.classList.remove('show'), ms);
  }

  // finish screen (showroom look): smoked-glass panel, painted gold chequered-flag emblem (LEVEL FAILED: a face hiding in its
  // hands), level · truck, icon tiles TIME / FUEL / DAMAGE / CANS (fail: PROGRESS), score + stars popping in one by one (a
  // haptic tick each), gold ↻ (play again / retry; tap or Enter), dark pills ▶▶ next · recover (fail) · ☰ menu. No words.
  const finishEl = mk('finish', '', root);
  const fp = mk('fin-panel', '', finishEl);
  const fHead = mk('fin-head', '', fp);
  const fTitle = mk('fin-title t-acc', '  ', fHead, 'span'); fTitle.dataset.icon = 'chequer'; fTitle.setAttribute('aria-label', 'finish');
  const newBest = mk('fin-new', '', fHead, 'span'); newBest.innerHTML = svg('trophy'); newBest.setAttribute('aria-label', 'new best');
  const fSub = mk('fin-sub', '', fp);
  const fStats = mk('fin-stats', '', fp);
  const ftile = (key, icon, label) => { const c = mk('fin-stat ' + key, '', fStats); const l = mk('fs-l', '', c); l.innerHTML = svg(icon); c.setAttribute('aria-label', label);
    return { c, l, v: mk('fs-v', '', c), n: mk('fs-n', '', c) }; };
  const tTime = ftile('time', 'clock', 'time'), tFuel = ftile('fuel', 'pump', 'fuel left'), tDmg = ftile('dmg', 'wrench', 'damage'), tCans = ftile('cans', 'can', 'jerry cans');
  const fScore = mk('fin-score', '', fp);
  mk('sc-l', '', fScore);
  const scV = mk('sc-v', '', fScore), scStars = mk('sc-stars', '', fScore), scB = mk('sc-b', '', fScore);
  const finishBtns = mk('fin-btns', '', fp);
  const again = mk('fin-go', '   ', finishBtns);
  again.dataset.glyph = 'reset'; again.setAttribute('role', 'button'); again.setAttribute('aria-label', 'play again (Enter)');
  const pill = (cls, icon, label) => { const el = mk('fin-menu ' + cls, '', finishBtns); el.innerHTML = svg(icon); el.setAttribute('role', 'button'); el.setAttribute('aria-label', label); el.title = label; return el; };
  // LEVEL FAILED (fell off the edge) reuses this panel: facepalm emblem, cause pictogram, TIME / FUEL / DAMAGE / PROGRESS, ↻ RETRY + recover
  const fReset = pill('fin-reset', 'recover', 'recover on the trail');
  on(fReset, 'pointerup', e => { e.preventDefault(); hideFinish(); handlers.onFailReset?.(); });
  // NEXT (finish only): same truck, next level in the manifest; greyed out on the last level (run.hasNext false)
  const fNext = pill('fin-next', 'next', 'next level (N)');
  on(fNext, 'pointerup', e => { e.preventDefault(); if (fNext.classList.contains('dis') || !finishShown()) return; hideFinish(); handlers.onNext?.(); });
  if (handlers.onMenu) {
    const fm = pill('', 'menu', 'levels');
    on(fm, 'pointerup', e => { e.preventDefault(); hideFinish(); handlers.onMenu(); });
  }
  const starT = [];
  const finPaint = [];
  if (typeof HTMLCanvasElement === 'function' && document.createElement('canvas').getContext) {   // forced-dark-proof, like the start screen
    finPaint.push(canvasPaint(fTitle, 'emblem', 0.45, offs), canvasPaint(again, 'start', 0, offs));
  }
  let againPid = null, setT = null;
  const finishShown = () => finishEl.classList.contains('show');
  const hideFinish = () => { againPid = null; again.classList.remove('on'); finishEl.classList.remove('show', 'failed'); root.classList.remove('finished'); starT.forEach(clearTimeout); starT.length = 0; };
  const finMode = fail => {                                             // FINISH <-> LEVEL FAILED emblem (canvas paints read data-icon)
    finishEl.classList.toggle('failed', fail);
    fTitle.dataset.icon = fail ? 'facepalm' : 'chequer'; fTitle.classList.toggle('t-fail', fail); fTitle.setAttribute('aria-label', fail ? 'level failed' : 'finish');
    again.setAttribute('aria-label', fail ? 'retry (Enter)' : 'play again (Enter)');
  };
  const playAgain = () => {
    if (!finishShown()) return;
    hideFinish();
    (handlers.onPlayAgain || handlers.onRestart)?.();
  };
  on(again, 'pointerdown', e => { e.preventDefault(); againPid = e.pointerId; again.classList.add('on'); });
  on(again, 'pointerup', e => { if (e.pointerId !== againPid) return; playAgain(); });
  on(again, 'pointercancel', () => { againPid = null; again.classList.remove('on'); });

  // KEYBOARD. Driving: W/↑ gas, S/↓/Space brake, A/← D/→ steer (adds to wheel / tilt in every steering mode), E winch, V rev,
  // 1 2 3 4WD / lockers, Q Z 4 L range, K I key, H horn, R recover (Shift+R restart), Enter play again.
  // Anytime: Esc MENU, G debug, M mute, C cam. MENU open: driving keys dead, ←/→ (A/D) = steering mode, Enter resumes.
  // Holds re-assert on auto-repeat (a key still down after RESUME / refocus drives again); one-shot keys ignore repeats.
  // Ctrl / Alt / Meta chords belong to the browser (and macOS drops keyups while Meta is down: they would stick), Meta keyup /
  // blur / hidden tab / pause release every key. handlers.active?() false (start screen): keydowns ignored, keyups still release.
  const momOf = code => MOMENTARY.find(m => m.keys.includes(code));
  on(window, 'keydown', e => {
    if (e.isComposing || (handlers.active && !handlers.active())) return;
    if (e.ctrlKey || e.metaKey || e.altKey) return;
    const mom = momOf(e.code);
    if (mom) { e.preventDefault(); if (!paused) keyHold(mom.id, e.code, true); return; }   // arrows / Space never scroll
    if (e.repeat) return;
    for (const o of OPTIONS) if (o.key === e.code) setOption(o.id, !state[o.id]);
    if (e.code === 'KeyC') setCam(state.cam === '2D' ? '2.5D' : '2D');
    if (e.code === 'Escape') { setPause(!paused); return; }
    if (e.code === 'KeyH' && !paused) setHorn('key', true);
    if (e.code === 'KeyO') nextSpot();
    if (paused) {
      if (e.code === 'Enter' || e.code === 'NumpadEnter') setPause(false);
      return;
    }
    for (const t of TOGGLES) if (t.key === e.code) {
      state[t.id] = !state[t.id]; els[t.id].classList.toggle('on', state[t.id]); tutUsed(t.id);
      if (DRIVE.includes(t.id)) shiftHook('drive', { id: t.id, on: state[t.id] });
      if (t.id === 'fourWD' && !state.fourWD && state.lockF) { state.lockF = false; syncLocks(); }
    }
    if (e.code === 'Digit2' || e.code === 'Digit3') lockStep();
    if (e.code === 'KeyV') setGate(state.gate === 'R' ? lastFwd : 'R');                        // R toggle
    if (e.code === 'KeyN' && !finishShown()) setGate(state.gate === 'N' ? lastDrive : 'N');      // neutral toggle
    if (e.code === 'Digit4' || e.code === 'KeyL') cycleRange(e.shiftKey ? -1 : 1);
    if (e.code === 'KeyQ') stepRange(-1);
    if (e.code === 'KeyZ') stepRange(1);
    if (e.code === 'KeyK' || e.code === 'KeyI') turnKey();
    if (e.code === 'KeyR') (e.shiftKey ? handlers.onRestart : handlers.onRecover)?.();
    if (e.code === 'Enter' || e.code === 'NumpadEnter') playAgain();
    if (e.code === 'KeyN' && finishShown() && !finishEl.classList.contains('failed') && !fNext.classList.contains('dis')) { hideFinish(); handlers.onNext?.(); }
  });
  // MENU open: ←/→ (A/D) step the STEERING mode (held keys were released by the pause; momentary keys are dead there)
  on(window, 'keydown', e => {
    if (!paused || e.repeat || e.isComposing || e.ctrlKey || e.metaKey || e.altKey || (handlers.active && !handlers.active())) return;
    const d = e.code === 'ArrowLeft' || e.code === 'KeyA' ? -1 : e.code === 'ArrowRight' || e.code === 'KeyD' ? 1 : 0;
    if (d) setSteerMode(STEER_MODES[(STEER_MODES.indexOf(ui.steer) + d + STEER_MODES.length) % STEER_MODES.length]);
  });
  on(window, 'keyup', e => {
    if (e.code === 'KeyH') setHorn('key', false);
    if (e.key === 'Meta' || e.key === 'OS') { releaseKeys(); return; }
    const mom = momOf(e.code);
    if (mom) keyHold(mom.id, e.code, false);
  });
  on(root, 'contextmenu', e => e.preventDefault());
  on(window, 'blur', () => releaseAll());
  if (ui.steer === 'tilt') startTilt();             // restored TILT: listens now (iOS waits for the next tap)

  return {
    state,
    resetControls,
    /** Open / close the pause panel (no arg = toggle). */
    pause(open = !paused) { setPause(open); },
    get paused() { return paused; },
    get ui() { return { ...ui }; },
    /** Switch steering mode ('buttons' | 'wheel' | 'tilt'; call from a user gesture for iOS tilt permission). */
    setSteerMode,
    get dodge() { return dodge; },
    get spot() { return spotBox ? { box: spotBox, mode: ui.spot } : null; },   // renderer frame.spot (null = no room)
    /** Refresh gauges from Vehicle.getState(). */
    update(v, redline = 5000) {
      // steering load 0..1 for the haptic feel: stopped / dry-steer (|v| < 0.3 m/s) = 1 → rolling ≥ 2 m/s = 0; F-LOCK ≥ 0.7
      steerLoad = Math.max(state.lockF ? 0.7 : 0, Math.max(0, Math.min(1, 1 - (Math.abs(v.speed || 0) - 0.3) / 1.7)));
      if (wheelBack && ui.steer !== 'tilt') {             // released virtual wheel: drawn at the front tyres' real angle (they ease back
        const tk = Math.max(-1, Math.min(1, (v.steer || 0) / (v.steerMax || 0.56)));   // while rolling, stay put when parked)
        const nw = tk - state.steer;
        if (Math.abs(nw - wheelVis) > 0.003) { wheelVis = nw; follow = true; syncSteer(); follow = false; }
      }
      // pedal fill = level actually applied (index.html ramps throttle), falls back to the HUD target
      drawPedal('gas', v.throttle ?? state.gas); drawPedal('brake', v.brake ?? state.brake);
      const gt = v.gear + (v.fourWD ? '4' : '2'); if (gear.textContent !== gt) gear.textContent = gt;   // unchanged text: no per-frame layout
      // a slot with another range / direction waits for a stop (v.pendingRange): the gate pulses slightly red until it commits;
      // the engaged slot's letter turns green, + a firm haptic when it engages (the clunk itself = audio state edges)
      const pend = !!v.pendingRange; if (shifter._p !== pend) { shifter._p = pend; shifter.classList.toggle('sync', pend); }
      const cg = v.gate || (v.reverse ? 'R' : v.range);
      if (cg && cg !== committed) {
        if (committed !== null) buzz('commit');
        committed = cg;
        for (const k of SLOTS) gateLbl[k].classList.toggle('cur', k === cg);
      }
      let sw = 0;                                          // sweep 0 -> 1 -> 0
      if (sweepT0 >= 0) { const k = (performance.now() - sweepT0) / SWEEP_MS; if (k >= 1) sweepT0 = -1; else sw = Math.sin(Math.PI * Math.min(1, k * 1.15)) ** 0.8; }
      const sp = Math.max(Math.abs(v.speed * 3.6), sw * SWEEP_KMH).toFixed(0); if (speed.textContent !== sp) { speed.textContent = sp; const w3 = sp.length > 2; if (speed._w3 !== w3) { speed._w3 = w3; speed.classList.toggle('w3', w3); } }   // 3 digits: tighter, never wraps (sweep to 140 grew the gauge)
      const r = Math.max(sw, Math.min(1, v.rpm / redline));
      scaleX(rpmBar, r);
      rpmBar.classList.toggle('hot', r > 0.9);
      const w = v.winch;
      winchRow.classList.toggle('show', w.attached || w.broken);
      winchRow.classList.toggle('snapped', !w.attached && !!w.broken);
      if (w.attached) {
        const t = Math.min(1, w.tension / w.breakTension);
        if (winchRow._ic !== 'hook') { winchRow._ic = 'hook'; winchTxt.innerHTML = svg('hook'); }
        scaleX(winchBar, t);
        winchBar.classList.toggle('hot', w.tension > w.maxPull * 0.9);
        winchStall.style.left = `${Math.min(100, (w.maxPull / w.breakTension) * 100).toFixed(1)}%`;
        winchRow.classList.toggle('crit', t > 0.85);
      } else if (w.broken) {
        if (winchRow._ic !== 'rope') { winchRow._ic = 'rope'; winchTxt.innerHTML = svg('rope'); }
        scaleX(winchBar, 0);
        winchRow.classList.remove('crit');
      }
      // rig glyph: per-wheel slip state + shock damage, per-axle shaft stress
      (v.wheels || []).forEach((o, i) => {
        const el = rigW[o.axle * 2 + (o.side === 'R' ? 1 : 0)] || rigW[i]; if (!el) return;
        const sl = o.slip || 0, st = !o.grounded ? 'air' : sl > 1.1 ? 'spin' : sl < -1.1 ? 'lock' : '';
        if (el._st !== st) { el._st = st; el.className = el.className.replace(/ (air|spin|lock)\b/g, '') + (st ? ' ' + st : ''); }
        const d = Math.round(Math.min(1, o.damage || 0) * 20) / 20;
        if (el._d !== d) { el._d = d; el.style.setProperty('--d', d); }
      });
      els.winch.classList.toggle('armed', w.attached);
      const h = v.hardcore;
      if (h) {
        const f = h.dry ? 0 : Math.max(0, Math.min(1, h.fuel ?? 1));
        scaleY(fuelM.fill, Math.max(f, sw)); hueBar(fuelM.row, Math.min(1, Math.max(f, sw) / 0.75));
        fuelM.row.classList.toggle('low', f < 0.2 && f >= 0.1 && !h.dry);   // amber pulse: ease off the throttle
        fuelM.row.classList.toggle('crit', f < 0.1 || !!h.dry);              // red pulse: about to stall dry
        const od = overallDamage(h);
        scaleY(dmgM.fill, Math.max(od, sw)); hueBar(dmgM.row, 1 - Math.min(1, Math.max(od, sw) / 0.85));
        dmgM.row.classList.toggle('warn', od > 0.4 && od <= 0.7);
        dmgM.row.classList.toggle('fail', od > 0.7);
        dmgM.row.classList.toggle('crit', od > 0.8);
        for (let a = 0; a < 2; a++) {
          const x = Math.min(1, h.axleStress[a] || 0), b = !!h.axleBroken[a], el = rigA[a];
          el.classList.toggle('snap', b); el.classList.toggle('crit', !b && x > 0.8);
          const sv = b ? 1 : Math.round(x * 20) / 20; if (el._s !== sv) { el._s = sv; el.style.setProperty('--s', sv); }
        }
        // fresh body / shock damage: DMG meter + rig glyph flash red (restartable CSS animation)
        const dmg = (h.body || 0) + (h.shock || []).reduce((s2, x) => s2 + x, 0);
        if (lastDmg >= 0 && dmg > lastDmg + 0.004) {
          for (const el of [dmgM.row, rig]) { el.classList.remove('hit'); void el.offsetWidth; el.classList.add('hit'); }
          clearTimeout(hitT); hitT = setTimeout(() => { dmgM.row.classList.remove('hit'); rig.classList.remove('hit'); }, 600);
        }
        lastDmg = dmg;
        // warnings on the rig drawing (no text): engine heat at the nose, gearbox heat mid-body, bogging wheels; both axles
        // snapped = the truck can't drive: RESET pulses (hold = restart, repaired). Engine state = the KEY's colour.
        const rs = (h.engineHot ? 'e-hot ' : (h.engineHeat || 0) > 0.6 ? 'e-warm ' : '') + (h.transHot ? 't-hot ' : h.xferHot ? 'x-hot ' : '')
          + (h.bog > 0.5 ? 'bog ' : '');
        if (rig._rs !== rs) { rig._rs = rs; rig.className = 'rig ' + rs + (rig.classList.contains('hit') ? 'hit' : ''); }
        const tt = Math.round(Math.min(1, h.transTemp || 0) * 20) / 20; if (rigBox._t !== tt) { rigBox._t = tt; rigBox.style.setProperty('--t', tt); }
        const both = !!(h.axleBroken[0] && h.axleBroken[1]); if (reset._both !== both) { reset._both = both; reset.classList.toggle('urge', both); }
        const mode = h.cranking ? 'crank' : v.engineOn === false ? (h.noStart ? 'fail' : 'off') : 'run';
        const lk = mode === 'crank' ? 'bat oil eng ' : (mode === 'fail' || (v.engineOn === false && h.stalled) ? 'bat oil ' : '') + (h.engineHot ? 'oilhot ' : '')
          + ((h.body || 0) > 0.5 || h.misfire ? 'eng ' : '');
        if (lk !== lampKey) { lampKey = lk; for (const k in lamp) lamp[k].classList.toggle('on', lk.includes(k + ' ') || (k === 'oil' && lk.includes('oilhot'))); lamp.oil.classList.toggle('hot', lk.includes('oilhot')); }
        if (mode !== keyMode) {
          if (keyMode === 'crank' && mode === 'run') sweepT0 = performance.now();   // engine caught: dash self-test sweep
          keyMode = mode;
          keyBtn.classList.toggle('off', mode === 'off'); keyBtn.classList.toggle('crank', mode === 'crank'); keyBtn.classList.toggle('run', mode === 'run');
          keyBtn.classList.toggle('fail', mode === 'fail');
          keyBtn.classList.toggle('stalled', !!h.stalled);
          keyBtn.title = mode === 'crank' ? 'cranking' : mode === 'fail' ? 'did not start: turn the key again (K)' : mode === 'off' ? 'start engine (K)' : 'stop engine (K)';
        }
      }
    },
    /** Tutorial / stage banner: hint(text) shows it until hint('') (or for ms if given). */
    hint(text, ms = 0) {
      hintEl.textContent = text || '';
      hintEl.classList.toggle('show', !!text);
      clearTimeout(hintEl._t);
      if (text && ms) hintEl._t = setTimeout(() => hintEl.classList.remove('show'), ms);
    },
    toast,
    /** Top-centre run label (plain text; '' hides). */
    /** RESET button field-repair charge 0..1 (1 = kit ready: green; recharging: a fill bar under the label). */
    /** Per-rig steering wheel art + badge tint (vehicle look.style, paint.body). */
    setRig(cfg = {}) { setRig(cfg.look?.style, cfg.paint?.body); },
    setRepair(f = 1) { const q = Math.round(Math.max(0, Math.min(1, f)) * 50) / 50; if (q !== reset._rep) { reset._rep = q; reset.style.setProperty('--rep', q); reset.classList.toggle('charging', q < 1); } },
    setRunInfo(t = '') { if (runInfo.textContent !== t) runInfo.textContent = t; },
    /**
     * Finish screen. run = { time (s), par? (s), level?, vehicle?, best? { isNew, prev (s) | null }, fuel, tank?, cans? [got, total],
     * hardcore? (getState().hardcore) } -> tiles + score (scoreRun). Returns the score object.
     */
    showFinish(run = {}, fail = false) {
      finMode(fail);
      const h = run.hardcore || {}, dmg = run.hardcore ? overallDamage(h) : 0, b = run.best;
      const clock = t => { const m = Math.floor(t / 60), sec = t - 60 * m; return m ? `${m}:${sec.toFixed(1).padStart(4, '0')}` : `${sec.toFixed(1)}s`; };
      const sc = scoreRun({ time: run.time, par: run.par, fuel: h.dry ? 0 : run.fuel ?? 1, tank: run.tank ?? 1, damage: dmg });
      newBest.classList.toggle('show', !!(b && b.isNew));
      fSub.textContent = [run.level, run.vehicle].filter(Boolean).join('  ·  ');
      // tile: value + optional note = [icon, text] (numbers only; icons say what they are)
      const set = (T, icon, v, note, lvl = 0) => {
        if (icon && T.l._ic !== icon) { T.l._ic = icon; T.l.innerHTML = svg(icon); }
        T.v.textContent = v; T.n.innerHTML = note ? (note[0] ? svg(note[0]) : '') + `<span>${note[1] || ''}</span>` : '';
        T.c.classList.toggle('warn', lvl === 1); T.c.classList.toggle('bad', lvl === 2); T.c.classList.toggle('good', lvl === 3);
      };
      set(tTime, 'clock', clock(run.time || 0), !b ? (run.par ? ['flag', clock(run.par)] : null)
        : b.isNew ? (b.prev != null ? ['trophy', `−${(b.prev - run.time).toFixed(1)}s`] : ['trophy', '']) : ['trophy', clock(b.prev)], b && b.isNew ? 3 : 0);
      const fl = h.dry ? 0 : Math.max(0, Math.min(1, run.fuel ?? 1));
      set(tFuel, 'pump', `${Math.round(fl * 100)}%`, null, h.dry || fl < 0.1 ? 2 : fl < 0.2 ? 1 : 0);
      set(tDmg, 'wrench', `${Math.round(dmg * 100)}%`, dmg < 0.05 ? ['check', ''] : null, dmg > 0.6 ? 2 : dmg > 0.25 ? 1 : dmg < 0.05 ? 3 : 0);
      const cn = run.cans || [0, 0];
      tCans.c.style.display = cn[1] ? '' : 'none';
      fStats.style.setProperty('--n', cn[1] ? 4 : 3);
      set(tCans, 'can', `${cn[0]}/${cn[1]}`, cn[0] ? ['plus', `${Math.round((run.canFuel || 0) * 100)}%`] : null, cn[1] && cn[0] === cn[1] ? 3 : 0);
      setT = set;
      scV.textContent = String(sc.total);
      // stars pop in one at a time (CSS .pop, staggered); each earned star = one crisp haptic tick
      scStars.textContent = ''; scStars.dataset.n = sc.stars;
      starT.forEach(clearTimeout); starT.length = 0;
      for (let i = 0; i < 3; i++) {
        const st = mk('st' + (i < sc.stars ? ' on' : ''), i < sc.stars ? '★' : '☆', scStars, 'span');
        st.style.setProperty('--i', i);
        if (!fail && i < sc.stars) starT.push(setTimeout(() => buzz('star'), 380 + 260 * i));
      }
      scB.innerHTML = [['clock', sc.time], ['pump', sc.fuel], ['wrench', sc.condition]].map(([ic, n]) => `<span>${svg(ic)}${n}</span>`).join('');
      fNext.classList.toggle('dis', !run.hasNext);
      finishEl.classList.add('show');
      root.classList.add('finished');
      requestAnimationFrame(() => finPaint.forEach(f => f()));
      return sc;
    },
    /** LEVEL FAILED panel: run {cause, causeIcon ('cliff' | 'bridge'), time, level, vehicle, fuel, tank, hardcore, dist, total} (m along the trail). */
    showFailed(run = {}) {
      this.showFinish({ ...run, best: null, par: null, cans: [0, 0] }, true);
      newBest.classList.remove('show');
      fSub.innerHTML = '';
      fSub.insertAdjacentHTML('beforeend', svg(run.causeIcon || 'cliff', 'cause'));
      mk('', [run.level, run.vehicle].filter(Boolean).join('  ·  '), fSub, 'span');
      fSub.title = run.cause || '';
      const d = Math.max(0, run.dist || 0), tot = Math.max(1, run.total || 1);
      tCans.c.style.display = ''; fStats.style.setProperty('--n', 4);
      setT(tCans, 'flag', `${Math.round(Math.min(1, d / tot) * 100)}%`, [null, `${Math.round(d)}/${Math.round(tot)} m`], 2);
      tTime.n.textContent = '';
      requestAnimationFrame(() => finPaint.forEach(f => f()));
    },
    /** Tutorial glow on controls (ids: key fourWD lockF lockR winch shifter wheel gas brake); [] clears. */
    glow,
    /** Phone haptic cue (engine/haptics.js kinds; k 0..1 for thump / crash). */
    haptic: buzz,
    hideFinish,
    destroy() { clearTimeout(hitT); stopTilt(); offs.forEach(f => f()); root.textContent = ''; },
  };
}

/**
 * Start-screen content selector (DOM only).
 * @param root      container (e.g. the #start overlay); its children are replaced
 * @param manifest  content.json: { levels: [{id, file, name, difficulty?, blurb?}], vehicles: [{id, file, name, blurb?}], vehicleSlots? }
 * @param initial   { level, vehicle } ids to preselect
 * @param onStart   fn({ level, vehicle }) — called from the START tap (a user gesture: unlock audio / fullscreen there)
 * @returns { selection, show(), hide(), setStatus(text), setBests({levelId: {time, vehicle?}}), destroy() }
 */
// ---- forced-dark-proof paint (start screen) ----
// Samsung Internet's "dark sites" mode ignores color-scheme and recolours CSS colours / gradients (gradient text turns dark,
// the gold START turns maroon) but never canvas pixels. So the metallic title, headers, card names, tags and START are painted
// on small canvases (.cv-paint) over their own DOM text, which stays (transparent) for layout + accessibility.
const STEEL = [[0, '#ffffff'], [0.38, '#eef0f2'], [0.56, '#c3c8ce'], [0.74, '#dfe2e6'], [1, '#b4bac1']];
const CHROME = [[0, '#ffffff'], [0.3, '#e9ecef'], [0.48, '#8f969e'], [0.54, '#c9ced3'], [0.72, '#f4f6f8'], [1, '#9aa0a8']];
const RED_T = [[0, '#ffd2c2'], [0.32, '#ff7a5a'], [0.52, '#d8301c'], [0.66, '#ff5a3a'], [0.82, '#ff9a7a'], [1, '#8e1a0c']];   // LEVEL FAILED
const GOLD_T = [[0, '#fff2b8'], [0.32, '#ffc63a'], [0.52, '#f08a12'], [0.66, '#ffb02e'], [0.82, '#ffd76a'], [1, '#c2610a']];
const GOLD_B = [[0, '#fff1a8'], [0.3, '#ffd23e'], [0.58, '#f0a514'], [0.8, '#ffc23a'], [1, '#c88410']];
const vgrad = (ctx, y0, y1, stops) => { const g = ctx.createLinearGradient(0, y0, 0, y1); for (const [t, c] of stops) g.addColorStop(t, c); return g; };
const pill = (ctx, x, y, w, h, keep) => { const r = h / 2; if (!keep) ctx.beginPath(); ctx.moveTo(x + r, y); ctx.arcTo(x + w, y, x + w, y + h, r); ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r); ctx.arcTo(x, y, x + w, y, r); ctx.closePath(); };
/** Text line metrics matching CSS layout: font, letter-spacing, left padding x, baseline y (content area centred in the line box). */
function textAt(ctx, cs) {
  ctx.font = `${cs.fontStyle} ${cs.fontWeight} ${cs.fontSize} ${cs.fontFamily}`;
  if ('letterSpacing' in ctx) ctx.letterSpacing = cs.letterSpacing === 'normal' ? '0px' : cs.letterSpacing;
  const m = ctx.measureText('Hg'), a = m.fontBoundingBoxAscent, d = m.fontBoundingBoxDescent, lh = parseFloat(cs.lineHeight) || a + d;
  return { fs: parseFloat(cs.fontSize), x: parseFloat(cs.paddingLeft) || 0, y: (parseFloat(cs.paddingTop) || 0) + (lh - a - d) / 2 + a };
}
/** The host's text runs in order (own text nodes + child spans, canvases skipped): [[text, span class | '']]. */
const runsOf = el => [...el.childNodes].filter(n => n.nodeType === 3 || (n.nodeType === 1 && n.tagName !== 'CANVAS')).map(n => [n.textContent, n.nodeType === 1 ? n.className : '']);
const PAINT = {
  steel(ctx, w, h, el, cs) {                                          // headers + card names: brushed steel, '//' in grey
    const L = textAt(ctx, cs); let x = L.x;
    ctx.shadowColor = 'rgba(0,0,0,0.85)'; ctx.shadowOffsetY = 2; ctx.shadowBlur = 1.5;
    for (const [t, cls] of runsOf(el)) { ctx.fillStyle = cls === 'menu-sl' ? '#9aa1a9' : vgrad(ctx, 0, h, STEEL); ctx.fillText(t, x, L.y); x += ctx.measureText(t).width; }
  },
  title(ctx, w, h, el, cs) {                                          // OFFROAD chrome | 2D gold: dark outline, drop shadow, gold glow
    const L = textAt(ctx, cs), t = el.dataset.t || el.textContent, acc = el.classList.contains('t-acc'), fs = L.fs;
    ctx.lineJoin = 'round'; ctx.lineWidth = 0.09 * fs; ctx.strokeStyle = ctx.fillStyle = '#17120d';
    if (acc) { ctx.shadowColor = el.classList.contains('t-fail') ? 'rgba(255,60,30,0.55)' : 'rgba(255,150,30,0.55)'; ctx.shadowBlur = 0.35 * fs; ctx.fillText(t, L.x, L.y); }
    ctx.shadowColor = 'rgba(0,0,0,0.7)'; ctx.shadowBlur = 0.08 * fs; ctx.shadowOffsetY = 0.05 * fs;
    ctx.strokeText(t, L.x, L.y); ctx.fillText(t, L.x, L.y);
    ctx.shadowColor = 'transparent'; ctx.fillStyle = vgrad(ctx, 0, h, el.classList.contains('t-fail') ? RED_T : acc ? GOLD_T : CHROME); ctx.fillText(t, L.x, L.y);
  },
  tag(ctx, w, h, el, cs) {                                            // difficulty pill: EASY green, MEDIUM / HARD orange
    const g = el.classList.contains('easy') || !(el.classList.contains('hard') || el.classList.contains('medium'));
    pill(ctx, 0.75, 0.75, w - 1.5, h - 1.5);
    ctx.fillStyle = vgrad(ctx, 0, h, g ? [[0, '#2c7a3c'], [1, '#145024']] : [[0, '#ffb238'], [1, '#d8700c']]); ctx.fill();
    ctx.lineWidth = 1.5; ctx.strokeStyle = g ? '#59e07a' : '#ffd27a'; ctx.stroke();
    const L = textAt(ctx, cs); ctx.textAlign = 'center';
    ctx.shadowColor = g ? 'rgba(0,0,0,0.6)' : 'rgba(90,40,0,0.8)'; ctx.shadowOffsetY = 1; ctx.shadowBlur = 1;
    ctx.fillStyle = g ? '#c8ffd2' : '#fff4e0'; ctx.fillText(cs.textTransform === 'uppercase' ? el.textContent.toUpperCase() : el.textContent, w / 2, L.y);
  },
  emblem(ctx, w, h, el) {                                             // finish / fail emblem: gold chequered flag | face hiding in its hands
    const S = Math.min(w * 0.95, h * 1.3), k = S / 24, ox = (w - S) / 2, oy = (h - S) / 2, dark = '#17120d';
    ctx.save(); ctx.translate(ox, oy); ctx.scale(k, k); ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    if (el.dataset.icon === 'facepalm') {
      ctx.shadowColor = 'rgba(255,90,40,0.55)'; ctx.shadowBlur = 5;
      ctx.beginPath(); ctx.arc(12, 13, 8.6, 0, Math.PI * 2); ctx.fillStyle = vgrad(ctx, 4, 22, GOLD_T); ctx.fill();
      ctx.shadowColor = 'transparent'; ctx.lineWidth = 1.3; ctx.strokeStyle = dark; ctx.stroke();
      ctx.beginPath(); ctx.moveTo(8.6, 18); ctx.quadraticCurveTo(10.3, 16.6, 12, 17.7); ctx.quadraticCurveTo(13.7, 18.8, 15.4, 17.4); ctx.stroke();   // wobbly grimace
      for (const [cx, rot] of [[7.3, -0.22], [16.7, 0.22]]) {         // two palms over the eyes: 4 rounded fingers + palm each
        ctx.save(); ctx.translate(cx, 10.6); ctx.rotate(rot);
        const hand = new Path2D(), cap = (x, y0, y1, r) => { hand.moveTo(x - r, y1); hand.lineTo(x - r, y0 + r); hand.arc(x, y0 + r, r, Math.PI, 0); hand.lineTo(x + r, y1); };
        hand.moveTo(-3.6, -0.6); hand.lineTo(3.6, -0.6); hand.lineTo(3.6, 1.6); hand.quadraticCurveTo(3.6, 4, 0, 4); hand.quadraticCurveTo(-3.6, 4, -3.6, 1.6); hand.closePath();
        [[-2.7, -3.6], [-0.9, -4.6], [0.9, -4.5], [2.7, -3.3]].forEach(([x, y0]) => { cap(x, y0, 0.2, 0.9); hand.closePath(); });
        ctx.fillStyle = vgrad(ctx, -4.6, 4, [[0, '#fff0bf'], [1, '#f2b23c']]);
        ctx.lineWidth = 1.1; ctx.strokeStyle = dark; ctx.stroke(hand); ctx.fill(hand);
        ctx.lineWidth = 0.55; ctx.beginPath(); for (const x of [-1.8, 0, 1.8]) { ctx.moveTo(x, -2.6); ctx.lineTo(x, 0); } ctx.stroke();   // finger gaps
        ctx.restore();
      }
      ctx.beginPath(); ctx.moveTo(20.6, 4.2); ctx.quadraticCurveTo(22.4, 7, 20.6, 7.6); ctx.quadraticCurveTo(18.8, 7, 20.6, 4.2);   // sweat drop
      ctx.fillStyle = '#8fd4ff'; ctx.fill(); ctx.lineWidth = 0.7; ctx.strokeStyle = dark; ctx.stroke();
    } else {
      ctx.shadowColor = 'rgba(255,150,30,0.55)'; ctx.shadowBlur = 5;
      const flag = new Path2D('M6 4.2c3-1.6 6 1.6 9 0s4.5-.6 4.5-.6v9c-1.5 0-1.5-1-4.5.6s-6-1.6-9 0z');
      ctx.fillStyle = vgrad(ctx, 3, 14, GOLD_T); ctx.fill(flag); ctx.shadowColor = 'transparent';
      ctx.save(); ctx.clip(flag); ctx.fillStyle = '#2a1a08';
      for (let i = 0; i < 5; i++) for (let j = 0; j < 4; j++) if ((i + j) % 2) ctx.fillRect(6 + i * 2.7, 2.4 + j * 2.8, 2.7, 2.8);
      ctx.restore();
      ctx.lineWidth = 1.1; ctx.strokeStyle = dark; ctx.stroke(flag);
      ctx.beginPath(); ctx.moveTo(5.6, 3); ctx.lineTo(5.6, 22); ctx.lineWidth = 2.6; ctx.strokeStyle = dark; ctx.stroke();
      ctx.lineWidth = 1.4; ctx.strokeStyle = vgrad(ctx, 3, 22, STEEL); ctx.stroke();
    }
    ctx.restore();
  },
  dots(ctx, w, h, el) {                                               // difficulty: 1-3 glowing dots (easy green · medium yellow · hard red)
    const lvl = el.classList.contains('hard') ? 3 : el.classList.contains('medium') ? 2 : 1, col = ['#4be37a', '#ffd23c', '#ff5a44'][lvl - 1];
    const r = Math.min(h * 0.32, w / 9), gap = (w - 6 * r) / 2;
    for (let i = 0; i < 3; i++) {
      const x = r + i * (2 * r + gap * 0.9) + 1, y = h / 2;
      ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2);
      if (i < lvl) {
        ctx.shadowColor = col; ctx.shadowBlur = r * 1.6;
        const g2 = ctx.createRadialGradient(x - r * 0.35, y - r * 0.35, 0, x, y, r); g2.addColorStop(0, '#ffffff'); g2.addColorStop(0.35, col); g2.addColorStop(1, col);
        ctx.fillStyle = g2; ctx.fill(); ctx.shadowColor = 'transparent';
      } else { ctx.fillStyle = '#23262c'; ctx.fill(); ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(255,255,255,0.22)'; ctx.stroke(); }
    }
  },
  ihead(ctx, w, h, el, cs) {                                          // column header: steel pictogram + grey '//'
    const L = textAt(ctx, cs), sz = L.fs * 1.25;
    ctx.shadowColor = 'rgba(0,0,0,0.85)'; ctx.shadowOffsetY = 1.5; ctx.shadowBlur = 1.5;
    paintIcon(ctx, el.dataset.icon, L.x, L.y - L.fs * 1.02, sz, { color: vgrad(ctx, 0, h, STEEL), lw: 2.2 });
    ctx.fillStyle = '#9aa1a9'; ctx.fillText('//', L.x + sz + L.fs * 0.35, L.y);
  },
  spec(ctx, w, h, el) {                                               // showroom spec plate: riveted dark metal, 4 icon columns, 5-notch gold bars
    const st = (el.dataset.stats || '').split(',').map(Number), icons = ['power', 'speed', 'grip', 'weight'];
    const R2 = 7;
    const plate = () => { ctx.beginPath(); ctx.moveTo(R2, 0.5); ctx.arcTo(w - 0.5, 0.5, w - 0.5, h - 0.5, R2); ctx.arcTo(w - 0.5, h - 0.5, 0.5, h - 0.5, R2); ctx.arcTo(0.5, h - 0.5, 0.5, 0.5, R2); ctx.arcTo(0.5, 0.5, w - 0.5, 0.5, R2); ctx.closePath(); };
    plate(); ctx.fillStyle = vgrad(ctx, 0, h, [[0, 'rgba(52,56,63,0.92)'], [0.5, 'rgba(30,33,38,0.92)'], [1, 'rgba(20,22,26,0.94)']]); ctx.fill();
    ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(255,190,80,0.55)'; ctx.stroke();
    ctx.strokeStyle = 'rgba(255,255,255,0.14)'; ctx.beginPath(); ctx.moveTo(R2, 2); ctx.lineTo(w - R2, 2); ctx.stroke();
    for (const [x, y] of [[4.5, h / 2], [w - 4.5, h / 2]]) {
      const g2 = ctx.createRadialGradient(x - 0.6, y - 0.6, 0, x, y, 2.2); g2.addColorStop(0, '#c9ced3'); g2.addColorStop(0.6, '#6a7078'); g2.addColorStop(1, 'rgba(0,0,0,0)');
      ctx.fillStyle = g2; ctx.beginPath(); ctx.arc(x, y, 2.2, 0, Math.PI * 2); ctx.fill();
    }
    const n = 4, cw = (w - 12) / n, isz = Math.max(10, Math.min(cw - 6, 24, h * 0.3)), top = 6 + isz + 4, bot = h - 6, seg = 5, gap = 1.5, sh = (bot - top - gap * (seg - 1)) / seg, bw = Math.min(9, cw * 0.42);
    for (let i = 0; i < n; i++) {
      const cx = 6 + cw * (i + 0.5);
      paintIcon(ctx, icons[i], cx - isz / 2, 5, isz, { color: vgrad(ctx, 5, 5 + isz, STEEL), lw: 2.3 });
      for (let k = 0; k < seg; k++) {
        const y = bot - (k + 1) * sh - k * gap, lit = k < (st[i] || 0);
        ctx.beginPath(); ctx.rect(cx - bw / 2, y, bw, sh);
        if (lit) { ctx.shadowColor = 'rgba(255,170,40,0.7)'; ctx.shadowBlur = 4; ctx.fillStyle = vgrad(ctx, y, y + sh, GOLD_B); ctx.fill(); ctx.shadowColor = 'transparent'; }
        else { ctx.fillStyle = 'rgba(0,0,0,0.45)'; ctx.fill(); ctx.lineWidth = 0.8; ctx.strokeStyle = 'rgba(255,255,255,0.1)'; ctx.stroke(); }
      }
    }
  },
  start(ctx, w, h, el, cs) {                                          // molten-gold pill: rim, top light, bottom shade, gloss, 4 rivets, embossed text
    const fs = parseFloat(cs.fontSize);
    pill(ctx, 0, 0, w, h); ctx.fillStyle = vgrad(ctx, 0, h, GOLD_B); ctx.fill();
    ctx.save(); pill(ctx, 0, 0, w, h); ctx.clip();
    ctx.fillStyle = 'rgba(255,255,230,0.75)'; ctx.beginPath(); ctx.rect(-5, -5, w + 10, h + 10); pill(ctx, 0, 0.08 * fs, w, h, true); ctx.fill('evenodd');
    ctx.fillStyle = vgrad(ctx, h * 0.55, h, [[0, 'rgba(140,70,0,0)'], [1, 'rgba(140,70,0,0.4)']]); ctx.fillRect(0, h * 0.55, w, h * 0.45);
    pill(ctx, w * 0.08, h * 0.06, w * 0.84, h * 0.38); ctx.fillStyle = vgrad(ctx, h * 0.06, h * 0.44, [[0, 'rgba(255,255,255,0.55)'], [1, 'rgba(255,255,255,0)']]); ctx.fill();
    ctx.restore();
    pill(ctx, 0.75, 0.75, w - 1.5, h - 1.5); ctx.lineWidth = 1.5; ctx.strokeStyle = '#fff0b0'; ctx.stroke();
    const c = 0.13 * fs, rr = 0.075 * fs;                             // rivet centres 0.7em / 0.22em in from the ends
    for (const [x, y] of [[0.7 * fs + c, 0.22 * fs + c], [0.7 * fs + c, h - 0.22 * fs - c], [w - 0.7 * fs - c, 0.22 * fs + c], [w - 0.7 * fs - c, h - 0.22 * fs - c]]) {
      const g = ctx.createRadialGradient(x, y, 0, x, y, rr); g.addColorStop(0, '#8a8f96'); g.addColorStop(0.55, '#8a8f96'); g.addColorStop(0.8, '#3a3d42'); g.addColorStop(1, 'rgba(0,0,0,0)');
      ctx.fillStyle = g; ctx.beginPath(); ctx.arc(x, y, rr, 0, Math.PI * 2); ctx.fill();
    }
    ctx.shadowColor = 'rgba(255,240,190,0.7)'; ctx.shadowOffsetY = 1;
    if (el.dataset.glyph) { const sz = h * 0.62; paintIcon(ctx, el.dataset.glyph, w / 2 - sz / 2, h / 2 - sz / 2, sz, { color: '#23170a', lw: 2.8 }); return; }
    const L = textAt(ctx, cs); ctx.textAlign = 'center';
    ctx.fillStyle = '#23170a'; ctx.fillText(el.firstChild.textContent, w / 2, L.y);
  },
};
/** Paint host el with PAINT[kind] on an overlaid canvas (pad = overflow room in em for outlines / glows); repaints on resize + font load. */
function canvasPaint(el, kind, pad, offs) {
  const cv = document.createElement('canvas'); cv.className = 'cv-paint'; cv.setAttribute('aria-hidden', 'true');
  el.classList.add('cv-host', 'cv-' + kind); el.appendChild(cv);
  const draw = () => {
    const w = el.offsetWidth, h = el.offsetHeight;
    if (!w || !h) return;
    const cs = getComputedStyle(el), p = pad * parseFloat(cs.fontSize), dpr = Math.min(window.devicePixelRatio || 1, 3);
    const W = w + 2 * p, H = h + 2 * p, bl = parseFloat(cs.borderLeftWidth) || 0, bt = parseFloat(cs.borderTopWidth) || 0;
    Object.assign(cv.style, { left: `${-p - bl}px`, top: `${-p - bt}px`, width: `${W}px`, height: `${H}px` });
    if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
    const ctx = cv.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, p * dpr, p * dpr); ctx.clearRect(-p, -p, W, H);
    PAINT[kind](ctx, w, h, el, cs);
  };
  if (typeof ResizeObserver === 'function') { const ro = new ResizeObserver(draw); ro.observe(el); offs.push(() => ro.disconnect()); }
  return draw;
}

/**
 * Start screen = showroom: MENU pill (top-left), OFFROAD2D title, LEVEL column (left), VEHICLE column (right), gold START pill;
 * the centre stays empty for the live canvas (truck on the glass platform). opts { onMenu?() MENU pill / Esc, blocked?() true = keys off }.
 */
export function createMenu(root, manifest, initial, onStart, onSelect, opts = {}) {   // onSelect?(selection): card picked (live preview)
  const offs = [];
  const on = (t, type, fn) => { t.addEventListener(type, fn); offs.push(() => t.removeEventListener(type, fn)); };
  const mk = (cls, text, parent, tag = 'div') => {
    const el = document.createElement(tag);
    el.className = cls; if (text) el.textContent = text;
    parent.appendChild(el); return el;
  };
  const levels = manifest.levels || [], vehicles = manifest.vehicles || [];
  const pick = (list, id) => (list.find(x => x.id === id) || list[0] || {}).id;
  const selection = { level: pick(levels, initial.level), vehicle: pick(vehicles, initial.vehicle) };
  root.textContent = '';
  root.classList.add('menu');
  const menuBtn = mk('menu-pill', '', root);
  menuBtn.innerHTML = svg('menu'); menuBtn.setAttribute('aria-label', 'menu'); menuBtn.title = 'menu (Esc)';
  menuBtn.setAttribute('role', 'button');
  on(menuBtn, 'pointerup', e => { e.preventDefault(); opts.onMenu?.(); });
  const title = mk('menu-title', '', root, 'h1');            // glass title: OFFROAD (white / silver) + 2D (amber-gold)
  for (const [cls, t] of [['t-main', 'OFFROAD'], ['t-acc', '2D']]) mk(cls, t, title, 'span').dataset.t = t;   // data-t: dark outline layer (CSS ::before)
  const bestEls = {}, starEls = {};
  const section = (title, list, key, slots, icon) => {
    const col = mk('menu-col ' + key, '', root);
    const hd = mk('menu-h', '\u2003\u2002 //', col); hd.dataset.icon = icon; hd.setAttribute('aria-label', title);
    const row = mk('menu-cards', '', col);
    const cards = [];
    for (const it of list) {
      const c = mk('card', '', row);
      c.setAttribute('role', 'button');
      if (it.thumb) {                                         // level teaser: real in-game snapshot (tools/level-thumbs.mjs)
        const img = mk('card-thumb', '', c, 'img'); img.alt = ''; img.decoding = 'async'; img.draggable = false;
        img.src = './' + it.thumb.replace(/^\.?\//, ''); img.onerror = () => img.remove();
      }
      const body = mk('card-body', '', c);
      mk('card-name', it.name || it.id, body);                // level: name / difficulty tag + best / subtitle; vehicle: name / trait
      if (key === 'level') {
        const meta = mk('card-meta', '', body);
        if (it.difficulty) { const dd = mk('card-dots ' + it.difficulty.toLowerCase(), '', meta, 'span'); dd.setAttribute('aria-label', it.difficulty); dd.title = it.difficulty; }
        bestEls[it.id] = mk('card-best', '', meta, 'span');
      }
      const sub = it.blurb || it.trait || (Array.isArray(it.chips) ? it.chips.slice(0, 2).join(' · ') : '');
      if (sub) mk('card-blurb', sub, body);
      if (key === 'level') {                                  // best-stars strip at the card's right edge (empty until cleared)
        const st = mk('card-stars', '', c); starEls[it.id] = [0, 1, 2].map(() => mk('cs', '☆', st, 'span'));
      }
      on(c, 'pointerup', e => { e.preventDefault(); if (selection[key] === it.id) return; selection[key] = it.id; sync(); onSelect?.({ ...selection }); });
      cards.push([c, it.id]);
    }
    for (let i = list.length; i < slots; i++) { const c = mk('card empty', '', row); mk('card-name', 'EMPTY SLOT', c); }
    return () => cards.forEach(([c, id]) => c.classList.toggle('sel', selection[key] === id));
  };
  const syncL = section('LEVEL', levels, 'level', 0, 'map');
  const syncV = section('VEHICLE', vehicles, 'vehicle', manifest.vehicleSlots || 0, 'truck');
  // spec plate (selected truck): power · top speed · grip · weight, 1-5 gold notches; bottom row beside START (right of it,
  // left of the VEHICLE column): below the showroom platform, so the truck never sits under it
  const spec = mk('spec', '', root); spec.setAttribute('aria-label', 'power, speed, grip, weight');
  const placeSpec = () => {
    const V = root.querySelector('.menu-col.vehicle')?.getBoundingClientRect(), F = root.querySelector('.menu-foot')?.getBoundingClientRect();
    if (!V || !V.width || !F || !F.width) return;
    const room = V.left - F.right - 16, hh = Math.round(Math.min(96, Math.max(48, F.height))), ww = Math.round(Math.min(hh * 2.1, room));
    spec.classList.toggle('tight', ww < 64);
    Object.assign(spec.style, { left: `${Math.round(F.right + 8 + Math.max(0, (room - ww) / 2))}px`, top: `${Math.round(F.bottom - hh)}px`, width: `${Math.max(0, ww)}px`, height: `${hh}px` });
  };
  const syncSpec = () => {
    const vi = vehicles.find(x => x.id === selection.vehicle) || {}, st = vi.stats || {};
    spec.dataset.stats = [st.power, st.speed, st.grip, st.weight].map(x => x | 0).join(',');
    spec.classList.toggle('gone', !vi.stats);
    spec.title = vi.stats ? `power ${st.power} · speed ${st.speed} · grip ${st.grip} · weight ${st.weight}` : '';
  };
  const sync = () => { syncL(); syncV(); syncSpec(); painters.length && requestAnimationFrame(() => specPaint?.()); };
  const painters = [];                                       // forced-dark-proof canvases (see PAINT)
  let specPaint = null, go = null;
  sync();
  const foot = mk('menu-foot', '', root);
  go = mk('go', 'START', foot);
  go.dataset.glyph = 'play'; go.setAttribute('aria-label', 'start'); go.title = 'start (Enter)';
  const status = mk('menu-status', '', foot, 'p');
  if (typeof HTMLCanvasElement === 'function' && document.createElement('canvas').getContext) {
    for (const el of root.querySelectorAll('.menu-title span')) painters.push(canvasPaint(el, 'title', 0.4, offs));
    for (const el of root.querySelectorAll('.menu-h')) painters.push(canvasPaint(el, 'ihead', 0.3, offs));
    for (const el of root.querySelectorAll('.card-name')) painters.push(canvasPaint(el, 'steel', 0.3, offs));
    for (const el of root.querySelectorAll('.card-dots')) painters.push(canvasPaint(el, 'dots', 0.4, offs));
    painters.push(canvasPaint(go, 'start', 0, offs));
    painters.push(specPaint = canvasPaint(spec, 'spec', 0, offs));
  }
  const repaint = () => { placeSpec(); painters.forEach(f => f()); };
  if (document.fonts) { document.fonts.ready.then(repaint); on(document.fonts, 'loadingdone', repaint); }
  on(window, 'resize', repaint); requestAnimationFrame(repaint);
  go.setAttribute('role', 'button');
  on(go, 'pointerup', e => { e.preventDefault(); e.stopPropagation(); onStart({ ...selection }); });
  const key = e => {
    if (root.classList.contains('hidden') || e.repeat || e.ctrlKey || e.metaKey || e.altKey) return;
    if (e.code === 'Escape') { e.preventDefault(); opts.onMenu?.(); return; }
    if (opts.blocked?.() || !(e.code === 'Enter' || e.code === 'Space')) return;
    e.preventDefault(); onStart({ ...selection });
  };
  on(window, 'keydown', key);
  return {
    selection,
    /** Programmatic pick (e.g. NEXT level): highlights the card, no onSelect. */
    select(key, id) { if (key in selection) { selection[key] = id; sync(); } },
    show() { root.classList.remove('hidden'); requestAnimationFrame(repaint); },
    hide() { root.classList.add('hidden'); },
    setStatus(t) { status.textContent = t; },
    /** Showroom stage between the columns (CSS px): {left, right, top, bottom} = free centre for the truck + platform. */
    stage() {
      const r = c => root.querySelector(c)?.getBoundingClientRect();
      const L = r('.menu-col.level'), V = r('.menu-col.vehicle'), T = r('.menu-title'), G = r('.go'), W = innerWidth || 1, H = innerHeight || 1;
      return { left: L ? L.right : 0, right: V ? V.left : W, top: T ? T.bottom : 0, bottom: G ? G.top : H };
    },
    /** bests = { levelId: { time (s), vehicle? (display name) } } -> "BEST 63.2 s · Rock Buggy" beside each level card's tag. */
    setBests(bests = {}) {
      for (const id in starEls) {                             // highest star count ever on that level (best score; old time-only bests = 1)
        const b = bests[id], n = !b || !isFinite(b.time) ? 0 : isFinite(b.score) ? starsOf(b.score) : 1;
        starEls[id].forEach((e, i) => { e.textContent = i < n ? '★' : '☆'; e.classList.toggle('on', i < n); });
      }
      for (const id in bestEls) {
        const b = bests[id], el = bestEls[id];
        el.textContent = '';
        if (!b || !isFinite(b.time)) continue;
        el.innerHTML = svg('clock') + `${b.time.toFixed(1)}s`; el.title = b.vehicle ? 'best · ' + b.vehicle : 'best';
      }
    },
    destroy() { offs.forEach(f => f()); root.textContent = ''; },
  };
}

