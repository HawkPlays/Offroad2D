// engine/hud.js — mobile landscape touch controls + gauges (DOM only; no physics / canvas code).
// Every control captures its own pointer, so Gas + Steer + Brake + Winch + toggles work simultaneously.
// Two-thumb layout: left = steering (wheel or ◄ ►) over BRAKE · WINCH · REV; right = XFER lever + GAS; bottom-centre = KEY 4WD F-LOCK R-LOCK;
// top bar = MENU (pause panel: resume, levels, CAM, DEBUG, MUTE, settings) + RESET; top-right = telemetry cluster.

const MOMENTARY = [
  { id: 'brake', label: 'BRAKE', cls: 'pedal brake', keys: ['ArrowDown', 'KeyS', 'Space'] },
  { id: 'gas',   label: 'GAS',   cls: 'pedal gas',   keys: ['ArrowUp', 'KeyW'] },
  { id: 'winch', label: 'WINCH', cls: 'btn round winch', keys: ['KeyE'] },
  { id: 'left',  label: '◄',     cls: 'btn steer',   keys: ['ArrowLeft', 'KeyA'] },    // steer: state.steer = clamp(wheel + right - left)
  { id: 'right', label: '►',     cls: 'btn steer',   keys: ['ArrowRight', 'KeyD'] },
];
// transfer-case gates, top -> bottom of the shifter; key 4 / L cycles (Shift = back, wraps), Q = up toward H, Z = down toward L-
const RANGES = ['H', 'L+', 'L', 'L-'];
const DRIVE = ['fourWD', 'lockF', 'lockR'];          // drivetrain toggles that fire onShift('drive')
const TOGGLES = [                                    // reset to off on every run (resetControls)
  { id: 'reverse', label: 'REV',    cls: 'btn round rev', key: 'KeyV' },
  { id: 'fourWD',  label: '4WD',    cls: 'btn tog',  key: 'Digit1' },
  { id: 'lockF',   label: 'F-LOCK', cls: 'btn tog lock',  key: 'Digit2' },
  { id: 'lockR',   label: 'R-LOCK', cls: 'btn tog lock', key: 'Digit3' },
];
const OPTIONS = [                                    // pause-panel switches; persist across runs
  { id: 'debug', label: 'DEBUG', key: 'KeyG' },
  { id: 'mute',  label: 'MUTE',  key: 'KeyM' },
];
/** Saved HUD prefs (o2d.ui): steer mode (MENU), control size scales + tilt range (MENU -> SETTINGS), tiltZero = tilt calibration (°). */
export const UI_DEFAULTS = { steer: 'wheel', wheel: 1, gas: 1, brake: 1, tiltMax: 25, tiltZero: 0 };
export const STEER_MODES = ['buttons', 'wheel', 'tilt'];
const UI_RANGE = { wheel: [0.7, 1.4], gas: [0.7, 1.4], brake: [0.7, 1.4], tiltMax: [12, 40], tiltZero: [-60, 60] };
const WHEEL_MAX = 2.36;                              // wheel lock-to-centre angle (135°) = full steer
const TILT_DEAD = 3;                                 // tilt deadzone (°) around the calibrated centre
const TILT_HINT = 1500, TILT_WAIT = 6000;            // ms of visible time without tilt data: "waiting" toast, then fall back to WHEEL

/** Phone roll in the screen plane (°, + = clockwise = steer right) from DeviceOrientation beta / gamma (°) and the screen
 *  orientation angle (°, 0 / 90 / 180 / 270), so it reads the same on either landscape side. Up in device axes =
 *  (−sinγ·cosβ, sinβ, cosγ·cosβ), rotated into screen axes (sx, sy). Held up (sy large): the exact roll atan2(−sx, sy), not
 *  shrunk by how far the phone is tipped back; lying flat (sy -> 0, where atan2 turns to noise): asin(−sx), i.e. tilting the
 *  edges; smoothstep blend between (sy 0.15 .. 0.5). */
export function tiltRoll(beta, gamma, angle = 0) {
  const r = Math.PI / 180, ux = -Math.sin(gamma * r) * Math.cos(beta * r), uy = Math.sin(beta * r), a = angle * r;
  const sx = ux * Math.cos(a) - uy * Math.sin(a), sy = ux * Math.sin(a) + uy * Math.cos(a);
  const flat = Math.asin(Math.max(-1, Math.min(1, -sx))), k = Math.max(0, Math.min(1, (sy - 0.15) / 0.35)), w = k * k * (3 - 2 * k);
  return (w > 0 ? w * Math.atan2(-sx, sy) + (1 - w) * flat : flat) / r;
}

/** Overall damage 0..1 for the single DMG bar: body, worst shock, front / rear axle (shaft stress, snapped = 1).
 *  The worst part dominates and the rest add a quarter, so one failure reads critical. Heat is in the status line instead. */
export function overallDamage(h) {
  const parts = [h.body || 0, Math.max(0, ...(h.shock || [0])), h.axleBroken[0] ? 1 : h.axleStress[0] || 0, h.axleBroken[1] ? 1 : h.axleStress[1] || 0];
  const worst = Math.max(...parts), sum = parts.reduce((a, b) => a + b, 0);
  return Math.min(1, worst + 0.25 * (sum - worst));
}

/**
 * @param root     container element (positioned over the canvas)
 * @param handlers { onRecover(), onRestart(), onPlayAgain(), onMenu() (level select), onPause?(paused), active?() (false: keydowns ignored), cam? '2.5D'|'2D', onCam?(mode),
 *                   ui? {steer, wheel, gas, brake} (saved sizes), onUi?(ui), onShift?(kind 'notch'|'commit'|'drive'|'key', info) }   (onPlayAgain falls back to onRestart)
 * @returns { state, resetControls(), update(vehicleState), toast(text), hint(text, ms?), showFinish(detail, {best?, hardcore?}), hideFinish(),
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
  for (const k in UI_RANGE) ui[k] = Math.min(UI_RANGE[k][1], Math.max(UI_RANGE[k][0], Number.isFinite(+ui[k]) ? +ui[k] : UI_DEFAULTS[k]));
  let paused = false;

  root.classList.add('hud');
  const zones = {};
  for (const z of ['tl', 'tr', 'bl', 'br', 'bc']) zones[z] = mk('hud-zone ' + z, '', root);
  // left thumb: bottom row BRAKE · WINCH · REV (hold WINCH while the right thumb feathers GAS), steering above (zones.bl = column-reverse)
  const thumbRow = mk('thumb-row', '', zones.bl), steerBox = mk('steer-box', '', zones.bl);
  const steerRow = mk('steer-row', '', steerBox);
  const place = { brake: thumbRow, reverse: thumbRow, winch: thumbRow, gas: zones.br, fourWD: zones.bc, lockF: zones.bc, lockR: zones.bc, left: steerRow, right: steerRow };
  const menuBtn = mk('btn small menu', 'MENU', zones.tl);

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

  // reset: tap = recover in place, hold 0.8 s = restart level
  const reset = mk('btn small reset', 'RESET', zones.tl);
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
    const el = els[t.id] = mk(t.cls, t.label, place[t.id]);
    on(el, 'pointerdown', e => { e.preventDefault(); state[t.id] = !state[t.id]; el.classList.toggle('on', state[t.id]); });
  }
  // TRANSFER-CASE SHIFTER (replaces the GEAR button): vertical notched gate H (top) · L+ · L · L- (bottom). state.range = requested gate,
  // state.low = range !== 'H'. Drag the knob (pointer capture, detent per gate) or tap above / below it to step one gate (tap on the knob =
  // next, wraps). Moving shift (> SHIFT_SPEED): knob sits on the requested gate with a pulsing amber sync ring until vehicle range commits.
  // handlers.onShift?(kind, info): 'notch' {range, from} each gate the lever passes · 'commit' {range} when v.range engages · 'drive' {id, on} 4WD / locker toggles.
  const shiftHook = (kind, info) => { try { handlers.onShift?.(kind, info); } catch (_) { /* audio is optional */ } };
  for (const t of TOGGLES) if (DRIVE.includes(t.id)) on(els[t.id], 'pointerdown', () => shiftHook('drive', { id: t.id, on: state[t.id] }));
  state.range = 'H'; state.low = false;
  const shifter = els.low = mk('shifter', '', zones.br);
  shifter.setAttribute('role', 'slider'); shifter.setAttribute('aria-label', 'transfer case range');
  mk('sh-title', 'XFER', shifter);
  const slot = mk('sh-slot', '', shifter), gateLbl = [];
  RANGES.forEach((r, i) => {
    mk('sh-notch', '', slot).style.setProperty('--g', i);
    const l = gateLbl[i] = mk('sh-lbl', r, shifter); l.style.setProperty('--g', i);
  });
  const knob = mk('sh-knob', '', shifter);
  mk('sh-sync', '', knob);
  const placeKnob = gi => shifter.style.setProperty('--gi', gi.toFixed(3));
  const setRange = (r, quiet = false) => {
    const from = state.range, i = RANGES.indexOf(r);
    state.range = r; state.low = r !== 'H';
    placeKnob(i);
    shifter.setAttribute('aria-valuetext', r);
    shifter.classList.toggle('low', state.low);
    gateLbl.forEach((l, k) => l.classList.toggle('req', k === i));
    if (!quiet && r !== from) {
      shiftHook('notch', { range: r, from });
      try { navigator.vibrate?.(10); } catch (_) { /* no haptics */ }
    }
  };
  const stepRange = (d, wrap = false) => {
    const i = RANGES.indexOf(state.range) + d, n = RANGES.length;
    if (wrap || (i >= 0 && i < n)) setRange(RANGES[(i + n) % n]);
  };
  const cycleRange = (d = 1) => stepRange(d, true);
  let shPid = null, shY0 = 0, shG0 = 0, shDrag = false;
  const gateStep = () => { const r = slot.getBoundingClientRect(); return (r.height || 90) / (RANGES.length - 1); };
  on(shifter, 'pointerdown', e => {
    e.preventDefault();
    if (shPid !== null) return;
    shPid = e.pointerId; shY0 = e.clientY; shG0 = RANGES.indexOf(state.range); shDrag = false;
    try { shifter.setPointerCapture(shPid); } catch (_) { /* synthetic pointers */ }
    shifter.classList.add('grab');
  });
  on(shifter, 'pointermove', e => {
    if (e.pointerId !== shPid) return;
    e.preventDefault();
    const dy = e.clientY - shY0;
    if (!shDrag && Math.abs(dy) < 6) return;
    if (!shDrag) { shDrag = true; shifter.classList.add('drag'); }
    const gi = Math.min(RANGES.length - 1, Math.max(0, shG0 + dy / gateStep()));
    const gate = Math.round(gi);
    if (RANGES[gate] !== state.range) setRange(RANGES[gate]);
    // detent feel: the knob lags toward the nearest notch (~60 % of the way between gates)
    placeKnob(gate + (gi - gate) * 0.6);
  });
  const shUp = e => {
    if (e.pointerId !== shPid) return;
    shPid = null;
    shifter.classList.remove('grab', 'drag');
    if (shDrag) { placeKnob(RANGES.indexOf(state.range)); return; }     // release: snap into the gate
    const kr = knob.getBoundingClientRect();
    if (e.type === 'pointercancel') return;
    if (e.clientY < kr.top) stepRange(-1);                // tap above the knob: one gate up (toward H)
    else if (e.clientY > kr.bottom) stepRange(1);         // tap below: one gate down (toward L-)
    else cycleRange(1);                                   // tap the knob itself: next gate, L- wraps to H
  };
  on(shifter, 'pointerup', shUp); on(shifter, 'pointercancel', shUp);
  on(shifter, 'lostpointercapture', e => { if (e.pointerId === shPid) { shPid = null; shifter.classList.remove('grab', 'drag'); placeKnob(RANGES.indexOf(state.range)); } });
  let committed = null;                                   // last v.range seen by update() (null = fresh run: no commit sound)
  // IGNITION KEY (left of 4WD): tap / key K = one key turn (state.ignition++): engine off or stalled -> crank, running -> shut off.
  // update() sets .off (pulsing amber: engine off / stalled), .crank (key turned, starter running) or .run (green); icon only.
  const keyBtn = els.ignition = mk('btn key off', '', zones.bc);
  zones.bc.insertBefore(keyBtn, zones.bc.firstChild);
  keyBtn.setAttribute('role', 'button'); keyBtn.setAttribute('aria-label', 'ignition key');
  keyBtn.innerHTML = '<svg class="key-ico" viewBox="0 0 40 20" aria-hidden="true"><circle cx="9" cy="10" r="7"/><circle class="key-hole" cx="7" cy="10" r="2.2"/>'
    + '<path d="M15 8.2h21l2.5 1.8-2.5 1.8h-3l-1.6 2.6-1.6-2.6-1.6 2.6-1.6-2.6H15z"/></svg>';
  let keyMode = 'off';
  const turnKey = () => {
    state.ignition = (state.ignition | 0) + 1;
    keyBtn.classList.remove('turn'); void keyBtn.offsetWidth; keyBtn.classList.add('turn');
    shiftHook('key', { n: state.ignition, mode: keyMode });
    try { navigator.vibrate?.(15); } catch (_) { /* no haptics */ }
  };
  on(keyBtn, 'pointerdown', e => { e.preventDefault(); turnKey(); });
  // fresh run / new truck: 2WD, lockers open, range H, engine-off key, pause panel closed; DOM classes synced immediately (sync ring cleared).
  // DEBUG / MUTE / CAM / sizes are settings and survive runs.
  const resetControls = () => {
    state.ignition = 0;
    for (const t of TOGGLES) { state[t.id] = false; els[t.id].classList.remove('on', 'pending', 'blink'); }
    shifter.classList.remove('sync');
    gateLbl.forEach(l => l.classList.remove('cur'));
    committed = null;
    setRange('H', true);
    if (paused) setPause(false);
  };
  // SHARED STEERING PIPELINE: state.steer = clamp(analog source of the active mode + right − left). Sources: steerIn.wheel (virtual
  // wheel), steerIn.tilt (phone tilt); BUTTONS mode has no analog source (◄ ► / A D ← → only). Keys work in every mode.
  // Steering also flips the camera to 2.5D (the lateral trail is only visible there); CAM: 2D stays a manual toggle.
  const steerIn = { wheel: 0, tilt: 0 };
  let wheelEl = null, wheelRot = null, wheelShown = 0;
  const syncSteer = () => {
    const analog = ui.steer === 'buttons' ? 0 : steerIn[ui.steer] || 0;
    state.steer = Math.max(-1, Math.min(1, analog + state.right - state.left));
    if (Math.abs(state.steer) > 0.05 && state.cam !== '2.5D') setCam('2.5D');
    const a = Math.round(state.steer * WHEEL_MAX * 57.2958);
    if (wheelRot && a !== wheelShown) { wheelShown = a; wheelRot.style.transform = `rotate(${a}deg)`; }
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
    const el = els[m.id] = mk(m.cls, pedals[m.id] ? '' : m.label, place[m.id]);
    if (m.id === 'left' || m.id === 'right') el.setAttribute('aria-label', m.id === 'left' ? 'steer left' : 'steer right');
    if (pedals[m.id]) {
      pedalUi[m.id] = { fill: mk('pedal-fill', '', el), lbl: mk('pedal-lbl', m.label, el), v: -1 };
      slidePedal(el, m.id);
    } else holdButton(el, down => press(m.id, down, 'touch'));
  }
  // VIRTUAL STEERING WHEEL (MENU -> SETTINGS: wheel | ◄ ► buttons): grab anywhere on the rim and turn; angle accumulates across the
  // ±180° seam, clamps at ±WHEEL_MAX (= full lock, analog steer −1..1) and springs back to centre on release. Keys still add ±1.
  wheelEl = mk('steer-wheel', '', steerBox);
  wheelEl.setAttribute('role', 'slider'); wheelEl.setAttribute('aria-label', 'steering wheel');
  wheelEl.innerHTML = '<svg class="sw-rot" viewBox="-50 -50 100 100" aria-hidden="true">'
    + '<circle class="sw-rim" r="42"/><circle class="sw-rim-in" r="35"/>'
    + '<path class="sw-grip" d="M-18-40A44 44 0 0 1 18-40" /><path class="sw-grip" d="M-41 14A44 44 0 0 1-43-6"/><path class="sw-grip" d="M41 14A44 44 0 0 0 43-6"/>'
    + '<path class="sw-spoke" d="M-35 2 L-12 -4 L-12 6 L-34 10Z M35 2 L12 -4 L12 6 L34 10Z M-6 12 L6 12 L5 35 L-5 35Z"/>'
    + '<circle class="sw-hub" r="14"/><circle class="sw-hub-ring" r="10.5"/><path class="sw-logo" d="M-6 4L-1-4 2 0 4-3 7 4Z"/>'
    + '<path class="sw-mark" d="M-3-44h6v6h-6z"/></svg><div class="sw-badge">TILT · tap = center</div>';
  wheelRot = wheelEl.firstChild;
  {
    let pid = null, prev = 0, ang = 0;
    const polar = e => {
      const r = wheelEl.getBoundingClientRect(), dx = e.clientX - (r.left + r.width / 2), dy = e.clientY - (r.top + r.height / 2);
      return { a: Math.atan2(dy, dx), k: Math.hypot(dx, dy) / (r.width / 2 || 1) };
    };
    const release = e => {
      if (e.pointerId !== pid) return;
      pid = null; ang = 0; steerIn.wheel = 0; wheelEl.classList.remove('grab'); syncSteer();
    };
    on(wheelEl, 'pointerdown', e => {
      e.preventDefault();
      if (ui.steer === 'tilt') { recenterTilt(); return; }   // TILT mode: the wheel is a read-only indicator, tap = re-center
      if (pid !== null || paused) return;
      pid = e.pointerId; prev = polar(e).a; ang = steerIn.wheel * WHEEL_MAX;
      try { wheelEl.setPointerCapture(pid); } catch (_) { /* synthetic pointers */ }
      wheelEl.classList.add('grab');
    });
    on(wheelEl, 'pointermove', e => {
      if (e.pointerId !== pid) return;
      e.preventDefault();
      const p = polar(e);
      if (p.k < 0.2) return;                              // too close to the hub: angle is noise
      let d = p.a - prev; prev = p.a;
      if (d > Math.PI) d -= 2 * Math.PI; else if (d < -Math.PI) d += 2 * Math.PI;
      ang = Math.max(-WHEEL_MAX, Math.min(WHEEL_MAX, ang + d));
      steerIn.wheel = paused ? 0 : ang / WHEEL_MAX; syncSteer();
    });
    on(wheelEl, 'pointerup', release); on(wheelEl, 'pointercancel', release); on(wheelEl, 'lostpointercapture', release);
  }
  // PHONE TILT: deviceorientation (or deviceorientationabsolute: whichever first brings beta / gamma; some Androids only fire
  // that one) -> tiltRoll(screen angle) − ui.tiltZero, deadzone TILT_DEAD°, full lock at ui.tiltMax°, light low-pass ->
  // steerIn.tilt. DeviceOrientationEvent.requestPermission (iOS 13+, recent Chrome) is asked at once: Chrome grants without a
  // prompt, iOS rejects without a gesture -> asked again from the next tap's touchend / click. Falls back to WHEEL (saved,
  // toast) only on genuine errors: no DeviceOrientationEvent, permission denied / blocked, or no beta/gamma within TILT_WAIT
  // ms of *visible* time with every source reporting empty samples (no sensor) or on http (needs HTTPS). A slow sensor gets
  // a "waiting" toast; after a plain timeout the listener keeps probing (WHEEL meanwhile) and late data switches TILT back.
  // Re-center = MENU ⟲ CENTER, tap the tilt indicator, or key T (saved as ui.tiltZero).
  const TILT_EVENTS = ['deviceorientation', 'deviceorientationabsolute'];
  const tilt = { on: false, live: false, raw: null, f: 0, timers: [], waitTap: false, autoZero: false, from: null, nulls: new Set(), probe: false, rejects: 0 };
  const screenAngle = () => { try { const a = screen.orientation?.angle ?? window.orientation ?? 0; return ((+a || 0) % 360 + 360) % 360; } catch (_) { return 0; } };
  const tiltClear = () => { tilt.timers.forEach(clearTimeout); tilt.timers.length = 0; };
  const onOrient = e => {
    if (tilt.from && e.type !== tilt.from) return;               // locked onto the first source that delivered data
    if (e.beta == null || e.gamma == null || !Number.isFinite(+e.beta) || !Number.isFinite(+e.gamma)) { tilt.nulls.add(e.type); return; }
    tilt.from = e.type;
    tilt.raw = tiltRoll(+e.beta, +e.gamma, screenAngle());
    if (!tilt.live) {
      tilt.live = true; tiltClear();
      if (tilt.probe) { tilt.probe = false; ui.steer = 'tilt'; applyUi(true); toast('TILT ACTIVE', 1600); }
    }
    if (ui.steer !== 'tilt') return;
    if (tilt.autoZero) { tilt.autoZero = false; setTiltZero(tilt.raw); }
    const d = tilt.raw - ui.tiltZero, m = Math.abs(d);
    const target = m <= TILT_DEAD ? 0 : Math.sign(d) * Math.min(1, (m - TILT_DEAD) / (ui.tiltMax - TILT_DEAD));
    tilt.f += (target - tilt.f) * 0.45;
    if (target === 0 && Math.abs(tilt.f) < 0.01) tilt.f = 0;
    steerIn.tilt = paused ? 0 : tilt.f; syncSteer();
  };
  const stopTilt = () => {
    if (tilt.on) for (const t of TILT_EVENTS) window.removeEventListener(t, onOrient);
    tiltClear();
    Object.assign(tilt, { on: false, live: false, f: 0, waitTap: false, from: null, probe: false });
    tilt.nulls.clear();
    steerIn.tilt = 0; syncSteer();
  };
  const failTilt = msg => { stopTilt(); toast(`${msg} · using WHEEL`, 3200); ui.steer = 'wheel'; applyUi(true); };
  const giveUp = () => {                                         // TILT_WAIT ms visible, no beta / gamma yet
    if (tilt.live || !tilt.on) return;
    if (!window.isSecureContext) return failTilt('TILT NEEDS HTTPS');
    if (tilt.nulls.size === TILT_EVENTS.length) return failTilt('NO TILT SENSOR');   // every source fires, all empty: no sensor
    tilt.probe = true; tiltClear();                              // silent so far: maybe slow; keep listening, WHEEL meanwhile
    toast('NO TILT DATA YET · using WHEEL (switches back when it arrives)', 3200);
    ui.steer = 'wheel'; applyUi(true); syncSteer();
  };
  const armTilt = () => {                                        // watchdog counts visible time only (background tab: no events)
    tiltClear();
    if (!tilt.on || tilt.live || tilt.probe || document.hidden) return;
    tilt.timers.push(setTimeout(() => { if (tilt.on && !tilt.live) toast('TILT · waiting for sensor…', 2000); }, TILT_HINT),
      setTimeout(giveUp, TILT_WAIT));
  };
  const goTilt = () => {
    if (ui.steer !== 'tilt' || tilt.on) return;
    for (const t of TILT_EVENTS) window.addEventListener(t, onOrient);
    Object.assign(tilt, { on: true, live: false, from: null, probe: false });
    tilt.nulls.clear();
    armTilt();
  };
  const askTilt = gesture => {                                   // gesture: from a touchend / click (iOS prompts only then)
    tilt.waitTap = false;
    let p;
    try { p = window.DeviceOrientationEvent.requestPermission(); } catch (err) { p = Promise.reject(err); }
    Promise.resolve(p).then(r => {
      if (ui.steer !== 'tilt') return;
      if (r === 'granted') { tilt.rejects = 0; goTilt(); } else failTilt('TILT DENIED · allow Motion & Orientation access');
    }, () => {                                                   // NotAllowedError: not a gesture -> next tap; taps rejected too: blocked
      if (ui.steer !== 'tilt' || tilt.on) return;
      if (gesture && ++tilt.rejects >= 3) return failTilt(window.isSecureContext ? 'TILT BLOCKED' : 'TILT NEEDS HTTPS');
      tilt.waitTap = true; toast('TAP THE SCREEN TO ENABLE TILT', 2800);
    });
  };
  const startTilt = () => {
    if (tilt.on) return;
    const DOE = window.DeviceOrientationEvent;
    if (typeof DOE === 'undefined') return failTilt(window.isSecureContext ? 'TILT NOT SUPPORTED' : 'TILT NEEDS HTTPS');
    if (typeof DOE.requestPermission === 'function') askTilt(false); else goTilt();
  };
  const setTiltZero = deg => { ui.tiltZero = Math.round(Math.max(-60, Math.min(60, deg)) * 10) / 10; tilt.f = 0; applyUi(true); };
  const recenterTilt = () => {
    if (ui.steer !== 'tilt') return;
    if (tilt.raw == null || !tilt.live) { toast('TILT · waiting for sensor'); return; }
    setTiltZero(tilt.raw); toast('TILT CENTERED', 1200);
  };
  for (const t of ['touchend', 'click']) on(window, t, () => { if (tilt.waitTap && ui.steer === 'tilt') askTilt(true); }, true);
  on(document, 'visibilitychange', () => { if (document.hidden) { tiltClear(); releaseAll(); } else armTilt(); });
  const setSteerMode = m => {
    if (!STEER_MODES.includes(m) || (m === ui.steer && !tilt.probe)) return;
    if (ui.steer === 'tilt' || tilt.on || tilt.waitTap) stopTilt();   // an explicit pick also ends a background probe
    steerIn.wheel = 0; ui.steer = m; applyUi(true); syncSteer();
    if (m === 'tilt') { tilt.autoZero = true; tilt.rejects = 0; startTilt(); }   // first sample becomes the centre
  };
  // keyboard routes pedals through setPedal (full press), everything else through press(.., 'key'). held[id] = codes down
  // for that control (A and ← both steer left: letting go of one keeps steering)
  const held = {};
  for (const m of MOMENTARY) held[m.id] = new Set();
  const keyPress = (id, down) => { if (pedals[id]) setPedal(id, 'key', down ? 1 : 0); else press(id, down, 'key'); };
  const keyHold = (id, code, down) => { const h = held[id]; down ? h.add(code) : h.delete(code); keyPress(id, h.size > 0); };
  const releaseKeys = () => { for (const m of MOMENTARY) { held[m.id].clear(); keyPress(m.id, false); } };
  const drawPedal = (id, val) => {
    const u = pedalUi[id], x = Math.round(Math.min(1, Math.max(0, +val || 0)) * 100) / 100;
    if (x !== u.v) { u.v = x; u.fill.style.transform = `scaleY(${x})`; }
  };
  const releaseAll = () => {
    releaseKeys();
    for (const m of MOMENTARY) pedals[m.id] ? setPedal(m.id, 'touch', 0) : press(m.id, false, 'touch');
    steerIn.wheel = 0; steerIn.tilt = 0; syncSteer();
  };

  // PAUSE PANEL (MENU / Esc): sim paused via handlers.onPause; RESUME · LEVELS (start screen) · STEERING ◄ ► | WHEEL | TILT + ⟲ CENTER ·
  // CAM · DEBUG · MUTE · SETTINGS (control sizes, tilt range)
  const pauseEl = mk('pause', '', root), card = mk('pause-card', '', pauseEl);
  const pMain = mk('p-view main', '', card), pSet = mk('p-view set', '', card);
  mk('p-title', 'PAUSED', pMain);
  const pBtn = (parent, cls, label, fn) => { const b = mk('btn p-btn ' + cls, label, parent); on(b, 'pointerup', e => { e.preventDefault(); fn(b); }); return b; };
  const row1 = mk('p-row', '', pMain);
  pBtn(row1, 'resume', 'RESUME', () => setPause(false));
  pBtn(row1, 'levels', 'LEVELS', () => { setPause(false); handlers.onMenu?.(); });
  const steerSeg = mk('p-row seg', '', pMain);
  mk('set-lbl', 'STEERING', steerSeg);
  const segBtns = {
    buttons: pBtn(steerSeg, 'seg-b', '◄ ►', () => setSteerMode('buttons')),
    wheel: pBtn(steerSeg, 'seg-b', 'WHEEL', () => setSteerMode('wheel')),
    tilt: pBtn(steerSeg, 'seg-b', 'TILT', () => setSteerMode('tilt')),
  };
  const centerBtn = pBtn(steerSeg, 'recenter', '⟲ CENTER', () => recenterTilt());
  centerBtn.setAttribute('aria-label', 're-center tilt');
  const row2 = mk('p-row', '', pMain);
  // camera toggle: label shows the live mode (2.5D oblique <-> flat 2D side view)
  state.cam = handlers.cam === '2D' ? '2D' : '2.5D';
  const camBtn = pBtn(row2, 'cam', '', () => setCam(state.cam === '2D' ? '2.5D' : '2D'));
  const setCam = m => { state.cam = m; camBtn.textContent = 'CAM: ' + m; camBtn.classList.toggle('on', m === '2.5D'); handlers.onCam?.(m); };
  setCam(state.cam);
  const setOption = (id, v) => { state[id] = v; els[id].classList.toggle('on', v); };
  for (const o of OPTIONS) { els[o.id] = pBtn(row2, 'opt ' + o.id, o.label, () => setOption(o.id, !state[o.id])); setOption(o.id, false); }
  pBtn(mk('p-row', '', pMain), 'settings', 'SETTINGS ›', () => card.classList.add('show-set'));
  mk('p-title', 'SETTINGS', pSet);
  const sliders = {};
  for (const [key, label, step, fmt] of [['wheel', 'WHEEL SIZE', 0.05], ['gas', 'GAS SIZE', 0.05], ['brake', 'BRAKE SIZE', 0.05],
    ['tiltMax', 'TILT RANGE', 1, v => `±${v}°`]]) {
    const row = mk('set-row', '', pSet), [lo, hi] = UI_RANGE[key];
    mk('set-lbl', label, row);
    const track = mk('set-track', '', row); mk('set-fill', '', track); mk('set-thumb', '', track);
    const val = mk('set-val', '', row);
    sliders[key] = () => { track.style.setProperty('--f', ((ui[key] - lo) / (hi - lo)).toFixed(3)); val.textContent = fmt ? fmt(ui[key]) : Math.round(ui[key] * 100) + '%'; };
    let pid = null;
    const at = e => { const r = track.getBoundingClientRect(), f = Math.min(1, Math.max(0, (e.clientX - r.left) / (r.width || 1))); setUi(key, Math.round((lo + f * (hi - lo)) / step) * step); };
    on(track, 'pointerdown', e => { e.preventDefault(); pid = e.pointerId; try { track.setPointerCapture(pid); } catch (_) { /* synthetic */ } at(e); });
    on(track, 'pointermove', e => { if (e.pointerId === pid) at(e); });
    const up = e => { if (e.pointerId === pid) pid = null; };
    on(track, 'pointerup', up); on(track, 'pointercancel', up);
  }
  const rowS = mk('p-row', '', pSet);
  pBtn(rowS, 'back', '‹ BACK', () => card.classList.remove('show-set'));
  pBtn(rowS, 'defaults', 'DEFAULTS', () => { for (const k in UI_RANGE) if (k !== 'tiltZero') ui[k] = UI_DEFAULTS[k]; applyUi(true); });   // keeps mode + calibration
  // left-cluster box for the canvas debug overlay (renderer dodges it); re-measured on resize / size change
  let dodge = null;
  // the bottom-centre tray stays screen-centred when it fits between the two thumb clusters, else slides / shrinks into the gap
  const measure = () => {
    const l = zones.bl.getBoundingClientRect(), r = zones.br.getBoundingClientRect(), row = thumbRow.getBoundingClientRect();
    dodge = l.width ? { top: l.top, right: l.right } : null;
    const a = row.right + 8, b = r.left - 8, tw = zones.bc.offsetWidth;
    if (!tw || !r.width || b <= a) return;
    const k = Math.min(1, (b - a) / tw), half = tw * k / 2, cx = Math.min(Math.max(innerWidth / 2, a + half), b - half);
    zones.bc.style.left = `${cx.toFixed(1)}px`; zones.bc.style.transform = `translateX(-50%) scale(${k.toFixed(3)})`;
  };
  const applyUi = save => {
    root.style.setProperty('--wheel-s', ui.wheel); root.style.setProperty('--gas-s', ui.gas); root.style.setProperty('--brake-s', ui.brake);
    root.classList.toggle('steer-btns', ui.steer === 'buttons'); root.classList.toggle('steer-tilt', ui.steer === 'tilt');
    for (const k in segBtns) segBtns[k].classList.toggle('on', ui.steer === k);
    centerBtn.classList.toggle('off', ui.steer !== 'tilt');
    for (const k in sliders) sliders[k]();
    if (save) { try { handlers.onUi?.({ ...ui }); } catch (_) { /* storage optional */ } }
    requestAnimationFrame(measure);
  };
  const setUi = (k, v) => { if (ui[k] === v) return; ui[k] = v; applyUi(true); };
  applyUi(false);
  on(window, 'resize', () => requestAnimationFrame(measure));
  function setPause(p) {
    p = !!p;
    if (p === paused) return;
    if (p) releaseAll();
    paused = p;
    card.classList.remove('show-set');
    pauseEl.classList.toggle('show', p); root.classList.toggle('paused', p); menuBtn.classList.toggle('on', p);
    try { handlers.onPause?.(p); } catch (_) { /* glue optional */ }
  }
  resetControls();

  // gauges
  const g = mk('gauges', '', zones.tr);
  const gear = mk('gear', 'L', g), speed = mk('speed', '0', g);
  const rpmBar = mk('bar-fill', '', mk('bar rpm', '', g));
  const winchRow = mk('winch-row', '', g), winchTxt = mk('winch-txt', '', winchRow);
  const winchTrack = mk('bar tension', '', winchRow), winchBar = mk('bar-fill', '', winchTrack);
  const winchStall = mk('bar-mark', '', winchTrack);              // tick at the drum's stall pull (fraction of break load)
  // rig glyph (top-down, front → right; L row on top): wheel dots spin (amber) / lock (white) / air (dim), red ring = shock damage;
  // axle bars glow green → amber → red with shaft stress, dashed red when snapped
  const rig = mk('rig', '', g), rigW = [], rigA = [];
  for (let a = 0; a < 2; a++) rigA.push(mk('rig-axle ' + (a ? 'r' : 'f'), '', rig));
  for (const pos of ['fl', 'fr', 'rl', 'rr']) rigW.push(mk('rig-w ' + pos, '', rig));
  mk('rig-body', '', rig);
  let lastDmg = -1, hitT = null;
  // one-line meters: label · bar · value. FUEL (amber, red < 20 %), DMG = overallDamage() (green -> amber -> red, pulses > 80 %)
  const meter = (cls, label) => {
    const row = mk('meter ' + cls, '', g);
    mk('m-lbl', label, row);
    const fill = mk('bar-fill', '', mk('m-bar', '', row));
    return { row, fill, val: mk('m-val', '', row) };
  };
  const fuelM = meter('fuel', 'FUEL'), dmgM = meter('dmg', 'DMG');
  const status = mk('hc-status', '', g);
  const hintEl = mk('hint', '', root);          // tutorial / stage banner (hint(text) / hint(''))
  const toastEl = mk('toast', '', root);
  let toastTimer = null;
  function toast(text, ms = 2200) {
    toastEl.textContent = text;
    toastEl.classList.add('show');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => toastEl.classList.remove('show'), ms);
  }

  // finish overlay: banner + PLAY AGAIN (tap, or Enter on desktop). Hidden until showFinish().
  const finishEl = mk('finish', '', root);
  const banner = mk('finish-banner', 'FINISH', finishEl);
  const newBest = mk('finish-newbest', 'NEW BEST!', banner, 'span');
  const finishDetail = mk('finish-detail', '', finishEl);
  const finishBest = mk('finish-best', '', finishEl);
  const finishWear = mk('finish-wear', '', finishEl);
  const finishBtns = mk('finish-btns', '', finishEl);
  const again = mk('finish-again', 'PLAY AGAIN', finishBtns);
  again.setAttribute('role', 'button');
  if (handlers.onMenu) {
    const fm = mk('finish-again alt', 'MENU', finishBtns);
    on(fm, 'pointerup', e => { e.preventDefault(); hideFinish(); handlers.onMenu(); });
  }
  let againPid = null;
  const finishShown = () => finishEl.classList.contains('show');
  const hideFinish = () => { againPid = null; again.classList.remove('on'); finishEl.classList.remove('show'); root.classList.remove('finished'); };
  const playAgain = () => {
    if (!finishShown()) return;
    hideFinish();
    (handlers.onPlayAgain || handlers.onRestart)?.();
  };
  on(again, 'pointerdown', e => { e.preventDefault(); againPid = e.pointerId; again.classList.add('on'); });
  on(again, 'pointerup', e => { if (e.pointerId !== againPid) return; playAgain(); });
  on(again, 'pointercancel', () => { againPid = null; again.classList.remove('on'); });

  // KEYBOARD. Driving: W/↑ gas, S/↓/Space brake, A/← D/→ steer (adds to wheel / tilt in every steering mode), E winch, V rev,
  // 1 2 3 4WD / lockers, Q Z 4 L range, K I key, R recover (Shift+R restart), T re-center tilt, Enter play again.
  // Anytime: Esc MENU, G debug, M mute, C cam. MENU open: driving keys dead, ←/→ (A/D) = steering mode, T re-centers, Enter resumes.
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
    if (e.code === 'KeyT') recenterTilt();
    if (paused) {
      if (e.code === 'Enter' || e.code === 'NumpadEnter') setPause(false);
      return;
    }
    for (const t of TOGGLES) if (t.key === e.code) {
      state[t.id] = !state[t.id]; els[t.id].classList.toggle('on', state[t.id]);
      if (DRIVE.includes(t.id)) shiftHook('drive', { id: t.id, on: state[t.id] });
    }
    if (e.code === 'Digit4' || e.code === 'KeyL') cycleRange(e.shiftKey ? -1 : 1);
    if (e.code === 'KeyQ') stepRange(-1);
    if (e.code === 'KeyZ') stepRange(1);
    if (e.code === 'KeyK' || e.code === 'KeyI') turnKey();
    if (e.code === 'KeyR') (e.shiftKey ? handlers.onRestart : handlers.onRecover)?.();
    if (e.code === 'Enter' || e.code === 'NumpadEnter') playAgain();
  });
  // MENU open: ←/→ (A/D) step the STEERING mode (held keys were released by the pause; momentary keys are dead there)
  on(window, 'keydown', e => {
    if (!paused || e.repeat || e.isComposing || e.ctrlKey || e.metaKey || e.altKey || (handlers.active && !handlers.active())) return;
    const d = e.code === 'ArrowLeft' || e.code === 'KeyA' ? -1 : e.code === 'ArrowRight' || e.code === 'KeyD' ? 1 : 0;
    if (d) setSteerMode(STEER_MODES[(STEER_MODES.indexOf(ui.steer) + d + STEER_MODES.length) % STEER_MODES.length]);
  });
  on(window, 'keyup', e => {
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
    /** Switch steering mode ('buttons' | 'wheel' | 'tilt'; call from a user gesture for iOS tilt permission) / re-center tilt. */
    setSteerMode, recenterTilt,
    get dodge() { return dodge; },
    /** Refresh gauges from Vehicle.getState(). */
    update(v, redline = 5000) {
      // pedal fill = level actually applied (index.html ramps throttle), falls back to the HUD target
      drawPedal('gas', v.throttle ?? state.gas); drawPedal('brake', v.brake ?? state.brake);
      gear.textContent = v.gear + (v.fourWD ? '4' : '2');
      // range shift waits for a stop (vehicle.pendingRange): amber sync ring on the knob until it commits; 'commit' hook on engage
      shifter.classList.toggle('sync', !!v.pendingRange);
      if (v.range && v.range !== committed) {
        if (committed !== null) shiftHook('commit', { range: v.range, from: committed });
        committed = v.range;
        const ci = RANGES.indexOf(v.range);
        gateLbl.forEach((l, k) => l.classList.toggle('cur', k === ci));
      }
      speed.textContent = Math.abs(v.speed * 3.6).toFixed(0);
      const r = Math.min(1, v.rpm / redline);
      rpmBar.style.transform = `scaleX(${r.toFixed(3)})`;
      rpmBar.classList.toggle('hot', r > 0.9);
      const w = v.winch;
      winchRow.classList.toggle('show', w.attached || w.broken);
      if (w.attached) {
        const t = Math.min(1, w.tension / w.breakTension);
        winchTxt.textContent = `${w.length.toFixed(1)}m ${(w.tension / 1000).toFixed(1)}kN · LOAD ${Math.round(t * 100)}%`;
        winchBar.style.transform = `scaleX(${t.toFixed(3)})`;
        winchBar.classList.toggle('hot', w.tension > w.maxPull * 0.9);
        winchStall.style.left = `${Math.min(100, (w.maxPull / w.breakTension) * 100).toFixed(1)}%`;
        winchRow.classList.toggle('crit', t > 0.85);
      } else if (w.broken) {
        winchTxt.textContent = 'ROPE SNAPPED';
        winchBar.style.transform = 'scaleX(0)';
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
        const f = Math.max(0, Math.min(1, h.fuel ?? 1)), ft = h.dry ? 'EMPTY' : `${Math.round(f * 100)}%`;
        if (fuelM.val.textContent !== ft) fuelM.val.textContent = ft;
        fuelM.fill.style.transform = `scaleX(${f.toFixed(3)})`;
        fuelM.row.classList.toggle('low', f < 0.2);
        const od = overallDamage(h), dt = `${Math.round(od * 100)}%`;
        if (dmgM.val.textContent !== dt) dmgM.val.textContent = dt;
        dmgM.fill.style.transform = `scaleX(${od.toFixed(3)})`;
        dmgM.row.classList.toggle('warn', od > 0.4 && od <= 0.7);
        dmgM.row.classList.toggle('fail', od > 0.7);
        dmgM.row.classList.toggle('crit', od > 0.8);
        for (let a = 0; a < 2; a++) {
          const x = Math.min(1, h.axleStress[a] || 0), b = !!h.axleBroken[a], el = rigA[a];
          el.classList.toggle('snap', b); el.classList.toggle('crit', !b && x > 0.8);
          el.style.setProperty('--s', b ? 1 : Math.round(x * 20) / 20);
        }
        // fresh body / shock damage: DMG meter + rig glyph flash red (restartable CSS animation)
        const dmg = (h.body || 0) + (h.shock || []).reduce((s2, x) => s2 + x, 0);
        if (lastDmg >= 0 && dmg > lastDmg + 0.004) {
          for (const el of [dmgM.row, rig]) { el.classList.remove('hit'); void el.offsetWidth; el.classList.add('hit'); }
          clearTimeout(hitT); hitT = setTimeout(() => { dmgM.row.classList.remove('hit'); rig.classList.remove('hit'); }, 600);
        }
        lastDmg = dmg;
        const msg = h.dry ? 'OUT OF FUEL · hold RESET' : h.cranking ? 'CRANKING…'
          : h.stalled ? 'STALLED · turn KEY (K) to crank' : h.engineOff ? 'ENGINE OFF · tap KEY (K) to start'
          : h.axleBroken[0] && h.axleBroken[1] ? 'BOTH AXLES SNAPPED · hold RESET'
          : h.axleBroken[0] ? 'FRONT AXLE SNAPPED' : h.axleBroken[1] ? 'REAR AXLE SNAPPED'
          : h.transHot ? 'TRANS HOT · brake, shift LOW' : h.xferHot ? 'XFER HOT · 2WD only' : h.engineHot ? 'ENGINE HOT · limp mode'
          : h.bog > 0.5 ? `BOGGING · TRANS ${Math.round(h.transTemp * 100)}% · shift LOW` : '';
        if (status.textContent !== msg) status.textContent = msg;
        const mode = h.cranking ? 'crank' : v.engineOn === false ? 'off' : 'run';
        if (mode !== keyMode) {
          keyMode = mode;
          keyBtn.classList.toggle('off', mode === 'off'); keyBtn.classList.toggle('crank', mode === 'crank'); keyBtn.classList.toggle('run', mode === 'run');
          keyBtn.classList.toggle('stalled', !!h.stalled);
          keyBtn.title = mode === 'crank' ? 'cranking' : mode === 'off' ? 'start engine (K)' : 'stop engine (K)';
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
    /**
     * Show the FINISH banner with a PLAY AGAIN button; detail = optional text under the banner (e.g. the time).
     * extra = { best?: { time, isNew, prev? } (s), hardcore?: getState().hardcore } -> NEW BEST! badge + wear summary.
     */
    showFinish(detail = '', extra = {}) {
      finishDetail.textContent = detail;
      const b = extra.best;
      newBest.classList.toggle('show', !!(b && b.isNew));
      finishBest.textContent = !b ? '' : b.isNew
        ? (b.prev != null ? `previous best ${b.prev.toFixed(1)} s (−${(b.prev - b.time).toFixed(1)} s)` : 'first clear · best time set')
        : `best ${b.prev.toFixed(1)} s (+${(b.time - b.prev).toFixed(1)} s)`;
      finishWear.textContent = '';
      const h = extra.hardcore;
      if (h) {
        const pc = v => `${Math.round(Math.min(1, Math.max(0, v)) * 100)}%`;
        const cells = [
          ['F-AX', h.axleBroken[0] ? 'SNAP' : pc(h.axleStress[0]), h.axleBroken[0] ? 2 : h.axleStress[0] > 0.6 ? 1 : 0],
          ['R-AX', h.axleBroken[1] ? 'SNAP' : pc(h.axleStress[1]), h.axleBroken[1] ? 2 : h.axleStress[1] > 0.6 ? 1 : 0],
          ['BODY', pc(h.body), h.body > 0.6 ? 2 : h.body > 0.25 ? 1 : 0],
          ['SHOCK', pc(Math.max(...h.shock)), Math.max(...h.shock) > 0.6 ? 2 : Math.max(...h.shock) > 0.25 ? 1 : 0],
          ['ENG', pc(h.engineHeat), h.engineHot ? 2 : h.engineHeat > 0.6 ? 1 : 0],
          ['FUEL', h.dry ? 'EMPTY' : pc(h.fuel ?? 1), h.dry ? 2 : (h.fuel ?? 1) < 0.2 ? 1 : 0],
          ['TRANS', pc(h.transTemp), h.transHot ? 2 : h.transTemp > 0.6 ? 1 : 0],
        ];
        for (const [lbl, val, lvl] of cells) {
          const c = mk('fw' + (lvl === 2 ? ' fail' : lvl === 1 ? ' warn' : ''), '', finishWear, 'span');
          mk('fw-l', lbl, c, 'b'); mk('fw-v', val, c, 'span');
        }
      }
      finishEl.classList.add('show');
      root.classList.add('finished');
    },
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
export function createMenu(root, manifest, initial, onStart) {
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
  mk('menu-title', 'OFFROAD2D', root, 'h1');
  mk('menu-sub', 'HARDCORE · lockers bind on rock · H bogs + cooks the trans on climbs · crashes break parts', root);
  const cols = mk('menu-cols', '', root);
  const bestEls = {};
  const section = (title, list, key, slots) => {
    const col = mk('menu-col', '', cols);
    mk('menu-h', title, col);
    const row = mk('menu-cards', '', col);
    const cards = [];
    for (const it of list) {
      const c = mk('card', '', row);
      c.setAttribute('role', 'button');
      mk('card-name', it.name || it.id, c);
      if (it.difficulty) mk('card-tag ' + it.difficulty.toLowerCase(), it.difficulty, c);
      if (key === 'level') bestEls[it.id] = mk('card-best', '', c);
      if (it.blurb) mk('card-blurb', it.blurb, c);
      on(c, 'pointerup', e => { e.preventDefault(); selection[key] = it.id; sync(); });
      cards.push([c, it.id]);
    }
    for (let i = list.length; i < slots; i++) { const c = mk('card empty', '', row); mk('card-name', 'EMPTY SLOT', c); mk('card-blurb', 'future truck', c); }
    return () => cards.forEach(([c, id]) => c.classList.toggle('sel', selection[key] === id));
  };
  const syncL = section('LEVEL', levels, 'level', 0);
  const syncV = section('VEHICLE', vehicles, 'vehicle', manifest.vehicleSlots || 0);
  const sync = () => { syncL(); syncV(); };
  sync();
  mk('menu-help', 'KEY (K) = crank / shut off · GAS / BRAKE · WHEEL / ◄ ► / phone TILT (MENU › STEERING, ⟲ CENTER or T re-centers tilt) steers across the trail (2.5D) · REV · WINCH hold = hook + reel, tap = release · 4WD F-LOCK R-LOCK · XFER lever: drag / tap H ↕ L+ ↕ L ↕ L- (Q / Z, 4), commits when stopped · FUEL burns with rpm² × gas + wheelspin · RESET tap = flip upright (damage stays), hold = restart & repair · MENU (Esc) = pause, CAM, DEBUG, MUTE, control sizes', root, 'p');
  const status = mk('menu-status', '', root, 'p');
  const go = mk('go', 'START', root);
  go.setAttribute('role', 'button');
  on(go, 'pointerup', e => { e.preventDefault(); e.stopPropagation(); onStart({ ...selection }); });
  const key = e => {
    if (root.classList.contains('hidden') || e.repeat || e.ctrlKey || e.metaKey || e.altKey || !(e.code === 'Enter' || e.code === 'Space')) return;
    e.preventDefault(); onStart({ ...selection });
  };
  on(window, 'keydown', key);
  return {
    selection,
    show() { root.classList.remove('hidden'); },
    hide() { root.classList.add('hidden'); },
    setStatus(t) { status.textContent = t; },
    /** bests = { levelId: { time (s), vehicle? (display name) } } -> "BEST 63.2 s · Rock Buggy" on each level card. */
    setBests(bests = {}) {
      for (const id in bestEls) {
        const b = bests[id];
        bestEls[id].textContent = b && isFinite(b.time) ? `BEST ${b.time.toFixed(1)} s${b.vehicle ? ' · ' + b.vehicle : ''}` : '';
      }
    },
    destroy() { offs.forEach(f => f()); root.textContent = ''; },
  };
}
