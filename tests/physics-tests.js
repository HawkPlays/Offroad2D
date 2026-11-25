// tests/physics-tests.js — headless physics test-suite (no rendering).
//   Node   : node tests/physics-tests.js            (exit code 1 on any FAIL)
//   Browser: open tests/index.html                  (results printed into the page + console)
const isNode = typeof process !== 'undefined' && !!(process.versions && process.versions.node);
if (isNode) {
  const { register } = await import('node:module');
  register('./https-loader.mjs', import.meta.url);   // lets Node import https://esm.sh modules
}
const { PhysicsWorld, FIXED_DT, MAX_FRAME_DT, RUT, TRAIL, TYRE } = await import('../engine/physics.js');
const { Vehicle, DEFAULT_HARDCORE, SHIFT_SPEED } = await import('../engine/vehicle.js');

async function loadJSON(rel) {
  const url = new URL(rel, import.meta.url);
  if (isNode) {
    const fs = await import('node:fs/promises');
    return JSON.parse(await fs.readFile(url, 'utf8'));
  }
  return (await fetch(url)).json();
}

// ---- tiny harness ------------------------------------------------------------------------------
const results = [];
const out = line => {
  console.log(line);
  if (!isNode) { const el = document.getElementById('out'); if (el) el.textContent += line + '\n'; }
};
const n2 = v => (Number.isFinite(v) ? v.toFixed(2) : String(v));
const n3 = v => (Number.isFinite(v) ? v.toFixed(3) : String(v));

/** A test body calls check(label, condition, detail) any number of times. */
const ONLY = isNode && process.env.O2D_ONLY ? new RegExp(process.env.O2D_ONLY, 'i') : null;   // dev: O2D_ONLY=<regex> runs matching tests only
const LV_ONLY = isNode && process.env.O2D_LEVELS ? new RegExp(process.env.O2D_LEVELS, 'i') : null;   // dev: content test on matching levels only (run levels in parallel processes)
function test(name, fn) {
  if (ONLY && !ONLY.test(name)) return;
  const checks = [];
  const check = (label, ok, detail = '') => checks.push({ label, ok: !!ok, detail });
  let error = null;
  try { fn(check); } catch (e) { error = e; }
  const ok = !error && checks.length > 0 && checks.every(c => c.ok);
  results.push({ name, ok });
  out(`${ok ? 'PASS' : 'FAIL'}  ${name}`);
  for (const c of checks) out(`      ${c.ok ? 'ok  ' : 'FAIL'} ${c.label}${c.detail ? '  [' + c.detail + ']' : ''}`);
  if (error) out(`      EXCEPTION ${error && error.stack ? error.stack : error}`);
}

// ---- scenario helpers --------------------------------------------------------------------------
const CFG = await loadJSON('../vehicles/crawler01.json');
const clone = o => JSON.parse(JSON.stringify(o));
const SLOPE_X0 = 10;

const flat = (surface = 'rock', extra = {}) => ({ ground: [{ surface, points: [[-200, 0], [400, 0]] }], ...extra });
/** Flat ground up to x = SLOPE_X0, then a constant incline. */
function slope(deg, surface = 'rock', extra = {}) {
  const t = Math.tan(deg * Math.PI / 180);
  return { ground: [{ surface, points: [[-200, 0], [SLOPE_X0, 0], [SLOPE_X0 + 300, 300 * t]] }], ...extra };
}
const onSlope = (deg, x) => ({ x, y: (x - SLOPE_X0) * Math.tan(deg * Math.PI / 180), angle: deg * Math.PI / 180 });

function make(level, spawn = { x: 0, y: 0 }, cfg = CFG) {
  const phys = new PhysicsWorld(level);
  const car = new Vehicle(phys, cfg, spawn);
  car.startEngine();                                   // spawns are engine-off (key ignition); scenarios start running
  return { phys, car };
}
/** Step `seconds` of fixed steps; each(stepIndex) is called after every step (return false to stop early). */
function run(phys, seconds, each) {
  const n = Math.round(seconds / FIXED_DT);
  for (let i = 0; i < n; i++) { phys.stepFixed(); if (each && each(i) === false) return; }
}
const allFinite = s => [s.x, s.y, s.angle, s.vx, s.vy, s.rpm, ...s.wheels.flatMap(w => [w.x, w.y, w.omega])].every(Number.isFinite);
const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
const mean = a => a.reduce((x, y) => x + y, 0) / a.length;

// ---- 1. suspension compression / rebound -------------------------------------------------------
test('suspension: static sag, compression, damped rebound', check => {
  const { phys, car } = make(flat());
  run(phys, 3);
  const s0 = car.getState();
  const g = 9.81, c = CFG.chassis, xf = CFG.axles[0].mount[0], xr = CFG.axles[1].mount[0];
  const loads = [c.mass * g * (c.com[0] - xr) / (xf - xr) / 2, c.mass * g * (xf - c.com[0]) / (xf - xr) / 2];
  const stat = s0.wheels.map(w => w.travel);
  s0.wheels.forEach((w, i) => {
    const sus = CFG.axles[w.axle].suspension;
    const expect = loads[w.axle] / sus.stiffness - (sus.preload || 0);
    check(`${w.name} static sag matches k (spring law)`, Math.abs(w.travel - expect) < 0.012,
      `travel ${n3(w.travel)} m, expected ${n3(expect)} m, ${n2(w.compression * 100)}% of travel`);
  });
  check('wheel loads sum to vehicle weight', Math.abs(s0.wheels.reduce((a, w) => a + w.load, 0) - (c.mass + 4 * CFG.wheels.mass) * g) < 300,
    `sum ${s0.wheels.reduce((a, w) => a + w.load, 0).toFixed(0)} N`);
  // shove the body down: suspension must compress, then rebound and settle back
  car.chassis.setLinearVelocity({ x: 0, y: -1.6 });
  let maxC = [0, 0, 0, 0], minAfter = [9, 9, 9, 9], peaked = false;
  run(phys, 3, i => {
    const s = car.getState();
    s.wheels.forEach((w, k) => {
      if (w.travel > maxC[k]) maxC[k] = w.travel;
      if (i > 30 && w.travel < minAfter[k]) minAfter[k] = w.travel;
    });
  });
  const s1 = car.getState();
  s1.wheels.forEach((w, k) => {
    check(`${w.name} compresses under load`, maxC[k] > stat[k] + 0.05, `peak ${n3(maxC[k])} m vs static ${n3(stat[k])} m`);
    check(`${w.name} rebound is damped (overshoot < 6 cm)`, minAfter[k] > stat[k] - 0.06, `min after peak ${n3(minAfter[k])} m`);
    check(`${w.name} returns to static height`, Math.abs(w.travel - stat[k]) < 0.006, `final ${n3(w.travel)} m`);
  });
});

// ---- 2. landing stability ----------------------------------------------------------------------
test('landing: 1.6 m drop settles upright, bump stops hold', check => {
  const { phys, car } = make(flat(), { x: 0, y: 1.6 });
  let maxComp = 0, minY = 9, maxAng = 0, finite = true, maxLoad = 0;
  run(phys, 5, () => {
    const s = car.getState();
    finite = finite && allFinite(s);
    for (const w of s.wheels) { maxComp = Math.max(maxComp, w.compression); maxLoad = Math.max(maxLoad, w.load); }
    minY = Math.min(minY, s.y); maxAng = Math.max(maxAng, Math.abs(s.angle));
  });
  const s = car.getState();
  check('state stays finite', finite);
  check('suspension uses most of its travel on impact', maxComp > 0.75, `peak ${n2(maxComp * 100)}%`);
  check('bump stops limit travel', maxComp < 1.06, `peak ${n2(maxComp * 100)}% of travel`);
  check('pitch stays controlled', maxAng < 0.3, `max |pitch| ${n3(maxAng)} rad`);
  check('comes to rest upright', Math.abs(s.angle) < 0.03 && Math.hypot(s.vx, s.vy) < 0.05,
    `pitch ${n3(s.angle)} rad, speed ${n3(Math.hypot(s.vx, s.vy))} m/s`);
  check('all four wheels grounded at rest', s.wheels.every(w => w.grounded));
  check('chassis never sank into the ground', minY > 0.35, `min chassis y ${n3(minY)} m`);
});

// ---- 3. tyre grip vs mud wheelspin -------------------------------------------------------------
test('traction: rock grips, mud spins', check => {
  const drive = surface => {
    const { phys, car } = make(flat(surface));
    run(phys, 1.5);
    const x0 = car.getState().x;
    Object.assign(car.controls, { throttle: 1, fourWD: true, range: 'L+' });
    const slips = [];
    run(phys, 1, () => slips.push(mean(car.getState().wheels.map(w => w.slip))));   // 1 s: Low tops out at ~14 km/h
    const s = car.getState();
    return { dist: s.x - x0, slip: mean(slips), speed: s.speed, rpm: s.rpm, bite: Math.min(...s.wheels.map(w => w.bite)) };
  };
  const rock = drive('rock'), mud = drive('mud');
  // TYRE: a gripping tyre keeps full bite, a spinning one loses a little (feathering the gas pays; mud loses least)
  const ice = drive('ice');
  check('bite: gripping rock = full, spinning ice < full, mud loss mild', rock.bite > 0.995 && ice.bite < 0.9 + TYRE.spin && ice.bite > 0.8 && mud.bite >= 0.95,
    `rock ${n3(rock.bite)} ice ${n3(ice.bite)} mud ${n3(mud.bite)}`);
  check('rock: launches hard', rock.dist > 2.5, `${n2(rock.dist)} m in 1 s, v=${n2(rock.speed)} m/s`);
  check('rock: little wheelspin', rock.slip < 0.4, `mean slip ${n2(rock.slip)} m/s`);
  check('mud: wheels spin', mud.slip > 1.2, `mean slip ${n2(mud.slip)} m/s, rpm ${mud.rpm.toFixed(0)}`);
  check('mud: much slower progress', mud.dist < 0.6 * rock.dist && mud.dist > 0.2, `${n2(mud.dist)} m vs rock ${n2(rock.dist)} m`);
});

// ---- 4. 2WD vs 4WD -----------------------------------------------------------------------------
test('transfer case: 4WD climbs a slippery 20° slope, 2WD (rear) cannot', check => {
  const climb = fourWD => {
    const { phys, car } = make(slope(20, 'slick', { surfaces: { slick: { mu: 0.5, rr: 0.01 } } }), onSlope(20, 30));
    car.controls.brake = 1;
    run(phys, 1.5);
    const x0 = car.getState().x;
    Object.assign(car.controls, { brake: 0, throttle: 1, fourWD, range: 'L+' });
    const front = [], rear = [], fslip = [], rslip = [];
    run(phys, 4, () => {
      const w = car.getState().wheels;
      front.push(w[0].driveTorque + w[1].driveTorque); rear.push(w[2].driveTorque + w[3].driveTorque);
      fslip.push((w[0].slip + w[1].slip) / 2); rslip.push((w[2].slip + w[3].slip) / 2);
    });
    return { dx: car.getState().x - x0, front: mean(front), rear: mean(rear), fslip: mean(fslip), rslip: mean(rslip), hold: x0 };
  };
  const two = climb(false), four = climb(true);
  check('brakes hold the vehicle on the slope first', Math.abs(two.hold - four.hold) < 0.05 && two.hold > 29, `x=${n2(two.hold)}`);
  check('4WD: climbs', four.dx > 3, `dx ${n2(four.dx)} m`);
  check('4WD: front axle receives drive torque', four.front > 500, `front ${four.front.toFixed(0)} N·m, rear ${four.rear.toFixed(0)} N·m`);
  check('2WD: no drive torque on front axle', Math.abs(two.front) < 5, `front ${two.front.toFixed(0)} N·m, rear ${two.rear.toFixed(0)} N·m`);
  check('2WD: rear wheels spin, vehicle does not climb', two.rslip > 1 && two.dx < 0.5, `rear slip ${n2(two.rslip)} m/s, dx ${n2(two.dx)} m`);
  check('4WD beats 2WD', four.dx > two.dx + 3, `4WD ${n2(four.dx)} m vs 2WD ${n2(two.dx)} m`);
});

// ---- 5. open vs locked differentials -----------------------------------------------------------
test('differentials: open diff stalls on split-mu, lockers transfer torque to the gripping side', check => {
  // Left wheels on ice, right wheels on rock, 10° climb.
  const climb = (lockF, lockR) => {
    const { phys, car } = make(slope(10, 'rock', { zones: [{ surface: 'ice', side: 'L' }] }), onSlope(10, 30));
    car.controls.brake = 1;
    run(phys, 1.5);
    const x0 = car.getState().x;
    Object.assign(car.controls, { brake: 0, throttle: 1, fourWD: true, range: 'L+', lockF, lockR });
    const tL = [], tR = [], dW = [], sL = [], sR = [];
    run(phys, 4, () => {
      const w = car.getState().wheels;
      tL.push(w[0].driveTorque + w[2].driveTorque); tR.push(w[1].driveTorque + w[3].driveTorque);
      dW.push(Math.abs(w[2].omega - w[3].omega)); sL.push(w[2].slip); sR.push(w[3].slip);
    });
    const s = car.getState();
    return { dx: s.x - x0, tL: mean(tL), tR: mean(tR), dW: mean(dW), sL: mean(sL), sR: mean(sR), surfL: s.wheels[2].surface, surfR: s.wheels[3].surface };
  };
  const open = climb(false, false), locked = climb(true, true), rearOnly = climb(false, true);
  check('split-mu zone applies per side', open.surfL === 'ice' && open.surfR === 'rock', `L on ${open.surfL}, R on ${open.surfR}`);
  check('open: both sides get (nearly) equal torque', Math.abs(open.tL - open.tR) < 0.12 * Math.max(Math.abs(open.tL), Math.abs(open.tR), 1),
    `L ${open.tL.toFixed(0)} N·m, R ${open.tR.toFixed(0)} N·m`);
  check('open: ice-side wheel spins away the drive', open.sL > 2 && open.dW > 3, `L slip ${n2(open.sL)} m/s, |wL-wR| ${n2(open.dW)} rad/s`);
  check('open: vehicle fails to climb', open.dx < 0.5, `dx ${n2(open.dx)} m`);
  check('locked: wheel speeds equal across the axle', locked.dW < 0.05, `|wL-wR| ${n3(locked.dW)} rad/s`);
  check('locked: torque goes to the gripping side', locked.tR > 4 * Math.max(locked.tL, 1), `L ${locked.tL.toFixed(0)} N·m, R ${locked.tR.toFixed(0)} N·m`);
  check('locked: vehicle climbs', locked.dx > 4, `dx ${n2(locked.dx)} m`);
  check('R-Lock alone also climbs (rear axle delivers)', rearOnly.dx > 2 && rearOnly.dx <= locked.dx + 0.3, `dx ${n2(rearOnly.dx)} m`);
});

// ---- 6. winch ----------------------------------------------------------------------------------
test('winch: range limit, pull, tension limit, breakage', check => {
  const deg = 28, lvl = () => slope(deg, 'slick', { surfaces: { slick: { mu: 0.35, rr: 0.02 } } });
  const anchorAt = x => ({ x, y: (x - SLOPE_X0) * Math.tan(deg * Math.PI / 180) + 0.6 });
  // (a) cannot climb without the winch
  {
    const { phys, car } = make(lvl(), onSlope(deg, 30));
    Object.assign(car.controls, { throttle: 1, fourWD: true, lockF: true, lockR: true, range: 'L+' });
    run(phys, 4);
    check('control: slope is unclimbable under engine power', car.getState().x < 30.3, `x ${n2(car.getState().x)} (start 30)`);
  }
  // (b) range limit + pull
  {
    const { phys, car } = make(lvl(), onSlope(deg, 30));
    check('anchor beyond max rope length is refused', car.winch.attach(anchorAt(60)) === false && !car.winch.attached,
      `maxLength ${car.winch.maxLength} m`);
    const ok = car.winch.attach(anchorAt(43));
    check('anchor in range attaches', ok && car.winch.attached, `rope ${n2(car.winch.length)} m`);
    const len0 = car.winch.length;
    run(phys, 2);                                   // rope must hold the vehicle on the slope
    const hold = { x: car.getState().x, speed: car.getState().speed, tension: car.winch.tension };
    const m = car.winch.mountWorld(), a = car.winch.anchorWorld();
    check('rope holds the vehicle (no reeling)', Math.abs(hold.speed) < 0.05 && Math.hypot(a.x - m.x, a.y - m.y) < len0 + 0.15,
      `speed ${n3(hold.speed)} m/s, span ${n2(Math.hypot(a.x - m.x, a.y - m.y))} m / rope ${n2(len0)} m, tension ${(hold.tension / 1000).toFixed(1)} kN`);
    car.controls.winch = 1;
    let maxT = 0;
    run(phys, 8, () => { maxT = Math.max(maxT, car.winch.tension); });
    const s = car.getState();
    check('reeling in drags the vehicle up the slope', s.x - hold.x > 2.5, `dx ${n2(s.x - hold.x)} m in 8 s`);
    check('rope shortened', car.winch.length < len0 - 2.5, `${n2(len0)} m -> ${n2(car.winch.length)} m`);
    check('tension stays under the motor stall limit, rope intact', maxT < car.winch.maxPull && maxT > 5000 && !car.winch.broken,
      `peak ${(maxT / 1000).toFixed(1)} kN (stall ${car.winch.maxPull / 1000} kN, break ${car.winch.breakTension / 1000} kN)`);
    car.controls.winch = 0; run(phys, 0.2);
    car.controls.winch = 1; run(phys, 0.1); car.controls.winch = 0; run(phys, 0.1);
    check('short tap releases the rope', !car.winch.attached);
  }
  // (c) shock load snaps the rope
  {
    const { phys, car } = make(flat('rock', { anchors: [{ x: 9, y: 0.8 }] }));
    run(phys, 1);
    car.controls.winch = 1; run(phys, 0.05); car.controls.winch = 0;
    check('button attaches to nearest level anchor', car.winch.attached, `rope ${n2(car.winch.length)} m`);
    car.winch.length += 2.0;                          // slack
    Object.assign(car.controls, { reverse: true, throttle: 1, fourWD: true, range: 'L+' }); // back away hard
    let peak = 0;
    run(phys, 3, () => { peak = Math.max(peak, car.winch.tension); });
    const ev = phys.events.find(e => e.type === 'winch-break');
    check('rope breaks when tension exceeds the limit', car.winch.broken && !car.winch.attached && !!ev,
      ev ? `snapped at ${(ev.tension / 1000).toFixed(1)} kN (limit ${car.winch.breakTension / 1000} kN)` : `peak ${(peak / 1000).toFixed(1)} kN`);
    const x1 = car.getState().x;
    run(phys, 1);
    check('vehicle is free after the break', car.getState().x < x1 - 1, `x ${n2(x1)} -> ${n2(car.getState().x)}`);
  }
});

// ---- 6b. crashes stall the engine; ramps / landings never do; reverse gear is slow ------------------------------
test('crash stall + reverse: head-on into a wall stalls, a ramp at speed does not; reverse tops out slow', check => {
  const drive = (pts, secs) => {
    const { phys, car } = make({ ground: [{ surface: 'rock', points: pts }] }, { x: 0, y: 0 });
    Object.assign(car.controls, { throttle: 1, fourWD: true, range: 'H' });
    let stall = null, vmax = 0;
    run(phys, secs, () => { vmax = Math.max(vmax, car.getState().speed); for (const e of phys.events) if (e.type === 'stall') stall = stall || e; phys.events.length = 0; });
    return { stall, vmax, s: car.getState() };
  };
  const wall = drive([[-50, 0], [14, 0], [14, 1.6], [40, 1.6]], 6);
  check('full gas into a 1.6 m wall: crash stalls the engine (cause impact)', wall.stall && wall.stall.cause === 'impact' && !wall.s.engineOn,
    `stall ${wall.stall ? wall.stall.cause : 'none'}, hit at up to ${n2(wall.vmax * 3.6)} km/h`);
  const ramp = drive([[-50, 0], [14, 0], [22, 2.9], [60, 2.9]], 7);
  check('full gas up a 20° ramp: no stall', !ramp.stall && ramp.s.engineOn && ramp.s.x > 22, `x ${n2(ramp.s.x)} m, ${ramp.stall ? 'stalled ' + ramp.stall.cause : 'running'}`);
  const { phys, car } = make(flat(), { x: 0, y: 0 });
  Object.assign(car.controls, { throttle: 1, fourWD: true, range: 'H', reverse: true });
  run(phys, 8);
  const v = Math.abs(car.getState().speed) * 3.6;
  check('reverse in H tops out <= 12 km/h', v > 6 && v <= 12, `${n2(v)} km/h`);
});

// ---- 6c. H-gate shifter (controls.gate: H L+ L L- N R) --------------------------------------------
test('H-gate: N rolls free + free-revs, R / range slots wait for a stop, N is instant', check => {
  const { phys, car } = make(flat(), { x: 0, y: 0 });
  const c = car.controls, st = () => car.getState();
  Object.assign(c, { gate: 'H', fourWD: true, throttle: 1 });
  run(phys, 3);
  const v0 = st().speed;
  c.gate = 'N'; run(phys, 1 / 30);
  check('into N while moving: immediate', st().gear === 'N' && !st().pendingRange, `gear ${st().gear}, pending ${st().pendingRange}`);
  run(phys, 1.5);
  const sN = st();
  check('N: no drive (coasts, no engine braking) and the engine free-revs', sN.speed > 0.6 * v0 && sN.speed < v0 + 0.05 && sN.rpm > 0.6 * CFG.engine.redlineRpm,
    `v ${n2(v0)} -> ${n2(sN.speed)} m/s, rpm ${n2(sN.rpm)}`);
  c.gate = 'R'; run(phys, 1 / 30);
  check('R requested while rolling forward: waits (pendingRange = R)', st().pendingRange === 'R' && st().gear === 'N', `gear ${st().gear}, pending ${st().pendingRange}`);
  c.throttle = 0; c.brake = 1; run(phys, 4);
  check('stopped: R engages (L+ gearing)', st().gear === 'R' && st().range === 'L+' && !st().pendingRange, `gear ${st().gear}, range ${st().range}`);
  c.brake = 0; c.throttle = 1; run(phys, 4);
  const vr = st().speed * 3.6;
  check('R drives backward, slow', vr < -3 && vr > -11, `${n2(vr)} km/h`);
  c.gate = 'H'; run(phys, 1 / 30);
  check('H while reversing: waits for the stop', st().pendingRange === 'H' && st().gear === 'R', `gear ${st().gear}, pending ${st().pendingRange}`);
});

// ---- 6d. RESET-tap recovery on an obstacle --------------------------------------------------------
const L03 = await loadJSON('../levels/level03.json'), BOGGER = await loadJSON('../vehicles/crawler02.json');
test('recover on a boulder: backs off to clear ground, no damage / stall, repeatable', check => {
  const phys = new PhysicsWorld(L03), car = new Vehicle(phys, BOGGER, { x: 2, y: phys.groundAt(2) + 0.4, z: 1.2 });
  car.startEngine(); Object.assign(car.controls, { fourWD: true, range: 'L' });
  run(phys, 1.5); car.controls.throttle = 1; run(phys, 8);     // crawls up onto the L03 partial boulder (x 9.5) and hangs on it
  const x0 = car.chassis.getPosition().x, h0 = car.getState().hardcore.body;
  car.controls.throttle = 0;
  let worst = 0, stalls = 0, xs = [];
  for (let k = 0; k < 3; k++) {
    car.recover(0.9, 0); xs.push(car.chassis.getPosition().x);
    run(phys, 2, () => { for (const e of phys.events.splice(0)) if (e.type === 'stall') stalls++; });
    worst = Math.max(worst, car.getState().hardcore.body - h0);
  }
  check('hung on the boulder before the RESET tap', x0 > 8.5 && x0 < 10.5, `x ${n2(x0)}`);
  check('recovers behind the boulder (footprint clear)', xs.every(x => x < 9.5 - 0.84 - 2.4), xs.map(n2).join(' '));
  check('no damage, no stall from 3 RESET taps', worst < 0.005 && stalls === 0 && car.engineRunning(), `body +${n3(worst)}, stalls ${stalls}`);
  const p = new PhysicsWorld(flat()), c2 = new Vehicle(p, CFG, { x: 0, y: 0 }); run(p, 1.5); c2.recover(0.9, 0);
  check('open ground: recovers in place', Math.abs(c2.chassis.getPosition().x) < 0.1, n2(c2.chassis.getPosition().x));
});

// ---- 7. chassis rollover -----------------------------------------------------------------------
test('rollover: roof/chassis collision, stuck on roof, recover', check => {
  // (a) dropped upside-down: must rest on its roof with wheels in the air
  {
    const NOCRASH = { ...clone(CFG), hardcore: { ...(CFG.hardcore || {}), crashBody: Infinity, crashWheel: Infinity } };   // fixture: roof physics, not crash stalls
    const { phys, car } = make(flat(), { x: 0, y: 3.2, angle: Math.PI }, NOCRASH);
    let finite = true;
    run(phys, 4, () => { finite = finite && allFinite(car.getState()); });
    const s = car.getState(), x0 = s.x;
    check('state stays finite', finite);
    check('rests upside-down on the roof', s.upsideDown && Math.abs(Math.abs(wrap(s.angle)) - Math.PI) < 0.35 && s.chassisContacts.length > 0,
      `pitch ${n2(wrap(s.angle))} rad, ${s.chassisContacts.length} roof contacts, chassis y ${n2(s.y)} m`);
    check('roof holds the body off the ground', s.y > 0.8 && s.wheels.every(w => !w.grounded), `chassis y ${n2(s.y)} m`);
    Object.assign(car.controls, { throttle: 1, fourWD: true });
    run(phys, 2);
    const s2 = car.getState();
    check('stuck: throttle only spins the wheels', Math.abs(s2.x - x0) < 0.6 && Math.abs(s2.wheels[2].omega) > 5,
      `dx ${n2(s2.x - x0)} m, wheel w ${n2(s2.wheels[2].omega)} rad/s`);
    car.controls.throttle = 0;
    car.recover();
    run(phys, 3);
    const s3 = car.getState();
    check('recover() puts it back on its wheels', !s3.upsideDown && Math.abs(s3.angle) < 0.05 && s3.wheels.every(w => w.grounded),
      `pitch ${n3(s3.angle)} rad`);
  }
  // (b) full throttle into a quarter-pipe: goes past vertical, falls over backwards onto its roof
  {
    const R = 6, pts = [[-100, 0], [20, 0]];
    for (let k = 1; k <= 12; k++) { const a = k * Math.PI / 24; pts.push([20 + R * Math.sin(a), R - R * Math.cos(a)]); }
    pts.push([20 + R, R + 6]);
    const cfg = clone(CFG); cfg.drivetrain.highTorque = 0.66;   // fixture: softer 4H pull, so the flip ends on the roof (not a full loop)
    cfg.hardcore = { ...(cfg.hardcore || {}), crashBody: Infinity, crashWheel: Infinity };   // fixture: the flip itself, not the crash stall
    const { phys, car } = make({ ground: [{ surface: 'rock', points: pts }] }, { x: 0, y: 0 }, cfg);
    Object.assign(car.controls, { throttle: 1, fourWD: true, range: 'H' });   // no lockers: they would snap at this speed
    let maxPitch = 0, finite = true;
    run(phys, 7, () => { const s = car.getState(); finite = finite && allFinite(s); maxPitch = Math.max(maxPitch, s.angle); });
    car.controls.throttle = 0;
    run(phys, 8);
    const s = car.getState();
    check('quarter-pipe climb pitches past vertical (backflip)', maxPitch > 2.2 && finite, `max pitch ${n2(maxPitch)} rad (${(maxPitch * 57.3).toFixed(0)}°)`);
    check('lands and stays on its roof, wheels in the air', s.upsideDown && s.wheels.every(w => !w.grounded) && Math.hypot(s.vx, s.vy) < 0.1,
      `pitch ${n2(wrap(s.angle))} rad, speed ${n3(Math.hypot(s.vx, s.vy))} m/s, grounded wheels ${s.wheels.filter(w => w.grounded).length}, ${s.chassisContacts.length} body contacts`);
  }
});

// ---- 8. fixed-timestep determinism -------------------------------------------------------------
test('determinism: identical runs, frame-rate independence, dt clamp', check => {
  const course = () => ({
    ground: [
      { surface: 'dirt', points: [[-20, 0], [6, 0], [7, 0.35], [10, 0.35], [10.6, 0.8], [14, 0.8]] },
      { surface: 'mud', points: [[14, 0.8], [18, 0.2], [24, 0.2], [30, 2.2]] },
      { surface: 'rock', points: [[30, 2.2], [34, 2.2], [35, -1.5], [40, -1.5], [41, 2.2], [80, 2.2]] },
    ],
    zones: [{ x0: 14, x1: 30, surface: 'ice', side: 'L' }],
    logs: [{ x: 4, y: 0.35, r: 0.3 }, { x: 12, y: 1.2, r: 0.35 }],
    bridges: [{ from: [34, 2.2], to: [41, 2.2], planks: 9 }],
    anchors: [{ x: 33, y: 3.2 }],
  });
  const script = (car, i) => {
    const c = car.controls;
    c.throttle = (i % 200) < 150 ? 1 : 0;
    c.brake = (i % 400) > 370 ? 1 : 0;
    c.fourWD = i > 60; c.lockR = i > 240; c.lockF = i > 420; c.range = i >= 130 && i < 700 ? 'L+' : 'H';   // High launch: clears the logs
    c.winch = i > 500 && i < 620 ? 1 : 0;
  };
  const fixedRun = () => {
    const { phys, car } = make(course(), { x: 0, y: 0 });
    const trace = [];
    for (let i = 0; i < 900; i++) { script(car, i); phys.stepFixed(); if (i % 150 === 149) trace.push(phys.hash()); }
    return { trace, x: car.getState().x, finite: allFinite(car.getState()) };
  };
  const a = fixedRun(), b = fixedRun();
  check('two identical runs produce bit-identical state', a.trace.join() === b.trace.join(), `hashes ${a.trace.join(' ')}`);
  check('the run actually exercised the course', a.finite && a.x > 15, `final x ${n2(a.x)} m`);
  // variable frame times must land on the same states (inputs keyed to the step counter)
  const frameRun = dts => {
    const { phys, car } = make(course(), { x: 0, y: 0 });
    const pre = () => script(car, phys.stepCount);
    phys.preStep.unshift(pre);
    let k = 0;
    while (phys.stepCount < 900) phys.step(dts[k++ % dts.length]);
    return { hash: phys.hash(), steps: phys.stepCount };
  };
  const f30 = frameRun([1 / 30]), f144 = frameRun([1 / 144]), jit = frameRun([0.004, 0.021, 0.009, 0.033, 0.016]);
  check('30 fps, 144 fps and jittery frames give the same state as fixed stepping',
    f30.hash === a.trace[5] && f144.hash === a.trace[5] && jit.hash === a.trace[5],
    `fixed ${a.trace[5]} 30fps ${f30.hash} 144fps ${f144.hash} jitter ${jit.hash}`);
  const { phys } = make(flat());
  const n = phys.step(5.0);
  check('a huge frame time is clamped', n === Math.round(MAX_FRAME_DT / FIXED_DT), `step(5 s) ran ${n} fixed steps (max ${Math.round(MAX_FRAME_DT / FIXED_DT)})`);
  check('sub-step frame runs zero steps and accumulates', phys.step(0.004) === 0 && phys.accumulator > 0, `accumulator ${n3(phys.accumulator)} s`);
});

// ---- 9. hardcore: drivetrain bind / axle stress / transfer-case heat ----------------------------
test('hardcore bind: lockers on rock at full throttle snap a shaft; feathering, mud and open diffs do not', check => {
  const ev = (phys, type) => phys.events.filter(e => e.type === type);
  // (a) 2WD + R-Lock on rock, full throttle in Low: rear shaft stress builds, then snaps -> no drive at all
  {
    const { phys, car } = make(flat('rock'));
    run(phys, 0.5);
    Object.assign(car.controls, { throttle: 1, lockR: true, range: 'L+' });
    let snapT = -1, rising = true, prev = 0;
    run(phys, 20, i => {
      const a = car.getState().hardcore.axleStress[1];
      if (snapT < 0 && a < prev - 1e-9) rising = false;
      prev = a;
      if (snapT < 0 && car.getState().hardcore.axleBroken[1]) snapT = i / 60;
      return snapT < 0;
    });
    const snaps = ev(phys, 'axle-snap');
    check('rear shaft stress rises monotonically while binding', rising);
    check('rear axle shaft snaps under sustained full-throttle bind', snapT > 2 && snapT < 20 && snaps.length === 1 && snaps[0].axle === 'rear',
      `snapped after ${n2(snapT)} s (${snaps.map(e => e.axle).join(',')})`);
    const t = [];
    run(phys, 1, () => { const w = car.getState().wheels; t.push(Math.abs(w[2].driveTorque) + Math.abs(w[3].driveTorque)); });
    const s = car.getState();
    check('snapped rear axle (2WD) delivers no drive, engine free-revs', mean(t) < 5 && s.rpm > 0.8 * CFG.engine.redlineRpm,
      `rear |T| ${mean(t).toFixed(1)} N·m, rpm ${s.rpm.toFixed(0)}`);
    car.reset();
    const r = car.getState().hardcore;
    check('reset() repairs the shaft', !r.axleBroken[1] && r.axleStress[1] === 0);
  }
  // (b) controls: same 12 s without lockers / on mud / feathered throttle -> no shaft stress damage
  const stress = (surface, ctl, secs = 12) => {
    const { phys, car } = make(flat(surface));
    run(phys, 0.5);
    Object.assign(car.controls, { range: 'L+', fourWD: true }, ctl);
    let maxA = 0;
    run(phys, secs, () => { const h = car.getState().hardcore; maxA = Math.max(maxA, h.axleStress[0], h.axleStress[1]); });
    return { maxA, snaps: ev(phys, 'axle-snap').length };
  };
  const open = stress('rock', { throttle: 1 }), mud = stress('mud', { throttle: 1, lockF: true, lockR: true });
  const feather = (() => {   // lockers on rock, ~1 m/s crawl with 40 % throttle
    const { phys, car } = make(flat('rock'));
    Object.assign(car.controls, { range: 'L+', fourWD: true, lockF: true, lockR: true });
    let maxA = 0;
    run(phys, 20, () => { const s = car.getState(); car.controls.throttle = s.speed > 1.2 ? 0 : 0.4; maxA = Math.max(maxA, ...s.hardcore.axleStress); });
    return { maxA, snaps: ev(phys, 'axle-snap').length };
  })();
  check('open diffs on rock: no shaft stress', open.maxA === 0 && open.snaps === 0, `max ${n3(open.maxA)}`);
  check('lockers on mud (low grip): no shaft stress', mud.maxA < 0.01 && mud.snaps === 0, `max ${n3(mud.maxA)}`);
  check('lockers on rock, feathered crawl: no snap', feather.maxA < 0.3 && feather.snaps === 0, `max ${n3(feather.maxA)}`);
  // (c) 4WD flat-out in High on rock: transfer case overheats -> forced 2WD, cools -> 4WD back
  {
    const { phys, car } = make(flat('rock'));
    Object.assign(car.controls, { throttle: 1, fourWD: true, range: 'H' });
    let hotT = -1;
    run(phys, 25, i => { if (car.getState().hardcore.xferHot) { hotT = i / 60; return false; } });
    const f = [];
    run(phys, 1, () => { const w = car.getState().wheels; f.push(Math.abs(w[0].driveTorque) + Math.abs(w[1].driveTorque)); });
    check('transfer case overheats under sustained 4WD bind', hotT > 3 && ev(phys, 'overheat').some(e => e.part === 'xfer'), `hot after ${n2(hotT)} s`);
    check('overheated transfer case drops to 2WD (front gets no drive)', mean(f) < 5 && car.controls.fourWD, `front |T| ${mean(f).toFixed(1)} N·m`);
    car.controls.throttle = 0; car.controls.brake = 1;
    run(phys, 15);
    check('it cools down and 4WD re-engages', !car.getState().hardcore.xferHot && ev(phys, 'cooled').some(e => e.part === 'xfer'),
      `heat ${n2(car.getState().hardcore.xferHeat)}`);
  }
});

// ---- 10. hardcore: torque converter (4H bogging, transTemp), Low range, restart ------------------
const ev = (phys, type) => phys.events.filter(e => e.type === type);
/** Vehicle held by its brakes on a `deg` rock incline, then full throttle (4WD) in the given range. */
function hillStart(deg, low, cfg = CFG) {
  const { phys, car } = make(slope(deg), onSlope(deg, 30), cfg);
  car.controls.brake = 1;
  run(phys, 1.5);
  const x0 = car.getState().x;
  Object.assign(car.controls, { brake: 0, throttle: 1, fourWD: true, range: low ? 'L+' : 'H' });
  return { phys, car, x0 };
}
/** Full-throttle 4H run-up on the flat into a `deg` rock ramp starting at SLOPE_X0. */
function runUp(deg, cfg = CFG, each) {
  const { phys, car } = make(slope(deg), { x: -60, y: 0 }, cfg);
  Object.assign(car.controls, { throttle: 1, fourWD: true, range: 'H' });
  let vIn = 0, maxX = -60;
  run(phys, 14, i => { const s = car.getState(); if (s.x < SLOPE_X0) vIn = s.speed; maxX = Math.max(maxX, s.x); if (each) each(car, s, i); if (s.x > SLOPE_X0 + 30) return false; });
  return { phys, car, vIn, up: maxX - SLOPE_X0 };
}
test('torque converter: 4H bogs (no stall) on steep climbs, transTemp slips the drive; Low crawls; momentum carries 4H', check => {
  const H = hillStart(26, false), Hc = CFG.hardcore;
  run(H.phys, 1);
  let s = H.car.getState();
  check('4H from standstill on 26° rock: bogs at converter stall rpm instead of stalling',
    s.engineOn && !H.phys.events.some(e => e.type === 'stall') && s.rpm > 2100 && s.rpm < 2400 && s.hardcore.bog > 0.5 && s.engineTorque > 0,
    `rpm ${s.rpm.toFixed(0)} (stall ${Hc.tcStallRpm}), bog ${n2(s.hardcore.bog)}, Te ${s.engineTorque.toFixed(0)} N·m`);
  check('bogged: does not climb', s.x - H.x0 < 0.5, `dx ${n2(s.x - H.x0)} m`);
  const t1 = s.hardcore.transTemp;
  let hotT = -1, mono = true, prev = t1, dxHot = 0;
  run(H.phys, 10, i => { const h = H.car.getState().hardcore; mono = mono && h.transTemp >= prev - 1e-9; prev = h.transTemp;
    if (h.transHot) { hotT = 1 + i / 60; dxHot = H.car.getState().x - H.x0; return false; } });
  check('bogging builds transTemp until the transmission overheats', t1 > 0.05 && mono && hotT > 1.5 && hotT < 10 && ev(H.phys, 'overheat').some(e => e.part === 'trans'),
    `transTemp ${n2(t1)} after 1 s, hot after ${n2(hotT)} s`);
  check('converter holds the grade while bogging (creeps, no runaway)', Math.abs(dxHot) < 3, `dx ${n2(dxHot)} m when hot`);
  H.car.controls.brake = 1;                          // overheated: driver holds it on the brakes...
  run(H.phys, 0.5);
  s = H.car.getState();
  check('overheated: 4H drive torque slips to 0 (engine still running)', s.engineOn && Math.abs(s.wheelTorque) < 1 && s.hardcore.transHot,
    `Tw ${s.wheelTorque.toFixed(1)} N·m, transTemp ${n2(s.hardcore.transTemp)}`);
  const x1 = s.x;                                     // ...and shifts to Low
  Object.assign(H.car.controls, { brake: 0, range: 'L+' });
  let cooledT = -1;
  run(H.phys, 6, i => { if (cooledT < 0 && !H.car.getState().hardcore.transHot) cooledT = i / 60; });
  s = H.car.getState();
  check('brake + Low: climbs the 26° grade while the transmission cools', s.x - x1 > 4 && !s.stalled, `dx ${n2(s.x - x1)} m in 6 s (Low)`);
  check('cools below transRecover -> 4H drive back', cooledT > 0 && s.hardcore.transTemp <= DEFAULT_HARDCORE.transRecover && ev(H.phys, 'cooled').some(e => e.part === 'trans'),
    `cooled after ${n2(cooledT)} s, transTemp ${n2(s.hardcore.transTemp)}`);
  // below bogAngle 4H still pulls from a standstill; the flat launch never bogs for long
  const G = hillStart(16, false);
  run(G.phys, 6);
  check('4H from standstill on 16° (below bogAngle): climbs, no overheat', G.car.getState().x - G.x0 > 10 && !G.car.getState().hardcore.transHot,
    `dx ${n2(G.car.getState().x - G.x0)} m, transTemp ${n2(G.car.getState().hardcore.transTemp)}`);
  {
    const { phys, car } = make(flat('rock'));
    Object.assign(car.controls, { throttle: 1, fourWD: true, range: 'H' });
    run(phys, 4);
    const f = car.getState();
    check('4H launch on the flat: converter locks up, no bog / heat', f.speed > 5 && f.hardcore.bog === 0 && f.hardcore.transTemp < 0.05 && f.rpm > Hc.tcStallRpm,
      `v ${n2(f.speed)} m/s, rpm ${f.rpm.toFixed(0)}, transTemp ${n3(f.hardcore.transTemp)}`);
  }
  // momentum: a fast run-up carries a 30° climb in 4H; killing the speed on the ramp bogs it
  const M = runUp(30);
  check('4H run-up carries a 30° climb (locked converter)', M.up > 20 && !M.car.getState().hardcore.transHot, `${n2(M.vIn * 3.6)} km/h in, ${n2(M.up)} m up the ramp`);
  const S = hillStart(30, false);
  run(S.phys, 3);
  check('same 30° ramp from a standstill in 4H: bogs', S.car.getState().x - S.x0 < 0.5 && S.car.getState().hardcore.bog > 0.5, `dx ${n2(S.car.getState().x - S.x0)} m`);
  let killAt = -1;
  const K = runUp(30, CFG, (car, st, i) => {                // a "bump": scrub the speed 4 m up the ramp, then full gas again
    if (killAt < 0 && st.x > SLOPE_X0 + 4) killAt = i;
    car.controls.brake = killAt >= 0 && i - killAt < 40 ? 1 : 0;
  });
  check('speed killed mid-climb: falls back into the bog, does not carry on', K.up < 15 && K.up < 0.5 * M.up && K.car.getState().hardcore.bog > 0.5,
    `${n2(K.up)} m up the ramp, bog ${n2(K.car.getState().hardcore.bog)}`);
  // Low range: ~0.35x the 4H top speed, lowMult x torque, strong engine braking on lift
  const top = (cfg, low) => {
    const { phys, car } = make(flat('rock'), { x: 0, y: 0 }, cfg);
    Object.assign(car.controls, { throttle: 1, fourWD: true, range: low ? 'L+' : 'H' });
    let v = 0; run(phys, low ? 5 : 9, () => { v = Math.max(v, car.getState().speed); });
    car.controls.throttle = 0;
    const v0 = car.getState().speed; run(phys, 0.15);
    return { v, dec: (v0 - car.getState().speed) / 0.15 };
  };
  const tH = top(CFG, false), tL = top(CFG, true), r = tL.v / tH.v;
  check('Low: top speed ~0.35x of High', r > 0.3 && r < 0.4, `${n2(tL.v * 3.6)} vs ${n2(tH.v * 3.6)} km/h (${n2(r)}x)`);
  check('Low: strong engine braking on throttle lift', tL.dec > 3 * tH.dec && tL.dec > 4, `${n2(tL.dec)} vs ${n2(tH.dec)} m/s² (High)`);
  // a stalled engine never restarts on GAS: only a key crank (crankTime) brings it back
  const R = hillStart(10, false);
  run(R.phys, 0.5);
  R.car._stall('test');
  run(R.phys, 0.5);
  s = R.car.getState();
  check('stalled: engine dead, no drive torque (only dead-engine drag)', !s.engineOn && s.engineTorque === 0 && s.stalled, `rpm ${s.rpm.toFixed(0)}, Te ${s.engineTorque}`);
  R.car.controls.throttle = 0; run(R.phys, 0.3); R.car.controls.throttle = 1; run(R.phys, 0.5);
  check('GAS release + tap does NOT restart a stalled engine', R.car.getState().stalled && !R.car.getState().engineOn);
  R.car.controls.brake = 1; R.car.controls.ignition++; run(R.phys, 0.1);
  check('key turn cranks (crank event, still not running)', R.car.getState().hardcore.cranking && !R.car.getState().engineOn && ev(R.phys, 'crank').length === 1);
  R.car.controls.throttle = 0; run(R.phys, 1.5);
  check('engine catches after crankTime, back at idle', R.car.getState().engineOn && !R.car.getState().stalled && ev(R.phys, 'restart').length === 1
    && Math.abs(R.car.getState().rpm - CFG.engine.idleRpm) < 150, `rpm ${R.car.getState().rpm.toFixed(0)}`);
});

// ---- 10b. ignition key: engine-off spawn, crank -> idle, shutdown, lug stalls need a crank ---------------------
test('ignition: spawns engine-off, key cranks to idle / shuts off; Low lugging + a boiled converter stall, only the key restarts', check => {
  {
    const phys = new PhysicsWorld(flat('rock')), car = new Vehicle(phys, CFG, { x: 0, y: 0 });
    run(phys, 1);
    let s = car.getState();
    check('spawn: engine off (rpm 0, no idle, not cranking)', !s.engineOn && s.rpm < 1 && !s.hardcore.cranking && s.hardcore.engineOff && !s.stalled, `rpm ${s.rpm.toFixed(0)}`);
    const x0 = s.x; car.controls.throttle = 1; run(phys, 2);
    s = car.getState();
    check('engine off: full GAS does nothing (no torque, no fuel burn)', Math.abs(s.x - x0) < 0.05 && s.engineTorque === 0 && s.fuel === 1, `dx ${n3(s.x - x0)} m`);
    car.controls.throttle = 0; car.controls.ignition++; run(phys, 0.3);
    s = car.getState();
    check('key turn: cranks (starter rpm below idle, crank event)', s.hardcore.cranking && !s.engineOn && s.rpm > 50 && s.rpm < CFG.engine.idleRpm && ev(phys, 'crank').length === 1,
      `rpm ${s.rpm.toFixed(0)} after 0.3 s`);
    run(phys, 1.2);
    s = car.getState();
    check('catches after crankTime -> idle (start event)', s.engineOn && ev(phys, 'start').length === 1 && Math.abs(s.rpm - CFG.engine.idleRpm) < 120, `rpm ${s.rpm.toFixed(0)}`);
    car.controls.throttle = 1; run(phys, 2);
    check('running: drives', car.getState().x - x0 > 2, `dx ${n2(car.getState().x - x0)} m`);
    car.controls.throttle = 0; car.controls.ignition++; run(phys, 1.5);
    s = car.getState();
    check('key again: shutdown event, engine off, rpm falls to 0', !s.engineOn && s.rpm < 20 && !s.stalled && ev(phys, 'shutdown').length === 1, `rpm ${s.rpm.toFixed(0)}`);
    car.startEngine(); car.reset();
    check('reset(): engine off again', !car.getState().engineOn && car.getState().hardcore.engineOff);
  }
  {  // Low: power-braking (Low torque out-pulls the brakes) lugs the engine to a stall; GAS alone never restarts it
    const { phys, car } = make(flat('rock'));
    Object.assign(car.controls, { range: 'L', fourWD: true, brake: 1, throttle: 1 });
    let stallT = -1; run(phys, 5, i => { if (stallT < 0 && car.getState().stalled) stallT = i / 60; });
    const e = ev(phys, 'stall');
    check('Low power-braking (full brake + full gas): lugs to a stall in ~1-3 s', stallT > 0.6 && stallT < 3.5 && e.length === 1 && e[0].cause === 'brake', `stalled after ${n2(stallT)} s (${e.map(x => x.cause)})`);
    car.controls.brake = 0;
    for (let k = 0; k < 3; k++) { car.controls.throttle = 0; run(phys, 0.4); car.controls.throttle = 1; run(phys, 0.4); }
    check('stalled: GAS taps never restart it', car.getState().stalled && !car.getState().engineOn && car.getState().engineTorque === 0);
    car.controls.throttle = 0; car.controls.ignition++; run(phys, 1.5);
    check('key crank restarts it (restart event)', car.getState().engineOn && ev(phys, 'restart').length === 1);
    car.controls.throttle = 0.6; run(phys, 4);
    check('restarted in Low: crawls, no new stall', car.getState().speed > 1 && ev(phys, 'stall').length === 1, `v ${n2(car.getState().speed)} m/s`);
  }
  {  // Low dragged backward against the gear (no gas, e.g. sliding back off a ledge): the back-driven engine stalls
    const { phys, car } = make(flat('rock'));
    Object.assign(car.controls, { range: 'L', fourWD: true });
    run(phys, 0.5);
    const R = CFG.wheels.radius, push = () => { car.chassis.setLinearVelocity({ x: -2, y: car.chassis.getLinearVelocity().y }); for (const w of car.wheels) w.body.setAngularVelocity(2 / R); };
    phys.postStep.push(push);                          // after each solve: the next step reads a back-driven shaft
    run(phys, 3);
    phys.postStep.splice(phys.postStep.indexOf(push), 1);
    const e = ev(phys, 'stall');
    check('Low, no gas, rolled back at 2 m/s against the gear: stalls (cause lug)', e.length === 1 && e[0].cause === 'lug' && car.getState().stalled, e.map(x => x.cause).join());
  }
  {  // 4H: bogged until the transmission boils, still floored -> the engine stalls
    const H = hillStart(26, false);
    let hotT = -1, stallT = -1;
    run(H.phys, 20, i => { const s = H.car.getState(); if (hotT < 0 && s.hardcore.transHot) hotT = i / 60; if (stallT < 0 && s.stalled) { stallT = i / 60; return false; } });
    const e = ev(H.phys, 'stall');
    check('4H bog: no stall while the converter is merely hot; floored past transHot it stalls', hotT > 1 && stallT > hotT + 2 && e.length === 1 && e[0].cause === 'trans',
      `trans hot ${n2(hotT)} s, stalled ${n2(stallT)} s`);
  }
  {  // dry tank: engine dies, cranks but never catches
    const { phys, car } = make(flat('rock'));
    car.hc.fuel = 1e-6; car.controls.throttle = 1; run(phys, 0.5);
    check('running dry kills the engine (stall + out-of-fuel)', car.getState().stalled && ev(phys, 'out-of-fuel').length === 1 && ev(phys, 'stall').some(x => x.cause === 'fuel'));
    car.controls.ignition++; run(phys, 1.5);
    check('dry: key cranks but no-start', !car.getState().engineOn && ev(phys, 'no-start').length === 1);
  }
});

// ---- 11. hardcore: impact / rollover damage --------------------------------------------------------
test('hardcore damage: hard hits and rollovers bend shocks and crush the body; damage cuts power', check => {
  const drop = h => { const { phys, car } = make(flat(), { x: 0, y: h }); run(phys, 3); return car.getState().hardcore; };
  const d16 = drop(1.6), d4 = drop(4);
  check('1.6 m drop: no damage', d16.body === 0 && Math.max(...d16.shock) === 0);
  check('4 m drop: bent shocks, body intact', Math.min(...d4.shock) > 0.03 && d4.body === 0, `shocks ${d4.shock.map(n2).join('/')}`);
  // full throttle (High) into a 1.2 m rock wall
  {
    const { phys, car } = make({ ground: [{ surface: 'rock', points: [[-200, 0], [40, 0], [40, 1.2], [400, 1.2]] }] }, { x: 15, y: 0 });
    Object.assign(car.controls, { throttle: 1, fourWD: true, range: 'H' });
    let vHit = 0;
    run(phys, 6, () => { const s = car.getState(); if (s.wheels[0].x < 39.4) vHit = s.speed; });
    const h = car.getState().hardcore;
    check('slamming a rock wall crushes the body', h.body > 0.15 && phys.events.some(e => e.type === 'damage'), `body ${n2(h.body)} at ~${n2(vHit)} m/s`);
    check('front shocks take the hit, rear ones are spared', h.shock[0] > h.shock[2] + 0.05, `shocks ${h.shock.map(n2).join('/')}`);
  }
  // rollover onto the roof; recover keeps damage, reset repairs
  {
    const { phys, car } = make(flat(), { x: 0, y: 2.5, angle: Math.PI });
    run(phys, 3);
    const h = car.getState().hardcore;
    check('rolling onto the roof: rollover damage once', phys.events.filter(e => e.type === 'rollover').length === 1 && h.body >= car.hcCfg.rollDamage,
      `body ${n2(h.body)}`);
    const b = h.body;
    car.recover(); run(phys, 2);
    check('recover() keeps the damage (hardcore)', car.getState().hardcore.body === b && !car.getState().upsideDown);
    car.reset();
    check('reset() repairs everything', car.getState().hardcore.body === 0 && Math.max(...car.getState().hardcore.shock) === 0);
  }
  // effects: body damage cuts power, shock damage sags the suspension
  const accel = body => {
    const { phys, car } = make(flat('rock'));
    run(phys, 0.5);
    car.hc.body = body;
    const x0 = car.getState().x;
    Object.assign(car.controls, { throttle: 1, fourWD: true, range: 'H' });
    run(phys, 2);
    return car.getState().x - x0;
  };
  const fresh = accel(0), wrecked = accel(1);
  check('a wrecked body loses power (slower launch)', wrecked < 0.85 * fresh, `${n2(wrecked)} m vs ${n2(fresh)} m in 2 s`);
  const sag = dmg => { const { phys, car } = make(flat()); car.hc.shock = [dmg, dmg, dmg, dmg]; run(phys, 3); return mean(car.getState().wheels.map(w => w.travel)); };
  const s0 = sag(0), s1 = sag(1);
  check('blown shocks sag onto their (warped, early) bump stops', s1 > s0 + 0.015, `static travel ${n3(s0)} -> ${n3(s1)} m`);
  // a blown front pair drops the nose; blown shocks keep bouncing after a drop instead of settling
  { const { phys, car } = make(flat()); car.hc.shock = [1, 1, 0, 0]; run(phys, 3);
    check('blown front shocks: the nose sags', car.getState().angle < -0.01, `pitch ${n2(car.getState().angle * 57.3)}°`); }
  const bounce = dmg => {
    const { phys, car } = make(flat(), { x: 0, y: 0.6 }); car.hc.shock = [dmg, dmg, dmg, dmg];
    let e = 0; run(phys, 3, i => { if (i > 60) e += Math.abs(car.getState().vy); });
    return e / 120;
  };
  const b0 = bounce(0), b1 = bounce(0.7);
  check('damaged shocks bounce erratically (more residual body motion)', b1 > 1.5 * b0, `mean |vy| ${n3(b0)} -> ${n3(b1)} m/s`);
  // misfires: heavy damage sputters under full throttle; a fresh truck never misfires
  const misfires = body => {
    const { phys, car } = make(flat('rock')); run(phys, 0.5); car.hc.body = body;
    Object.assign(car.controls, { throttle: 1, fourWD: true, range: 'H' });
    let n = 0; run(phys, 3, () => { if (car.getState().hardcore.misfire) n++; });
    return n;
  };
  const m0 = misfires(0), m1 = misfires(0.9);
  check('wrecked engine bay misfires under full throttle, fresh one never', m0 === 0 && m1 > 10, `${m0} vs ${m1} misfire steps in 3 s`);
  // fast lockers: High range, lockers on, rock, > 25 km/h -> shaft snaps sooner than without the speed penalty
  const snapAt = gain => {
    const cfg = clone(CFG); cfg.hardcore = { lockerFastGain: gain };
    const { phys, car } = make(flat('rock'), { x: 0, y: 0 }, cfg);
    Object.assign(car.controls, { throttle: 1, fourWD: true, lockF: true, lockR: true, range: 'H' });
    let t = -1, vmax = 0;
    run(phys, 25, i => { const s = car.getState(); vmax = Math.max(vmax, s.speed); if (s.hardcore.axleBroken.some(b => b)) { t = i / 60; return false; } });
    return { t, vmax };
  };
  const fast = snapAt(car_default_gain()), slow = snapAt(1);
  check('lockers left on above 25 km/h on rock snap a shaft sooner', fast.t > 0 && (slow.t < 0 || fast.t < 0.9 * slow.t) && fast.vmax > 6.94,
    `snap ${n2(fast.t)} s vs ${n2(slow.t)} s without the penalty (vmax ${n2(fast.vmax * 3.6)} km/h)`);
});
function car_default_gain() { return DEFAULT_HARDCORE.lockerFastGain; }

// ---- 12. content: manifest + level02 design intents ------------------------------------------------
const { FALLBACK_MANIFEST, mergeManifest } = await import('../engine/manifest.js');
const MANIFEST = await loadJSON('../content.json');
const LEVELS = {};
for (const l of MANIFEST.levels) LEVELS[l.id] = await loadJSON('../' + l.file);
const VEHICLES = {};
for (const v of MANIFEST.vehicles) VEHICLES[v.id] = await loadJSON('../' + v.file);
const SCHEMA = ['name', 'chassis', 'wheels', 'axles', 'engine', 'drivetrain', 'brakes', 'winch'];
const groundY = (lv, x) => {
  for (const r of lv.ground) for (let i = 1; i < r.points.length; i++) {
    const a = r.points[i - 1], b = r.points[i];
    if (a[0] <= x && b[0] >= x && b[0] > a[0]) return a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0]);
  }
  return 0;
};
// Line-picking driver for the content bots (a sensible player). Hard: partial-width rock boulders (z0 / z1) and tall
// side shelves (zone dy >= 0.3) are walls. Soft: lane zones that rut or sink (dig surface or dy < 0) are avoided when a
// wall-free line allows; jerry cans (cans) within 9 m pull the line over them (tyre footprint). Among wall-free trail z it
// scores distance from its own z + soft lanes - cans + a pull to the centre (hysteresis keeps the target); a P-loop on heading steers.
const CRAWL_FUEL = 0.1;   // the planned crawl still has this much fuel at the finish (fuel = part of the plan, never a squeeze)
function linePlanner(lv, car, opt = {}) {
  const hard = [], soft = [], half = q => [q.side === 'R' ? 0 : (q.z0 ?? -9), q.side === 'L' ? 0 : (q.z1 ?? 9)];
  const dig = n => /mud|bog|clay/.test(n) || ((lv.surfaces || {})[n] || {}).dig > 0;
  for (const l of lv.logs || []) if ((l.z0 != null || l.z1 != null) && /rock/.test(l.surface || '')) hard.push({ x0: l.x - l.r, x1: l.x + l.r, lo: l.z0 ?? -9, hi: l.z1 ?? 9 });
  for (const q of lv.zones || []) {
    const [lo, hi] = half(q), lane = q.side === 'L' || q.side === 'R' || q.z0 != null || q.z1 != null;
    if ((q.dy || 0) >= 0.3 && !q.climb) hard.push({ x0: q.x0 - 0.25, x1: q.x1 + 0.25, lo, hi });   // climb: formation block (drive over)
    else if (lane && ((q.dy || 0) < 0 || dig(q.surface))) soft.push({ x0: q.x0, x1: q.x1, lo, hi });
  }
  for (const br of lv.bridges || []) for (const sd of [-1, 1]) {      // bridge rope rails (1.85) + entry posts: walls beside the deck
    const x0 = Math.min(br.from[0], br.to[0]) - 1.5, x1 = Math.max(br.from[0], br.to[0]) + 0.3;
    hard.push(sd > 0 ? { x0, x1, lo: 1.75, hi: 9 } : { x0, x1, lo: -9, hi: -1.75 });
  }
  const cans = opt.cans === false ? [] : (lv.cans || []).map(c => ({ ...c }));
  const ht = car.halfTrack, hw = car.tyreWidth / 2, c = car.controls, foot = ht + hw;
  const hit = (z, o, m) => !(z + foot + m < o.lo || z - foot - m > o.hi);
  const zs = Array.from({ length: 45 }, (_, k) => -1.1 + k * 0.05);
  let tgt = 0;
  return s => {
    const rx = Math.min(s.wheels[2].x, s.wheels[3].x) - s.wheels[2].radius, fx = Math.max(s.wheels[0].x, s.wheels[1].x);
    const H = hard.filter(o => o.x1 + 0.3 > rx && o.x0 < s.x + 8), S = soft.filter(o => o.x1 > rx && o.x0 < fx + 5);
    const C = cans.filter(k => !k.got && k.x > rx && k.x < s.x + 9);
    for (const k of cans) if (!k.got && k.x < rx) k.got = true;
    let ok = zs.filter(z => H.every(o => !hit(z, o, 0.15)));
    if (!ok.length && H.length) ok = zs.filter(z => !hit(z, H[0], 0.15));
    if (!ok.length) ok = zs;
    const wet = (z, o) => Math.max(0, Math.min(z + foot, o.hi) - Math.max(z - foot, o.lo)) / foot;   // footprint share in the lane
    const score = z => Math.abs(z - s.z) * 0.6 + Math.abs(z) * 0.15 + S.reduce((a, o) => a + 1.2 * wet(z, o), 0)
      + 4 * Math.max(0, Math.abs(z) + foot - 1.75)                                                     // outer tyre up on the slick berm
      - C.reduce((a, k) => a + (Math.abs(k.z - z) < foot - 0.05 ? 3 : 0), 0);
    let best = ok[0];
    for (const z of ok) if (score(z) < score(best)) best = z;
    if (!ok.some(z => Math.abs(z - tgt) < 0.026) || score(tgt) > score(best) + 0.3) tgt = best;
    const want = Math.max(-0.33, Math.min(0.33, 1.6 * (tgt - s.z)));
    c.steer = Math.max(-1, Math.min(1, 4 * (want - s.yaw)));
  };
}
test('content: manifest entries load and settle; 4-stage continuous trails are crawlable on a budget', check => {
  for (const [id, lv] of Object.entries(LEVELS)) {
    const cont = lv.ground.every((r, i) => i === 0 || (r.points[0][0] === lv.ground[i - 1].points.at(-1)[0] && r.points[0][1] === lv.ground[i - 1].points.at(-1)[1]));
    for (const [vid, vcfg] of Object.entries(VEHICLES)) {
      const { phys, car } = make(lv, lv.spawn, vcfg);
      run(phys, 2);
      const s = car.getState();
      check(`${id} + ${vid}: ground runs continuous, spawn settles on 4 wheels`, cont && s.wheels.every(w => w.grounded) && Math.abs(s.angle) < 0.1 && lv.finish.x > lv.spawn.x,
        `${lv.name}, finish x ${lv.finish.x}`);
    }
  }
  // static-host manifest: engine/manifest.js fallback mirrors content.json (ids, files, names, card text) so no directory listing is needed
  const same = (A, B, ks) => A.length === B.length && A.every((x, i) => ks.every(k => JSON.stringify(x[k]) === JSON.stringify(B[i][k])));
  const merged = mergeManifest(null);
  check('manifest: engine/manifest.js FALLBACK_MANIFEST mirrors content.json; merge keeps all levels + trucks',
    same(FALLBACK_MANIFEST.levels, MANIFEST.levels, ['id', 'file', 'thumb', 'name', 'difficulty', 'blurb']) && same(FALLBACK_MANIFEST.vehicles, MANIFEST.vehicles, ['id', 'file', 'name', 'chips', 'trait', 'stats'])
      && merged.levels.length === MANIFEST.levels.length && merged.vehicles.length === MANIFEST.vehicles.length && mergeManifest({ levels: [MANIFEST.levels[1]] }).levels.length === MANIFEST.levels.length
      && MANIFEST.levels.every(l => !l.blurb || l.blurb.length <= 20) && MANIFEST.vehicles.every(v => (v.chips || []).length <= 2 && (!v.trait || v.trait.length <= 20)),
    `${MANIFEST.levels.length} levels, ${MANIFEST.vehicles.length} trucks`);
  for (const [vid, v] of Object.entries(VEHICLES)) check(`${vid}: schema keys present, 2 axles, convex parts <= 8 pts`,
    SCHEMA.every(k => k in v) && v.axles.length === 2 && v.chassis.shapes.every(sh => sh.points.length <= 8), v.name);
  // continuous 4-stage trails: stage posts, no long flats past the spawn pad, split lanes, embedded rocks, off-camber
  const crawlBot = (lv, cfg = CFG, secs = 420, steer = true) => {      // 4WD + lockers + L, feathered gas, winch when stuck, release past the anchor
    const { phys, car } = make(lv, lv.spawn, cfg), c = car.controls, line = linePlanner(lv, car);
    Object.assign(c, { fourWD: true, lockF: true, lockR: true, range: 'L' });
    let best = 0, bestI = 0, wOn = false, tap = 0, res = 'stuck', winched = false;
    run(phys, secs, i => {
      const s = car.getState();
      c.throttle = s.speed > 1.6 ? 0 : 0.8;
      if (steer) line(s);
      if (!s.engineOn && !s.hardcore.cranking && !s.hardcore.dry) c.ignition++;
      if (s.x > best + 0.3) { best = s.x; bestI = i; }
      if (!wOn && i - bestI > 150) wOn = true;
      if (wOn && ((car.winch.attached && car.winch.length <= car.winch.minLength + 0.05) || i - bestI < 30)) wOn = false;
      c.winch = wOn ? 1 : 0;
      if (!wOn && s.winch.attached && s.x > s.winch.ax - 1.2) { tap++; c.winch = tap < 4 ? 1 : 0; if (tap > 8) tap = 0; } else if (!wOn) tap = 0;
      winched = winched || car.winch.attached;
      if (Math.cos(s.angle) < -0.2) { res = 'flip'; return false; }
      if (s.x > lv.finish.x) { res = 'pass'; return false; }
      if (i - bestI > 900) return false;
    });
    const s = car.getState(), got = phys.cans.filter(k => k.taken);
    return { res, x: s.x, fuel: s.fuel, t: phys.time, winched, cans: got.length, used: phys.tank + got.reduce((a, k) => a + k.fuel, 0) - s.fuel };
  };
  // mindless masher: full gas in H, steers the same line round boulders, never winches (only re-cranks a stall);
  // fourWD = also 4WD + both lockers
  const mashBot = (lv, cfg, fourWD = false, secs = 120) => {
    const { phys, car } = make(lv, lv.spawn, cfg), c = car.controls, line = linePlanner(lv, car);
    if (fourWD) Object.assign(c, { fourWD: true, lockF: true, lockR: true });
    let best = 0, bestI = 0, res = 'stuck';
    run(phys, secs, i => {
      const s = car.getState();
      c.throttle = 1;
      line(s);
      if (!s.engineOn && !s.hardcore.cranking && !s.hardcore.dry) c.ignition++;   // stalled: re-crank with the key
      if (s.x > best + 0.3) { best = s.x; bestI = i; }
      if (Math.cos(s.angle) < -0.2) { res = 'flip'; return false; }
      if (s.x > lv.finish.x) { res = 'pass'; return false; }
      if (i - bestI > 900) return false;
    });
    const s = car.getState(), ev = new Set(phys.events.map(e => e.type)), shock = Math.max(...s.hardcore.shock);
    const penalty = ['axle-snap', 'overheat', 'out-of-fuel', 'rollover'].filter(k => ev.has(k)).concat(shock >= 0.8 ? [`shock ${n2(shock)}`] : []);
    return { res, x: s.x, t: phys.time, penalty, used: 1 - s.fuel };
  };
  const tag = r => `${r.res} x ${n2(r.x)}${r.penalty && r.penalty.length ? ' ' + r.penalty.join('/') : ''}`;
  const fuelRuns = {}, mashRuns = {};
  for (const id of ['level01', 'level02', 'level03', 'level05'].filter(id => !LV_ONLY || LV_ONLY.test(id))) {
    const lv = LEVELS[id], st = lv.stages || [];
    let flatRun = 0, flatMax = 0;
    for (const r of lv.ground) for (let i = 1; i < r.points.length; i++) {
      const a = r.points[i - 1], b = r.points[i];
      if (a[0] < 6 || a[0] >= lv.finish.x - 3 || (lv.bridges || []).some(br => a[0] >= br.from[0] && b[0] <= br.to[0])) continue;   // spawn pad, run-out, gorge floors
      flatRun = Math.abs(b[1] - a[1]) < 0.03 * (b[0] - a[0]) ? flatRun + b[0] - a[0] : 0; flatMax = Math.max(flatMax, flatRun);
    }
    const split = (lv.zones || []).some(z => z.side === 'L') && (lv.zones || []).some(z => z.side === 'R');
    const rocks = (lv.logs || []).filter(l => l.surface === 'rock' && l.embed > 0).length, camber = (lv.zones || []).filter(z => z.dy).length;
    check(`${id}: 5 ordered stage posts, no flat > 6 m, split lanes, embedded rocks, off-camber, >= 245 m long`,
      st.length === 5 && lv.finish.x - lv.spawn.x >= 245 && st.every((s, i) => s.name === `STAGE ${i + 1}` && (i === 0 || s.x > st[i - 1].x) && s.x < lv.finish.x) && flatMax <= 6 && split && rocks >= 3 && camber >= 2,
      `stages ${st.length}, flat ${n2(flatMax)} m, rocks ${rocks}, camber zones ${camber}`);
    // winch anchors only at high-consequence hang-ups: not in stage 1 (tutorial excepted), and within 14 m before each
    // anchor a rut-able surface, an off-camber zone or a sustained climb (3 m window averaging >= 20 deg)
    const dig = n => /mud|bog|clay/.test(n) || ((lv.surfaces || {})[n] || {}).dig > 0;
    const hangUp = a => (lv.zones || []).some(z => (z.dy || dig(z.surface)) && (z.x1 ?? 1e9) >= a.x - 14 && (z.x0 ?? -1e9) <= a.x + 2) ||
      lv.ground.some(r => dig(r.surface) && r.points.at(-1)[0] >= a.x - 14 && r.points[0][0] <= a.x + 2) ||
      Array.from({ length: 23 }, (_, k) => a.x - 14 + k * 0.5).some(x => Math.atan2(groundY(lv, x + 3) - groundY(lv, x), 3) >= 20 * Math.PI / 180);
    const anchors = lv.anchors || [];
    check(`${id}: winch anchors (${anchors.length}) sit only at hang-ups (ruts / off-camber / sustained climb), none in stage 1`,
      anchors.length <= 3 && anchors.every(a => hangUp(a) && (id === 'level01' || a.x > (st[1] || {}).x)),
      anchors.map(a => `${a.name} x ${a.x}`).join(', '));
    const crawl = {}, mash = {}, mash4 = {}, straight = {};
    for (const [vid, vcfg] of Object.entries(VEHICLES)) {
      crawl[vid] = crawlBot(lv, vcfg); mash[vid] = mashBot(lv, vcfg); mash4[vid] = mashBot(lv, vcfg, true); straight[vid] = crawlBot(lv, vcfg, 300, false);
    }
    // line choices: walls (partial boulders, tall side shelves) in both lanes + a mix of softer choices (climbable side
    // ledges, sunken bogs, half-width trunks, jerry cans) so the trail is not one obstacle repeated
    const part = l => l.z0 != null || l.z1 != null, Z = lv.zones || [];
    const wallL = (lv.logs || []).filter(l => part(l) && /rock/.test(l.surface || '')).map(l => (l.z1 < 0 ? 'L' : 'R'))
      .concat(Z.filter(q => (q.dy || 0) >= 0.6).map(q => (q.side === 'L' || q.z1 < 0 ? 'L' : 'R')));
    const kinds = new Set([...(lv.logs || []).filter(part).map(l => (/rock/.test(l.surface || '') ? 'boulder' : 'trunk')),
      ...Z.filter(q => (q.dy || 0) >= 0.6).map(() => 'shelf'), ...Z.filter(q => q.dy >= 0.3 && q.dy < 0.6).map(() => 'ledge'),
      ...Z.filter(q => q.dy < 0).map(() => 'bog'), ...Z.filter(q => q.climb).map(() => 'formation')]);
    check(`${id}: the line matters -- walls in both lanes, >= 3 kinds of line choice; the same crawl without steering never finishes`,
      wallL.length >= 3 && wallL.includes('L') && wallL.includes('R') && kinds.size >= 3 && Object.values(straight).every(b => b.res !== 'pass'),
      `${wallL.length} walls, ${[...kinds].join('/')}; ` + Object.entries(straight).map(([v, b]) => `${v} ${b.res} x ${n2(b.x)}`).join(', '));
    check(`${id}: every truck crawls it (4WD, lockers, L, winch if stuck) with >= ${CRAWL_FUEL * 100} % fuel${id === 'level01' ? '' : ', no winch needed'}`,
      Object.values(crawl).every(b => b.res === 'pass' && b.fuel >= CRAWL_FUEL && (id === 'level01' || !b.winched)),
      Object.entries(crawl).map(([v, b]) => `${v} ${b.res} ${n2(b.t)} s ${Math.round(b.fuel * 100)}%${b.winched ? ' W' : ''} (${b.cans} can)`).join(', '));
    fuelRuns[id] = crawl; mashRuns[id] = mash4;
    check(`${id}: mashing full gas in 2WD H never finishes (ledges bog the converter, trans overheats)`,
      Object.values(mash).every(b => b.res !== 'pass'), Object.entries(mash).map(([v, b]) => `${v} ${tag(b)}`).join(', '));
    check(`${id}: mashing full gas in 4WD H + lockers fails or pays (axle snap, overheat, bent shocks)`,
      Object.values(mash4).every(b => b.res !== 'pass' || b.penalty.length > 0), Object.entries(mash4).map(([v, b]) => `${v} ${tag(b)}`).join(', '));
  }
  // fuel: no jerry cans, every level starts full; a careful crawl burns a modest share of the tank, mashing full gas burns
  // fuel far faster per metre (lifted throttle, Low range and no wheelspin are what save it)
  const order = MANIFEST.levels.map(l => l.id).filter(id => !LV_ONLY || LV_ONLY.test(id)), per = [], mashPer = [];
  for (const id of order) for (const b of Object.values(fuelRuns[id] || {})) per.push(b.used / Math.max(1, b.t));
  for (const id of order) for (const b of Object.values(mashRuns[id] || {})) if (b.t > 20) mashPer.push(b.used / b.t);
  const runsAll = order.flatMap(id => Object.values(fuelRuns[id] || {})), usedAll = runsAll.map(b => b.used), avg = a => a.reduce((x, y) => x + y, 0) / Math.max(1, a.length);
  check('fuel: no jerry cans / start tanks; a careful crawl uses 10-30 % of a full tank per level (36 % if it had to winch), full-gas mashing burns >= 1.8x faster',
    order.every(id => !(LEVELS[id].cans || []).length && (LEVELS[id].tank ?? 1) === 1) && runsAll.every(b => b.used >= 0.1 && b.used <= (b.winched ? 0.36 : 0.3)) && avg(mashPer) >= 1.8 * avg(per),
    `crawl used ${Math.round(Math.min(...usedAll) * 100)}-${Math.round(Math.max(...usedAll) * 100)} % (avg ${Math.round(avg(usedAll) * 100)} %); per minute crawl ${(avg(per) * 6000).toFixed(1)} %, mash ${(avg(mashPer) * 6000).toFixed(1)} %`);
  // tutorial = visual only: each stage glows the controls it needs (no banner text); the key glows at spawn
  const glows = (LEVELS.level01.stages || []).flatMap(s => s.glow || []).concat(LEVELS.level01.glow || []);
  check('level01 tutorial glows teach KEY, steering, 4WD, shifter, F/R locks, WINCH (no hint text)',
    ['key', 'wheel', 'fourWD', 'shifter', 'lockF', 'lockR', 'winch'].every(k => glows.includes(k)) && (LEVELS.level01.stages || []).every(s => !s.hint), glows.join(' '));
  const other = Object.keys(LEVELS).filter(id => id !== 'level01');
  check('non-tutorial levels: no crossing hints, every stage has a short stuck tip',
    other.every(id => (LEVELS[id].stages || []).every(s => !s.hint && typeof s.stuck === 'string' && s.stuck.length > 0 && s.stuck.length <= 70)), other.join(', '));
});

// ---- 12b. archetypes: Rock Crawler / Mega Bogger / Rock Buggy ------------------------------------
test('archetypes: crawler Low torque + braking, bogger trans spike + heavy tyres, buggy 4H speed + momentum', check => {
  const A = { crawler01: VEHICLES.crawler01, crawler02: VEHICLES.crawler02, crawler03: VEHICLES.crawler03 };
  const ids = Object.keys(A);
  const mult = {}, brake = {}, spike = {}, top = {};
  for (const id of ids) {
    const { car } = make(flat(), { x: 0, y: 0 }, A[id]);
    mult[id] = car.gearScale('L+') / car.gearScale('H');
    // 4L throttle-lift engine braking (deceleration from Low top speed)
    {
      const { phys, car } = make(flat('rock'), { x: 0, y: 0 }, A[id]);
      Object.assign(car.controls, { throttle: 1, fourWD: true, range: 'L+' });
      run(phys, 5); car.controls.throttle = 0;
      const v0 = car.getState().speed; run(phys, 0.15); brake[id] = (v0 - car.getState().speed) / 0.15;
    }
    // stopped dead on a 26° rock incline in 4H: time until the transmission overheats
    { const H = hillStart(26, false, A[id]); let t = -1; run(H.phys, 10, i => { if (H.car.getState().hardcore.transHot) { t = i / 60; return false; } }); spike[id] = t; }
    { const { phys, car } = make(flat('rock'), { x: 0, y: 0 }, A[id]); Object.assign(car.controls, { throttle: 1, fourWD: true, range: 'H' });
      let v = 0; run(phys, 10, () => { v = Math.max(v, car.getState().speed); }); top[id] = v; }
  }
  const f = o => ids.map(id => `${id} ${n2(o[id])}`).join(', ');
  check('4L torque multipliers: crawler 3.5x, bogger 3.0x, buggy 2.2x',
    Math.abs(mult.crawler01 - 3.5) < 1e-6 && Math.abs(mult.crawler02 - 3.0) < 1e-6 && Math.abs(mult.crawler03 - 2.2) < 1e-6, f(mult));
  check('Rock Crawler: strongest 4L engine braking', brake.crawler01 > 1.3 * Math.max(brake.crawler02, brake.crawler03), `${f(brake)} m/s²`);
  check('Mega Bogger: fastest transTemp spike stopped dead on a 26° incline', spike.crawler02 > 0 && ids.every(id => id === 'crawler02' || (spike[id] > spike.crawler02 * 1.3)),
    `hot after ${f(spike)} s`);
  check('Mega Bogger: heaviest tyres (highest wheel inertia per radius²)',
    ids.every(id => id === 'crawler02' || A[id].wheels.inertia / A[id].wheels.radius ** 2 < A.crawler02.wheels.inertia / A.crawler02.wheels.radius ** 2));
  check('Rock Buggy: highest 4H top speed', ids.every(id => id === 'crawler03' || top[id] < 0.8 * top.crawler03), `${f(top.crawler01 ? Object.fromEntries(ids.map(i => [i, top[i] * 3.6])) : top)} km/h`);
  const bm = runUp(35, A.crawler03);
  check('Rock Buggy: 4H momentum climb up a 35° rock ramp', bm.up > 20, `${n2(bm.vIn * 3.6)} km/h in, ${n2(bm.up)} m up`);
  const bb = hillStart(18, false, A.crawler03), cc = hillStart(18, false, A.crawler01);
  run(bb.phys, 5); run(cc.phys, 5);
  check('Rock Buggy: bogs from a dead stop on 18° rock in 4H (Rock Crawler still pulls it)',
    bb.car.getState().x - bb.x0 < 0.5 && bb.car.getState().hardcore.bog > 0.5 && cc.car.getState().x - cc.x0 > 3,
    `buggy dx ${n2(bb.car.getState().x - bb.x0)} m, crawler dx ${n2(cc.car.getState().x - cc.x0)} m`);
  const c22 = hillStart(23, false, A.crawler01); run(c22.phys, 3);
  check('Rock Crawler: bogs above 22° in 4H', c22.car.getState().x - c22.x0 < 0.5 && c22.car.getState().hardcore.bog > 0.5, `dx ${n2(c22.car.getState().x - c22.x0)} m on 23°`);
});

// ---- 12c. default drivetrain, H / L+ / L / L- ranges, fuel ------------------------------------------
test('drivetrain defaults 2WD/H; L+ / L / L- ~15 / 10 / 5 km/h with rising torque + braking; fuel punishes revs', check => {
  {
    const { car } = make(flat()), s = car.getState();
    check('fresh truck: 2WD, F/R lockers open, gear H, full tank', !s.fourWD && !s.lockF && !s.lockR && s.gear === 'H' && s.range === 'H' && s.fuel === 1, `${s.gear} ${s.fourWD ? '4WD' : '2WD'}`);
  }
  {  // range shifts commit only when stopped: a moving request waits as pendingRange
    const { phys, car } = make(flat('rock'));
    Object.assign(car.controls, { throttle: 1, fourWD: true, range: 'H' });
    run(phys, 3);
    car.controls.range = 'L-'; run(phys, 0.2);
    const m = car.getState(), mv = { range: m.range, pend: m.pendingRange, v: m.speed };
    Object.assign(car.controls, { throttle: 0, brake: 1 }); run(phys, 6);
    const s = car.getState();
    check('moving range request stays pending, commits once stopped', mv.range === 'H' && mv.pend === 'L-' && mv.v > 1 && s.range === 'L-' && s.pendingRange === null,
      `at ${n2(mv.v)} m/s: ${mv.range} (pending ${mv.pend}) -> stopped: ${s.range}`);
    car.controls.range = 'H'; run(phys, 0.05); car.controls.range = 'L-';
    car.reset(); car.startEngine();
    // backward: reversing, then coasting backward (negative speed) also keeps the request pending until stopped
    Object.assign(car.controls, { brake: 0, throttle: 1, reverse: true, fourWD: true, range: 'H' }); run(phys, 3);
    const rv = car.getState().speed;
    car.controls.range = 'L'; run(phys, 0.2);
    const snap = () => { const g = car.getState(); return { speed: g.speed, range: g.range, pendingRange: g.pendingRange, gear: g.gear }; };   // getState() is reused
    const r1 = snap(); car.controls.throttle = 0; run(phys, 0.1);
    const r2 = snap();
    Object.assign(car.controls, { brake: 1 }); run(phys, 6);
    const r3 = snap();
    Object.assign(car.controls, { brake: 0, throttle: 1 }); run(phys, 3);
    const r4 = snap();
    check('rolling backward: request stays pending (reversing + coasting), commits once stopped, reverse still drives in L',
      rv < -1 && r1.range === 'H' && r1.pendingRange === 'L' && r2.speed < -SHIFT_SPEED && r2.pendingRange === 'L' && r3.range === 'L' && r3.pendingRange === null
        && r4.gear === 'R' && r4.range === 'L' && r4.speed < -0.5,
      `reverse ${n2(rv)} m/s: ${r1.range} (pending ${r1.pendingRange}), coast ${n2(r2.speed)} m/s pending ${r2.pendingRange} -> stopped ${r3.range}, then ${n2(r4.speed)} m/s in ${r4.gear}/${r4.range}`);
    car.reset();
    check('reset(): back to H, no pending shift', car.range() === 'H' && car.getState().pendingRange === null && car.controls.range === 'H');
  }
  const top = {}, brake = {}, mult = {};
  for (const r of ['L+', 'L', 'L-']) {
    const { phys, car } = make(flat('rock'));
    Object.assign(car.controls, { throttle: 1, fourWD: true, range: r });
    mult[r] = car.gearScale(r) / car.gearScale('H');
    let v = 0; run(phys, 6, () => { v = Math.max(v, car.getState().speed); }); top[r] = v * 3.6;
    car.controls.throttle = 0; const v0 = car.getState().speed; run(phys, 0.15); brake[r] = (v0 - car.getState().speed) / 0.15 / Math.max(v0, 0.1);
  }
  const f = o => ['L+', 'L', 'L-'].map(r => `${r} ${n2(o[r])}`).join(', ');
  check('governed top speeds L+ ~15, L ~10, L- ~5 km/h', Math.abs(top['L+'] - 15) < 3 && Math.abs(top.L - 10) < 2.5 && Math.abs(top['L-'] - 5) < 1.5, `${f(top)} km/h`);
  check('torque multiplier rises L+ < L < L-', mult['L+'] < mult.L && mult.L < mult['L-'], f(mult));
  check('L- has the strongest engine braking (relative decel)', brake['L-'] > brake.L && brake.L > brake['L+'] * 0.9, f(brake));
  // fuel: smooth crawl vs redline wheelspin (mud, tethered behind -> spins in place)
  const burn = (thr, range, secs, surface = 'rock') => {
    const { phys, car } = make(flat(surface));
    Object.assign(car.controls, { throttle: thr, fourWD: true, range });
    let dryAt = -1; run(phys, secs, i => { if (dryAt < 0 && car.getState().hardcore.dry) dryAt = i / 60; });
    return { car, phys, fuel: car.getState().fuel, dryAt };
  };
  const crawl = burn(0.3, 'L', 30), rev = burn(1, 'H', 30, 'ice');
  check('fuel: full-gas ice wheelspin burns > 4x a 30 % crawl in L', (1 - rev.fuel) > 4 * (1 - crawl.fuel), `used ${n3(1 - rev.fuel)} vs ${n3(1 - crawl.fuel)} in 30 s`);
  const dry = burn(1, 'H', 300, 'ice');
  check('fuel: redline wheelspin empties the tank (< 4 min: a whole level of mashing ~half a tank) -> out-of-fuel event, no torque',
    dry.dryAt > 0 && dry.dryAt < 240 && ev(dry.phys, 'out-of-fuel').length === 1 && Math.abs(dry.car.engineTorque) < 1e-9, `dry after ${n2(dry.dryAt)} s`);
  dry.car.recover(); const kept = dry.car.getState().fuel; dry.car.reset();
  check('fuel: recover keeps the empty tank, reset refuels', kept === 0 && dry.car.getState().fuel === 1);
  // level start tank + jerry cans: a can is collected only when the truck's footprint passes over it (trail z)
  {
    const lane = z => {
      const { phys, car } = make(flat('rock', { tank: 0.4, cans: [{ x: 8, y: 0, z: 1.5, fuel: 0.2 }] }), { x: 0, y: 0, z });
      const f0 = car.getState().fuel;
      Object.assign(car.controls, { fourWD: true, range: 'L+' });
      run(phys, 6, () => { car.controls.throttle = car.getState().speed > 1.5 ? 0 : 0.5; });
      const s = car.getState(), e = ev(phys, 'fuel-can');
      car.reset();
      return { f0, f: s.fuel, x: s.x, got: e.length, amt: e[0] ? e[0].amount : 0, taken: phys.getDynamic().cans[0], refill: car.getState().fuel };
    };
    const over = lane(0.9), by = lane(-0.9);
    check('fuel: level tank sets the start fill; driving over a jerry can adds its fuel (fuel-can event), passing beside it does not; reset refills to the level tank',
      over.f0 === 0.4 && over.got === 1 && Math.abs(over.amt - 0.2) < 1e-9 && over.taken && over.f > 0.55 && by.got === 0 && by.x > 8 && by.f < 0.4 && over.refill === 0.4,
      `start ${n2(over.f0)}, over: ${over.got} can +${n2(over.amt)} -> ${n3(over.f)}; beside (x ${n2(by.x)}): ${by.got} can -> ${n3(by.f)}; reset ${n2(over.refill)}`);
    const small = clone(CFG); small.fuel = { tank: 0.5 };
    const used = cfg => { const { phys, car } = make(flat('rock'), { x: 0, y: 0 }, cfg); Object.assign(car.controls, { range: 'L', throttle: 1 }); run(phys, 10); return 1 - car.getState().fuel; };
    const u1 = used(CFG), u2 = used(small);
    check('fuel: a truck fuel.tank of 0.5 burns its tank twice as fast', Math.abs(u2 / u1 - 2) < 0.02, `${n3(u1)} vs ${n3(u2)} per 10 s`);
  }
});

// ---- 13. soft terrain: wheelspin digs ruts -------------------------------------------------------
test('soft terrain: wheelspin digs ruts into mud (collider sinks), gentle crawl stays on top', check => {
  const tethered = (level = flat('mud')) => {        // rope to a post behind the truck: full throttle spins in place
    const { phys, car } = make(level);
    run(phys, 1);
    car.winch.attach({ x: -6, y: 0.9 });
    Object.assign(car.controls, { throttle: 1, fourWD: true, range: 'L+' });
    const depth = [], slip = [];
    let minBot = 9;
    run(phys, 5, i => {
      const s = car.getState();
      for (const w of s.wheels) minBot = Math.min(minBot, w.y - w.radius);
      slip.push(mean(s.wheels.map(w => w.slip)));
      if (i % 60 === 59) depth.push(phys.maxRutDepth());
    });
    return { phys, car, depth, minBot, slip: mean(slip), max: phys.maxRutDepth() };
  };
  const a = tethered();
  check('held in place on mud at full throttle: high wheelspin', a.slip > 2 && !a.car.winch.broken, `mean slip ${n2(a.slip)} m/s`);
  check('rut depth increases every second under sustained wheelspin', a.depth[0] > 0.03 && a.depth.every((d, i) => !i || d > a.depth[i - 1]),
    `max depth ${a.depth.map(n3).join(' -> ')} m`);
  check('the truck digs itself into a deep rut, capped at maxDepth', a.max > 0.25 && a.max <= RUT.maxDepth + 1e-9, `${n3(a.max)} m (cap ${RUT.maxDepth} m)`);
  check('collider follows the rut: tyres sink below the original surface', a.minBot < -0.2, `lowest tyre bottom ${n3(a.minBot)} m`);
  check('the rut sits under the spinning wheels', a.car.getState().wheels.every(w => a.phys.rutDepthAt(w.x) > 0.15),
    a.car.getState().wheels.map(w => n2(a.phys.rutDepthAt(w.x))).join('/'));
  const b = tethered();
  check('rutting is deterministic', b.phys.hash() === a.phys.hash() && b.max === a.max, `${a.phys.hash()} vs ${b.phys.hash()}`);
  // gentle crawl: speed-capped 30 % throttle keeps the tread below the dig threshold
  {
    const { phys, car } = make(flat('mud'));
    Object.assign(car.controls, { fourWD: true, range: 'L+' });
    const x0 = car.getState().x;
    run(phys, 8, () => { car.controls.throttle = car.getState().speed > 1.2 ? 0 : 0.3; });
    check('gentle crawl on mud stays on top (no rut)', phys.maxRutDepth() < 0.01 && car.getState().x - x0 > 5,
      `depth ${n3(phys.maxRutDepth())} m, dx ${n2(car.getState().x - x0)} m`);
  }
  const rock = tethered(flat('rock')), off = tethered(flat('mud', { surfaces: { mud: { dig: 0 } } }));
  check('rock never ruts', rock.phys.ruts.length === 0 && rock.max === 0);
  check('surface dig: 0 disables rutting', off.phys.ruts.length === 0 && off.max === 0 && off.slip > 2, `slip ${n2(off.slip)} m/s`);
});

// ---- 14. 2.5D lateral steering: trail z, yaw, rotated wheel sampling, split-mu, independent lane ruts -------------
test('lateral steering: z / yaw across the trail, wheels sampled at their rotated (x, z), split-mu and lane ruts', check => {
  const D2R = Math.PI / 180, T = TRAIL, h = s => Math.abs(s.wheels[1].lz);
  // (a) steer right while rolling (from the left side of the trail): heading builds to the ±20° limit, z moves right, left side
  // leads; release -> self-centres
  {
    const { phys, car } = make(flat(), { x: 0, y: 0, z: -T.half });
    run(phys, 1);
    Object.assign(car.controls, { throttle: 0.5, fourWD: true, range: 'L+', steer: 1 });
    let maxYaw = 0, lead = 0, zMax = -9, front = 0;
    run(phys, 1.6, () => {
      const s = car.getState();
      maxYaw = Math.max(maxYaw, s.yaw); zMax = Math.max(zMax, s.z); front = Math.max(front, s.wheels[0].steer);
      if (s.yaw > 19.9 * D2R) lead = s.wheels[0].x - s.wheels[1].x;
    });
    const s1 = car.getState(), ys = s1.yaw, x1 = s1.x;
    check('steer right: heading reaches the 20° limit (never beyond)', maxYaw > 19.9 * D2R && maxYaw <= 20 * D2R + 1e-9, `max yaw ${n2(maxYaw / D2R)}°`);
    check('steer right: truck moves right across the trail (z +)', s1.z > 0.2 && zMax - s1.z < 1e-9, `z ${n3(s1.z)} m after ${n2(x1)} m`);
    check('front wheels turn, rears do not', front > 25 * D2R && s1.wheels[2].steer === 0, `front ${n2(front / D2R)}°`);
    check('yawed: left wheels lead the right ones by ~2·halfTrack·sin(yaw)', Math.abs(lead - 2 * h(s1) * Math.sin(20 * D2R)) < 0.06,
      `FL-FR dx ${n3(lead)} m, expected ${n3(2 * h(s1) * Math.sin(20 * D2R))} m`);
    check('wheel z = truck z + rotated offsets', Math.abs(s1.wheels[0].z - (s1.z + CFG.axles[0].mount[0] * Math.sin(ys) - h(s1) * Math.cos(ys))) < 1e-9,
      `FL z ${n3(s1.wheels[0].z)} m`);
    car.controls.steer = 0;
    run(phys, 2.5);
    const s2 = car.getState();
    check('released: heading self-centres while rolling', Math.abs(s2.yaw) < 1.5 * D2R && Math.abs(s2.steer) < 3 * D2R, `yaw ${n2(s2.yaw / D2R)}°, steer ${n2(s2.steer / D2R)}°`);
    // held into the edge: no invisible clamp -- the berm only eases the nose, the truck climbs it, its tyres roll past the rim
    // and lose their ground (edgeDrop), it tips about the inner tyres from rest (no snap / pop) and falls -> 'fell-off'
    car.controls.steer = 1;
    let zm = 0, berm = false, firstOff = -1, rel = -1, upMax = 0, roll0 = null, fell = null;
    phys.events.length = 0;
    run(phys, 7, () => {
      const s = car.getState(), vy = car.chassis.getLinearVelocity().y;
      zm = Math.max(zm, s.z); berm = berm || s.wheels.some(w => w.surface === 'berm');
      if (firstOff < 0 && s.offEdge) firstOff = phys.time;
      if (firstOff >= 0 && rel < 0) upMax = Math.max(upMax, vy);
      if (rel < 0 && car.offEdge) rel = phys.time;                  // chassis centre past the rim: ground contact released
      if (roll0 === null && Math.abs(s.roll) > 1e-6) roll0 = Math.abs(s.roll);
      for (const e of phys.events.splice(0)) if (e.type === 'fell-off' && !fell) fell = { t: phys.time, side: e.side };
    });
    check('held into the edge: climbs the berm and passes the rim (no clamp at TRAIL.half)', berm && zm > T.rim - h(car.getState()) && firstOff > 0,
      `z max ${n3(zm)} m (rim ${T.rim}), berm ${berm}, first tyre off at ${n2(firstOff)} s`);
    check('going over: no pop -- chassis never kicked upward while the tyres lose ground (vy < 1 m/s), roll starts from rest',
      upMax < 1 && roll0 !== null && roll0 < 0.01, `max vy ${n3(upMax)} m/s, first roll step ${n3((roll0 ?? 0) / D2R)}°, released at ${n2(rel)} s`);
    check('falls: fell-off event (right side) once the chassis is FALL_DEPTH under the trail', !!fell && fell.side === 'R' && car.getState().fell,
      fell ? `at ${n2(fell.t)} s, chassis y ${n2(car.getState().y)} m` : 'no event');
    car.controls.steer = 0; car.controls.throttle = 0;
    car.recover();
    run(phys, 2);
    const sr = car.getState();
    check('recover() after a fall: back on the trail, level, contacts restored', Math.abs(sr.z) <= T.half && !sr.fell && !sr.offEdge && sr.roll === 0 && sr.wheels.every(w => w.grounded),
      `z ${n3(sr.z)} m, y ${n2(sr.y)} m, grounded ${sr.wheels.filter(w => w.grounded).length}/4`);
  }
  // (a2) edge lip: 80 % steer toward the edge never carries a tyre over the crest (the truck slides along it); full lock held
  //      into it wears through in ~lipTime and the truck goes over
  {
    const { phys, car } = make(flat(), { x: 0, y: 0, z: 0 });
    run(phys, 1);
    Object.assign(car.controls, { throttle: 0.5, fourWD: true, range: 'L+', steer: 0.8 });
    let wz = 0;
    run(phys, 6, () => { for (const w of car.getState().wheels) wz = Math.max(wz, w.z); });
    check('edge lip: 80 % steer into the edge never carries a tyre over the crest', wz <= T.lipZ + 1e-6 && !car.getState().offEdge,
      `max tyre z ${n3(wz)} m (crest ${T.lipZ})`);
    car.controls.steer = 1;
    const t0 = phys.time; let tOver = -1;
    run(phys, 4, () => { if (tOver < 0 && car.getState().wheels.some(w => w.z > T.lipZ + 0.02)) tOver = phys.time - t0; });
    check('edge lip: full lock held into it wears through (~lipTime s) and a tyre crosses', tOver > 0.5 && tOver < 2.5, `crossed after ${n2(tOver)} s`);
  }
  // (b) split-mu by position: ice on the left half of a 10° climb, open diffs. Centred: left tyres on ice -> stuck.
  //     Moved over to the right (z = TRAIL.soft): all four tyres sample z > 0 (rock) -> climbs on open diffs.
  const climb = z => {
    const { phys, car } = make(slope(10, 'rock', { zones: [{ surface: 'ice', side: 'L' }] }), { ...onSlope(10, 30), z });
    car.controls.brake = 1;
    run(phys, 1.5);
    const x0 = car.getState().x;
    Object.assign(car.controls, { brake: 0, throttle: 1, fourWD: true, range: 'L+' });
    const dW = [];
    run(phys, 4, () => { const w = car.getState().wheels; dW.push(Math.abs(w[2].omega - w[3].omega)); });
    const s = car.getState();
    return { dx: s.x - x0, dW: mean(dW), surf: s.wheels.map(w => w.surface).join('/'), z: s.wheels.map(w => n2(w.z)).join('/') };
  };
  const mid = climb(0), right = climb(T.soft);
  check('centred: left tyres on ice, open diff spins them, no climb', mid.dx < 0.5 && mid.dW > 3 && /^ice\/rock\/ice\/rock$/.test(mid.surf),
    `${mid.surf} at z ${mid.z}, dx ${n2(mid.dx)} m, |wL-wR| ${n2(mid.dW)} rad/s`);
  check('moved right: every tyre on rock, open diffs climb', right.dx > 4 && right.dW < 0.5 && /^rock\/rock\/rock\/rock$/.test(right.surf),
    `${right.surf} at z ${right.z}, dx ${n2(right.dx)} m`);
  // (c) angled log: yawed 20° the left front tyre meets the log well before the right front one
  {
    const { phys, car } = make(flat('rock', { logs: [{ x: 7, y: 0.25, r: 0.25 }] }), { x: 0, y: 0, z: -1.1 });
    run(phys, 1.6);
    Object.assign(car.controls, { throttle: 0.6, fourWD: true, range: 'L+', lockF: true, lockR: true, steer: 1 });
    let tL = -1, tR = -1, xs = null, maxDy = 0;
    run(phys, 6, i => {
      const s = car.getState(), hit = w => w.contacts.some(c => c.surface === 'wood');
      if (tL < 0 && hit(s.wheels[0])) { tL = i; xs = { yaw: s.yaw, dx: s.wheels[0].x - s.wheels[1].x }; }
      if (tR < 0 && hit(s.wheels[1])) tR = i;
      if (tL >= 0 && tR < 0) maxDy = Math.max(maxDy, s.wheels[0].y - s.wheels[1].y);
      if (tR >= 0 && i > tR + 30) return false;
    });
    check('angled log: front-left tyre climbs it first, front-right later', tL >= 0 && tR > tL + 6 && maxDy > 0.1,
      `FL hit at ${n2(tL / 60)} s (yaw ${xs ? n2(xs.yaw / D2R) : '-'}°, FL ahead ${xs ? n3(xs.dx) : '-'} m), FR at ${n2(tR / 60)} s, FL up to ${n3(maxDy)} m above FR`);
  }
  // (d) ruts per lane: tethered wheelspin on mud whose right half is firm rock -> deep left ruts, untouched right lanes + centre
  {
    const { phys, car } = make(flat('mud', { zones: [{ surface: 'rock', side: 'R' }] }));
    run(phys, 1);
    car.winch.attach({ x: -6, y: 0.9 });
    Object.assign(car.controls, { throttle: 1, fourWD: true, range: 'L+' });
    run(phys, 4);
    const s = car.getState(), at = (w, z) => phys.rutDepthAt(w.x, z);
    const L = [s.wheels[0], s.wheels[2]].map(w => at(w, w.z)), R = [s.wheels[1], s.wheels[3]].map(w => at(w, w.z));
    check('left (mud) lanes rut under the spinning left tyres', L.every(d => d > 0.15), L.map(n3).join('/'));
    check('right (rock) lanes and the centre hump stay untouched', R.every(d => d === 0) && at(s.wheels[0], 0) < 0.005,
      `R ${R.map(n3).join('/')} m, centre ${n3(at(s.wheels[0], 0))} m`);
    check('left tyres sink into their own ruts, right ones stay on top', s.wheels[0].y < s.wheels[1].y - 0.12 && s.wheels[2].y < s.wheels[3].y - 0.12,
      `FL ${n3(s.wheels[0].y)} vs FR ${n3(s.wheels[1].y)} m`);
  }
  // (e) partial-width boulder (rock log z0..z1 = left lane) and a tall left rock shelf (zone dy): walls you steer around
  {
    const BX = 9, rk = { x: BX, y: 0.9, r: 0.85, surface: 'rock', density: 2600, length: 1.6, embed: 0.1, z0: -3, z1: -0.45 };
    const lane = (z, ctl, secs = 14, level = flat('rock', { logs: [rk] })) => {
      const { phys, car } = make(level, { x: 0, y: 0, z });
      run(phys, 1.6);
      Object.assign(car.controls, ctl);
      let yMax = -9, edge = 9, low = 9, zIn = 9;
      run(phys, secs, () => {
        const s = car.getState();
        yMax = Math.max(yMax, s.y); low = Math.min(low, ...s.wheels.map(w => w.y));
        if (s.x > BX && s.x < 18) zIn = Math.min(zIn, s.z);
        for (const w of [s.wheels[0], s.wheels[2]]) if (Math.abs(w.x - BX) < rk.r + w.radius) edge = Math.min(edge, w.z - car.tyreWidth / 2);
        if (car.controls.steerAt != null && s.x > car.controls.steerAt) car.controls.steer = s.z > -0.6 ? -1 : 0;   // turn in, then hold the line
      });
      return { s: car.getState(), yMax, edge, low, zIn };
    };
    const crawlC = { throttle: 0.6, fourWD: true, lockF: true, lockR: true, range: 'L' };
    const straight = lane(0, crawlC), beside = lane(0.9, crawlC);
    check('boulder in the left lane: a centred crawl stalls against it', straight.s.x < BX - 0.5, `x ${n2(straight.s.x)} m (boulder at ${BX} m)`);
    check('steered right of it (z 0.9) the truck rolls past untouched', beside.s.x > BX + 6 && beside.edge > rk.z1, `x ${n2(beside.s.x)} m, left tyre edge >= ${n3(beside.edge)} m`);
    const bash = lane(0, { throttle: 1, fourWD: true, lockF: true, lockR: true }, 8);
    check('4H full gas: no wheelie up its flank (stopped, chassis not launched)', bash.s.x < BX + 0.5 && bash.yMax < 1.6, `x ${n2(bash.s.x)} m, chassis y max ${n3(bash.yMax)} m`);
    const wall = lane(0.9, { ...crawlC, steerAt: BX - 3 });
    check('steering into it alongside: it is a side wall, tyres never slide into its lane', wall.edge > rk.z1 - 0.02 && wall.s.x > BX + 4,
      `left tyre edge >= ${n3(wall.edge)} m (wall ${rk.z1}), x ${n2(wall.s.x)} m`);
    const shelf = lane(0.9, { ...crawlC, steerAt: 6 }, 14, flat('rock', { zones: [{ x0: 7, x1: 20, surface: 'rock', z1: -0.45, dy: 0.9 }] }));
    check('tall shelf (dy 0.9) beside the lane: steering into it holds z, no tyre falls through its collider; past it the truck turns in',
      shelf.zIn > 0.3 && shelf.low > 0.2 && shelf.s.x > 18 && shelf.s.z < 0, `z >= ${n3(shelf.zIn)} m alongside, lowest hub ${n3(shelf.low)} m, then z ${n3(shelf.s.z)} m at x ${n2(shelf.s.x)} m`);
  }
});

// ---- tilt steering: gravity-vector roll (engine/tilt.js, pure; hud.js wires devicemotion / deviceorientation) ------
const { screenRoll, tiltSteer, TiltFilter, upFromEuler, tiltRoll, TILT } = await import('../engine/tilt.js');
test('tilt: gravity roll is screen-relative, recline-independent, either landscape side; filter + curve + iOS sign', check => {
  const D = Math.PI / 180;
  // device-frame reaction-to-gravity vector (m/s²) of a phone at screen angle a, rolled r (clockwise) in its plane, reclined p
  const dev = (r, p, a, g = 9.81) => {
    const sx = -Math.sin(r * D) * Math.cos(p * D), sy = Math.cos(r * D) * Math.cos(p * D), sz = Math.sin(p * D), c = Math.cos(a * D), s = Math.sin(a * D);
    return [g * (sx * c + sy * s), g * (-sx * s + sy * c), g * sz];
  };
  let worst = 0, centre = 0;
  for (const a of [90, 270]) for (const p of [0, 20, 45, 60, 65]) {
    for (const r of [-30, -15, -5, 5, 15, 30]) worst = Math.max(worst, Math.abs(screenRoll(...dev(r, p, a), a) - r));
    centre = Math.max(centre, Math.abs(screenRoll(...dev(0, p, a), a)));
  }
  check('roll exact for 0-65° recline on both landscape sides', worst < 0.01 && centre < 1e-9, `worst error ${worst.toExponential(1)}°, centre ${centre.toExponential(1)}°`);
  const flatEdge = screenRoll(-Math.sin(8 * D), 0, Math.cos(8 * D), 0);
  check('lying flat: tilting the edges reads the edge tilt (no atan2 blow-up)', Math.abs(flatEdge - 8) < 0.01, `${n3(flatEdge)}°`);
  const eu = Math.abs(tiltRoll(30, -12, 90) - screenRoll(...upFromEuler(30, -12), 90));
  check('deviceorientation fallback shares the pipeline', eu < 1e-9 && Number.isFinite(tiltRoll(90, 0, 0)), `diff ${eu}`);
  const st = [0, 2, TILT.dead, 3, 6, 12, 20, 25, 40].map(d => tiltSteer(d, 25));
  const mono = st.every((v, i) => !i || v >= st[i - 1]);
  check('deadzone + curve: 0 inside the deadzone, smooth start, monotonic, full lock at tiltMax, odd',
    st[2] === 0 && st[3] > 0 && st[3] < 0.01 && mono && st[7] === 1 && st[8] === 1 && tiltSteer(-12, 25) === -st[5],
    st.map(v => v.toFixed(3)).join(' '));
  const run2 = (sgn, jitterAmp = 0) => {                       // 2 s held at 15° then a 0.25 s turn to -20°, 60 Hz, seeded jitter
    const f = new TiltFilter(); let seed = 7, t = 0, rest = [], lag = null;
    const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647 - 0.5) * 2 * jitterAmp;
    for (let i = 0; i < 180; i++, t += 1 / 60) {
      const r = i < 120 ? 15 : Math.max(-20, 15 - (i - 120) * 35 / 15);
      const v = dev(r, 40, 90).map(x => sgn * x + rnd());
      const out = f.push(...v, t, 90, true);
      if (i >= 90 && i < 120) rest.push(out);
      if (lag == null && i > 120 && out < -18) lag = (i - 135) / 60;
    }
    return { mean: mean(rest), sd: Math.sqrt(mean(rest.map(x => (x - mean(rest)) ** 2))), lag, inv: f.inv };
  };
  const an = run2(1), ios = run2(-1), noisy = run2(1, 0.6), raw = 0.6 / Math.sqrt(3) / 9.81 / Math.cos(40 * D) / D;
  check('Android sign kept, iOS (gravity, not its reaction) detected + flipped: same roll', an.inv === 1 && ios.inv === -1 && Math.abs(an.mean - 15) < 0.05 && Math.abs(ios.mean - 15) < 0.05,
    `android ${n3(an.mean)}°, ios ${n3(ios.mean)}°`);
  check('1€ filter: hand jitter (±0.6 m/s²) damped >= 2.5x at rest, a quick 35° turn lands within 0.15 s', noisy.sd < raw / 2.5 && Math.abs(noisy.mean - 15) < 1 && an.lag != null && an.lag < 0.15,
    `rest sd ${n3(noisy.sd)}° (raw ~${n3(raw)}°), turn lag ${an.lag == null ? '-' : n3(an.lag)} s`);
  const f2 = new TiltFilter(), r0 = f2.push(0, 0, 0, 0, 90), r1 = f2.push(NaN, 1, 1, 0.1, 90);
  check('bad samples (zero / NaN) are ignored', r0 === 0 && r1 === 0 && f2.v === null, `${r0} ${r1}`);
});

// ---- summary -----------------------------------------------------------------------------------
const failed = results.filter(r => !r.ok);
out('');
out(`${results.length - failed.length}/${results.length} tests passed${failed.length ? ' — FAILED: ' + failed.map(f => f.name).join('; ') : ''}`);
if (isNode) process.exitCode = failed.length ? 1 : 0;
export { results };
