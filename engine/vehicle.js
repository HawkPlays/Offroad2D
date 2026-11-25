// engine/vehicle.js — Offroad2D vehicle model. NO rendering / DOM code in this file.
// 2.5D layout: every axle has a LEFT and a RIGHT wheel body that overlap in the side view but have
// their own suspension, contact, friction, angular velocity and lane collider (physics addLane).
// Lateral layer: the truck has a trail position z (+ = right / near) and heading yaw (+ = nose right) on top of the
// planck x-y plane. Wheels sit at their rotated (x, z): the WheelJoint anchors slide along chassis x by
// lx(cos yaw - 1) - lz sin yaw (left side leads when turning right), and each wheel's lane follows its own z.
import { planck, CAT, LANES, TRAIL } from './physics.js';

const Vec2 = planck.Vec2;
const DIR = -1;                 // forward (+x) rolling == negative angular velocity
const FL = 0, FR = 1, RL = 2, RR = 3;
const STOP_ARM = 3;             // lever length used by the travel-limit rope joints (m)
const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
const DEG = Math.PI / 180;

// Steering (2.5D, controls.steer -1 left .. +1 right; a vehicle JSON may tune any key via `steer {}`):
//   maxYaw    heading limit (rad)        maxSteer  front-wheel angle at full lock (rad)     rack  wheel-angle slew (rad/s)
//   center    released: front wheels counter-steer at -center·yaw, the heading self-centres while rolling
//   berm      past TRAIL.soft, heading into the berm turns the nose back toward straight (rad/s per m/s at TRAIL.half)
// Kinematic bicycle on the trail speed vx: yaw' = vx/cos(yaw)·tan(steer)/wheelbase, z' = vx·tan(yaw).
export const DEFAULT_STEER = { maxYaw: 20 * DEG, maxSteer: 32 * DEG, rack: 3, center: 2, berm: 1.5 };

// ---------------------------------------------------------------------------------------------
// Hardcore rules (always on; a vehicle JSON may tune any key via `hardcore {}`).
//   bind   = throttle * power * min(1 + |v| / bindSpeed, bindCap) * grip(contact mu in bindMu range, min of the pair)
//   axle   : locked diff on a driven axle; stress grows by (bind - axleFree) * axleGain / s, snaps at 1
//   xfer   : 4WD centre lock (both axles gripping); heat at 1 -> forced 2WD until it cools to xferRecover
//   engine : rpm > engineHotRpm * redline at > 70 % throttle heats it; at 1 power x enginePowerHot
//   torque converter (High range / 4H, automatic): the engine never stalls. With the driven tread slower than
//            bogSpeed under > 25 % throttle the converter sits at stall: engine rpm clamps to tcStallRpm and wheel
//            torque is capped at the grade-holding torque of a bogAngle° slope (mass * g * sin * radius). The cap
//            blends quadratically to full 4H torque at lockSpeed (lock-up), so a fast run-up carries a 25-35° climb
//            until a bump knocks the tread speed back down. Bogging heats transTemp (transGain / s * throttle * bog,
//            x transSteepGain above transSteepDeg of climb); at 1 the 4H drive torque slips to 0 until it cools
//            below transRecover (cools faster off the gas / in Low, where the converter is unloaded). Back-driven
//            (rolling back under gas) the stall torque rises up to (1 + tcRollback) x, so it creeps instead of running away.
//   Low (4L, drivetrain keys): torque = lowMult x 4H (else lowRatio * efficiency), top speed governed to lowSpeed
//            (default 0.35) x the 4H top speed, throttle-lift engine braking x lowBrake (default 1).
//   ignition: every spawn / reset() starts with the engine OFF (hc.off). The key (controls.ignition = press counter, each
//            change = one key turn, or ignition()) cranks an off / stalled engine for crankTime s, then it runs; a running
//            engine is switched off. An empty tank cranks but never catches ('no-start'). Gas never restarts it.
//   stall  : lug meter hc.lug (+lugGain x load / s, -lugCool / s) -> 1 stalls (hc.stalled, 'stall' {cause}): Low range
//            (direct clutch, no converter) back-driven against the gear faster than stallBack m/s (gas resists), or
//            power-braking (> 50 % brake + > 50 % gas); 4H with the transmission boiled (transHot)
//            and > 70 % gas into a bog (x transHotLug). Running dry kills it too. Only a key crank restarts it.
//   damage : chassis contact closing speed > bodyHit m/s, wheel hit across the suspension axis > wheelHit,
//            along it > dropHit; rollover onto the roof adds rollDamage / rollShockDamage once.
//            body damage cuts power (bodyPowerLoss); shock damage softens springs, kills damping, warps travel,
//            sags the corner (shockSag) and makes its spring / damper flutter (shockFlutter, deterministic noise).
//   misfire: throttle > 70 % with max(body, engine heat) > misfireFrom: ~0.1 s slots drop power by misfireCut
//            (chance up to misfireMax); body damage also lowers the engine-hot rpm (engineHotBody) and heats faster.
//   fast lockers: above lockerFastSpeed on grip >= lockerFastGrip, axle stress builds lockerFastGain x faster.
// ---------------------------------------------------------------------------------------------
export const DEFAULT_HARDCORE = {
  bindMu: [0.7, 1.0], bindSpeed: 3, bindCap: 3.5,
  axleFree: 0.8, axleGain: 0.12, axleCool: 0.08,
  xferFree: 1.1, xferGain: 0.05, xferCool: 0.06, xferRecover: 0.4,
  engineHotRpm: 0.88, engineGain: 0.045, engineCool: 0.03, engineRecover: 0.55, enginePowerHot: 0.4,
  tcStallRpm: 2250, bogAngle: 22, bogSpeed: 0.6, lockSpeed: 5, tcRollback: 0.6,
  transGain: 0.18, transSteepDeg: 25, transSteepGain: 1.5, transCool: 0.1, transRecover: 0.7,
  crankTime: 0.9, lugGain: 0.8, lugCool: 1.0, stallBack: 0.5, stallBackRef: 1.0, transHotLug: 0.25,
  bodyHit: 2.2, bodyScale: 0.08, wheelHit: 3.0, wheelScale: 0.08, dropHit: 6.5, dropScale: 0.08,
  rollDamage: 0.12, rollShockDamage: 0.06,
  bodyPowerLoss: 0.5, shockStiffnessLoss: 0.3, shockDampingLoss: 0.9, shockWarp: 0.4, shockSag: 0.35, shockFlutter: 0.35,
  misfireFrom: 0.45, misfireMax: 0.35, misfireCut: 0.7, engineHotBody: 0.12,
  lockerFastSpeed: 6.94, lockerFastGrip: 0.8, lockerFastGain: 1.4,
};
/** Deterministic hash noise in [-1, 1] (replaces Math.random so replays stay bit-identical). */
const noise = (a, b) => { const v = Math.sin(a * 12.9898 + b * 78.233) * 43758.5453; return 2 * (v - Math.floor(v)) - 1; };
// Transfer-case ranges: speed = x lowSpeed governor (L+ ~15, L ~10, L- ~5 km/h), torque = x lowMult, brake = x lowBrake.
export const RANGES = ['H', 'L+', 'L', 'L-'];
export const SHIFT_SPEED = 0.3;   // m/s: a range change requested faster than this waits as pendingRange until the truck stops
export const RANGE_CFG = { 'L+': { speed: 1.0, torque: 1.0, brake: 1.0 }, L: { speed: 0.66, torque: 1.25, brake: 1.3 }, 'L-': { speed: 0.33, torque: 1.6, brake: 1.8 } };
// Fuel (per truck `fuel {}`): burn/s = idle + rev * (rpm/redline)^2 * throttle + spin * min(wheelspin / spinRef, 1) * throttle
export const DEFAULT_FUEL = { idle: 0.0004, rev: 0.008, spin: 0.009, spinRef: 3 };
const freshDamage = () => ({
  axle: [0, 0], axleBroken: [false, false], xfer: 0, xferHot: false, engine: 0, engineHot: false,
  off: true, stalled: false, lug: 0, bog: 0, trans: 0, transHot: false, crank: 0, body: 0, shock: [0, 0, 0, 0], rolled: false, misfire: false, fuel: 1, dry: false,
});

// ---------------------------------------------------------------------------------------------
// DrivelineJoint: engine motor + transfer case + axle differentials solved INSIDE the planck
// velocity solver, so torque split reacts to tyre friction within the same step.
//   shaft speed (forward +) = DIR * (mean(driven wheel w) - chassis w)
//   open diff   : drive impulse is split equally between the two wheels of an axle
//   locked diff : extra constraint wL == wR      (moves torque to the wheel with grip)
//   4WD (locked centre): extra constraint mean(front) == mean(rear)
// ---------------------------------------------------------------------------------------------
class DrivelineJoint extends planck.Joint {
  constructor(chassis, wheels) {
    super({ collideConnected: false }, chassis, wheels[RL]);
    this.m_type = 'o2d-driveline';
    this.ch = chassis;
    this.wh = wheels;                 // [FL, FR, RL, RR] bodies
    this.fourWD = false;
    this.lockF = false;
    this.lockR = false;
    this.broken = [false, false];     // snapped axle shafts [front, rear]: that axle gets no drive
    this.target = 0;                  // shaft speed limit / target (forward +, rad/s)
    this.minImp = 0;                  // motor impulse clamp (forward +, N*m*s)
    this.maxImp = 0;
    this.initImp = 0;                 // impulse applied up-front (full engine torque)
    this.imp = [0, 0, 0, 0];          // per-wheel net driveline impulse this step (forward +)
    this.motorImp = 0;
    this.drv = [];                    // indices of the wheels currently connected to the engine
    this._it = 0;
    this._K = 1;
  }

  getAnchorA() { return this.m_bodyA.getWorldCenter(); }
  getAnchorB() { return this.m_bodyB.getWorldCenter(); }
  getReactionForce() { return Vec2(0, 0); }
  getReactionTorque() { return 0; }
  _reset() {}

  /** Driven wheels for the current transfer-case / breakage state. */
  driven() {
    const d = this.drv;
    d.length = 0;
    if (this.fourWD && !this.broken[0]) d.push(FL, FR);
    if (!this.broken[1]) d.push(RL, RR);
    return d;
  }

  _motor(l) {
    const w = this.wh, d = this.drv, n = d.length;
    if (!n) return;
    const s = DIR * l / n;
    for (let k = 0; k < n; k++) { const i = d[k]; w[i].c_velocity.w += w[i].m_invI * s; this.imp[i] += l / n; }
    this.ch.c_velocity.w -= this.ch.m_invI * DIR * l;   // drive-torque reaction on the chassis
    this.motorImp += l;
  }

  _shaft() {
    const w = this.wh, d = this.drv, n = d.length;
    if (!n) return 0;
    let m = 0;
    for (let k = 0; k < n; k++) m += w[d[k]].c_velocity.w;
    return DIR * (m / n - this.ch.c_velocity.w);
  }

  _lock(a, b) {
    const A = this.wh[a], B = this.wh[b];
    const l = -(A.c_velocity.w - B.c_velocity.w) / (A.m_invI + B.m_invI);
    A.c_velocity.w += A.m_invI * l;
    B.c_velocity.w -= B.m_invI * l;
    this.imp[a] += DIR * l;
    this.imp[b] -= DIR * l;
  }

  _center() {
    const w = this.wh;
    const c = 0.5 * (w[FL].c_velocity.w + w[FR].c_velocity.w) - 0.5 * (w[RL].c_velocity.w + w[RR].c_velocity.w);
    const K = 0.25 * (w[0].m_invI + w[1].m_invI + w[2].m_invI + w[3].m_invI);
    const l = -c / K;
    for (let i = 0; i < 4; i++) {
      const j = i < 2 ? 0.5 : -0.5;
      w[i].c_velocity.w += w[i].m_invI * j * l;
      this.imp[i] += DIR * j * l;
    }
  }

  initVelocityConstraints() {
    const w = this.wh, d = this.driven(), n = d.length;
    this._it = 0;
    this.motorImp = 0;
    this.imp[0] = this.imp[1] = this.imp[2] = this.imp[3] = 0;
    let s = 0;
    for (let k = 0; k < n; k++) s += w[d[k]].m_invI;
    this._K = this.ch.m_invI + (n ? s / (n * n) : 0);
    if (this.initImp !== 0) this._motor(this.initImp);
  }

  solveVelocityConstraints() {
    // Iteration 0 lets tyre contacts absorb the up-front drive impulse first; from iteration 1 the
    // governor trims whatever would push the shaft past its speed limit (free-rev / rev limiter).
    if (this._it++ > 0 && this.drv.length) {
      let l = (this.target - this._shaft()) / this._K;
      const acc = clamp(this.motorImp + l, this.minImp, this.maxImp);
      l = acc - this.motorImp;
      if (l !== 0) this._motor(l);
    }
    const center = this.fourWD && !this.broken[0] && !this.broken[1];
    for (let pass = 0; pass < 2; pass++) {
      if (center) this._center();
      if (this.lockF && !this.broken[0]) this._lock(FL, FR);
      if (this.lockR && !this.broken[1]) this._lock(RL, RR);
    }
  }

  solvePositionConstraints() { return true; }
}

function torqueAt(curve, rpm) {
  if (rpm <= curve[0][0]) return curve[0][1];
  for (let i = 1; i < curve.length; i++) {
    if (rpm <= curve[i][0]) {
      const a = curve[i - 1], b = curve[i];
      return a[1] + (b[1] - a[1]) * (rpm - a[0]) / (b[0] - a[0]);
    }
  }
  return curve[curve.length - 1][1];
}

// ---------------------------------------------------------------------------------------------
export class Vehicle {
  /**
   * @param phys  PhysicsWorld
   * @param cfg   vehicle JSON (see CLAUDE.md schema)
   * @param spawn {x, y, angle?}  y = ground height under the vehicle (tyres are placed just above it)
   */
  constructor(phys, cfg, spawn = { x: 0, y: 0 }) {
    this.phys = phys;
    this.cfg = cfg;
    this.resetControls();
    this.rpm = 0;               // engine OFF on spawn (key ignition cranks it)
    this.engineTorque = 0;      // N*m at the crank (requested)
    this.wheelTorque = 0;       // N*m total at the wheels actually delivered last step (forward +)
    this.reverse = false;
    this.spawn = { x: spawn.x, y: spawn.y, angle: spawn.angle || 0, z: spawn.z || 0 };
    this.steerCfg = { ...DEFAULT_STEER, ...(cfg.steer || {}) };
    this.z = clamp(this.spawn.z, -TRAIL.half, TRAIL.half);   // trail position (m, + = right / near)
    this.yaw = 0;               // heading (rad, + = nose right)
    this.steerAngle = 0;        // front-wheel steer angle (rad)
    this.vz = 0;                // lateral speed (m/s)
    this._winchDown = false;
    this._winchSteps = 0;
    this._winchWasAttached = false;
    this._state = null;
    this.hcCfg = { ...DEFAULT_HARDCORE, ...(cfg.hardcore || {}) };
    this.fuelCfg = { ...DEFAULT_FUEL, ...(cfg.fuel || {}) };
    this.hc = freshDamage();
    this._climb = 0;            // climb angle in the drive direction (deg, + = uphill), set each step
    this._build();
    this._pre = dt => this._preStep(dt);
    this._post = dt => this._postStep(dt);
    phys.preStep.push(this._pre);
    phys.postStep.push(this._post);
  }

  // -- construction ----------------------------------------------------------------------------
  _build() {
    const { phys, cfg } = this, world = phys.world, c = cfg.chassis, wc = cfg.wheels;
    const g = Math.abs(phys.gravity.y);
    const xf = cfg.axles[0].mount[0], xr = cfg.axles[1].mount[0];
    const loadF = c.mass * g * (c.com[0] - xr) / (xf - xr) / 2;      // static load per front wheel
    const loadR = c.mass * g * (xf - c.com[0]) / (xf - xr) / 2;
    this.wheelbase = xf - xr;
    this.halfTrack = cfg.track ? cfg.track / 2 : clamp(0.3 * (xf - xr), 0.75, 1.15);   // wheel-centre |lz| (m)
    this.tyreWidth = clamp(wc.radius * 0.8, 0.32, 0.55);

    // static wheel-centre heights (chassis frame) -> chassis origin height above ground
    this.axleInfo = cfg.axles.map((ax, i) => {
      const s = ax.suspension, n = Math.hypot(ax.axis ? ax.axis[0] : 0, ax.axis ? ax.axis[1] : 1);
      const axis = ax.axis ? [ax.axis[0] / n, ax.axis[1] / n] : [0, 1];
      const p = s.preload || 0;
      const cStatic = clamp((i === 0 ? loadF : loadR) / s.stiffness, p, p + s.travel);
      return { axis, p, cStatic, travel: s.travel };
    });
    const a0 = this.axleInfo[0], m0 = cfg.axles[0].mount;
    const wheelY = m0[1] + a0.axis[1] * (a0.cStatic - a0.p);
    this.rideOffset = wc.radius + 0.03 - wheelY;                      // chassis origin above ground

    const sp = this.spawn;
    this.chassis = world.createBody({
      type: 'dynamic', angle: sp.angle,
      position: Vec2(sp.x - Math.sin(sp.angle) * this.rideOffset, sp.y + Math.cos(sp.angle) * this.rideOffset),
      allowSleep: false, angularDamping: 0.05,
    });
    this.chassis.setUserData({ kind: 'chassis', contacts: [] });
    phys.track(this.chassis);
    for (const s of c.shapes) {
      this.chassis.createFixture({
        shape: new planck.PolygonShape(s.points.map(p => Vec2(p[0], p[1]))),
        density: 0, friction: c.friction, restitution: c.restitution ?? 0.05,
        filterGroupIndex: -1, filterCategoryBits: CAT.VEHICLE, filterMaskBits: 0xffff & ~LANES,
        userData: { bodyMu: c.friction, part: s.kind },
      });
    }
    this.chassis.setMassData({
      mass: c.mass, center: Vec2(c.com[0], c.com[1]),
      I: c.inertia + c.mass * (c.com[0] * c.com[0] + c.com[1] * c.com[1]),
    });

    this.wheels = [];
    cfg.axles.forEach((ax, ai) => {
      const info = this.axleInfo[ai], s = ax.suspension;
      const axisL = Vec2(info.axis[0], info.axis[1]);
      const mountL = Vec2(ax.mount[0], ax.mount[1]);                              // full droop
      const freeL = Vec2(mountL.x - axisL.x * info.p, mountL.y - axisL.y * info.p); // spring free point
      for (const side of ['L', 'R']) {
        const freeW = this.chassis.getWorldPoint(freeL);
        const body = world.createBody({ type: 'dynamic', position: freeW, allowSleep: false, angularDamping: 0.02 });
        const lz = side === 'L' ? -this.halfTrack : this.halfTrack, zw = this.z + lz;
        const lane = phys.addLane(zw, this.tyreWidth);
        const tire = { side, grip: wc.grip ?? 1, z: zw, w: this.tyreWidth, lane };
        body.createFixture({
          shape: new planck.CircleShape(wc.radius), density: 0, friction: 1, restitution: 0,
          filterGroupIndex: -1, filterCategoryBits: CAT.VEHICLE, filterMaskBits: (0xffff & ~(CAT.GROUND | LANES)) | lane.bit,
          userData: { tire },
        });
        body.setMassData({ mass: wc.mass, center: Vec2(0, 0), I: wc.inertia });
        body.setUserData({ kind: 'wheel', contacts: [] });
        phys.track(body);
        const joint = world.createJoint(new planck.WheelJoint(
          { enableMotor: true, maxMotorTorque: 0, motorSpeed: 0, frequencyHz: 4, dampingRatio: 0.5, collideConnected: false },
          this.chassis, body, freeW, this.chassis.getWorldVector(axisL)));
        // hard travel limits (full droop / full bump) as one-sided rope joints along the axis
        const droop = world.createJoint(new planck.RopeJoint({
          maxLength: STOP_ARM, collideConnected: false,
          localAnchorA: Vec2(mountL.x + axisL.x * STOP_ARM, mountL.y + axisL.y * STOP_ARM), localAnchorB: Vec2(0, 0),
        }, this.chassis, body));
        const bump = world.createJoint(new planck.RopeJoint({
          maxLength: STOP_ARM + s.travel, collideConnected: false,
          localAnchorA: Vec2(mountL.x - axisL.x * STOP_ARM, mountL.y - axisL.y * STOP_ARM), localAnchorB: Vec2(0, 0),
        }, this.chassis, body));
        this.wheels.push({
          body, joint, droop, bump, side, axle: ai, name: ax.name + side, sus: s, info, tire, lane, lx: ax.mount[0], lz, z: zw, shift: 0,
          freeL, mountL, axisL, brakeBias: ax.brakeBias ?? 1,
          shockTopL: ax.shockTop ? Vec2(ax.shockTop[0], ax.shockTop[1])
            : Vec2(mountL.x + axisL.x * (s.travel + 0.5), mountL.y + axisL.y * (s.travel + 0.5)),
          linkPivotL: ax.linkPivot ? Vec2(ax.linkPivot[0], ax.linkPivot[1]) : Vec2(mountL.x * 0.3, mountL.y + 0.3),
          travel: 0, speed: 0, load: 0, surface: null, driveTorque: 0, springForce: 0,
        });
      }
    });
    // converter stall torque: just holds a bogAngle° grade from a standstill (wheel torque, N*m)
    this.tcCap = (c.mass + 4 * wc.mass) * g * Math.sin(this.hcCfg.bogAngle * Math.PI / 180) * wc.radius;
    this.driveline = world.createJoint(new DrivelineJoint(this.chassis, this.wheels.map(w => w.body)));
    this.winch = phys.createWinch(this.chassis, cfg.winch);
    this._placeWheels();
  }

  _placeWheels() {
    this._lateralSync();
    for (const w of this.wheels) {
      const c = w.info.cStatic;
      w.body.setTransform(this.chassis.getWorldPoint(Vec2(w.freeL.x + w.shift + w.axisL.x * c, w.freeL.y + w.axisL.y * c)), 0);
      w.body.setLinearVelocity(this.chassis.getLinearVelocity());
      w.body.setAngularVelocity(0);
    }
  }

  /** Respawn: teleport (default = spawn, trail centre) AND repair every hardcore failure / damage. */
  reset(x = this.spawn.x, y = this.spawn.y, angle = this.spawn.angle, z = this.spawn.z) {
    this.repair();
    this.resetControls();
    this.z = clamp(z, -TRAIL.half, TRAIL.half);
    this._teleport(x, y, angle);
  }

  /** Fresh driver inputs: pedals off, 2WD, range H, lockers open; also clears the applied driveline state (no carried-over shift). */
  resetControls() {
    this.controls = { throttle: 0, brake: 0, reverse: false, fourWD: false, lockF: false, lockR: false, low: false, range: 'H', winch: 0, steer: 0, ignition: 0 };
    this._key = 0;                                   // last seen controls.ignition (each change = one key turn)
    if (this.driveline) { this.driveline.fourWD = false; this.driveline.lockF = false; this.driveline.lockR = false; }
    this._range = 'H'; this.pendingRange = null;     // committed transfer-case range + a shift waiting for a stop
  }

  /** Wheel anchors (chassis x) + world z of every wheel / lane from the current z, yaw. */
  _lateralSync() {
    const cy = Math.cos(this.yaw), sy = Math.sin(this.yaw);
    for (const w of this.wheels) {
      const s = w.lx * (cy - 1) - w.lz * sy, d = s - w.shift;
      if (d !== 0) { w.joint.m_localAnchorA.x += d; w.droop.m_localAnchorA.x += d; w.bump.m_localAnchorA.x += d; w.shift = s; }
      w.z = this.z + w.lx * sy + w.lz * cy;
      w.tire.z = w.lane.z = w.z;
      w.lane.x = w.body.getPosition().x;
    }
  }

  /** Steering / heading / trail position (kinematic layer over the planck x-y plane), then sync anchors + lanes. */
  _lateral(dt) {
    const S = this.steerCfg, T = TRAIL, vx = this.chassis.getLinearVelocity().x;
    const st = clamp(this.controls.steer || 0, -1, 1), dir = Math.abs(vx) > 0.3 ? Math.sign(vx) : this.reverse ? -1 : 1;
    const tgt = st ? st * S.maxSteer : clamp(-dir * S.center * this.yaw, -S.maxSteer, S.maxSteer);
    this.steerAngle += clamp(tgt - this.steerAngle, -S.rack * dt, S.rack * dt);
    let yaw = clamp(this.yaw + vx / Math.cos(this.yaw) * Math.tan(this.steerAngle) / this.wheelbase * dt, -S.maxYaw, S.maxYaw);
    // berm: heading into it (z' away from the centre) the nose is turned back toward straight -- never past it
    const over = Math.abs(this.z) - T.soft, into = Math.sign(this.z) * vx * Math.tan(yaw);
    if (over > 0 && into > 0) {
      const k = S.berm * Math.min(1, over / (T.half - T.soft)) * Math.abs(vx) * dt;
      yaw = yaw > 0 ? Math.max(0, yaw - k) : Math.min(0, yaw + k);
    }
    this.yaw = yaw;
    this.vz = vx * Math.tan(this.yaw);
    let z = this.z + this.vz * dt;
    if (Math.abs(z) > T.half) { z = Math.sign(z) * T.half; this.vz = 0; }
    this.z = z;
    this._lateralSync();
  }

  /** Clear all damage, heat, broken axles and a stall; refuel. The engine is left OFF (crank it with the key). */
  repair() {
    this.hc = freshDamage();
    this.rpm = 0;
  }

  /** True while the engine runs (not off / stalled / cranking). */
  engineRunning() { return !this.hc.off && this.hc.crank <= 0; }

  /** Key turn: a running engine shuts off ('shutdown'); an off / stalled one cranks ('crank'), catching after crankTime. */
  ignition() {
    const hc = this.hc;
    if (hc.crank > 0) return;                        // already cranking
    if (!hc.off) { hc.off = true; hc.lug = 0; this._event('shutdown', { heat: hc.engine }); }
    else { hc.crank = this.hcCfg.crankTime; this._event('crank', { stalled: hc.stalled }); }
  }

  /** Start instantly (no crank; tests / bots). */
  startEngine() {
    const hc = this.hc;
    if (hc.dry) return false;
    hc.off = false; hc.stalled = false; hc.crank = 0; hc.lug = 0;
    this.rpm = this.cfg.engine.idleRpm;
    return true;
  }

  /** Teleport: (x, y) = ground point under the vehicle, angle = ground slope. Zeroes motion, releases the winch. */
  _teleport(x, y, angle) {
    this.chassis.setTransform(Vec2(x - Math.sin(angle) * this.rideOffset, y + Math.cos(angle) * this.rideOffset), angle);
    this.chassis.setLinearVelocity(Vec2(0, 0));
    this.chassis.setAngularVelocity(0);
    this.yaw = 0; this.steerAngle = 0; this.vz = 0;
    this._placeWheels();
    this.winch.detach();
    this.winch.broken = false;
    this.reverse = false;
    if (this.engineRunning()) this.rpm = this.cfg.engine.idleRpm;
  }

  /** Recover in place: set the vehicle upright, `lift` metres above where it is. Damage is KEPT (hardcore). */
  recover(lift = 0.9) {
    const p = this.chassis.getWorldCenter();
    this.hc.rolled = false;
    this._teleport(p.x, p.y - this.rideOffset + lift, 0);   // keeps the trail position z
  }

  destroy() {
    const P = this.phys, w = P.world;
    P.preStep.splice(P.preStep.indexOf(this._pre), 1);
    P.postStep.splice(P.postStep.indexOf(this._post), 1);
    P.winches.splice(P.winches.indexOf(this.winch), 1);
    for (const wh of this.wheels) { P.tracked.splice(P.tracked.indexOf(wh.body), 1); P.releaseLane(wh.lane); w.destroyBody(wh.body); }
    P.tracked.splice(P.tracked.indexOf(this.chassis), 1);
    w.destroyBody(this.chassis);
  }

  // -- per-step logic --------------------------------------------------------------------------
  forwardSpeed() {
    const f = this.chassis.getWorldVector(Vec2(1, 0)), v = this.chassis.getLinearVelocity();
    return v.x * f.x + v.y * f.y;
  }

  /** Propeller-shaft speed (forward +, rad/s) of the wheels currently connected to the engine. */
  shaftSpeed() {
    const d = this.driveline.driven(), n = d.length;
    if (!n) return 0;
    let m = 0;
    for (let k = 0; k < n; k++) m += this.wheels[d[k]].body.getAngularVelocity();
    return DIR * (m / n - this.chassis.getAngularVelocity());
  }

  /** Wheel torque per engine N*m in a range (no converter cap): 4H = highRatio * eff * highTorque, 4L = lowMult x 4H. */
  gearScale(low = this.range()) {
    const d = this.cfg.drivetrain, hi = d.highRatio * d.efficiency * (d.highTorque ?? 1);
    const r = low === true ? 'L' : low || 'H', m = RANGE_CFG[r];
    return m ? m.torque * (d.lowMult != null ? d.lowMult * hi : d.lowRatio * d.efficiency) : hi;
  }

  /** Committed range 'H' | 'L+' | 'L' | 'L-' (a moving shift request sits in pendingRange until |speed| <= SHIFT_SPEED). */
  range() { return this._range; }

  /** Requested range (controls.range; legacy controls.low true = 'L'). */
  _wantRange() { const r = this.controls.range; return RANGE_CFG[r] ? r : this.controls.low ? 'L' : 'H'; }

  /** Chassis speed magnitude (m/s): direction-free, so rolling / driving backward gates a shift like forward motion. */
  _shiftSpeed() { const v = this.chassis.getLinearVelocity(); return Math.hypot(v.x, v.y); }

  /** Transfer case: commit the requested range only when (nearly) stopped (|v| <= SHIFT_SPEED, any direction), else hold it as pendingRange. */
  _shift() {
    const want = this._wantRange();
    if (want === this._range) this.pendingRange = null;
    else if (this._shiftSpeed() <= SHIFT_SPEED) { this._range = want; this.pendingRange = null; }
    else this.pendingRange = want;
  }

  /** Gear ratio of the selected range (Low ranges are lowRatio / speed factor). */
  _ratio(r = this.range()) { const d = this.cfg.drivetrain, m = RANGE_CFG[r]; return m ? d.lowRatio / m.speed : d.highRatio; }

  _event(type, extra) { this.phys.events.push({ type, step: this.phys.stepCount, ...extra }); }

  /** Engine dies by itself (lugged / dry): off + stalled, only a key crank restarts it. */
  _stall(cause) {
    const hc = this.hc;
    hc.off = true; hc.stalled = true; hc.lug = 0; hc.crank = 0;
    this._event('stall', { cause, heat: hc.engine });
  }

  _preStep(dt) {
    const { cfg, controls: c, driveline: dl } = this, e = cfg.engine, d = cfg.drivetrain;
    const thr = clamp(c.throttle || 0, 0, 1), brk = clamp(c.brake || 0, 0, 1);

    // pedals: gas drives in the selected direction (controls.reverse), brake always brakes / holds
    this.reverse = !!c.reverse;
    this._lateral(dt);
    const drive = this.reverse ? -thr : thr, brake = brk;

    // transfer case / lockers (an overheated transfer case drops to 2WD; snapped shafts carry no drive)
    const hc = this.hc, H = this.hcCfg;
    dl.fourWD = !!c.fourWD && !hc.xferHot; dl.lockF = !!c.lockF; dl.lockR = !!c.lockR;
    dl.broken[0] = hc.axleBroken[0]; dl.broken[1] = hc.axleBroken[1];
    const connected = dl.driven().length > 0;

    this._shift();
    // engine + gearbox. 4L: lowRatio gearing, torque = lowMult x 4H, governed to lowSpeed x 4H top speed.
    const rng = this.range(), M = RANGE_CFG[rng], low = !!M, G = this._ratio(rng);
    const shaft = this.shaftSpeed();
    const a = Math.abs(drive), sgn = drive < 0 ? -1 : 1;
    const rpmShaft = Math.abs(shaft) * G * 30 / Math.PI;
    const vTread = (a > 0.001 ? sgn : 1) * shaft * cfg.wheels.radius;   // driven tread speed, + = the way the engine drives
    this._climb = (a > 0.001 ? sgn : 1) * this.chassis.getAngle() * 180 / Math.PI;

    // ignition key: each change of controls.ignition = one key turn; a crank catches after crankTime (dry tank: no-start)
    const key = c.ignition | 0;
    if (key !== this._key) { this._key = key; this.ignition(); }
    if (hc.crank > 0) {
      hc.crank = Math.max(0, hc.crank - dt);
      if (hc.crank <= 0) {
        if (hc.dry) this._event('no-start');
        else { this._event(hc.stalled ? 'restart' : 'start'); hc.off = false; hc.stalled = false; hc.lug = 0; }
      }
    }
    const on = !hc.off && hc.crank <= 0;

    // torque converter (4H): bog = 1 with the tread stopped under throttle, 0 at bogSpeed and above
    hc.bog = on && !low && connected && a > 0.25 ? clamp(1 - vTread / H.bogSpeed, 0, 1) : 0;
    // lugging -> stall: Low back-driven against the gear / fighting the brakes; 4H hammering a boiled converter in a bog
    let lug = 0;
    if (on && connected) {
      if (low) {
        const back = clamp((-vTread - H.stallBack) / H.stallBackRef, 0, 1) * (1 - 0.6 * a);
        lug = Math.max(back, brk > 0.5 && a > 0.5 ? brk * a : 0);      // power-braking: Low torque out-pulls the brakes, the engine lugs
      } else if (hc.transHot && a > 0.7) lug = H.transHotLug * hc.bog * a;
    }
    hc.lug = lug > 0 ? hc.lug + lug * H.lugGain * dt : Math.max(0, hc.lug - H.lugCool * dt);
    if (on && hc.lug >= 1) this._stall(low ? (lug > 0 && brk > 0.5 ? 'brake' : 'lug') : 'trans');

    // misfires: a battered engine / body sputters under heavy throttle (one roll per ~0.1 s slot)
    const sev = Math.max(hc.body, hc.engine), step = this.phys.stepCount || 0;
    hc.misfire = on && a > 0.7 && sev > H.misfireFrom
      && (noise(Math.floor(step / 6), 7.1) * 0.5 + 0.5) < H.misfireMax * (sev - H.misfireFrom) / (1 - H.misfireFrom);
    const power = (hc.dry ? 0 : 1) * (1 - H.bodyPowerLoss * hc.body) * (hc.engineHot ? H.enginePowerHot : 1) * (hc.misfire ? 1 - H.misfireCut : 1);
    this.power = power;
    // slipping coupling: Low launches like a clutch (launchRpm), High flashes the converter to its stall rpm
    const slipRpm = a * ((low ? e.launchRpm : H.tcStallRpm) - e.idleRpm);
    const rpmEff = clamp(Math.max(rpmShaft, e.idleRpm + slipRpm), e.idleRpm, e.redlineRpm);
    const Te = on ? a * power * torqueAt(e.torqueCurve, rpmEff) : 0;
    let Tw = Te * this.gearScale(rng);
    if (!low) {
      // converter: grade-holding stall torque -> full torque at lock-up; an overheated transmission slips to 0.
      // Back-driven (rolling back against the gas) its torque ratio climbs, up to (1 + tcRollback) x: it holds, creeping.
      const cap = a * power * this.tcCap * (1 + H.tcRollback * clamp(-vTread / H.bogSpeed, 0, 1)), k = clamp((vTread - H.bogSpeed) / (H.lockSpeed - H.bogSpeed), 0, 1);
      Tw = hc.transHot ? 0 : Math.min(Tw, cap + (Tw - cap) * k * k);
    }
    this.engineTorque = Te;
    if (on && a > 0.001 && connected) {
      const sTop = e.redlineRpm * Math.PI / 30 / d.highRatio;
      const sRed = low ? Math.min(e.redlineRpm * Math.PI / 30 / G, (d.lowSpeed ?? 0.35) * M.speed * sTop) : sTop;
      const alpha = Te / (e.inertia * G);                    // free-rev shaft acceleration
      const lim = Math.min(sRed, Math.max(sgn * shaft, 0) + alpha * dt);
      const fade = clamp((sRed - sgn * shaft) / (0.06 * sRed), 0, 1);   // soft rev limiter / Low governor
      dl.target = sgn * lim;
      dl.initImp = sgn * Tw * dt * fade;
      dl.minImp = sgn > 0 ? 0 : -Tw * dt;
      dl.maxImp = sgn > 0 ? Tw * dt : 0;
    } else {
      // in-gear engine braking toward standstill (Low x lowBrake; a dead engine in gear drags 3x harder)
      const Tb = e.engineBrake * G * dt * (on ? 1 : 3) * (low ? (d.lowBrake ?? 1) * M.brake : 1);
      dl.target = 0; dl.initImp = 0; dl.minImp = -Tb; dl.maxImp = Tb;
    }
    if (hc.crank > 0) this._rpmTarget = e.idleRpm * 0.35 * (1 - 0.6 * hc.crank / H.crankTime);   // starter spin-up
    else if (!this.engineRunning()) this._rpmTarget = 0;
    else if (!connected) this._rpmTarget = e.idleRpm + a * (e.redlineRpm - e.idleRpm) * 0.92;   // free-revs (no load)
    else this._rpmTarget = Math.max(e.idleRpm, a > 0.001 ? rpmEff : rpmShaft);

    // suspension (progressive bump stop folded into the implicit joint spring), brakes, rolling resistance
    const A = this.chassis, mA = 1 / A.getMass(), iA = A.m_invI;
    const pA = A.getWorldCenter();
    for (let wi = 0; wi < 4; wi++) {
      const w = this.wheels[wi], s = w.sus, j = w.joint, dmg = hc.shock[wi];
      const C = j.getJointTranslation(), sp = j.getJointSpeed();
      const x = C - w.info.p;                                  // compression from full droop
      const xb = s.travel - (s.bumpStopLength || 0) - H.shockWarp * dmg * s.travel;   // warped shock bottoms early
      const fl = dmg > 0 ? noise(step >> 2, wi + 1) : 0;                          // blown valving: erratic spring / damper
      const k = s.stiffness * (1 - H.shockStiffnessLoss * dmg) * (1 + H.shockFlutter * dmg * fl * 0.5);
      let F = k * Math.max(C - H.shockSag * dmg * s.travel, 0);                   // lost preload: the corner sags
      if (x > xb && s.bumpStopStiffness) F += s.bumpStopStiffness * (x - xb);
      const kEq = C > 1e-3 ? F / C : k;
      const cDamp = (sp >= 0 ? s.damping : (s.reboundDamping ?? s.damping)) * (1 - H.shockDampingLoss * dmg)
        * (1 - H.shockFlutter * dmg * Math.abs(fl));
      const pB = w.body.getPosition(), axW = A.getWorldVector(w.axisL);
      const sAx = (pB.x - pA.x) * axW.y - (pB.y - pA.y) * axW.x;
      const mEff = 1 / (mA + 1 / w.body.getMass() + iA * sAx * sAx);
      const om = Math.sqrt(kEq / mEff);
      j.setSpringFrequencyHz(om / (2 * Math.PI));
      j.setSpringDampingRatio(cDamp / (2 * mEff * om));
      w.springForce = F;

      // wheel load + surface from last step's contacts
      const recs = w.body.getUserData().contacts;
      let imp = 0, rr = 0, surf = null;
      for (let i = 0; i < recs.length; i++) {
        imp += recs[i].impulse;
        const sr = this.phys.surface(recs[i].surface).rr || 0;
        if (sr >= rr) { rr = sr; surf = recs[i].surface; }
      }
      w.load = imp / dt;
      w.surface = surf;
      j.setMaxMotorTorque(brake * cfg.brakes.torque * w.brakeBias + rr * w.load * cfg.wheels.radius);
    }

    // winch button: press = attach (if free) + reel in while held; short tap while attached = release
    const wn = this.winch, down = !!c.winch;
    if (down && !this._winchDown) {
      this._winchWasAttached = wn.attached;
      this._winchSteps = 0;
      if (!wn.attached) {
        const f = this.chassis.getWorldVector(Vec2(1, 0));
        wn.attachNearest(this.phys.anchors, f.x, f.y);
      }
    }
    if (down) this._winchSteps++;
    if (!down && this._winchDown && this._winchWasAttached && this._winchSteps < 15) wn.detach();
    wn.reel = down && wn.attached ? 1 : 0;
    this._winchDown = down;
  }

  _postStep(dt) {
    const dl = this.driveline;
    let sum = 0;
    for (let i = 0; i < 4; i++) { this.wheels[i].driveTorque = dl.imp[i] / dt; sum += dl.imp[i]; }
    this.wheelTorque = sum / dt;
    const e = this.cfg.engine, G = this._ratio();
    const rpmShaft = Math.abs(this.shaftSpeed()) * G * 30 / Math.PI;
    const on = this.engineRunning();
    const tgt = on ? clamp(Math.max(this._rpmTarget || e.idleRpm, rpmShaft), e.idleRpm, e.redlineRpm * 1.05) : this._rpmTarget;
    this.rpm += (tgt - this.rpm) * Math.min(1, dt * (on ? 14 : 6));
    this._hardcore(dt, on);
    this._fuel(dt, on);
  }

  // -- fuel: burn ~ (rpm/redline)^2 x throttle + wheelspin; empty tank = no power ('out-of-fuel')
  _fuel(dt, on) {
    const hc = this.hc, F = this.fuelCfg, e = this.cfg.engine;
    if (!on || hc.dry) return;
    const thr = clamp(this.controls.throttle || 0, 0, 1), r = this.rpm / e.redlineRpm;
    const v = this.forwardSpeed(), R = this.cfg.wheels.radius, w = this.chassis.getAngularVelocity();
    let spin = 0;
    for (const wh of this.wheels) spin = Math.max(spin, Math.abs(DIR * (wh.body.getAngularVelocity() - w) * R - v));
    hc.fuel = Math.max(0, hc.fuel - (F.idle + F.rev * r * r * thr + F.spin * Math.min(spin / F.spinRef, 1) * thr) * dt);
    if (hc.fuel <= 0) { hc.dry = true; this._event('out-of-fuel'); this._stall('fuel'); }
  }

  // -- hardcore wear / failure (runs after every step, using this step's contacts) ---------------
  _hardcore(dt, on) {
    const hc = this.hc, H = this.hcCfg, c = this.controls, dl = this.driveline;
    const [m0, m1] = H.bindMu;
    const grip = wh => {
      const r = wh.body.getUserData().contacts;
      let mu = 0;
      for (let i = 0; i < r.length; i++) if (r[i].mu > mu) mu = r[i].mu;
      return clamp((mu - m0) / (m1 - m0), 0, 1);
    };
    const W = this.wheels;
    const gF = Math.min(grip(W[FL]), grip(W[FR])), gR = Math.min(grip(W[RL]), grip(W[RR]));
    const thr = on && dl.driven().length ? clamp(c.throttle || 0, 0, 1) : 0;
    const load = thr * this.power * Math.min(1 + Math.abs(this.forwardSpeed()) / H.bindSpeed, H.bindCap);
    const wear = (v, bind, free, gain, cool) =>
      bind > free ? v + (bind - free) * gain * dt : Math.max(0, v - cool * (1 - bind / free) * dt);

    // locked diffs on driven, gripping axles -> axle-shaft stress
    const lockOn = [dl.fourWD && dl.lockF, dl.lockR], g = [gF, gR];
    for (let ai = 0; ai < 2; ai++) {
      if (hc.axleBroken[ai]) continue;
      const bind = lockOn[ai] ? g[ai] * load : 0;
      const fast = Math.abs(this.forwardSpeed()) > H.lockerFastSpeed && g[ai] >= H.lockerFastGrip ? H.lockerFastGain : 1;
      hc.axle[ai] = wear(hc.axle[ai], bind, H.axleFree, H.axleGain * fast, H.axleCool);
      if (hc.axle[ai] >= 1) {
        hc.axle[ai] = 1; hc.axleBroken[ai] = true;
        this._event('axle-snap', { axle: this.cfg.axles[ai].name });
      }
    }
    // locked centre (4WD) with both axles gripping -> transfer-case heat
    if (hc.xferHot) {
      hc.xfer = Math.max(0, hc.xfer - H.xferCool * dt);
      if (hc.xfer <= H.xferRecover) { hc.xferHot = false; this._event('cooled', { part: 'xfer' }); }
    } else {
      const bind = dl.fourWD && !hc.axleBroken[0] && !hc.axleBroken[1] ? Math.min(gF, gR) * load : 0;
      hc.xfer = wear(hc.xfer, bind, H.xferFree, H.xferGain, H.xferCool);
      if (hc.xfer >= 1) { hc.xfer = 1; hc.xferHot = true; this._event('overheat', { part: 'xfer' }); }
    }
    // transmission: a bogged torque converter (4H) churns its fluid -> transTemp; at 1 the 4H drive slips
    if (hc.transHot) {
      hc.trans = Math.max(0, hc.trans - H.transCool * (this.range() !== 'H' ? 1 : 1 - 0.6 * thr) * dt);
      if (hc.trans <= H.transRecover) { hc.transHot = false; this._event('cooled', { part: 'trans' }); }
    } else if (hc.bog > 0) {
      hc.trans += H.transGain * thr * hc.bog * (this._climb > H.transSteepDeg ? H.transSteepGain : 1) * dt;
      if (hc.trans >= 1) { hc.trans = 1; hc.transHot = true; this._event('overheat', { part: 'trans' }); }
    } else hc.trans = Math.max(0, hc.trans - H.transCool * (1 - 0.6 * thr) * dt);
    // engine: sustained high rpm under heavy throttle
    const e = this.cfg.engine, hot = on && thr > 0.7 && this.rpm > (H.engineHotRpm - H.engineHotBody * hc.body) * e.redlineRpm;
    hc.engine = hot ? hc.engine + H.engineGain * (1 + hc.body) * thr * dt : Math.max(0, hc.engine - H.engineCool * (1 - 0.6 * thr) * dt);
    if (!hc.engineHot && hc.engine >= 1) { hc.engine = 1; hc.engineHot = true; this._event('overheat', { part: 'engine' }); }
    else if (hc.engineHot && hc.engine <= H.engineRecover) { hc.engineHot = false; this._event('cooled', { part: 'engine' }); }
    if (hc.engine > 1) hc.engine = 1;

    // impacts: chassis slams, wheel hits across / along the suspension axis
    const A = this.chassis, fx = A.getWorldVector(Vec2(1, 0)), up = A.getWorldVector(Vec2(0, 1));
    let bodyHit = 0, hx = 0;
    for (const r of A.getUserData().contacts) if (r.vn > bodyHit) { bodyHit = r.vn; hx = A.getLocalPoint(Vec2(r.x, r.y)).x; }
    let dBody = 0;
    if (bodyHit > H.bodyHit) {
      dBody = (bodyHit - H.bodyHit) * H.bodyScale;
      const ai = hx >= 0 ? 0 : 1;                         // the axle nearest the hit takes half of it
      hc.shock[2 * ai] = Math.min(1, hc.shock[2 * ai] + dBody * 0.5);
      hc.shock[2 * ai + 1] = Math.min(1, hc.shock[2 * ai + 1] + dBody * 0.5);
    }
    let dShock = 0;
    for (let wi = 0; wi < 4; wi++) {
      let d = 0;
      for (const r of W[wi].body.getUserData().contacts) {
        if (!(r.vn > 0)) continue;
        const across = r.vn * Math.abs(r.nx * fx.x + r.ny * fx.y), along = r.vn * Math.abs(r.nx * up.x + r.ny * up.y);
        d = Math.max(d, (across - H.wheelHit) * H.wheelScale, (along - H.dropHit) * H.dropScale);
      }
      if (d > 0) { hc.shock[wi] = Math.min(1, hc.shock[wi] + d); dShock = Math.max(dShock, d); }
    }
    // rollover: first roof landing of each roll
    const upside = Math.cos(A.getAngle());
    if (!hc.rolled && upside < -0.3 && A.getUserData().contacts.length) {
      hc.rolled = true;
      dBody += H.rollDamage;
      for (let wi = 0; wi < 4; wi++) hc.shock[wi] = Math.min(1, hc.shock[wi] + H.rollShockDamage);
      this._event('rollover');
    } else if (hc.rolled && upside > 0.5) hc.rolled = false;
    if (dBody > 0) hc.body = Math.min(1, hc.body + dBody);
    if (dBody + dShock > 0.02) this._event('damage', { body: dBody, shock: dShock, total: hc.body });
  }

  // -- plain-data views (renderer / HUD / tests consume only these) ----------------------------
  /** Static description for drawing: chassis polygons in the chassis frame. */
  getShape() {
    const c = this.cfg;
    return {
      shapes: c.chassis.shapes.map(s => ({ kind: s.kind, points: s.points })),
      wheelRadius: c.wheels.radius,
      halfTrack: this.halfTrack, tyreWidth: this.tyreWidth, wheelbase: this.wheelbase,
      winchMount: c.winch.mount,
      redlineRpm: c.engine.redlineRpm,
    };
  }

  getState() {
    const st = this._state || (this._state = { wheels: this.wheels.map(() => ({ contacts: [] })), winch: {}, com: {}, chassisContacts: [] });
    const b = this.chassis, p = b.getPosition(), v = b.getLinearVelocity(), cm = b.getWorldCenter(), c = this.controls;
    const R = this.cfg.wheels.radius, dtInv = 60;
    st.x = p.x; st.y = p.y; st.angle = b.getAngle();
    st.vx = v.x; st.vy = v.y; st.omega = b.getAngularVelocity();
    st.speed = this.forwardSpeed();
    st.com.x = cm.x; st.com.y = cm.y;
    st.rpm = this.rpm; st.engineTorque = this.engineTorque; st.wheelTorque = this.wheelTorque;
    const rng = this.range();
    st.gear = this.reverse ? 'R' : rng; st.range = rng; st.pendingRange = this.pendingRange;
    st.reverse = this.reverse;
    st.fourWD = !!c.fourWD; st.lockF = !!c.lockF; st.lockR = !!c.lockR; st.low = rng !== 'H';
    st.throttle = c.throttle; st.brake = c.brake;
    st.upsideDown = Math.cos(st.angle) < 0;
    st.z = this.z; st.yaw = this.yaw; st.vz = this.vz; st.steer = this.steerAngle; st.steerInput = c.steer || 0;
    const hc = this.hc, hs = st.hardcore || (st.hardcore = {});
    hs.axleStress = hc.axle; hs.axleBroken = hc.axleBroken; hs.xferHeat = hc.xfer; hs.xferHot = hc.xferHot;
    hs.engineHeat = hc.engine; hs.engineHot = hc.engineHot; hs.stalled = hc.stalled; hs.cranking = hc.crank > 0;
    hs.engineOff = hc.off && hc.crank <= 0; hs.lugStall = hc.lug; hs.crank = hc.crank > 0 ? 1 - hc.crank / this.hcCfg.crankTime : 0;
    hs.bog = hc.bog; hs.transTemp = hc.trans; hs.transHot = hc.transHot; hs.body = hc.body; hs.shock = hc.shock; hs.power = this.power ?? 1; hs.misfire = hc.misfire; hs.fuel = hc.fuel; hs.dry = hc.dry;
    st.fuel = hc.fuel;
    hs.lug = Number.isFinite(hc.bog) ? hc.bog : 0;                  // lugging 0..1 = converter bog (debug / exhaust smoke)
    st.engineOn = this.engineRunning(); st.engineRunning = st.engineOn;
    st.stalled = hc.stalled;
    for (let i = 0; i < this.wheels.length; i++) {
      const w = this.wheels[i], o = st.wheels[i], wp = w.body.getPosition();
      const C = w.joint.getJointTranslation();
      const top = b.getWorldPoint(w.shockTopL), piv = b.getWorldPoint(w.linkPivotL);
      o.name = w.name; o.side = w.side; o.axle = w.axle; o.radius = R;
      o.x = wp.x; o.y = wp.y; o.angle = w.body.getAngle(); o.omega = w.body.getAngularVelocity();
      o.z = w.z; o.lz = w.lz; o.dx = w.shift;                 // trail z, chassis-local z (±half track), yaw anchor shift (chassis x)
      o.hx = wp.x - Math.cos(st.angle) * w.shift; o.hy = wp.y - Math.sin(st.angle) * w.shift;   // hub without the yaw shift
      o.steer = w.axle === 0 ? this.steerAngle : 0;
      o.mountX = top.x; o.mountY = top.y;              // shock upper mount (world)
      o.linkX = piv.x; o.linkY = piv.y;                // suspension link pivot on the frame (world)
      o.travel = C - w.info.p;                         // m of compression from full droop
      o.compression = (C - w.info.p) / w.sus.travel;   // 0 = full droop, 1 = full bump
      o.susSpeed = w.joint.getJointSpeed();
      o.springForce = w.springForce;
      o.driveTorque = w.driveTorque;                   // N*m from the driveline (forward +)
      o.surfaceSpeed = DIR * (o.omega - st.omega) * R; // tyre tread speed relative to chassis (forward +)
      o.slip = o.surfaceSpeed - st.speed;              // m/s of wheelspin (+) / lock-up (-)
      const recs = w.body.getUserData().contacts;
      o.contacts.length = 0;
      let load = 0, surf = null;
      for (let k = 0; k < recs.length; k++) {
        const r = recs[k], f = r.impulse * dtInv;
        o.contacts.push({ x: r.x, y: r.y, nx: r.nx, ny: r.ny, force: f, vn: r.vn, surface: r.surface, mu: r.mu, terrain: r.terrain });
        load += f; surf = r.surface;
      }
      o.load = load; o.surface = surf; o.grounded = recs.length > 0;
      o.damage = hc.shock[i];                          // 0..1 shock damage (warped / blown)
    }
    st.chassisContacts.length = 0;
    for (const r of b.getUserData().contacts) st.chassisContacts.push({ x: r.x, y: r.y, nx: r.nx, ny: r.ny, force: r.impulse * dtInv, vn: r.vn, surface: r.surface, terrain: r.terrain });
    const wn = this.winch, ws = st.winch, m = wn.mountWorld(), an = wn.anchorWorld();
    ws.attached = wn.attached; ws.broken = wn.broken; ws.tension = wn.tension; ws.peakTension = wn.peakTension;
    ws.length = wn.length; ws.maxLength = wn.maxLength; ws.maxPull = wn.maxPull; ws.breakTension = wn.breakTension;
    ws.reeling = wn.reel > 0;
    ws.mx = m.x; ws.my = m.y; ws.ax = an ? an.x : 0; ws.ay = an ? an.y : 0;
    return st;
  }
}
