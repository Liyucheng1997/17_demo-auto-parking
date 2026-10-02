/**
 * 闭环仿真器：规划轨迹 → 控制器 → 执行器动力学 → 车辆运动学 → 传感器 → 控制器 …
 *
 * 状态机：
 *   DRIVE  按当前挡位段跟踪轨迹，纵向 v_cmd = min(v_ref(s), √(2·a·剩余距离))
 *   SHIFT  换挡停顿：车辆静止，转向机预先打到下一段起点转角（原地转向）
 *   DONE   到达终点，挂 P 挡
 *   ABORT  紧急制动后无法继续 / 超时
 */

import { angleDiff, createRng } from "../core/math.js";
import { stepVehicle } from "../model/vehicle.js";
import { CONTROLLERS, projectOnPath, purePursuit, rearWheelFeedback } from "../control/controllers.js";
import { sampleAt, DEFAULT_SPEED_CONFIG } from "../planning/speedProfile.js";
import { createUltrasonicArray, nearestInMotion } from "../perception/ultrasonic.js";

export const PHASES = Object.freeze({ DRIVE: "drive", SHIFT: "shift", DONE: "done", ABORT: "abort" });

export class ParkingSimulator {
  constructor({
    plan,
    vehicle,
    checker,
    controller = CONTROLLERS.PURE_PURSUIT,
    controlConfig = {},
    dt = 0.02,
    localizationNoise = 0,
    sensorNoise = 0,
    aebDistance = 0.1,
    seed = 1,
  }) {
    this.plan = plan;
    this.vehicle = vehicle;
    this.checker = checker;
    this.controller = controller;
    this.controlConfig = controlConfig;
    this.dt = dt;
    this.localizationNoise = localizationNoise;
    this.aebDistance = aebDistance;
    this.rng = createRng(seed);
    this.sensors = createUltrasonicArray(vehicle, { noiseStd: sensorNoise, rng: this.rng });
    this.segments = plan.segments;
    this.state = { ...plan.start, v: 0, delta: 0, accel: 0, steerRate: 0 };
    this.phase = PHASES.DRIVE;
    this.segIndex = 0;
    this.hint = 0;
    this.time = 0;
    this.shiftTimer = 0;
    this.distance = 0;
    this.collided = false;
    this.aebTriggered = 0;
    this.readings = [];
    this.lastControl = null;
    this.minClearance = Infinity;
    this.log = { t: [], v: [], vRef: [], delta: [], deltaCmd: [], ey: [], ePsi: [], s: [], gear: [], x: [], y: [], theta: [], segIndex: [] };
    this.gearShiftTime = DEFAULT_SPEED_CONFIG.gearShiftTime;
  }

  get gear() {
    if (this.phase === PHASES.DONE) return "P";
    if (this.phase === PHASES.SHIFT) return "N";
    return this.segments[this.segIndex].dir > 0 ? "D" : "R";
  }

  get finished() {
    return this.phase === PHASES.DONE || this.phase === PHASES.ABORT;
  }

  #measuredPose() {
    if (!this.localizationNoise) return this.state;
    const n = () => (this.rng() - 0.5) * 2 * this.localizationNoise;
    return { ...this.state, x: this.state.x + n(), y: this.state.y + n(), theta: this.state.theta + n() * 0.02 };
  }

  step() {
    if (this.finished) return this.snapshot();
    const dt = this.dt;
    let command = { v: 0, delta: this.state.delta };
    let vRef = 0;
    let tracking = { ey: 0, ePsi: 0 };
    let sGlobal = this.segments[this.segIndex].sStart;

    this.readings = this.sensors.scan(this.state, this.checker);

    if (this.phase === PHASES.SHIFT) {
      const next = this.segments[this.segIndex];
      command = { v: 0, delta: next.points[0].steer };
      this.shiftTimer -= dt;
      if (this.shiftTimer <= 0 && Math.abs(this.state.v) < 0.01) {
        this.phase = PHASES.DRIVE;
      }
    } else if (this.phase === PHASES.DRIVE) {
      const seg = this.segments[this.segIndex];
      const pts = seg.points;
      const dir = seg.dir;
      const measured = this.#measuredPose();
      const projection = projectOnPath(pts, measured, this.hint);
      this.hint = projection.index;
      const remaining = seg.length - projection.s;
      sGlobal = seg.sStart + Math.min(seg.length, Math.max(0, projection.s));

      const ref = sampleAt(pts, projection.s + 0.3);
      vRef = Math.max(ref.v, 0.2);
      const vStop = Math.sqrt(2 * this.vehicle.maxDecel * 0.6 * Math.max(0, remaining - 0.02));
      let vCmd = remaining < 0.03 ? 0 : Math.min(vRef, vStop);

      const control =
        this.controller === CONTROLLERS.REAR_WHEEL
          ? rearWheelFeedback({ pose: measured, v: this.state.v, dir, points: pts, projection, vehicle: this.vehicle, config: this.controlConfig })
          : purePursuit({ pose: measured, v: this.state.v, dir, points: pts, projection, vehicle: this.vehicle, config: this.controlConfig });
      this.lastControl = { ...control, projection, remaining };
      tracking = { ey: projection.ey, ePsi: angleDiff(measured.theta, projection.ref.theta) };

      // 自动紧急制动 (AEB)：运动方向超声波距离过近
      const nearest = nearestInMotion(this.readings, dir);
      if (nearest < this.aebDistance && remaining > 0.1) {
        vCmd = 0;
        this.aebTriggered += 1;
      }
      command = { v: dir * vCmd, delta: control.delta };
      vRef = dir * Math.min(vRef, vStop);

      if (remaining < 0.03 && Math.abs(this.state.v) < 0.02) {
        if (this.segIndex === this.segments.length - 1) {
          this.phase = PHASES.DONE;
        } else {
          this.segIndex += 1;
          this.hint = 0;
          this.phase = PHASES.SHIFT;
          this.shiftTimer = this.gearShiftTime;
        }
      }
    }

    const prev = this.state;
    this.state = stepVehicle(this.vehicle, this.state, command, dt);
    this.distance += Math.hypot(this.state.x - prev.x, this.state.y - prev.y);
    this.time += dt;
    this.lastCommand = command;

    if (this.checker.collides(this.state) && !this.collided) {
      // 带安全裕度的检测命中后，再用 0 裕度确认是否真实刮碰
      const { distance } = this.checker.clearance(this.state, 1);
      if (distance <= 1e-3) this.collided = true;
    }
    if (Math.round(this.time / dt) % 5 === 0) {
      this.minClearance = Math.min(this.minClearance, this.checker.clearance(this.state, 2).distance);
    }
    if (this.time > this.plan.trajectory.duration * 3 + 30) this.phase = PHASES.ABORT;

    const L = this.log;
    L.t.push(this.time);
    L.v.push(this.state.v);
    L.vRef.push(vRef);
    L.delta.push(this.state.delta);
    L.deltaCmd.push(command.delta);
    L.ey.push(tracking.ey);
    L.ePsi.push(tracking.ePsi);
    L.s.push(sGlobal);
    L.gear.push(this.gear);
    L.x.push(this.state.x);
    L.y.push(this.state.y);
    L.theta.push(this.state.theta);
    L.segIndex.push(this.segIndex);
    return this.snapshot();
  }

  snapshot() {
    return {
      time: this.time,
      state: this.state,
      phase: this.phase,
      gear: this.gear,
      segIndex: this.segIndex,
      control: this.lastControl,
      readings: this.readings,
      distance: this.distance,
    };
  }

  /** 最终泊车精度 */
  result() {
    const goal = this.plan.goal;
    const s = this.state;
    const c = Math.cos(goal.theta);
    const sn = Math.sin(goal.theta);
    const dx = s.x - goal.x;
    const dy = s.y - goal.y;
    return {
      success: this.phase === PHASES.DONE && !this.collided,
      phase: this.phase,
      time: this.time,
      distance: this.distance,
      positionError: Math.hypot(dx, dy),
      longitudinalError: c * dx + sn * dy,
      lateralError: -sn * dx + c * dy,
      headingError: angleDiff(s.theta, goal.theta),
      collided: this.collided,
      aebTriggered: this.aebTriggered,
      minClearance: this.minClearance,
      maxLateralError: Math.max(...this.log.ey.map(Math.abs)),
    };
  }

  /** 无界面批量运行（测试与基准） */
  run(maxSteps = 200000) {
    for (let i = 0; i < maxSteps && !this.finished; i += 1) this.step();
    return this.result();
  }
}
