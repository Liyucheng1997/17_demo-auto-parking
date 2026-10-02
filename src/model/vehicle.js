/**
 * 车辆模型
 *
 * 参考点取后轴中心 (rear axle center)，状态 X = [x, y, θ, v, δ]。
 * 运动学自行车模型 (Kinematic Bicycle Model)：
 *   ẋ = v·cosθ,  ẏ = v·sinθ,  θ̇ = v·tanδ / L
 * 执行器模型：
 *   转向：一阶惯性 τ_δ + 转角速率限幅 |δ̇| ≤ δ̇_max + 机械限位 |δ| ≤ δ_max
 *   纵向：一阶惯性 τ_v + 加/减速度限幅
 */

import { clamp, deg2rad, normalizeAngle } from "../core/math.js";
import { orientedBox } from "../core/geometry.js";

/** 典型 C 级轿车参数（尺寸单位 m，角度单位 rad） */
export const DEFAULT_VEHICLE = Object.freeze({
  length: 4.7,
  width: 1.85,
  wheelbase: 2.8,
  frontOverhang: 0.95,
  rearOverhang: 0.95,
  track: 1.58,
  wheelRadius: 0.33,
  maxSteer: deg2rad(36),
  maxSteerRate: deg2rad(40),
  steerTimeConstant: 0.12,
  steeringRatio: 15.5,
  maxSpeedForward: 2.0,
  maxSpeedReverse: 1.2,
  maxAccel: 0.8,
  maxDecel: 1.2,
  speedTimeConstant: 0.25,
  maxLateralAccel: 1.0,
});

export function createVehicleParams(overrides = {}) {
  const params = { ...DEFAULT_VEHICLE, ...overrides };
  if (overrides.wheelbase !== undefined && overrides.length === undefined) {
    params.length = params.wheelbase + params.frontOverhang + params.rearOverhang;
  }
  return Object.freeze(params);
}

/** 后轴中心最小转弯半径 R_min = L / tan(δ_max) */
export function minTurningRadius(params) {
  return params.wheelbase / Math.tan(params.maxSteer);
}

/** 车身外廓最大扫掠半径（外侧前角点），用于展示转弯通过性 */
export function outerSweepRadius(params) {
  const r = minTurningRadius(params);
  return Math.hypot(r + params.width / 2, params.wheelbase + params.frontOverhang);
}

/** 车身几何中心相对后轴中心沿车身方向的偏移 */
export function centerOffset(params) {
  return params.length / 2 - params.rearOverhang;
}

/** 车辆外廓多边形（可加安全裕度 margin） */
export function vehicleFootprint(params, pose, margin = 0) {
  const d = centerOffset(params);
  return orientedBox(
    pose.x + d * Math.cos(pose.theta),
    pose.y + d * Math.sin(pose.theta),
    pose.theta,
    params.length + 2 * margin,
    params.width + 2 * margin,
  );
}

/**
 * 阿克曼转向几何：等效自行车模型转角 δ → 左、右前轮实际转角。
 * 内侧轮转角大于外侧轮，使四个车轮绕同一瞬心转动。
 */
export function ackermannAngles(params, delta) {
  if (Math.abs(delta) < 1e-6) return { left: 0, right: 0 };
  const R = params.wheelbase / Math.tan(delta);
  const half = params.track / 2;
  return {
    left: Math.atan(params.wheelbase / (R - half)),
    right: Math.atan(params.wheelbase / (R + half)),
  };
}

/**
 * 在恒定 (v·dt, δ) 下对自行车模型做精确的圆弧积分。
 * ds 为有符号弧长（倒车为负）。规划器的运动基元与仿真器共用此函数，
 * 保证“规划用的模型”与“被控对象”一致。
 */
export function propagatePose(pose, ds, delta, wheelbase) {
  const tanDelta = Math.tan(delta);
  if (Math.abs(tanDelta) < 1e-9) {
    return {
      x: pose.x + ds * Math.cos(pose.theta),
      y: pose.y + ds * Math.sin(pose.theta),
      theta: pose.theta,
    };
  }
  const R = wheelbase / tanDelta;
  const dTheta = ds / R;
  const theta = pose.theta + dTheta;
  return {
    x: pose.x + R * (Math.sin(theta) - Math.sin(pose.theta)),
    y: pose.y + R * (Math.cos(pose.theta) - Math.cos(theta)),
    theta: normalizeAngle(theta),
  };
}

/**
 * 带执行器动力学的单步仿真。
 * state: { x, y, theta, v, delta }，command: { v, delta }（期望车速与期望前轮转角）
 */
export function stepVehicle(params, state, command, dt) {
  // 转向执行器：一阶惯性 + 速率限幅 + 机械限位
  const deltaCmd = clamp(command.delta, -params.maxSteer, params.maxSteer);
  const desiredRate = (deltaCmd - state.delta) / Math.max(params.steerTimeConstant, dt);
  const rate = clamp(desiredRate, -params.maxSteerRate, params.maxSteerRate);
  const delta = clamp(state.delta + rate * dt, -params.maxSteer, params.maxSteer);

  // 纵向执行器：一阶惯性 + 加减速限幅
  const vCmd = clamp(command.v, -params.maxSpeedReverse, params.maxSpeedForward);
  const desiredAccel = (vCmd - state.v) / Math.max(params.speedTimeConstant, dt);
  const braking = Math.sign(desiredAccel) !== Math.sign(state.v) && Math.abs(state.v) > 1e-3;
  const accelLimit = braking ? params.maxDecel : params.maxAccel;
  const accel = clamp(desiredAccel, -accelLimit, accelLimit);
  let v = state.v + accel * dt;
  if (Math.abs(vCmd) < 1e-6 && Math.sign(v) !== Math.sign(state.v)) v = 0;

  // 位姿：取步长内平均车速做圆弧积分
  const vMean = (state.v + v) / 2;
  const pose = propagatePose(state, vMean * dt, (state.delta + delta) / 2, params.wheelbase);
  return { ...pose, v, delta, accel, steerRate: rate };
}

/**
 * 12 路超声波雷达布置（车辆坐标系，原点为后轴中心，x 向前，y 向左）。
 * UPA：前后各 4 个，短距 (0.15–2.5 m)；APA：侧向 4 个，长距 (0.3–4.5 m)。
 */
export function ultrasonicLayout(params) {
  const front = params.wheelbase + params.frontOverhang;
  const rear = -params.rearOverhang;
  const hw = params.width / 2;
  const upa = { fov: deg2rad(60), minRange: 0.15, maxRange: 2.5, kind: "UPA" };
  const apa = { fov: deg2rad(30), minRange: 0.3, maxRange: 4.5, kind: "APA" };
  return [
    { id: "FL", x: front - 0.12, y: hw - 0.12, yaw: deg2rad(55), ...upa },
    { id: "FLM", x: front, y: hw * 0.35, yaw: deg2rad(8), ...upa },
    { id: "FRM", x: front, y: -hw * 0.35, yaw: deg2rad(-8), ...upa },
    { id: "FR", x: front - 0.12, y: -hw + 0.12, yaw: deg2rad(-55), ...upa },
    { id: "RL", x: rear + 0.12, y: hw - 0.12, yaw: deg2rad(125), ...upa },
    { id: "RLM", x: rear, y: hw * 0.35, yaw: deg2rad(172), ...upa },
    { id: "RRM", x: rear, y: -hw * 0.35, yaw: deg2rad(-172), ...upa },
    { id: "RR", x: rear + 0.12, y: -hw + 0.12, yaw: deg2rad(-125), ...upa },
    { id: "SFL", x: front - 0.45, y: hw, yaw: deg2rad(90), ...apa },
    { id: "SFR", x: front - 0.45, y: -hw, yaw: deg2rad(-90), ...apa },
    { id: "SRL", x: rear + 0.45, y: hw, yaw: deg2rad(90), ...apa },
    { id: "SRR", x: rear + 0.45, y: -hw, yaw: deg2rad(-90), ...apa },
  ];
}
