import { test } from "node:test";
import assert from "node:assert/strict";
import {
  ackermannAngles,
  createVehicleParams,
  minTurningRadius,
  propagatePose,
  stepVehicle,
  vehicleFootprint,
} from "../src/model/vehicle.js";
import { angleDiff } from "../src/core/math.js";

const vehicle = createVehicleParams();

test("最小转弯半径 R = L / tan(δmax)", () => {
  assert.ok(Math.abs(minTurningRadius(vehicle) - vehicle.wheelbase / Math.tan(vehicle.maxSteer)) < 1e-12);
  assert.ok(minTurningRadius(vehicle) > 3.5 && minTurningRadius(vehicle) < 4.5, "符合 C 级车常识");
});

test("满舵绕行一整圈回到原点（圆弧精确积分）", () => {
  const R = minTurningRadius(vehicle);
  let pose = { x: 1, y: 2, theta: 0.3 };
  const steps = 360;
  for (let i = 0; i < steps; i += 1) pose = propagatePose(pose, (2 * Math.PI * R) / steps, vehicle.maxSteer, vehicle.wheelbase);
  assert.ok(Math.hypot(pose.x - 1, pose.y - 2) < 1e-9);
  assert.ok(Math.abs(angleDiff(pose.theta, 0.3)) < 1e-9);
});

test("倒车与前进互逆", () => {
  const start = { x: 0, y: 0, theta: 0.7 };
  const fwd = propagatePose(start, 1.3, 0.4, vehicle.wheelbase);
  const back = propagatePose(fwd, -1.3, 0.4, vehicle.wheelbase);
  assert.ok(Math.hypot(back.x, back.y) < 1e-9 && Math.abs(angleDiff(back.theta, 0.7)) < 1e-9);
});

test("阿克曼几何：内侧轮转角大于外侧轮", () => {
  const { left, right } = ackermannAngles(vehicle, 0.4);
  assert.ok(left > 0.4 && right < 0.4 && right > 0);
});

test("执行器约束：转角速率、机械限位、车速上限", () => {
  let state = { x: 0, y: 0, theta: 0, v: 0, delta: 0 };
  const dt = 0.02;
  state = stepVehicle(vehicle, state, { v: 10, delta: 2 }, dt);
  assert.ok(Math.abs(state.delta) <= vehicle.maxSteerRate * dt + 1e-12, "单步转角变化不超过速率上限");
  for (let i = 0; i < 1000; i += 1) state = stepVehicle(vehicle, state, { v: 10, delta: 2 }, dt);
  assert.ok(Math.abs(state.delta - vehicle.maxSteer) < 1e-9, "转角饱和于机械限位");
  assert.ok(state.v <= vehicle.maxSpeedForward + 1e-9, "车速不超过前进上限");
});

test("车身外廓尺寸与参考点偏置", () => {
  const fp = vehicleFootprint(vehicle, { x: 0, y: 0, theta: 0 });
  const xs = fp.points.map((p) => p.x);
  assert.ok(Math.abs(Math.min(...xs) + vehicle.rearOverhang) < 1e-9);
  assert.ok(Math.abs(Math.max(...xs) - (vehicle.wheelbase + vehicle.frontOverhang)) < 1e-9);
});
