/**
 * 超声波雷达模型：每个探头在其视场角 (FOV) 内发射若干条射线，
 * 与障碍物多边形求交，取最近回波距离；超出量程或低于盲区返回 null。
 * 可选高斯测距噪声（可复现随机数）。
 */

import { toWorld } from "../core/math.js";
import { rayPolygonDistance } from "../core/geometry.js";
import { ultrasonicLayout } from "../model/vehicle.js";

export function createUltrasonicArray(vehicle, { rays = 7, noiseStd = 0, rng = Math.random } = {}) {
  const sensors = ultrasonicLayout(vehicle);

  function gaussian() {
    const u = Math.max(1e-12, rng());
    const v = rng();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }

  /** 对给定车辆位姿与障碍物检测器进行一次扫描 */
  function scan(pose, checker) {
    return sensors.map((sensor) => {
      const origin = toWorld(pose, sensor.x, sensor.y);
      const yaw = pose.theta + sensor.yaw;
      let best = Infinity;
      const nearby = checker.candidates(origin.x, origin.y, sensor.maxRange);
      for (let i = 0; i < rays; i += 1) {
        const a = yaw - sensor.fov / 2 + (sensor.fov * i) / (rays - 1);
        const dx = Math.cos(a);
        const dy = Math.sin(a);
        for (const obstacle of nearby) {
          const d = rayPolygonDistance(origin.x, origin.y, dx, dy, obstacle.polygon);
          if (d < best) best = d;
        }
      }
      let distance = best + (noiseStd > 0 ? gaussian() * noiseStd : 0);
      if (!(distance <= sensor.maxRange) || distance < sensor.minRange) distance = best < sensor.minRange ? sensor.minRange : null;
      return { ...sensor, origin, yawWorld: yaw, distance };
    });
  }

  return { sensors, scan };
}

/** 运动方向上（前/后）最近的雷达距离，用于紧急制动 */
export function nearestInMotion(readings, dir) {
  const group = dir >= 0 ? ["FL", "FLM", "FRM", "FR"] : ["RL", "RLM", "RRM", "RR"];
  let best = Infinity;
  for (const r of readings) {
    if (group.includes(r.id) && r.distance !== null) best = Math.min(best, r.distance);
  }
  return best;
}
