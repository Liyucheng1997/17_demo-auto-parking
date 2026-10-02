/**
 * 目标位姿生成：车位几何 + 泊车方式 → 后轴中心目标位姿。
 *
 * 垂直车位：
 *   - 倒车入库（默认）：车头朝外，车尾距车位底线 backClearance；
 *   - 车头入库：车头朝里，车头距车位底线 backClearance。
 * 侧方车位：车身沿车位长度方向居中，航向遵循道路通行方向。
 */

import { normalizeAngle } from "../core/math.js";

export const PARKING_MODES = Object.freeze({
  REVERSE_IN: "reverse-in",
  HEAD_IN: "head-in",
});

export function goalPoseForSlot(slot, vehicle, { mode = PARKING_MODES.REVERSE_IN, backClearance = 0.35 } = {}) {
  if (slot.type === "parallel") {
    const theta = slot.heading;
    // 车身几何中心放在车位中心，后轴中心在其后方
    const d = vehicle.length / 2 - vehicle.rearOverhang;
    return {
      x: slot.center.x - d * Math.cos(theta),
      y: slot.center.y - d * Math.sin(theta),
      theta: normalizeAngle(theta),
    };
  }

  const ux = Math.cos(slot.inward);
  const uy = Math.sin(slot.inward);
  if (mode === PARKING_MODES.HEAD_IN) {
    // 车头朝里：后轴距开口 = 深度 - 间隙 - 前悬 - 轴距
    const depthOfAxle = slot.depth - backClearance - vehicle.frontOverhang - vehicle.wheelbase;
    return {
      x: slot.opening.x + ux * depthOfAxle,
      y: slot.opening.y + uy * depthOfAxle,
      theta: normalizeAngle(slot.inward),
    };
  }
  // 倒车入库：后轴距开口 = 深度 - 间隙 - 后悬
  const depthOfAxle = slot.depth - backClearance - vehicle.rearOverhang;
  return {
    x: slot.opening.x + ux * depthOfAxle,
    y: slot.opening.y + uy * depthOfAxle,
    theta: normalizeAngle(slot.inward + Math.PI),
  };
}
