/**
 * 横向跟踪控制器。两种控制律均同时支持前进与倒车：
 *
 * 1) 纯跟踪 Pure Pursuit（几何法）
 *    在参考路径上取前视距离 L_d 处的目标点，求通过后轴中心并与当前“运动方向”
 *    相切、经过目标点的圆弧：κ = 2·sin α / L_d。
 *    倒车时运动方向为 θ+π，且转向曲率 tanδ/L 与运动曲率反号：δ = −atan(L·κ)。
 *
 * 2) 后轮反馈 Rear-Wheel Feedback（Samson 型，基于误差动力学的 Lyapunov 设计）
 *    ω = v·κ_r·cos e_θ / (1 − κ_r·e_y) − k_θ·|v|·e_θ − k_y·v·(sin e_θ / e_θ)·e_y
 *    δ = atan(L·ω / v)
 *    对曲率信息利用更充分，泊车末端定位精度更高。
 */

import { angleDiff, clamp, toLocal } from "../core/math.js";

export const CONTROLLERS = Object.freeze({
  PURE_PURSUIT: "pure-pursuit",
  REAR_WHEEL: "rear-wheel-feedback",
});

export const DEFAULT_CONTROL_CONFIG = Object.freeze({
  lookaheadBase: 1.0,
  lookaheadGain: 0.6,
  lookaheadMin: 1.0,
  lookaheadMax: 2.6,
  kTheta: 1.6,
  kY: 0.9,
});

/** 找到路径上离车辆最近的点（从 hint 开始在窗口内搜索，保证单调前进） */
export function projectOnPath(points, pose, hint = 0, window = 60) {
  let best = hint;
  let bestDist = Infinity;
  const end = Math.min(points.length - 1, hint + window);
  for (let i = Math.max(0, hint - 5); i <= end; i += 1) {
    const d = (points[i].x - pose.x) ** 2 + (points[i].y - pose.y) ** 2;
    if (d < bestDist) {
      bestDist = d;
      best = i;
    }
  }
  // 在相邻线段上做连续投影，得到精确弧长与横向误差
  const i0 = Math.max(0, best - 1);
  const i1 = Math.min(points.length - 1, best + 1);
  let s = points[best].s;
  let ref = points[best];
  let ey = 0;
  let found = false;
  for (const [a, b] of [
    [points[i0], points[best]],
    [points[best], points[i1]],
  ]) {
    const vx = b.x - a.x;
    const vy = b.y - a.y;
    const len2 = vx * vx + vy * vy;
    if (len2 < 1e-12) continue;
    const t = ((pose.x - a.x) * vx + (pose.y - a.y) * vy) / len2;
    if (t < 0 || t > 1) continue;
    const px = a.x + vx * t;
    const py = a.y + vy * t;
    const len = Math.sqrt(len2);
    const cross = (vx * (pose.y - py) - vy * (pose.x - px)) / len;
    s = a.s + t * (b.s - a.s);
    ey = cross;
    ref = { ...a, x: px, y: py, s, kappa: a.kappa + (b.kappa - a.kappa) * t, theta: a.theta + angleDiff(b.theta, a.theta) * t };
    found = true;
    break;
  }
  if (!found) {
    // 在端点外：横向误差取到端点切线的距离
    const p = points[best];
    const tx = Math.cos(p.theta) * p.dir;
    const ty = Math.sin(p.theta) * p.dir;
    ey = tx * (pose.y - p.y) - ty * (pose.x - p.x);
    s = p.s + tx * (pose.x - p.x) + ty * (pose.y - p.y);
  }
  // ey 以“运动方向左侧”为正，换算到车身坐标系（倒车时左右互换）
  return { index: best, s, ref, ey, eTheta: angleDiff(pose.theta, ref.theta) };
}

/** 沿路径取弧长 s 处的点；超出末端时沿末端切线延长，保证末端收敛稳定 */
export function pointAtS(points, s) {
  const last = points[points.length - 1];
  if (s >= last.s) {
    const extra = s - last.s;
    const dirX = Math.cos(last.theta) * last.dir;
    const dirY = Math.sin(last.theta) * last.dir;
    return { x: last.x + dirX * extra, y: last.y + dirY * extra, theta: last.theta };
  }
  let lo = 0;
  let hi = points.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (points[mid].s <= s) lo = mid;
    else hi = mid;
  }
  const a = points[lo];
  const b = points[hi];
  const t = (s - a.s) / Math.max(1e-9, b.s - a.s);
  return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t, theta: a.theta };
}

export function purePursuit({ pose, v, dir, points, projection, vehicle, config }) {
  const cfg = { ...DEFAULT_CONTROL_CONFIG, ...config };
  const ld = clamp(cfg.lookaheadBase + cfg.lookaheadGain * Math.abs(v), cfg.lookaheadMin, cfg.lookaheadMax);
  const target = pointAtS(points, projection.s + ld);
  const local = toLocal(pose, target.x, target.y);
  // 运动坐标系：倒车时绕 z 轴旋转 π
  const mx = dir > 0 ? local.x : -local.x;
  const my = dir > 0 ? local.y : -local.y;
  const dist = Math.max(0.3, Math.hypot(mx, my));
  const alpha = Math.atan2(my, mx);
  const kappa = (2 * Math.sin(alpha)) / dist;
  const delta = dir > 0 ? Math.atan(vehicle.wheelbase * kappa) : -Math.atan(vehicle.wheelbase * kappa);
  return { delta, target, lookahead: ld, alpha };
}

export function rearWheelFeedback({ pose, dir, projection, vehicle, config }) {
  const cfg = { ...DEFAULT_CONTROL_CONFIG, ...config };
  const ref = projection.ref;
  // 在“运动坐标系”中设计（倒车时航向 +π，二者之差不变）
  const ePsi = angleDiff(pose.theta, ref.theta);
  const ey = projection.ey;
  // 存储的 κ 为转向曲率，换算为运动方向曲率
  const kRef = dir * (ref.kappa || 0);
  const sinc = Math.abs(ePsi) < 1e-4 ? 1 : Math.sin(ePsi) / ePsi;
  // 李雅普诺夫函数 V = k_y·e_y²/2 + e_ψ²/2，下式使 V̇ = −k_θ·|v|·e_ψ² ≤ 0
  const kappaMotion =
    (kRef * Math.cos(ePsi)) / Math.max(0.2, 1 - kRef * ey) - cfg.kTheta * ePsi - cfg.kY * ey * sinc;
  const delta = Math.atan(vehicle.wheelbase * dir * kappaMotion);
  return { delta, target: null, lookahead: 0, ePsi, ey };
}
