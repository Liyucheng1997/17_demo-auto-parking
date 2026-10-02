/**
 * 路径平滑：分段（以换挡点为界）做带约束的梯度下降。
 *
 * 目标函数：J = w_s·Σ‖x_{i-1} − 2x_i + x_{i+1}‖² + w_d·Σ‖x_i − x_i⁰‖²
 *   - 平滑项 w_s：抑制 Hybrid A* 基元拼接处的转角阶跃；
 *   - 保真项 w_d：防止路径偏离已验证安全的搜索结果太远。
 * 约束（平滑后逐点校验，不满足则减弱平滑强度重试，最终可回退到原路径）：
 *   - 首尾各两点固定 ⇒ 段端位姿与换挡点位姿不变；
 *   - 全程车身外廓无碰撞；
 *   - |κ| ≤ κ_max·(1 + tol)，保证车辆可跟踪。
 */

import { annotateSegment, recomputeHeadings, resample } from "./pathUtils.js";

export const DEFAULT_SMOOTHER_CONFIG = Object.freeze({
  spacing: 0.2,
  smoothWeight: 0.3,
  dataWeight: 0.08,
  iterations: 250,
  curvatureTolerance: 0.03,
  endRamp: 6,
  attempts: 4,
});

export function smoothSegment(segment, { checker, vehicle, config = {} }) {
  const cfg = { ...DEFAULT_SMOOTHER_CONFIG, ...config };
  const dir = segment.dir;
  const base = resample(segment.points, cfg.spacing);
  const kappaMax = Math.tan(vehicle.maxSteer) / vehicle.wheelbase;

  if (base.length < 6) {
    annotateSegment(base, dir, vehicle.wheelbase);
    return { dir, points: base, smoothed: false, attempt: 0 };
  }

  // 原始路径在基元拼接点处的三点曲率可能略超 κ_max，平滑结果不得比它更差
  const reference = base.map((p) => ({ ...p }));
  annotateSegment(reference, dir, vehicle.wheelbase);
  const kappaLimit = Math.max(
    kappaMax * (1 + cfg.curvatureTolerance),
    ...reference.map((p) => Math.abs(p.kappa)),
  );

  let ws = cfg.smoothWeight;
  for (let attempt = 1; attempt <= cfg.attempts; attempt += 1) {
    const pts = gradientDescent(base, ws, cfg.dataWeight, cfg.iterations, cfg.endRamp);
    recomputeHeadings(pts, dir);
    annotateSegment(pts, dir, vehicle.wheelbase);
    const curvatureOk = pts.every((p) => Math.abs(p.kappa) <= kappaLimit);
    const collisionFree = curvatureOk && checker.firstCollision(pts) < 0;
    if (curvatureOk && collisionFree) {
      return { dir, points: pts, smoothed: true, attempt, smoothWeight: ws };
    }
    ws *= 0.4;
  }
  annotateSegment(base, dir, vehicle.wheelbase);
  return { dir, points: base, smoothed: false, attempt: cfg.attempts };
}

function gradientDescent(base, ws, wd, iterations, ramp) {
  const n = base.length;
  const x = base.map((p) => p.x);
  const y = base.map((p) => p.y);
  const x0 = x.slice();
  const y0 = y.slice();
  // 平滑权重在段两端线性渐入：端点附近保持原始（已验证可行的）几何，避免端部扭折
  const w = new Float64Array(n);
  for (let i = 0; i < n; i += 1) w[i] = ws * Math.min(1, Math.min(i - 1, n - 2 - i) / ramp);
  for (let k = 0; k < iterations; k += 1) {
    for (let i = 2; i < n - 2; i += 1) {
      x[i] += w[i] * (x[i - 1] + x[i + 1] - 2 * x[i]) + wd * (x0[i] - x[i]);
      y[i] += w[i] * (y[i - 1] + y[i + 1] - 2 * y[i]) + wd * (y0[i] - y[i]);
    }
  }
  return base.map((p, i) => ({ ...p, x: x[i], y: y[i] }));
}
