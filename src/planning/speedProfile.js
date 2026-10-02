/**
 * 速度规划：在每个行驶段上生成满足约束的速度曲线 v(s)。
 *
 * 速度上限取以下约束的最小值：
 *   1. 挡位限速：前进 v_fwd，倒车 v_rev；
 *   2. 横向加速度：v ≤ √(a_lat / |κ|)；
 *   3. 转向速率：v ≤ δ̇_max / |dδ/ds| —— 泊车场景中最主要的约束，
 *      曲率变化越剧烈，车辆就要越慢，转向机才“打得过来”；
 *   4. 障碍物距离：离障碍物越近越慢，v ≤ v_min + k·clearance；
 *   5. 曲率减速：v ≤ v_gear·(1 − c·|κ|/κ_max)，大转角时降低车速以保证跟踪精度。
 * 上限曲线先做滑动窗口最小值滤波（形态学腐蚀），消除相邻约束之间“刚加速就刹车”的锯齿；
 * 然后做前向（加速度）/后向（减速度）两遍扫描，段首尾速度为 0（换挡停车）。
 */

import { clamp } from "../core/math.js";

export const DEFAULT_SPEED_CONFIG = Object.freeze({
  steerRateUsage: 0.6,
  clearanceGain: 1.2,
  curvatureSlowdown: 0.55,
  minSpeed: 0.25,
  minSteerSpeed: 0.06,
  erosionWindow: 1.2,
  gearShiftTime: 0.8,
  clearanceSampleStep: 4,
});

export function profileSegment(segment, { vehicle, checker, config = {} }) {
  const cfg = { ...DEFAULT_SPEED_CONFIG, ...config };
  const pts = segment.points;
  const n = pts.length;
  const vLimit = segment.dir > 0 ? vehicle.maxSpeedForward : vehicle.maxSpeedReverse;
  const limits = new Array(n).fill(vLimit);
  const reasons = new Array(n).fill("gear");

  const kappaMax = Math.tan(vehicle.maxSteer) / vehicle.wheelbase;
  let clearance = Infinity;
  for (let i = 0; i < n; i += 1) {
    const p = pts[i];
    const k = Math.abs(p.kappa);
    if (k > 1e-6) {
      const vLat = Math.sqrt(vehicle.maxLateralAccel / k);
      if (vLat < limits[i]) {
        limits[i] = vLat;
        reasons[i] = "lateral";
      }
      const vCurv = vLimit * (1 - cfg.curvatureSlowdown * Math.min(1, k / kappaMax));
      if (vCurv < limits[i]) {
        limits[i] = vCurv;
        reasons[i] = "curvature";
      }
    }
    const a = pts[Math.max(0, i - 1)];
    const b = pts[Math.min(n - 1, i + 1)];
    const dDelta = Math.abs(b.steer - a.steer) / Math.max(1e-6, b.s - a.s);
    if (dDelta > 1e-6) {
      const vSteer = (vehicle.maxSteerRate * cfg.steerRateUsage) / dDelta;
      if (vSteer < limits[i]) {
        limits[i] = vSteer;
        reasons[i] = "steer-rate";
      }
    }
    if (checker && i % cfg.clearanceSampleStep === 0) clearance = checker.clearance(p, 2.5).distance;
    p.clearance = clearance;
    const vClear = cfg.minSpeed + cfg.clearanceGain * clearance;
    if (vClear < limits[i]) {
      limits[i] = vClear;
      reasons[i] = "clearance";
    }
    // 转向速率约束允许低于蠕行速度（必要时几乎原地打方向）
    limits[i] = Math.max(reasons[i] === "steer-rate" ? cfg.minSteerSpeed : cfg.minSpeed, limits[i]);
  }

  // 滑动窗口最小值滤波（窗口按弧长计）
  const eroded = limits.slice();
  const reasonsEroded = reasons.slice();
  const half = cfg.erosionWindow / 2;
  let lo = 0;
  for (let i = 0; i < n; i += 1) {
    while (pts[i].s - pts[lo].s > half) lo += 1;
    for (let j = lo; j < n && pts[j].s - pts[i].s <= half; j += 1) {
      if (limits[j] < eroded[i]) {
        eroded[i] = limits[j];
        reasonsEroded[i] = reasons[j];
      }
    }
  }
  for (let i = 0; i < n; i += 1) {
    limits[i] = eroded[i];
    reasons[i] = reasonsEroded[i];
  }

  const v = limits.slice();
  v[0] = 0;
  v[n - 1] = 0;
  for (let i = 1; i < n; i += 1) {
    const ds = pts[i].s - pts[i - 1].s;
    v[i] = Math.min(v[i], Math.sqrt(v[i - 1] ** 2 + 2 * vehicle.maxAccel * 0.8 * ds));
  }
  for (let i = n - 2; i >= 0; i -= 1) {
    const ds = pts[i + 1].s - pts[i].s;
    v[i] = Math.min(v[i], Math.sqrt(v[i + 1] ** 2 + 2 * vehicle.maxDecel * 0.6 * ds));
  }

  let t = 0;
  pts[0].t = 0;
  for (let i = 0; i < n; i += 1) {
    pts[i].vLimit = limits[i];
    pts[i].limitReason = reasons[i];
    pts[i].v = v[i];
    if (i > 0) {
      const ds = pts[i].s - pts[i - 1].s;
      t += (2 * ds) / Math.max(1e-3, v[i] + v[i - 1]);
      pts[i].t = t;
    }
  }
  return { duration: t, maxSpeed: Math.max(...v) };
}

/** 生成全程轨迹（附加全局弧长、全局时间，换挡停顿计入时间） */
export function buildTrajectory(segments, options) {
  const cfg = { ...DEFAULT_SPEED_CONFIG, ...(options.config || {}) };
  let tOffset = 0;
  let sOffset = 0;
  const out = [];
  segments.forEach((seg, k) => {
    const { duration } = profileSegment(seg, options);
    seg.tStart = tOffset;
    seg.sStart = sOffset;
    seg.length = seg.points[seg.points.length - 1].s;
    seg.duration = duration;
    for (const p of seg.points) {
      out.push({ ...p, segment: k, sGlobal: sOffset + p.s, tGlobal: tOffset + p.t });
    }
    tOffset += duration + (k < segments.length - 1 ? cfg.gearShiftTime : 0);
    sOffset += seg.length;
  });
  return { points: out, duration: tOffset, length: sOffset };
}

/** 段内按弧长插值查询参考量 */
export function sampleAt(points, s) {
  if (s <= 0) return points[0];
  const last = points[points.length - 1];
  if (s >= last.s) return last;
  let lo = 0;
  let hi = points.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (points[mid].s <= s) lo = mid;
    else hi = mid;
  }
  const a = points[lo];
  const b = points[hi];
  const t = clamp((s - a.s) / Math.max(1e-9, b.s - a.s), 0, 1);
  return { ...a, v: a.v + (b.v - a.v) * t, kappa: a.kappa + (b.kappa - a.kappa) * t, s };
}
