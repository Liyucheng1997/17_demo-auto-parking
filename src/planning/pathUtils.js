/**
 * 路径工具：按挡位切分、重采样、计算弧长/曲率/前轮转角。
 *
 * 曲率采用“转向曲率”定义 κ = tanδ / L = dθ / (dir·ds)，
 * 这样前进/倒车时 κ 与前轮转角 δ 的符号关系一致，可直接换算 δ = atan(L·κ)。
 */

import { angleDiff, normalizeAngle } from "../core/math.js";

/** 把路径按行驶方向切分为若干段（相邻段共享换挡点 cusp） */
export function splitByDirection(path) {
  if (path.length < 2) return [];
  const segments = [];
  let current = [path[0]];
  for (let i = 1; i < path.length; i += 1) {
    const p = path[i];
    if (p.dir !== current[current.length - 1].dir && current.length > 1) {
      segments.push(finishSegment(current));
      const cusp = current[current.length - 1];
      current = [{ ...cusp, dir: p.dir }];
    }
    current.push(p);
  }
  if (current.length > 1) segments.push(finishSegment(current));
  return segments;
}

function finishSegment(points) {
  const dir = points[points.length - 1].dir;
  return { dir, points: points.map((p) => ({ ...p, dir })) };
}

/** 按固定间距重采样（保持首尾点不变，位姿航向沿用原始插值） */
export function resample(points, spacing) {
  const out = [{ ...points[0] }];
  let carry = 0;
  for (let i = 1; i < points.length; i += 1) {
    const a = points[i - 1];
    const b = points[i];
    const len = Math.hypot(b.x - a.x, b.y - a.y);
    let d = spacing - carry;
    while (d <= len - 1e-9) {
      const t = d / len;
      out.push({
        ...a,
        x: a.x + (b.x - a.x) * t,
        y: a.y + (b.y - a.y) * t,
        theta: normalizeAngle(a.theta + angleDiff(b.theta, a.theta) * t),
      });
      d += spacing;
    }
    carry = len - (d - spacing);
  }
  const last = points[points.length - 1];
  const tail = out[out.length - 1];
  if (Math.hypot(tail.x - last.x, tail.y - last.y) < spacing * 0.35 && out.length > 1) out.pop();
  out.push({ ...last });
  return out;
}

/**
 * 计算段内累计弧长 s、曲率 κ、前轮转角 δ。
 * 曲率用三点外接圆（Menger 曲率）由位置直接求得，κ = 4·面积 / (|ab|·|bc|·|ca|)，
 * 不依赖航向插值，端点取相邻点的值。
 */
export function annotateSegment(points, dir, wheelbase) {
  let s = 0;
  points[0].s = 0;
  for (let i = 1; i < points.length; i += 1) {
    s += Math.hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
    points[i].s = s;
  }
  const n = points.length;
  for (let i = 1; i < n - 1; i += 1) {
    points[i].kappa = dir * mengerCurvature(points[i - 1], points[i], points[i + 1]);
  }
  if (n >= 3) {
    points[0].kappa = points[1].kappa;
    points[n - 1].kappa = points[n - 2].kappa;
  } else {
    for (const p of points) p.kappa = 0;
  }
  for (const p of points) p.steer = Math.atan(wheelbase * p.kappa);
  return s;
}

/** 有符号三点曲率（沿 a→b→c 左转为正） */
export function mengerCurvature(a, b, c) {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const bcx = c.x - b.x;
  const bcy = c.y - b.y;
  const cross = abx * bcy - aby * bcx;
  const denom = Math.hypot(abx, aby) * Math.hypot(bcx, bcy) * Math.hypot(c.x - a.x, c.y - a.y);
  return denom > 1e-12 ? (2 * cross) / denom : 0;
}

/** 根据点列几何重算航向（倒车时航向 = 切线方向 + π），首尾航向保持不变 */
export function recomputeHeadings(points, dir) {
  for (let i = 1; i < points.length - 1; i += 1) {
    const a = points[i - 1];
    const b = points[i + 1];
    const tangent = Math.atan2(b.y - a.y, b.x - a.x);
    points[i].theta = normalizeAngle(dir > 0 ? tangent : tangent + Math.PI);
  }
}

export function pathLength(points) {
  let s = 0;
  for (let i = 1; i < points.length; i += 1) s += Math.hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
  return s;
}
