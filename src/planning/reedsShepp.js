/**
 * Reeds-Shepp 曲线
 *
 * 对于只能以最小转弯半径 ρ 转向、可前进也可倒车的车辆，任意两位姿间的
 * 最短路径一定属于 48 种“词”之一，由圆弧 (L/R) 与直线 (S) 组合而成：
 *   CSC、CCC、CCCC、CCSC、CCSCC（及其时间翻转、镜像、反向变换）。
 * 公式编号对应 Reeds & Shepp, 1990, "Optimal paths for a car that goes
 * both forwards and backwards"，实现思路参考 OMPL ReedsSheppStateSpace。
 *
 * 在 Hybrid A* 中 RS 曲线有两个用途：
 *   1. 无障碍非完整约束启发函数 h1 = RS 最短长度；
 *   2. 解析扩展 (analytic expansion)：直接尝试用 RS 曲线连接到目标。
 */

import { normalizeAngle } from "../core/math.js";

const PI = Math.PI;
const HALF_PI = PI / 2;
const ZERO = 1e-10;

const mod2pi = (x) => {
  let v = x % (2 * PI);
  if (v < -PI) v += 2 * PI;
  else if (v > PI) v -= 2 * PI;
  return v;
};

const polar = (x, y) => [Math.hypot(x, y), Math.atan2(y, x)];

function tauOmega(u, v, xi, eta, phi) {
  const delta = mod2pi(u - v);
  const A = Math.sin(u) - Math.sin(delta);
  const B = Math.cos(u) - Math.cos(delta) - 1;
  const t1 = Math.atan2(eta * A - xi * B, xi * A + eta * B);
  const t2 = 2 * (Math.cos(delta) - Math.cos(v) - Math.cos(u)) + 3;
  const tau = t2 < 0 ? mod2pi(t1 + PI) : mod2pi(t1);
  const omega = mod2pi(tau - u + v - phi);
  return [tau, omega];
}

// —— 基础公式（均以 L+ 开头，其余词由对称变换得到）——

// 8.1  L+ S+ L+
function LpSpLp(x, y, phi) {
  const [u, t] = polar(x - Math.sin(phi), y - 1 + Math.cos(phi));
  if (t >= -ZERO) {
    const v = mod2pi(phi - t);
    if (v >= -ZERO) return [t, u, v];
  }
  return null;
}

// 8.2  L+ S+ R+
function LpSpRp(x, y, phi) {
  const [u1raw, t1] = polar(x + Math.sin(phi), y - 1 - Math.cos(phi));
  const u1 = u1raw * u1raw;
  if (u1 >= 4) {
    const u = Math.sqrt(u1 - 4);
    const theta = Math.atan2(2, u);
    const t = mod2pi(t1 + theta);
    const v = mod2pi(t - phi);
    if (t >= -ZERO && v >= -ZERO) return [t, u, v];
  }
  return null;
}

// 8.3 / 8.4  L+ R- L+
function LpRmL(x, y, phi) {
  const xi = x - Math.sin(phi);
  const eta = y - 1 + Math.cos(phi);
  const [u1, theta] = polar(xi, eta);
  if (u1 <= 4) {
    const u = -2 * Math.asin(0.25 * u1);
    const t = mod2pi(theta + 0.5 * u + PI);
    const v = mod2pi(phi - t + u);
    if (t >= -ZERO && u <= ZERO) return [t, u, v];
  }
  return null;
}

// 8.7  L+ R+ L- R-
function LpRupLumRm(x, y, phi) {
  const xi = x + Math.sin(phi);
  const eta = y - 1 - Math.cos(phi);
  const rho = 0.25 * (2 + Math.hypot(xi, eta));
  if (rho <= 1) {
    const u = Math.acos(rho);
    const [t, v] = tauOmega(u, -u, xi, eta, phi);
    if (t >= -ZERO && v <= ZERO) return [t, u, v];
  }
  return null;
}

// 8.8  L+ R- L- R+
function LpRumLumRp(x, y, phi) {
  const xi = x + Math.sin(phi);
  const eta = y - 1 - Math.cos(phi);
  const rho = (20 - xi * xi - eta * eta) / 16;
  if (rho >= 0 && rho <= 1) {
    const u = -Math.acos(rho);
    if (u >= -HALF_PI) {
      const [t, v] = tauOmega(u, u, xi, eta, phi);
      if (t >= -ZERO && v >= -ZERO) return [t, u, v];
    }
  }
  return null;
}

// 8.9  L+ R- S- L-
function LpRmSmLm(x, y, phi) {
  const xi = x - Math.sin(phi);
  const eta = y - 1 + Math.cos(phi);
  const [rho, theta] = polar(xi, eta);
  if (rho >= 2) {
    const r = Math.sqrt(rho * rho - 4);
    const u = 2 - r;
    const t = mod2pi(theta + Math.atan2(r, -2));
    const v = mod2pi(phi - HALF_PI - t);
    if (t >= -ZERO && u <= ZERO && v <= ZERO) return [t, u, v];
  }
  return null;
}

// 8.10  L+ R- S- R-
function LpRmSmRm(x, y, phi) {
  const xi = x + Math.sin(phi);
  const eta = y - 1 - Math.cos(phi);
  const [rho, theta] = polar(-eta, xi);
  if (rho >= 2) {
    const t = theta;
    const u = 2 - rho;
    const v = mod2pi(t + HALF_PI - phi);
    if (t >= -ZERO && u <= ZERO && v <= ZERO) return [t, u, v];
  }
  return null;
}

// 8.11  L+ R- S- L- R+
function LpRmSLmRp(x, y, phi) {
  const xi = x + Math.sin(phi);
  const eta = y - 1 - Math.cos(phi);
  const [rho] = polar(xi, eta);
  if (rho >= 2) {
    const u = 4 - Math.sqrt(rho * rho - 4);
    if (u <= ZERO) {
      const t = mod2pi(Math.atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
      const v = mod2pi(t - phi);
      if (t >= -ZERO && v >= -ZERO) return [t, u, v];
    }
  }
  return null;
}

// —— 对称变换 ——
// timeflip: (x, y, φ) → (-x, y, -φ)，所有段长取反（前进 ↔ 倒车）
// reflect : (x, y, φ) → (x, -y, -φ)，L ↔ R
const FLIP_L_R = { L: "R", R: "L", S: "S" };

function pushCandidate(out, types, lengths, reflect) {
  const segTypes = reflect ? types.map((t) => FLIP_L_R[t]) : types;
  const segments = [];
  for (let i = 0; i < segTypes.length; i += 1) {
    if (Math.abs(lengths[i]) > 1e-10) segments.push({ type: segTypes[i], length: lengths[i] });
  }
  if (!segments.length) return;
  const total = segments.reduce((sum, s) => sum + Math.abs(s.length), 0);
  out.push({ word: segTypes.join(""), segments, length: total });
}

/** 对一个基础公式应用 4 种变换（原始 / 时间翻转 / 镜像 / 两者） */
function applyTransforms(out, formula, x, y, phi, types, build) {
  const variants = [
    [x, y, phi, false, false],
    [-x, y, -phi, true, false],
    [x, -y, -phi, false, true],
    [-x, -y, phi, true, true],
  ];
  for (const [vx, vy, vphi, timeflip, reflect] of variants) {
    const r = formula(vx, vy, vphi);
    if (!r) continue;
    const sign = timeflip ? -1 : 1;
    const lengths = build(r).map((l) => l * sign);
    pushCandidate(out, types, lengths, reflect);
  }
}

function CSC(out, x, y, phi) {
  applyTransforms(out, LpSpLp, x, y, phi, ["L", "S", "L"], ([t, u, v]) => [t, u, v]);
  applyTransforms(out, LpSpRp, x, y, phi, ["L", "S", "R"], ([t, u, v]) => [t, u, v]);
}

function CCC(out, x, y, phi) {
  applyTransforms(out, LpRmL, x, y, phi, ["L", "R", "L"], ([t, u, v]) => [t, u, v]);
  // backwards: 反向行驶同一路径（段顺序倒置）
  const xb = x * Math.cos(phi) + y * Math.sin(phi);
  const yb = x * Math.sin(phi) - y * Math.cos(phi);
  applyTransforms(out, LpRmL, xb, yb, phi, ["L", "R", "L"], ([t, u, v]) => [v, u, t]);
}

function CCCC(out, x, y, phi) {
  applyTransforms(out, LpRupLumRm, x, y, phi, ["L", "R", "L", "R"], ([t, u, v]) => [t, u, -u, v]);
  applyTransforms(out, LpRumLumRp, x, y, phi, ["L", "R", "L", "R"], ([t, u, v]) => [t, u, u, v]);
}

function CCSC(out, x, y, phi) {
  applyTransforms(out, LpRmSmLm, x, y, phi, ["L", "R", "S", "L"], ([t, u, v]) => [t, -HALF_PI, u, v]);
  applyTransforms(out, LpRmSmRm, x, y, phi, ["L", "R", "S", "R"], ([t, u, v]) => [t, -HALF_PI, u, v]);
  const xb = x * Math.cos(phi) + y * Math.sin(phi);
  const yb = x * Math.sin(phi) - y * Math.cos(phi);
  applyTransforms(out, LpRmSmLm, xb, yb, phi, ["L", "S", "R", "L"], ([t, u, v]) => [v, u, -HALF_PI, t]);
  applyTransforms(out, LpRmSmRm, xb, yb, phi, ["R", "S", "R", "L"], ([t, u, v]) => [v, u, -HALF_PI, t]);
}

function CCSCC(out, x, y, phi) {
  applyTransforms(
    out,
    LpRmSLmRp,
    x,
    y,
    phi,
    ["L", "R", "S", "L", "R"],
    ([t, u, v]) => [t, -HALF_PI, u, -HALF_PI, v],
  );
}

/**
 * 计算从 start 到 goal 的全部可行 RS 路径（已按长度升序排列）。
 * 段长单位为米，正值前进、负值倒车。
 */
export function reedsSheppPaths(start, goal, radius) {
  const dx = goal.x - start.x;
  const dy = goal.y - start.y;
  const c = Math.cos(start.theta);
  const s = Math.sin(start.theta);
  const x = (c * dx + s * dy) / radius;
  const y = (-s * dx + c * dy) / radius;
  const phi = normalizeAngle(goal.theta - start.theta);

  const out = [];
  CSC(out, x, y, phi);
  CCC(out, x, y, phi);
  CCCC(out, x, y, phi);
  CCSC(out, x, y, phi);
  CCSCC(out, x, y, phi);

  for (const path of out) {
    path.length *= radius;
    for (const seg of path.segments) seg.length *= radius;
  }
  out.sort((a, b) => a.length - b.length);
  return out;
}

const abs3 = ([t, u, v]) => Math.abs(t) + Math.abs(u) + Math.abs(v);
const abs4 = ([t, u, v]) => Math.abs(t) + 2 * Math.abs(u) + Math.abs(v);
const absCCSC = ([t, u, v]) => Math.abs(t) + HALF_PI + Math.abs(u) + Math.abs(v);
const absCCSCC = ([t, u, v]) => Math.abs(t) + PI + Math.abs(u) + Math.abs(v);

function minOver(formula, x, y, phi, measure) {
  let best = Infinity;
  let r = formula(x, y, phi);
  if (r) best = Math.min(best, measure(r));
  r = formula(-x, y, -phi);
  if (r) best = Math.min(best, measure(r));
  r = formula(x, -y, -phi);
  if (r) best = Math.min(best, measure(r));
  r = formula(-x, -y, phi);
  if (r) best = Math.min(best, measure(r));
  return best;
}

/**
 * RS 最短路径长度。只计算长度、不构造路径对象，
 * 比 reedsSheppPaths 快约 5 倍，供 Hybrid A* 启发函数高频调用。
 */
export function reedsSheppDistance(start, goal, radius) {
  const dx = goal.x - start.x;
  const dy = goal.y - start.y;
  const c = Math.cos(start.theta);
  const s = Math.sin(start.theta);
  const x = (c * dx + s * dy) / radius;
  const y = (-s * dx + c * dy) / radius;
  const phi = normalizeAngle(goal.theta - start.theta);
  const xb = x * Math.cos(phi) + y * Math.sin(phi);
  const yb = x * Math.sin(phi) - y * Math.cos(phi);
  const best = Math.min(
    minOver(LpSpLp, x, y, phi, abs3),
    minOver(LpSpRp, x, y, phi, abs3),
    minOver(LpRmL, x, y, phi, abs3),
    minOver(LpRmL, xb, yb, phi, abs3),
    minOver(LpRupLumRm, x, y, phi, abs4),
    minOver(LpRumLumRp, x, y, phi, abs4),
    minOver(LpRmSmLm, x, y, phi, absCCSC),
    minOver(LpRmSmRm, x, y, phi, absCCSC),
    minOver(LpRmSmLm, xb, yb, phi, absCCSC),
    minOver(LpRmSmRm, xb, yb, phi, absCCSC),
    minOver(LpRmSLmRp, x, y, phi, absCCSCC),
  );
  return best * radius;
}

/**
 * 沿 RS 路径按步长 step 采样，返回带方向与曲率的路径点。
 * 每个点：{ x, y, theta, dir: ±1, kappa }
 */
export function sampleReedsShepp(start, path, radius, step = 0.1) {
  const points = [{ x: start.x, y: start.y, theta: start.theta, dir: Math.sign(path.segments[0].length) || 1, kappa: 0 }];
  let pose = { x: start.x, y: start.y, theta: start.theta };
  for (const seg of path.segments) {
    const dir = Math.sign(seg.length);
    const total = Math.abs(seg.length);
    const curvature = seg.type === "L" ? 1 / radius : seg.type === "R" ? -1 / radius : 0;
    const n = Math.max(1, Math.ceil(total / step));
    const ds = total / n;
    points[points.length - 1].kappa = curvature;
    for (let i = 0; i < n; i += 1) {
      pose = advance(pose, dir * ds, curvature);
      points.push({ ...pose, dir, kappa: curvature });
    }
  }
  return points;
}

function advance(pose, ds, curvature) {
  if (Math.abs(curvature) < 1e-12) {
    return { x: pose.x + ds * Math.cos(pose.theta), y: pose.y + ds * Math.sin(pose.theta), theta: pose.theta };
  }
  const theta = pose.theta + ds * curvature;
  const R = 1 / curvature;
  return {
    x: pose.x + R * (Math.sin(theta) - Math.sin(pose.theta)),
    y: pose.y - R * (Math.cos(theta) - Math.cos(pose.theta)),
    theta: normalizeAngle(theta),
  };
}

/** 计算 RS 路径的换挡次数 */
export function gearSwitches(path) {
  let count = 0;
  for (let i = 1; i < path.segments.length; i += 1) {
    if (Math.sign(path.segments[i].length) !== Math.sign(path.segments[i - 1].length)) count += 1;
  }
  return count;
}
