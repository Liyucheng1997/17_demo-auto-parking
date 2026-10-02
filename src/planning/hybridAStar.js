/**
 * Hybrid A* 路径搜索（Dolgov et al., 2008, "Practical Search Techniques in
 * Path Planning for Autonomous Driving"）。
 *
 * 与栅格 A* 的关键区别：
 *   - 节点保存连续位姿 (x, y, θ)，离散栅格 (x, y, θ) 只用于判重/剪枝；
 *   - 节点扩展使用车辆运动学基元：{前进, 倒车} × {N 个前轮转角} 的定长圆弧，
 *     得到的路径天然满足最小转弯半径约束；
 *   - 代价函数显式惩罚倒车、换挡、大转角与转角突变；
 *   - 启发函数 h = max(h_RS, h_2D)：
 *       h_RS：无障碍 Reeds-Shepp 最短长度（非完整约束），
 *       h_2D：有障碍 2D Dijkstra 距离（完整约束）；
 *   - 解析扩展：接近目标时尝试直接用 RS 曲线连接目标，命中即结束搜索。
 */

import { MinHeap, angleDiff, normalizeAngle } from "../core/math.js";
import { minTurningRadius, propagatePose } from "../model/vehicle.js";
import { gearSwitches, reedsSheppDistance, reedsSheppPaths, sampleReedsShepp } from "./reedsShepp.js";
import { lookupHeuristic } from "./heuristic.js";

export const DEFAULT_PLANNER_CONFIG = Object.freeze({
  xyResolution: 0.4,
  thetaBins: 72,
  stepLength: 0.9,
  collisionStep: 0.25,
  steerSamples: 5,
  reversePenalty: 1.6,
  gearSwitchPenalty: 4.0,
  steerPenalty: 0.25,
  steerChangePenalty: 0.5,
  heuristicWeight: 1.2,
  rsHeuristicRange: 22,
  analyticRange: 18,
  maxRsCandidates: 8,
  maxIterations: 60000,
  timeBudgetMs: 10000,
  recordTree: true,
});

/**
 * @param {object} p
 * @param {{x,y,theta}} p.start  起点（后轴中心）
 * @param {{x,y,theta}} p.goal   终点（后轴中心）
 * @param {object} p.vehicle     车辆参数
 * @param {import('./collision.js').CollisionChecker} p.checker
 * @param {object} p.heuristic2d buildHolonomicHeuristic 结果
 * @param {{width,height}} p.bounds
 */
export function hybridAStar({ start, goal, vehicle, checker, heuristic2d, bounds, config = {} }) {
  const cfg = { ...DEFAULT_PLANNER_CONFIG, ...config };
  const t0 = now();
  const radius = minTurningRadius(vehicle);
  const { xyResolution: res, thetaBins } = cfg;
  const cols = Math.ceil(bounds.width / res);
  const rows = Math.ceil(bounds.height / res);
  const thetaStep = (2 * Math.PI) / thetaBins;

  const steers = [];
  for (let i = 0; i < cfg.steerSamples; i += 1) {
    steers.push(-vehicle.maxSteer + (2 * vehicle.maxSteer * i) / (cfg.steerSamples - 1));
  }
  const primitives = [];
  for (const dir of [1, -1]) for (const steer of steers) primitives.push({ dir, steer });
  const subSteps = Math.max(1, Math.ceil(cfg.stepLength / cfg.collisionStep));

  const keyOf = (pose) => {
    const c = Math.floor(pose.x / res);
    const r = Math.floor(pose.y / res);
    if (c < 0 || r < 0 || c >= cols || r >= rows) return -1;
    const t = Math.floor((normalizeAngle(pose.theta) + Math.PI) / thetaStep) % thetaBins;
    return (t * rows + r) * cols + c;
  };

  const heuristic = (pose) => {
    const h2 = lookupHeuristic(heuristic2d, pose.x, pose.y);
    if (!Number.isFinite(h2)) return Infinity;
    const euclid = Math.hypot(goal.x - pose.x, goal.y - pose.y);
    const hRs = euclid < cfg.rsHeuristicRange ? reedsSheppDistance(pose, goal, radius) : 0;
    return Math.max(h2, hRs) * cfg.heuristicWeight;
  };

  const stats = {
    iterations: 0,
    generated: 0,
    collisionPruned: 0,
    rsAttempts: 0,
    rsCandidatesChecked: 0,
    collisionChecksStart: checker.checks,
  };
  const tree = [];
  const rsTrials = [];
  const closed = new Uint8Array(cols * rows * thetaBins);
  const bestG = new Map();
  const open = new MinHeap();

  const startNode = { x: start.x, y: start.y, theta: start.theta, g: 0, dir: 0, steer: 0, parent: null, trace: [] };
  if (checker.collides(start)) return failure("起点位姿与障碍物冲突", stats, tree, rsTrials, t0, checker);
  if (checker.collides(goal)) return failure("目标位姿与障碍物冲突（车位空间不足）", stats, tree, rsTrials, t0, checker);
  const h0 = heuristic(start);
  if (!Number.isFinite(h0)) return failure("目标不可达（2D 启发函数无通路）", stats, tree, rsTrials, t0, checker);
  open.push(startNode, h0);

  while (open.size) {
    if (stats.iterations >= cfg.maxIterations || now() - t0 > cfg.timeBudgetMs) {
      return failure("搜索超出迭代/时间预算", stats, tree, rsTrials, t0, checker);
    }
    const node = open.pop();
    const key = keyOf(node);
    if (closed[key]) continue;
    closed[key] = 1;
    stats.iterations += 1;
    if (cfg.recordTree && node.parent) tree.push(packTrace(node));

    // —— 解析扩展：尝试 Reeds-Shepp 直连目标 ——
    const dGoal = Math.hypot(goal.x - node.x, goal.y - node.y);
    if (dGoal < cfg.analyticRange) {
      const interval = Math.max(1, Math.floor(dGoal / 2));
      if (stats.iterations % interval === 0) {
        const shot = analyticExpansion(node, goal, radius, vehicle, checker, cfg, stats, rsTrials);
        if (shot) {
          return success(node, shot, stats, tree, rsTrials, t0, checker, vehicle);
        }
      }
    }
    if (dGoal < 0.1 && Math.abs(angleDiff(node.theta, goal.theta)) < 0.03) {
      return success(node, null, stats, tree, rsTrials, t0, checker, vehicle);
    }

    // —— 运动基元扩展 ——
    for (const prim of primitives) {
      const ds = (prim.dir * cfg.stepLength) / subSteps;
      const trace = [];
      let pose = node;
      let blocked = false;
      for (let i = 0; i < subSteps; i += 1) {
        pose = propagatePose(pose, ds, prim.steer, vehicle.wheelbase);
        if (checker.collides(pose)) {
          blocked = true;
          break;
        }
        trace.push(pose);
      }
      if (blocked) {
        stats.collisionPruned += 1;
        continue;
      }
      const childKey = keyOf(pose);
      if (childKey < 0 || childKey === key || closed[childKey]) continue;
      const g = node.g + transitionCost(node, prim, cfg, vehicle);
      const known = bestG.get(childKey);
      if (known !== undefined && known <= g) continue;
      const h = heuristic(pose);
      if (!Number.isFinite(h)) continue;
      bestG.set(childKey, g);
      stats.generated += 1;
      open.push({ x: pose.x, y: pose.y, theta: pose.theta, g, dir: prim.dir, steer: prim.steer, parent: node, trace }, g + h);
    }
  }
  return failure("开放列表耗尽：在当前约束下无可行路径", stats, tree, rsTrials, t0, checker);
}

function transitionCost(node, prim, cfg, vehicle) {
  const len = cfg.stepLength;
  let cost = len * (prim.dir < 0 ? cfg.reversePenalty : 1);
  if (node.dir !== 0 && node.dir !== prim.dir) cost += cfg.gearSwitchPenalty;
  cost += (cfg.steerPenalty * Math.abs(prim.steer) * len) / vehicle.maxSteer;
  cost += (cfg.steerChangePenalty * Math.abs(prim.steer - node.steer)) / vehicle.maxSteer;
  return cost;
}

/** 按与 Hybrid A* 相同的代价模型评估 RS 路径 */
function rsCost(node, path, cfg) {
  let cost = 0;
  let prevDir = node.dir;
  let prevSteer = node.steer === 0 ? 0 : Math.sign(node.steer);
  for (const seg of path.segments) {
    const dir = Math.sign(seg.length);
    const len = Math.abs(seg.length);
    const steer = seg.type === "L" ? 1 : seg.type === "R" ? -1 : 0;
    cost += len * (dir < 0 ? cfg.reversePenalty : 1);
    if (prevDir !== 0 && dir !== prevDir) cost += cfg.gearSwitchPenalty;
    cost += cfg.steerPenalty * Math.abs(steer) * len;
    cost += cfg.steerChangePenalty * Math.abs(steer - prevSteer);
    prevDir = dir;
    prevSteer = steer;
  }
  return cost;
}

function analyticExpansion(node, goal, radius, vehicle, checker, cfg, stats, rsTrials) {
  stats.rsAttempts += 1;
  const dGoal = Math.hypot(goal.x - node.x, goal.y - node.y);
  const candidates = reedsSheppPaths(node, goal, radius)
    .filter((p) => p.length < dGoal * 3 + 8)
    .map((p) => ({ path: p, cost: rsCost(node, p, cfg) }))
    .sort((a, b) => a.cost - b.cost)
    .slice(0, cfg.maxRsCandidates);

  for (const { path, cost } of candidates) {
    stats.rsCandidatesChecked += 1;
    const points = sampleReedsShepp(node, path, radius, cfg.collisionStep);
    const hit = checker.firstCollision(points);
    if (cfg.recordTree && rsTrials.length < 400) {
      rsTrials.push({ iteration: stats.iterations, points: packPoints(points, hit), word: path.word, success: hit < 0 });
    }
    if (hit < 0) {
      return { path, cost, points: sampleReedsShepp(node, path, radius, 0.1), radius };
    }
  }
  return null;
}

function packTrace(node) {
  const pts = new Float32Array((node.trace.length + 1) * 2);
  pts[0] = node.parent.x;
  pts[1] = node.parent.y;
  node.trace.forEach((p, i) => {
    pts[2 * i + 2] = p.x;
    pts[2 * i + 3] = p.y;
  });
  return { pts, dir: node.dir };
}

function packPoints(points, hit) {
  const end = hit >= 0 ? hit + 1 : points.length;
  const pts = new Float32Array(end * 2);
  for (let i = 0; i < end; i += 1) {
    pts[2 * i] = points[i].x;
    pts[2 * i + 1] = points[i].y;
  }
  return pts;
}

function success(node, shot, stats, tree, rsTrials, t0, checker, vehicle) {
  // 回溯 Hybrid A* 部分
  const chain = [];
  for (let n = node; n; n = n.parent) chain.push(n);
  chain.reverse();
  const path = [];
  const root = chain[0];
  const firstDir = chain[1]?.dir || (shot ? Math.sign(shot.path.segments[0].length) : 1);
  path.push({ x: root.x, y: root.y, theta: root.theta, dir: firstDir, steer: 0, source: "search" });
  for (let i = 1; i < chain.length; i += 1) {
    const n = chain[i];
    for (const p of n.trace) path.push({ x: p.x, y: p.y, theta: p.theta, dir: n.dir, steer: n.steer, source: "search" });
  }
  const searchCount = path.length;
  // 拼接解析扩展的 RS 段
  if (shot) {
    for (let i = 1; i < shot.points.length; i += 1) {
      const p = shot.points[i];
      path.push({
        x: p.x,
        y: p.y,
        theta: p.theta,
        dir: p.dir,
        steer: Math.atan(vehicle.wheelbase * p.kappa),
        source: "rs",
      });
    }
  }
  // 起点方向与第一段一致
  if (path.length > 1) path[0].dir = path[1].dir;

  return {
    success: true,
    path,
    searchPointCount: searchCount,
    analytic: shot ? { word: shot.path.word, segments: shot.path.segments, length: shot.path.length, gearSwitches: gearSwitches(shot.path) } : null,
    cost: node.g + (shot ? shot.cost : 0),
    stats: finishStats(stats, t0, checker),
    tree,
    rsTrials,
  };
}

function failure(reason, stats, tree, rsTrials, t0, checker) {
  return { success: false, reason, path: [], stats: finishStats(stats, t0, checker), tree, rsTrials };
}

function finishStats(stats, t0, checker) {
  const { collisionChecksStart, ...rest } = stats;
  return { ...rest, collisionChecks: checker.checks - collisionChecksStart, timeMs: now() - t0 };
}

function now() {
  return typeof performance !== "undefined" ? performance.now() : Date.now();
}
