/**
 * 泊车规划流水线（纯函数，可在主线程或 Web Worker 中运行）：
 *
 *   ① 环境建模   障碍物 → 占据栅格 + 欧氏距离场
 *   ② 目标位姿   车位几何 + 泊车方式 → 后轴中心目标位姿；垂直车位另设“预目标”，
 *                最后一段为直线入库，保证航向在停车前收敛
 *   ③ 启发函数   有障碍 2D Dijkstra 代价图
 *   ④ 路径搜索   Hybrid A* + Reeds-Shepp 解析扩展
 *   ⑤ 路径平滑   按挡位分段、梯度下降平滑 + 碰撞/曲率校验
 *   ⑥ 速度规划   多约束速度上限 + 前后向扫描
 */

import { createVehicleParams, minTurningRadius } from "../model/vehicle.js";
import { buildGridMap } from "../model/gridMap.js";
import { findSlot, sceneObstacles } from "../model/parkingLot.js";
import { CollisionChecker } from "./collision.js";
import { goalPoseForSlot, PARKING_MODES } from "./goal.js";
import { buildHolonomicHeuristic } from "./heuristic.js";
import { hybridAStar } from "./hybridAStar.js";
import { splitByDirection } from "./pathUtils.js";
import { smoothSegment } from "./smoother.js";
import { buildTrajectory } from "./speedProfile.js";

export const DEFAULT_SAFETY_MARGIN = 0.15;
/** 规划只使用 88% 的最大前轮转角，为跟踪控制预留纠偏余量 */
export const DEFAULT_STEER_USAGE = 0.88;

/** 构建规划/仿真共用的环境：障碍物、栅格地图、碰撞检测器 */
export function buildEnvironment(lot, vehicle, { margin = DEFAULT_SAFETY_MARGIN, resolution = 0.2 } = {}) {
  const obstacles = sceneObstacles(lot);
  const gridMap = buildGridMap(obstacles, lot, resolution);
  const checker = new CollisionChecker(obstacles, vehicle, { margin, gridMap, bounds: lot });
  return { obstacles, gridMap, checker };
}

export function planParking({
  lot,
  slotId,
  start = lot.start,
  vehicle = createVehicleParams(),
  mode = PARKING_MODES.REVERSE_IN,
  margin = DEFAULT_SAFETY_MARGIN,
  steerUsage = DEFAULT_STEER_USAGE,
  searchConfig = {},
  smootherConfig = {},
  speedConfig = {},
  smoothing = true,
  finalStraight = 1.0,
}) {
  const stages = [];
  const timed = (key, name, fn) => {
    const t = now();
    const value = fn();
    stages.push({ key, name, ms: now() - t });
    return value;
  };

  const slot = findSlot(lot, slotId);
  if (!slot) throw new Error(`未知车位 ${slotId}`);

  const planVehicle = createVehicleParams({ ...vehicle, maxSteer: vehicle.maxSteer * steerUsage });
  const env = timed("env", "环境建模", () => buildEnvironment(lot, vehicle, { margin }));
  const { goal, preGoal, entryDir } = timed("goal", "目标位姿", () => {
    const g = goalPoseForSlot(slot, vehicle, { mode });
    if (slot.type !== "perpendicular" || finalStraight <= 0) return { goal: g, preGoal: g, entryDir: 0 };
    // 倒车入库：最终段倒车 (dir=-1)，预目标在车头方向外侧；车头入库则相反
    const dir = mode === PARKING_MODES.HEAD_IN ? 1 : -1;
    const pre = {
      x: g.x - dir * finalStraight * Math.cos(g.theta),
      y: g.y - dir * finalStraight * Math.sin(g.theta),
      theta: g.theta,
    };
    return { goal: g, preGoal: pre, entryDir: dir };
  });
  const heuristic2d = timed("heuristic", "启发函数", () =>
    buildHolonomicHeuristic(env.gridMap, preGoal, vehicle.width / 2 - 0.25),
  );
  const search = timed("search", "Hybrid A*", () =>
    hybridAStar({
      start,
      goal: preGoal,
      vehicle: planVehicle,
      checker: env.checker,
      heuristic2d,
      bounds: lot,
      config: searchConfig,
    }),
  );
  if (search.success && entryDir !== 0) {
    // 追加直线入库段
    const n = Math.max(2, Math.ceil(finalStraight / 0.1));
    for (let i = 1; i <= n; i += 1) {
      const t = i / n;
      search.path.push({
        x: preGoal.x + (goal.x - preGoal.x) * t,
        y: preGoal.y + (goal.y - preGoal.y) * t,
        theta: goal.theta,
        dir: entryDir,
        steer: 0,
        source: "entry",
      });
    }
  }

  const base = {
    slotId,
    mode,
    start,
    goal,
    preGoal,
    vehicle,
    planVehicle,
    margin,
    radius: minTurningRadius(planVehicle),
    gridMap: env.gridMap,
    heuristic2d,
    search,
    stages,
  };
  if (!search.success) return { ...base, success: false, reason: search.reason };

  const rawSegments = splitByDirection(search.path);
  const segments = timed("smooth", "路径平滑", () =>
    rawSegments.map((seg) =>
      smoothSegment(seg, {
        checker: env.checker,
        vehicle: planVehicle,
        config: smoothing ? smootherConfig : { ...smootherConfig, attempts: 0 },
      }),
    ),
  );
  const trajectory = timed("speed", "速度规划", () =>
    buildTrajectory(segments, { vehicle, checker: env.checker, config: speedConfig }),
  );

  const minClearance = Math.min(...trajectory.points.map((p) => p.clearance ?? Infinity));
  return {
    ...base,
    success: true,
    rawPath: search.path,
    segments,
    trajectory,
    metrics: {
      length: trajectory.length,
      duration: trajectory.duration,
      gearShifts: segments.length - 1,
      segments: segments.length,
      minClearance,
      maxAbsSteer: Math.max(...trajectory.points.map((p) => Math.abs(p.steer))),
      smoothedSegments: segments.filter((s) => s.smoothed).length,
      planningMs: stages.reduce((sum, s) => sum + s.ms, 0),
    },
  };
}

function now() {
  return typeof performance !== "undefined" ? performance.now() : Date.now();
}
