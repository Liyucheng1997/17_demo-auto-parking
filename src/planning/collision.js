/**
 * 碰撞检测器：车辆外廓 (OBB + 安全裕度) 与障碍物凸多边形。
 *
 * 三级检测流水线：
 *   1. 距离场快速通道：车身中心到障碍物的距离 > 外接圆半径 ⇒ 必然无碰撞；
 *   2. 空间哈希粗筛：只取车身外接圆覆盖的哈希格中的障碍物；
 *   3. SAT 精确检测：分离轴定理判定凸多边形相交。
 */

import { polygonDistance, polygonsIntersect } from "../core/geometry.js";
import { centerOffset, vehicleFootprint } from "../model/vehicle.js";
import { sampleDistance } from "../model/gridMap.js";

export class CollisionChecker {
  constructor(obstacles, vehicle, { margin = 0.15, gridMap = null, bounds = null, cellSize = 2.5 } = {}) {
    this.vehicle = vehicle;
    this.margin = margin;
    this.gridMap = gridMap;
    this.bounds = bounds;
    this.cellSize = cellSize;
    this.offset = centerOffset(vehicle);
    this.circumradius = Math.hypot(vehicle.length / 2 + margin, vehicle.width / 2 + margin);
    this.hash = new Map();
    this.obstacles = obstacles;
    this.checks = 0;
    for (const obstacle of obstacles) this.#insert(obstacle);
  }

  #key(i, j) {
    return i * 100003 + j;
  }

  #insert(obstacle) {
    const p = obstacle.polygon;
    const s = this.cellSize;
    const i0 = Math.floor((p.cx - p.radius) / s);
    const i1 = Math.floor((p.cx + p.radius) / s);
    const j0 = Math.floor((p.cy - p.radius) / s);
    const j1 = Math.floor((p.cy + p.radius) / s);
    for (let i = i0; i <= i1; i += 1) {
      for (let j = j0; j <= j1; j += 1) {
        const key = this.#key(i, j);
        if (!this.hash.has(key)) this.hash.set(key, []);
        this.hash.get(key).push(obstacle);
      }
    }
  }

  /** 查询半径 r 圆内可能相交的障碍物 */
  candidates(cx, cy, r) {
    const s = this.cellSize;
    const found = new Set();
    const i0 = Math.floor((cx - r) / s);
    const i1 = Math.floor((cx + r) / s);
    const j0 = Math.floor((cy - r) / s);
    const j1 = Math.floor((cy + r) / s);
    for (let i = i0; i <= i1; i += 1) {
      for (let j = j0; j <= j1; j += 1) {
        const bucket = this.hash.get(this.#key(i, j));
        if (bucket) for (const o of bucket) found.add(o);
      }
    }
    return found;
  }

  /** 位姿 (后轴中心) 是否碰撞 */
  collides(pose) {
    this.checks += 1;
    const cx = pose.x + this.offset * Math.cos(pose.theta);
    const cy = pose.y + this.offset * Math.sin(pose.theta);
    if (this.bounds) {
      if (cx < 0 || cy < 0 || cx > this.bounds.width || cy > this.bounds.height) return true;
    }
    if (this.gridMap) {
      const clearance = sampleDistance(this.gridMap, cx, cy);
      if (clearance > this.circumradius + this.gridMap.resolution) return false;
    }
    const footprint = vehicleFootprint(this.vehicle, pose, this.margin);
    for (const obstacle of this.candidates(cx, cy, this.circumradius)) {
      if (polygonsIntersect(footprint, obstacle.polygon)) return true;
    }
    return false;
  }

  /** 沿位姿序列检测；返回第一个碰撞点索引，无碰撞返回 -1 */
  firstCollision(poses) {
    for (let i = 0; i < poses.length; i += 1) {
      if (this.collides(poses[i])) return i;
    }
    return -1;
  }

  /** 车身外廓（不含裕度）到最近障碍物的距离与对应障碍物 */
  clearance(pose, searchRadius = 4) {
    const footprint = vehicleFootprint(this.vehicle, pose, 0);
    let best = Infinity;
    let nearest = null;
    for (const obstacle of this.candidates(footprint.cx, footprint.cy, footprint.radius + searchRadius)) {
      const d = polygonDistance(footprint, obstacle.polygon);
      if (d < best) {
        best = d;
        nearest = obstacle;
      }
    }
    return { distance: best, obstacle: nearest };
  }
}
