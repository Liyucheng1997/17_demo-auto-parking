/**
 * 基础数学工具。全工程统一使用 SI 单位：米、弧度、秒。
 * 坐标系：世界系 x 向东、y 向北，航向角 θ 由 x 轴逆时针为正。
 */

export const TAU = Math.PI * 2;

export const clamp = (value, min, max) => Math.min(max, Math.max(min, value));

export const lerp = (a, b, t) => a + (b - a) * t;

export const deg2rad = (deg) => (deg * Math.PI) / 180;

export const rad2deg = (rad) => (rad * 180) / Math.PI;

/** 把角度归一化到 (-π, π] */
export function normalizeAngle(angle) {
  let a = angle % TAU;
  if (a <= -Math.PI) a += TAU;
  else if (a > Math.PI) a -= TAU;
  return a;
}

/** 两角度的有符号差 a - b，结果在 (-π, π] */
export const angleDiff = (a, b) => normalizeAngle(a - b);

export const hypot = Math.hypot;

/** 世界坐标点转换到以 pose 为原点的局部坐标 */
export function toLocal(pose, x, y) {
  const dx = x - pose.x;
  const dy = y - pose.y;
  const c = Math.cos(pose.theta);
  const s = Math.sin(pose.theta);
  return { x: c * dx + s * dy, y: -s * dx + c * dy };
}

/** 局部坐标点转换回世界坐标 */
export function toWorld(pose, x, y) {
  const c = Math.cos(pose.theta);
  const s = Math.sin(pose.theta);
  return { x: pose.x + c * x - s * y, y: pose.y + s * x + c * y };
}

/** 可复现的伪随机数（mulberry32），用于测试与噪声仿真 */
export function createRng(seed = 1) {
  let t = seed >>> 0;
  return () => {
    t = (t + 0x6d2b79f5) >>> 0;
    let r = Math.imul(t ^ (t >>> 15), 1 | t);
    r ^= r + Math.imul(r ^ (r >>> 7), 61 | r);
    return ((r ^ (r >>> 14)) >>> 0) / 4294967296;
  };
}

/** 二叉堆优先队列（最小堆），Hybrid A* 与 Dijkstra 共用 */
export class MinHeap {
  constructor() {
    this.items = [];
    this.priorities = [];
  }

  get size() {
    return this.items.length;
  }

  push(item, priority) {
    const items = this.items;
    const prio = this.priorities;
    let i = items.length;
    items.push(item);
    prio.push(priority);
    while (i > 0) {
      const parent = (i - 1) >> 1;
      if (prio[parent] <= priority) break;
      items[i] = items[parent];
      prio[i] = prio[parent];
      i = parent;
    }
    items[i] = item;
    prio[i] = priority;
  }

  pop() {
    const items = this.items;
    const prio = this.priorities;
    const top = items[0];
    const lastItem = items.pop();
    const lastPrio = prio.pop();
    const n = items.length;
    if (n > 0) {
      let i = 0;
      while (true) {
        const left = 2 * i + 1;
        if (left >= n) break;
        const right = left + 1;
        const child = right < n && prio[right] < prio[left] ? right : left;
        if (prio[child] >= lastPrio) break;
        items[i] = items[child];
        prio[i] = prio[child];
        i = child;
      }
      items[i] = lastItem;
      prio[i] = lastPrio;
    }
    return top;
  }
}
