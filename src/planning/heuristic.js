/**
 * 2D 完整约束 + 障碍物启发函数 h2（holonomic-with-obstacles）。
 *
 * 忽略车辆运动学约束，在栅格上从目标点做 8 邻域 Dijkstra，得到每个栅格到目标的
 * 最短可通行距离。可通行判定：距离场 ≥ 车宽一半（车身至少要能“侧身”通过）。
 * 它能提前发现死胡同、绕行，弥补 RS 启发函数“看不见障碍物”的缺陷。
 */

import { MinHeap } from "../core/math.js";

export function buildHolonomicHeuristic(gridMap, goal, clearance) {
  const { cols, rows, resolution, dist } = gridMap;
  const cost = new Float64Array(cols * rows).fill(Infinity);
  const passable = new Uint8Array(cols * rows);
  for (let i = 0; i < passable.length; i += 1) passable[i] = dist[i] >= clearance ? 1 : 0;

  const gc = Math.min(cols - 1, Math.max(0, Math.floor(goal.x / resolution)));
  const gr = Math.min(rows - 1, Math.max(0, Math.floor(goal.y / resolution)));
  const goalIndex = gr * cols + gc;
  passable[goalIndex] = 1;
  cost[goalIndex] = 0;

  const heap = new MinHeap();
  heap.push(goalIndex, 0);
  const moves = [
    [1, 0, 1],
    [-1, 0, 1],
    [0, 1, 1],
    [0, -1, 1],
    [1, 1, Math.SQRT2],
    [1, -1, Math.SQRT2],
    [-1, 1, Math.SQRT2],
    [-1, -1, Math.SQRT2],
  ];
  while (heap.size) {
    const index = heap.pop();
    const c = index % cols;
    const r = (index - c) / cols;
    const base = cost[index];
    for (const [dc, dr, w] of moves) {
      const nc = c + dc;
      const nr = r + dr;
      if (nc < 0 || nr < 0 || nc >= cols || nr >= rows) continue;
      const ni = nr * cols + nc;
      if (!passable[ni]) continue;
      const next = base + w * resolution;
      if (next < cost[ni]) {
        cost[ni] = next;
        heap.push(ni, next);
      }
    }
  }
  return { cols, rows, resolution, cost, passable };
}

export function lookupHeuristic(h, x, y) {
  const c = Math.floor(x / h.resolution);
  const r = Math.floor(y / h.resolution);
  if (c < 0 || r < 0 || c >= h.cols || r >= h.rows) return Infinity;
  return h.cost[r * h.cols + c];
}
