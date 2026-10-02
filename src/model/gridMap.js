/**
 * 栅格环境模型：占据栅格 (occupancy grid) + 欧氏距离场 (EDT)。
 *
 * - 占据栅格：把障碍物多边形光栅化；
 * - 距离场：每个栅格中心到最近障碍物的距离，采用 Felzenszwalb & Huttenlocher
 *   的线性时间可分离平方距离变换，O(N) 复杂度。
 * 距离场用于：碰撞检测快速通道、2D 启发函数可通行性判定、路径平滑的障碍物项、
 * 以及界面上的“代价地图”可视化。
 */

import { pointInPolygon, pointPolygonDistance } from "../core/geometry.js";

const INF = 1e20;

/** 一维平方距离变换（下包络抛物线） */
function edt1d(f, n, d, v, z) {
  let k = 0;
  v[0] = 0;
  z[0] = -INF;
  z[1] = INF;
  for (let q = 1; q < n; q += 1) {
    let s = (f[q] + q * q - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
    while (s <= z[k]) {
      k -= 1;
      s = (f[q] + q * q - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
    }
    k += 1;
    v[k] = q;
    z[k] = s;
    z[k + 1] = INF;
  }
  k = 0;
  for (let q = 0; q < n; q += 1) {
    while (z[k + 1] < q) k += 1;
    d[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
  }
}

/** 二维欧氏距离变换，返回以栅格为单位的距离 */
export function distanceTransform(occ, cols, rows) {
  const grid = new Float64Array(cols * rows);
  for (let i = 0; i < grid.length; i += 1) grid[i] = occ[i] ? 0 : INF;
  const n = Math.max(cols, rows);
  const f = new Float64Array(n);
  const d = new Float64Array(n);
  const v = new Int32Array(n);
  const z = new Float64Array(n + 1);
  for (let x = 0; x < cols; x += 1) {
    for (let y = 0; y < rows; y += 1) f[y] = grid[y * cols + x];
    edt1d(f, rows, d, v, z);
    for (let y = 0; y < rows; y += 1) grid[y * cols + x] = d[y];
  }
  for (let y = 0; y < rows; y += 1) {
    for (let x = 0; x < cols; x += 1) f[x] = grid[y * cols + x];
    edt1d(f, cols, d, v, z);
    for (let x = 0; x < cols; x += 1) grid[y * cols + x] = d[x];
  }
  const out = new Float32Array(cols * rows);
  for (let i = 0; i < out.length; i += 1) out[i] = Math.sqrt(grid[i]);
  return out;
}

/**
 * 构建栅格地图。
 * @param {Array} obstacles  带 polygon 的障碍物列表
 * @param {{width:number,height:number}} bounds 地图范围 (m)
 */
export function buildGridMap(obstacles, bounds, resolution = 0.2) {
  const cols = Math.ceil(bounds.width / resolution);
  const rows = Math.ceil(bounds.height / resolution);
  const occ = new Uint8Array(cols * rows);
  const half = resolution / 2;
  const touch = resolution * 0.5;

  for (const obstacle of obstacles) {
    const poly = obstacle.polygon;
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    for (const p of poly.points) {
      minX = Math.min(minX, p.x);
      minY = Math.min(minY, p.y);
      maxX = Math.max(maxX, p.x);
      maxY = Math.max(maxY, p.y);
    }
    const c0 = Math.max(0, Math.floor((minX - touch) / resolution));
    const c1 = Math.min(cols - 1, Math.floor((maxX + touch) / resolution));
    const r0 = Math.max(0, Math.floor((minY - touch) / resolution));
    const r1 = Math.min(rows - 1, Math.floor((maxY + touch) / resolution));
    for (let r = r0; r <= r1; r += 1) {
      for (let c = c0; c <= c1; c += 1) {
        const x = c * resolution + half;
        const y = r * resolution + half;
        if (pointInPolygon(x, y, poly) || pointPolygonDistance(x, y, poly) <= touch) {
          occ[r * cols + c] = 1;
        }
      }
    }
  }

  // 地图边界视为障碍（围墙位于地图范围之外，栅格中心采样不到）
  for (let c = 0; c < cols; c += 1) {
    occ[c] = 1;
    occ[(rows - 1) * cols + c] = 1;
  }
  for (let r = 0; r < rows; r += 1) {
    occ[r * cols] = 1;
    occ[r * cols + cols - 1] = 1;
  }

  const dist = distanceTransform(occ, cols, rows);
  for (let i = 0; i < dist.length; i += 1) dist[i] *= resolution;
  return { resolution, cols, rows, width: bounds.width, height: bounds.height, occ, dist };
}

/** 双线性插值查询距离场（m），越界视为 0 */
export function sampleDistance(map, x, y) {
  const gx = x / map.resolution - 0.5;
  const gy = y / map.resolution - 0.5;
  const c = Math.floor(gx);
  const r = Math.floor(gy);
  if (c < 0 || r < 0 || c >= map.cols - 1 || r >= map.rows - 1) return 0;
  const fx = gx - c;
  const fy = gy - r;
  const i = r * map.cols + c;
  const d = map.dist;
  return (
    d[i] * (1 - fx) * (1 - fy) +
    d[i + 1] * fx * (1 - fy) +
    d[i + map.cols] * (1 - fx) * fy +
    d[i + map.cols + 1] * fx * fy
  );
}

/** 距离场梯度（指向远离障碍物的方向），用于路径平滑 */
export function distanceGradient(map, x, y) {
  const h = map.resolution;
  return {
    x: (sampleDistance(map, x + h, y) - sampleDistance(map, x - h, y)) / (2 * h),
    y: (sampleDistance(map, x, y + h) - sampleDistance(map, x, y - h)) / (2 * h),
  };
}
