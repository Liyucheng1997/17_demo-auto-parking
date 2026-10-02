/**
 * 环境模型：地下停车场（单位 m）。
 *
 *   y ↑
 *  30.6 ┌──────────────────────── 北墙 ────────────────────────┐
 *       │        A 排：12 个垂直车位 2.5 × 5.3                  │
 *  25.0 │  ─────────────── 主通道（6.5 m）──────────────────    │
 *  18.5 │        B 排：12 个垂直车位                            │
 *  13.2 │        ████████ 绿化隔离带 ████████                   │
 *  11.2 │        N 排：侧方车位（6.2 m，较紧）                   │
 *   8.7 │  ─────────────── 支路（6.0 m）────────────────────    │
 *   2.7 │        S 排：侧方车位（7.0 m）                         │
 *     0 └──────────────────────────────────────────────────────┘ → x
 *       0     西侧连通道 (0–9)                  东侧连通道 (41–50)  50
 */

import { createRng, deg2rad } from "../core/math.js";
import { circlePolygon, orientedBox, rectPolygon } from "../core/geometry.js";

export const LOT = Object.freeze({
  width: 50,
  height: 30.6,
  wallThickness: 0.4,
  perpendicular: { width: 2.5, depth: 5.3, count: 12, x0: 10 },
  rows: {
    A: { y0: 25.0, inward: Math.PI / 2 },
    B: { y0: 18.5, inward: -Math.PI / 2 },
  },
  aisle: { y0: 18.5, y1: 25.0 },
  street: { y0: 2.7, y1: 8.7 },
  island: { x0: 9, x1: 41, y0: 11.2, y1: 13.2 },
  parallelRows: {
    N: { y0: 8.7, depth: 2.5, length: 6.2, x0: 12.4, count: 4, gap: 0.6, heading: Math.PI, inward: Math.PI / 2 },
    S: { y0: 2.7, depth: 2.5, length: 7.0, x0: 11, count: 4, gap: 0, heading: 0, inward: -Math.PI / 2 },
  },
});

const DEFAULT_OCCUPANCY = {
  A: [0, 1, 3, 4, 6, 8, 9, 11],
  B: [0, 2, 3, 5, 6, 8, 10, 11],
  N: [1, 3],
  S: [0, 2],
};

/** 被占车位上的社会车辆尺寸库：真实场景中车型、停放姿态都存在差异 */
const PARKED_MODELS = [
  { name: "紧凑型", length: 4.35, width: 1.78 },
  { name: "中型轿车", length: 4.7, width: 1.84 },
  { name: "中型 SUV", length: 4.75, width: 1.92 },
  { name: "MPV", length: 5.05, width: 1.95 },
];

function buildPerpendicularSlots() {
  const { width, depth, count, x0 } = LOT.perpendicular;
  const slots = [];
  for (const [rowId, row] of Object.entries(LOT.rows)) {
    for (let i = 0; i < count; i += 1) {
      const xCenter = x0 + width * (i + 0.5);
      const opening = { x: xCenter, y: row.y0 };
      const inwardY = Math.sin(row.inward);
      slots.push({
        id: `${rowId}-${String(i + 1).padStart(2, "0")}`,
        row: rowId,
        index: i,
        type: "perpendicular",
        width,
        depth,
        opening,
        inward: row.inward,
        center: { x: xCenter, y: row.y0 + (inwardY * depth) / 2 },
        polygon: orientedBox(xCenter, row.y0 + (inwardY * depth) / 2, row.inward, depth, width),
      });
    }
  }
  return slots;
}

function buildParallelSlots() {
  const slots = [];
  for (const [rowId, row] of Object.entries(LOT.parallelRows)) {
    for (let i = 0; i < row.count; i += 1) {
      const xCenter = row.x0 + (row.length + row.gap) * i + row.length / 2;
      const yCenter = row.y0 + (Math.sin(row.inward) * row.depth) / 2;
      slots.push({
        id: `${rowId}-${String(i + 1).padStart(2, "0")}`,
        row: rowId,
        index: i,
        type: "parallel",
        width: row.length,
        depth: row.depth,
        heading: row.heading,
        inward: row.inward,
        opening: { x: xCenter, y: row.y0 },
        center: { x: xCenter, y: yCenter },
        polygon: orientedBox(xCenter, yCenter, row.heading, row.length, row.depth),
      });
    }
  }
  return slots;
}

function buildStaticObstacles() {
  const { width: W, height: H, wallThickness: t, island } = LOT;
  const obstacles = [
    { id: "wall-s", kind: "wall", polygon: rectPolygon(-t, -t, W + 2 * t, t) },
    { id: "wall-n", kind: "wall", polygon: rectPolygon(-t, H, W + 2 * t, t) },
    { id: "wall-w", kind: "wall", polygon: rectPolygon(-t, 0, t, H) },
    { id: "wall-e", kind: "wall", polygon: rectPolygon(W, 0, t, H) },
    {
      id: "island",
      kind: "island",
      polygon: rectPolygon(island.x0, island.y0, island.x1 - island.x0, island.y1 - island.y0),
    },
  ];
  // 结构立柱：位于各排车位两端
  const pillarSize = 0.6;
  const pillars = [
    { x: 9.6, y: 27.65 },
    { x: 40.4, y: 27.65 },
    { x: 9.6, y: 15.85 },
    { x: 40.4, y: 15.85 },
  ];
  pillars.forEach((p, i) => {
    obstacles.push({
      id: `pillar-${i + 1}`,
      kind: "pillar",
      polygon: rectPolygon(p.x - pillarSize / 2, p.y - pillarSize / 2, pillarSize, pillarSize),
    });
  });
  return obstacles;
}

/** 依据车位与随机种子生成一辆停放车辆（带姿态扰动） */
function parkedCarFor(slot, seed) {
  const rng = createRng(seed);
  const model = PARKED_MODELS[Math.floor(rng() * PARKED_MODELS.length)];
  const heading =
    slot.type === "parallel"
      ? slot.heading
      : slot.inward + (rng() < 0.7 ? Math.PI : 0); // 70% 倒车入库
  const lateral = (rng() - 0.5) * 0.24;
  const longitudinal =
    slot.type === "parallel" ? (rng() - 0.5) * 0.5 : (slot.depth - model.length) / 2 - 0.25 + rng() * 0.2;
  const yaw = deg2rad((rng() - 0.5) * 4);
  const axis = slot.type === "parallel" ? slot.heading : slot.inward;
  const nx = -Math.sin(axis);
  const ny = Math.cos(axis);
  const cx = slot.center.x + Math.cos(axis) * longitudinal + nx * lateral;
  const cy = slot.center.y + Math.sin(axis) * longitudinal + ny * lateral;
  return {
    id: `car-${slot.id}`,
    kind: "car",
    slotId: slot.id,
    model: model.name,
    pose: { x: cx, y: cy, theta: heading + yaw },
    length: model.length,
    width: model.width,
    color: Math.floor(rng() * 6),
    polygon: orientedBox(cx, cy, heading + yaw, model.length, model.width),
  };
}

/**
 * 创建停车场场景。场景是纯数据，可被规划器、仿真器与渲染器共享，
 * 也可以序列化后送入 Web Worker。
 */
export function createParkingLot({ occupancy = DEFAULT_OCCUPANCY, cones = [], seed = 7 } = {}) {
  const slots = [...buildPerpendicularSlots(), ...buildParallelSlots()];
  for (const slot of slots) {
    slot.occupied = (occupancy[slot.row] || []).includes(slot.index);
  }
  const lot = {
    width: LOT.width,
    height: LOT.height,
    layout: LOT,
    slots,
    staticObstacles: buildStaticObstacles(),
    cones: [],
    seed,
    start: { x: 3.2, y: 21.75, theta: 0 },
  };
  cones.forEach((c) => addCone(lot, c.x, c.y));
  return lot;
}

export function addCone(lot, x, y) {
  const cone = { id: `cone-${lot.cones.length + 1}-${Date.now() % 1e6}`, kind: "cone", x, y, radius: 0.3 };
  cone.polygon = circlePolygon(x, y, cone.radius, 10);
  lot.cones.push(cone);
  return cone;
}

export function removeConeAt(lot, x, y) {
  const index = lot.cones.findIndex((c) => Math.hypot(c.x - x, c.y - y) < 0.6);
  if (index >= 0) lot.cones.splice(index, 1);
  return index >= 0;
}

export function parkedCars(lot) {
  return lot.slots.filter((s) => s.occupied).map((s) => parkedCarFor(s, lot.seed * 131 + hashId(s.id)));
}

function hashId(id) {
  let h = 0;
  for (const ch of id) h = (h * 31 + ch.charCodeAt(0)) >>> 0;
  return h;
}

/** 当前场景下全部障碍物（静态结构 + 停放车辆 + 锥桶） */
export function sceneObstacles(lot) {
  return [...lot.staticObstacles, ...parkedCars(lot), ...lot.cones];
}

export function findSlot(lot, id) {
  return lot.slots.find((s) => s.id === id);
}
