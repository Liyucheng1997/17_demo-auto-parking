import { test } from "node:test";
import assert from "node:assert/strict";
import {
  orientedBox,
  rectPolygon,
  polygonsIntersect,
  polygonDistance,
  rayPolygonDistance,
  pointPolygonDistance,
} from "../src/core/geometry.js";
import { angleDiff, MinHeap, normalizeAngle, createRng } from "../src/core/math.js";

test("SAT：相交、分离、旋转后相交", () => {
  const a = rectPolygon(0, 0, 2, 1);
  assert.equal(polygonsIntersect(a, rectPolygon(1.5, 0.5, 2, 2)), true);
  assert.equal(polygonsIntersect(a, rectPolygon(2.1, 0, 1, 1)), false);
  // 45° 旋转的方块，外接圆相交但实际分离
  const diamond = orientedBox(3.2, 1.6, Math.PI / 4, 1, 1);
  assert.equal(polygonsIntersect(a, diamond), false);
  assert.equal(polygonsIntersect(a, orientedBox(2.4, 1.0, Math.PI / 4, 1, 1)), true);
});

test("多边形距离与点距离", () => {
  const a = rectPolygon(0, 0, 1, 1);
  const b = rectPolygon(3, 0, 1, 1);
  assert.ok(Math.abs(polygonDistance(a, b) - 2) < 1e-9);
  assert.equal(polygonDistance(a, rectPolygon(0.5, 0.5, 1, 1)), 0);
  assert.ok(Math.abs(pointPolygonDistance(0.5, 0.5, a) + 0.5) < 1e-9, "内部为负");
  assert.ok(Math.abs(pointPolygonDistance(2, 0.5, a) - 1) < 1e-9);
});

test("射线求交", () => {
  const box = rectPolygon(2, -1, 1, 2);
  assert.ok(Math.abs(rayPolygonDistance(0, 0, 1, 0, box) - 2) < 1e-9);
  assert.equal(rayPolygonDistance(0, 0, -1, 0, box), Infinity);
});

test("角度归一化与最小堆", () => {
  assert.ok(Math.abs(normalizeAngle(3 * Math.PI) - Math.PI) < 1e-12);
  assert.ok(Math.abs(angleDiff(0.1, -0.1 + 2 * Math.PI) - 0.2) < 1e-12);
  const heap = new MinHeap();
  const rng = createRng(5);
  const values = Array.from({ length: 500 }, () => rng());
  values.forEach((v) => heap.push(v, v));
  const out = [];
  while (heap.size) out.push(heap.pop());
  assert.deepEqual(out, [...values].sort((x, y) => x - y));
});
