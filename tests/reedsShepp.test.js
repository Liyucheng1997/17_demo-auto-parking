import { test } from "node:test";
import assert from "node:assert/strict";
import { reedsSheppDistance, reedsSheppPaths, sampleReedsShepp } from "../src/planning/reedsShepp.js";
import { angleDiff, createRng } from "../src/core/math.js";

const R = 4;

test("RS 路径终点精确到达目标位姿（随机 500 组）", () => {
  const rng = createRng(42);
  for (let i = 0; i < 500; i += 1) {
    const start = { x: rng() * 20 - 10, y: rng() * 20 - 10, theta: (rng() * 2 - 1) * Math.PI };
    const goal = { x: rng() * 20 - 10, y: rng() * 20 - 10, theta: (rng() * 2 - 1) * Math.PI };
    const paths = reedsSheppPaths(start, goal, R);
    assert.ok(paths.length > 0, "至少存在一条 RS 路径");
    for (const path of paths) {
      const pts = sampleReedsShepp(start, path, R, 0.05);
      const end = pts[pts.length - 1];
      assert.ok(Math.hypot(end.x - goal.x, end.y - goal.y) < 1e-6, `${path.word} 位置误差`);
      assert.ok(Math.abs(angleDiff(end.theta, goal.theta)) < 1e-6, `${path.word} 航向误差`);
    }
  }
});

test("纯直线前进 / 倒车", () => {
  const fwd = reedsSheppPaths({ x: 0, y: 0, theta: 0 }, { x: 5, y: 0, theta: 0 }, R)[0];
  assert.ok(Math.abs(fwd.length - 5) < 1e-9);
  const rev = reedsSheppPaths({ x: 0, y: 0, theta: 0 }, { x: -3, y: 0, theta: 0 }, R)[0];
  assert.ok(Math.abs(rev.length - 3) < 1e-9);
  assert.ok(rev.segments.every((s) => s.length < 0));
});

test("最短路径不长于欧氏距离的下界且满足三角不等式", () => {
  const rng = createRng(7);
  for (let i = 0; i < 200; i += 1) {
    const p = () => ({ x: rng() * 16 - 8, y: rng() * 16 - 8, theta: (rng() * 2 - 1) * Math.PI });
    const a = p(), b = p(), c = p();
    const d = (u, v) => reedsSheppPaths(u, v, R)[0].length;
    assert.ok(d(a, b) >= Math.hypot(a.x - b.x, a.y - b.y) - 1e-9);
    assert.ok(d(a, c) <= d(a, b) + d(b, c) + 1e-6);
    assert.ok(Math.abs(d(a, b) - d(b, a)) < 1e-6, "RS 距离对称");
  }
});

test("快速距离函数与完整求解结果一致", () => {
  const rng = createRng(3);
  for (let i = 0; i < 300; i += 1) {
    const a = { x: rng() * 20 - 10, y: rng() * 20 - 10, theta: (rng() * 2 - 1) * Math.PI };
    const b = { x: rng() * 20 - 10, y: rng() * 20 - 10, theta: (rng() * 2 - 1) * Math.PI };
    assert.ok(Math.abs(reedsSheppDistance(a, b, R) - reedsSheppPaths(a, b, R)[0].length) < 1e-9);
  }
});
