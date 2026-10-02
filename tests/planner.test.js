import { test } from "node:test";
import assert from "node:assert/strict";
import { createParkingLot, addCone } from "../src/model/parkingLot.js";
import { createVehicleParams, vehicleFootprint } from "../src/model/vehicle.js";
import { buildEnvironment, planParking } from "../src/planning/planner.js";
import { polygonsIntersect } from "../src/core/geometry.js";
import { angleDiff } from "../src/core/math.js";

const vehicle = createVehicleParams();
const lot = createParkingLot();
const env = buildEnvironment(lot, vehicle);
const freeSlots = lot.slots.filter((s) => !s.occupied);

/** 用 0 裕度、逐点 SAT 独立校验整条轨迹（不依赖规划器内部的检测器） */
function assertCollisionFree(plan, obstacles) {
  for (const p of plan.trajectory.points) {
    const fp = vehicleFootprint(vehicle, p, 0);
    for (const o of obstacles) {
      assert.equal(polygonsIntersect(fp, o.polygon), false, `${plan.slotId} 在 (${p.x.toFixed(2)}, ${p.y.toFixed(2)}) 与 ${o.id} 相交`);
    }
  }
}

for (const slot of freeSlots) {
  test(`倒车入库/侧方：${slot.id} 可规划且轨迹安全`, () => {
    const plan = planParking({ lot, slotId: slot.id, vehicle });
    assert.equal(plan.success, true, plan.reason);
    const pts = plan.trajectory.points;
    const first = pts[0];
    const last = pts[pts.length - 1];
    assert.ok(Math.hypot(first.x - lot.start.x, first.y - lot.start.y) < 1e-6, "起点一致");
    assert.ok(Math.hypot(last.x - plan.goal.x, last.y - plan.goal.y) < 1e-3, "终点到达目标");
    assert.ok(Math.abs(angleDiff(last.theta, plan.goal.theta)) < 1e-3, "终点航向一致");
    assertCollisionFree(plan, env.obstacles);
    // 曲率不超过车辆物理极限
    const kMax = Math.tan(vehicle.maxSteer) / vehicle.wheelbase;
    for (const p of pts) assert.ok(Math.abs(p.kappa) <= kMax * 1.15, `κ=${p.kappa.toFixed(3)} 超限`);
    // 速度曲线：段首尾为 0、不超过挡位限速
    for (const seg of plan.segments) {
      assert.equal(seg.points[0].v, 0);
      assert.equal(seg.points[seg.points.length - 1].v, 0);
      const vMax = seg.dir > 0 ? vehicle.maxSpeedForward : vehicle.maxSpeedReverse;
      for (const p of seg.points) assert.ok(p.v <= vMax + 1e-9 && p.v <= p.vLimit + 1e-9);
    }
  });
}

test("车头入库：A-06 可规划", () => {
  const plan = planParking({ lot, slotId: "A-06", vehicle, mode: "head-in" });
  assert.equal(plan.success, true, plan.reason);
  const last = plan.trajectory.points.at(-1);
  assert.equal(plan.segments.at(-1).dir, 1, "最后一段前进入库");
  assert.ok(Math.abs(angleDiff(last.theta, Math.PI / 2)) < 1e-3, "车头朝里");
});

test("临时障碍物（锥桶）迫使规划绕行", () => {
  const blocked = createParkingLot();
  const base = planParking({ lot: blocked, slotId: "A-08", vehicle });
  // 在原路径中段放置锥桶
  const mid = base.trajectory.points[Math.floor(base.trajectory.points.length * 0.35)];
  addCone(blocked, mid.x, mid.y);
  const detour = planParking({ lot: blocked, slotId: "A-08", vehicle });
  assert.equal(detour.success, true, detour.reason);
  const cone = blocked.cones[0];
  for (const p of detour.trajectory.points) {
    assert.equal(polygonsIntersect(vehicleFootprint(vehicle, p, 0), cone.polygon), false);
  }
});

test("目标车位被占时规划失败并给出原因", () => {
  const plan = planParking({ lot, slotId: "A-01", vehicle });
  assert.equal(plan.success, false);
  assert.match(plan.reason, /冲突|不可达/);
});
