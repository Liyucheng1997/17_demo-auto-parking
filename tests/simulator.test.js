import { test } from "node:test";
import assert from "node:assert/strict";
import { createParkingLot } from "../src/model/parkingLot.js";
import { createVehicleParams } from "../src/model/vehicle.js";
import { buildEnvironment, planParking } from "../src/planning/planner.js";
import { ParkingSimulator } from "../src/sim/simulator.js";
import { CONTROLLERS } from "../src/control/controllers.js";

const vehicle = createVehicleParams();
const lot = createParkingLot();
const env = buildEnvironment(lot, vehicle);
const cases = ["A-03", "B-05", "N-01", "S-04"];

for (const controller of Object.values(CONTROLLERS)) {
  for (const slotId of cases) {
    test(`闭环泊车 ${controller} → ${slotId}：精度达标且无刮碰`, () => {
      const plan = planParking({ lot, slotId, vehicle });
      assert.equal(plan.success, true, plan.reason);
      const result = new ParkingSimulator({ plan, vehicle, checker: env.checker, controller }).run();
      assert.equal(result.phase, "done");
      assert.equal(result.collided, false);
      assert.ok(result.positionError < 0.1, `位置误差 ${(result.positionError * 100).toFixed(1)} cm`);
      assert.ok(Math.abs(result.headingError) < (3 * Math.PI) / 180, `航向误差 ${((result.headingError * 180) / Math.PI).toFixed(2)}°`);
    });
  }
}

test("定位噪声 ±3 cm 下仍可完成泊车", () => {
  const plan = planParking({ lot, slotId: "B-08", vehicle });
  const result = new ParkingSimulator({
    plan,
    vehicle,
    checker: env.checker,
    controller: CONTROLLERS.REAR_WHEEL,
    localizationNoise: 0.03,
    seed: 3,
  }).run();
  assert.equal(result.collided, false);
  assert.ok(result.positionError < 0.15);
});
