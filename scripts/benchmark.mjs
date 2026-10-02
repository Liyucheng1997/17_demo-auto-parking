/**
 * 基准测试：对默认场景中全部空闲车位 × 泊车方式 × 控制器做“规划 + 闭环仿真”，
 * 输出 Markdown 表格与汇总统计。
 *
 *   npm run bench                  # 打印表格
 *   npm run bench -- --json out.json  # 同时写出 JSON 明细
 */

import { writeFileSync } from "node:fs";
import { createParkingLot } from "../src/model/parkingLot.js";
import { createVehicleParams } from "../src/model/vehicle.js";
import { buildEnvironment, planParking } from "../src/planning/planner.js";
import { ParkingSimulator } from "../src/sim/simulator.js";
import { CONTROLLERS } from "../src/control/controllers.js";

const vehicle = createVehicleParams();
const lot = createParkingLot();
const env = buildEnvironment(lot, vehicle);
const rows = [];

for (const slot of lot.slots.filter((s) => !s.occupied)) {
  const modes = slot.type === "perpendicular" ? ["reverse-in", "head-in"] : ["parallel"];
  for (const mode of modes) {
    const plan = planParking({ lot, slotId: slot.id, vehicle, mode: mode === "parallel" ? "reverse-in" : mode });
    for (const controller of Object.values(CONTROLLERS)) {
      const row = {
        slot: slot.id,
        mode,
        controller,
        planned: plan.success,
        planMs: plan.stages.reduce((s, st) => s + st.ms, 0),
        iterations: plan.search.stats.iterations,
      };
      if (plan.success) {
        const r = new ParkingSimulator({ plan, vehicle, checker: env.checker, controller }).run();
        Object.assign(row, {
          length: plan.metrics.length,
          gearShifts: plan.metrics.gearShifts,
          planDuration: plan.metrics.duration,
          simTime: r.time,
          posErrCm: r.positionError * 100,
          headErrDeg: Math.abs((r.headingError * 180) / Math.PI),
          maxEyCm: r.maxLateralError * 100,
          minClearance: r.minClearance,
          success: r.success,
        });
      }
      rows.push(row);
    }
  }
}

const f = (v, d = 1) => (v === undefined ? "--" : v.toFixed(d));
const MODE = { "reverse-in": "倒车入库", "head-in": "车头入库", parallel: "侧方" };
const CTRL = { "pure-pursuit": "纯跟踪", "rear-wheel-feedback": "后轮反馈" };
console.log("| 车位 | 方式 | 控制器 | 规划 ms | 扩展节点 | 路径 m | 换挡 | 仿真 s | 位置误差 cm | 航向误差 ° | 最大横向误差 cm | 最小障碍距离 m | 结果 |");
console.log("|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|");
for (const r of rows) {
  console.log(
    `| ${r.slot} | ${MODE[r.mode]} | ${CTRL[r.controller]} | ${f(r.planMs, 0)} | ${r.iterations} | ${f(r.length)} | ${r.gearShifts ?? "--"} | ${f(r.simTime)} | ${f(r.posErrCm)} | ${f(r.headErrDeg, 2)} | ${f(r.maxEyCm)} | ${f(r.minClearance, 2)} | ${r.success ? "✅" : "❌"} |`,
  );
}

const ok = rows.filter((r) => r.success);
const mean = (key, list = ok) => list.reduce((s, r) => s + r[key], 0) / Math.max(1, list.length);
console.log("");
console.log(`成功率：${ok.length}/${rows.length}`);
for (const controller of Object.values(CONTROLLERS)) {
  const list = ok.filter((r) => r.controller === controller);
  console.log(
    `${CTRL[controller]}：平均位置误差 ${f(mean("posErrCm", list), 2)} cm，平均航向误差 ${f(mean("headErrDeg", list), 2)}°，平均最大横向误差 ${f(mean("maxEyCm", list), 1)} cm`,
  );
}
const planned = rows.filter((r, i) => r.planned && i % 2 === 0);
console.log(`平均规划耗时 ${f(mean("planMs", planned), 0)} ms，最长 ${f(Math.max(...planned.map((r) => r.planMs)), 0)} ms`);

const jsonArg = process.argv.indexOf("--json");
if (jsonArg > 0) {
  writeFileSync(process.argv[jsonArg + 1], JSON.stringify(rows, null, 2));
  console.log(`明细已写入 ${process.argv[jsonArg + 1]}`);
}
