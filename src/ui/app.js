/**
 * 应用控制器：界面状态机 + 交互 + 渲染循环。
 *
 *   idle ──选位──▶ selected ──规划──▶ planning ──▶ planned ──开始──▶ driving ──▶ done
 *                     ▲                  │失败                                    │
 *                     └──────────────────┴─────── 修改场景/参数 ◀─────────────────┘
 */

import { addCone, createParkingLot, findSlot, removeConeAt } from "../model/parkingLot.js";
import {
  createVehicleParams,
  DEFAULT_VEHICLE,
  minTurningRadius,
  outerSweepRadius,
  vehicleFootprint,
} from "../model/vehicle.js";
import { pointInPolygon } from "../core/geometry.js";
import { deg2rad, normalizeAngle, rad2deg } from "../core/math.js";
import { buildEnvironment } from "../planning/planner.js";
import { ParkingSimulator } from "../sim/simulator.js";
import { createUltrasonicArray } from "../perception/ultrasonic.js";
import { MapRenderer } from "./renderer.js";
import { SERIES_COLORS, TimeSeriesChart } from "./charts.js";
import { Dashboard } from "./dashboard.js";
import { LAYERS, STAGES } from "./stages.js";

const $ = (id) => document.getElementById(id);
const UI = {
  canvas: $("mapCanvas"),
  hint: $("canvasHint"),
  tooltip: $("tooltip"),
  layerChips: $("layerChips"),
  editButton: $("editButton"),
  followButton: $("followButton"),
  resetViewButton: $("resetViewButton"),
  playButton: $("playButton"),
  playIcon: $("playIcon"),
  simClock: $("simClock"),
  timeline: $("timeline"),
  speedSelect: $("speedSelect"),
  stagePill: $("stagePill"),
  selectedSpot: $("selectedSpot"),
  spotType: $("spotType"),
  modeControl: $("modeControl"),
  controllerControl: $("controllerControl"),
  replayToggle: $("replayToggle"),
  planButton: $("planButton"),
  parkButton: $("parkButton"),
  resetButton: $("resetButton"),
  pipelineList: $("pipelineList"),
  planTotal: $("planTotal"),
  stageIndex: $("stageDetailIndex"),
  stageTitle: $("stageDetailTitle"),
  stageSubtitle: $("stageDetailSubtitle"),
  stageBody: $("stageDetailBody"),
  stageFormula: $("stageDetailFormula"),
  kpiGrid: $("kpiGrid"),
  resultBadge: $("resultBadge"),
  steerRange: $("steerRange"),
  usageRange: $("usageRange"),
  marginRange: $("marginRange"),
  noiseRange: $("noiseRange"),
  modelFacts: $("modelFacts"),
  systemDot: $("systemDot"),
  systemText: $("systemText"),
  toast: $("toast"),
};

const SEARCH_REPLAY_MS = 2800;

const state = {
  lot: createParkingLot(),
  sceneVersion: 1,
  vehicle: createVehicleParams(),
  steerUsage: 0.88,
  margin: 0.15,
  noise: 0,
  env: null,
  start: null,
  selectedSlotId: null,
  hoverSlotId: null,
  mode: "reverse-in",
  controller: "rear-wheel-feedback",
  phase: "idle",
  plan: null,
  planCounter: 0,
  requestId: 0,
  replayStart: 0,
  searchProgress: 1,
  sim: null,
  simTarget: 0,
  playing: false,
  viewTime: null,
  layers: Object.fromEntries(LAYERS.map((l) => [l.key, ["tree", "rs", "path", "sensors", "tracking"].includes(l.key)])),
  focusStage: "search",
  editMode: false,
  follow: false,
  drag: null,
  egoInvalid: false,
  egoHover: false,
};
state.start = { ...state.lot.start };

const renderer = new MapRenderer(UI.canvas, state.lot);
const dashboard = new Dashboard($("dashboardCanvas"));
const charts = {
  speed: new TimeSeriesChart($("chartSpeed"), {
    title: "车速 v",
    unit: "m/s",
    symmetric: true,
    series: [
      { key: "vRef", label: "参考", color: SERIES_COLORS.reference, dash: [5, 3] },
      { key: "v", label: "实际", color: SERIES_COLORS.actual },
    ],
  }),
  steer: new TimeSeriesChart($("chartSteer"), {
    title: "前轮转角 δ",
    unit: "°",
    symmetric: true,
    decimals: 1,
    series: [
      { key: "deltaCmd", label: "指令", color: SERIES_COLORS.reference, dash: [5, 3] },
      { key: "delta", label: "实际", color: SERIES_COLORS.actual },
    ],
  }),
  lateral: new TimeSeriesChart($("chartLateral"), {
    title: "横向跟踪误差 e_y",
    unit: "cm",
    symmetric: true,
    decimals: 1,
    series: [{ key: "ey", label: "e_y", color: SERIES_COLORS.actual }],
  }),
  heading: new TimeSeriesChart($("chartHeading"), {
    title: "航向跟踪误差 e_ψ",
    unit: "°",
    symmetric: true,
    decimals: 2,
    series: [{ key: "ePsi", label: "e_ψ", color: SERIES_COLORS.actual }],
  }),
};
let sensorArray = createUltrasonicArray(state.vehicle);

// —— 规划 Worker ——
let worker = null;
try {
  worker = new Worker(new URL("../workers/planner.worker.js", import.meta.url), { type: "module" });
  worker.onmessage = (e) => onPlanResult(e.data);
  worker.onerror = () => {
    worker = null;
  };
} catch {
  worker = null;
}

// ================= 环境与参数 =================

function rebuildEnvironment() {
  state.env = buildEnvironment(state.lot, state.vehicle, { margin: state.margin });
  state.sceneVersion += 1;
  validateStart();
}

function validateStart() {
  state.egoInvalid = state.env.checker.collides(state.start);
}

function rebuildVehicle() {
  state.vehicle = createVehicleParams({ ...DEFAULT_VEHICLE, maxSteer: deg2rad(Number(UI.steerRange.value)) });
  state.steerUsage = Number(UI.usageRange.value);
  state.margin = Number(UI.marginRange.value);
  state.noise = Number(UI.noiseRange.value);
  sensorArray = createUltrasonicArray(state.vehicle);
  rebuildEnvironment();
  renderModelFacts();
}

function renderModelFacts() {
  const v = state.vehicle;
  const planR = v.wheelbase / Math.tan(v.maxSteer * state.steerUsage);
  $("steerOut").textContent = `${UI.steerRange.value}°`;
  $("usageOut").textContent = `${Math.round(state.steerUsage * 100)}%`;
  $("marginOut").textContent = `${state.margin.toFixed(2)} m`;
  $("noiseOut").textContent = state.noise > 0 ? `±${(state.noise * 100).toFixed(0)} cm` : "关闭";
  const facts = [
    ["车长 × 车宽", `${v.length.toFixed(2)} × ${v.width.toFixed(2)} m`],
    ["轴距 L / 轮距", `${v.wheelbase.toFixed(2)} / ${v.track.toFixed(2)} m`],
    ["前悬 / 后悬", `${v.frontOverhang.toFixed(2)} / ${v.rearOverhang.toFixed(2)} m`],
    ["方向盘最大转角", `${Math.round(rad2deg(v.maxSteer) * v.steeringRatio)}°`],
    ["最小转弯半径（后轴）", `${minTurningRadius(v).toFixed(2)} m`],
    ["规划转弯半径", `${planR.toFixed(2)} m`],
    ["外廓扫掠半径", `${outerSweepRadius(v).toFixed(2)} m`],
    ["转向速率上限", `${Math.round(rad2deg(v.maxSteerRate))}°/s`],
    ["车速上限 前进/倒车", `${v.maxSpeedForward} / ${v.maxSpeedReverse} m/s`],
    ["加速度 / 减速度", `${v.maxAccel} / ${v.maxDecel} m/s²`],
  ];
  UI.modelFacts.innerHTML = facts.map(([k, val]) => `<div><dt>${k}</dt><dd>${val}</dd></div>`).join("");
}

// ================= 状态切换 =================

function setPhase(phase) {
  state.phase = phase;
  const slot = state.selectedSlotId ? findSlot(state.lot, state.selectedSlotId) : null;
  const pills = {
    idle: "等待选位",
    selected: "车位已锁定",
    planning: "正在规划",
    planned: "轨迹就绪",
    failed: "规划失败",
    driving: "自动泊车中",
    done: "泊车完成",
  };
  UI.stagePill.textContent = pills[phase] || phase;
  UI.stagePill.classList.toggle("warn", phase === "failed");
  UI.planButton.disabled = !slot || phase === "planning" || phase === "driving" || state.egoInvalid;
  UI.parkButton.disabled = phase !== "planned";
  UI.playButton.disabled = !(phase === "driving" || phase === "done");
  UI.timeline.disabled = phase !== "done";
  const busy = phase === "planning" || phase === "driving";
  UI.systemDot.classList.toggle("busy", busy);
  UI.systemText.textContent = phase === "planning" ? "规划计算中" : phase === "driving" ? "车辆控制已接管" : "系统就绪";
  for (const button of [...UI.modeControl.children, ...UI.controllerControl.children]) {
    button.disabled = busy;
  }
  updateHint();
}

function updateHint() {
  const hints = {
    idle: "点击绿色空闲车位选择目标；拖动白色车辆修改起点（滚轮旋转），在空白处滚轮缩放",
    selected: "目标已锁定，点击“规划路径”",
    planning: "Hybrid A* 正在 Web Worker 中搜索…",
    planned: "轨迹就绪：点击右侧流水线各阶段查看算法细节，或开始泊车",
    failed: "规划失败：尝试更换车位、减小安全裕度或提高转角利用率",
    driving: "闭环控制中 · 图中白线为实际行驶轨迹",
    done: "泊车完成 · 拖动时间轴回看，或选择新车位从当前位置继续",
  };
  UI.hint.textContent = state.editMode ? "编辑模式：点击车位切换占用状态，点击空地放置 / 移除锥桶" : hints[state.phase];
  if (state.egoInvalid && !state.editMode) UI.hint.textContent = "起点与障碍物冲突，请拖动车辆到空旷位置";
}

function selectSlot(slot) {
  if (state.phase === "driving" || state.phase === "planning") return;
  if (state.phase === "done") adoptFinalPose();
  stopSim();
  state.selectedSlotId = slot.id;
  state.plan = null;
  UI.selectedSpot.textContent = slot.id;
  UI.spotType.textContent = slot.type === "parallel" ? `侧方 ${slot.width.toFixed(1)} m` : "垂直 2.5 × 5.3 m";
  setPhase("selected");
  resetPipeline();
  renderKpis();
  clearCharts();
  showToast(`已选择 ${slot.id} 车位`);
}

/** 泊车完成后以当前位置作为下一次规划的起点（可演示“泊出 + 再泊入”） */
function adoptFinalPose() {
  if (!state.sim) return;
  const s = state.sim.state;
  state.start = { x: s.x, y: s.y, theta: s.theta };
  validateStart();
}

function invalidatePlan(message) {
  if (state.phase === "driving") return;
  stopSim();
  state.plan = null;
  const slot = state.selectedSlotId ? findSlot(state.lot, state.selectedSlotId) : null;
  if (slot && slot.occupied) {
    state.selectedSlotId = null;
    UI.selectedSpot.textContent = "尚未选择";
    UI.spotType.textContent = "--";
  }
  setPhase(state.selectedSlotId ? "selected" : "idle");
  resetPipeline();
  renderKpis();
  clearCharts();
  if (message) showToast(message);
}

function stopSim() {
  state.sim = null;
  state.playing = false;
  state.viewTime = null;
  updatePlayIcon();
  UI.simClock.textContent = "0.0 s";
  UI.timeline.value = 0;
}

// ================= 规划 =================

function planRoute() {
  if (!state.selectedSlotId || state.egoInvalid) return;
  stopSim();
  state.plan = null;
  setPhase("planning");
  resetPipeline(true);
  const request = {
    lot: state.lot,
    slotId: state.selectedSlotId,
    start: state.start,
    vehicle: { ...state.vehicle },
    mode: state.mode,
    margin: state.margin,
    steerUsage: state.steerUsage,
  };
  const id = ++state.requestId;
  if (worker) {
    worker.postMessage({ id, request });
  } else {
    // 回退：主线程同步规划
    setTimeout(async () => {
      const { planParking } = await import("../planning/planner.js");
      try {
        onPlanResult({ id, plan: planParking(request) });
      } catch (error) {
        onPlanResult({ id, error: error.message });
      }
    }, 30);
  }
}

function onPlanResult({ id, plan, error }) {
  if (id !== state.requestId || state.phase !== "planning") return;
  if (error || !plan) {
    setPhase("failed");
    showToast(`规划异常：${error}`);
    return;
  }
  plan.id = ++state.planCounter;
  state.plan = plan;
  renderPipeline();
  renderKpis();
  if (!plan.success) {
    setPhase("failed");
    showToast(`规划失败：${plan.reason}`);
    return;
  }
  state.searchProgress = UI.replayToggle.checked ? 0 : 1;
  state.replayStart = performance.now();
  setPhase("planned");
  showPlannedProfile();
  focusStage(UI.replayToggle.checked ? "search" : state.focusStage, false);
  showToast(
    `规划完成：${plan.metrics.length.toFixed(1)} m，换挡 ${plan.metrics.gearShifts} 次，耗时 ${plan.metrics.planningMs.toFixed(0)} ms`,
  );
}

// ================= 仿真 =================

function startParking() {
  if (state.phase !== "planned") return;
  state.searchProgress = 1;
  state.sim = new ParkingSimulator({
    plan: state.plan,
    vehicle: state.vehicle,
    checker: state.env.checker,
    controller: state.controller,
    localizationNoise: state.noise,
  });
  state.simTarget = 0;
  state.playing = true;
  state.viewTime = null;
  updatePlayIcon();
  setPhase("driving");
  focusStage("control", false);
}

function stepSimulation(dtReal) {
  const sim = state.sim;
  if (!sim || state.phase !== "driving" || !state.playing) return;
  state.simTarget += dtReal * Number(UI.speedSelect.value);
  let guard = 0;
  while (sim.time < state.simTarget && !sim.finished && guard < 3000) {
    sim.step();
    guard += 1;
  }
  if (sim.finished) finishParking();
}

function finishParking() {
  const sim = state.sim;
  const result = sim.result();
  state.playing = false;
  state.viewTime = sim.time;
  setPhase("done");
  updatePlayIcon();
  UI.timeline.value = 1000;
  renderKpis(result);
  renderPipeline(result);
  updateCharts(true);
  showToast(
    result.success
      ? `泊车完成：位置误差 ${(result.positionError * 100).toFixed(1)} cm，航向误差 ${Math.abs(rad2deg(result.headingError)).toFixed(2)}°`
      : `泊车异常：${result.collided ? "发生刮碰" : "未能完成"}`,
  );
}

function togglePlay() {
  if (state.phase === "driving") {
    state.playing = !state.playing;
    if (state.playing) state.simTarget = state.sim.time;
  } else if (state.phase === "done") {
    // 回放：从时间轴当前位置开始
    const end = state.sim.time;
    if (state.viewTime === null || state.viewTime >= end - 0.05) state.viewTime = 0;
    state.playing = !state.playing;
  }
  updatePlayIcon();
}

function updatePlayIcon() {
  UI.playIcon.setAttribute("d", state.playing ? "M7 5h4v14H7zM13 5h4v14h-4z" : "M8 5v14l11-7z");
}

/** 当前显示的车辆状态（仿真中取实时状态，完成后按时间轴回看） */
function displayedEgo() {
  const sim = state.sim;
  if (!sim) return { ...state.start, delta: 0, v: 0 };
  if (state.phase === "done" && state.viewTime !== null) {
    const i = logIndexAt(state.viewTime);
    const L = sim.log;
    return { x: L.x[i], y: L.y[i], theta: L.theta[i], delta: L.delta[i], v: L.v[i], gear: L.gear[i] };
  }
  return { ...sim.state, gear: sim.gear };
}

function logIndexAt(t) {
  const ts = state.sim.log.t;
  let lo = 0;
  let hi = ts.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (ts[mid] < t) lo = mid;
    else hi = mid;
  }
  return hi;
}

// ================= 流水线面板 =================

function resetPipeline(running = false) {
  UI.planTotal.textContent = "-- ms";
  UI.pipelineList.innerHTML = STAGES.map(
    (s, i) => `<li data-key="${s.key}" class="${running && s.key !== "control" ? "running" : ""} ${s.key === state.focusStage ? "focus" : ""}">
      <span class="idx">${String(i + 1).padStart(2, "0")}</span>
      <div><strong>${s.name}</strong><small>${s.subtitle}</small></div>
      <span class="ms">${running && s.key !== "control" ? "…" : ""}</span>
    </li>`,
  ).join("");
}

function renderPipeline(result = state.sim?.finished ? state.sim.result() : null) {
  const plan = state.plan;
  if (!plan) return resetPipeline();
  const ms = Object.fromEntries(plan.stages.map((s) => [s.key, s.ms]));
  const st = plan.search.stats;
  const deg = (r) => `${rad2deg(r).toFixed(0)}°`;
  const info = {
    env: [`${plan.gridMap.cols}×${plan.gridMap.rows} 栅格 · 分辨率 ${plan.gridMap.resolution} m`, ms.env],
    goal: [`(${plan.goal.x.toFixed(2)}, ${plan.goal.y.toFixed(2)}, ${deg(plan.goal.theta)})`, ms.goal],
    heuristic: [`R_plan = ${plan.radius.toFixed(2)} m · Dijkstra 代价图`, ms.heuristic],
    search: plan.success
      ? [`扩展 ${st.iterations} 节点 · RS 尝试 ${st.rsAttempts} 次 · ${plan.search.analytic?.word || "--"}`, ms.search]
      : [plan.reason, ms.search],
    smooth: plan.success ? [`${plan.metrics.smoothedSegments}/${plan.metrics.segments} 段完成平滑`, ms.smooth] : ["--", null],
    speed: plan.success
      ? [`${plan.metrics.length.toFixed(1)} m · ${plan.metrics.duration.toFixed(1)} s · 换挡 ${plan.metrics.gearShifts}`, ms.speed]
      : ["--", null],
    control: result
      ? [`终点误差 ${(result.positionError * 100).toFixed(1)} cm / ${Math.abs(rad2deg(result.headingError)).toFixed(2)}°`, null]
      : [state.controller === "pure-pursuit" ? "纯跟踪 Pure Pursuit" : "后轮反馈 Rear-Wheel Feedback", null],
  };
  UI.pipelineList.innerHTML = STAGES.map((s, i) => {
    const [text, time] = info[s.key];
    let cls = "";
    if (s.key === "control") cls = result ? (result.success ? "done" : "failed") : "";
    else if (time !== null && time !== undefined) cls = s.key === "search" && !plan.success ? "failed" : "done";
    return `<li data-key="${s.key}" class="${cls} ${s.key === state.focusStage ? "focus" : ""}">
      <span class="idx">${cls === "done" ? "✓" : String(i + 1).padStart(2, "0")}</span>
      <div><strong>${s.name}</strong><small title="${text}">${text}</small></div>
      <span class="ms">${time !== null && time !== undefined ? `${time.toFixed(time < 10 ? 1 : 0)} ms` : ""}</span>
    </li>`;
  }).join("");
  UI.planTotal.textContent = `${plan.stages.reduce((sum, s) => sum + s.ms, 0).toFixed(0)} ms`;
  return undefined;
}

function focusStage(key, applyLayers = true) {
  state.focusStage = key;
  const index = STAGES.findIndex((s) => s.key === key);
  const stage = STAGES[index];
  UI.stageIndex.textContent = String(index + 1).padStart(2, "0");
  UI.stageTitle.textContent = stage.name;
  UI.stageSubtitle.textContent = stage.subtitle;
  UI.stageBody.textContent = stage.body;
  UI.stageFormula.textContent = stage.formula;
  for (const li of UI.pipelineList.children) li.classList.toggle("focus", li.dataset.key === key);
  if (applyLayers) {
    for (const layer of LAYERS) state.layers[layer.key] = stage.layers.includes(layer.key);
    renderLayerChips();
    if (key === "search" && state.plan?.success && state.phase === "planned") {
      state.searchProgress = 0;
      state.replayStart = performance.now();
    }
  }
}

// ================= 评估 KPI =================

function renderKpis(result = state.sim?.finished ? state.sim.result() : null) {
  const plan = state.plan;
  const m = plan?.success ? plan.metrics : null;
  const fmt = (value, unit, cls = "") => `<dd class="${cls}">${value}<small>${unit}</small></dd>`;
  const items = [
    ["规划总耗时", m ? fmt(m.planningMs.toFixed(0), "ms") : fmt("--", "")],
    ["搜索扩展节点", plan ? fmt(plan.search.stats.iterations, "个") : fmt("--", "")],
    ["路径长度", m ? fmt(m.length.toFixed(1), "m") : fmt("--", "")],
    ["换挡次数", m ? fmt(m.gearShifts, "次") : fmt("--", "")],
    ["规划最小障碍距离", m ? fmt(m.minClearance.toFixed(2), "m") : fmt("--", "")],
    ["规划用时", m ? fmt(m.duration.toFixed(1), "s") : fmt("--", "")],
  ];
  if (result) {
    const posCm = result.positionError * 100;
    const headDeg = Math.abs(rad2deg(result.headingError));
    items.push(
      ["终点位置误差", fmt(posCm.toFixed(1), "cm", posCm < 10 ? "good" : "bad")],
      ["终点航向误差", fmt(headDeg.toFixed(2), "°", headDeg < 3 ? "good" : "bad")],
      ["最大横向跟踪误差", fmt((result.maxLateralError * 100).toFixed(1), "cm")],
      ["实际最小障碍距离", fmt(result.minClearance.toFixed(2), "m", result.collided ? "bad" : "good")],
      ["实际用时 / 里程", fmt(`${result.time.toFixed(1)} s / ${result.distance.toFixed(1)}`, "m")],
      ["碰撞 / AEB 触发", fmt(`${result.collided ? "是" : "否"} / ${result.aebTriggered}`, "", result.collided ? "bad" : "good")],
    );
    UI.resultBadge.textContent = result.success ? "泊车成功" : "泊车失败";
    UI.resultBadge.className = `result-badge ${result.success ? "ok" : "bad"}`;
  } else {
    UI.resultBadge.textContent = plan ? (plan.success ? "已规划" : "规划失败") : "未运行";
    UI.resultBadge.className = `result-badge ${plan && !plan.success ? "bad" : ""}`;
  }
  UI.kpiGrid.innerHTML = items.map(([k, v]) => `<div><dt>${k}</dt>${v}</div>`).join("");
}

// ================= 图表 =================

function gearBands(segments, timeOf) {
  return segments.map((seg, i) => ({
    t0: timeOf(i, 0),
    t1: timeOf(i, 1),
    color: seg.dir > 0 ? "rgba(69, 230, 167, 0.06)" : "rgba(255, 162, 76, 0.08)",
  }));
}

function clearCharts() {
  for (const chart of Object.values(charts)) chart.setData(null, { duration: 10 });
}

function showPlannedProfile() {
  const plan = state.plan;
  const pts = plan.trajectory.points;
  const data = {
    t: pts.map((p) => p.tGlobal),
    vRef: pts.map((p) => p.v * p.dir),
    deltaCmd: pts.map((p) => rad2deg(p.steer)),
  };
  const bands = gearBands(plan.segments, (i, end) =>
    end ? plan.segments[i].tStart + plan.segments[i].duration : plan.segments[i].tStart,
  );
  const duration = plan.trajectory.duration;
  charts.speed.setData(data, { duration, bands });
  charts.steer.setData(data, { duration, bands });
  charts.lateral.setData(null, { duration, bands });
  charts.heading.setData(null, { duration, bands });
  for (const chart of Object.values(charts)) chart.setCursor(null);
}

let lastChartUpdate = 0;
function updateCharts(force = false) {
  const sim = state.sim;
  if (!sim) return;
  const now = performance.now();
  if (!force && now - lastChartUpdate < 80) return;
  lastChartUpdate = now;
  const L = sim.log;
  const data = {
    t: L.t,
    v: L.v,
    vRef: L.vRef,
    delta: L.delta.map(rad2deg),
    deltaCmd: L.deltaCmd.map(rad2deg),
    ey: L.ey.map((e) => e * 100),
    ePsi: L.ePsi.map(rad2deg),
  };
  const duration = Math.max(sim.time, state.plan.trajectory.duration * 1.08);
  const bands = [];
  let startIdx = 0;
  for (let i = 1; i <= L.t.length; i += 1) {
    if (i === L.t.length || L.gear[i] !== L.gear[startIdx]) {
      const g = L.gear[startIdx];
      if (g === "D" || g === "R") {
        bands.push({ t0: L.t[startIdx], t1: L.t[i - 1], color: g === "D" ? "rgba(69, 230, 167, 0.06)" : "rgba(255, 162, 76, 0.08)" });
      }
      startIdx = i;
    }
  }
  for (const chart of Object.values(charts)) {
    chart.setData(data, { duration, bands });
    chart.setCursor(state.phase === "done" ? state.viewTime : null);
  }
}

// ================= 图层开关 =================

function renderLayerChips() {
  UI.layerChips.innerHTML = LAYERS.map(
    (l) => `<button type="button" class="chip ${state.layers[l.key] ? "on" : ""}" data-key="${l.key}" style="--chip-color:${l.color}"><i></i>${l.label}</button>`,
  ).join("");
}

UI.layerChips.addEventListener("click", (e) => {
  const chip = e.target.closest(".chip");
  if (!chip) return;
  const key = chip.dataset.key;
  state.layers[key] = !state.layers[key];
  if (key === "speedColor" && state.layers.speedColor) state.layers.path = true;
  renderLayerChips();
});

// ================= 地图交互 =================

function eventPoint(e) {
  const rect = UI.canvas.getBoundingClientRect();
  const px = e.clientX - rect.left;
  const py = e.clientY - rect.top;
  return { px, py, ...renderer.screenToWorld(px, py) };
}

function slotAt(x, y) {
  return state.lot.slots.find((s) => pointInPolygon(x, y, s.polygon));
}

function canEditStart() {
  return ["idle", "selected", "planned", "failed", "done"].includes(state.phase) && !state.editMode;
}

function onEgo(x, y) {
  if (state.sim && state.phase !== "done") return false;
  const pose = state.phase === "done" ? displayedEgo() : state.start;
  return pointInPolygon(x, y, vehicleFootprint(state.vehicle, pose, 0.1));
}

UI.canvas.addEventListener("pointerdown", (e) => {
  const p = eventPoint(e);
  UI.canvas.setPointerCapture(e.pointerId);
  if (canEditStart() && onEgo(p.x, p.y)) {
    if (state.phase === "done") adoptFinalPose();
    const pose = state.start;
    state.drag = { kind: "ego", dx: pose.x - p.x, dy: pose.y - p.y, moved: false };
  } else {
    state.drag = { kind: "pan", px: p.px, py: p.py, moved: false };
  }
});

UI.canvas.addEventListener("pointermove", (e) => {
  const p = eventPoint(e);
  const drag = state.drag;
  if (drag?.kind === "ego") {
    drag.moved = true;
    state.start = { x: p.x + drag.dx, y: p.y + drag.dy, theta: state.start.theta };
    if (state.sim) stopSim();
    validateStart();
    return;
  }
  if (drag?.kind === "pan") {
    const dx = p.px - drag.px;
    const dy = p.py - drag.py;
    if (Math.hypot(dx, dy) > 4 || drag.moved) {
      drag.moved = true;
      if (renderer.zoom > 1) renderer.pan(dx, dy);
      drag.px = p.px;
      drag.py = p.py;
    }
    return;
  }
  state.egoHover = canEditStart() && onEgo(p.x, p.y);
  const slot = slotAt(p.x, p.y);
  state.hoverSlotId = slot?.id || null;
  UI.canvas.style.cursor = state.egoHover ? "grab" : slot ? "pointer" : "crosshair";
  showTooltip(p, slot);
});

UI.canvas.addEventListener("pointerup", (e) => {
  const drag = state.drag;
  state.drag = null;
  if (!drag) return;
  if (drag.kind === "ego") {
    if (drag.moved) invalidatePlan(state.egoInvalid ? "起点与障碍物冲突" : "起点已更新");
    return;
  }
  if (drag.moved) return;
  const p = eventPoint(e);
  const slot = slotAt(p.x, p.y);
  if (state.editMode) {
    editSceneAt(p, slot);
    return;
  }
  if (!slot) return;
  if (slot.occupied) {
    showToast(`${slot.id} 已被占用，请选择空闲车位（或进入编辑模式修改）`);
    return;
  }
  selectSlot(slot);
});

UI.canvas.addEventListener("pointerleave", () => {
  state.hoverSlotId = null;
  state.egoHover = false;
  UI.tooltip.hidden = true;
});

UI.canvas.addEventListener(
  "wheel",
  (e) => {
    e.preventDefault();
    const p = eventPoint(e);
    if (canEditStart() && onEgo(p.x, p.y) && state.phase !== "done") {
      state.start = { ...state.start, theta: normalizeAngle(state.start.theta + (e.deltaY > 0 ? -1 : 1) * deg2rad(5)) };
      validateStart();
      if (state.plan) invalidatePlan();
      return;
    }
    state.follow = false;
    UI.followButton.classList.remove("on");
    renderer.zoomAt(p.px, p.py, e.deltaY > 0 ? 1 / 1.15 : 1.15);
  },
  { passive: false },
);

function editSceneAt(p, slot) {
  if (state.phase === "driving" || state.phase === "planning") return;
  if (slot && slot.id !== state.selectedSlotId) {
    slot.occupied = !slot.occupied;
    rebuildEnvironment();
    invalidatePlan(`${slot.id} 已设为${slot.occupied ? "占用" : "空闲"}`);
    return;
  }
  if (removeConeAt(state.lot, p.x, p.y)) {
    rebuildEnvironment();
    invalidatePlan("已移除锥桶");
    return;
  }
  addCone(state.lot, p.x, p.y);
  rebuildEnvironment();
  invalidatePlan("已放置锥桶（临时障碍物）");
}

function showTooltip(p, slot) {
  if (!slot && !state.egoHover) {
    UI.tooltip.hidden = true;
    return;
  }
  let html;
  if (state.egoHover) {
    const s = state.start;
    html = `<strong>本车起点</strong><br>后轴中心 (${s.x.toFixed(2)}, ${s.y.toFixed(2)}) m<br>航向 ${rad2deg(s.theta).toFixed(0)}° · 拖动移动 / 滚轮旋转`;
  } else {
    const kind = slot.type === "parallel" ? `侧方车位 ${slot.width.toFixed(1)} × ${slot.depth.toFixed(1)} m` : "垂直车位 2.5 × 5.3 m";
    html = `<strong>${slot.id}</strong> · ${slot.occupied ? "<span style='color:#ff766f'>已占用</span>" : "<span style='color:#45e6a7'>空闲</span>"}<br>${kind}`;
  }
  UI.tooltip.innerHTML = html;
  UI.tooltip.hidden = false;
  const wrap = UI.canvas.getBoundingClientRect();
  const left = Math.min(p.px + 14, wrap.width - UI.tooltip.offsetWidth - 8);
  const top = p.py + 16 + UI.tooltip.offsetHeight > wrap.height ? p.py - UI.tooltip.offsetHeight - 10 : p.py + 16;
  UI.tooltip.style.left = `${left}px`;
  UI.tooltip.style.top = `${top}px`;
}

// ================= 控件事件 =================

function bindSegmented(container, onChange) {
  container.addEventListener("click", (e) => {
    const button = e.target.closest("button");
    if (!button || button.disabled) return;
    for (const b of container.children) b.classList.toggle("active", b === button);
    onChange(button.dataset.value);
  });
}

bindSegmented(UI.modeControl, (value) => {
  state.mode = value;
  if (state.plan) invalidatePlan(`泊车方式：${value === "head-in" ? "车头入库" : "倒车入库"}，请重新规划`);
});

bindSegmented(UI.controllerControl, (value) => {
  state.controller = value;
  if (state.phase === "done") {
    state.sim = null;
    setPhase("planned");
    state.searchProgress = 1;
    showPlannedProfile();
    renderKpis();
    renderPipeline();
  } else if (state.plan) {
    renderPipeline();
  }
  showToast(`跟踪控制器：${value === "pure-pursuit" ? "纯跟踪 Pure Pursuit" : "后轮反馈 Rear-Wheel Feedback"}`);
});

UI.pipelineList.addEventListener("click", (e) => {
  const li = e.target.closest("li");
  if (li) focusStage(li.dataset.key);
});

UI.planButton.addEventListener("click", planRoute);
UI.parkButton.addEventListener("click", startParking);
UI.playButton.addEventListener("click", togglePlay);
UI.resetButton.addEventListener("click", () => {
  if (state.phase === "planning") state.requestId += 1;
  state.start = { ...state.lot.start };
  state.selectedSlotId = null;
  UI.selectedSpot.textContent = "尚未选择";
  UI.spotType.textContent = "--";
  validateStart();
  state.phase = "idle";
  invalidatePlan();
  renderer.resetView();
});

UI.editButton.addEventListener("click", () => {
  state.editMode = !state.editMode;
  UI.editButton.classList.toggle("on", state.editMode);
  updateHint();
});

UI.followButton.addEventListener("click", () => {
  state.follow = !state.follow;
  UI.followButton.classList.toggle("on", state.follow);
  if (!state.follow) renderer.resetView();
});

UI.resetViewButton.addEventListener("click", () => {
  state.follow = false;
  UI.followButton.classList.remove("on");
  renderer.resetView();
});

UI.timeline.addEventListener("input", () => {
  if (state.phase !== "done" || !state.sim) return;
  state.playing = false;
  updatePlayIcon();
  state.viewTime = (Number(UI.timeline.value) / 1000) * state.sim.time;
  updateCharts(true);
});

for (const input of [UI.steerRange, UI.usageRange, UI.marginRange]) {
  input.addEventListener("input", () => {
    rebuildVehicle();
    invalidatePlan();
  });
}
UI.noiseRange.addEventListener("input", () => {
  state.noise = Number(UI.noiseRange.value);
  renderModelFacts();
});

let toastTimer;
function showToast(message) {
  window.clearTimeout(toastTimer);
  UI.toast.textContent = message;
  UI.toast.classList.add("show");
  toastTimer = window.setTimeout(() => UI.toast.classList.remove("show"), 2800);
}

// ================= 渲染循环 =================

function resizeAll() {
  renderer.resize();
  dashboard.resize();
  for (const chart of Object.values(charts)) chart.resize();
  for (const chart of Object.values(charts)) chart.render();
}

new ResizeObserver(resizeAll).observe(document.querySelector(".layout"));

let lastFrame = performance.now();
function frame(now) {
  const dt = Math.min(0.1, (now - lastFrame) / 1000);
  lastFrame = now;

  if (state.plan && state.searchProgress < 1) {
    state.searchProgress = Math.min(1, (now - state.replayStart) / SEARCH_REPLAY_MS);
    if (state.searchProgress >= 1 && state.phase === "planned" && state.focusStage === "search") {
      UI.hint.textContent = "搜索完成：绿色曲线为命中目标的 Reeds-Shepp 解析扩展";
    }
  }

  stepSimulation(dt);
  if (state.phase === "done" && state.playing && state.sim) {
    state.viewTime = Math.min(state.sim.time, (state.viewTime || 0) + dt * Number(UI.speedSelect.value));
    if (state.viewTime >= state.sim.time) {
      state.playing = false;
      updatePlayIcon();
    }
    updateCharts(true);
  }

  const sim = state.sim;
  const ego = displayedEgo();
  const live = sim && state.phase === "driving";
  const readings = sensorArray.scan(ego, state.env.checker);

  if (sim) {
    const t = state.phase === "done" ? state.viewTime ?? sim.time : sim.time;
    UI.simClock.textContent = `${t.toFixed(1)} s`;
    if (live) {
      UI.timeline.value = Math.min(1000, (sim.time / (state.plan.trajectory.duration * 1.08)) * 1000);
      updateCharts();
    } else if (state.phase === "done" && state.playing) {
      UI.timeline.value = (t / sim.time) * 1000;
    }
  }

  if (state.follow && sim) renderer.follow(ego.x, ego.y, 2.4);

  let trail = null;
  if (sim) {
    const L = sim.log;
    const end = state.phase === "done" && state.viewTime !== null ? logIndexAt(state.viewTime) : L.x.length - 1;
    trail = [];
    for (let i = 0; i <= end; i += 3) trail.push({ x: L.x[i], y: L.y[i] });
  }

  renderer.render({
    lot: state.lot,
    obstacles: state.env.obstacles,
    sceneVersion: state.sceneVersion,
    vehicle: state.vehicle,
    plan: state.plan,
    layers: state.layers,
    searchProgress: state.plan ? state.searchProgress : 1,
    selectedSlotId: state.selectedSlotId,
    hoverSlotId: state.hoverSlotId,
    ego,
    gear: live ? sim.gear : ego.gear || "P",
    braking: live && sim.lastCommand && Math.abs(sim.lastCommand.v) < Math.abs(sim.state.v) - 0.05,
    readings: state.layers.sensors ? readings : null,
    control: live ? sim.lastControl : null,
    trail,
    egoInvalid: state.egoInvalid && !sim,
    egoHover: state.egoHover,
  });

  dashboard.render({
    vehicle: state.vehicle,
    state: ego,
    gear: live ? sim.gear : sim && state.phase === "done" ? ego.gear || "P" : "P",
    readings: readings || [],
  });

  for (const chart of Object.values(charts)) chart.render();
  requestAnimationFrame(frame);
}

// ================= 启动 =================

rebuildVehicle();
renderLayerChips();
resetPipeline();
focusStage("search", false);
renderKpis();
clearCharts();
setPhase("idle");
resizeAll();
requestAnimationFrame(frame);
