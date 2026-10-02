/**
 * 地图渲染器（Canvas 2D）。
 *
 * 直接在世界坐标系（米，y 轴向上）中绘图：通过 setTransform 把米映射为像素，
 * 文字等需要屏幕坐标的元素单独切换变换。
 * 性能策略：静态背景、搜索树分别缓存在离屏画布上，搜索树按回放进度增量绘制。
 */

import { ackermannAngles, centerOffset, vehicleFootprint } from "../model/vehicle.js";

export const PALETTE = {
  bg: "#0a1412",
  asphalt: "#101b19",
  line: "rgba(214, 232, 226, 0.55)",
  lineFaint: "rgba(214, 232, 226, 0.16)",
  label: "rgba(214, 232, 226, 0.5)",
  accent: "#45e6a7",
  forward: "#45e6a7",
  reverse: "#ffa24c",
  treeForward: "rgba(90, 160, 255, 0.32)",
  treeReverse: "rgba(255, 150, 70, 0.32)",
  danger: "#ff6b66",
  warning: "#f4c86a",
  ego: "#2fd39a",
  parked: ["#4f5f63", "#3d5660", "#665f52", "#4a5360", "#56695f", "#6a5757"],
};

export class MapRenderer {
  constructor(canvas, world) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.world = world;
    this.dpr = 1;
    this.width = 0;
    this.height = 0;
    this.zoom = 1;
    this.center = { x: world.width / 2, y: world.height / 2 };
    this.bgCache = { canvas: document.createElement("canvas"), key: null };
    this.treeCache = { canvas: document.createElement("canvas"), key: null, drawn: 0 };
    this.heatCache = new Map();
  }

  // —— 视图变换 ——

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    this.dpr = Math.min(2, window.devicePixelRatio || 1);
    this.width = Math.max(1, rect.width);
    this.height = Math.max(1, rect.height);
    this.canvas.width = Math.round(this.width * this.dpr);
    this.canvas.height = Math.round(this.height * this.dpr);
    for (const cache of [this.bgCache, this.treeCache]) {
      cache.canvas.width = this.canvas.width;
      cache.canvas.height = this.canvas.height;
      cache.key = null;
    }
  }

  get baseScale() {
    return Math.min(this.width / (this.world.width + 1.6), this.height / (this.world.height + 1.6));
  }

  get scale() {
    return this.baseScale * this.zoom;
  }

  get viewKey() {
    return `${this.width}x${this.height}@${this.zoom.toFixed(4)}:${this.center.x.toFixed(3)},${this.center.y.toFixed(3)}`;
  }

  worldToScreen(x, y) {
    const s = this.scale;
    return { x: (x - this.center.x) * s + this.width / 2, y: this.height / 2 - (y - this.center.y) * s };
  }

  screenToWorld(px, py) {
    const s = this.scale;
    return { x: (px - this.width / 2) / s + this.center.x, y: (this.height / 2 - py) / s + this.center.y };
  }

  zoomAt(px, py, factor) {
    const before = this.screenToWorld(px, py);
    this.zoom = Math.min(8, Math.max(1, this.zoom * factor));
    const after = this.screenToWorld(px, py);
    this.center.x += before.x - after.x;
    this.center.y += before.y - after.y;
    this.clampCenter();
  }

  pan(dxPx, dyPx) {
    this.center.x -= dxPx / this.scale;
    this.center.y += dyPx / this.scale;
    this.clampCenter();
  }

  follow(x, y, zoom) {
    this.zoom += (zoom - this.zoom) * 0.08;
    this.center.x += (x - this.center.x) * 0.1;
    this.center.y += (y - this.center.y) * 0.1;
    this.clampCenter();
  }

  resetView() {
    this.zoom = 1;
    this.center = { x: this.world.width / 2, y: this.world.height / 2 };
  }

  clampCenter() {
    const halfW = this.width / 2 / this.scale;
    const halfH = this.height / 2 / this.scale;
    const pad = 0.8;
    const clampAxis = (c, half, size) =>
      half * 2 >= size + 2 * pad ? size / 2 : Math.min(size + pad - half, Math.max(half - pad, c));
    this.center.x = clampAxis(this.center.x, halfW, this.world.width);
    this.center.y = clampAxis(this.center.y, halfH, this.world.height);
  }

  applyWorld(ctx) {
    const s = this.scale * this.dpr;
    ctx.setTransform(
      s,
      0,
      0,
      -s,
      this.dpr * (this.width / 2 - this.center.x * this.scale),
      this.dpr * (this.height / 2 + this.center.y * this.scale),
    );
  }

  applyScreen(ctx) {
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
  }

  /** 屏幕像素宽度换算为世界坐标长度 */
  px(n) {
    return n / this.scale;
  }

  // —— 主绘制 ——

  render(frame) {
    const ctx = this.ctx;
    this.applyScreen(ctx);
    ctx.clearRect(0, 0, this.width, this.height);

    this.#drawBackground(frame);
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.drawImage(this.bgCache.canvas, 0, 0);

    this.applyWorld(ctx);
    const L = frame.layers;
    const plan = frame.plan;
    if (plan?.gridMap && L.costmap) this.#drawHeat(ctx, "costmap", plan, plan.gridMap);
    if (plan?.heuristic2d && L.heuristic) this.#drawHeat(ctx, "heuristic", plan, plan.heuristic2d);

    this.#drawSlotStates(ctx, frame);
    if (plan) this.#drawGoal(ctx, plan);

    if (plan?.search && L.tree) {
      this.#drawTree(plan, frame.searchProgress);
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      ctx.drawImage(this.treeCache.canvas, 0, 0);
      this.applyWorld(ctx);
    }
    if (plan?.search && L.rs) this.#drawRsTrials(ctx, plan, frame.searchProgress);

    const showPath = plan?.success && frame.searchProgress >= 1;
    if (showPath && L.raw) this.#drawRawPath(ctx, plan);
    if (showPath && L.sweep) this.#drawSweep(ctx, plan, frame.vehicle);
    if (showPath && L.path) this.#drawTrajectory(ctx, plan, L.speedColor);
    if (frame.trail && frame.trail.length > 1) this.#drawTrail(ctx, frame.trail);

    if (L.sensors && frame.readings) this.#drawSensors(ctx, frame.readings);
    if (L.tracking && frame.control && frame.ego) this.#drawTracking(ctx, frame);
    if (frame.ego) {
      this.drawEgo(ctx, frame.vehicle, frame.ego, {
        gear: frame.gear,
        braking: frame.braking,
        invalid: frame.egoInvalid,
        highlight: frame.egoHover,
      });
    }

    this.applyScreen(ctx);
    this.#drawScaleBar(ctx);
    if (frame.searchProgress < 1 && plan?.search) this.#drawSearchBadge(ctx, plan, frame.searchProgress);
  }

  // —— 静态背景（缓存） ——

  #drawBackground(frame) {
    const key = `${this.viewKey}|${frame.sceneVersion}`;
    if (this.bgCache.key === key) return;
    this.bgCache.key = key;
    const ctx = this.bgCache.canvas.getContext("2d");
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, this.bgCache.canvas.width, this.bgCache.canvas.height);
    this.applyWorld(ctx);
    const { lot } = frame;
    const W = lot.width;
    const H = lot.height;

    ctx.fillStyle = PALETTE.bg;
    ctx.fillRect(-5, -5, W + 10, H + 10);
    ctx.fillStyle = PALETTE.asphalt;
    ctx.fillRect(0, 0, W, H);

    // 1 m 网格
    ctx.strokeStyle = "rgba(160, 200, 190, 0.045)";
    ctx.lineWidth = this.px(1);
    ctx.beginPath();
    for (let x = 0; x <= W; x += 1) {
      ctx.moveTo(x, 0);
      ctx.lineTo(x, H);
    }
    for (let y = 0; y <= H; y += 1) {
      ctx.moveTo(0, y);
      ctx.lineTo(W, y);
    }
    ctx.stroke();

    this.#drawLaneMarkings(ctx, lot);
    for (const slot of lot.slots) this.#drawSlotLines(ctx, slot);

    for (const o of frame.obstacles) {
      if (o.kind === "wall") this.#drawWall(ctx, o);
      else if (o.kind === "island") this.#drawIsland(ctx, o);
      else if (o.kind === "pillar") this.#drawPillar(ctx, o);
      else if (o.kind === "car") this.#drawParkedCar(ctx, o);
      else if (o.kind === "cone") this.#drawCone(ctx, o);
    }

    // 车位编号
    for (const slot of lot.slots) {
      const inset = slot.type === "parallel" ? 0 : 0.45;
      const lx = slot.opening.x + Math.cos(slot.inward) * inset;
      const ly = slot.opening.y + Math.sin(slot.inward) * (slot.type === "parallel" ? slot.depth / 2 : inset);
      const anchorY = slot.type === "parallel" ? ly + (slot.inward > 0 ? 0.75 : -0.75) : ly;
      this.text(ctx, slot.id, lx, anchorY, { size: 9.5, color: slot.occupied ? "rgba(214,232,226,0.32)" : "rgba(69,230,167,0.85)", weight: 700 });
    }
    this.text(ctx, "入口 ENTRANCE", 1.2, 25.2, { size: 10, color: "rgba(69,230,167,0.7)", align: "left", weight: 700 });
    this.text(ctx, "主通道 6.5 m", 25, 22.5, { size: 10, color: "rgba(214,232,226,0.28)" });
    this.text(ctx, "支路 6.0 m", 25, 6.45, { size: 10, color: "rgba(214,232,226,0.28)" });
  }

  #drawLaneMarkings(ctx, lot) {
    const { aisle, street } = lot.layout;
    const midA = (aisle.y0 + aisle.y1) / 2;
    const midS = (street.y0 + street.y1) / 2;
    ctx.strokeStyle = "rgba(244, 200, 106, 0.32)";
    ctx.lineWidth = 0.1;
    ctx.setLineDash([1.2, 1.2]);
    ctx.beginPath();
    ctx.moveTo(4.5, midA);
    ctx.lineTo(45.5, midA);
    ctx.moveTo(4.5, midS);
    ctx.lineTo(45.5, midS);
    ctx.moveTo(4.5, midS);
    ctx.lineTo(4.5, midA);
    ctx.moveTo(45.5, midS);
    ctx.lineTo(45.5, midA);
    ctx.stroke();
    ctx.setLineDash([]);
    // 行车方向箭头：双向通行、靠右行驶
    ctx.fillStyle = "rgba(214, 232, 226, 0.12)";
    const arrow = (x, y, theta) => {
      ctx.save();
      ctx.translate(x, y);
      ctx.rotate(theta);
      ctx.beginPath();
      ctx.moveTo(1.3, 0);
      ctx.lineTo(0.3, 0.6);
      ctx.lineTo(0.3, 0.22);
      ctx.lineTo(-1.2, 0.22);
      ctx.lineTo(-1.2, -0.22);
      ctx.lineTo(0.3, -0.22);
      ctx.lineTo(0.3, -0.6);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
    };
    for (const x of [15, 35]) {
      arrow(x, midA - 1.6, 0);
      arrow(x + 4, midA + 1.6, Math.PI);
      arrow(x, midS - 1.5, 0);
      arrow(x + 4, midS + 1.5, Math.PI);
    }
  }

  #drawSlotLines(ctx, slot) {
    const p = slot.polygon.points;
    ctx.strokeStyle = PALETTE.line;
    ctx.lineWidth = 0.1;
    ctx.beginPath();
    if (slot.type === "perpendicular") {
      ctx.moveTo(p[1].x, p[1].y);
      ctx.lineTo(p[0].x, p[0].y);
      ctx.lineTo(p[3].x, p[3].y);
      ctx.lineTo(p[2].x, p[2].y);
    } else {
      ctx.moveTo(p[0].x, p[0].y);
      for (let i = 1; i <= 4; i += 1) ctx.lineTo(p[i % 4].x, p[i % 4].y);
    }
    ctx.stroke();
    if (!slot.occupied) {
      ctx.fillStyle = "rgba(69, 230, 167, 0.035)";
      fillPolygon(ctx, slot.polygon);
    }
  }

  #drawWall(ctx, o) {
    ctx.fillStyle = "#1f2d2a";
    fillPolygon(ctx, o.polygon);
    ctx.strokeStyle = "rgba(160, 190, 182, 0.35)";
    ctx.lineWidth = this.px(1);
    strokePolygon(ctx, o.polygon);
  }

  #drawIsland(ctx, o) {
    ctx.fillStyle = "#13301f";
    fillPolygon(ctx, o.polygon);
    ctx.strokeStyle = "rgba(120, 200, 140, 0.45)";
    ctx.lineWidth = 0.08;
    strokePolygon(ctx, o.polygon);
    const pts = o.polygon.points;
    const x0 = Math.min(...pts.map((p) => p.x));
    const x1 = Math.max(...pts.map((p) => p.x));
    const yc = (Math.min(...pts.map((p) => p.y)) + Math.max(...pts.map((p) => p.y))) / 2;
    for (let x = x0 + 1.2; x < x1 - 0.5; x += 2.4) {
      ctx.fillStyle = "rgba(70, 150, 90, 0.55)";
      ctx.beginPath();
      ctx.arc(x, yc + ((x * 7) % 3) * 0.12 - 0.12, 0.62, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = "rgba(110, 190, 120, 0.35)";
      ctx.beginPath();
      ctx.arc(x - 0.15, yc + 0.1, 0.3, 0, Math.PI * 2);
      ctx.fill();
    }
    this.text(ctx, "绿化带", (x0 + x1) / 2, yc, { size: 9, color: "rgba(160, 220, 170, 0.6)" });
  }

  #drawPillar(ctx, o) {
    ctx.fillStyle = "#3a4643";
    fillPolygon(ctx, o.polygon);
    ctx.strokeStyle = "rgba(244, 200, 106, 0.7)";
    ctx.lineWidth = 0.06;
    strokePolygon(ctx, o.polygon);
    const p = o.polygon.points;
    ctx.beginPath();
    ctx.moveTo(p[0].x, p[0].y);
    ctx.lineTo(p[2].x, p[2].y);
    ctx.moveTo(p[1].x, p[1].y);
    ctx.lineTo(p[3].x, p[3].y);
    ctx.stroke();
  }

  #drawCone(ctx, o) {
    ctx.fillStyle = "#ff8a3d";
    ctx.beginPath();
    ctx.arc(o.x, o.y, o.radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "#fff3e6";
    ctx.beginPath();
    ctx.arc(o.x, o.y, o.radius * 0.45, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "#ff8a3d";
    ctx.beginPath();
    ctx.arc(o.x, o.y, o.radius * 0.2, 0, Math.PI * 2);
    ctx.fill();
  }

  #drawParkedCar(ctx, car) {
    const color = PALETTE.parked[car.color % PALETTE.parked.length];
    drawCarBody(ctx, car.pose.x, car.pose.y, car.pose.theta, car.length, car.width, color, { parked: true });
  }

  // —— 动态层 ——

  #drawSlotStates(ctx, frame) {
    const t = performance.now();
    for (const slot of frame.lot.slots) {
      const selected = slot.id === frame.selectedSlotId;
      const hovered = slot.id === frame.hoverSlotId;
      if (!selected && !hovered) continue;
      if (selected) {
        const pulse = 0.12 + Math.sin(t / 380) * 0.05;
        ctx.fillStyle = `rgba(69, 230, 167, ${pulse})`;
        fillPolygon(ctx, slot.polygon);
        ctx.strokeStyle = PALETTE.accent;
        ctx.lineWidth = this.px(2);
        strokePolygon(ctx, slot.polygon);
      } else {
        ctx.fillStyle = slot.occupied ? "rgba(255, 107, 102, 0.1)" : "rgba(69, 230, 167, 0.1)";
        fillPolygon(ctx, slot.polygon);
        ctx.strokeStyle = slot.occupied ? "rgba(255, 107, 102, 0.6)" : "rgba(69, 230, 167, 0.7)";
        ctx.lineWidth = this.px(1.5);
        strokePolygon(ctx, slot.polygon);
      }
    }
  }

  #drawGoal(ctx, plan) {
    const fp = vehicleFootprint(plan.vehicle, plan.goal, 0);
    ctx.setLineDash([this.px(6), this.px(4)]);
    ctx.strokeStyle = PALETTE.accent;
    ctx.lineWidth = this.px(1.5);
    strokePolygon(ctx, fp);
    if (plan.preGoal && (plan.preGoal.x !== plan.goal.x || plan.preGoal.y !== plan.goal.y)) {
      ctx.strokeStyle = "rgba(69, 230, 167, 0.35)";
      strokePolygon(ctx, vehicleFootprint(plan.vehicle, plan.preGoal, 0));
    }
    ctx.setLineDash([]);
    // 目标后轴中心与航向
    drawPoseMarker(ctx, plan.goal, PALETTE.accent, this.px(1.6), 0.9);
  }

  #drawHeat(ctx, kind, plan, grid) {
    const key = `${kind}:${plan.id}`;
    let entry = this.heatCache.get(kind);
    if (!entry || entry.key !== key) {
      entry = { key, canvas: kind === "costmap" ? costmapImage(grid) : heuristicImage(grid) };
      this.heatCache.set(kind, entry);
    }
    ctx.save();
    ctx.imageSmoothingEnabled = true;
    ctx.globalAlpha = 0.85;
    ctx.drawImage(entry.canvas, 0, 0, grid.cols * grid.resolution, grid.rows * grid.resolution);
    ctx.restore();
  }

  #drawTree(plan, progress) {
    const tree = plan.search.tree || [];
    const key = `${this.viewKey}|${plan.id}`;
    const cache = this.treeCache;
    const ctx = cache.canvas.getContext("2d");
    if (cache.key !== key) {
      cache.key = key;
      cache.drawn = 0;
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      ctx.clearRect(0, 0, cache.canvas.width, cache.canvas.height);
    }
    const target = Math.floor(tree.length * Math.min(1, progress));
    if (target < cache.drawn) {
      cache.key = null;
      return this.#drawTree(plan, progress);
    }
    if (target === cache.drawn) return undefined;
    this.applyWorld(ctx);
    ctx.lineWidth = this.px(1);
    for (const dir of [1, -1]) {
      ctx.strokeStyle = dir > 0 ? PALETTE.treeForward : PALETTE.treeReverse;
      ctx.beginPath();
      for (let i = cache.drawn; i < target; i += 1) {
        const edge = tree[i];
        if (edge.dir !== dir) continue;
        const pts = edge.pts;
        ctx.moveTo(pts[0], pts[1]);
        for (let k = 2; k < pts.length; k += 2) ctx.lineTo(pts[k], pts[k + 1]);
      }
      ctx.stroke();
    }
    // 新扩展节点高亮（搜索前沿）
    if (progress < 1) {
      ctx.fillStyle = "rgba(200, 230, 255, 0.5)";
      for (let i = Math.max(cache.drawn, target - 40); i < target; i += 1) {
        const pts = tree[i].pts;
        const n = pts.length;
        ctx.fillRect(pts[n - 2] - this.px(1), pts[n - 1] - this.px(1), this.px(2), this.px(2));
      }
    }
    cache.drawn = target;
    return undefined;
  }

  #drawRsTrials(ctx, plan, progress) {
    const trials = plan.search.rsTrials || [];
    if (!trials.length) return;
    const treeLen = plan.search.tree?.length || 1;
    const current = progress >= 1 ? Infinity : treeLen * progress;
    const visible = trials.filter((t) => t.iteration <= current + 1);
    const recent = visible.slice(-(progress >= 1 ? 3 : 6));
    for (const trial of recent) {
      const pts = trial.points;
      ctx.strokeStyle = trial.success ? "rgba(69, 230, 167, 0.95)" : "rgba(255, 107, 102, 0.55)";
      ctx.lineWidth = this.px(trial.success ? 2 : 1.2);
      ctx.setLineDash(trial.success ? [] : [this.px(4), this.px(3)]);
      ctx.beginPath();
      ctx.moveTo(pts[0], pts[1]);
      for (let k = 2; k < pts.length; k += 2) ctx.lineTo(pts[k], pts[k + 1]);
      ctx.stroke();
      ctx.setLineDash([]);
      if (!trial.success && pts.length >= 2) {
        const x = pts[pts.length - 2];
        const y = pts[pts.length - 1];
        const r = this.px(4);
        ctx.strokeStyle = PALETTE.danger;
        ctx.lineWidth = this.px(1.6);
        ctx.beginPath();
        ctx.moveTo(x - r, y - r);
        ctx.lineTo(x + r, y + r);
        ctx.moveTo(x - r, y + r);
        ctx.lineTo(x + r, y - r);
        ctx.stroke();
      }
    }
  }

  #drawRawPath(ctx, plan) {
    const path = plan.search.path;
    ctx.strokeStyle = "rgba(255, 255, 255, 0.55)";
    ctx.lineWidth = this.px(1);
    ctx.setLineDash([this.px(3), this.px(3)]);
    ctx.beginPath();
    ctx.moveTo(path[0].x, path[0].y);
    for (const p of path) ctx.lineTo(p.x, p.y);
    ctx.stroke();
    ctx.setLineDash([]);
    // 解析扩展段起点
    const rsStart = path.findIndex((p) => p.source === "rs");
    if (rsStart > 0) {
      const p = path[rsStart - 1];
      ctx.fillStyle = "#ffffff";
      ctx.beginPath();
      ctx.arc(p.x, p.y, this.px(4), 0, Math.PI * 2);
      ctx.fill();
      this.text(ctx, `RS ${plan.search.analytic?.word || ""}`, p.x, p.y + this.px(14), { size: 10, color: "#ffffff", weight: 700 });
    }
  }

  #drawSweep(ctx, plan, vehicle) {
    ctx.lineWidth = this.px(1);
    let lastS = -Infinity;
    for (const p of plan.trajectory.points) {
      if (p.sGlobal - lastS < 0.9) continue;
      lastS = p.sGlobal;
      ctx.strokeStyle = p.dir > 0 ? "rgba(69, 230, 167, 0.16)" : "rgba(255, 162, 76, 0.2)";
      strokePolygon(ctx, vehicleFootprint(vehicle, p, 0));
    }
  }

  #drawTrajectory(ctx, plan, speedColor) {
    const pts = plan.trajectory.points;
    const vmax = Math.max(0.1, ...pts.map((p) => p.v));
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    // 外发光
    ctx.lineWidth = this.px(9);
    for (const seg of plan.segments) {
      ctx.strokeStyle = seg.dir > 0 ? "rgba(69, 230, 167, 0.1)" : "rgba(255, 162, 76, 0.12)";
      tracePoints(ctx, seg.points);
      ctx.stroke();
    }
    ctx.lineWidth = this.px(2.4);
    if (speedColor) {
      for (let i = 1; i < pts.length; i += 1) {
        if (pts[i].segment !== pts[i - 1].segment) continue;
        ctx.strokeStyle = speedRamp(pts[i].v / vmax);
        ctx.beginPath();
        ctx.moveTo(pts[i - 1].x, pts[i - 1].y);
        ctx.lineTo(pts[i].x, pts[i].y);
        ctx.stroke();
      }
    } else {
      for (const seg of plan.segments) {
        ctx.strokeStyle = seg.dir > 0 ? PALETTE.forward : PALETTE.reverse;
        tracePoints(ctx, seg.points);
        ctx.stroke();
      }
    }
    // 行驶方向箭头
    for (const seg of plan.segments) {
      const n = seg.points.length;
      for (let i = 8; i < n - 4; i += 16) {
        const p = seg.points[i];
        const heading = seg.dir > 0 ? p.theta : p.theta + Math.PI;
        ctx.save();
        ctx.translate(p.x, p.y);
        ctx.rotate(heading);
        ctx.fillStyle = seg.dir > 0 ? PALETTE.forward : PALETTE.reverse;
        const a = this.px(5);
        ctx.beginPath();
        ctx.moveTo(a, 0);
        ctx.lineTo(-a * 0.7, a * 0.7);
        ctx.lineTo(-a * 0.7, -a * 0.7);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
      }
    }
    // 换挡点
    plan.segments.slice(0, -1).forEach((seg, i) => {
      const p = seg.points[seg.points.length - 1];
      ctx.fillStyle = "#0a1412";
      ctx.strokeStyle = PALETTE.warning;
      ctx.lineWidth = this.px(2);
      ctx.beginPath();
      ctx.arc(p.x, p.y, this.px(7), 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      this.text(ctx, String(i + 1), p.x, p.y, { size: 9, color: PALETTE.warning, weight: 800 });
    });
  }

  #drawTrail(ctx, trail) {
    ctx.strokeStyle = "rgba(255, 255, 255, 0.85)";
    ctx.lineWidth = this.px(1.2);
    ctx.beginPath();
    ctx.moveTo(trail[0].x, trail[0].y);
    for (const p of trail) ctx.lineTo(p.x, p.y);
    ctx.stroke();
  }

  #drawSensors(ctx, readings) {
    for (const r of readings) {
      const range = r.distance ?? r.maxRange;
      const color = sensorColor(r.distance);
      ctx.fillStyle = color.fill;
      ctx.strokeStyle = color.stroke;
      ctx.lineWidth = this.px(1);
      ctx.beginPath();
      ctx.moveTo(r.origin.x, r.origin.y);
      ctx.arc(r.origin.x, r.origin.y, range, r.yawWorld - r.fov / 2, r.yawWorld + r.fov / 2);
      ctx.closePath();
      ctx.fill();
      if (r.distance !== null) {
        ctx.beginPath();
        ctx.arc(r.origin.x, r.origin.y, range, r.yawWorld - r.fov / 2, r.yawWorld + r.fov / 2);
        ctx.lineWidth = this.px(2);
        ctx.stroke();
      }
    }
  }

  #drawTracking(ctx, frame) {
    const { control, ego, vehicle } = frame;
    // 瞬时转向中心 (ICR) 与后轴转弯圆
    if (Math.abs(ego.delta) > 0.01) {
      const R = vehicle.wheelbase / Math.tan(ego.delta);
      if (Math.abs(R) < 40) {
        const cx = ego.x - R * Math.sin(ego.theta);
        const cy = ego.y + R * Math.cos(ego.theta);
        ctx.strokeStyle = "rgba(244, 200, 106, 0.45)";
        ctx.lineWidth = this.px(1);
        ctx.setLineDash([this.px(4), this.px(4)]);
        ctx.beginPath();
        ctx.arc(cx, cy, Math.abs(R), 0, Math.PI * 2);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.beginPath();
        ctx.moveTo(ego.x, ego.y);
        ctx.lineTo(cx, cy);
        ctx.stroke();
        ctx.fillStyle = PALETTE.warning;
        ctx.beginPath();
        ctx.arc(cx, cy, this.px(3), 0, Math.PI * 2);
        ctx.fill();
        this.text(ctx, `ICR  R=${Math.abs(R).toFixed(2)} m`, cx, cy - this.px(12), { size: 10, color: PALETTE.warning });
      }
    }
    const ref = control.projection?.ref;
    if (ref) {
      ctx.strokeStyle = "rgba(255,255,255,0.8)";
      ctx.lineWidth = this.px(1);
      ctx.beginPath();
      ctx.moveTo(ego.x, ego.y);
      ctx.lineTo(ref.x, ref.y);
      ctx.stroke();
      ctx.fillStyle = "#ffffff";
      ctx.beginPath();
      ctx.arc(ref.x, ref.y, this.px(3), 0, Math.PI * 2);
      ctx.fill();
    }
    if (control.target) {
      ctx.strokeStyle = "rgba(90, 160, 255, 0.9)";
      ctx.lineWidth = this.px(1.4);
      ctx.beginPath();
      ctx.moveTo(ego.x, ego.y);
      ctx.lineTo(control.target.x, control.target.y);
      ctx.stroke();
      ctx.fillStyle = "#5aa0ff";
      ctx.beginPath();
      ctx.arc(control.target.x, control.target.y, this.px(5), 0, Math.PI * 2);
      ctx.fill();
      this.text(ctx, `前视点 Ld=${control.lookahead.toFixed(2)} m`, control.target.x, control.target.y + this.px(14), { size: 10, color: "#9cc4ff" });
    }
  }

  drawEgo(ctx, vehicle, ego, { gear, braking, invalid, highlight } = {}) {
    const d = centerOffset(vehicle);
    const cx = ego.x + d * Math.cos(ego.theta);
    const cy = ego.y + d * Math.sin(ego.theta);
    drawCarBody(ctx, cx, cy, ego.theta, vehicle.length, vehicle.width, invalid ? "#8a3b38" : PALETTE.ego, {
      ego: true,
      reverse: gear === "R",
      braking,
      glow: !invalid,
    });
    // 车轮（前轮按阿克曼几何转向）
    const { left, right } = ackermannAngles(vehicle, ego.delta || 0);
    const wheels = [
      [0, vehicle.track / 2, 0],
      [0, -vehicle.track / 2, 0],
      [vehicle.wheelbase, vehicle.track / 2, left],
      [vehicle.wheelbase, -vehicle.track / 2, right],
    ];
    ctx.save();
    ctx.translate(ego.x, ego.y);
    ctx.rotate(ego.theta);
    for (const [wx, wy, a] of wheels) {
      ctx.save();
      ctx.translate(wx, wy);
      ctx.rotate(a);
      ctx.fillStyle = "#050807";
      ctx.fillRect(-vehicle.wheelRadius, -0.12, vehicle.wheelRadius * 2, 0.24);
      ctx.strokeStyle = a !== 0 ? PALETTE.warning : "rgba(255,255,255,0.35)";
      ctx.lineWidth = this.px(1);
      ctx.strokeRect(-vehicle.wheelRadius, -0.12, vehicle.wheelRadius * 2, 0.24);
      ctx.restore();
    }
    // 后轴中心（运动学参考点）
    ctx.fillStyle = "#ffffff";
    ctx.beginPath();
    ctx.arc(0, 0, this.px(2.6), 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "rgba(255,255,255,0.4)";
    ctx.lineWidth = this.px(1);
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(vehicle.wheelbase, 0);
    ctx.stroke();
    ctx.restore();
    if (invalid || highlight) {
      ctx.strokeStyle = invalid ? PALETTE.danger : "rgba(255,255,255,0.8)";
      ctx.lineWidth = this.px(1.5);
      ctx.setLineDash([this.px(4), this.px(3)]);
      strokePolygon(ctx, vehicleFootprint(vehicle, ego, 0.15));
      ctx.setLineDash([]);
    }
  }

  #drawScaleBar(ctx) {
    const meters = this.scale > 60 ? 1 : this.scale > 25 ? 2 : 5;
    const len = meters * this.scale;
    const x = 16;
    const y = this.height - 16;
    ctx.strokeStyle = "rgba(214, 232, 226, 0.75)";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x, y - 5);
    ctx.lineTo(x, y);
    ctx.lineTo(x + len, y);
    ctx.lineTo(x + len, y - 5);
    ctx.stroke();
    ctx.fillStyle = "rgba(214, 232, 226, 0.85)";
    ctx.font = "600 11px Inter, 'Microsoft YaHei', sans-serif";
    ctx.textAlign = "left";
    ctx.textBaseline = "bottom";
    ctx.fillText(`${meters} m`, x + len + 6, y + 1);
  }

  #drawSearchBadge(ctx, plan, progress) {
    const n = Math.floor((plan.search.tree?.length || 0) * progress);
    const label = `Hybrid A* 搜索回放 · 已扩展 ${n} 个节点`;
    ctx.font = "700 12px Inter, 'Microsoft YaHei', sans-serif";
    const w = ctx.measureText(label).width + 24;
    ctx.fillStyle = "rgba(7, 16, 15, 0.82)";
    roundRect(ctx, this.width / 2 - w / 2, 14, w, 30, 15);
    ctx.fill();
    ctx.strokeStyle = "rgba(90, 160, 255, 0.6)";
    ctx.stroke();
    ctx.fillStyle = "#cfe2ff";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(label, this.width / 2, 29);
  }

  /** 在世界坐标位置绘制屏幕尺寸的文字 */
  text(ctx, str, x, y, { size = 10, color = PALETTE.label, align = "center", weight = 600 } = {}) {
    const p = this.worldToScreen(x, y);
    ctx.save();
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.font = `${weight} ${size}px Inter, "Microsoft YaHei", sans-serif`;
    ctx.fillStyle = color;
    ctx.textAlign = align;
    ctx.textBaseline = "middle";
    ctx.fillText(str, p.x, p.y);
    ctx.restore();
  }
}

// —— 绘图工具 ——

function tracePolygon(ctx, polygon) {
  const pts = polygon.points;
  ctx.beginPath();
  ctx.moveTo(pts[0].x, pts[0].y);
  for (let i = 1; i < pts.length; i += 1) ctx.lineTo(pts[i].x, pts[i].y);
  ctx.closePath();
}

function fillPolygon(ctx, polygon) {
  tracePolygon(ctx, polygon);
  ctx.fill();
}

function strokePolygon(ctx, polygon) {
  tracePolygon(ctx, polygon);
  ctx.stroke();
}

function tracePoints(ctx, points) {
  ctx.beginPath();
  ctx.moveTo(points[0].x, points[0].y);
  for (let i = 1; i < points.length; i += 1) ctx.lineTo(points[i].x, points[i].y);
}

function roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

function drawPoseMarker(ctx, pose, color, lineWidth, length) {
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.beginPath();
  ctx.moveTo(pose.x, pose.y);
  ctx.lineTo(pose.x + length * Math.cos(pose.theta), pose.y + length * Math.sin(pose.theta));
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(pose.x, pose.y, lineWidth * 2.2, 0, Math.PI * 2);
  ctx.fill();
}

/** 俯视车身：圆角车体、风挡、车顶、灯组。坐标为车身几何中心。 */
export function drawCarBody(ctx, cx, cy, theta, length, width, color, opts = {}) {
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(theta);
  const hl = length / 2;
  const hw = width / 2;
  if (opts.glow) {
    ctx.shadowColor = "rgba(69, 230, 167, 0.55)";
    ctx.shadowBlur = 18;
  }
  roundRect(ctx, -hl, -hw, length, width, 0.42);
  ctx.fillStyle = color;
  ctx.fill();
  ctx.shadowBlur = 0;
  ctx.strokeStyle = opts.ego ? "rgba(190, 255, 225, 0.9)" : "rgba(255,255,255,0.12)";
  ctx.lineWidth = 0.04;
  ctx.stroke();
  // 风挡 + 车顶
  ctx.fillStyle = opts.ego ? "rgba(6, 40, 30, 0.85)" : "rgba(10, 18, 17, 0.6)";
  ctx.beginPath();
  ctx.moveTo(hl * 0.42, -hw * 0.78);
  ctx.lineTo(hl * 0.18, -hw * 0.86);
  ctx.lineTo(-hl * 0.5, -hw * 0.86);
  ctx.lineTo(-hl * 0.62, -hw * 0.74);
  ctx.lineTo(-hl * 0.62, hw * 0.74);
  ctx.lineTo(-hl * 0.5, hw * 0.86);
  ctx.lineTo(hl * 0.18, hw * 0.86);
  ctx.lineTo(hl * 0.42, hw * 0.78);
  ctx.closePath();
  ctx.fill();
  ctx.fillStyle = opts.ego ? "rgba(70, 230, 167, 0.25)" : "rgba(255,255,255,0.05)";
  roundRect(ctx, -hl * 0.4, -hw * 0.66, hl * 0.52, hw * 1.32, 0.15);
  ctx.fill();
  // 后视镜
  ctx.fillStyle = color;
  ctx.fillRect(hl * 0.22, hw - 0.02, 0.18, 0.16);
  ctx.fillRect(hl * 0.22, -hw - 0.14, 0.18, 0.16);
  // 灯组
  ctx.fillStyle = opts.parked ? "rgba(255, 245, 220, 0.25)" : "rgba(235, 255, 248, 0.95)";
  ctx.fillRect(hl - 0.1, hw - 0.42, 0.08, 0.3);
  ctx.fillRect(hl - 0.1, -hw + 0.12, 0.08, 0.3);
  const tail = opts.braking ? "#ff3b30" : opts.parked ? "rgba(255, 80, 70, 0.35)" : "rgba(255, 70, 60, 0.75)";
  ctx.fillStyle = tail;
  ctx.fillRect(-hl + 0.02, hw - 0.4, 0.08, 0.28);
  ctx.fillRect(-hl + 0.02, -hw + 0.12, 0.08, 0.28);
  if (opts.reverse) {
    ctx.fillStyle = "#ffffff";
    ctx.fillRect(-hl + 0.02, hw - 0.62, 0.08, 0.14);
    ctx.fillRect(-hl + 0.02, -hw + 0.48, 0.08, 0.14);
  }
  ctx.restore();
}

function sensorColor(distance) {
  if (distance === null) return { fill: "rgba(69, 230, 167, 0.035)", stroke: "rgba(69,230,167,0.2)" };
  if (distance < 0.3) return { fill: "rgba(255, 70, 60, 0.28)", stroke: "rgba(255, 90, 80, 0.95)" };
  if (distance < 0.6) return { fill: "rgba(255, 140, 60, 0.22)", stroke: "rgba(255, 160, 80, 0.9)" };
  if (distance < 1.2) return { fill: "rgba(244, 200, 106, 0.16)", stroke: "rgba(244, 200, 106, 0.85)" };
  return { fill: "rgba(69, 230, 167, 0.08)", stroke: "rgba(69, 230, 167, 0.6)" };
}

/** 速度着色：单色相（青绿）由暗到亮 */
function speedRamp(t) {
  const k = Math.max(0, Math.min(1, t));
  const l = 28 + k * 50;
  return `hsl(158, 75%, ${l}%)`;
}

/** 代价地图：离障碍物越近越“热” */
function costmapImage(grid) {
  const { cols, rows, dist } = grid;
  const canvas = document.createElement("canvas");
  canvas.width = cols;
  canvas.height = rows;
  const ctx = canvas.getContext("2d");
  const img = ctx.createImageData(cols, rows);
  for (let r = 0; r < rows; r += 1) {
    for (let c = 0; c < cols; c += 1) {
      const d = dist[r * cols + c];
      const risk = Math.exp(-d / 0.9);
      const i = (r * cols + c) * 4;
      img.data[i] = 255;
      img.data[i + 1] = 90 + 110 * (1 - risk);
      img.data[i + 2] = 60;
      img.data[i + 3] = d <= 0 ? 150 : 170 * risk;
    }
  }
  ctx.putImageData(img, 0, 0);
  return canvas;
}

/** 启发函数图：到目标的 2D 最短距离，越近越亮，每 5 m 一条等值线 */
function heuristicImage(h) {
  const { cols, rows, cost } = h;
  let max = 0;
  for (const v of cost) if (Number.isFinite(v) && v > max) max = v;
  const canvas = document.createElement("canvas");
  canvas.width = cols;
  canvas.height = rows;
  const ctx = canvas.getContext("2d");
  const img = ctx.createImageData(cols, rows);
  for (let i = 0; i < cost.length; i += 1) {
    const v = cost[i];
    const o = i * 4;
    if (!Number.isFinite(v)) {
      img.data[o] = 0;
      img.data[o + 1] = 0;
      img.data[o + 2] = 0;
      img.data[o + 3] = 90;
      continue;
    }
    const t = 1 - v / max;
    const band = v % 5 < 0.25 ? 1 : 0;
    img.data[o] = 40 + 50 * t + band * 80;
    img.data[o + 1] = 90 + 120 * t + band * 60;
    img.data[o + 2] = 200 + 55 * t;
    img.data[o + 3] = 40 + 120 * t * t + band * 70;
  }
  ctx.putImageData(img, 0, 0);
  return canvas;
}
