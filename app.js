const canvas = document.getElementById("parkingCanvas");
const ctx = canvas.getContext("2d");

const UI = {
  clock: document.getElementById("clock"),
  selectedSpot: document.getElementById("selectedSpot"),
  spotDistance: document.getElementById("spotDistance"),
  metricDistance: document.getElementById("metricDistance"),
  metricTime: document.getElementById("metricTime"),
  metricNodes: document.getElementById("metricNodes"),
  stagePill: document.getElementById("stagePill"),
  planButton: document.getElementById("planButton"),
  parkButton: document.getElementById("parkButton"),
  resetButton: document.getElementById("resetButton"),
  speedRange: document.getElementById("speedRange"),
  speedOutput: document.getElementById("speedOutput"),
  canvasHint: document.getElementById("canvasHint"),
  availableCount: document.getElementById("availableCount"),
  toast: document.getElementById("toast"),
  steps: [...document.querySelectorAll(".step")],
};

const COLORS = {
  asphalt: "#101a18",
  asphaltLight: "#14211f",
  line: "rgba(184, 211, 203, 0.36)",
  lineFaint: "rgba(184, 211, 203, 0.13)",
  text: "rgba(217, 232, 227, 0.48)",
  accent: "#45e6a7",
  accentGlow: "rgba(69, 230, 167, 0.22)",
  occupied: "#34433f",
  occupiedEdge: "#60716c",
  ego: "#43e2a5",
  obstacle: "#1c2926",
};

const MAP = {
  width: 1200,
  height: 680,
  spotWidth: 88,
  spotDepth: 142,
  startX: 255,
  gap: 14,
  topY: 76,
  bottomY: 462,
  laneTop: 238,
  laneBottom: 442,
  scaleMeters: 0.055,
};

const occupiedTop = new Set([0, 1, 3, 5, 7]);
const occupiedBottom = new Set([0, 2, 3, 6, 8]);
const parkingSpots = [];

for (let row = 0; row < 2; row += 1) {
  for (let index = 0; index < 9; index += 1) {
    const occupied = row === 0 ? occupiedTop.has(index) : occupiedBottom.has(index);
    parkingSpots.push({
      id: `${row === 0 ? "A" : "B"}-${String(index + 1).padStart(2, "0")}`,
      row,
      index,
      orientation: "perpendicular",
      side: row === 0 ? "north" : "south",
      x: MAP.startX + index * (MAP.spotWidth + MAP.gap),
      y: row === 0 ? MAP.topY : MAP.bottomY,
      width: MAP.spotWidth,
      height: MAP.spotDepth,
      occupied,
    });
  }
}

parkingSpots.push(
  {
    id: "C-01",
    row: 2,
    index: 0,
    orientation: "parallel",
    side: "north",
    x: 930,
    y: 244,
    width: 150,
    height: 72,
    occupied: false,
    targetAngle: Math.PI,
  },
  {
    id: "C-02",
    row: 3,
    index: 1,
    orientation: "parallel",
    side: "south",
    x: 930,
    y: 368,
    width: 150,
    height: 72,
    occupied: false,
    targetAngle: Math.PI,
  },
);

const initialCar = {
  x: 110,
  y: 342,
  angle: 0,
  width: 42,
  length: 82,
};

const state = {
  selected: null,
  route: [],
  rawRoute: [],
  car: { ...initialCar },
  phase: "select",
  animationId: null,
  animationStartedAt: 0,
  animationDuration: 0,
  routeLengths: [],
  totalRouteLength: 0,
  hoverSpot: null,
};

function updateClock() {
  UI.clock.textContent = new Intl.DateTimeFormat("zh-CN", {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: false,
  }).format(new Date());
}

function roundedRect(context, x, y, width, height, radius) {
  const r = Math.min(radius, width / 2, height / 2);
  context.beginPath();
  context.moveTo(x + r, y);
  context.arcTo(x + width, y, x + width, y + height, r);
  context.arcTo(x + width, y + height, x, y + height, r);
  context.arcTo(x, y + height, x, y, r);
  context.arcTo(x, y, x + width, y, r);
  context.closePath();
}

function drawParkingSurface() {
  const gradient = ctx.createLinearGradient(0, 0, 0, MAP.height);
  gradient.addColorStop(0, COLORS.asphaltLight);
  gradient.addColorStop(0.48, COLORS.asphalt);
  gradient.addColorStop(1, "#0d1715");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, MAP.width, MAP.height);

  ctx.save();
  ctx.globalAlpha = 0.23;
  for (let x = 20; x < MAP.width; x += 36) {
    for (let y = 16; y < MAP.height; y += 34) {
      const alpha = ((x * 17 + y * 13) % 11) / 90;
      ctx.fillStyle = `rgba(184, 211, 203, ${alpha})`;
      ctx.fillRect(x, y, 1.2, 1.2);
    }
  }
  ctx.restore();

  ctx.strokeStyle = COLORS.lineFaint;
  ctx.lineWidth = 1;
  ctx.setLineDash([14, 14]);
  ctx.beginPath();
  ctx.moveTo(48, 340);
  ctx.lineTo(1152, 340);
  ctx.stroke();
  ctx.setLineDash([]);

  drawDirectionArrows();
  drawEntrance();
  drawSafetyIslands();
}

function drawDirectionArrows() {
  ctx.save();
  ctx.fillStyle = "rgba(188, 211, 204, 0.16)";
  for (const x of [410, 710, 1010]) {
    ctx.beginPath();
    ctx.moveTo(x + 20, 329);
    ctx.lineTo(x + 37, 340);
    ctx.lineTo(x + 20, 351);
    ctx.lineTo(x + 20, 345);
    ctx.lineTo(x - 13, 345);
    ctx.lineTo(x - 13, 335);
    ctx.lineTo(x + 20, 335);
    ctx.closePath();
    ctx.fill();
  }
  ctx.restore();
}

function drawEntrance() {
  ctx.save();
  ctx.strokeStyle = "rgba(69, 230, 167, 0.45)";
  ctx.lineWidth = 1;
  ctx.setLineDash([5, 5]);
  ctx.beginPath();
  ctx.moveTo(37, 286);
  ctx.lineTo(37, 398);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.translate(53, 342);
  ctx.rotate(-Math.PI / 2);
  ctx.fillStyle = "rgba(69, 230, 167, 0.65)";
  ctx.font = "600 10px Inter, sans-serif";
  ctx.textAlign = "center";
  ctx.fillText("ENTRANCE", 0, 0);
  ctx.restore();
}

function drawSafetyIslands() {
  for (const x of [196, 1164]) {
    ctx.save();
    for (const y of [224, 426]) {
      roundedRect(ctx, x - 17, y, 34, 30, 8);
      ctx.fillStyle = COLORS.obstacle;
      ctx.fill();
      ctx.strokeStyle = "rgba(131, 158, 150, 0.18)";
      ctx.stroke();
      ctx.strokeStyle = "rgba(244, 200, 106, 0.16)";
      ctx.beginPath();
      ctx.moveTo(x - 10, y + 23);
      ctx.lineTo(x + 10, y + 7);
      ctx.stroke();
    }
    ctx.restore();
  }

  ctx.save();
  roundedRect(ctx, 624, 309, 94, 64, 9);
  ctx.fillStyle = "#172522";
  ctx.fill();
  ctx.strokeStyle = "rgba(244, 200, 106, 0.38)";
  ctx.setLineDash([7, 5]);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = "rgba(244, 200, 106, 0.58)";
  ctx.font = "700 8px Inter, sans-serif";
  ctx.textAlign = "center";
  ctx.fillText("TEMP ZONE", 671, 345);
  for (const x of [635, 707]) {
    ctx.fillStyle = "#f4c86a";
    ctx.beginPath();
    ctx.arc(x, 320, 3, 0, Math.PI * 2);
    ctx.arc(x, 362, 3, 0, Math.PI * 2);
    ctx.fill();
  }
  ctx.restore();
}

function drawParkingSpots(timestamp) {
  for (const spot of parkingSpots) {
    const selected = state.selected?.id === spot.id;
    const hovered = state.hoverSpot?.id === spot.id && !spot.occupied;
    const pulse = 0.5 + Math.sin(timestamp / 450) * 0.12;

    ctx.save();
    ctx.strokeStyle = selected
      ? COLORS.accent
      : hovered
        ? "rgba(69, 230, 167, 0.8)"
        : COLORS.line;
    ctx.lineWidth = selected ? 2 : 1;
    if (!spot.occupied) {
      ctx.fillStyle = selected
        ? `rgba(69, 230, 167, ${0.08 + pulse * 0.08})`
        : hovered
          ? "rgba(69, 230, 167, 0.08)"
          : "rgba(69, 230, 167, 0.018)";
      ctx.fillRect(spot.x, spot.y, spot.width, spot.height);
    }
    ctx.strokeRect(spot.x, spot.y, spot.width, spot.height);

    const isParallel = spot.orientation === "parallel";
    const labelY = isParallel
      ? spot.y + 15
      : spot.row === 0
        ? spot.y + 17
        : spot.y + spot.height - 10;
    ctx.fillStyle = selected
      ? COLORS.accent
      : spot.occupied
        ? "rgba(210, 226, 221, 0.26)"
        : "rgba(69, 230, 167, 0.66)";
    ctx.font = "600 9px Inter, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText(spot.id, spot.x + spot.width / 2, labelY);

    if (!spot.occupied) {
      const centerY = isParallel
        ? spot.y + spot.height / 2 + 9
        : spot.row === 0
          ? spot.y + spot.height / 2 + 10
          : spot.y + spot.height / 2 - 10;
      ctx.font = "500 9px Inter, sans-serif";
      ctx.fillStyle = selected ? COLORS.accent : "rgba(69, 230, 167, 0.43)";
      ctx.fillText(
        selected ? "PARALLEL TARGET" : isParallel ? "PARALLEL" : "AVAILABLE",
        spot.x + spot.width / 2,
        centerY,
      );

      ctx.strokeStyle = selected ? COLORS.accent : "rgba(69, 230, 167, 0.24)";
      ctx.beginPath();
      ctx.arc(spot.x + spot.width / 2, centerY - 20, selected ? 8 : 5, 0, Math.PI * 2);
      ctx.stroke();
      if (selected) {
        ctx.fillStyle = COLORS.accent;
        ctx.beginPath();
        ctx.arc(spot.x + spot.width / 2, centerY - 20, 2, 0, Math.PI * 2);
        ctx.fill();
      }
    }
    ctx.restore();

    if (spot.occupied) {
      drawCar(
        spot.x + spot.width / 2,
        spot.y + spot.height / 2,
        spot.orientation === "parallel"
          ? spot.targetAngle
          : spot.row === 0
            ? -Math.PI / 2
            : Math.PI / 2,
        getParkedCarColor(spot.index, spot.row),
        false,
      );
    }
  }
}

function getParkedCarColor(index, row) {
  const palette = ["#4b5e59", "#263f42", "#5a5850", "#394d55", "#525d61"];
  return palette[(index + row * 2) % palette.length];
}

function drawCar(x, y, angle, color, isEgo = false) {
  const carWidth = isEgo ? state.car.width : 42;
  const carLength = isEgo ? state.car.length : 78;

  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(angle);

  if (isEgo) {
    ctx.shadowColor = "rgba(69, 230, 167, 0.45)";
    ctx.shadowBlur = 18;
  }

  const bodyGradient = ctx.createLinearGradient(
    -carLength / 2,
    0,
    carLength / 2,
    0,
  );
  bodyGradient.addColorStop(0, shadeColor(color, -18));
  bodyGradient.addColorStop(0.5, color);
  bodyGradient.addColorStop(1, shadeColor(color, -8));

  roundedRect(ctx, -carLength / 2, -carWidth / 2, carLength, carWidth, 12);
  ctx.fillStyle = bodyGradient;
  ctx.fill();
  ctx.shadowBlur = 0;
  ctx.strokeStyle = isEgo ? "rgba(144, 255, 210, 0.72)" : "rgba(210, 225, 220, 0.16)";
  ctx.lineWidth = 1;
  ctx.stroke();

  roundedRect(
    ctx,
    -carLength * 0.18,
    -carWidth * 0.39,
    carLength * 0.39,
    carWidth * 0.78,
    7,
  );
  ctx.fillStyle = isEgo ? "#123a31" : "#172725";
  ctx.fill();

  ctx.fillStyle = isEgo ? "rgba(150, 255, 216, 0.26)" : "rgba(165, 190, 183, 0.1)";
  ctx.fillRect(-carLength * 0.13, -carWidth * 0.34, 2, carWidth * 0.68);
  ctx.fillRect(carLength * 0.15, -carWidth * 0.34, 2, carWidth * 0.68);

  ctx.fillStyle = "#d9fff0";
  ctx.globalAlpha = isEgo ? 0.9 : 0.38;
  ctx.fillRect(carLength / 2 - 7, -carWidth / 2 + 5, 4, 7);
  ctx.fillRect(carLength / 2 - 7, carWidth / 2 - 12, 4, 7);
  ctx.fillStyle = "#ff6d67";
  ctx.fillRect(-carLength / 2 + 3, -carWidth / 2 + 5, 4, 7);
  ctx.fillRect(-carLength / 2 + 3, carWidth / 2 - 12, 4, 7);
  ctx.globalAlpha = 1;

  if (isEgo) {
    ctx.fillStyle = "#062219";
    ctx.font = "800 8px Inter, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText("YOU", 3, 0);
  }
  ctx.restore();
}

function shadeColor(hex, amount) {
  const value = Number.parseInt(hex.slice(1), 16);
  const red = Math.max(0, Math.min(255, (value >> 16) + amount));
  const green = Math.max(0, Math.min(255, ((value >> 8) & 0xff) + amount));
  const blue = Math.max(0, Math.min(255, (value & 0xff) + amount));
  return `rgb(${red}, ${green}, ${blue})`;
}

function drawRoute(timestamp) {
  if (state.route.length < 2) return;

  ctx.save();
  ctx.lineCap = "round";
  ctx.lineJoin = "round";

  ctx.strokeStyle = "rgba(69, 230, 167, 0.09)";
  ctx.lineWidth = 13;
  traceRoute();
  ctx.stroke();

  ctx.strokeStyle = "rgba(69, 230, 167, 0.9)";
  ctx.lineWidth = 2;
  ctx.setLineDash([9, 9]);
  ctx.lineDashOffset = -(timestamp / 35);
  traceRoute();
  ctx.stroke();
  ctx.setLineDash([]);

  for (let i = 1; i < state.route.length - 1; i += 9) {
    const point = state.route[i];
    ctx.fillStyle = "rgba(69, 230, 167, 0.8)";
    ctx.beginPath();
    ctx.arc(point.x, point.y, 2.5, 0, Math.PI * 2);
    ctx.fill();
  }
  ctx.restore();
}

function traceRoute() {
  ctx.beginPath();
  ctx.moveTo(state.route[0].x, state.route[0].y);
  for (let i = 1; i < state.route.length; i += 1) {
    ctx.lineTo(state.route[i].x, state.route[i].y);
  }
}

function drawSensorField(timestamp) {
  if (state.phase !== "parking") return;
  const radius = 64 + Math.sin(timestamp / 260) * 4;
  ctx.save();
  ctx.translate(state.car.x, state.car.y);
  ctx.strokeStyle = "rgba(69, 230, 167, 0.15)";
  ctx.lineWidth = 1;
  ctx.setLineDash([4, 6]);
  ctx.beginPath();
  ctx.arc(0, 0, radius, 0, Math.PI * 2);
  ctx.stroke();
  ctx.restore();
}

function render(timestamp = 0) {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawParkingSurface();
  drawParkingSpots(timestamp);
  drawRoute(timestamp);
  drawSensorField(timestamp);
  drawCar(state.car.x, state.car.y, state.car.angle, COLORS.ego, true);
}

function canvasPoint(event) {
  const rect = canvas.getBoundingClientRect();
  return {
    x: ((event.clientX - rect.left) / rect.width) * canvas.width,
    y: ((event.clientY - rect.top) / rect.height) * canvas.height,
  };
}

function findSpotAt(point) {
  return parkingSpots.find(
    (spot) =>
      point.x >= spot.x &&
      point.x <= spot.x + spot.width &&
      point.y >= spot.y &&
      point.y <= spot.y + spot.height,
  );
}

canvas.addEventListener("mousemove", (event) => {
  if (state.phase === "parking" || state.phase === "complete") return;
  const spot = findSpotAt(canvasPoint(event));
  state.hoverSpot = spot && !spot.occupied ? spot : null;
  canvas.style.cursor = state.hoverSpot ? "pointer" : "crosshair";
  render(performance.now());
});

canvas.addEventListener("mouseleave", () => {
  state.hoverSpot = null;
  render(performance.now());
});

canvas.addEventListener("click", (event) => {
  if (state.phase === "parking" || state.phase === "complete") return;
  const spot = findSpotAt(canvasPoint(event));
  if (!spot) return;
  if (spot.occupied) {
    showToast(`${spot.id} 车位已被占用，请选择绿色空闲车位`);
    return;
  }
  selectSpot(spot);
});

function selectSpot(spot) {
  state.selected = spot;
  state.route = [];
  state.rawRoute = [];
  state.phase = "selected";
  state.car = { ...initialCar };

  const directDistance = Math.hypot(
    spot.x + spot.width / 2 - state.car.x,
    spot.y + spot.height / 2 - state.car.y,
  );
  const meters = directDistance * MAP.scaleMeters;

  UI.selectedSpot.textContent = `${spot.id} · ${getSpotLocation(spot)}`;
  UI.spotDistance.textContent = `${meters.toFixed(1)} m`;
  UI.metricDistance.textContent = "--";
  UI.metricTime.textContent = "--";
  UI.metricNodes.textContent = "--";
  UI.stagePill.textContent = "车位已锁定";
  UI.planButton.disabled = false;
  UI.parkButton.disabled = true;
  UI.canvasHint.textContent = "目标已锁定，点击“规划安全路线”";
  setStep(1);
  render(performance.now());
  showToast(`已选择 ${spot.id} 车位`);
}

function getSpotLocation(spot) {
  if (spot.orientation === "parallel") {
    return spot.side === "north" ? "北侧方" : "南侧方";
  }
  return spot.side === "north" ? "北侧" : "南侧";
}

function setStep(activeStep, completeThrough = activeStep - 1) {
  UI.steps.forEach((step, index) => {
    const number = index + 1;
    step.classList.toggle("active", number === activeStep);
    step.classList.toggle("complete", number <= completeThrough);
    const badge = step.querySelector(":scope > span");
    badge.textContent = number <= completeThrough ? "✓" : `0${number}`;
  });
}

function createGrid() {
  const cell = 14;
  const cols = Math.ceil(MAP.width / cell);
  const rows = Math.ceil(MAP.height / cell);
  const blocked = new Uint8Array(cols * rows);

  const markRect = (x, y, width, height, margin = 0) => {
    const startCol = Math.max(0, Math.floor((x - margin) / cell));
    const endCol = Math.min(cols - 1, Math.ceil((x + width + margin) / cell));
    const startRow = Math.max(0, Math.floor((y - margin) / cell));
    const endRow = Math.min(rows - 1, Math.ceil((y + height + margin) / cell));
    for (let row = startRow; row <= endRow; row += 1) {
      for (let col = startCol; col <= endCol; col += 1) {
        blocked[row * cols + col] = 1;
      }
    }
  };

  markRect(0, 0, MAP.width, 50);
  markRect(0, 630, MAP.width, 50);
  markRect(624, 309, 94, 64, 25);

  parkingSpots.forEach((spot) => {
    if (spot.occupied) {
      const carX = spot.x + spot.width / 2 - 21;
      const carY = spot.y + spot.height / 2 - 39;
      markRect(carX, carY, 42, 78, 20);
    }
  });

  return { cell, cols, rows, blocked };
}

function findPath(start, goal) {
  const grid = createGrid();
  const { cell, cols, rows, blocked } = grid;
  const toGrid = (point) => ({
    col: Math.max(0, Math.min(cols - 1, Math.floor(point.x / cell))),
    row: Math.max(0, Math.min(rows - 1, Math.floor(point.y / cell))),
  });
  const startNode = toGrid(start);
  const goalNode = toGrid(goal);
  blocked[startNode.row * cols + startNode.col] = 0;
  blocked[goalNode.row * cols + goalNode.col] = 0;

  const key = (col, row) => row * cols + col;
  const heuristic = (col, row) =>
    Math.hypot(goalNode.col - col, goalNode.row - row);
  const open = [
    {
      col: startNode.col,
      row: startNode.row,
      g: 0,
      f: heuristic(startNode.col, startNode.row),
      direction: -1,
    },
  ];
  const cameFrom = new Int32Array(cols * rows).fill(-1);
  const gScore = new Float64Array(cols * rows).fill(Number.POSITIVE_INFINITY);
  const closed = new Uint8Array(cols * rows);
  gScore[key(startNode.col, startNode.row)] = 0;

  const directions = [
    [1, 0],
    [1, 1],
    [0, 1],
    [-1, 1],
    [-1, 0],
    [-1, -1],
    [0, -1],
    [1, -1],
  ];

  while (open.length) {
    let bestIndex = 0;
    for (let i = 1; i < open.length; i += 1) {
      if (open[i].f < open[bestIndex].f) bestIndex = i;
    }
    const current = open.splice(bestIndex, 1)[0];
    const currentKey = key(current.col, current.row);
    if (closed[currentKey]) continue;
    closed[currentKey] = 1;

    if (current.col === goalNode.col && current.row === goalNode.row) {
      const path = [];
      let cursor = currentKey;
      while (cursor !== -1) {
        const row = Math.floor(cursor / cols);
        const col = cursor % cols;
        path.push({ x: col * cell + cell / 2, y: row * cell + cell / 2 });
        cursor = cameFrom[cursor];
      }
      path.reverse();
      path[0] = { ...start };
      path[path.length - 1] = { ...goal };
      return { path, grid };
    }

    directions.forEach(([dx, dy], directionIndex) => {
      const nextCol = current.col + dx;
      const nextRow = current.row + dy;
      if (nextCol < 0 || nextCol >= cols || nextRow < 0 || nextRow >= rows) return;
      const nextKey = key(nextCol, nextRow);
      if (blocked[nextKey] || closed[nextKey]) return;

      if (dx !== 0 && dy !== 0) {
        if (
          blocked[key(current.col + dx, current.row)] ||
          blocked[key(current.col, current.row + dy)]
        ) {
          return;
        }
      }

      const movementCost = dx === 0 || dy === 0 ? 1 : Math.SQRT2;
      const turnPenalty =
        current.direction >= 0 && current.direction !== directionIndex ? 0.16 : 0;
      const tentative = current.g + movementCost + turnPenalty;
      if (tentative >= gScore[nextKey]) return;

      cameFrom[nextKey] = currentKey;
      gScore[nextKey] = tentative;
      open.push({
        col: nextCol,
        row: nextRow,
        g: tentative,
        f: tentative + heuristic(nextCol, nextRow),
        direction: directionIndex,
      });
    });
  }
  return null;
}

function isLineClear(a, b, grid) {
  const distance = Math.hypot(b.x - a.x, b.y - a.y);
  const steps = Math.ceil(distance / (grid.cell * 0.45));
  for (let i = 0; i <= steps; i += 1) {
    const t = i / steps;
    const x = a.x + (b.x - a.x) * t;
    const y = a.y + (b.y - a.y) * t;
    const col = Math.floor(x / grid.cell);
    const row = Math.floor(y / grid.cell);
    if (
      col < 0 ||
      col >= grid.cols ||
      row < 0 ||
      row >= grid.rows ||
      grid.blocked[row * grid.cols + col]
    ) {
      return false;
    }
  }
  return true;
}

function simplifyPath(path, grid) {
  if (path.length < 3) return path;
  const simplified = [path[0]];
  let anchor = 0;
  while (anchor < path.length - 1) {
    let next = path.length - 1;
    while (next > anchor + 1 && !isLineClear(path[anchor], path[next], grid)) {
      next -= 1;
    }
    simplified.push(path[next]);
    anchor = next;
  }
  return simplified;
}

function catmullRom(points, samplesPerSegment = 16) {
  if (points.length < 2) return points;
  const padded = [points[0], ...points, points[points.length - 1]];
  const output = [];

  for (let i = 0; i < padded.length - 3; i += 1) {
    const p0 = padded[i];
    const p1 = padded[i + 1];
    const p2 = padded[i + 2];
    const p3 = padded[i + 3];
    for (let step = 0; step < samplesPerSegment; step += 1) {
      const t = step / samplesPerSegment;
      const t2 = t * t;
      const t3 = t2 * t;
      output.push({
        x:
          0.5 *
          (2 * p1.x +
            (-p0.x + p2.x) * t +
            (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 +
            (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3),
        y:
          0.5 *
          (2 * p1.y +
            (-p0.y + p2.y) * t +
            (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 +
            (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3),
      });
    }
  }
  output.push(points[points.length - 1]);
  return output;
}

function planRoute() {
  if (!state.selected || state.phase === "parking") return;

  UI.planButton.disabled = true;
  UI.stagePill.textContent = "正在计算";
  UI.canvasHint.textContent = "正在搜索可行路径…";
  setStep(2, 1);

  window.setTimeout(() => {
    const spot = state.selected;
    const target = {
      x: spot.x + spot.width / 2,
      y: spot.y + spot.height / 2,
    };
    const isParallel = spot.orientation === "parallel";
    const approach = isParallel
      ? {
          x: target.x + 62,
          y: spot.side === "north" ? target.y + 44 : target.y - 44,
        }
      : {
          x: target.x,
          y: spot.row === 0 ? 310 : 375,
        };
    const preApproach = isParallel
      ? {
          x: target.x - 170,
          y: 342,
        }
      : {
          x: target.x - 54,
          y: spot.row === 0 ? 350 : 335,
        };

    const result = findPath(
      { x: state.car.x, y: state.car.y },
      preApproach,
    );

    if (!result) {
      UI.stagePill.textContent = "规划失败";
      UI.planButton.disabled = false;
      showToast("当前车位没有可行路径，请选择其他车位");
      return;
    }

    const simplified = simplifyPath(result.path, result.grid);
    const controlPoints = isParallel
      ? [
          ...simplified,
          approach,
          { x: target.x + 58, y: target.y },
          target,
        ]
      : [...simplified, approach, target];
    state.rawRoute = controlPoints;
    state.route = catmullRom(controlPoints, 20);
    calculateRouteMetrics();
    state.phase = "planned";
    UI.stagePill.textContent = "路线已就绪";
    UI.planButton.disabled = false;
    UI.parkButton.disabled = false;
    UI.canvasHint.textContent = "路径规划完成，可以开始自动泊车";
    setStep(2, 1);
    render(performance.now());
    showToast("安全路线规划完成，已避开全部占用车位");
  }, 520);
}

function calculateRouteMetrics() {
  state.routeLengths = [0];
  let total = 0;
  for (let i = 1; i < state.route.length; i += 1) {
    total += Math.hypot(
      state.route[i].x - state.route[i - 1].x,
      state.route[i].y - state.route[i - 1].y,
    );
    state.routeLengths.push(total);
  }
  state.totalRouteLength = total;
  const meters = total * MAP.scaleMeters;
  UI.metricDistance.textContent = `${meters.toFixed(1)} m`;
  updateEstimatedTime();
  UI.metricNodes.textContent = String(state.rawRoute.length);
}

function updateEstimatedTime() {
  if (!state.totalRouteLength) return;
  const meters = state.totalRouteLength * MAP.scaleMeters;
  const speedMultiplier = Number(UI.speedRange.value);
  const seconds = Math.max(3, meters / 1.25 / speedMultiplier);
  UI.metricTime.textContent = `${Math.round(seconds)} s`;
}

function startParking() {
  if (state.phase !== "planned" || state.route.length < 2) return;
  state.phase = "parking";
  state.animationStartedAt = performance.now();
  const speed = Number(UI.speedRange.value);
  const meters = state.totalRouteLength * MAP.scaleMeters;
  state.animationDuration = Math.max(3, meters / 1.25 / speed) * 1000;
  UI.planButton.disabled = true;
  UI.parkButton.disabled = true;
  UI.speedRange.disabled = true;
  UI.stagePill.textContent = "自动泊车中";
  UI.canvasHint.textContent = "车辆控制已接管 · 360° 障碍物监测中";
  setStep(3, 2);
  state.animationId = requestAnimationFrame(animateParking);
}

function animateParking(timestamp) {
  if (state.phase !== "parking") return;
  const elapsed = timestamp - state.animationStartedAt;
  const linearProgress = Math.min(1, elapsed / state.animationDuration);
  const progress = smoothStep(linearProgress);
  const targetDistance = progress * state.totalRouteLength;

  let segment = 1;
  while (
    segment < state.routeLengths.length - 1 &&
    state.routeLengths[segment] < targetDistance
  ) {
    segment += 1;
  }
  const previousDistance = state.routeLengths[segment - 1];
  const segmentLength = state.routeLengths[segment] - previousDistance || 1;
  const local = (targetDistance - previousDistance) / segmentLength;
  const from = state.route[segment - 1];
  const to = state.route[segment];
  state.car.x = from.x + (to.x - from.x) * local;
  state.car.y = from.y + (to.y - from.y) * local;

  const lookAhead = state.route[Math.min(segment + 2, state.route.length - 1)];
  const desiredAngle = Math.atan2(lookAhead.y - state.car.y, lookAhead.x - state.car.x);
  state.car.angle = lerpAngle(state.car.angle, desiredAngle, 0.15);

  if (linearProgress < 1) {
    state.animationId = requestAnimationFrame(animateParking);
  } else {
    completeParking();
  }
}

function smoothStep(value) {
  return value * value * (3 - 2 * value);
}

function lerpAngle(from, to, amount) {
  let difference = ((to - from + Math.PI) % (Math.PI * 2)) - Math.PI;
  if (difference < -Math.PI) difference += Math.PI * 2;
  return from + difference * amount;
}

function completeParking() {
  const spot = state.selected;
  state.phase = "complete";
  state.car.x = spot.x + spot.width / 2;
  state.car.y = spot.y + spot.height / 2;
  state.car.angle =
    spot.orientation === "parallel"
      ? spot.targetAngle
      : spot.row === 0
        ? -Math.PI / 2
        : Math.PI / 2;
  UI.stagePill.textContent = "泊车完成";
  UI.canvasHint.textContent = `${spot.id} 泊车完成 · 车辆已挂入 P 挡`;
  UI.speedRange.disabled = false;
  setStep(3, 3);
  render(performance.now());
  showToast(`泊车完成，车辆已安全停入 ${spot.id}`);
}

function resetDemo() {
  if (state.animationId) cancelAnimationFrame(state.animationId);
  state.selected = null;
  state.route = [];
  state.rawRoute = [];
  state.car = { ...initialCar };
  state.phase = "select";
  state.animationId = null;
  state.hoverSpot = null;

  UI.selectedSpot.textContent = "尚未选择";
  UI.spotDistance.textContent = "-- m";
  UI.metricDistance.textContent = "--";
  UI.metricTime.textContent = "--";
  UI.metricNodes.textContent = "--";
  UI.stagePill.textContent = "等待选位";
  UI.planButton.disabled = true;
  UI.parkButton.disabled = true;
  UI.speedRange.disabled = false;
  UI.canvasHint.textContent = "点击绿色空闲车位进行选择";
  setStep(1, 0);
  render(performance.now());
}

let toastTimer;
function showToast(message) {
  window.clearTimeout(toastTimer);
  UI.toast.textContent = message;
  UI.toast.classList.add("show");
  toastTimer = window.setTimeout(() => UI.toast.classList.remove("show"), 2600);
}

UI.planButton.addEventListener("click", planRoute);
UI.parkButton.addEventListener("click", startParking);
UI.resetButton.addEventListener("click", resetDemo);
UI.speedRange.addEventListener("input", () => {
  UI.speedOutput.textContent = `${Number(UI.speedRange.value).toFixed(1)}×`;
  const percentage =
    ((Number(UI.speedRange.value) - Number(UI.speedRange.min)) /
      (Number(UI.speedRange.max) - Number(UI.speedRange.min))) *
    100;
  UI.speedRange.style.background = `linear-gradient(90deg, var(--accent) ${percentage}%, #2b3c37 ${percentage}%)`;
  updateEstimatedTime();
});

UI.availableCount.textContent = `${parkingSpots.filter((spot) => !spot.occupied).length} 个`;
updateClock();
window.setInterval(updateClock, 1000);

function visualLoop(timestamp) {
  render(timestamp);
  requestAnimationFrame(visualLoop);
}

requestAnimationFrame(visualLoop);
