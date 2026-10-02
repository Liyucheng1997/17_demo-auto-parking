/**
 * 几何与碰撞原语：凸多边形、有向包围盒 (OBB)、分离轴定理 (SAT)、射线求交。
 * 多边形统一表示为 { points: [{x,y}...], cx, cy, radius }，
 * 其中 (cx, cy, radius) 是外接圆，用于粗筛 (broad phase)。
 */

/** 由顶点构造凸多边形并预计算外接圆 */
export function makePolygon(points) {
  let cx = 0;
  let cy = 0;
  for (const p of points) {
    cx += p.x;
    cy += p.y;
  }
  cx /= points.length;
  cy /= points.length;
  let radius = 0;
  for (const p of points) radius = Math.max(radius, Math.hypot(p.x - cx, p.y - cy));
  return { points, cx, cy, radius };
}

/**
 * 有向矩形：中心 (cx, cy)，朝向 theta，长 length（沿朝向）、宽 width。
 * 顶点顺序：左前、左后、右后、右前（逆时针）。
 */
export function orientedBox(cx, cy, theta, length, width) {
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  const hl = length / 2;
  const hw = width / 2;
  const corner = (lx, ly) => ({ x: cx + c * lx - s * ly, y: cy + s * lx + c * ly });
  return makePolygon([corner(hl, hw), corner(-hl, hw), corner(-hl, -hw), corner(hl, -hw)]);
}

/** 轴对齐矩形 */
export function rectPolygon(x, y, width, height) {
  return makePolygon([
    { x, y },
    { x: x + width, y },
    { x: x + width, y: y + height },
    { x, y: y + height },
  ]);
}

/** 用正多边形近似圆形障碍物（锥桶、立柱） */
export function circlePolygon(cx, cy, r, sides = 10) {
  const points = [];
  for (let i = 0; i < sides; i += 1) {
    const a = (i / sides) * Math.PI * 2;
    points.push({ x: cx + r * Math.cos(a), y: cy + r * Math.sin(a) });
  }
  return makePolygon(points);
}

function projectInterval(points, ax, ay) {
  let min = Infinity;
  let max = -Infinity;
  for (const p of points) {
    const d = p.x * ax + p.y * ay;
    if (d < min) min = d;
    if (d > max) max = d;
  }
  return [min, max];
}

function hasSeparatingAxis(a, b) {
  const pts = a.points;
  for (let i = 0; i < pts.length; i += 1) {
    const p = pts[i];
    const q = pts[(i + 1) % pts.length];
    const ax = -(q.y - p.y);
    const ay = q.x - p.x;
    const [minA, maxA] = projectInterval(pts, ax, ay);
    const [minB, maxB] = projectInterval(b.points, ax, ay);
    if (maxA < minB || maxB < minA) return true;
  }
  return false;
}

/** 两个凸多边形是否相交（外接圆粗筛 + SAT 精检） */
export function polygonsIntersect(a, b) {
  const dx = a.cx - b.cx;
  const dy = a.cy - b.cy;
  const r = a.radius + b.radius;
  if (dx * dx + dy * dy > r * r) return false;
  return !hasSeparatingAxis(a, b) && !hasSeparatingAxis(b, a);
}

/** 点到线段距离 */
export function pointSegmentDistance(px, py, ax, ay, bx, by) {
  const vx = bx - ax;
  const vy = by - ay;
  const len2 = vx * vx + vy * vy;
  let t = len2 > 0 ? ((px - ax) * vx + (py - ay) * vy) / len2 : 0;
  t = Math.max(0, Math.min(1, t));
  return Math.hypot(px - (ax + t * vx), py - (ay + t * vy));
}

export function pointInPolygon(px, py, polygon) {
  const pts = polygon.points;
  let inside = false;
  for (let i = 0, j = pts.length - 1; i < pts.length; j = i, i += 1) {
    const a = pts[i];
    const b = pts[j];
    if (a.y > py !== b.y > py && px < ((b.x - a.x) * (py - a.y)) / (b.y - a.y) + a.x) {
      inside = !inside;
    }
  }
  return inside;
}

/** 点到多边形的有符号距离（内部为负） */
export function pointPolygonDistance(px, py, polygon) {
  const pts = polygon.points;
  let best = Infinity;
  for (let i = 0; i < pts.length; i += 1) {
    const a = pts[i];
    const b = pts[(i + 1) % pts.length];
    best = Math.min(best, pointSegmentDistance(px, py, a.x, a.y, b.x, b.y));
  }
  return pointInPolygon(px, py, polygon) ? -best : best;
}

/** 两个凸多边形之间的最小距离（相交时为 0） */
export function polygonDistance(a, b) {
  if (polygonsIntersect(a, b)) return 0;
  let best = Infinity;
  for (const [p, q] of [
    [a, b],
    [b, a],
  ]) {
    const pts = q.points;
    for (const v of p.points) {
      for (let i = 0; i < pts.length; i += 1) {
        const s = pts[i];
        const t = pts[(i + 1) % pts.length];
        best = Math.min(best, pointSegmentDistance(v.x, v.y, s.x, s.y, t.x, t.y));
      }
    }
  }
  return best;
}

/**
 * 射线与多边形求交，返回最近交点距离（无交点返回 Infinity）。
 * 射线：起点 (ox, oy)，单位方向 (dx, dy)。
 */
export function rayPolygonDistance(ox, oy, dx, dy, polygon) {
  const pts = polygon.points;
  let best = Infinity;
  for (let i = 0; i < pts.length; i += 1) {
    const a = pts[i];
    const b = pts[(i + 1) % pts.length];
    const ex = b.x - a.x;
    const ey = b.y - a.y;
    const denom = dx * ey - dy * ex;
    if (Math.abs(denom) < 1e-12) continue;
    const t = ((a.x - ox) * ey - (a.y - oy) * ex) / denom;
    const u = ((a.x - ox) * dy - (a.y - oy) * dx) / denom;
    if (t >= 0 && u >= 0 && u <= 1 && t < best) best = t;
  }
  return best;
}
