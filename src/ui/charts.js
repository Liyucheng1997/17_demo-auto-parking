/**
 * 轻量时间序列图表（Canvas）：多条曲线共用一个 y 轴，含图例、十字准线悬停提示、
 * 当前时刻游标、换挡区间底色。用于速度、转角、跟踪误差等遥测数据。
 */

export const SERIES_COLORS = {
  reference: "#3987e5",
  actual: "#d95926",
};

export class TimeSeriesChart {
  constructor(canvas, { title, unit, series, yMin = null, yMax = null, symmetric = false, decimals = 2 }) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.title = title;
    this.unit = unit;
    this.series = series;
    this.yMin = yMin;
    this.yMax = yMax;
    this.symmetric = symmetric;
    this.decimals = decimals;
    this.data = null;
    this.cursor = null;
    this.hoverX = null;
    this.duration = 1;
    this.bands = [];
    canvas.addEventListener("mousemove", (e) => {
      const rect = canvas.getBoundingClientRect();
      this.hoverX = e.clientX - rect.left;
      this.render();
    });
    canvas.addEventListener("mouseleave", () => {
      this.hoverX = null;
      this.render();
    });
  }

  /** data: { t: number[], [key]: number[] }；bands: [{t0,t1,color}] */
  setData(data, { duration, bands = [] } = {}) {
    this.data = data;
    this.duration = Math.max(1, duration || (data?.t?.length ? data.t[data.t.length - 1] : 1));
    this.bands = bands;
  }

  setCursor(t) {
    this.cursor = t;
  }

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    this.dpr = Math.min(2, window.devicePixelRatio || 1);
    this.w = rect.width;
    this.h = rect.height;
    this.canvas.width = Math.round(rect.width * this.dpr);
    this.canvas.height = Math.round(rect.height * this.dpr);
  }

  #range() {
    let lo = Infinity;
    let hi = -Infinity;
    if (this.data) {
      for (const s of this.series) {
        for (const v of this.data[s.key] || []) {
          if (v < lo) lo = v;
          if (v > hi) hi = v;
        }
      }
    }
    if (!Number.isFinite(lo)) {
      lo = this.yMin ?? -1;
      hi = this.yMax ?? 1;
    }
    if (this.yMin !== null) lo = Math.min(lo, this.yMin);
    if (this.yMax !== null) hi = Math.max(hi, this.yMax);
    if (this.symmetric) {
      const m = Math.max(Math.abs(lo), Math.abs(hi), 1e-3);
      lo = -m;
      hi = m;
    }
    const pad = (hi - lo) * 0.08 || 0.1;
    return [lo - pad, hi + pad];
  }

  render() {
    if (!this.w) this.resize();
    const ctx = this.ctx;
    const { w, h, dpr } = this;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const m = { l: 40, r: 10, t: 26, b: 20 };
    const pw = w - m.l - m.r;
    const ph = h - m.t - m.b;
    const [lo, hi] = this.#range();
    const X = (t) => m.l + (t / this.duration) * pw;
    const Y = (v) => m.t + (1 - (v - lo) / (hi - lo)) * ph;

    // 标题与图例
    ctx.font = "700 11.5px Inter, 'Microsoft YaHei', sans-serif";
    ctx.fillStyle = "#edf7f4";
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    ctx.fillText(`${this.title}`, m.l - 30, 11);
    ctx.font = "500 10.5px Inter, 'Microsoft YaHei', sans-serif";
    let lx = w - m.r;
    ctx.textAlign = "right";
    for (const s of [...this.series].reverse()) {
      const tw = ctx.measureText(s.label).width;
      ctx.fillStyle = "#9db3ac";
      ctx.fillText(s.label, lx, 11);
      ctx.strokeStyle = s.color;
      ctx.lineWidth = 2;
      ctx.setLineDash(s.dash || []);
      ctx.beginPath();
      ctx.moveTo(lx - tw - 22, 11);
      ctx.lineTo(lx - tw - 6, 11);
      ctx.stroke();
      ctx.setLineDash([]);
      lx -= tw + 32;
    }

    // 挡位区间底色
    for (const b of this.bands) {
      ctx.fillStyle = b.color;
      ctx.fillRect(X(b.t0), m.t, Math.max(0, X(b.t1) - X(b.t0)), ph);
    }

    // 网格与坐标轴
    ctx.strokeStyle = "rgba(146, 196, 181, 0.1)";
    ctx.lineWidth = 1;
    ctx.fillStyle = "#829b94";
    ctx.font = "500 10px Inter, sans-serif";
    ctx.textAlign = "right";
    const ticks = niceTicks(lo, hi, 4);
    for (const v of ticks) {
      const y = Y(v);
      ctx.beginPath();
      ctx.moveTo(m.l, y);
      ctx.lineTo(w - m.r, y);
      ctx.stroke();
      ctx.fillText(formatTick(v), m.l - 6, y);
    }
    if (lo < 0 && hi > 0) {
      ctx.strokeStyle = "rgba(146, 196, 181, 0.32)";
      ctx.beginPath();
      ctx.moveTo(m.l, Y(0));
      ctx.lineTo(w - m.r, Y(0));
      ctx.stroke();
    }
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    for (const t of niceTicks(0, this.duration, 6)) {
      if (t < 0 || t > this.duration) continue;
      ctx.fillText(`${formatTick(t)}s`, X(t), h - m.b + 5);
    }
    ctx.textAlign = "left";
    ctx.textBaseline = "top";
    ctx.fillText(this.unit, 4, m.t - 2);

    // 曲线
    const data = this.data;
    if (data?.t?.length) {
      ctx.save();
      ctx.beginPath();
      ctx.rect(m.l, m.t, pw, ph);
      ctx.clip();
      for (const s of this.series) {
        const arr = data[s.key];
        if (!arr) continue;
        ctx.strokeStyle = s.color;
        ctx.lineWidth = 2;
        ctx.lineJoin = "round";
        ctx.setLineDash(s.dash || []);
        ctx.beginPath();
        const step = Math.max(1, Math.floor(arr.length / (pw * 1.5)));
        for (let i = 0; i < arr.length; i += step) {
          const x = X(data.t[i]);
          const y = Y(arr[i]);
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        }
        ctx.stroke();
        ctx.setLineDash([]);
      }
      ctx.restore();
    } else {
      ctx.fillStyle = "#5f756f";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.font = "500 11px 'Microsoft YaHei', sans-serif";
      ctx.fillText("开始泊车后实时记录", m.l + pw / 2, m.t + ph / 2);
    }

    // 当前时刻游标
    if (this.cursor !== null && data?.t?.length) {
      ctx.strokeStyle = "rgba(237, 247, 244, 0.5)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(X(this.cursor), m.t);
      ctx.lineTo(X(this.cursor), m.t + ph);
      ctx.stroke();
    }

    // 悬停十字准线与提示
    if (this.hoverX !== null && data?.t?.length && this.hoverX >= m.l && this.hoverX <= w - m.r) {
      const t = ((this.hoverX - m.l) / pw) * this.duration;
      const i = nearestIndex(data.t, t);
      const x = X(data.t[i]);
      ctx.strokeStyle = "rgba(237, 247, 244, 0.7)";
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(x, m.t);
      ctx.lineTo(x, m.t + ph);
      ctx.stroke();
      ctx.setLineDash([]);
      const lines = [`t = ${data.t[i].toFixed(2)} s`];
      for (const s of this.series) {
        const v = data[s.key]?.[i];
        if (v === undefined) continue;
        ctx.fillStyle = s.color;
        ctx.strokeStyle = "#0b1715";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(x, Y(v), 4, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        lines.push(`${s.label}: ${v.toFixed(this.decimals)} ${this.unit}`);
      }
      ctx.font = "500 10.5px Inter, 'Microsoft YaHei', sans-serif";
      const bw = Math.max(...lines.map((l) => ctx.measureText(l).width)) + 16;
      const bh = lines.length * 15 + 8;
      const bx = x + bw + 12 > w ? x - bw - 8 : x + 8;
      ctx.fillStyle = "rgba(7, 16, 15, 0.94)";
      ctx.strokeStyle = "rgba(146, 196, 181, 0.3)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.rect(bx, m.t + 2, bw, bh);
      ctx.fill();
      ctx.stroke();
      ctx.fillStyle = "#edf7f4";
      ctx.textAlign = "left";
      ctx.textBaseline = "top";
      lines.forEach((l, k) => ctx.fillText(l, bx + 8, m.t + 7 + k * 15));
    }
  }
}

function nearestIndex(arr, t) {
  let lo = 0;
  let hi = arr.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (arr[mid] < t) lo = mid;
    else hi = mid;
  }
  return Math.abs(arr[lo] - t) < Math.abs(arr[hi] - t) ? lo : hi;
}

function niceTicks(lo, hi, count) {
  const span = hi - lo;
  if (span <= 0) return [lo];
  const raw = span / count;
  const mag = 10 ** Math.floor(Math.log10(raw));
  const norm = raw / mag;
  const step = (norm < 1.5 ? 1 : norm < 3 ? 2 : norm < 7 ? 5 : 10) * mag;
  const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(Math.abs(v) < 1e-12 ? 0 : v);
  return out;
}

function formatTick(v) {
  if (Math.abs(v) >= 10 || Number.isInteger(v)) return String(Math.round(v * 10) / 10);
  return v.toFixed(Math.abs(v) < 1 ? 2 : 1).replace(/\.?0+$/, "");
}
