/**
 * 驾驶舱仪表：方向盘转角（= 前轮转角 × 转向传动比）、挡位、车速、
 * 12 路超声波雷达俯视环形示意。
 */

import { ultrasonicLayout } from "../model/vehicle.js";

export class Dashboard {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
  }

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    this.dpr = Math.min(2, window.devicePixelRatio || 1);
    this.w = rect.width;
    this.h = rect.height;
    this.canvas.width = Math.round(rect.width * this.dpr);
    this.canvas.height = Math.round(rect.height * this.dpr);
  }

  render({ vehicle, state, gear = "P", readings = [] }) {
    if (!this.w) this.resize();
    const ctx = this.ctx;
    const { w, h, dpr } = this;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const delta = state?.delta || 0;
    const v = state?.v || 0;
    const wheelDeg = (delta * 180) / Math.PI * vehicle.steeringRatio;

    // —— 方向盘 ——
    const r = Math.min(h * 0.36, w * 0.16);
    const cx = r + 14;
    const cy = h / 2 + 2;
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate((-wheelDeg * Math.PI) / 180);
    ctx.strokeStyle = "#2b3f3a";
    ctx.lineWidth = r * 0.2;
    ctx.beginPath();
    ctx.arc(0, 0, r * 0.86, 0, Math.PI * 2);
    ctx.stroke();
    ctx.strokeStyle = "#45e6a7";
    ctx.lineWidth = r * 0.2;
    ctx.beginPath();
    ctx.arc(0, 0, r * 0.86, -Math.PI / 2 - 0.16, -Math.PI / 2 + 0.16);
    ctx.stroke();
    ctx.strokeStyle = "#2b3f3a";
    ctx.lineWidth = r * 0.14;
    ctx.beginPath();
    ctx.moveTo(-r * 0.8, 0);
    ctx.lineTo(r * 0.8, 0);
    ctx.moveTo(0, 0);
    ctx.lineTo(0, r * 0.8);
    ctx.stroke();
    ctx.fillStyle = "#132622";
    ctx.beginPath();
    ctx.arc(0, 0, r * 0.3, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
    ctx.fillStyle = "#edf7f4";
    ctx.font = `700 ${Math.round(r * 0.26)}px Inter, sans-serif`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(`${Math.round(wheelDeg)}°`, cx, cy);
    ctx.fillStyle = "#829b94";
    ctx.font = "500 10px 'Microsoft YaHei', sans-serif";
    ctx.fillText(`前轮 ${(delta * 57.2958).toFixed(1)}°`, cx, h - 8);

    // —— 挡位与车速 ——
    const gx = cx + r + 26;
    ctx.textAlign = "left";
    ctx.font = "800 13px Inter, sans-serif";
    ["P", "R", "N", "D"].forEach((g, i) => {
      const active = g === gear;
      ctx.fillStyle = active ? (g === "R" ? "#ffa24c" : "#45e6a7") : "#3a4f4a";
      ctx.fillText(g, gx + i * 17, 20);
    });
    ctx.fillStyle = "#edf7f4";
    ctx.font = "700 30px Inter, sans-serif";
    ctx.fillText(`${(Math.abs(v) * 3.6).toFixed(1)}`, gx, h / 2 + 8);
    ctx.fillStyle = "#829b94";
    ctx.font = "500 10.5px Inter, sans-serif";
    ctx.fillText("km/h", gx, h / 2 + 32);

    // —— 雷达环 ——
    const rx = w - Math.min(w * 0.22, h * 0.62) - 10;
    const ry = h / 2;
    const scale = Math.min(h / 9.5, (w - rx) / 4.6);
    ctx.save();
    ctx.translate(rx + scale * 1.2, ry);
    ctx.rotate(-Math.PI / 2);
    // 车身（后轴原点 → 车身中心）
    const off = vehicle.length / 2 - vehicle.rearOverhang;
    ctx.fillStyle = "#1c3a31";
    ctx.strokeStyle = "rgba(69, 230, 167, 0.7)";
    ctx.lineWidth = 1;
    const bx = (off - vehicle.length / 2) * scale;
    const by = (-vehicle.width / 2) * scale;
    ctx.fillRect(bx, by, vehicle.length * scale, vehicle.width * scale);
    ctx.strokeRect(bx, by, vehicle.length * scale, vehicle.width * scale);
    const layout = ultrasonicLayout(vehicle);
    for (const sensor of layout) {
      const reading = readings.find((rd) => rd.id === sensor.id);
      const d = reading?.distance ?? null;
      const color = d === null ? "rgba(69,230,167,0.18)" : d < 0.3 ? "#ff4b44" : d < 0.6 ? "#ff9a4a" : d < 1.2 ? "#f4c86a" : "#45e6a7";
      ctx.strokeStyle = color;
      ctx.lineWidth = 3.2;
      ctx.lineCap = "round";
      const rr = 0.45 * scale;
      const sx = sensor.x * scale;
      const sy = -sensor.y * scale;
      ctx.beginPath();
      ctx.arc(sx, sy, rr, -sensor.yaw - 0.35, -sensor.yaw + 0.35);
      ctx.stroke();
    }
    ctx.restore();
    const nearest = readings.reduce((m, rd) => (rd.distance !== null && rd.distance < m ? rd.distance : m), Infinity);
    ctx.fillStyle = nearest < 0.3 ? "#ff6b66" : "#829b94";
    ctx.font = "600 10.5px 'Microsoft YaHei', sans-serif";
    ctx.textAlign = "center";
    ctx.fillText(Number.isFinite(nearest) ? `最近 ${nearest.toFixed(2)} m` : "雷达无回波", rx + scale * 1.2, h - 8);
  }
}
