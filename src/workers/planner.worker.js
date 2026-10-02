/**
 * 规划 Web Worker：把计算密集的 Hybrid A* 搜索移出主线程，界面保持流畅。
 * 输入/输出均为可结构化克隆的纯数据。
 */

import { planParking } from "../planning/planner.js";

self.onmessage = (event) => {
  const { id, request } = event.data;
  try {
    const plan = planParking(request);
    self.postMessage({ id, plan });
  } catch (error) {
    self.postMessage({ id, error: error?.message || String(error) });
  }
};
