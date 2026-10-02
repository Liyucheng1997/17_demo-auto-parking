import { test } from "node:test";
import assert from "node:assert/strict";
import { distanceTransform } from "../src/model/gridMap.js";
import { createRng } from "../src/core/math.js";

test("EDT 与暴力解一致", () => {
  const cols = 37;
  const rows = 23;
  const rng = createRng(11);
  const occ = new Uint8Array(cols * rows);
  for (let i = 0; i < occ.length; i += 1) occ[i] = rng() < 0.04 ? 1 : 0;
  const dist = distanceTransform(occ, cols, rows);
  for (let r = 0; r < rows; r += 1) {
    for (let c = 0; c < cols; c += 1) {
      let best = Infinity;
      for (let rr = 0; rr < rows; rr += 1) {
        for (let cc = 0; cc < cols; cc += 1) {
          if (occ[rr * cols + cc]) best = Math.min(best, Math.hypot(rr - r, cc - c));
        }
      }
      assert.ok(Math.abs(dist[r * cols + c] - best) < 1e-4, `(${c},${r})`);
    }
  }
});
