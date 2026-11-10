// SPDX-License-Identifier: Apache-2.0
import { test, expect } from '@playwright/test';

type MapFrame = {
  originX: number;
  originY: number;
  resolution: number;
  widthCells: number;
  heightCells: number;
  goal: { x: number; y: number; px: number; py: number };
  start: { px: number; py: number };
  canvas: { width: number; height: number };
  tebTime: number;
  mpcTime: number;
  tebSuccess: number;
  mpcSuccess: number;
};

async function waitReady(page: import('@playwright/test').Page) {
  await page.waitForSelector('[data-sim-ready="1"]', { timeout: 60_000 });
}

async function step(page: import('@playwright/test').Page, n: number) {
  return page.evaluate((ticks) => (globalThis as { __stepTicks?: (n: number) => string }).__stepTicks?.(ticks) ?? '', n);
}

async function frame(page: import('@playwright/test').Page): Promise<MapFrame> {
  return page.evaluate(() => (globalThis as { __mapFrame?: () => MapFrame }).__mapFrame?.() as MapFrame);
}

test('honesty label, separate planner clocks, and S1 traces', async ({ page }) => {
  await page.goto('./');
  await expect(page).toHaveTitle(/simulation only/i);
  await expect(page.locator('h1')).toHaveText(/TEB vs MPC \(OSQP\) local planners — simulation only/);
  await expect(page.locator('#honesty')).toContainText(/2D kinematic simulation/i);
  await expect(page.locator('#legend')).toContainText('TEB-style');
  await expect(page.locator('#legend')).toContainText('MPC (OSQP)');
  await waitReady(page);
  const after = await step(page, 20);
  expect(after).toMatch(/teb_time=1\.00s/);
  expect(after).toMatch(/mpc_time=1\.00s/);
  expect(after).not.toMatch(/teb_time=2\.00s/);
  expect(after).toMatch(/teb_time=1\.00s path=0\.[1-9]/);
  expect(after).toMatch(/mpc_time=1\.00s path=0\.[1-9]/);
  const wasmResp = await page.request.get('tml_wasm.wasm');
  expect(wasmResp.headers()['content-type']).toMatch(/wasm|octet-stream/);
});

test('S4 uses its own origin, stays in frame, and lets MPC succeed', async ({ page }) => {
  await page.goto('./');
  await waitReady(page);
  await page.locator('#scenario').selectOption('4');
  const before = await frame(page);
  expect(before.originX).toBeCloseTo(-6, 6);
  expect(before.originY).toBeCloseTo(-4, 6);
  expect(before.widthCells).toBe(360);
  expect(before.goal.x).toBeCloseTo(-2, 6);
  expect(before.goal.px).toBeGreaterThan(40);
  expect(before.goal.px).toBeLessThan(before.canvas.width - 40);
  expect(before.goal.py).toBeGreaterThan(20);
  expect(before.goal.py).toBeLessThan(before.canvas.height - 20);

  let info = before;
  for (let i = 0; i < 8; ++i) {
    if (info.tebSuccess === 1 && info.mpcSuccess === 1) break;
    await step(page, 80);
    info = await frame(page);
  }
  expect(info.mpcSuccess).toBe(1);
  expect(info.tebSuccess).toBe(1);
  expect(info.tebTime).toBeLessThan(25);
  expect(info.mpcTime).toBeLessThan(25);
  expect(info.mpcTime).toBeGreaterThan(1);
  await page.screenshot({ path: 'demo-screenshot.png', fullPage: true });
  await page.screenshot({ path: '../docs/demo.png', fullPage: true });
});
