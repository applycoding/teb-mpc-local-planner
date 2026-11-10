// SPDX-License-Identifier: Apache-2.0
import { test, expect } from '@playwright/test';

test('loads wasm demo and runs both planners', async ({ page }) => {
  await page.goto('./');
  await expect(page.getByText('TEB-style')).toBeVisible();
  await expect(page.getByText('MPC (OSQP)')).toBeVisible();
  await page.waitForSelector('[data-sim-ready="1"]', { timeout: 60_000 });
  const metrics = page.locator('#metrics');
  const after = await page.evaluate(() => (globalThis as { __stepTicks?: (n: number) => string }).__stepTicks?.(20) ?? '');
  expect(after).not.toMatch(/^scenario S1\nsim_time=0\.00s/);
  expect(after).toMatch(/teb path=0\.[1-9]/);
  expect(after).toMatch(/mpc path=0\.[1-9]/);
  const wasmResp = await page.request.get('tml_wasm.wasm');
  expect(wasmResp.headers()['content-type']).toMatch(/wasm|octet-stream/);
  await page.screenshot({ path: 'demo-screenshot.png', fullPage: true });
  await page.screenshot({ path: '../docs/demo.png', fullPage: true });
});
