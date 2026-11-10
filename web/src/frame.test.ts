// SPDX-License-Identifier: Apache-2.0
import { describe, expect, it } from 'vitest';
import { fitMapToCanvas, isInsideCanvas, worldToCanvas } from './frame';

const canvas = { width: 640, height: 360 };

describe('per-scenario map framing', () => {
  it('fits the S1 origin and keeps the goal off the canvas edge', () => {
    const frame = fitMapToCanvas(
      { originX: -2, originY: -4, resolution: 0.05, widthCells: 320, heightCells: 160 },
      canvas.width,
      canvas.height,
    );
    expect(frame.originX).toBe(-2);
    expect(frame.originY).toBe(-4);
    const goal = worldToCanvas(frame, 8, 0);
    const start = worldToCanvas(frame, 0, 0);
    expect(isInsideCanvas(goal.px, goal.py, canvas.width, canvas.height, 20)).toBe(true);
    expect(isInsideCanvas(start.px, start.py, canvas.width, canvas.height, 20)).toBe(true);
    expect(goal.px).toBeGreaterThan(start.px);
  });

  it('uses the S4 origin (-6, -4) and does not pin the goal to the left edge', () => {
    const frame = fitMapToCanvas(
      { originX: -6, originY: -4, resolution: 0.05, widthCells: 360, heightCells: 160 },
      canvas.width,
      canvas.height,
    );
    expect(frame.originX).toBe(-6);
    expect(frame.originY).toBe(-4);
    const goal = worldToCanvas(frame, -2, 0.5);
    const start = worldToCanvas(frame, 0, 0);
    const left = worldToCanvas(frame, -6, 0);
    // The old hardcoded origin (-2, -4) placed the S4 goal at px = 0.
    expect(goal.px).toBeGreaterThan(40);
    expect(goal.px).toBeLessThan(canvas.width - 40);
    expect(isInsideCanvas(goal.px, goal.py, canvas.width, canvas.height, 20)).toBe(true);
    expect(isInsideCanvas(start.px, start.py, canvas.width, canvas.height, 20)).toBe(true);
    expect(goal.px).toBeGreaterThan(left.px + 20);
    expect(start.px).toBeGreaterThan(goal.px);
  });
});
