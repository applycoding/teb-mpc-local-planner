// SPDX-License-Identifier: Apache-2.0

export type MapExtent = {
  originX: number;
  originY: number;
  resolution: number;
  widthCells: number;
  heightCells: number;
};

export type FittedFrame = {
  scale: number;
  padX: number;
  padY: number;
  worldW: number;
  worldH: number;
  originX: number;
  originY: number;
  canvasWidth: number;
  canvasHeight: number;
};

export function fitMapToCanvas(
  extent: MapExtent,
  canvasWidth: number,
  canvasHeight: number,
  margin = 16,
): FittedFrame {
  const worldW = extent.widthCells * extent.resolution;
  const worldH = extent.heightCells * extent.resolution;
  const innerW = Math.max(1, canvasWidth - 2 * margin);
  const innerH = Math.max(1, canvasHeight - 2 * margin);
  const scale = Math.min(innerW / Math.max(worldW, 1e-9), innerH / Math.max(worldH, 1e-9));
  const padX = (canvasWidth - worldW * scale) / 2;
  const padY = (canvasHeight - worldH * scale) / 2;
  return {
    scale,
    padX,
    padY,
    worldW,
    worldH,
    originX: extent.originX,
    originY: extent.originY,
    canvasWidth,
    canvasHeight,
  };
}

export function worldToCanvas(frame: FittedFrame, x: number, y: number): { px: number; py: number } {
  return {
    px: frame.padX + (x - frame.originX) * frame.scale,
    py: frame.canvasHeight - (frame.padY + (y - frame.originY) * frame.scale),
  };
}

export function worldLengthToCanvas(frame: FittedFrame, meters: number): number {
  return meters * frame.scale;
}

export function isInsideCanvas(px: number, py: number, width: number, height: number, inset = 8): boolean {
  return px >= inset && py >= inset && px <= width - inset && py <= height - inset;
}
