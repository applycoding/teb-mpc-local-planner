// SPDX-License-Identifier: Apache-2.0
import { fitMapToCanvas, worldLengthToCanvas, worldToCanvas, type FittedFrame } from './frame';

type TmlModule = {
  cwrap: (name: string, ret: string | null, args: string[]) => (...a: number[]) => number;
  HEAPF64?: Float64Array;
};

const canvas = document.getElementById('c') as HTMLCanvasElement;
const ctx = canvas.getContext('2d');
const metricsEl = document.getElementById('metrics') as HTMLPreElement;
const scenarioSel = document.getElementById('scenario') as HTMLSelectElement;

let mod: TmlModule;
let playing = false;
let fns: Record<string, (...a: number[]) => number> = {};

function wrap(name: string, ret: string, args: string[] = []) {
  return mod.cwrap(name, ret, args);
}

function currentFrame(): FittedFrame {
  return fitMapToCanvas(
    {
      originX: fns.originX(),
      originY: fns.originY(),
      resolution: fns.resolution(),
      widthCells: fns.mapW(),
      heightCells: fns.mapH(),
    },
    canvas.width,
    canvas.height,
  );
}

function drawTrace(
  frame: FittedFrame,
  count: number,
  xs: (i: number) => number,
  ys: (i: number) => number,
  color: string,
) {
  if (!ctx || count < 2) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  for (let i = 0; i < count; ++i) {
    const p = worldToCanvas(frame, xs(i), ys(i));
    if (i === 0) ctx.moveTo(p.px, p.py);
    else ctx.lineTo(p.px, p.py);
  }
  ctx.stroke();
}

function drawScene() {
  if (!ctx || !mod) return;
  const frame = currentFrame();
  ctx.fillStyle = '#102018';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // Map bounds.
  const sw = worldToCanvas(frame, frame.originX, frame.originY);
  const ne = worldToCanvas(frame, frame.originX + frame.worldW, frame.originY + frame.worldH);
  ctx.fillStyle = '#16261c';
  ctx.fillRect(sw.px, ne.py, ne.px - sw.px, sw.py - ne.py);
  ctx.strokeStyle = '#2f5a47';
  ctx.strokeRect(sw.px, ne.py, ne.px - sw.px, sw.py - ne.py);

  // Occupancy cells.
  const occN = fns.occCount();
  const res = fns.resolution();
  ctx.fillStyle = '#3d5a4c';
  for (let k = 0; k < occN; ++k) {
    const i = fns.occI(k);
    const j = fns.occJ(k);
    const x0 = frame.originX + i * res;
    const y0 = frame.originY + j * res;
    const p0 = worldToCanvas(frame, x0, y0);
    const p1 = worldToCanvas(frame, x0 + res, y0 + res);
    ctx.fillRect(p0.px, p1.py, Math.max(1, p1.px - p0.px), Math.max(1, p0.py - p1.py));
  }

  // Explicit obstacle rectangles (on top of occupancy).
  const nRects = fns.nRects();
  ctx.fillStyle = 'rgba(48, 80, 64, 0.85)';
  ctx.strokeStyle = '#6a9a80';
  for (let i = 0; i < nRects; ++i) {
    const p0 = worldToCanvas(frame, fns.rectX0(i), fns.rectY0(i));
    const p1 = worldToCanvas(frame, fns.rectX1(i), fns.rectY1(i));
    const x = Math.min(p0.px, p1.px);
    const y = Math.min(p0.py, p1.py);
    const w = Math.abs(p1.px - p0.px);
    const h = Math.abs(p1.py - p0.py);
    ctx.fillRect(x, y, w, h);
    ctx.strokeRect(x, y, w, h);
  }

  // Moving obstacles at each planner's time (usually the same).
  const times = [fns.tebT(), fns.mpcT()];
  const uniqueTimes = Math.abs(times[0] - times[1]) < 1e-9 ? [times[0]] : times;
  const nMovers = fns.nMovers();
  for (const t of uniqueTimes) {
    for (let i = 0; i < nMovers; ++i) {
      const x = fns.moverX(i) + fns.moverVx(i) * t;
      const y = fns.moverY(i) + fns.moverVy(i) * t;
      const p = worldToCanvas(frame, x, y);
      const r = Math.max(3, worldLengthToCanvas(frame, fns.moverR(i)));
      ctx.fillStyle = 'rgba(224, 122, 122, 0.85)';
      ctx.strokeStyle = '#ffd0d0';
      ctx.beginPath();
      ctx.arc(p.px, p.py, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
  }

  drawTrace(frame, fns.tebTraceN(), fns.tebTraceX, fns.tebTraceY, '#8fd4ff');
  drawTrace(frame, fns.mpcTraceN(), fns.mpcTraceX, fns.mpcTraceY, '#ffb88f');

  const goal = worldToCanvas(frame, fns.goalX(), fns.goalY());
  ctx.strokeStyle = '#ffe566';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(goal.px, goal.py, 8, 0, Math.PI * 2);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(goal.px - 10, goal.py);
  ctx.lineTo(goal.px + 10, goal.py);
  ctx.moveTo(goal.px, goal.py - 10);
  ctx.lineTo(goal.px, goal.py + 10);
  ctx.stroke();

  const pT = worldToCanvas(frame, fns.tebX(), fns.tebY());
  const pM = worldToCanvas(frame, fns.mpcX(), fns.mpcY());
  ctx.fillStyle = '#8fd4ff';
  ctx.beginPath();
  ctx.arc(pT.px, pT.py, 6, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = '#ffb88f';
  ctx.beginPath();
  ctx.arc(pM.px, pM.py, 6, 0, Math.PI * 2);
  ctx.fill();

  metricsEl.textContent =
    `scenario S${fns.scenarioId()}\n` +
    `teb_time=${fns.tebT().toFixed(2)}s path=${fns.tebLen().toFixed(3)}m success=${fns.tebOk()}\n` +
    `mpc_time=${fns.mpcT().toFixed(2)}s path=${fns.mpcLen().toFixed(3)}m success=${fns.mpcOk()}`;
}

function bind() {
  fns = {
    reset: wrap('tml_reset', 'number'),
    setScenario: wrap('tml_set_scenario', 'number', ['number']),
    stepTicks: wrap('tml_step_ticks', 'number', ['number']),
    scenarioId: wrap('tml_scenario_id', 'number'),
    tebX: wrap('tml_teb_x', 'number'),
    tebY: wrap('tml_teb_y', 'number'),
    mpcX: wrap('tml_mpc_x', 'number'),
    mpcY: wrap('tml_mpc_y', 'number'),
    tebT: wrap('tml_teb_sim_time', 'number'),
    mpcT: wrap('tml_mpc_sim_time', 'number'),
    tebOk: wrap('tml_teb_success', 'number'),
    mpcOk: wrap('tml_mpc_success', 'number'),
    tebLen: wrap('tml_teb_path_len', 'number'),
    mpcLen: wrap('tml_mpc_path_len', 'number'),
    originX: wrap('tml_map_origin_x', 'number'),
    originY: wrap('tml_map_origin_y', 'number'),
    resolution: wrap('tml_map_resolution', 'number'),
    mapW: wrap('tml_map_width', 'number'),
    mapH: wrap('tml_map_height', 'number'),
    goalX: wrap('tml_goal_x', 'number'),
    goalY: wrap('tml_goal_y', 'number'),
    nRects: wrap('tml_n_rects', 'number'),
    rectX0: wrap('tml_rect_x0', 'number', ['number']),
    rectY0: wrap('tml_rect_y0', 'number', ['number']),
    rectX1: wrap('tml_rect_x1', 'number', ['number']),
    rectY1: wrap('tml_rect_y1', 'number', ['number']),
    nMovers: wrap('tml_n_movers', 'number'),
    moverX: wrap('tml_mover_x', 'number', ['number']),
    moverY: wrap('tml_mover_y', 'number', ['number']),
    moverR: wrap('tml_mover_radius', 'number', ['number']),
    moverVx: wrap('tml_mover_vx', 'number', ['number']),
    moverVy: wrap('tml_mover_vy', 'number', ['number']),
    occCount: wrap('tml_occ_count', 'number'),
    occI: wrap('tml_occ_i', 'number', ['number']),
    occJ: wrap('tml_occ_j', 'number', ['number']),
    tebTraceN: wrap('tml_teb_trace_count', 'number'),
    tebTraceX: wrap('tml_teb_trace_x', 'number', ['number']),
    tebTraceY: wrap('tml_teb_trace_y', 'number', ['number']),
    mpcTraceN: wrap('tml_mpc_trace_count', 'number'),
    mpcTraceX: wrap('tml_mpc_trace_x', 'number', ['number']),
    mpcTraceY: wrap('tml_mpc_trace_y', 'number', ['number']),
  };
}

function mapFrameInfo() {
  const frame = currentFrame();
  const goal = worldToCanvas(frame, fns.goalX(), fns.goalY());
  const start = worldToCanvas(frame, fns.tebX(), fns.tebY());
  return {
    originX: fns.originX(),
    originY: fns.originY(),
    resolution: fns.resolution(),
    widthCells: fns.mapW(),
    heightCells: fns.mapH(),
    goal: { x: fns.goalX(), y: fns.goalY(), px: goal.px, py: goal.py },
    start: { px: start.px, py: start.py },
    canvas: { width: canvas.width, height: canvas.height },
    tebTime: fns.tebT(),
    mpcTime: fns.mpcT(),
    tebSuccess: fns.tebOk(),
    mpcSuccess: fns.mpcOk(),
  };
}

async function init() {
  type Factory = (cfg?: { locateFile?: (path: string) => string; noInitialRun?: boolean }) => Promise<TmlModule>;
  const factory = (globalThis as { createTmlModule?: Factory }).createTmlModule;
  if (!factory) {
    throw new Error('createTmlModule not loaded');
  }
  mod = await factory({
    locateFile: (p: string) => `${import.meta.env.BASE_URL}${p}`,
    noInitialRun: true,
  });
  bind();
  fns.reset();
  fns.setScenario(1);
  drawScene();
  document.body.dataset.simReady = '1';
  const g = globalThis as {
    __stepTicks?: (n: number) => string;
    __mapFrame?: () => ReturnType<typeof mapFrameInfo>;
  };
  g.__stepTicks = (n: number) => {
    fns.stepTicks(n);
    drawScene();
    return metricsEl.textContent ?? '';
  };
  g.__mapFrame = () => mapFrameInfo();
  scenarioSel.addEventListener('change', () => {
    fns.setScenario(Number(scenarioSel.value));
    playing = false;
    drawScene();
  });
  document.getElementById('reset')?.addEventListener('click', () => {
    fns.setScenario(Number(scenarioSel.value));
    playing = false;
    drawScene();
  });
  document.getElementById('step')?.addEventListener('click', () => {
    fns.stepTicks(20);
    drawScene();
  });
  document.getElementById('play')?.addEventListener('click', () => {
    playing = !playing;
  });
  const loop = () => {
    if (playing) {
      fns.stepTicks(2);
      drawScene();
    }
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);
}

init().catch((err) => {
  if (metricsEl) metricsEl.textContent = `WASM load failed: ${err}`;
});
