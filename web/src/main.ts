// SPDX-License-Identifier: Apache-2.0
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

const M2P = (x: number, y: number, ox: number, oy: number, res: number) => ({
  px: (x - ox) / res,
  py: canvas.height - (y - oy) / res,
});

function drawScene() {
  if (!ctx || !mod) return;
  const tml_scenario_id = mod.cwrap('tml_scenario_id', 'number', []);
  const ox = -2;
  const oy = -4;
  const res = 0.05;
  ctx.fillStyle = '#102018';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.strokeStyle = '#2f5a47';
  ctx.strokeRect(0, 0, canvas.width, canvas.height);
  const teb_x = mod.cwrap('tml_teb_x', 'number', []);
  const teb_y = mod.cwrap('tml_teb_y', 'number', []);
  const mpc_x = mod.cwrap('tml_mpc_x', 'number', []);
  const mpc_y = mod.cwrap('tml_mpc_y', 'number', []);
  const pT = M2P(teb_x(), teb_y(), ox, oy, res);
  const pM = M2P(mpc_x(), mpc_y(), ox, oy, res);
  ctx.fillStyle = '#8fd4ff';
  ctx.font = '16px sans-serif';
  ctx.fillText('TEB-style', 40, 28);
  ctx.fillStyle = '#ffb88f';
  ctx.fillText('MPC (OSQP)', 200, 28);
  ctx.fillStyle = '#8fd4ff';
  ctx.beginPath();
  ctx.arc(pT.px, pT.py, 6, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = '#ffb88f';
  ctx.beginPath();
  ctx.arc(pM.px, pM.py, 6, 0, Math.PI * 2);
  ctx.fill();
  const sim_t = mod.cwrap('tml_sim_time', 'number', []);
  const teb_len = mod.cwrap('tml_teb_path_len', 'number', []);
  const mpc_len = mod.cwrap('tml_mpc_path_len', 'number', []);
  const teb_ok = mod.cwrap('tml_teb_success', 'number', []);
  const mpc_ok = mod.cwrap('tml_mpc_success', 'number', []);
  metricsEl.textContent =
    `scenario S${tml_scenario_id()}\n` +
    `sim_time=${sim_t().toFixed(2)}s\n` +
    `teb path=${teb_len().toFixed(3)}m success=${teb_ok()}\n` +
    `mpc path=${mpc_len().toFixed(3)}m success=${mpc_ok()}`;
}

async function init() {
  type Factory = (cfg?: { locateFile?: (path: string) => string }) => Promise<TmlModule>;
  const factory = (globalThis as { createTmlModule?: Factory }).createTmlModule;
  if (!factory) {
    throw new Error('createTmlModule not loaded');
  }
  mod = await factory({
    locateFile: (p: string) => `${import.meta.env.BASE_URL}${p}`,
    noInitialRun: true,
  });
  const reset = mod.cwrap('tml_reset', 'number', []);
  const setScenario = mod.cwrap('tml_set_scenario', 'number', ['number']);
  reset();
  setScenario(1);
  const stepTicks = mod.cwrap('tml_step_ticks', 'number', ['number']);
  drawScene();
  document.body.dataset.simReady = '1';
  (globalThis as { __stepTicks?: (n: number) => string }).__stepTicks = (n: number) => {
    stepTicks(n);
    drawScene();
    return metricsEl.textContent ?? '';
  };
  scenarioSel.addEventListener('change', () => {
    setScenario(Number(scenarioSel.value));
    playing = false;
    drawScene();
  });
  document.getElementById('reset')?.addEventListener('click', () => {
    setScenario(Number(scenarioSel.value));
    playing = false;
    drawScene();
  });
  document.getElementById('step')?.addEventListener('click', () => {
    stepTicks(20);
    drawScene();
  });
  document.getElementById('play')?.addEventListener('click', () => {
    playing = !playing;
  });
  const loop = () => {
    if (playing) {
      stepTicks(2);
      drawScene();
    }
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);
}

init().catch((err) => {
  if (metricsEl) metricsEl.textContent = `WASM load failed: ${err}`;
});
