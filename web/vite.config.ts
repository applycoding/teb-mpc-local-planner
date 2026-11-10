// SPDX-License-Identifier: Apache-2.0
import { defineConfig } from 'vite';

export default defineConfig({
  base: '/teb-mpc-local-planner/',
  build: {
    outDir: 'dist',
  },
  test: {
    include: ['src/**/*.test.ts'],
    exclude: ['**/node_modules/**', '**/emsdk/**', '**/e2e/**', '**/dist/**'],
  },
});
