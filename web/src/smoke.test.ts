// SPDX-License-Identifier: Apache-2.0
import { describe, expect, it } from 'vitest';
import viteConfig from '../vite.config';

describe('demo shell', () => {
  it('has the Pages base path in the Vite config', () => {
    expect(viteConfig.base).toBe('/teb-mpc-local-planner/');
  });
});
