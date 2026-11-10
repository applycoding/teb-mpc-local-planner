// SPDX-License-Identifier: Apache-2.0
/// <reference types="vite/client" />

declare module '../public/tml_wasm.js' {
  type ModuleFactory = (config?: { locateFile?: (path: string) => string }) => Promise<{
    cwrap: (name: string, ret: string | null, args: string[]) => (...a: number[]) => number;
  }>;
  const createTmlModule: ModuleFactory;
  export default createTmlModule;
}
