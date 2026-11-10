#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EMSDK_DIR="${ROOT}/.emsdk"
EMSDK_VERSION="7.0.0"
EMSDK_MANAGER_SHA="1693ce7359fc9694f198694f902a2cc061239b86"

if [[ ! -d "${EMSDK_DIR}" ]]; then
  git clone https://github.com/emscripten-core/emsdk.git "${EMSDK_DIR}"
  git -C "${EMSDK_DIR}" checkout "${EMSDK_MANAGER_SHA}"
fi

"${EMSDK_DIR}/emsdk" install "${EMSDK_VERSION}"
"${EMSDK_DIR}/emsdk" activate "${EMSDK_VERSION}"
# shellcheck disable=SC1091
source "${EMSDK_DIR}/emsdk_env.sh"

rm -rf "${ROOT}/build-wasm"
emcmake cmake -S "${ROOT}" -B "${ROOT}/build-wasm" \
  -DCMAKE_BUILD_TYPE=Release \
  -DTML_BUILD_TESTS=OFF \
  -DOSQP_ALGEBRA_BACKEND=builtin \
  -DOSQP_USE_LONG=OFF
cmake --build "${ROOT}/build-wasm" -j"$(nproc)" --target tml_wasm

mkdir -p "${ROOT}/web/public"
cp -f "${ROOT}/build-wasm/cpp/tml_wasm.js" "${ROOT}/web/public/"
cp -f "${ROOT}/build-wasm/cpp/tml_wasm.wasm" "${ROOT}/web/public/"
if [[ -f "${ROOT}/build-wasm/cpp/tml_wasm.data" ]]; then
  cp -f "${ROOT}/build-wasm/cpp/tml_wasm.data" "${ROOT}/web/public/"
fi
