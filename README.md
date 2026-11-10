<!-- SPDX-License-Identifier: Apache-2.0 -->
# teb-mpc-local-planner

Portfolio demonstration (**simulation only**). A differential-drive **kinematic teaching model** (not a dynamic vehicle, not a real Nav2 controller) is driven through scripted scenes while two local planners are compared:

- **TEB-style** single-band timed-elastic optimizer (teaching code, **not** [teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner)).
- **MPC (OSQP)** — linearized kinematic receding horizon with [OSQP v1.0.0](https://github.com/osqp/osqp); **not** MPPI and **not** a `nav2_core::Controller` plugin.

See `PLAN.md`, `REFERENCES.md`, and `DEVIATIONS.md`.

## Build (C++17)

```bash
cmake -S . -B build -DCMAKE_CXX_COMPILER=g++-13
cmake --build build
ctest --test-dir build
python3 tools/check_no_heap.py
```

## License

Apache-2.0. Copyright 2026 Arpan Bom. See `LICENSE` and `NOTICE`.

Author: **Arpan Bom** <applycoding@gmail.com>
