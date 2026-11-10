# SPDX-License-Identifier: Apache-2.0
add_library(tml_core
  src/core/math.cpp
  src/core/plant.cpp
  src/core/kinematics.cpp
  src/core/costmap.cpp
  src/core/geometry.cpp
  src/core/geometry_query.cpp
  src/core/reference.cpp
  src/core/teb.cpp
  src/core/metrics.cpp
  src/core/scenario.cpp
)
target_include_directories(tml_core PUBLIC ${CMAKE_SOURCE_DIR}/cpp/include)
target_link_libraries(tml_core PUBLIC nlohmann_json::nlohmann_json)

add_library(tml_osqp src/osqp/mpc_solver.cpp)
target_include_directories(tml_osqp PUBLIC ${CMAKE_SOURCE_DIR}/cpp/include)
target_link_libraries(tml_osqp PUBLIC tml_core osqpstatic)

add_library(tml_sim
  src/sim/runner.cpp
  src/sim/wasm_exports.cpp
)
target_include_directories(tml_sim PUBLIC ${CMAKE_SOURCE_DIR}/cpp/include)
target_link_libraries(tml_sim PUBLIC tml_core tml_osqp nlohmann_json::nlohmann_json)

if(TML_BUILD_TESTS)
  add_subdirectory(test)
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
  add_executable(tml_wasm src/sim/wasm_main.cpp)
  target_link_libraries(tml_wasm PRIVATE tml_sim tml_core tml_osqp osqpstatic)
  target_link_options(tml_wasm PRIVATE
    "SHELL:-sMODULARIZE=1"
    "SHELL:-sEXPORT_NAME=createTmlModule"
    "SHELL:-sENVIRONMENT=web"
    "SHELL:-sALLOW_MEMORY_GROWTH=1"
    "SHELL:-sSTACK_SIZE=5MB"
    "SHELL:-sFILESYSTEM=1"
    "SHELL:-sEXPORTED_FUNCTIONS=['_main','_tml_reset','_tml_set_scenario','_tml_load_scenario_json','_tml_step_ticks','_tml_teb_x','_tml_teb_y','_tml_mpc_x','_tml_mpc_y','_tml_sim_time','_tml_scenario_id','_tml_teb_success','_tml_mpc_success','_tml_teb_path_len','_tml_mpc_path_len','_tml_teb_time_to_goal','_tml_mpc_time_to_goal']"
    "SHELL:-sEXPORTED_RUNTIME_METHODS=ccall,cwrap"
    "SHELL:--preload-file=${CMAKE_SOURCE_DIR}/fixtures@/fixtures"
  )
endif()
