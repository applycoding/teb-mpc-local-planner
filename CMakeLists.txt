# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(teb-mpc-local-planner VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(TML_BUILD_TESTS "Build GoogleTest suite" ON)
option(TML_BUILD_PYTHON "Build pybind11 bindings" OFF)
option(TML_BUILD_ROS "Build ROS 2 Humble adapter" OFF)

if(TML_BUILD_PYTHON)
  message(FATAL_ERROR "TML_BUILD_PYTHON is not implemented in v1")
endif()
if(TML_BUILD_ROS)
  add_subdirectory(ros2/tml_planner)
endif()

include(cmake/FetchOSQP.cmake)
include(cmake/FetchGTest.cmake)
include(cmake/FetchJson.cmake)

enable_testing()
add_subdirectory(cpp)
