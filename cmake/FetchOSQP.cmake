# SPDX-License-Identifier: Apache-2.0
include(FetchContent)

set(OSQP_URL "https://github.com/osqp/osqp/releases/download/v1.0.0/osqp-v1.0.0-src.tar.gz")
set(OSQP_URL_HASH "SHA256=ec0bb8fd34625d0ea44274ab3e991aa56e3e360ba30935ae62476557b101c646")
set(QDLDL_URL "https://github.com/osqp/qdldl/archive/refs/tags/v0.1.8.tar.gz")
set(QDLDL_URL_HASH "SHA256=ecf113fd6ad8714f16289eb4d5f4d8b27842b6775b978c39def5913f983f6daa")

file(READ "${CMAKE_SOURCE_DIR}/cmake/pins.sha256" _pins_txt)
if(NOT _pins_txt MATCHES "ec0bb8fd34625d0ea44274ab3e991aa56e3e360ba30935ae62476557b101c646")
  message(FATAL_ERROR "cmake/pins.sha256 OSQP hash mismatch")
endif()
if(NOT _pins_txt MATCHES "ecf113fd6ad8714f16289eb4d5f4d8b27842b6775b978c39def5913f983f6daa")
  message(FATAL_ERROR "cmake/pins.sha256 QDLDL hash mismatch")
endif()

set(OSQP_ALGEBRA_BACKEND "builtin" CACHE STRING "" FORCE)
set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
set(OSQP_USE_FLOAT OFF CACHE BOOL "" FORCE)
set(OSQP_USE_LONG OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_PRINTING OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_INTERRUPT OFF CACHE BOOL "" FORCE)
set(OSQP_VERSION "1.0.0" CACHE STRING "" FORCE)
set(QDLDL_LONG OFF CACHE BOOL "" FORCE)

set(OSQP_CUSTOM_MODULE "${CMAKE_CURRENT_LIST_DIR}/OverrideQDLDL.cmake" CACHE FILEPATH "" FORCE)

FetchContent_Declare(
  osqp
  URL ${OSQP_URL}
  URL_HASH ${OSQP_URL_HASH}
)
FetchContent_MakeAvailable(osqp)

if(NOT TARGET osqpstatic AND TARGET osqp)
  add_library(osqpstatic ALIAS osqp)
endif()

# OSQPInt size is asserted in cpp/test/test_osqp_pin.cpp after headers are available.
