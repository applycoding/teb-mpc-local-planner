# SPDX-License-Identifier: Apache-2.0
if(NOT TML_BUILD_TESTS)
  return()
endif()
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG 063de7e9578f82b369302001269680b4b1553359
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
