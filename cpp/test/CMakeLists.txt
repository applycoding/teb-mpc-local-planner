# SPDX-License-Identifier: Apache-2.0
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
list(SORT TEST_SOURCES)
add_executable(tml_tests ${TEST_SOURCES})
target_link_libraries(tml_tests PRIVATE tml_sim tml_core tml_osqp GTest::gtest_main)
target_compile_definitions(tml_tests PRIVATE FIXTURE_DIR="${CMAKE_SOURCE_DIR}/fixtures/scenarios" TML_TESTING)

include(GoogleTest)
gtest_discover_tests(tml_tests WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} DISCOVERY_MODE PRE_TEST)
add_test(NAME tml_tests_manual COMMAND tml_tests WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
