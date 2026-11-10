# SPDX-License-Identifier: Apache-2.0
include(FetchContent)
FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG 55f93686c01528224f448c19128836e7df245f72
)
FetchContent_MakeAvailable(json)
