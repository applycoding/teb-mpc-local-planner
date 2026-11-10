# SPDX-License-Identifier: Apache-2.0
# Override OSQP's mutable qdldl tag with hashed archive (PLAN.md pin).
include(FetchContent)
FetchContent_Declare(
  qdldl
  URL https://github.com/osqp/qdldl/archive/refs/tags/v0.1.8.tar.gz
  URL_HASH SHA256=ecf113fd6ad8714f16289eb4d5f4d8b27842b6775b978c39def5913f983f6daa
)
