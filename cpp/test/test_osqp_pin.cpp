// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <osqp.h>

#include <cstring>

TEST(OsqpPin, VersionAndIntSize) {
  EXPECT_STREQ(osqp_version(), "1.0.0");
  EXPECT_EQ(sizeof(OSQPInt), sizeof(int));
}
