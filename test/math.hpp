#pragma once
#include "sapien/math/math.h"
#include <gtest/gtest.h>

#define EXPECT_VEC3_EQ(a, b)                                                                      \
  {                                                                                               \
    EXPECT_EQ((a).x, (b).x);                                                                      \
    EXPECT_EQ((a).y, (b).y);                                                                      \
    EXPECT_FLOAT_EQ((a).z, (b).z);                                                                \
  }

#define EXPECT_QUAT_EQ(a, b)                                                                      \
  {                                                                                               \
    auto __result = (a) * (b);                                                                    \
    __result.normalize();                                                                         \
    EXPECT_FLOAT_EQ(std::abs(__result.w), 1.f);                                                   \
  }

#define EXPECT_POSE_EQ(a, b)                                                                      \
  EXPECT_VEC3_EQ((a).p, (b).p);                                                                   \
  EXPECT_QUAT_EQ((a).q, (b).q);
