#include "sapien/math/vec3.h"
#include <gtest/gtest.h>
using namespace sapien;

TEST(Math, Vec3) {
  Vec3 a(1.f, 2.f, 3.f);
  EXPECT_NEAR(a.x, 1.f, 1e-6);
  EXPECT_NEAR(a.y, 2.f, 1e-6);
  EXPECT_NEAR(a.z, 3.f, 1e-6);

  Vec3 b;
  EXPECT_NEAR(b.x, 0.f, 1e-6);
  EXPECT_NEAR(b.y, 0.f, 1e-6);
  EXPECT_NEAR(b.z, 0.f, 1e-6);

  Vec3 c(4.f);
  EXPECT_NEAR(c.x, 4.f, 1e-6);
  EXPECT_NEAR(c.y, 4.f, 1e-6);
  EXPECT_NEAR(c.z, 4.f, 1e-6);
}

TEST(Math, Vec3Ops) {
  Vec3 a(1.f, 2.f, 3.f);
  Vec3 b(4.f, 7.f, 10.f);

  {
    Vec3 c = -a;
    EXPECT_NEAR(c.x, -1.f, 1e-6);
    EXPECT_NEAR(c.y, -2.f, 1e-6);
    EXPECT_NEAR(c.z, -3.f, 1e-6);
  }

  // vector +-*/
  {
    Vec3 c = a + b;
    EXPECT_NEAR(c.x, 5.f, 1e-6);
    EXPECT_NEAR(c.y, 9.f, 1e-6);
    EXPECT_NEAR(c.z, 13.f, 1e-6);
  }
  {
    Vec3 c = a - b;
    EXPECT_NEAR(c.x, -3.f, 1e-6);
    EXPECT_NEAR(c.y, -5.f, 1e-6);
    EXPECT_NEAR(c.z, -7.f, 1e-6);
  }
  {
    Vec3 c = a * b;
    EXPECT_NEAR(c.x, 4.f, 1e-6);
    EXPECT_NEAR(c.y, 14.f, 1e-6);
    EXPECT_NEAR(c.z, 30.f, 1e-6);
  }
  {
    Vec3 c = a / b;
    EXPECT_NEAR(c.x, 1.f / 4.f, 1e-6);
    EXPECT_NEAR(c.y, 2.f / 7.f, 1e-6);
    EXPECT_NEAR(c.z, 3.f / 10.f, 1e-6);
  }

  // scalar +-*/
  {
    Vec3 c = a + 2.f;
    EXPECT_NEAR(c.x, 3.f, 1e-6);
    EXPECT_NEAR(c.y, 4.f, 1e-6);
    EXPECT_NEAR(c.z, 5.f, 1e-6);
  }
  {
    Vec3 c = a - 2.f;
    EXPECT_NEAR(c.x, -1.f, 1e-6);
    EXPECT_NEAR(c.y, 0.f, 1e-6);
    EXPECT_NEAR(c.z, 1.f, 1e-6);
  }
  {
    Vec3 c = a * 2.f;
    EXPECT_NEAR(c.x, 2.f, 1e-6);
    EXPECT_NEAR(c.y, 4.f, 1e-6);
    EXPECT_NEAR(c.z, 6.f, 1e-6);
  }
  {
    Vec3 c = a / 2.f;
    EXPECT_NEAR(c.x, 0.5f, 1e-6);
    EXPECT_NEAR(c.y, 1.f, 1e-6);
    EXPECT_NEAR(c.z, 1.5f, 1e-6);
  }

  // TODO: inplace ops

  // dot, cross
  {
    float c = a.dot(b);
    EXPECT_NEAR(c, 48.f, 1e-6);
  }
  {
    Vec3 c = a.cross(b);
    EXPECT_NEAR(c.x, -1.f, 1e-6);
    EXPECT_NEAR(c.y, 2.f, 1e-6);
    EXPECT_NEAR(c.z, -1.f, 1e-6);
  }

  // TODO
}
