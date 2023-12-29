#include "sapien/physx/physx.h"
#include <gtest/gtest.h>

using namespace sapien;
using namespace sapien::physx;

TEST(PhysxMaterial, Create) {
  auto mat = std::make_shared<PhysxMaterial>(0.25, 0.2, 0.1);
  ASSERT_TRUE(mat->getPxMaterial());
  EXPECT_FLOAT_EQ(mat->getStaticFriction(), 0.25);
  EXPECT_FLOAT_EQ(mat->getDynamicFriction(), 0.2);
  EXPECT_FLOAT_EQ(mat->getRestitution(), 0.1);
  EXPECT_FLOAT_EQ(mat->getPxMaterial()->getStaticFriction(), 0.25);
  EXPECT_FLOAT_EQ(mat->getPxMaterial()->getDynamicFriction(), 0.2);
  EXPECT_FLOAT_EQ(mat->getPxMaterial()->getRestitution(), 0.1);
}

TEST(PhysxMaterial, Set) {
  auto mat = std::make_shared<PhysxMaterial>(0.25, 0.2, 0.1);
  mat->setStaticFriction(0.35);
  mat->setDynamicFriction(0.3);
  mat->setRestitution(0.2);
  ASSERT_TRUE(mat->getPxMaterial());
  EXPECT_FLOAT_EQ(mat->getStaticFriction(), 0.35);
  EXPECT_FLOAT_EQ(mat->getDynamicFriction(), 0.3);
  EXPECT_FLOAT_EQ(mat->getRestitution(), 0.2);
  EXPECT_FLOAT_EQ(mat->getPxMaterial()->getStaticFriction(), 0.35);
  EXPECT_FLOAT_EQ(mat->getPxMaterial()->getDynamicFriction(), 0.3);
  EXPECT_FLOAT_EQ(mat->getPxMaterial()->getRestitution(), 0.2);
}
