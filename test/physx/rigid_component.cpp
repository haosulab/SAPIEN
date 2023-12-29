#include "math.hpp"
#include "sapien/physx/physx.h"
#include <gtest/gtest.h>

using namespace sapien;
using namespace sapien::physx;

TEST(PhysxRigidDynamicComponent, Create) {
  auto mat = std::make_shared<PhysxMaterial>(0.2, 0.25, 0.1);
  auto shape = std::make_shared<PhysxCollisionShapeBox>(Vec3{1.5, 2.5, 3.5}, mat);

  shape->setDensity(500.f);
  shape->setLocalPose(Pose({1, 0, 0}, {0, 1, 0, 0}));

  auto body = std::make_shared<PhysxRigidDynamicComponent>();
  body->attachCollision(shape);

  body->setLinearVelocity({0, 2, 0});
  body->setAngularVelocity({0, 0, 3});
  body->setDisableGravity(true);
  body->setName("body");

  EXPECT_EQ(body->getName(), "body");
  EXPECT_TRUE(body->getDisableGravity());
  EXPECT_VEC3_EQ(body->getLinearVelocity(), Vec3(0, 2, 0));
  EXPECT_VEC3_EQ(body->getAngularVelocity(), Vec3(0, 0, 3));

  EXPECT_TRUE((body->getCMassLocalPose().p - Vec3(1, 0, 0)).length() < 1e-4);
  EXPECT_NEAR(body->getMass(), 52500, 1);
}
