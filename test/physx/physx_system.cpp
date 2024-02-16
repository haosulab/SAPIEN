#include "sapien/entity.h"
#include "sapien/physx/physx.h"
#include "sapien/scene.h"
#include <gtest/gtest.h>

using namespace sapien;
using namespace sapien::physx;

TEST(Engine, Creation) {
  // NOTE: it also fails if other tests fail to release PhysxEngine
  {
    auto engine = PhysxEngine::Get(1.f, 2.f);
    auto scale = engine->getPxPhysics()->getTolerancesScale();
    ASSERT_FLOAT_EQ(scale.length, 1.f);
    ASSERT_FLOAT_EQ(scale.speed, 2.f);
  }

  {
    auto engine = PhysxEngine::Get(0.1f, 0.2f);
    auto scale = engine->getPxPhysics()->getTolerancesScale();
    ASSERT_FLOAT_EQ(scale.length, 0.1f);
    ASSERT_FLOAT_EQ(scale.speed, 0.2f);
  }
}

TEST(PhysxSystemCpu, Creation) {
  auto scene = std::make_shared<Scene>();
  scene->addSystem(std::make_shared<PhysxSystemCpu>());

  auto mat = std::make_shared<PhysxMaterial>(0.3, 0.3, 0.1);
  auto shape = std::make_shared<PhysxCollisionShapePlane>(mat);
  shape->setLocalPose({{0.f, 0.f, 0.f}, {0.7071068, 0, -0.7071068, 0}});

  auto staticComponent = std::make_shared<PhysxRigidStaticComponent>();
  staticComponent->attachCollision(std::move(shape));

  auto entity = std::make_shared<Entity>();
  entity->addComponent(staticComponent);
  entity->setName("ground");

  scene->addEntity(entity);
}
