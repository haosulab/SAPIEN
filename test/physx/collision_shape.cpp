#include "math.hpp"
#include "sapien/physx/physx.h"
#include <filesystem>
#include <gtest/gtest.h>

using namespace sapien;
using namespace sapien::physx;

TEST(PhysxCollisionShapeBox, Default) {
  auto shape1 = std::make_shared<PhysxCollisionShapeBox>(Vec3{1.5, 2.5, 3.5});
  EXPECT_EQ(shape1->getPhysicalMaterial(), PhysxDefault::GetDefaultMaterial());

  PhysxDefault::SetDefaultMaterial(0, 0, 0);
  auto shape2 = std::make_shared<PhysxCollisionShapeBox>(Vec3{1.5, 2.5, 3.5});
  EXPECT_EQ(shape2->getPhysicalMaterial(), PhysxDefault::GetDefaultMaterial());
  EXPECT_NE(shape1->getPhysicalMaterial(), shape2->getPhysicalMaterial());
}

TEST(PhysxCollisionShapePlane, Create) {
  auto mat = std::make_shared<PhysxMaterial>(0.2, 0.25, 0.1);
  auto shape = std::make_shared<PhysxCollisionShapePlane>(mat);

  ASSERT_TRUE(shape->getPxShape());
}

TEST(PhysxCollisionShapeBox, Create) {
  auto mat = std::make_shared<PhysxMaterial>(0.2, 0.25, 0.1);
  auto shape = std::make_shared<PhysxCollisionShapeBox>(Vec3{1.5, 2.5, 3.5}, mat);

  ASSERT_TRUE(shape->getPxShape());
  EXPECT_VEC3_EQ(shape->getHalfLengths(), Vec3(1.5, 2.5, 3.5));
}

TEST(PhysxCollisionShapeSphere, Create) {
  auto mat = std::make_shared<PhysxMaterial>(0.2, 0.25, 0.1);
  auto shape = std::make_shared<PhysxCollisionShapeSphere>(0.5, mat);

  ASSERT_TRUE(shape->getPxShape());
  EXPECT_FLOAT_EQ(shape->getRadius(), 0.5);
}

TEST(PhysxCollisionShapeCapsule, Create) {
  auto mat = std::make_shared<PhysxMaterial>(0.2, 0.25, 0.1);
  auto shape = std::make_shared<PhysxCollisionShapeCapsule>(0.5, 1.5, mat);

  ASSERT_TRUE(shape->getPxShape());
  EXPECT_FLOAT_EQ(shape->getRadius(), 0.5);
  EXPECT_FLOAT_EQ(shape->getHalfLength(), 1.5);
}

TEST(PhysxCollisionShapeConvexMesh, Create) {
  {
    auto meshfile =
        std::filesystem::path(__FILE__).parent_path().parent_path() / "assets" / "cube.obj";
    auto mat = std::make_shared<PhysxMaterial>(0.2, 0.25, 0.1);
    auto shape = std::make_shared<PhysxCollisionShapeConvexMesh>(meshfile.string(),
                                                                 Vec3(0.5, 0.4, 0.3), mat);
    auto mesh = MeshManager::Get()->loadConvexMesh(meshfile.string());

    ASSERT_TRUE(shape->getPxShape());
    EXPECT_EQ(shape->getMesh(), mesh);
    EXPECT_VEC3_EQ(shape->getScale(), Vec3(0.5, 0.4, 0.3));
  }

  {
    auto meshfile =
        std::filesystem::path(__FILE__).parent_path().parent_path() / "assets" / "doublecube.obj";
    auto mat = std::make_shared<PhysxMaterial>(0.2, 0.25, 0.1);
    auto shapes =
        PhysxCollisionShapeConvexMesh::LoadMultiple(meshfile.string(), Vec3(0.5, 0.4, 0.3), mat);
    auto meshes = MeshManager::Get()->loadConvexMeshGroup(meshfile.string());
    EXPECT_EQ(shapes.size(), meshes.size());
    for (uint32_t i = 0; i < shapes.size(); ++i) {
      EXPECT_EQ(shapes[i]->getMesh(), meshes[i]);
      EXPECT_VEC3_EQ(shapes[i]->getScale(), Vec3(0.5, 0.4, 0.3));
    }
  }

  MeshManager::Clear();
}

TEST(PhysxCollisionShapeTriangleMesh, Create) {
  auto meshfile =
      std::filesystem::path(__FILE__).parent_path().parent_path() / "assets" / "cube.obj";

  auto mat = std::make_shared<PhysxMaterial>(0.2, 0.25, 0.1);
  auto shape = std::make_shared<PhysxCollisionShapeTriangleMesh>(meshfile.string(),
                                                                 Vec3(0.5, 0.4, 0.3), mat);
  auto mesh = MeshManager::Get()->loadTriangleMesh(meshfile.string());

  ASSERT_TRUE(shape->getPxShape());
  EXPECT_EQ(shape->getMesh(), mesh);
  EXPECT_VEC3_EQ(shape->getScale(), Vec3(0.5, 0.4, 0.3));

  MeshManager::Clear();
}
