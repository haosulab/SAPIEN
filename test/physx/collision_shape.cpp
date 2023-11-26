#include "math.hpp"
#include "sapien/physx/physx.h"
#include "serialize.hpp"
#include <filesystem>
#include <gtest/gtest.h>

using namespace sapien;
using namespace sapien::physx;

TEST(PhysxCollisionShapeBox, Default) {
  auto shape1 = std::make_shared<PhysxCollisionShapeBox>(Vec3{1.5, 2.5, 3.5});
  EXPECT_EQ(shape1->getPhysicalMaterial(), PhysxDefault::Get().getDefaultMaterial());

  PhysxDefault::Get().setDefaultMaterial(0, 0, 0);
  auto shape2 = std::make_shared<PhysxCollisionShapeBox>(Vec3{1.5, 2.5, 3.5});
  EXPECT_EQ(shape2->getPhysicalMaterial(), PhysxDefault::Get().getDefaultMaterial());
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
    auto meshfile = std::filesystem::path(__FILE__).parent_path().parent_path() /
                    "assets" / "cube.obj";
    auto mat = std::make_shared<PhysxMaterial>(0.2, 0.25, 0.1);
    auto shape = std::make_shared<PhysxCollisionShapeConvexMesh>(meshfile.string(),
                                                                 Vec3(0.5, 0.4, 0.3), mat);
    auto mesh = MeshManager::Get()->loadConvexMesh(meshfile.string());

    ASSERT_TRUE(shape->getPxShape());
    EXPECT_EQ(shape->getMesh(), mesh);
    EXPECT_VEC3_EQ(shape->getScale(), Vec3(0.5, 0.4, 0.3));
  }

  {
    auto meshfile = std::filesystem::path(__FILE__).parent_path().parent_path() /
                    "assets" / "doublecube.obj";
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
  auto meshfile = std::filesystem::path(__FILE__).parent_path().parent_path() /
                  "assets" / "cube.obj";

  auto mat = std::make_shared<PhysxMaterial>(0.2, 0.25, 0.1);
  auto shape = std::make_shared<PhysxCollisionShapeTriangleMesh>(meshfile.string(),
                                                                 Vec3(0.5, 0.4, 0.3), mat);
  auto mesh = MeshManager::Get()->loadTriangleMesh(meshfile.string());

  ASSERT_TRUE(shape->getPxShape());
  EXPECT_EQ(shape->getMesh(), mesh);
  EXPECT_VEC3_EQ(shape->getScale(), Vec3(0.5, 0.4, 0.3));

  MeshManager::Clear();
}

TEST(PhysxCollisionShapeBox, Serialize) {
  auto mat = std::make_shared<PhysxMaterial>(0.2, 0.25, 0.1);
  auto shape = std::make_shared<PhysxCollisionShapeBox>(Vec3{1.5, 2.5, 3.5}, mat);

  std::array<uint32_t, 4> groups{2, 3, 4, 5};
  shape->setCollisionGroups(groups);
  shape->setContactOffset(0.2);
  shape->setRestOffset(0.1);
  shape->setDensity(500.f);
  shape->setLocalPose(Pose({1, 2, 3}, {0, 1, 0, 0}));
  shape->setTorsionalPatchRadius(0.2);
  shape->setMinTorsionalPatchRadius(0.1);

  // polymorphism
  std::shared_ptr<PhysxCollisionShape> baseShape = shape;
  auto shape2 = std::dynamic_pointer_cast<PhysxCollisionShapeBox>(saveload(baseShape));

  ASSERT_TRUE(shape2);
  ASSERT_TRUE(shape2->getPxShape());
  EXPECT_VEC3_EQ(shape2->getHalfLengths(), Vec3(1.5, 2.5, 3.5));

  EXPECT_EQ(shape2->getCollisionGroups(), groups);
  EXPECT_FLOAT_EQ(shape2->getRestOffset(), 0.1);
  EXPECT_FLOAT_EQ(shape2->getContactOffset(), 0.2);
  EXPECT_FLOAT_EQ(shape2->getDensity(), 500.f);
  EXPECT_POSE_EQ(shape2->getLocalPose(), Pose({1, 2, 3}, {0, 1, 0, 0}));
  EXPECT_FLOAT_EQ(shape2->getTorsionalPatchRadius(), 0.2);
  EXPECT_FLOAT_EQ(shape2->getMinTorsionalPatchRadius(), 0.1);
}

TEST(PhysxCollisionShapePlane, Serialize) {
  auto mat = std::make_shared<PhysxMaterial>(0.2, 0.25, 0.1);
  auto shape = saveload(std::make_shared<PhysxCollisionShapePlane>(mat));

  ASSERT_TRUE(shape->getPxShape());
}

TEST(PhysxCollisionShapeSphere, Serialize) {
  auto mat = std::make_shared<PhysxMaterial>(0.2, 0.25, 0.1);
  auto shape = saveload(std::make_shared<PhysxCollisionShapeSphere>(0.5, mat));

  ASSERT_TRUE(shape->getPxShape());
  EXPECT_FLOAT_EQ(shape->getRadius(), 0.5);
}

TEST(PhysxCollisionShapeCapsule, Serialize) {
  auto mat = std::make_shared<PhysxMaterial>(0.2, 0.25, 0.1);
  auto shape = saveload(std::make_shared<PhysxCollisionShapeCapsule>(0.5, 1.5, mat));

  ASSERT_TRUE(shape->getPxShape());
  EXPECT_FLOAT_EQ(shape->getRadius(), 0.5);
  EXPECT_FLOAT_EQ(shape->getHalfLength(), 1.5);
}

TEST(PhysxCollisionShapeConvexMesh, Serialize) {
  auto meshfile = std::filesystem::path(__FILE__).parent_path().parent_path() /
                  "assets" / "cube.obj";
  auto mat = std::make_shared<PhysxMaterial>(0.2, 0.25, 0.1);
  auto shape =
      std::make_shared<PhysxCollisionShapeConvexMesh>(meshfile.string(), Vec3(0.5, 0.4, 0.3), mat);

  auto shape2 = saveload(shape);
  ASSERT_TRUE(shape2->getPxShape());
  EXPECT_VEC3_EQ(shape2->getScale(), Vec3(0.5, 0.4, 0.3));
  auto mesh = shape->getMesh();
  auto vs = mesh->getVertices();
  auto ts = mesh->getTriangles();

  ASSERT_EQ(vs.rows(), 8);
  ASSERT_EQ(vs.cols(), 3);
  ASSERT_EQ(ts.rows(), 12);
  ASSERT_EQ(ts.cols(), 3);

  MeshManager::Clear();
}

TEST(PhysxCollisionShapeTriangleMesh, Serialize) {
  auto meshfile = std::filesystem::path(__FILE__).parent_path().parent_path() /
                  "assets" / "cube.obj";

  auto mat = std::make_shared<PhysxMaterial>(0.2, 0.25, 0.1);
  auto shape = std::make_shared<PhysxCollisionShapeTriangleMesh>(meshfile.string(),
                                                                 Vec3(0.5, 0.4, 0.3), mat);
  auto shape2 = saveload(shape);

  ASSERT_TRUE(shape->getPxShape());
  EXPECT_VEC3_EQ(shape->getScale(), Vec3(0.5, 0.4, 0.3));

  auto mesh = shape->getMesh();
  auto vs = mesh->getVertices();
  auto ts = mesh->getTriangles();
  ASSERT_EQ(vs.rows(), 8);
  ASSERT_EQ(vs.cols(), 3);
  ASSERT_EQ(ts.rows(), 12);
  ASSERT_EQ(ts.cols(), 3);

  MeshManager::Clear();
}

TEST(PhysxCollisionShapeTriangleMesh, SerializePolymorphism) {
  auto meshfile = std::filesystem::path(__FILE__).parent_path().parent_path() /
                  "assets" / "cube.obj";
  auto mat = std::make_shared<PhysxMaterial>(0.2, 0.25, 0.1);
  auto shape = std::make_shared<PhysxCollisionShapeSphere>(0.5, mat);

  std::shared_ptr<PhysxCollisionShape> shape0 =
      saveload(std::make_shared<PhysxCollisionShapePlane>(mat));
  std::shared_ptr<PhysxCollisionShape> shape1 =
      std::make_shared<PhysxCollisionShapeBox>(Vec3{1.5, 2.5, 3.5}, mat);
  std::shared_ptr<PhysxCollisionShape> shape2 =
      std::make_shared<PhysxCollisionShapeSphere>(0.5, mat);
  std::shared_ptr<PhysxCollisionShape> shape3 =
      std::make_shared<PhysxCollisionShapeCapsule>(0.5, 1.5, mat);
  std::shared_ptr<PhysxCollisionShape> shape4 =
      std::make_shared<PhysxCollisionShapeConvexMesh>(meshfile.string(), Vec3(0.5, 0.4, 0.3), mat);
  std::shared_ptr<PhysxCollisionShape> shape5 = std::make_shared<PhysxCollisionShapeTriangleMesh>(
      meshfile.string(), Vec3(0.5, 0.4, 0.3), mat);

  shape0 = saveload(shape0);
  shape1 = saveload(shape1);
  shape2 = saveload(shape2);
  shape3 = saveload(shape3);
  shape4 = saveload(shape4);
  shape5 = saveload(shape5);

  EXPECT_TRUE(std::dynamic_pointer_cast<PhysxCollisionShapePlane>(shape0));
  EXPECT_TRUE(std::dynamic_pointer_cast<PhysxCollisionShapeBox>(shape1));
  EXPECT_TRUE(std::dynamic_pointer_cast<PhysxCollisionShapeSphere>(shape2));
  EXPECT_TRUE(std::dynamic_pointer_cast<PhysxCollisionShapeCapsule>(shape3));
  EXPECT_TRUE(std::dynamic_pointer_cast<PhysxCollisionShapeConvexMesh>(shape4));
  EXPECT_TRUE(std::dynamic_pointer_cast<PhysxCollisionShapeTriangleMesh>(shape5));
}
