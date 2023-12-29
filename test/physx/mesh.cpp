#include "sapien/physx/mesh.h"
#include <filesystem>
#include <gtest/gtest.h>

using namespace sapien;
using namespace sapien::physx;

static auto CubeVertices() {
  Vertices vs;
  vs.resize(8, 3);
  vs.row(0) << 0.5, 1.5, -1.0;
  vs.row(1) << 0.5, -1.5, -1.0;
  vs.row(2) << 0.5, 1.5, 1.0;
  vs.row(3) << 0.5, -1.5, 1.0;
  vs.row(4) << -0.5, 1.5, -1.0;
  vs.row(5) << -0.5, -1.5, -1.0;
  vs.row(6) << -0.5, 1.5, 1.0;
  vs.row(7) << -0.5, -1.5, 1.0;
  return vs;
}

static auto Cube() {
  Vertices vs;
  vs.resize(8, 3);
  vs.row(0) << 0.5, 1.5, -1.0;
  vs.row(1) << 0.5, -1.5, -1.0;
  vs.row(2) << 0.5, 1.5, 1.0;
  vs.row(3) << 0.5, -1.5, 1.0;
  vs.row(4) << -0.5, 1.5, -1.0;
  vs.row(5) << -0.5, -1.5, -1.0;
  vs.row(6) << -0.5, 1.5, 1.0;
  vs.row(7) << -0.5, -1.5, 1.0;

  Triangles ts;
  ts.resize(12, 3);
  ts.row(0) << 0, 4, 6;
  ts.row(1) << 0, 6, 2;
  ts.row(2) << 3, 2, 6;
  ts.row(3) << 3, 6, 7;
  ts.row(4) << 7, 6, 4;
  ts.row(5) << 7, 4, 5;
  ts.row(6) << 5, 1, 3;
  ts.row(7) << 5, 3, 7;
  ts.row(8) << 1, 0, 2;
  ts.row(9) << 1, 2, 3;
  ts.row(10) << 5, 4, 0;
  ts.row(11) << 5, 0, 1;

  return std::tuple{vs, ts};
}

TEST(PhysxConvexMesh, All) {
  auto vs = CubeVertices();
  {
    auto mesh = std::make_shared<PhysxConvexMesh>(vs);

    EXPECT_TRUE(mesh->getPxMesh());
    EXPECT_EQ(mesh->getPxMesh()->getReferenceCount(), 1);
    EXPECT_FALSE(mesh->hasFilename());
    EXPECT_THROW(mesh->getFilename(), std::runtime_error);
    EXPECT_FALSE(mesh->hasPart());
    EXPECT_THROW(mesh->getPart(), std::runtime_error);

    auto vs2 = mesh->getVertices();
    EXPECT_EQ(vs2.rows(), 8);
    EXPECT_EQ(vs2.cols(), 3);
    for (int i = 0; i < vs.rows(); ++i) {
      float m = 1e5;
      for (int j = 0; j < vs2.rows(); ++j) {
        m = std::min((vs.row(i) - vs2.row(j)).norm(), m);
      }
      EXPECT_LE(m, 1e-5);
    }

    auto ts = mesh->getTriangles();
    EXPECT_EQ(ts.rows(), 12);
    EXPECT_EQ(ts.cols(), 3);
  }

  {
    auto mesh = std::make_shared<PhysxConvexMesh>(vs, "cube.obj");

    EXPECT_TRUE(mesh->getPxMesh());
    EXPECT_EQ(mesh->getPxMesh()->getReferenceCount(), 1);
    EXPECT_TRUE(mesh->hasFilename());
    EXPECT_EQ(mesh->getFilename(), "cube.obj");
    EXPECT_FALSE(mesh->hasPart());
    EXPECT_THROW(mesh->getPart(), std::runtime_error);

    auto vs2 = mesh->getVertices();
    EXPECT_EQ(vs2.rows(), 8);
    EXPECT_EQ(vs2.cols(), 3);
    for (int i = 0; i < vs.rows(); ++i) {
      float m = 1e5;
      for (int j = 0; j < vs2.rows(); ++j) {
        m = std::min((vs.row(i) - vs2.row(j)).norm(), m);
      }
      EXPECT_LE(m, 1e-5);
    }

    auto ts = mesh->getTriangles();
    EXPECT_EQ(ts.rows(), 12);
    EXPECT_EQ(ts.cols(), 3);
  }

  {
    auto mesh = std::make_shared<PhysxConvexMesh>(vs, "cube.obj", 2);

    EXPECT_TRUE(mesh->getPxMesh());
    EXPECT_EQ(mesh->getPxMesh()->getReferenceCount(), 1);
    EXPECT_TRUE(mesh->hasFilename());
    EXPECT_EQ(mesh->getFilename(), "cube.obj");
    EXPECT_TRUE(mesh->hasPart());
    EXPECT_EQ(mesh->getPart(), 2);

    auto vs2 = mesh->getVertices();
    EXPECT_EQ(vs2.rows(), 8);
    EXPECT_EQ(vs2.cols(), 3);
    for (int i = 0; i < vs.rows(); ++i) {
      float m = 1e5;
      for (int j = 0; j < vs2.rows(); ++j) {
        m = std::min((vs.row(i) - vs2.row(j)).norm(), m);
      }
      EXPECT_LE(m, 1e-5);
    }

    auto ts = mesh->getTriangles();
    EXPECT_EQ(ts.rows(), 12);
    EXPECT_EQ(ts.cols(), 3);
  }

  {
    auto meshfile =
        std::filesystem::path(__FILE__).parent_path().parent_path() / "assets" / "cube.obj";
    ASSERT_TRUE(std::filesystem::is_regular_file(meshfile));

    auto mesh = std::make_shared<PhysxConvexMesh>(meshfile.string());

    EXPECT_TRUE(mesh->getPxMesh());
    EXPECT_EQ(mesh->getPxMesh()->getReferenceCount(), 1);
    EXPECT_TRUE(mesh->hasFilename());
    EXPECT_EQ(mesh->getFilename(), meshfile.string());
    EXPECT_FALSE(mesh->hasPart());
    EXPECT_THROW(mesh->getPart(), std::runtime_error);

    auto vs2 = mesh->getVertices();
    EXPECT_EQ(vs2.rows(), 8);
    EXPECT_EQ(vs2.cols(), 3);
    for (int i = 0; i < vs.rows(); ++i) {
      float m = 1e5;
      for (int j = 0; j < vs2.rows(); ++j) {
        m = std::min((vs.row(i) - vs2.row(j)).norm(), m);
      }
      EXPECT_LE(m, 1e-5);
    }

    auto ts = mesh->getTriangles();
    EXPECT_EQ(ts.rows(), 12);
    EXPECT_EQ(ts.cols(), 3);
  }

  {
    auto meshfile =
        std::filesystem::path(__FILE__).parent_path().parent_path() / "assets" / "doublecube.obj";
    ASSERT_TRUE(std::filesystem::is_regular_file(meshfile));

    auto meshes = PhysxConvexMesh::LoadByConnectedParts(meshfile.string());

    EXPECT_EQ(meshes.size(), 2);
    {
      EXPECT_TRUE(meshes[0]->getPxMesh());
      EXPECT_EQ(meshes[0]->getPxMesh()->getReferenceCount(), 1);
      EXPECT_TRUE(meshes[0]->hasFilename());
      EXPECT_EQ(meshes[0]->getFilename(), meshfile.string());
      EXPECT_TRUE(meshes[0]->hasPart());
      EXPECT_EQ(meshes[0]->getPart(), 0);

      auto vs2 = meshes[0]->getVertices();
      EXPECT_EQ(vs2.rows(), 8);
      EXPECT_EQ(vs2.cols(), 3);

      auto ts = meshes[0]->getTriangles();
      EXPECT_EQ(ts.rows(), 12);
      EXPECT_EQ(ts.cols(), 3);
    }

    {
      EXPECT_TRUE(meshes[1]->getPxMesh());
      EXPECT_EQ(meshes[1]->getPxMesh()->getReferenceCount(), 1);
      EXPECT_TRUE(meshes[1]->hasFilename());
      EXPECT_EQ(meshes[1]->getFilename(), meshfile.string());
      EXPECT_TRUE(meshes[1]->hasPart());
      EXPECT_EQ(meshes[1]->getPart(), 1);

      auto vs2 = meshes[1]->getVertices();
      EXPECT_EQ(vs2.rows(), 8);
      EXPECT_EQ(vs2.cols(), 3);

      auto ts = meshes[1]->getTriangles();
      EXPECT_EQ(ts.rows(), 12);
      EXPECT_EQ(ts.cols(), 3);
    }
  }
}

TEST(PhysxTriangleMesh, All) {
  auto [vs, ts] = Cube();

  {
    auto mesh = std::make_shared<PhysxTriangleMesh>(vs, ts);

    EXPECT_TRUE(mesh->getPxMesh());
    EXPECT_EQ(mesh->getPxMesh()->getReferenceCount(), 1);
    EXPECT_FALSE(mesh->hasFilename());
    EXPECT_THROW(mesh->getFilename(), std::runtime_error);

    auto vs2 = mesh->getVertices();
    EXPECT_EQ(vs2.rows(), 8);
    EXPECT_EQ(vs2.cols(), 3);
    for (int i = 0; i < vs.rows(); ++i) {
      float m = 1e5;
      for (int j = 0; j < vs2.rows(); ++j) {
        m = std::min((vs.row(i) - vs2.row(j)).norm(), m);
      }
      EXPECT_LE(m, 1e-5);
    }

    auto ts = mesh->getTriangles();
    EXPECT_EQ(ts.rows(), 12);
    EXPECT_EQ(ts.cols(), 3);
  }

  {
    auto mesh = std::make_shared<PhysxTriangleMesh>(vs, ts, "cube.obj");

    EXPECT_TRUE(mesh->getPxMesh());
    EXPECT_EQ(mesh->getPxMesh()->getReferenceCount(), 1);
    EXPECT_TRUE(mesh->hasFilename());
    EXPECT_EQ(mesh->getFilename(), "cube.obj");

    auto vs2 = mesh->getVertices();
    EXPECT_EQ(vs2.rows(), 8);
    EXPECT_EQ(vs2.cols(), 3);
    for (int i = 0; i < vs.rows(); ++i) {
      float m = 1e5;
      for (int j = 0; j < vs2.rows(); ++j) {
        m = std::min((vs.row(i) - vs2.row(j)).norm(), m);
      }
      EXPECT_LE(m, 1e-5);
    }

    auto ts = mesh->getTriangles();
    EXPECT_EQ(ts.rows(), 12);
    EXPECT_EQ(ts.cols(), 3);
  }

  {
    auto meshfile =
        std::filesystem::path(__FILE__).parent_path().parent_path() / "assets" / "cube.obj";
    ASSERT_TRUE(std::filesystem::is_regular_file(meshfile));

    auto mesh = std::make_shared<PhysxTriangleMesh>(meshfile.string());

    EXPECT_TRUE(mesh->getPxMesh());
    EXPECT_EQ(mesh->getPxMesh()->getReferenceCount(), 1);
    EXPECT_TRUE(mesh->hasFilename());
    EXPECT_EQ(mesh->getFilename(), meshfile.string());

    auto vs2 = mesh->getVertices();
    EXPECT_EQ(vs2.rows(), 8);
    EXPECT_EQ(vs2.cols(), 3);
    for (int i = 0; i < vs.rows(); ++i) {
      float m = 1e5;
      for (int j = 0; j < vs2.rows(); ++j) {
        m = std::min((vs.row(i) - vs2.row(j)).norm(), m);
      }
      EXPECT_LE(m, 1e-5);
    }

    auto ts = mesh->getTriangles();
    EXPECT_EQ(ts.rows(), 12);
    EXPECT_EQ(ts.cols(), 3);
  }
}
