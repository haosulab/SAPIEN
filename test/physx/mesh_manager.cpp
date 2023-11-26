#include "sapien/physx/mesh_manager.h"
#include "sapien/physx/physx_system.h"
#include <algorithm>
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

TEST(PhysxTriangleMesh, LoadConvex) {
  auto vs = CubeVertices();
  auto meshfile = std::filesystem::path(__FILE__).parent_path().parent_path() /
                  "assets" / "cube.obj";
  ASSERT_TRUE(std::filesystem::is_regular_file(meshfile));

  auto mesh = MeshManager::Get()->loadConvexMesh(meshfile.string());
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

  MeshManager::Clear();
}

TEST(PhysxTriangleMesh, LoadTriangle) {
  auto vs = CubeVertices();
  auto meshfile = std::filesystem::path(__FILE__).parent_path().parent_path() /
                  "assets" / "cube.obj";
  ASSERT_TRUE(std::filesystem::is_regular_file(meshfile));

  auto mesh = MeshManager::Get()->loadTriangleMesh(meshfile.string());
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

  MeshManager::Clear();
}

TEST(PhysxTriangleMesh, Filename) {
  {
    auto meshfile = std::filesystem::path(__FILE__).parent_path().parent_path() /
                    "assets" / "cube.obj";
    auto rfile = std::filesystem::relative(meshfile);
    ASSERT_NE(meshfile.string(), rfile.string());

    auto convex = MeshManager::Get()->loadConvexMesh(rfile.string());
    auto meshes = MeshManager::Get()->loadConvexMeshGroup(rfile.string());
    auto triangle = MeshManager::Get()->loadTriangleMesh(rfile.string());

    EXPECT_EQ(convex->getFilename(), std::filesystem::canonical(meshfile).string());
    EXPECT_EQ(triangle->getFilename(), std::filesystem::canonical(meshfile).string());
  }

  {
    auto meshfile = std::filesystem::path(__FILE__).parent_path().parent_path() /
                    "assets" / "doublecube.obj";
    auto rfile = std::filesystem::relative(meshfile);
    ASSERT_NE(meshfile.string(), rfile.string());

    auto meshes = MeshManager::Get()->loadConvexMeshGroup(rfile.string());

    EXPECT_EQ(meshes.size(), 2);
    for (auto m : meshes) {
      EXPECT_EQ(m->getFilename(), std::filesystem::canonical(meshfile).string());
    }
  }

  MeshManager::Clear();
}

TEST(PhysxTriangleMesh, Cache) {
  {
    auto meshfile = std::filesystem::path(__FILE__).parent_path().parent_path() /
                    "assets" / "cube.obj";
    auto c1 = MeshManager::Get()->loadConvexMesh(meshfile.string());
    auto c2 = MeshManager::Get()->loadConvexMesh(meshfile.string());
    EXPECT_EQ(c1, c2);

    auto t1 = MeshManager::Get()->loadTriangleMesh(meshfile.string());
    auto t2 = MeshManager::Get()->loadTriangleMesh(meshfile.string());
    EXPECT_EQ(t1, t2);
  }

  {
    auto meshfile = std::filesystem::path(__FILE__).parent_path().parent_path() /
                    "assets" / "doublecube.obj";
    auto c1 = MeshManager::Get()->loadConvexMeshGroup(meshfile.string());
    auto c2 = MeshManager::Get()->loadConvexMeshGroup(meshfile.string());
    EXPECT_EQ(c1, c2);
  }

  MeshManager::Clear();
}
