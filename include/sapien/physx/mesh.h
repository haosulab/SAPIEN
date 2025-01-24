/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once
#include "sapien/math/bounding_box.h"
#include <Eigen/Eigen>
#include <PxPhysicsAPI.h>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sapien {
namespace physx {
class PhysxEngine;
class PhysxSDFShapeConfig;

using Vertices = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;
using Triangles = Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor>;

class PhysxConvexMesh {
public:
  PhysxConvexMesh(Vertices const &vertices);
  PhysxConvexMesh(Vertices const &vertices, std::string const &filename);
  PhysxConvexMesh(Vertices const &vertices, std::string const &filename, int part);
  PhysxConvexMesh(std::string const &filename);

  static std::vector<std::shared_ptr<PhysxConvexMesh>>
  LoadByConnectedParts(std::string const &filename);

  static std::shared_ptr<PhysxConvexMesh> CreateCylinder();

  ::physx::PxConvexMesh *getPxMesh() const { return mMesh; }
  bool hasFilename() { return mFilename.has_value(); }
  std::string getFilename() {
    if (hasFilename()) {
      return mFilename.value();
    }
    throw std::runtime_error("no filename is associated with the mesh");
  }
  bool hasPart() { return mPart.has_value(); }
  int getPart() {
    if (hasPart()) {
      return mPart.value();
    }
    throw std::runtime_error("no part is associated with the mesh");
  }

  Vertices getVertices() const;
  Triangles getTriangles() const;
  AABB const &getAABB() const { return mAABB; }

  ~PhysxConvexMesh() {
    if (mMesh) {
      mMesh->release();
    }
  }

private:
  void loadMesh(Vertices const &vertices);
  std::shared_ptr<PhysxEngine> mEngine;
  ::physx::PxConvexMesh *mMesh{};
  std::optional<std::string> mFilename;
  std::optional<int> mPart;

  AABB mAABB;

  PhysxConvexMesh() {}
};

class PhysxTriangleMesh {
public:
  PhysxTriangleMesh(Vertices const &vertices, Triangles const &triangles, bool generateSDF);
  PhysxTriangleMesh(Vertices const &vertices, Triangles const &triangles,
                    std::string const &filename, bool generateSDF);
  PhysxTriangleMesh(std::string const &filename, bool generateSDF);

  ::physx::PxTriangleMesh *getPxMesh() const { return mMesh; }
  bool hasFilename() { return mFilename.has_value(); }
  std::string getFilename() {
    if (hasFilename()) {
      return mFilename.value();
    }
    throw std::runtime_error("no filename is associated with the mesh");
  }

  Vertices getVertices() const;
  Triangles getTriangles() const;
  AABB const &getAABB() const { return mAABB; }

  bool getSDFEnabled() const { return mSDF; }
  float getSDFSpacing() const { return mSDFSpacing; }
  uint32_t getSDFSubgridSize() const { return mSDFSubgridSize; }

  ~PhysxTriangleMesh() {
    if (mMesh) {
      mMesh->release();
    }
  }

private:
  void loadMesh(Vertices const &vertices, Triangles const &triangles, bool generateSDF);
  std::shared_ptr<PhysxEngine> mEngine;
  ::physx::PxTriangleMesh *mMesh{};
  std::optional<std::string> mFilename;

  bool mSDF{};
  float mSDFSpacing{};
  uint32_t mSDFSubgridSize;

  AABB mAABB;
  PhysxTriangleMesh() {}
};

} // namespace physx
} // namespace sapien
