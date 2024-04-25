#pragma once
#include "sapien/math/bounding_box.h"
#include "sapien/math/pose.h"
#include <Eigen/Eigen>
#include <PxPhysicsAPI.h>
#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "material.h"
#include "mesh.h"

namespace sapien {
namespace physx {

class PhysxEngine;
class PhysxMaterial;
class PhysxConvexMesh;
class PhysxTriangleMesh;
class PhysxRigidBaseComponent;

class PhysxCollisionShape {
public:
  PhysxCollisionShape() {}
  inline ::physx::PxShape *getPxShape() const { return mPxShape; }

  void setCollisionGroups(std::array<uint32_t, 4> groups);
  std::array<uint32_t, 4> getCollisionGroups() const;

  void setRestOffset(float offset);
  float getRestOffset() const;

  void setContactOffset(float offset);
  float getContactOffset() const;

  void setTorsionalPatchRadius(float radius);
  float getTorsionalPatchRadius() const;

  void setMinTorsionalPatchRadius(float radius);
  float getMinTorsionalPatchRadius() const;

  Pose getLocalPose() const;
  void setLocalPose(Pose const &pose);

  void setIsTrigger(bool trigger);
  bool isTrigger() const;

  void setIsSceneQuery(bool query);
  bool isSceneQuery() const;

  void setPhysicalMaterial(std::shared_ptr<PhysxMaterial> material);
  std::shared_ptr<PhysxMaterial> getPhysicalMaterial() const;

  void setDensity(float density) {
    if (mPxShape->getActor()) {
      throw std::runtime_error(
          "failed to set density: density may not be changed once the shape is attached");
    }
    mDensity = density;
  }
  float getDensity() const { return mDensity; }

  void internalSetParent(PhysxRigidBaseComponent *parent) { mParent = parent; };
  std::shared_ptr<PhysxRigidBaseComponent> getParent() const;

  virtual AABB getLocalAABB() const = 0;
  virtual AABB getGlobalAABBFast() const;
  virtual AABB computeGlobalAABBTight() const;

  PhysxCollisionShape(PhysxCollisionShape const &) = delete;
  PhysxCollisionShape(PhysxCollisionShape &&) = default;
  PhysxCollisionShape &operator=(PhysxCollisionShape const &) = delete;
  PhysxCollisionShape &operator=(PhysxCollisionShape &&) = default;

  virtual ~PhysxCollisionShape();

  virtual std::shared_ptr<PhysxCollisionShape> clone() const = 0;

protected:
  std::shared_ptr<PhysxEngine> mEngine{};
  std::shared_ptr<PhysxMaterial> mPhysicalMaterial{};
  ::physx::PxShape *mPxShape{};
  float mDensity{1000.f};

  // TODO: unattach shape and clear parent
  PhysxRigidBaseComponent *mParent{};

  void copyProperties(PhysxCollisionShape &target) const;

  void setDefaultProperties();
};

class PhysxCollisionShapePlane : public PhysxCollisionShape {
public:
  PhysxCollisionShapePlane(std::shared_ptr<PhysxMaterial> material = nullptr);
  AABB getLocalAABB() const override;

  std::shared_ptr<PhysxCollisionShape> clone() const override;
};

class PhysxCollisionShapeBox : public PhysxCollisionShape {
public:
  PhysxCollisionShapeBox(Vec3 halfLengths, std::shared_ptr<PhysxMaterial> material = nullptr);
  Vec3 getHalfLengths() const;
  AABB getLocalAABB() const override;

  std::shared_ptr<PhysxCollisionShape> clone() const override;
};

class PhysxCollisionShapeCapsule : public PhysxCollisionShape {
public:
  PhysxCollisionShapeCapsule(float radius, float halfLength,
                             std::shared_ptr<PhysxMaterial> material = nullptr);
  float getRadius() const;
  float getHalfLength() const;
  AABB getLocalAABB() const override;

  std::shared_ptr<PhysxCollisionShape> clone() const override;
};

class PhysxCollisionShapeCylinder : public PhysxCollisionShape {
public:
  PhysxCollisionShapeCylinder(float radius, float halfLength,
                              std::shared_ptr<PhysxMaterial> material = nullptr);
  float getRadius() const;
  float getHalfLength() const;
  AABB getLocalAABB() const override;

  std::shared_ptr<PhysxCollisionShape> clone() const override;

private:
  std::shared_ptr<PhysxConvexMesh> mMesh;
  float mRadius;
  float mHalfLength;
};

class PhysxCollisionShapeSphere : public PhysxCollisionShape {
public:
  PhysxCollisionShapeSphere(float radius, std::shared_ptr<PhysxMaterial> material = nullptr);
  float getRadius() const;
  AABB getLocalAABB() const override;

  std::shared_ptr<PhysxCollisionShape> clone() const override;

  // TODO: implement this properly
  // AABB getGlobalAABBFast() const override;
};

class PhysxCollisionShapeConvexMesh : public PhysxCollisionShape {
public:
  static std::vector<std::shared_ptr<PhysxCollisionShapeConvexMesh>>
  LoadMultiple(std::string const &filename, Vec3 scale,
               std::shared_ptr<PhysxMaterial> material = nullptr);

  PhysxCollisionShapeConvexMesh(std::string const &filename, Vec3 const &scale,
                                std::shared_ptr<PhysxMaterial>);
  PhysxCollisionShapeConvexMesh(
      Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &vertices, Vec3 const &scale,
      std::shared_ptr<PhysxMaterial> material = nullptr);

  // internal use only
  PhysxCollisionShapeConvexMesh(std::shared_ptr<PhysxConvexMesh> mesh, Vec3 const &scale,
                                std::shared_ptr<PhysxMaterial> material = nullptr);
  // PhysxCollisionShapeConvexMesh(PhysxCollisionShapeConvexMesh const &other);

  Vec3 getScale() const;
  Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> getVertices() const;
  Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> getTriangles() const;

  std::shared_ptr<PhysxConvexMesh> getMesh() const { return mMesh; };
  AABB getLocalAABB() const override;
  AABB computeGlobalAABBTight() const override;

  std::shared_ptr<PhysxCollisionShape> clone() const override;

private:
  std::shared_ptr<PhysxConvexMesh> mMesh;
  AABB mLocalAABB;
};

class PhysxCollisionShapeTriangleMesh : public PhysxCollisionShape {
public:
  // TODO: support rotation?
  PhysxCollisionShapeTriangleMesh(std::string const &filename, Vec3 const &scale,
                                  std::shared_ptr<PhysxMaterial> material = nullptr);
  PhysxCollisionShapeTriangleMesh(
      Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &vertices,
      Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> const &triangles,
      Vec3 const &scale, std::shared_ptr<PhysxMaterial> material = nullptr);

  // internal use only
  PhysxCollisionShapeTriangleMesh(std::shared_ptr<PhysxTriangleMesh> mesh, Vec3 const &scale,
                                  std::shared_ptr<PhysxMaterial> material = nullptr);

  Vec3 getScale() const;
  Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> getVertices() const;
  Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> getTriangles() const;

  std::shared_ptr<PhysxTriangleMesh> getMesh() const { return mMesh; };
  AABB getLocalAABB() const override;
  AABB computeGlobalAABBTight() const override;

  std::shared_ptr<PhysxCollisionShape> clone() const override;

private:
  std::shared_ptr<PhysxTriangleMesh> mMesh;
  AABB mLocalAABB;
};

} // namespace physx
} // namespace sapien
