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

#include <cereal/archives/binary.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/optional.hpp>
#include <cereal/types/polymorphic.hpp>

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

  // serialization
  template <class Archive> void saveBase(Archive &ar) const {
    ar(getCollisionGroups(), getRestOffset(), getContactOffset(), getTorsionalPatchRadius(),
       getMinTorsionalPatchRadius(), getLocalPose(), isTrigger(), isSceneQuery(), mDensity);
  }
  template <class Archive> void loadBase(Archive &ar) {
    if (!mPxShape) {
      throw std::runtime_error("loading PhysxCollisionShape base class is not allowed.");
    }
    std::array<uint32_t, 4> groups;
    float restOffset, contactOffset, patchRadius, minPatchRadius;
    Pose localPose;
    bool trigger, sceneQuery;
    ar(groups, restOffset, contactOffset, patchRadius, minPatchRadius, localPose, trigger,
       sceneQuery, mDensity);

    setCollisionGroups(groups);
    setContactOffset(contactOffset);
    setRestOffset(restOffset);
    setTorsionalPatchRadius(patchRadius);
    setMinTorsionalPatchRadius(minPatchRadius);
    setLocalPose(localPose);
    setIsTrigger(trigger);
    setIsSceneQuery(sceneQuery);
  }

  template <class Archive> void save(Archive &archive) const {
    throw std::runtime_error("cereal workaround. should never be called.");
  }
  template <class Archive> void load(Archive &archive) {
    throw std::runtime_error("cereal workaround. should never be called.");
  }

  virtual ~PhysxCollisionShape();

protected:
  std::shared_ptr<PhysxEngine> mEngine{};
  std::shared_ptr<PhysxMaterial> mPhysicalMaterial{};
  ::physx::PxShape *mPxShape{};
  float mDensity{1000.f};

  // TODO: unattach shape and clear parent
  PhysxRigidBaseComponent *mParent{};
};

class PhysxCollisionShapePlane : public PhysxCollisionShape {
public:
  PhysxCollisionShapePlane(std::shared_ptr<PhysxMaterial> material = nullptr);
  AABB getLocalAABB() const override;

  template <class Archive> void save(Archive &ar) const {
    ar(mPhysicalMaterial);
    PhysxCollisionShape::saveBase(ar);
  }
  template <class Archive>
  static void load_and_construct(Archive &ar,
                                 cereal::construct<PhysxCollisionShapePlane> &construct) {
    std::shared_ptr<PhysxMaterial> m;
    ar(m);
    construct(m);
    construct->PhysxCollisionShape::loadBase(ar);
  }
};

class PhysxCollisionShapeBox : public PhysxCollisionShape {
public:
  PhysxCollisionShapeBox(Vec3 halfLengths, std::shared_ptr<PhysxMaterial> material = nullptr);
  Vec3 getHalfLengths() const;
  AABB getLocalAABB() const override;

  template <class Archive> void save(Archive &ar) const {
    ar(mPhysicalMaterial, getHalfLengths());
    PhysxCollisionShape::saveBase(ar);
  }

  template <class Archive>
  static void load_and_construct(Archive &ar,
                                 cereal::construct<PhysxCollisionShapeBox> &construct) {
    Vec3 l;
    std::shared_ptr<PhysxMaterial> m;
    ar(m, l);
    construct(l, m);
    construct->PhysxCollisionShape::loadBase(ar);
  }
};

class PhysxCollisionShapeCapsule : public PhysxCollisionShape {
public:
  PhysxCollisionShapeCapsule(float radius, float halfLength,
                             std::shared_ptr<PhysxMaterial> material = nullptr);
  float getRadius() const;
  float getHalfLength() const;
  AABB getLocalAABB() const override;

  template <class Archive> void save(Archive &ar) const {
    ar(mPhysicalMaterial, getRadius(), getHalfLength());
    PhysxCollisionShape::saveBase(ar);
  }
  template <class Archive>
  static void load_and_construct(Archive &ar,
                                 cereal::construct<PhysxCollisionShapeCapsule> &construct) {
    float r, l;
    std::shared_ptr<PhysxMaterial> m;
    ar(m, r, l);
    construct(r, l, m);
    construct->PhysxCollisionShape::loadBase(ar);
  }
};

class PhysxCollisionShapeCylinder : public PhysxCollisionShape {
public:
  PhysxCollisionShapeCylinder(float radius, float halfLength,
                              std::shared_ptr<PhysxMaterial> material = nullptr);
  float getRadius() const;
  float getHalfLength() const;
  AABB getLocalAABB() const override;

  template <class Archive> void save(Archive &ar) const {
    ar(mPhysicalMaterial, getRadius(), getHalfLength());
    PhysxCollisionShape::saveBase(ar);
  }
  template <class Archive>
  static void load_and_construct(Archive &ar,
                                 cereal::construct<PhysxCollisionShapeCylinder> &construct) {
    float r, l;
    std::shared_ptr<PhysxMaterial> m;
    ar(m, r, l);
    construct(r, l, m);
    construct->PhysxCollisionShape::loadBase(ar);
  }

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

  // TODO: implement this properly
  // AABB getGlobalAABBFast() const override;

  template <class Archive> void save(Archive &ar) const {
    ar(mPhysicalMaterial, getRadius());
    PhysxCollisionShape::saveBase(ar);
  }
  template <class Archive>
  static void load_and_construct(Archive &ar,
                                 cereal::construct<PhysxCollisionShapeSphere> &construct) {
    float r;
    std::shared_ptr<PhysxMaterial> m;
    ar(m, r);
    construct(r, m);
    construct->PhysxCollisionShape::loadBase(ar);
  }
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
  Vec3 getScale() const;
  Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> getVertices() const;
  Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> getTriangles() const;

  std::shared_ptr<PhysxConvexMesh> getMesh() const { return mMesh; };
  AABB getLocalAABB() const override;
  AABB computeGlobalAABBTight() const override;

  // TODO: serialize AABB
  template <class Archive> void save(Archive &ar) const {
    ar(mPhysicalMaterial, getScale(), mMesh);
    PhysxCollisionShape::saveBase(ar);
  }
  template <class Archive>
  static void load_and_construct(Archive &ar,
                                 cereal::construct<PhysxCollisionShapeConvexMesh> &construct) {
    std::shared_ptr<PhysxConvexMesh> mesh;
    Vec3 scale;
    std::shared_ptr<PhysxMaterial> m;
    ar(m, scale, mesh);
    construct(mesh, scale, m);
    construct->PhysxCollisionShape::loadBase(ar);
  }

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

  // TODO: serialize AABB
  template <class Archive> void save(Archive &ar) const {
    ar(mPhysicalMaterial, getScale(), mMesh);
    PhysxCollisionShape::saveBase(ar);
  }
  template <class Archive>
  static void load_and_construct(Archive &ar,
                                 cereal::construct<PhysxCollisionShapeTriangleMesh> &construct) {
    std::shared_ptr<PhysxTriangleMesh> mesh;
    Vec3 scale;
    std::shared_ptr<PhysxMaterial> m;
    ar(m, scale, mesh);
    construct(mesh, scale, m);
    construct->PhysxCollisionShape::loadBase(ar);
  }

private:
  std::shared_ptr<PhysxTriangleMesh> mMesh;
  AABB mLocalAABB;
};

} // namespace physx
} // namespace sapien

CEREAL_REGISTER_TYPE(sapien::physx::PhysxCollisionShapePlane);
CEREAL_REGISTER_TYPE(sapien::physx::PhysxCollisionShapeBox);
CEREAL_REGISTER_TYPE(sapien::physx::PhysxCollisionShapeSphere);
CEREAL_REGISTER_TYPE(sapien::physx::PhysxCollisionShapeCapsule);
CEREAL_REGISTER_TYPE(sapien::physx::PhysxCollisionShapeCylinder);
CEREAL_REGISTER_TYPE(sapien::physx::PhysxCollisionShapeConvexMesh);
CEREAL_REGISTER_TYPE(sapien::physx::PhysxCollisionShapeTriangleMesh);

CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::physx::PhysxCollisionShape,
                                     sapien::physx::PhysxCollisionShapePlane);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::physx::PhysxCollisionShape,
                                     sapien::physx::PhysxCollisionShapeBox);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::physx::PhysxCollisionShape,
                                     sapien::physx::PhysxCollisionShapeSphere);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::physx::PhysxCollisionShape,
                                     sapien::physx::PhysxCollisionShapeCapsule);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::physx::PhysxCollisionShape,
                                     sapien::physx::PhysxCollisionShapeCylinder);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::physx::PhysxCollisionShape,
                                     sapien::physx::PhysxCollisionShapeConvexMesh);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::physx::PhysxCollisionShape,
                                     sapien::physx::PhysxCollisionShapeTriangleMesh);
