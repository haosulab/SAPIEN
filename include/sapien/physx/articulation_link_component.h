#pragma once
#include "articulation.h"
#include "base_component.h"
#include "collision_shape.h"
#include "physx_system.h"
#include "rigid_component.h"
#include "sapien/math/conversion.h"
#include "sapien/math/pose.h"
#include <vector>

namespace sapien {
class Entity;
namespace physx {
class PhysxArticulationLinkComponent;

class PhysxArticulationJoint {
public:
  PhysxArticulationJoint(std::weak_ptr<PhysxArticulationLinkComponent> link);

  void setType(::physx::PxArticulationJointType::Enum type);
  ::physx::PxArticulationJointType::Enum getType() const;

  void setAnchorPoseInChild(Pose const &pose);
  Pose getAnchorPoseInChild() const;
  void setAnchorPoseInParent(Pose const &pose);
  Pose getAnchorPoseInParent() const;

  float getFriction() const;
  void setFriction(float friction);

  Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> getLimit() const;
  void setLimit(Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> const &limit);

  Eigen::VectorXf getArmature() const;
  void setArmature(Eigen::VectorXf const &armature);

  void setDriveProperties(float stiffness, float damping, float maxForce,
                          ::physx::PxArticulationDriveType::Enum type);

  float getDriveStiffness() const;
  float getDriveDamping() const;
  float getDriveForceLimit() const;
  ::physx::PxArticulationDriveType::Enum getDriveType() const;

  Eigen::VectorXf getDriveTargetPosition() const;
  void setDriveTargetPosition(Eigen::VectorXf const &position);
  void setDriveTargetPosition(float position);

  Eigen::VectorXf getDriveTargetVelocity() const;
  void setDriveTargetVelocity(Eigen::VectorXf const &velocity);
  void setDriveTargetVelocity(float velocity);

  uint32_t getDof() const;

  std::shared_ptr<PhysxArticulationLinkComponent> getParentLink() const;
  std::shared_ptr<PhysxArticulationLinkComponent> getChildLink() const;

  Pose getGlobalPose() const;

public:
  std::string getName() const { return mName; }
  void setName(std::string const &name) { mName = name; }

private:
  std::string mName;
  ::physx::PxArticulationJointReducedCoordinate *getPxJoint() const;
  std::vector<::physx::PxArticulationAxis::Enum> mAxes;
  std::weak_ptr<PhysxArticulationLinkComponent> mLink;
};

class PhysxArticulationLinkComponent : public PhysxRigidBodyComponent {
public:
  static std::shared_ptr<PhysxArticulationLinkComponent>
  Create(std::shared_ptr<PhysxArticulationLinkComponent> parent = nullptr);

  /** never call it directly */
  PhysxArticulationLinkComponent(std::shared_ptr<PhysxArticulationLinkComponent> parent);

  bool isRoot() const;

  void onAddToScene(Scene &scene) override;
  void onRemoveFromScene(Scene &scene) override;
  void onSetPose(Pose const &pose) override;
  void syncPoseToEntity();

  void setParent(std::shared_ptr<PhysxArticulationLinkComponent> parent);

  std::shared_ptr<PhysxArticulationLinkComponent> getParent() const { return mParent; }
  std::vector<std::shared_ptr<PhysxArticulationLinkComponent>> getChildren() const {
    std::vector<std::shared_ptr<PhysxArticulationLinkComponent>> result;
    for (auto c : mChildren) {
      result.push_back(c.lock());
    }
    return result;
  }

  std::shared_ptr<PhysxArticulation> getArticulation() const { return mArticulation; }
  ::physx::PxArticulationLink *getPxActor() const override { return mPxLink; }
  std::shared_ptr<PhysxArticulationJoint> getJoint() const { return mJoint; }
  int getIndex() const;

  bool isSleeping() const;
  void wakeUp();
  void putToSleep();

  void internalSetPxLink(::physx::PxArticulationLink *link) {
    mPxLink = link;
    if (link) {
      mPxLink->userData = this;
      internalUpdateMass();
    }
  }

  void internalSetGpuPoseIndex(int index) { mGpuPoseIndex = index; }

  // index in all pose array
  int getGpuPoseIndex() const { return mGpuPoseIndex; }

  ~PhysxArticulationLinkComponent();

  static std::vector<std::shared_ptr<PhysxArticulationLinkComponent>>
  cloneArticulation(std::shared_ptr<PhysxArticulationLinkComponent> root);

private:
  PhysxArticulationLinkComponent() {}

  std::shared_ptr<PhysxArticulationLinkComponent> mParent;
  std::vector<std::weak_ptr<PhysxArticulationLinkComponent>> mChildren;

  std::shared_ptr<PhysxArticulation> mArticulation;
  ::physx::PxArticulationLink *mPxLink{};

  std::shared_ptr<PhysxArticulationJoint>
      mJoint; // cannot use unique_ptr for pybind11 compatibility

  int mGpuPoseIndex{-1};
};

} // namespace physx
} // namespace sapien
