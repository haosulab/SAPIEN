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
  void afterStep();

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
  int getIndex() { return mIndex; }
  std::shared_ptr<PhysxArticulationJoint> getJoint() const { return mJoint; }

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
  void internalSetIndex(int index) { mIndex = index; }
  ~PhysxArticulationLinkComponent();

  std::vector<uint64_t> getSerializationDependencies() const override {
    if (mParent) {
      return {mParent->mId};
    }
    return {};
  }

private:
  friend cereal::access;
  PhysxArticulationLinkComponent() {}
  template <class Archive> void save(Archive &ar) const;
  template <class Archive> void load(Archive &ar);

  std::shared_ptr<PhysxArticulationLinkComponent> mParent;
  std::vector<std::weak_ptr<PhysxArticulationLinkComponent>> mChildren;

  std::shared_ptr<PhysxArticulation> mArticulation;
  ::physx::PxArticulationLink *mPxLink{};

  int mIndex{-1};
  std::shared_ptr<PhysxArticulationJoint>
      mJoint; // cannot use unique_ptr for pybind11 compatibility
};

template <class Archive> void PhysxArticulationLinkComponent::save(Archive &ar) const {
  ar(mParent, mArticulation, mIndex);

  if (!mParent) {
    mArticulation->getRootLinearVelocity();
    mArticulation->getRootAngularVelocity();
    mArticulation->getRootPose();
    int jointType = mJoint->getType();

    ar(jointType, mArticulation->getRootLinearVelocity(), mArticulation->getRootAngularVelocity(),
       mArticulation->getRootPose());

  } else {
    Pose parentPose = mJoint->getAnchorPoseInParent();
    Pose childPose = mJoint->getAnchorPoseInChild();
    int jointType = mJoint->getType();
    float jointFriction = mJoint->getFriction();
    int jointMotion[6]{};
    float jointLimit[6][2]{};
    float jointDrive[6][3]{};
    int jointDriveType[6]{};
    float jointDriveTarget[6]{};
    float jointDriveVelocity[6]{};
    float jointPosition[6]{};
    float jointVelocity[6]{};
    float jointArmature[6]{};

    auto pxjoint = getPxActor()->getInboundJoint();
    std::vector<::physx::PxArticulationAxis::Enum> axes = {
        ::physx::PxArticulationAxis::eTWIST,  ::physx::PxArticulationAxis::eSWING1,
        ::physx::PxArticulationAxis::eSWING2, ::physx::PxArticulationAxis::eX,
        ::physx::PxArticulationAxis::eY,      ::physx::PxArticulationAxis::eZ,
    };
    for (int i = 0; i < 6; ++i) {
      jointMotion[i] = pxjoint->getMotion(axes[i]);

      auto limit = pxjoint->getLimitParams(axes[i]);
      jointLimit[i][0] = limit.low;
      jointLimit[i][1] = limit.high;

      auto drive = pxjoint->getDriveParams(axes[i]);
      jointDrive[i][0] = drive.stiffness;
      jointDrive[i][1] = drive.damping;
      jointDrive[i][2] = drive.maxForce;
      jointDriveType[i] = drive.driveType;

      jointDriveTarget[i] = pxjoint->getDriveTarget(axes[i]);
      jointDriveVelocity[i] = pxjoint->getDriveVelocity(axes[i]);
      jointPosition[i] = pxjoint->getJointPosition(axes[i]);
      jointVelocity[i] = pxjoint->getJointVelocity(axes[i]);
      jointArmature[i] = pxjoint->getArmature(axes[i]);
    }
    ar(parentPose, childPose, jointType, jointFriction, jointMotion, jointLimit, jointDrive,
       jointDriveType, jointDriveTarget, jointDriveVelocity, jointPosition, jointVelocity,
       jointArmature);
  }
  ar(cereal::base_class<PhysxRigidBodyComponent>(this));
}

template <class Archive> void PhysxArticulationLinkComponent::load(Archive &ar) {
  ar(mParent, mArticulation, mIndex);

  auto sthis = std::static_pointer_cast<PhysxArticulationLinkComponent>(shared_from_this());
  mArticulation->internalAddLinkAtIndex(*this, mParent.get(), mIndex);
  mJoint = std::make_shared<PhysxArticulationJoint>(sthis);
  if (mParent) {
    mParent->mChildren.push_back(sthis);
  }

  if (!mParent) {
    int jointType;
    Vec3 linearVelocity;
    Vec3 angularVelocity;
    Pose pose;

    ar(jointType, linearVelocity, angularVelocity, pose);
    mJoint->setType(::physx::PxArticulationJointType::Enum(jointType));
    mArticulation->setRootLinearVelocity(linearVelocity);
    mArticulation->setRootAngularVelocity(angularVelocity);
    mArticulation->setRootPose(pose);
  } else {
    Pose parentPose = mJoint->getAnchorPoseInParent();
    Pose childPose = mJoint->getAnchorPoseInChild();
    int jointType;
    float jointFriction;
    int jointMotion[6]{};
    float jointLimit[6][2]{};
    float jointDrive[6][3]{};
    int jointDriveType[6]{};
    float jointDriveTarget[6]{};
    float jointDriveVelocity[6]{};
    float jointPosition[6]{};
    float jointVelocity[6]{};
    float jointArmature[6]{};

    ar(parentPose, childPose, jointType, jointFriction, jointMotion, jointLimit, jointDrive,
       jointDriveType, jointDriveTarget, jointDriveVelocity, jointPosition, jointVelocity,
       jointArmature);

    mJoint->setType(::physx::PxArticulationJointType::Enum(jointType));
    mJoint->setFriction(jointFriction);
    mJoint->setAnchorPoseInParent(parentPose);
    mJoint->setAnchorPoseInChild(childPose);

    auto pxjoint = getPxActor()->getInboundJoint();
    std::vector<::physx::PxArticulationAxis::Enum> axes = {
        ::physx::PxArticulationAxis::eTWIST,  ::physx::PxArticulationAxis::eSWING1,
        ::physx::PxArticulationAxis::eSWING2, ::physx::PxArticulationAxis::eX,
        ::physx::PxArticulationAxis::eY,      ::physx::PxArticulationAxis::eZ,
    };
    for (int i = 0; i < 6; ++i) {
      pxjoint->setMotion(axes[i], ::physx::PxArticulationMotion::Enum(jointMotion[i]));
      pxjoint->setLimitParams(axes[i], {jointLimit[i][0], jointLimit[i][1]});
      pxjoint->setDriveParams(
          axes[i],
          ::physx::PxArticulationDrive(jointDrive[i][0], jointDrive[i][1], jointDrive[i][2],
                                       ::physx::PxArticulationDriveType::Enum(jointDriveType[i])));
      pxjoint->setDriveTarget(axes[i], jointDriveTarget[i]);
      pxjoint->setDriveVelocity(axes[i], jointDriveVelocity[i]);
      pxjoint->setJointPosition(axes[i], jointPosition[i]);
      pxjoint->setJointVelocity(axes[i], jointVelocity[i]);
      pxjoint->setArmature(axes[i], jointArmature[i]);
    }
  }
  ar(cereal::base_class<PhysxRigidBodyComponent>(this));
}

} // namespace physx
} // namespace sapien

CEREAL_REGISTER_TYPE(sapien::physx::PhysxArticulationLinkComponent);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::physx::PhysxRigidBodyComponent,
                                     sapien::physx::PhysxArticulationLinkComponent);
