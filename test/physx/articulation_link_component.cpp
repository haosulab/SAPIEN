#include "math.hpp"
#include "sapien/physx/physx.h"
#include <gtest/gtest.h>

using namespace sapien;
using namespace sapien::physx;

TEST(PhysxArticulationLinkComponent, Create) {
  // root
  auto root = PhysxArticulationLinkComponent::Create();
  EXPECT_EQ(root->getJoint()->getType(), ::physx::PxArticulationJointType::eUNDEFINED);
  EXPECT_FALSE(root->getArticulation()->getPxArticulation()->getArticulationFlags() &
               ::physx::PxArticulationFlag::eFIX_BASE);
  EXPECT_THROW(root->getJoint()->setType(::physx::PxArticulationJointType::eREVOLUTE),
               std::runtime_error);
  root->getJoint()->setType(::physx::PxArticulationJointType::eFIX);
  EXPECT_TRUE(root->getArticulation()->getPxArticulation()->getArticulationFlags() &
              ::physx::PxArticulationFlag::eFIX_BASE);
  EXPECT_EQ(root->getJoint()->getDof(), 0);

  // revolute
  auto l1 = PhysxArticulationLinkComponent::Create(root);
  EXPECT_EQ(root->getArticulation(), l1->getArticulation());

  l1->getJoint()->setType(::physx::PxArticulationJointType::eREVOLUTE);
  l1->getJoint()->setAnchorPoseInParent(Pose({1, 2, 3}, {0, 1, 0, 0}));
  l1->getJoint()->setAnchorPoseInChild(Pose({-1, -2, -3}, {0, 0, 1, 0}));
  l1->getJoint()->setFriction(0.3);
  {
    Eigen::MatrixX2f limit(1, 2);
    limit << -0.5, 1.5;
    l1->getJoint()->setLimit(limit);
    Eigen::VectorXf a(1);
    a << 0.6f;
    l1->getJoint()->setArmature(a);
  }

  l1->getJoint()->setDriveProperties(1000, 100, 10000,
                                     ::physx::PxArticulationDriveType::eACCELERATION);
  l1->getJoint()->setDriveTargetPosition(0.6);
  l1->getJoint()->setDriveTargetVelocity(0.7);

  EXPECT_EQ(l1->getJoint()->getDof(), 1);
  EXPECT_EQ(l1->getJoint()->getType(), ::physx::PxArticulationJointType::eREVOLUTE);
  EXPECT_POSE_EQ(l1->getJoint()->getAnchorPoseInParent(), Pose({1, 2, 3}, {0, 1, 0, 0}));
  EXPECT_POSE_EQ(l1->getJoint()->getAnchorPoseInChild(), Pose({-1, -2, -3}, {0, 0, 1, 0}));
  EXPECT_FLOAT_EQ(l1->getJoint()->getFriction(), 0.3);

  EXPECT_EQ(l1->getJoint()->getLimit().rows(), 1);
  EXPECT_FLOAT_EQ(l1->getJoint()->getLimit()(0, 0), -0.5);
  EXPECT_FLOAT_EQ(l1->getJoint()->getLimit()(0, 1), 1.5);

  EXPECT_EQ(l1->getJoint()->getArmature().size(), 1);
  EXPECT_FLOAT_EQ(l1->getJoint()->getArmature()(0), 0.6f);

  EXPECT_FLOAT_EQ(l1->getJoint()->getDriveStiffness(), 1000);
  EXPECT_FLOAT_EQ(l1->getJoint()->getDriveDamping(), 100);
  EXPECT_FLOAT_EQ(l1->getJoint()->getDriveForceLimit(), 10000);
  EXPECT_EQ(l1->getJoint()->getDriveType(), ::physx::PxArticulationDriveType::eACCELERATION);

  EXPECT_FLOAT_EQ(l1->getJoint()->getDriveTargetPosition()(0), 0.6);
  EXPECT_FLOAT_EQ(l1->getJoint()->getDriveTargetVelocity()(0), 0.7);

  // fixed
  auto l2 = PhysxArticulationLinkComponent::Create(l1);
  l2->getJoint()->setAnchorPoseInParent(Pose({2, 4, 6}, {0, 0, 0, 1}));
  l2->getJoint()->setAnchorPoseInChild(Pose({-2, -4, -6}, {0, 1, 0, 0}));
  EXPECT_EQ(l2->getJoint()->getType(), ::physx::PxArticulationJointType::eFIX);
  EXPECT_EQ(l2->getJoint()->getDof(), 0);
  EXPECT_POSE_EQ(l2->getJoint()->getAnchorPoseInParent(), Pose({2, 4, 6}, {0, 0, 0, 1}));
  EXPECT_POSE_EQ(l2->getJoint()->getAnchorPoseInChild(), Pose({-2, -4, -6}, {0, 1, 0, 0}));

  // prismatic
  auto l3 = PhysxArticulationLinkComponent::Create(root);
  EXPECT_EQ(root->getArticulation(), l3->getArticulation());
  l3->getJoint()->setType(::physx::PxArticulationJointType::ePRISMATIC);
  EXPECT_EQ(l3->getJoint()->getType(), ::physx::PxArticulationJointType::ePRISMATIC);
}
