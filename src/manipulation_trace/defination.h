//
// Created by sim on 10/20/19.
//

#pragma once

#include <PxPhysicsAPI.h>
#include <vector>
namespace sapien::ManipulationTrace {

enum StateType { X_STATE, Q_STATE };
enum StateBasedCommand { FREE, LINEAR_INTERPOLATE, MOBILITY_INTERPOLATE };
enum StateLessCommand { GRIPPER_CLOSE, GRIPPER_OPEN };

struct MtState {};

struct MtXState : public MtState {
  physx::PxTransform pose;
  physx::PxReal rotTolerance;
  physx::PxReal transTolerance;

  physx::PxReal rotMetric(const physx::PxTransform &evalPose) {
    physx::PxReal innerProduct = evalPose.q.dot(pose.q);
    return 1 - innerProduct * innerProduct;
  };

  physx::PxReal transMetric(const physx::PxTransform &evalPose) {
    auto diff = evalPose.p - pose.p;
    return diff.magnitude();
  }
};

struct MtQState : public MtState {
  std::vector<PxReal> qpos;
  std::vector<PxReal> qvel;
  std::vector<size_t > qmask;
  physx::PxReal jointTolerance;

  physx::PxReal jointMetric(const std::vector<PxReal> &q) {
    assert(q.size() == qpos.size());
    physx::PxReal distance = 0;
    for (size_t i = 0; i < qmask.size(); ++i) {
      auto index = qmask[i];
      distance += q[index] - qpos[index];
    }
    return distance;
  }
};

struct IMtGoal {
  virtual StateType get_state_type() const = 0;
  virtual MtQState get_joint_goal() const = 0;
  virtual MtXState get_pose_goal() const = 0;
};

struct MtAction {
  bool stateless;
  physx::PxRigidBody *object;
  MtState *endEffectorStartState;
  MtState *endEffectorGoalState = nullptr;
  MtState *objectStartState;
  MtState *objectGoalState = nullptr;

  // Replay action and return whether it achieve within tolerance
  virtual bool replay() = 0;
};

class MtStateLessAction : MtAction {
  static const bool stateless = true;
  StateLessCommand command;
  bool replay() override;
};

class MtStatefulAction : MtAction {
  static const bool stateless = false;
  StateBasedCommand command;
  bool replay() override;
};

class ManipulationTrace {
  std::vector<MtAction> actions;
  std::string robotLink;
  inline bool replay() {
    for (auto &action : actions) {
      if (!action.replay()) {
        return false;
      }
    }
    return true;
  };
};

} // namespace sapien::ManipulationTrace
