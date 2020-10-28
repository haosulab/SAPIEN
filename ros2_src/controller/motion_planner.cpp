#include "motion_planner.h"
#include "manager/motion_planning_config.hpp"
#include "manager/robot_manager.h"
#include "sapien_actor_base.h"
#include "sapien_scene.h"
#include <spdlog/spdlog.h>
namespace sapien::ros2 {

MotionPlanner::MotionPlanner(rclcpp::Node::SharedPtr node, rclcpp::Clock::SharedPtr clock,
                             const moveit::planning_interface::MoveItCppPtr &moveitCpp,
                             const std::string &groupName, const std::string &serviceName,
                             robot_state::RobotState *robotState)
    : mNode(std::move(node)), mClock(std::move(clock)), mMoveItCpp(moveitCpp),
      mManagerRobotState(robotState) {
  auto pipelineNames = moveitCpp->getPlanningPipelineNames(groupName);
  auto logger = spdlog::get("SAPIEN_ROS2");
  logger->info("There are {} pipelines loaded for group {}", pipelineNames.size(), groupName);
  for (const auto &pipelineName : pipelineNames) {
    logger->info(pipelineName);
  }

  mComponent = std::make_unique<PlanningComponent>(groupName, mMoveItCpp);
  mJointModelGroup = mMoveItCpp->getRobotModel()->getJointModelGroup(groupName);
  mEEName = mJointModelGroup->getLinkModelNames().back();
  mBaseName = mJointModelGroup->getLinkModelNames()[0];

  mPlanRequestParams.planning_pipeline = *pipelineNames.begin();
  mPlanRequestParams.planner_id = serviceName;
  mPlanRequestParams.planning_attempts = 10;
}
bool MotionPlanner::setGoalState(const physx::PxTransform &Pose, const std::string &linkName) {
  geometry_msgs::msg::PoseStamped targetPose;
  std::string targetLink = linkName.empty() ? mEEName : linkName;
  targetPose.header.frame_id = mBaseName;
  targetPose.pose.orientation.w = Pose.q.w;
  targetPose.pose.orientation.x = Pose.q.x;
  targetPose.pose.orientation.y = Pose.q.y;
  targetPose.pose.orientation.z = Pose.q.z;
  targetPose.pose.position.x = Pose.p.x;
  targetPose.pose.position.y = Pose.p.y;
  targetPose.pose.position.z = Pose.p.z;
  return mComponent->setGoal(targetPose, targetLink);
}
MotionPlan MotionPlanner::plan() {
  auto solution = mComponent->plan(mPlanRequestParams);
  if (solution.error_code != PlanningComponent::MoveItErrorCode::SUCCESS) {
    return MotionPlan(0, 0);
  }
  auto pointsNum = solution.trajectory->getWayPointCount();
  auto jointNames = solution.trajectory->getGroup()->getActiveJointModelNames();
  auto dof = jointNames.size();

  MotionPlan plan(dof, pointsNum);
  auto duration = solution.trajectory->getWayPointDurations();

  for (size_t j = 0; j < pointsNum; ++j) {
    plan.duration(j) = duration[j];
    auto state = solution.trajectory->getWayPoint(j);
    auto position = state.getVariablePositions();
    plan.position.block(j, 0, 1, dof) = Eigen::Map<Eigen::RowVectorXd>(position, 1, dof);
    auto velocity = state.getVariableVelocities();
    plan.velocity.block(j, 0, 1, dof) = Eigen::Map<Eigen::RowVectorXd>(velocity, 1, dof);
    auto acceleration = state.getVariableAccelerations();
    plan.acceleration.block(j, 0, 1, dof) = Eigen::Map<Eigen::RowVectorXd>(acceleration, 1, dof);
    auto effort = state.getVariableEffort();
    plan.effort.block(j, 0, 1, dof) = Eigen::Map<Eigen::RowVectorXd>(effort, 1, dof);
  }
  plan.jointNames = jointNames;

  return plan;
}

bool MotionPlanner::updateCollisionObjects(SActorBase *actor) {
  if (actor->getCollisionShapes().empty()) {
    return false;
  }
  moveit_msgs::msg::CollisionObject collisionObject;
  planning_scene_monitor::LockedPlanningSceneRW scene(mMoveItCpp->getPlanningSceneMonitor());
  std::vector<moveit_msgs::msg::CollisionObject> collisionObjs;
  scene->getCollisionObjectMsgs(collisionObjs);
  auto name = actor->getName();
  bool known = false;
  auto rootPose = mManager->getRootPose().getInverse();
  for (auto &collisionObj : collisionObjs) {
    if (collisionObj.id == name) {
      known = true;
      break;
    }
  }

  if (!known) {
    auto collisions = actor->getCollisionShapes();
    for (auto &collision : collisions) {
      auto shape = collision.get();
      if (shape->type == "convex_mesh") {
        auto geometry = static_cast<SConvexMeshGeometry *>(shape->geometry.get());
        auto scale = geometry->scale;
        shape_msgs::msg::Mesh mesh;
        for (size_t l = 0; l < geometry->vertices.size(); l += 3) {
          geometry_msgs::msg::Point point;
          point.set__x(geometry->vertices[l] * scale[0]);
          point.set__y(geometry->vertices[l + 1] * scale[1]);
          point.set__z(geometry->vertices[l + 2] * scale[2]);
          mesh.vertices.push_back(point);
        }
        auto index = geometry->indices;
        for (size_t m = 0; m < index.size(); m += 3) {
          shape_msgs::msg::MeshTriangle triangle;
          triangle.set__vertex_indices({index[m], index[m + 1], index[m + 2]});
          mesh.triangles.push_back(triangle);
        }
        auto meshPose = fillPose(rootPose * actor->getPose() * shape->pose);
        collisionObject.meshes.push_back(mesh);
        collisionObject.mesh_poses.push_back(meshPose);
      }
    }
    collisionObject.set__operation(moveit_msgs::msg::CollisionObject::ADD);
  } else {
    auto collisions = actor->getCollisionShapes();
    for (auto &collision : collisions) {
      auto meshPose = fillPose(rootPose * actor->getPose() * collision->pose);
      collisionObject.mesh_poses.push_back(meshPose);
    }
    collisionObject.set__operation(moveit_msgs::msg::CollisionObject::MOVE);
  }
  collisionObject.set__id(name);
  collisionObject.header.frame_id = mBaseName;
  scene->processCollisionObjectMsg(collisionObject);
  return true;
}

geometry_msgs::msg::Pose MotionPlanner::fillPose(const physx::PxTransform &pose) {
  geometry_msgs::msg::Pose meshPose;
  meshPose.position.x = pose.p.x;
  meshPose.position.y = pose.p.y;
  meshPose.position.z = pose.p.z;
  meshPose.orientation.w = pose.q.w;
  meshPose.orientation.x = pose.q.x;
  meshPose.orientation.y = pose.q.y;
  meshPose.orientation.z = pose.q.z;
  return meshPose;
}
}; // namespace sapien::ros2
