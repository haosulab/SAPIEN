//
// Created by sim on 9/24/19.
//
#include "physx_fake_controllers.h"
#include <boost/thread.hpp>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/console.h>
#include <ros/param.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

constexpr const char *DEFAULT_TYPE = "interpolate";
constexpr const char *ROBOT_DESCRIPTION = "robot_description";

namespace robot_interface {
BaseFakeController::BaseFakeController(const std::string &name,
                                       const std::vector<std::string> &joints,
                                       const ros::Publisher &pub)
    : moveit_controller_manager::MoveItControllerHandle(name), joints_(joints), pub_(pub) {
  std::stringstream ss;
  ss << "Fake controller '" << name << "' with joints [ ";
  std::copy(joints.begin(), joints.end(), std::ostream_iterator<std::string>(ss, " "));
  ss << "]";
  ROS_INFO_STREAM(ss.str());
}

void BaseFakeController::getJoints(std::vector<std::string> &joints) const { joints = joints_; }

moveit_controller_manager::ExecutionStatus BaseFakeController::getLastExecutionStatus() {
  return moveit_controller_manager::ExecutionStatus::SUCCEEDED;
}

ThreadedController::ThreadedController(const std::string &name,
                                       const std::vector<std::string> &joints,
                                       const ros::Publisher &pub)
    : BaseFakeController(name, joints, pub) {
  cancel_ = false;
}

ThreadedController::~ThreadedController() { ThreadedController::cancelTrajectory(); }

void ThreadedController::cancelTrajectory() {
  cancel_ = true;
  thread_.join();
}

bool ThreadedController::sendTrajectory(const moveit_msgs::RobotTrajectory &t) {
  cancelTrajectory(); // cancel any previous fake motion
  cancel_ = false;
  status_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
  thread_ = boost::thread(boost::bind(&ThreadedController::execTrajectory, this, t));
  return true;
}

bool ThreadedController::cancelExecution() {
  cancelTrajectory();
  ROS_INFO("Fake trajectory execution cancelled");
  status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
  return true;
}

bool ThreadedController::waitForExecution(const ros::Duration & /*timeout*/) {
  thread_.join();
  status_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  return true;
}

moveit_controller_manager::ExecutionStatus ThreadedController::getLastExecutionStatus() {
  return status_;
}

InterpolatingController::InterpolatingController(const std::string &name,
                                                 const std::vector<std::string> &joints,
                                                 const ros::Publisher &pub)
    : ThreadedController(name, joints, pub), rate_(10) {
  double r;
  if (ros::param::get("~fake_interpolating_controller_rate", r))
    rate_ = ros::WallRate(r);
}

InterpolatingController::~InterpolatingController() = default;

void interpolate(sensor_msgs::JointState &js, const trajectory_msgs::JointTrajectoryPoint &prev,
                 const trajectory_msgs::JointTrajectoryPoint &next, const ros::Duration &elapsed) {
  double duration = (next.time_from_start - prev.time_from_start).toSec();
  double alpha = 1.0;
  if (duration > std::numeric_limits<double>::epsilon())
    alpha = (elapsed - prev.time_from_start).toSec() / duration;

  js.position.resize(prev.positions.size());
  for (std::size_t i = 0, end = prev.positions.size(); i < end; ++i) {
    js.position[i] = prev.positions[i] + alpha * (next.positions[i] - prev.positions[i]);
  }
}

void InterpolatingController::execTrajectory(const moveit_msgs::RobotTrajectory &t) {
  ROS_INFO("Fake execution of trajectory");
  if (t.joint_trajectory.points.empty()) {
    ROS_INFO("No points in the received trajectory");
    return;
  }

  sensor_msgs::JointState js;
  js.header = t.joint_trajectory.header;
  js.name = t.joint_trajectory.joint_names;

  const std::vector<trajectory_msgs::JointTrajectoryPoint> &points = t.joint_trajectory.points;
  auto prev = points.begin(), next = points.begin() + 1, end = points.end();

  ros::Time start_time = ros::Time::now();
  while (!cancelled()) {
    ros::Duration elapsed = ros::Time::now() - start_time;
    // hop to next targetted via point
    while (next != end && elapsed > next->time_from_start) {
      ++prev;
      ++next;
    }
    if (next == end)
      break;

    double duration = (next->time_from_start - prev->time_from_start).toSec();
    ROS_DEBUG("elapsed: %.3f via points %td,%td / %td  alpha: %.3f", elapsed.toSec(),
              prev - points.begin(), next - points.begin(), end - points.begin(),
              duration > std::numeric_limits<double>::epsilon()
                  ? (elapsed - prev->time_from_start).toSec() / duration
                  : 1.0);
    interpolate(js, *prev, *next, elapsed);
    js.header.stamp = ros::Time::now();
    pub_.publish(js);
    rate_.sleep();
  }
  if (cancelled())
    return;

  ros::Duration elapsed = ros::Time::now() - start_time;
  ROS_DEBUG("elapsed: %.3f via points %td,%td / %td  alpha: 1.0", elapsed.toSec(),
            prev - points.begin(), next - points.begin(), end - points.begin());

  // publish last point
  interpolate(js, *prev, *prev, prev->time_from_start);
  js.header.stamp = ros::Time::now();
  pub_.publish(js);

  ROS_DEBUG("Fake execution of trajectory: done");
}

class MoveItFakeControllerManager : public moveit_controller_manager::MoveItControllerManager {
public:
  MoveItFakeControllerManager() : node_handle_("~") {
    if (!node_handle_.hasParam("controller_list")) {
      ROS_ERROR_STREAM_NAMED("MoveItFakeControllerManager", "No controller_list specified.");
      return;
    }

    XmlRpc::XmlRpcValue controller_list;
    node_handle_.getParam("controller_list", controller_list);
    if (controller_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_ERROR_NAMED("MoveItFakeControllerManager",
                      "controller_list should be specified as an array");
      return;
    }

    /* by setting latch to true we preserve the initial joint state while other nodes launch */
    bool latch = true;
    pub_ = node_handle_.advertise<sensor_msgs::JointState>("fake_controller_joint_states", 100,
                                                           latch);

    /* publish initial pose */
    XmlRpc::XmlRpcValue initial;
    if (node_handle_.getParam("initial", initial)) {
      sensor_msgs::JointState js = loadInitialJointValues(initial);
      js.header.stamp = ros::Time::now();
      pub_.publish(js);
    }

    /* actually create each controller */
    for (int i = 0; i < controller_list.size(); ++i) {
      if (!controller_list[i].hasMember("name") || !controller_list[i].hasMember("joints")) {
        ROS_ERROR_NAMED("MoveItFakeControllerManager",
                        "Name and joints must be specified for each controller");
        continue;
      }

      try {
        const std::string name = std::string(controller_list[i]["name"]);

        if (controller_list[i]["joints"].getType() != XmlRpc::XmlRpcValue::TypeArray) {
          ROS_ERROR_STREAM_NAMED("MoveItFakeControllerManager",
                                 "The list of joints for controller "
                                     << name << " is not specified as an array");
          continue;
        }
        std::vector<std::string> joints;
        joints.reserve(controller_list[i]["joints"].size());
        for (int j = 0; j < controller_list[i]["joints"].size(); ++j)
          joints.emplace_back(std::string(controller_list[i]["joints"][j]));

        const std::string &type = controller_list[i].hasMember("type")
                                      ? std::string(controller_list[i]["type"])
                                      : DEFAULT_TYPE;
        if (type == "interpolate")
          controllers_[name] = std::make_shared<InterpolatingController>(name, joints, pub_);
        else
          ROS_ERROR_STREAM("Unknown fake controller type: " << type);
      } catch (...) {
        ROS_ERROR_NAMED("MoveItFakeControllerManager",
                        "Caught unknown exception while parsing controller information");
      }
    }
  }

  sensor_msgs::JointState loadInitialJointValues(XmlRpc::XmlRpcValue &param) const {
    sensor_msgs::JointState js;

    if (param.getType() != XmlRpc::XmlRpcValue::TypeArray || param.size() == 0) {
      ROS_ERROR_ONCE_NAMED("loadInitialJointValues",
                           "Parameter 'initial' should be an array of (group, pose) "
                           "structs.");
      return js;
    }

    robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_DESCRIPTION);
    const robot_model::RobotModelPtr &robot_model = robot_model_loader.getModel();
    typedef std::map<std::string, double> JointPoseMap;
    JointPoseMap joints;

    for (int i = 0, end = param.size(); i != end; ++i) {
      try {
        std::string group_name = std::string(param[i]["group"]);
        std::string pose_name = std::string(param[i]["pose"]);
        if (!robot_model->hasJointModelGroup(group_name)) {
          ROS_WARN_STREAM_NAMED("loadInitialJointValues",
                                "Unknown joint model group: " << group_name);
          continue;
        }
        moveit::core::JointModelGroup *jmg = robot_model->getJointModelGroup(group_name);
        moveit::core::RobotState robot_state(robot_model);
        const std::vector<std::string> &joint_names = jmg->getActiveJointModelNames();

        if (!robot_state.setToDefaultValues(jmg, pose_name)) {
          ROS_WARN_NAMED("loadInitialJointValues", "Unknown pose '%s' for group '%s'.",
                         pose_name.c_str(), group_name.c_str());
          continue;
        }
        ROS_INFO_NAMED("loadInitialJointValues", "Set joints of group '%s' to pose '%s'.",
                       group_name.c_str(), pose_name.c_str());

        for (const auto &joint_name : joint_names) {
          const moveit::core::JointModel *jm = robot_state.getJointModel(joint_name);
          if (!jm) {
            ROS_WARN_STREAM_NAMED("loadInitialJointValues", "Unknown joint: " << joint_name);
            continue;
          }
          if (jm->getVariableCount() != 1) {
            ROS_WARN_STREAM_NAMED("loadInitialJointValues",
                                  "Cannot handle multi-variable joint: " << joint_name);
            continue;
          }

          joints[joint_name] = robot_state.getJointPositions(jm)[0];
        }
      } catch (...) {
        ROS_ERROR_ONCE_NAMED("loadInitialJointValues",
                             "Caught unknown exception while reading initial pose "
                             "information.");
      }
    }

    // fill the joint state
    for (JointPoseMap::const_iterator it = joints.begin(), end = joints.end(); it != end; ++it) {
      js.name.push_back(it->first);
      js.position.push_back(it->second);
    }
    return js;
  }

  ~MoveItFakeControllerManager() override = default;

  /*
   * Get a controller, by controller name (which was specified in the controllers.yaml
   */
  moveit_controller_manager::MoveItControllerHandlePtr
  getControllerHandle(const std::string &name) override {
    auto it = controllers_.find(name);
    if (it != controllers_.end())
      return it->second;
    else
      ROS_FATAL_STREAM("No such controller: " << name);
    return moveit_controller_manager::MoveItControllerHandlePtr();
  }

  /*
   * Get the list of controller names.
   */
  void getControllersList(std::vector<std::string> &names) override {
    for (auto it = controllers_.begin(); it != controllers_.end(); ++it)
      names.push_back(it->first);
    ROS_INFO_STREAM("Returned " << names.size() << " controllers in list");
  }

  /*
   * Fake controllers are always active
   */
  void getActiveControllers(std::vector<std::string> &names) override {
    getControllersList(names);
  }

  /*
   * Fake controllers are always loaded
   */
  virtual void getLoadedControllers(std::vector<std::string> &names) { getControllersList(names); }

  /*
   * Get the list of joints that a controller can control.
   */
  void getControllerJoints(const std::string &name, std::vector<std::string> &joints) override {
    auto it = controllers_.find(name);
    if (it != controllers_.end()) {
      it->second->getJoints(joints);
    } else {
      ROS_WARN("The joints for controller '%s' are not known. Perhaps the controller "
               "configuration is not loaded on "
               "the param server?",
               name.c_str());
      joints.clear();
    }
  }

  /*
   * Controllers are all active and default.
   */
  moveit_controller_manager::MoveItControllerManager::ControllerState
  getControllerState(const std::string &name) override {
    moveit_controller_manager::MoveItControllerManager::ControllerState state;
    state.active_ = true;
    state.default_ = true;
    return state;
  }

  /* Cannot switch our controllers */
  bool switchControllers(const std::vector<std::string> &activate,
                         const std::vector<std::string> &deactivate) override {
    return false;
  }

protected:
  ros::NodeHandle node_handle_;
  ros::Publisher pub_;
  std::map<std::string, std::shared_ptr<class BaseFakeController>> controllers_;
};

} // namespace robot_interface
