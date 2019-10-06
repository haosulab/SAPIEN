//
// Created by sim on 10/5/19.
//

#include "cartesian_velocity_controller.h"
#include "controllable_articulation_wrapper.h"
#include "joint_pub_node.h"
#include "joint_trajectory_controller.h"
#include "velocity_control_service.h"
#include <map>

namespace sapien::robot {

class ControllerManger {
private:
  ControllableArticulationWrapper *wrapper;
  std::unique_ptr<JointPubNode> jointPubNode = nullptr;
  std::map<std::string, std::unique_ptr<CartesianVelocityController>>
      name2CartesianVelocityController;
  std::map<std::string, std::unique_ptr<VelocityControllerServer>> name2JointVelocityController;
  std::map<std::string, std::unique_ptr<GroupControllerNode>> name2GroupTrajectoryController;

  std::string robotName;
  std::unique_ptr<ros::NodeHandle> nh;
  float timestep;

  // Spinner and callback management
  ros::AsyncSpinner spinner;

public:
  ControllerManger(const std::string &robotName, ControllableArticulationWrapper *wrapper);

  // Function for add controllers
  void createJointPubNode(double pubFrequency, double updateFrequency);
  void createCartesianVelocityController(const std::string &groupName);
  void createJointVelocityController(const std::vector<std::string> &jointNames,
                                     const std::string &serviceName);
  void createGroupTrajectoryController(const std::string &groupName);

  // Manage controller manager handle
  ros::NodeHandle *getHandle() const;
  std::string getRobotName() const;
  void start();
  void stop();
  void removeController(const std::string &);
};
} // namespace sapien::robot

// TODO: do something when timestep change
