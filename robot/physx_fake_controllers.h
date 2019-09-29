//
// Created by sim on 9/24/19.
//
#pragma once

#include <boost/thread/thread.hpp>
#include <moveit/controller_manager/controller_manager.h>
#include <moveit/macros/class_forward.h>
#include <ros/publisher.h>
#include <ros/rate.h>
#include <utility>

namespace robot_interface {

class BaseFakeController : public moveit_controller_manager::MoveItControllerHandle {
public:
  BaseFakeController(const std::string &name, const std::vector<std::string> &joints,
                     const ros::Publisher &pub);

  moveit_controller_manager::ExecutionStatus getLastExecutionStatus() override;
  void getJoints(std::vector<std::string> &joints) const;

protected:
  std::vector<std::string> joints_;
  const ros::Publisher &pub_;
};

class ThreadedController : public BaseFakeController {
public:
  ThreadedController(const std::string &name, const std::vector<std::string> &joints,
                     const ros::Publisher &pub);
  ~ThreadedController() override;

  bool sendTrajectory(const moveit_msgs::RobotTrajectory &t) override;
  bool cancelExecution() override;
  bool waitForExecution(const ros::Duration &) override;
  moveit_controller_manager::ExecutionStatus getLastExecutionStatus() override;

protected:
  bool cancelled() { return cancel_; }

private:
  virtual void execTrajectory(const moveit_msgs::RobotTrajectory &t) = 0;
  virtual void cancelTrajectory();

private:
  boost::thread thread_;
  bool cancel_;
  moveit_controller_manager::ExecutionStatus status_;
};

class InterpolatingController : public ThreadedController {
public:
  InterpolatingController(const std::string &name, const std::vector<std::string> &joints,
                          const ros::Publisher &pub);
  ~InterpolatingController() override;

protected:
  void execTrajectory(const moveit_msgs::RobotTrajectory &t) override;

private:
  ros::WallRate rate_;
};
} // namespace robot_interface
