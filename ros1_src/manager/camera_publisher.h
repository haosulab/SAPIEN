#pragma once
#include <renderer/camera.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "renderer/render_interface.h"

namespace sapien::ros1 {

class SceneManager;

class CameraPublisher {
  friend SceneManager;

protected:
  // ROS Handle
  ros::NodeHandlePtr mNode;
  //  ros::CallbackQueue mQueue;

  // SAPIEN camera
  Renderer::OptifuserCamera *mCamera;

  // A pointer to the ROS node.  A node will be instantiated if it does not exist.
  ros::Publisher mPointCloudPub;
  ros::Publisher mDepthImagePub;
  ros::Publisher mColorImagePub;

  // Counter
  uint16_t mPointCloudCount = 0;
  uint16_t mColorImageCount = 0;
  uint16_t mDepthImageCount = 0;

  // PointCloud specification
  float mPointCloudCutoff = 0.25;
  float mPointCloudCutoffMax = 10;
  uint32_t mWidth;
  uint32_t mHeight;

  // Visual message
  sensor_msgs::PointCloud2 mPointCloudMsg;
  sensor_msgs::Image mColorImageMsg;
  sensor_msgs::Image mDepthImageMsg;
  std::string mFrameName;

  // Timer
  ros::Duration mInterval;
  ros::Time mNextTime;

public:
  // Init
  CameraPublisher(Renderer::ICamera *camera, const ros::NodeHandlePtr &node,
                  const std::string &frameName, double frequency);

protected:
  // Connection handling function
  inline void PointCloudConnect() { mPointCloudCount++; }
  inline void DepthImageConnect() { mDepthImageCount++; }
  inline void ColorImageConnect() { mColorImageCount++; }
  inline void PointCloudDisconnect() { mPointCloudCount--; }
  inline void DepthImageDisconnect() { mDepthImageCount--; }
  inline void ColorImageDisconnect() { mColorImageCount--; }

  // Update Function
  void update();
  void publishDepthImage(const ros::Time &now);
  void publishColorImage(const ros::Time &now);
  void publishPointCloud(const ros::Time &now);

  static inline uint8_t cutoff(float num) {
    auto color = num * 255;
    color = color > 255 ? 255 : color;
    color = color < 0 ? 0 : color;
    return static_cast<uint8_t>(color);
  }
};
}