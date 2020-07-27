#pragma once
#include "sensor_msgs/image_encodings.h"
#include <Eigen/Dense>
#include <camera.h>
#include <image_transport/image_transport.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float64.h>

#include "renderer/render_interface.h"
#include "sapien_scene.h"

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
  //  ros::Timer mTimer;

  // Matrix cache for computational purpose
  //  Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>

public:
  // Init
  CameraPublisher(Renderer::ICamera *camera, const ros::NodeHandlePtr &node,
                  const std::string &frameName, double frequency)
      : mNode(new ros::NodeHandle(*node, camera->getName())),
        mCamera(static_cast<Renderer::OptifuserCamera *>(camera)), mWidth(camera->getWidth()),
        mHeight(camera->getHeight()), mPointCloudMsg(), mColorImageMsg(), mDepthImageMsg(),
        mFrameName(frameName) {
    std::string pointCloudTopicName = "points";
    std::string colorImageTopicName = "color";
    std::string depthImageTopicName = "depth";

    // Init publisher
    ros::AdvertiseOptions pointCloudOption =
        ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
            pointCloudTopicName, 1, boost::bind(&CameraPublisher::PointCloudConnect, this),
            boost::bind(&CameraPublisher::PointCloudDisconnect, this), ros::VoidPtr(), nullptr);
    mPointCloudPub = mNode->advertise(pointCloudOption);

    ros::AdvertiseOptions colorOption = ros::AdvertiseOptions::create<sensor_msgs::Image>(
        colorImageTopicName, 1, boost::bind(&CameraPublisher::ColorImageConnect, this),
        boost::bind(&CameraPublisher::ColorImageDisconnect, this), ros::VoidPtr(), nullptr);
    mColorImagePub = mNode->advertise(colorOption);

    ros::AdvertiseOptions depthOption = ros::AdvertiseOptions::create<sensor_msgs::Image>(
        depthImageTopicName, 1, boost::bind(&CameraPublisher::DepthImageConnect, this),
        boost::bind(&CameraPublisher::DepthImageDisconnect, this), ros::VoidPtr(), nullptr);
    mDepthImagePub = mNode->advertise(depthOption);

    // Init message
    mDepthImageMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    mDepthImageMsg.height = mHeight;
    mDepthImageMsg.width = mWidth;
    mDepthImageMsg.step = sizeof(float) * mWidth;
    mDepthImageMsg.data.resize(mWidth * mHeight * sizeof(float));
    mDepthImageMsg.is_bigendian = 0;
    mDepthImageMsg.header.frame_id = mFrameName;

    mColorImageMsg.encoding = sensor_msgs::image_encodings::RGBA8;
    mColorImageMsg.height = mHeight;
    mColorImageMsg.width = mWidth;
    mColorImageMsg.step = mWidth * 4;
    mColorImageMsg.data.resize(mWidth * mHeight * 4);
    mColorImageMsg.is_bigendian = 0;
    mColorImageMsg.header.frame_id = mFrameName;

    mPointCloudMsg.is_dense = false;
    mPointCloudMsg.is_bigendian = 0;
    mPointCloudMsg.header.frame_id = mFrameName;

    // Timer
    mInterval = ros::Duration(1 / frequency);
    mNextTime = ros::Time(0);
  }

protected:
  // Connection handling function
  inline void PointCloudConnect() { mPointCloudCount++; }
  inline void DepthImageConnect() { mDepthImageCount++; }
  inline void ColorImageConnect() { mColorImageCount++; }
  inline void PointCloudDisconnect() { mPointCloudCount--; }
  inline void DepthImageDisconnect() { mDepthImageCount--; }
  inline void ColorImageDisconnect() { mColorImageCount--; }

  // Update Function
  void update() {
    auto now = ros::Time::now();
    if (now < mNextTime) {
      return;
    } else {
      mNextTime += mInterval;
    }
    if (mPointCloudCount < 1 && mDepthImageCount < 1 && mColorImageCount < 1) {
      return;
    }
    mCamera->takePicture();
    if (mDepthImageCount > 0) {
      publishDepthImage(now);
    }
    if (mColorImageCount > 0) {
      publishColorImage(now);
    }
    if (mPointCloudCount > 0) {
      publishPointCloud(now);
    }
  }

  void publishDepthImage(const ros::Time &now) {
    using namespace Eigen;
    auto cloudImage = mCamera->getCustomRGBA();
    auto depthImage = mCamera->getDepth();
    mDepthImageMsg.header.stamp = now;
    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    auto *dest = (float *)(&(mDepthImageMsg.data[0]));

    // convert depth based on cutoff
    for (uint32_t j = 0; j < mHeight; j++) {
      for (uint32_t i = 0; i < mWidth; i++) {
        auto index = j * mWidth + i;
        auto value = -cloudImage[index * 4 + 2];
        if (value > this->mPointCloudCutoff && value < this->mPointCloudCutoffMax &&
            depthImage[index] < 0.999) {
          dest[i + j * mWidth] = value;
        } else {
          dest[i + j * mWidth] = bad_point;
        }
      }
    }
    mDepthImagePub.publish(mDepthImageMsg);
  }

  void publishColorImage(const ros::Time &now) {
    auto colorImage = mCamera->getColorRGBA();
    auto toCopyFrom = colorImage.data();
    mColorImageMsg.header.stamp = now;
    auto *dest = (uint8_t *)(&(mColorImageMsg.data[0]));

    int index = 0;
    for (uint32_t j = 0; j < mHeight; j++) {
      for (uint32_t i = 0; i < mWidth; i++) {
        for (uint32_t k = 0; k < 4; k++) {
          float color = toCopyFrom[index] * 255;
          color = color > 255 ? 255 : color;
          color = color < 0 ? 0 : color;
          dest[index++] = static_cast<uint8_t>(color);
        }
      }
    }
    mColorImagePub.publish(mColorImageMsg);
  };

  void publishPointCloud(const ros::Time &now) {
    auto cloudImage = mCamera->getCustomRGBA();
    auto colorImage = mCamera->getColorRGBA();
    auto depthImage = mCamera->getDepth();
    mPointCloudMsg.header.stamp = now;
    const float badPoint = std::numeric_limits<float>::quiet_NaN();

    sensor_msgs::PointCloud2Modifier pcd_modifier(mPointCloudMsg);
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    pcd_modifier.resize(mWidth * mHeight);

    sensor_msgs::PointCloud2Iterator<float> iter_x(mPointCloudMsg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(mPointCloudMsg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(mPointCloudMsg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(mPointCloudMsg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(mPointCloudMsg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(mPointCloudMsg, "b");

    for (uint32_t j = 0; j < mHeight; j++) {
      for (uint32_t i = 0; i < mWidth;
           i++, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_b, ++iter_g) {
        auto index = j * mWidth + i;
        uint32_t numPos = index * 4;
        auto value = -cloudImage[numPos + 2];
        if (value > this->mPointCloudCutoff && value < this->mPointCloudCutoffMax &&
            depthImage[index] < 0.999) {
          *iter_x = cloudImage[numPos + 0];
          *iter_y = cloudImage[numPos + 1];
          *iter_z = cloudImage[numPos + 2];
        } else {
          *iter_x = *iter_y = *iter_z = badPoint;
        }
        *iter_r = cutoff(colorImage[numPos + 0]);
        *iter_g = cutoff(colorImage[numPos + 1]);
        *iter_b = cutoff(colorImage[numPos + 2]);
      }
    }
    mPointCloudPub.publish(mPointCloudMsg);
  }

  static inline uint8_t cutoff(float num) {
    auto color = num * 255;
    color = color > 255 ? 255 : color;
    color = color < 0 ? 0 : color;
    return static_cast<uint8_t>(color);
  }
};
}