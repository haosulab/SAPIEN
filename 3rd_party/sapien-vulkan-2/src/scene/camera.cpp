/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "svulkan2/scene/camera.h"
#include <vector>

namespace svulkan2 {
namespace scene {

Camera::Camera(std::string const &name) : Node(name) {}

void Camera::setPerspectiveParameters(float near, float far, float fx, float fy, float cx,
                                      float cy, float width, float height, float skew) {
  mType = Camera::Type::ePerspective;

  mWidth = width;
  mHeight = height;
  mNear = near;
  mFar = far;

  mFx = fx;
  mFy = fy;
  mCx = cx;
  mCy = cy;
  mSkew = skew;

  mProjectionMatrix = math::fullPerspective(near, far, fx, fy, cx, cy, width, height, skew);
}

void Camera::setIntrinsicMatrix(glm::mat3 const &intrinsic, float near, float far, float width,
                                float height) {
  setPerspectiveParameters(near, far, intrinsic[0][0], intrinsic[1][1], intrinsic[2][0],
                           intrinsic[2][1], width, height, intrinsic[1][0]);
}

void Camera::setPerspectiveParameters(float near, float far, float fovy, float width,
                                      float height) {
  mType = Camera::Type::ePerspective;
  float f = math::fov2focal(fovy, height);
  setPerspectiveParameters(near, far, f, f, width / 2, height / 2, width, height, 0);
}

void Camera::setOrthographicParameters(float near, float far, float scaling, float width,
                                       float height) {
  float aspect = width / height;
  setOrthographicParameters(near, far, -scaling * aspect, scaling * aspect, -scaling, scaling,
                            width, height);
}

void Camera::setOrthographicParameters(float near, float far, float left, float right,
                                       float bottom, float top, float width, float height) {
  mType = Camera::Type::eOrthographic;
  mNear = near;
  mFar = far;

  mLeft = left;
  mRight = right;
  mBottom = bottom;
  mTop = top;

  mProjectionMatrix = math::ortho(left, right, bottom, top, near, far);
}

void Camera::setWidth(float width) { mWidth = width; }
void Camera::setHeight(float height) { mHeight = height; }

void Camera::uploadToDevice(core::Buffer &cameraBuffer, StructDataLayout const &cameraLayout) {
  std::vector<char> buffer(cameraLayout.size);

  auto viewMatrix = glm::affineInverse(mTransform.worldModelMatrix);
  auto projInv = glm::inverse(mProjectionMatrix);
  std::memcpy(buffer.data() + cameraLayout.elements.at("viewMatrix").offset, &viewMatrix[0][0],
              64);
  std::memcpy(buffer.data() + cameraLayout.elements.at("viewMatrixInverse").offset,
              &mTransform.worldModelMatrix[0][0], 64);
  std::memcpy(buffer.data() + cameraLayout.elements.at("projectionMatrix").offset,
              &mProjectionMatrix[0][0], 64);
  std::memcpy(buffer.data() + cameraLayout.elements.at("projectionMatrixInverse").offset,
              &projInv[0][0], 64);

  auto it = cameraLayout.elements.find("prevViewMatrix");
  if (it != cameraLayout.elements.end()) {
    auto prevViewMatrix = glm::affineInverse(mTransform.prevWorldModelMatrix);
    std::memcpy(buffer.data() + it->second.offset, &prevViewMatrix[0][0], 64);
  }

  it = cameraLayout.elements.find("prevViewMatrixInverse");
  if (it != cameraLayout.elements.end()) {
    std::memcpy(buffer.data() + cameraLayout.elements.at("prevViewMatrixInverse").offset,
                &mTransform.prevWorldModelMatrix[0][0], 64);
  }

  it = cameraLayout.elements.find("width");
  if (it != cameraLayout.elements.end()) {
    float fwidth = static_cast<float>(mWidth);
    std::memcpy(buffer.data() + cameraLayout.elements.at("width").offset, &fwidth, sizeof(float));
  }

  it = cameraLayout.elements.find("height");
  if (it != cameraLayout.elements.end()) {
    float fheight = static_cast<float>(mHeight);
    std::memcpy(buffer.data() + cameraLayout.elements.at("height").offset, &fheight,
                sizeof(float));
  }

  cameraBuffer.upload<char>(buffer);
}

float Camera::getWidth() const {
  if (mType == Camera::Type::eUndefined) {
    throw std::runtime_error("Camera is not initialized with parameters.");
  }
  if (mType == Camera::Type::eMatrix) {
    throw std::runtime_error(
        "Camera initialized by projection matrix does not have this property");
  }

  return mWidth;
}

float Camera::getHeight() const {
  if (mType == Camera::Type::eUndefined) {
    throw std::runtime_error("Camera is not initialized with parameters.");
  }
  if (mType == Camera::Type::eMatrix) {
    throw std::runtime_error(
        "Camera initialized by projection matrix does not have this property");
  }

  return mHeight;
}

float Camera::getNear() const {
  if (mType == Camera::Type::eUndefined) {
    throw std::runtime_error("Camera is not initialized with parameters.");
  }
  if (mType == Camera::Type::eMatrix) {
    throw std::runtime_error(
        "Camera initialized by projection matrix does not have this property");
  }
  return mNear;
}

float Camera::getFar() const {
  if (mType == Camera::Type::eUndefined) {
    throw std::runtime_error("Camera is not initialized with parameters.");
  }
  if (mType == Camera::Type::eMatrix) {
    throw std::runtime_error(
        "Camera initialized by projection matrix does not have this property");
  }
  return mFar;
}

float Camera::getFovx() const {
  if (mType == Camera::Type::eUndefined) {
    throw std::runtime_error("Camera is not initialized with parameters.");
  }
  if (mType == Camera::Type::eMatrix) {
    throw std::runtime_error(
        "Camera initialized by projection matrix does not have this property");
  }
  return math::focal2fov(mFx, mWidth);
}

float Camera::getFovy() const {
  if (mType == Camera::Type::eUndefined) {
    throw std::runtime_error("Camera is not initialized with parameters.");
  }
  if (mType == Camera::Type::eMatrix) {
    throw std::runtime_error(
        "Camera initialized by projection matrix does not have this property");
  }
  return math::focal2fov(mFy, mHeight);
}

float Camera::getOrthographicLeft() const {
  if (mType == Camera::Type::eUndefined) {
    throw std::runtime_error("Camera is not initialized with parameters.");
  }
  if (mType != eOrthographic) {
    throw std::runtime_error("Only orthographic camera has this property.");
  }
  return mLeft;
}
float Camera::getOrthographicRight() const {
  if (mType == Camera::Type::eUndefined) {
    throw std::runtime_error("Camera is not initialized with parameters.");
  }
  if (mType != eOrthographic) {
    throw std::runtime_error("Only orthographic camera has this property.");
  }
  return mRight;
}
float Camera::getOrthographicBottom() const {
  if (mType == Camera::Type::eUndefined) {
    throw std::runtime_error("Camera is not initialized with parameters.");
  }
  if (mType != eOrthographic) {
    throw std::runtime_error("Only orthographic camera has this property.");
  }
  return mBottom;
}
float Camera::getOrthographicTop() const {
  if (mType == Camera::Type::eUndefined) {
    throw std::runtime_error("Camera is not initialized with parameters.");
  }
  if (mType != eOrthographic) {
    throw std::runtime_error("Only orthographic camera has this property.");
  }
  return mTop;
}

float Camera::getFx() const {
  if (mType == Camera::Type::eUndefined) {
    throw std::runtime_error("Camera is not initialized with parameters.");
  }
  if (mType != Camera::Type::ePerspective) {
    throw std::runtime_error("Only camera created by full intrinsic matrix "
                             "properties has this property.");
  }

  return mFx;
}

float Camera::getFy() const {
  if (mType == Camera::Type::eUndefined) {
    throw std::runtime_error("Camera is not initialized with parameters.");
  }
  if (mType != Camera::Type::ePerspective) {
    throw std::runtime_error("Only camera created by full intrinsic matrix "
                             "properties has this property.");
  }
  return mFy;
}
float Camera::getCx() const {
  if (mType == Camera::Type::eUndefined) {
    throw std::runtime_error("Camera is not initialized with parameters.");
  }
  if (mType != Camera::Type::ePerspective) {
    throw std::runtime_error("Only camera created by full intrinsic matrix "
                             "properties has this property.");
  }
  return mCx;
}
float Camera::getCy() const {
  if (mType == Camera::Type::eUndefined) {
    throw std::runtime_error("Camera is not initialized with parameters.");
  }
  if (mType != Camera::Type::ePerspective) {
    throw std::runtime_error("Only camera created by full intrinsic matrix "
                             "properties has this property.");
  }
  return mCy;
}

float Camera::getSkew() const {
  if (mType == Camera::Type::eUndefined) {
    throw std::runtime_error("Camera is not initialized with parameters.");
  }
  if (mType != Camera::Type::ePerspective) {
    throw std::runtime_error("Only camera created by full intrinsic matrix "
                             "properties has this property.");
  }
  return mSkew;
}

} // namespace scene
} // namespace svulkan2