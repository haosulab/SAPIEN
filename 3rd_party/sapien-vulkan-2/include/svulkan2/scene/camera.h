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
#pragma once
#include "node.h"
#include "svulkan2/common/config.h"
#include <vector>

namespace svulkan2 {
namespace scene {

class Camera : public Node {
public:
  enum Type { eOrthographic, ePerspective, eMatrix, eUndefined };

private:
  glm::mat4 mProjectionMatrix{1};

  float mWidth{1};
  float mHeight{1};
  float mNear{0.01};
  float mFar{10};

  float mFx{0};
  float mFy{0};
  float mCx{0};
  float mCy{0};
  float mSkew{0};

  // used by orthographic
  float mLeft{0};
  float mRight{0};
  float mTop{0};
  float mBottom{0};

  Camera::Type mType{eUndefined};

public:
  Camera(std::string const &name = "");

  void setPerspectiveParameters(float near, float far, float fovy, float width, float height);

  void setPerspectiveParameters(float near, float far, float fx, float fy, float cx, float cy,
                                float width, float height, float skew);

  void setIntrinsicMatrix(glm::mat3 const &intrinsic, float near, float far, float width,
                          float height);

  void setOrthographicParameters(float near, float far, float scaling, float width, float height);
  void setOrthographicParameters(float near, float far, float left, float right, float bottom,
                                 float top, float width, float height);

  void setWidth(float width);
  void setHeight(float height);

  float getWidth() const;
  float getHeight() const;
  float getNear() const;
  float getFar() const;
  float getFovx() const;
  float getFovy() const;

  float getOrthographicLeft() const;
  float getOrthographicRight() const;
  float getOrthographicBottom() const;
  float getOrthographicTop() const;

  float getFx() const;
  float getFy() const;
  float getCx() const;
  float getCy() const;
  float getSkew() const;

  Camera::Type getCameraType() const { return mType; };

  void uploadToDevice(core::Buffer &cameraBuffer, StructDataLayout const &cameraLayout);

  inline glm::mat4 getProjectionMatrix() const { return mProjectionMatrix; }
};

} // namespace scene
} // namespace svulkan2