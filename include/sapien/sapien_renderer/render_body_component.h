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

#include "../component.h"
#include "render_shape.h"
#include "sapien/math/bounding_box.h"
#include "sapien/math/pose.h"
#include <memory>
#include <string>
#include <svulkan2/scene/node.h>
#include <svulkan2/scene/object.h>
#include <svulkan2/scene/scene.h>
#include <vector>

namespace sapien {
class Entity;
namespace sapien_renderer {

class SapienRenderBodyComponent : public Component {
public:
  SapienRenderBodyComponent();

  std::shared_ptr<SapienRenderBodyComponent> attachRenderShape(std::shared_ptr<RenderShape> shape);
  std::vector<std::shared_ptr<RenderShape>> getRenderShapes() const { return mRenderShapes; };

  AABB getGlobalAABBFast() const;
  AABB computeGlobalAABBTight() const;

  void onAddToScene(Scene &scene) override;
  void onRemoveFromScene(Scene &scene) override;

  // called by system to sync pose
  void internalUpdate();

  svulkan2::scene::Node *internalGetNode() const { return mNode; }

  void setVisibility(float v);
  float getVisibility() const;

  void setShadingMode(int mode);
  int getShadingMode() const;

  void setProperty(std::string const &name, int value);
  void setProperty(std::string const &name, float value);
  void setProperty(std::string const &name, Vec3 const &value);
  void setTexture(std::string const &name, std::shared_ptr<SapienRenderTexture> texture);
  void setTextureArray(std::string const &name,
                       std::vector<std::shared_ptr<SapienRenderTexture>> textures);

  ~SapienRenderBodyComponent();
  SapienRenderBodyComponent(SapienRenderBodyComponent const &) = delete;
  SapienRenderBodyComponent &operator=(SapienRenderBodyComponent const &) = delete;
  SapienRenderBodyComponent(SapienRenderBodyComponent const &&) = delete;
  SapienRenderBodyComponent &operator=(SapienRenderBodyComponent const &&) = delete;

  void disableRenderId();
  void enableRenderId();
  bool getRenderIdDisabled() const { return mRenderIdDisabled; }

  std::shared_ptr<SapienRenderBodyComponent> clone() const;

private:
  std::vector<std::shared_ptr<RenderShape>> mRenderShapes{};
  svulkan2::scene::Node *mNode{};

  float mVisibility{1.f};
  int mShadingMode{0};

  bool mRenderIdDisabled{false};
};

} // namespace sapien_renderer
} // namespace sapien
