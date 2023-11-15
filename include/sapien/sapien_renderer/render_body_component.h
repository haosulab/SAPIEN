#pragma once

#include "../component.h"
#include "render_shape.h"
#include "sapien/math/bounding_box.h"
#include "sapien/math/pose.h"
#include "sapien/serialize.h"
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

  template <class Archive> void save(Archive &ar) const {
    ar(mRenderShapes, mVisibility, mShadingMode);
  }
  template <class Archive> void load(Archive &ar) { ar(mRenderShapes, mVisibility, mShadingMode); }

  void disableRenderId();
  void enableRenderId();
  bool getRenderIdDisabled() const { return mRenderIdDisabled; }

private:
  std::vector<std::shared_ptr<RenderShape>> mRenderShapes{};
  svulkan2::scene::Node *mNode{};

  float mVisibility{1.f};
  int mShadingMode{0};

  bool mRenderIdDisabled{false};
};

} // namespace sapien_renderer
} // namespace sapien

CEREAL_REGISTER_TYPE(sapien::sapien_renderer::SapienRenderBodyComponent);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::Component,
                                     sapien::sapien_renderer::SapienRenderBodyComponent);
