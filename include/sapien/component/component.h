#pragma once
#include "../serialize.h"
#include "sapien/math/pose.h"
#include <PxPhysicsAPI.h>
#include <memory>
#include <string>

namespace sapien {
class Entity;
class Scene;

namespace component {

class Component : public std::enable_shared_from_this<Component> {
  static uint64_t gNextComponentId;

public:
  Component() : mId(gNextComponentId++) {}
  std::shared_ptr<Entity> getEntity() const { return mEntity.lock(); }
  std::string getName() const { return mName; }
  void setName(std::string const &name) { mName = name; }

  /** called right after entity is added to scene or component is added to entity in scene */
  virtual void onAddToScene(Scene &scene) = 0;

  /** called righ before entity is removed from scene or component is removed from entity in scene
   */
  virtual void onRemoveFromScene(Scene &scene) = 0;

  /** called right after parent entity's pose is modified */
  virtual void onSetPose(Pose const &){};

  /** called right after component is attached to entity */
  virtual void onAttach(){};

  /** called right before the component is detached from entity */
  virtual void onDetach(){};

  std::shared_ptr<Scene> getScene();

  void enable();
  void disable();
  void setEnabled(bool enabled);
  bool getEnabled() const { return mEnabled; }

  /** same as Entity::getPose */
  Pose getPose() const;

  /** same as Entity::setPose */
  void setPose(Pose const &);

  // internal only, set parent entity
  void internalSetEntity(std::shared_ptr<Entity> const &entity) { mEntity = entity; }

  virtual ~Component() = default;

  virtual std::vector<uint64_t> getSerializationDependencies() const { return {}; }
  template <class Archive> void save(Archive &ar) const { ar(mName, mEnabled); }
  template <class Archive> void load(Archive &ar) { ar(mName, mEnabled); }

protected:
  std::weak_ptr<Entity> mEntity{};
  std::string mName;
  bool mEnabled{true};

  uint64_t mId{};
};

} // namespace component
} // namespace sapien

CEREAL_REGISTER_TYPE(sapien::component::Component);
