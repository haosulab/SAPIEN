#pragma once
#include "sapien/math/pose.h"
#include <PxPhysicsAPI.h>
#include <memory>
#include <string>

namespace sapien {
class Entity;
class Scene;

class Component : public std::enable_shared_from_this<Component> {
public:
  Component();

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

  uint64_t getId() const { return mId; }

  virtual ~Component() = default;

protected:
  std::weak_ptr<Entity> mEntity{};
  std::string mName;
  bool mEnabled{true};

  uint64_t mId{};
};

struct comp_cmp {
  bool operator()(std::shared_ptr<Component> const &a, std::shared_ptr<Component> const &b) const {
    if (!a) {
      return true;
    }
    if (!b) {
      return true;
    }
    return a->getId() < b->getId();
  }
};

} // namespace sapien
