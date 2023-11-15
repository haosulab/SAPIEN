#pragma once
#include "math/pose.h"
#include <PxPhysicsAPI.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace sapien {
class Component;
class Scene;

class Entity : public std::enable_shared_from_this<Entity> {
public:
  Entity();

  inline std::string getName() { return mName; };
  inline void setName(const std::string &name) { mName = name; }

  std::shared_ptr<Scene> getScene();

  template <std::derived_from<Component> T> std::shared_ptr<T> getComponent() const {
    auto it = std::find_if(mComponents.begin(), mComponents.end(),
                           [](auto &c) { return dynamic_cast<T *>(c.get()) != nullptr; });
    if (it == mComponents.end()) {
      return nullptr;
    }
    return std::static_pointer_cast<T>(*it);
  }

  std::shared_ptr<Entity> addComponent(std::shared_ptr<Component> component);
  void removeComponent(std::shared_ptr<Component> component);

  std::vector<std::shared_ptr<Component>> getComponents() const { return mComponents; }

  Pose getPose() const;
  void setPose(Pose const &);

  /** called when added to scene */
  void onAddToScene(Scene &);

  /** called when removed from a scene */
  void onRemoveFromScene(Scene &);

  /** same as Scene::addEntity */
  std::shared_ptr<Entity> addToScene(Scene &);

  /** same as Scene::removeEntity */
  void removeFromScene();

  ~Entity() = default;
  Entity &operator=(Entity const &other) = delete;
  Entity &operator=(Entity &&other) = delete;
  Entity(Entity const &other) = delete;
  Entity(Entity &&other) = delete;

  // called when added to a scene
  inline void internalSetScene(Scene *scene) { mScene = scene; }

  // called internally to set the pose of this entity.
  // this function does not propagate to components
  void internalSyncPose(Pose const &);

  // called internally to swap in python components to replace placeholder components
  void internalSwapInComponent(uint32_t index, std::shared_ptr<Component> component);

  uint64_t getId() const { return mId; }
  void internalSetId(uint64_t id) { mId = id; }

  uint64_t getPerSceneId() const { return mPerSceneId; }
  void internalSetPerSceneId(uint64_t id) { mPerSceneId = id; }

  template <class Archive> void save(Archive &ar) const { ar(mName, mPose, mComponents); }
  template <class Archive> void load(Archive &ar) {
    std::vector<std::shared_ptr<Component>> components;
    ar(mName, mPose, components);
    for (auto &c : components) {
      addComponent(c);
    }
  }

protected:
  uint64_t mId{};
  uint64_t mPerSceneId{0};

  std::string mName{};
  Scene *mScene{};
  Pose mPose;
  std::vector<std::shared_ptr<Component>> mComponents;
};

} // namespace sapien
