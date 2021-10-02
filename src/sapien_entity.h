#pragma once
#include <PxPhysicsAPI.h>
#include <string>

namespace sapien {
using namespace physx;

class SScene;

class SEntity {

public:
  SEntity(SScene *scene);
  inline std::string getName() { return mName; };
  inline void setName(const std::string &name) { mName = name; }
  inline SScene *getScene() { return mParentScene; }

  virtual ~SEntity() = default;
  virtual PxTransform getPose() const = 0;

  SEntity &operator=(SEntity const &other) = delete;
  SEntity &operator=(SEntity &&other) = delete;
  SEntity(SEntity const &other) = delete;
  SEntity(SEntity &&other) = delete;

protected:
  std::string mName{};
  SScene *mParentScene{};
};

} // namespace sapien
