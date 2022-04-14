#pragma once
#include "sapien/actor_builder.h"
#include <PxPhysicsAPI.h>
#include <memory>
#include <sstream>
#include <vector>

namespace sapien {
using namespace physx;

class SLink;
class SJoint;
class SScene;
class ArticulationBuilder;
class SArticulation;
class SKArticulation;

class LinkBuilder : public ActorBuilder {
  friend ArticulationBuilder;

public:
  struct JointRecord {
    PxArticulationJointType::Enum jointType = PxArticulationJointType::eFIX;
    std::vector<std::array<PxReal, 2>> limits = {};
    PxTransform parentPose = {{0, 0, 0}, PxIdentity};
    PxTransform childPose = {{0, 0, 0}, PxIdentity};
    PxReal friction = 0;
    PxReal damping = 0;
    std::string name = "";
  };

protected:
  JointRecord mJointRecord;

  ArticulationBuilder *mArticulationBuilder;
  int mIndex;
  int mParent = -1;

  std::string mName;

public:
  LinkBuilder(ArticulationBuilder *articulationBuilder, int index, int parentIndex = -1);
  LinkBuilder(LinkBuilder const &other) = delete;
  LinkBuilder &operator=(LinkBuilder const &other) = delete;

  inline int getIndex() const { return mIndex; };
  inline int getParent() { return mParent; };
  inline void setParent(int parentIndex) { mParent = parentIndex; };

  inline std::string getName() { return mName; }
  inline void setName(std::string const &name) { mName = name; }

  std::string getJointName();
  void setJointName(std::string const &jointName);

  void setJointProperties(PxArticulationJointType::Enum jointType,
                          std::vector<std::array<PxReal, 2>> const &limits,
                          PxTransform const &parentPose, PxTransform const &childPose,
                          PxReal friction = 0.f, PxReal damping = 0.f);

  inline const JointRecord &getJoint() const { return mJointRecord; };

  std::string summary() const;

private:
  bool build(SArticulation &articulation) const;
  bool buildKinematic(SKArticulation &articulation) const;
  bool checkJointProperties() const;
};

class ArticulationBuilder : public std::enable_shared_from_this<ArticulationBuilder>{

  std::vector<std::shared_ptr<LinkBuilder>> mLinkBuilders;

  SScene *mScene;

public:
  ArticulationBuilder(SScene *scene = nullptr);

  inline void setScene(SScene *scene) {
    mScene = scene;
    for (auto lb : mLinkBuilders) {
      lb->setScene(scene);
    }
  }
  inline SScene *getScene() { return mScene; }

  std::shared_ptr<LinkBuilder> createLinkBuilder(std::shared_ptr<LinkBuilder> parent = nullptr);
  std::shared_ptr<LinkBuilder> createLinkBuilder(int parentIdx);

  SArticulation *build(bool fixBase = false) const;
  SKArticulation *buildKinematic() const;

  std::string summary() const;

  std::vector<LinkBuilder *> getLinkBuilders();

private:
  bool checkTreeProperties() const;

  bool prebuild(std::vector<int> &tosort) const;
};

} // namespace sapien
