#pragma once
#include "actor_builder.h"
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

  struct JointRecord {
    PxArticulationJointType::Enum jointType = PxArticulationJointType::eFIX;
    std::vector<std::array<float, 2>> limits = {};
    PxTransform parentPose = {{0, 0, 0}, PxIdentity};
    PxTransform childPose = {{0, 0, 0}, PxIdentity};
    PxReal friction = 0;
    PxReal damping = 0;
    std::string name = "";
  } mJointRecord;

  ArticulationBuilder *mArticulationBuilder;
  int mIndex;
  int mParent = -1;

  std::string mName;

public:
  LinkBuilder(ArticulationBuilder *articulationBuilder, int index, int parentIndex = -1);
  LinkBuilder(LinkBuilder const &other) = default;

  inline int getIndex() const { return mIndex; };
  inline void setParent(int parentIndex) { mParent = parentIndex; };

  inline void setName(std::string const &name) { mName = name; }

  void setJointName(std::string const &jointName);
  void setJointProperties(PxArticulationJointType::Enum jointType,
                          std::vector<std::array<float, 2>> const &limits,
                          PxTransform const &parentPose, PxTransform const &childPose,
                          PxReal friction = 0.f, PxReal damping = 0.f);

  std::string summary() const;

private:
  bool build(SArticulation &articulation) const;
  bool buildKinematic(SKArticulation &articulation) const;
  bool checkJointProperties() const;
};

class ArticulationBuilder {

  std::vector<std::unique_ptr<LinkBuilder>> mLinkBuilders;

  SScene *mScene;

public:
  ArticulationBuilder(SScene *scene = nullptr);

  inline void setScene(SScene *scene) { mScene = scene; }
  inline SScene *getScene() { return mScene; }

  LinkBuilder *createLinkBuilder(LinkBuilder *parent = nullptr);
  LinkBuilder *createLinkBuilder(int parentIdx);

  SArticulation *build(bool fixBase = false) const;
  SKArticulation *buildKinematic() const;

  std::string summary() const;

private:
  bool checkTreeProperties() const;

  bool prebuild(std::vector<int> &tosort) const;
};

} // namespace sapien
