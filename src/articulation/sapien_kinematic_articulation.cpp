#include "sapien/articulation/sapien_kinematic_articulation.h"
#include "sapien/articulation/sapien_joint.h"
#include "sapien/articulation/sapien_kinematic_joint.h"
#include "sapien/articulation/sapien_link.h"
#include "sapien/sapien_scene.h"
#include <spdlog/spdlog.h>

#define CHECK_SIZE(v)                                                                             \
  {                                                                                               \
    if ((v).size() != dof()) {                                                                    \
      throw std::runtime_error("Input vector size does not match DOF of articulation");           \
      return;                                                                                     \
    }                                                                                             \
  }

namespace sapien {

std::vector<SLinkBase *> SKArticulation::getBaseLinks() {
  std::vector<SLinkBase *> result;
  result.reserve(mLinks.size());
  for (auto &link : mLinks) {
    result.push_back(link.get());
  }
  return result;
}

std::vector<SJointBase *> SKArticulation::getBaseJoints() {
  std::vector<SJointBase *> result;
  result.reserve(mJoints.size());
  for (auto &joint : mJoints) {
    result.push_back(joint.get());
  }
  return result;
}

SLinkBase *SKArticulation::getRootLink() const { return mRootLink; }

EArticulationType SKArticulation::getType() const { return EArticulationType::KINEMATIC; }

uint32_t SKArticulation::dof() const { return mDof; }

std::vector<physx::PxReal> SKArticulation::getQpos() const {
  std::vector<physx::PxReal> result;
  for (auto &j : mJoints) {
    auto p = j->getPos();
    result.insert(result.end(), p.begin(), p.end());
  }
  return result;
}

void SKArticulation::setQpos(const std::vector<physx::PxReal> &v) {
  CHECK_SIZE(v);
  auto it = v.begin();
  for (auto &j : mJoints) {
    uint32_t dof = j->getDof();
    j->setPos({it, it + dof});
    it += dof;
  }
}

std::vector<physx::PxReal> SKArticulation::getQvel() const {
  std::vector<physx::PxReal> result;
  for (auto &j : mJoints) {
    auto v = j->getVel();
    result.insert(result.end(), v.begin(), v.end());
  }
  return result;
}
void SKArticulation::setQvel(const std::vector<physx::PxReal> &v) {
  CHECK_SIZE(v);
  auto it = v.begin();
  for (auto &j : mJoints) {
    uint32_t dof = j->getDof();
    j->setVel({it, it + dof});
    it += dof;
  }
}

std::vector<physx::PxReal> SKArticulation::getQacc() const {
  return std::vector<PxReal>(0, dof());
}
void SKArticulation::setQacc(const std::vector<physx::PxReal> &v) {
  spdlog::get("SAPIEN")->warn("Setting qacc for kinematic articulation has no effect");
}

std::vector<physx::PxReal> SKArticulation::getQf() const { return std::vector<PxReal>(0, dof()); }
void SKArticulation::setQf(const std::vector<physx::PxReal> &v) {
  spdlog::get("SAPIEN")->warn("Setting qf for kinematic articulation has no effect");
}
void SKArticulation::setRootPose(const physx::PxTransform &T) {
  mRootLink->getPxActor()->setGlobalPose(T);
}

std::vector<std::array<physx::PxReal, 2>> SKArticulation::getQlimits() const {
  std::vector<std::array<physx::PxReal, 2>> result;
  for (auto &j : mJoints) {
    auto l = j->getLimits();
    result.insert(result.end(), l.begin(), l.end());
  }
  return result;
}

void SKArticulation::setQlimits(std::vector<std::array<physx::PxReal, 2>> const &v) const {
  CHECK_SIZE(v);
  auto it = v.begin();
  for (auto &j : mJoints) {
    uint32_t dof = j->getDof();
    j->setLimits({it, it + dof});
    it += dof;
  }
}

void SKArticulation::setDriveTarget(std::vector<physx::PxReal> const &v) {
  CHECK_SIZE(v);
  auto it = v.begin();
  for (auto &j : mJoints) {
    uint32_t dof = j->getDof();
    j->setDriveTarget({it, it + dof});
    it += dof;
  }
}

std::vector<physx::PxReal> SKArticulation::getDriveTarget() const {
  return std::vector<PxReal>(0, dof());
}

void SKArticulation::prestep() {
  auto time = mParentScene->getTimestep();
  EventArticulationStep s;
  s.articulation = this;
  s.time = time;
  EventEmitter<EventArticulationStep>::emit(s);

  for (auto &l : mLinks) {
    EventActorStep s;
    s.actor = l.get();
    s.time = time;
    l->EventEmitter<EventActorStep>::emit(s);
  }

  std::vector<PxTransform> poses(mJoints.size());
  poses[mSortedIndices[0]] = mJoints[mSortedIndices[0]]->getChildLink()->getPose();

  for (uint32_t n = 1; n < mSortedIndices.size(); ++n) {
    uint32_t idx = mSortedIndices[n];
    mJoints[idx]->updatePos(mParentScene->getTimestep());
    poses[idx] = poses[mJoints[idx]->getParentLink()->getIndex()] *
                 mJoints[idx]->getChild2ParentTransform();
    mLinks[idx]->getPxActor()->setKinematicTarget(poses[idx]);
  }
}

SKArticulation::SKArticulation(SScene *scene) : SArticulationDrivable(scene) {}

} // namespace sapien

#undef CHECK_SIZE
