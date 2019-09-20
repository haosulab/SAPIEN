#include "articulation_wrapper.h"
#include <iostream>
#include <sstream>

uint32_t PxArticulationWrapper::dof() const { return articulation->getDofs(); }

void PxArticulationWrapper::updateCache() {
  articulation->copyInternalStateToCache(*cache, PxArticulationCache::eALL);
}

void PxArticulationWrapper::updateArticulation() {
  articulation->applyCache(*cache, PxArticulationCache::eALL);
}

std::vector<PxReal> PxArticulationWrapper::qpos(PxU32 index) {
  PxU32 dof = links[index]->getInboundJointDof();
  return std::vector<PxReal>(&cache->jointPosition[dofStarts[index]],
                             &cache->jointPosition[dofStarts[index + dof]]);
}

std::vector<PxReal> PxArticulationWrapper::qpos(const std::string &name) {
  return qpos(namedLinks[name]->getLinkIndex());
}

void PxArticulationWrapper::set_qpos(PxU32 index, const std::vector<PxReal> &v) {
  PxU32 dof = links[index]->getInboundJointDof();

  APP_ASSERT_FATAL(dof == v.size(), "vector size does not match dof");
  for (PxU32 i = 0; i < dof; ++i) {
    cache->jointPosition[dofStarts[index] + i] = v[i];
  }
}

void PxArticulationWrapper::set_qpos(const std::string &name, const std::vector<PxReal> &v) {
  set_qpos(namedLinks[name]->getLinkIndex(), v);
}

void PxArticulationWrapper::set_qpos_unchecked(const std::vector<PxReal> &v) {
  for (size_t i = 0; i < v.size(); ++i) {
    cache->jointPosition[i] = v[i];
  }
}

std::vector<PxReal> PxArticulationWrapper::qvel(PxU32 index) {
  PxU32 dof = links[index]->getInboundJointDof();
  return std::vector<PxReal>(&cache->jointVelocity[dofStarts[index]],
                             &cache->jointVelocity[dofStarts[index + dof]]);
}

std::vector<PxReal> PxArticulationWrapper::qvel(const std::string &name) {
  return qvel(namedLinks[name]->getLinkIndex());
}

void PxArticulationWrapper::set_qvel_unchecked(const std::vector<PxReal> &v) {
  for (size_t i = 0; i < v.size(); ++i) {
    cache->jointVelocity[i] = v[i];
  }
}

void PxArticulationWrapper::set_qf_unchecked(const std::vector<PxReal> &v) {
  for (size_t i = 0; i < v.size(); ++i) {
    cache->jointForce[i] = v[i];
  }
}

std::vector<PxReal> PxArticulationWrapper::qacc(PxU32 index) {
  PxU32 dof = links[index]->getInboundJointDof();
  return std::vector<PxReal>(&cache->jointAcceleration[dofStarts[index]],
                             &cache->jointAcceleration[dofStarts[index + dof]]);
}

std::vector<PxReal> PxArticulationWrapper::qacc(const std::string &name) {
  return qacc(namedLinks[name]->getLinkIndex());
}

std::vector<PxReal> PxArticulationWrapper::qf(PxU32 index) {
  PxU32 dof = links[index]->getInboundJointDof();
  return std::vector<PxReal>(&cache->jointForce[dofStarts[index]],
                             &cache->jointForce[dofStarts[index + dof]]);
}

std::vector<PxReal> PxArticulationWrapper::qf(const std::string &name) {
  return qf(namedLinks[name]->getLinkIndex());
}

std::string PxArticulationWrapper::summary() const {
  std::stringstream ss;
  ss << "|----------------- Summary -----------------|\n";
  ss << "| Name:       " << articulation->getName() << std::endl;
  ss << "| DOF:        " << dof() << std::endl;
  ss << "| Links:      " << articulation->getNbLinks() << std::endl;

  for (size_t i = 0; i < links.size(); ++i) {
    auto link = links[i];
    ss << "Link " << i << ": " << link->getName() << std::endl;
    auto pose = link->getGlobalPose(); // link2global

    ss << "  Global pose: [" << pose.p.x << " " << pose.p.y << " " << pose.p.z << "]"
       << " [" << pose.q.x << " " << pose.q.y << " " << pose.q.z << " " << pose.q.w << "]\n";

    pose = link->getCMassLocalPose(); // cm2link
    ss << "  CM local pose: [" << pose.p.x << " " << pose.p.y << " " << pose.p.z << "]"
       << " [" << pose.q.x << " " << pose.q.y << " " << pose.q.z << " " << pose.q.w << "]\n";

    pose = link->getGlobalPose() * link->getCMassLocalPose();
    ss << "  CM global pose: [" << pose.p.x << " " << pose.p.y << " " << pose.p.z << "]"
       << " [" << pose.q.x << " " << pose.q.y << " " << pose.q.z << " " << pose.q.w << "]\n";

    int nbShapes = link->getNbShapes();
    ss << "  Collision shapes: " << nbShapes << std::endl;
    PxShape *shapes[nbShapes];
    link->getShapes(shapes, nbShapes);
    for (int i = 0; i < nbShapes; ++i) {
      auto data = shapes[i]->getSimulationFilterData();
      char buf[100];
      sprintf(buf, "    %d: %04x %04x %04x %04x\n", i, data.word3, data.word2, data.word1,
              data.word0);
      ss << buf;
    }
  }

  return ss.str();
}
