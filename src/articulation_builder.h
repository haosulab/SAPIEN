#pragma once
#include "id_generator.h"
#include "render_interface.h"
#include "simulation.h"
#include <PxPhysicsAPI.h>
#include <memory>
#include <vector>

#define APP_ASSERT_FATAL(exp, msg)                                                                \
  if (!(exp)) {                                                                                   \
    std::cerr << msg << std::endl;                                                                \
    exit(1);                                                                                      \
  }

#define APP_ASSERT_WAN(exp, msg)                                                                  \
  if (!(exp)) {                                                                                   \
    std::cerr << msg << std::endl;                                                                \
  }

using namespace physx;

// TODO: proof read and test this struct
struct PxArticulationInterface {
  PxArticulationReducedCoordinate *articulation = nullptr;
  PxArticulationCache *cache = nullptr;
  std::vector<PxArticulationLink *> links;
  std::map<std::string, PxArticulationLink *> namedLinks;

  std::vector<PxU32> dofStarts;

  void updateCache() { articulation->copyInternalStateToCache(*cache, PxArticulationCache::eALL); }

  void updateArticulation() { articulation->applyCache(*cache, PxArticulationCache::eALL); }

  std::vector<PxReal> qpos(PxU32 index) {
    PxU32 dof = links[index]->getInboundJointDof();
    return std::vector<PxReal>(&cache->jointPosition[dofStarts[index]],
                               &cache->jointPosition[dofStarts[index + dof]]);
  }

  std::vector<PxReal> qpos(const std::string &name) {
    return qpos(namedLinks[name]->getLinkIndex());
  }

  void set_qpos(PxU32 index, const std::vector<PxReal> &v) {
    PxU32 dof = links[index]->getInboundJointDof();

    APP_ASSERT_FATAL(dof == v.size(), "vector size does not match dof");
    for (PxU32 i = 0; i < dof; ++i) {
      cache->jointPosition[dofStarts[index] + i] = v[i];
    }
  }

  void set_qpos_unchecked(const std::vector<PxReal> &v) {
    for (PxU32 i = 0; i < v.size(); ++i) {
      cache->jointPosition[i] = v[i];
    }
  }

  void set_qpos(const std::string &name, const std::vector<PxReal> &v) {
    set_qpos(namedLinks[name]->getLinkIndex(), v);
  }
  // TODO: setter for all

  std::vector<PxReal> qvel(PxU32 index) {
    PxU32 dof = links[index]->getInboundJointDof();
    return std::vector<PxReal>(&cache->jointVelocity[dofStarts[index]],
                               &cache->jointVelocity[dofStarts[index + dof]]);
  }

  std::vector<PxReal> qvel(const std::string &name) {
    return qvel(namedLinks[name]->getLinkIndex());
  }

  std::vector<PxReal> qacc(PxU32 index) {
    PxU32 dof = links[index]->getInboundJointDof();
    return std::vector<PxReal>(&cache->jointAcceleration[dofStarts[index]],
                               &cache->jointAcceleration[dofStarts[index + dof]]);
  }

  std::vector<PxReal> qacc(const std::string &name) {
    return qacc(namedLinks[name]->getLinkIndex());
  }

  std::vector<PxReal> qf(PxU32 index) {
    PxU32 dof = links[index]->getInboundJointDof();
    return std::vector<PxReal>(&cache->jointForce[dofStarts[index]],
                               &cache->jointForce[dofStarts[index + dof]]);
  }

  std::vector<PxReal> qf(const std::string &name) { return qf(namedLinks[name]->getLinkIndex()); }
};

class PxArticulationBuilder {
  PxSimulation *mSimulation = nullptr;
  PxPhysics *mPhysicsSDK = nullptr;
  PxCooking *mCooking = nullptr;
  IRenderer *mRenderer = nullptr;
  PxArticulationReducedCoordinate *mArticulation = nullptr;

  std::vector<PxArticulationLink *> mLinks;

  std::vector<physx_id_t> mRenderIds;
  std::vector<PxShape *> mShapes;
  std::vector<PxReal> mDensities;
  std::map<std::string, PxArticulationLink *> namedLinks;
  uint32_t mCount = 0;

public:
  PxArticulationBuilder(PxSimulation *simulation);

  /**
   *  add a link to the internal articulation, return the added link
   *  @param parent The parent link
   *  @param pose the pose of the new link added (this parameter seems to only matter for root
   * node)
   *  @return The added new link
   */
  PxArticulationLink *addLink(PxArticulationLink *parent,
                              const PxTransform &pose = PxTransform({0, 0, 0}, PxIdentity),
                              const std::string &name = "");

  /**
   *  TODO: support scaling and different primitives
   */
  void addBoxShapeToLink(PxArticulationLink &link,
                         const PxTransform &pose = {{0, 0, 0}, PxIdentity},
                         const PxVec3 &size = {1, 1, 1}, PxMaterial *material = nullptr);

  void addCylinderShapeToLink(PxArticulationLink &link,
                              const PxTransform &pose = {{0, 0, 0}, PxIdentity}, PxReal radius = 1,
                              PxReal length = 1, PxMaterial *material = nullptr);

  void addSphereShapeToLink(PxArticulationLink &link,
                            const PxTransform &pose = {{0, 0, 0}, PxIdentity}, PxReal radius = 1,
                            PxMaterial *material = nullptr);

  void addConvexObjShapeToLink(PxArticulationLink &link, const std::string &filename,
                               const PxTransform &pose = PxTransform({0, 0, 0}, PxIdentity),
                               const PxVec3 &scale = {1, 1, 1}, PxMaterial *material = nullptr);

  /**
   *  explicitly set the mass and inertia of a link
   */
  void setLinkMassAndInertia(PxArticulationLink &link, PxReal mass, const PxTransform &cMassPose,
                             const PxVec3 &inertia);

  /**
   *  compute the mass and inertia of a link by a constant density
   *  TODO: support different densities for different shapes
   */
  void updateLinkMassAndInertia(PxArticulationLink &link, PxReal density = 1.f);

  void addBoxVisualToLink(PxArticulationLink &link,
                          const PxTransform &pose = {{0, 0, 0}, PxIdentity},
                          const PxVec3 &size = {1, 1, 1});

  void addCylinderVisualToLink(PxArticulationLink &link,
                               const PxTransform &pose = {{0, 0, 0}, PxIdentity},
                               PxReal radius = 1, PxReal length = 1);

  void addSphereVisualToLink(PxArticulationLink &link,
                             const PxTransform &pose = {{0, 0, 0}, PxIdentity}, PxReal radius = 1);

  void addObjVisualToLink(PxArticulationLink &link, const std::string &filename,
                          const PxTransform &pose = PxTransform({0, 0, 0}, PxIdentity),
                          const PxVec3 &scale = {1, 1, 1});

  PxArticulationInterface build(bool fixBase = true);
};
