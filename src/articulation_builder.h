#pragma once
#include "articulation_wrapper.h"
#include "id_generator.h"
#include "render_interface.h"
#include "simulation.h"
#include <PxPhysicsAPI.h>
#include <memory>
#include <vector>

namespace sapien {
using namespace physx;

class ArticulationBuilder {
  Simulation *mSimulation = nullptr;
  PxPhysics *mPhysicsSDK = nullptr;
  PxCooking *mCooking = nullptr;
  Renderer::IPhysxRenderer *mRenderer = nullptr;
  PxArticulationReducedCoordinate *mArticulation = nullptr;

  std::vector<PxArticulationLink *> mLinks;

  std::vector<physx_id_t> mRenderIds;
  std::vector<PxShape *> mShapes;
  std::vector<PxReal> mDensities;
  std::map<std::string, PxArticulationLink *> mNamedLinks;
  std::map<PxArticulationLink *, std::string> mLink2JointName;
  std::map<PxArticulationLink *, physx_id_t> mLink2LinkId;

public:
  ArticulationBuilder(ArticulationBuilder const &other) = delete;
  const ArticulationBuilder &operator=(ArticulationBuilder const &other) = delete;

  ArticulationBuilder(Simulation *simulation);

  /**
   *  add a link to the internal articulation, return the added link
   *  @param parent The parent link
   *  @param pose the pose of the new link added (this parameter seems to only matter for root
   * node)
   *  @return The added new link
   */
  PxArticulationLink *
  addLink(PxArticulationLink *parent, const PxTransform &pose = PxTransform({0, 0, 0}, PxIdentity),
          const std::string &name = "", const std::string &jointName = "",
          PxArticulationJointType::Enum jointType = PxArticulationJointType::eUNDEFINED,
          std::vector<std::array<float, 2>> const &limits = {},
          PxTransform const &parentPose = {{0, 0, 0}, PxIdentity},
          PxTransform const &childPose = {{0, 0, 0}, PxIdentity});

  void addBoxShapeToLink(PxArticulationLink &link,
                         const PxTransform &pose = {{0, 0, 0}, PxIdentity},
                         const PxVec3 &size = {1, 1, 1}, PxMaterial *material = nullptr);

  void addCapsuleShapeToLink(PxArticulationLink &link,
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
                          const PxVec3 &size = {1, 1, 1}, const PxVec3 &color = {1, 1, 1});

  void addCapsuleVisualToLink(PxArticulationLink &link,
                              const PxTransform &pose = {{0, 0, 0}, PxIdentity}, PxReal radius = 1,
                              PxReal length = 1, const PxVec3 &color = {1, 1, 1});

  void addSphereVisualToLink(PxArticulationLink &link,
                             const PxTransform &pose = {{0, 0, 0}, PxIdentity}, PxReal radius = 1,
                             const PxVec3 &color = {1, 1, 1});

  void addObjVisualToLink(PxArticulationLink &link, const std::string &filename,
                          const PxTransform &pose = PxTransform({0, 0, 0}, PxIdentity),
                          const PxVec3 &scale = {1, 1, 1});

  ArticulationWrapper *build(bool fixBase = true, bool balanceForce = false);
};

} // namespace sapien
