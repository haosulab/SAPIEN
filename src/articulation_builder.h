#pragma once
#include "articulation_wrapper.h"
#include "id_generator.h"
#include "render_interface.h"
#include "simulation.h"
#include <PxPhysicsAPI.h>
#include <memory>
#include <vector>

using namespace physx;

class PxArticulationBuilder {
  PxSimulation *mSimulation = nullptr;
  PxPhysics *mPhysicsSDK = nullptr;
  PxCooking *mCooking = nullptr;
  Renderer::IPhysxRenderer *mRenderer = nullptr;
  PxArticulationReducedCoordinate *mArticulation = nullptr;

  std::vector<PxArticulationLink *> mLinks;

  std::vector<physx_id_t> mRenderIds;
  std::vector<PxShape *> mShapes;
  std::vector<PxReal> mDensities;
  std::map<std::string, PxArticulationLink *> namedLinks;
  std::map<PxArticulationLink *, std::string> link2JointName;

public:
  PxArticulationBuilder(PxArticulationBuilder const &other) = delete;
  const PxArticulationBuilder &operator=(PxArticulationBuilder const &other) = delete;

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
                              const std::string &name = "", const std::string &jointName = "");

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

  PxArticulationWrapper *build(bool fixBase = true);
};
