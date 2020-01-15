#pragma once
#include "id_generator.h"
#include "render_interface.h"
#include <PxPhysicsAPI.h>
#include <memory>
#include <vector>

namespace sapien {
using namespace physx;

class SScene;
class Simulation;
class SActor;
class SActorStatic;

namespace Renderer {
class IPxrRididbody;
}

class ActorBuilder {
protected:
  struct ActorBuilderShapeRecord {
    enum Type { SingleMesh, MultipleMeshes, Box, Capsule, Sphere } type;
    // mesh, scale also for box
    std::string filename;
    PxVec3 scale;

    // capsule, radius also for sphere
    PxReal radius;
    PxReal length;

    // common
    PxMaterial *material;
    PxTransform pose;
    PxReal density;
  };

  struct ActorBuilderVisualRecord {
    enum Type { Mesh, Box, Capsule, Sphere } type;

    std::string filename;
    PxVec3 scale;

    PxReal radius;
    PxReal length;

    PxVec3 color;

    PxTransform pose;
    std::string name;
  };

  std::vector<ActorBuilderShapeRecord> mShapeRecord;
  std::vector<ActorBuilderVisualRecord> mVisualRecord;

  SScene *mScene;

  bool mUseDensity = true;
  PxReal mMass = 1;
  PxTransform mCMassPose = {{0, 0, 0}, PxIdentity};
  PxVec3 mInertia = {1, 1, 1};

  struct {
    uint32_t w0 = 1, w1 = 1, w2 = 0, w3 = 0;
  } mCollisionGroup;

public:
  explicit ActorBuilder(SScene *scene = nullptr);
  ActorBuilder(ActorBuilder const &other) = default;
  ActorBuilder &operator=(ActorBuilder const &other) = default;

  void addConvexShapeFromObj(const std::string &filename,
                             const PxTransform &pose = {{0, 0, 0}, PxIdentity},
                             const PxVec3 &scale = {1, 1, 1}, PxMaterial *material = nullptr,
                             PxReal density = 1000.f);

  void addMultipleConvexShapesFromObj(const std::string &filename,
                                      const PxTransform &pose = {{0, 0, 0}, PxIdentity},
                                      const PxVec3 &scale = {1, 1, 1},
                                      PxMaterial *material = nullptr, PxReal density = 1000.f);

  void addBoxShape(const PxTransform &pose = {{0, 0, 0}, PxIdentity},
                   const PxVec3 &size = {1, 1, 1}, PxMaterial *material = nullptr,
                   PxReal density = 1000.f);

  void addCapsuleShape(const PxTransform &pose = {{0, 0, 0}, PxIdentity}, PxReal radius = 1,
                       PxReal halfLength = 1, PxMaterial *material = nullptr,
                       PxReal density = 1000.f);

  void addSphereShape(const PxTransform &pose = {{0, 0, 0}, PxIdentity}, PxReal radius = 1,
                      PxMaterial *material = nullptr, PxReal density = 1000.f);

  /* Visual functions */
  void addBoxVisual(const PxTransform &pose = {{0, 0, 0}, PxIdentity},
                    const PxVec3 &size = {1, 1, 1}, const PxVec3 &color = {1, 1, 1},
                    std::string const &name = "");

  void addCapsuleVisual(const PxTransform &pose = {{0, 0, 0}, PxIdentity}, PxReal radius = 1,
                        PxReal halfLength = 1, const PxVec3 &color = {1, 1, 1},
                        std::string const &name = "");

  void addSphereVisual(const PxTransform &pose = {{0, 0, 0}, PxIdentity}, PxReal radius = 1,
                       const PxVec3 &color = {1, 1, 1}, std::string const &name = "");

  void addObjVisual(const std::string &filename,
                    const PxTransform &pose = PxTransform({0, 0, 0}, PxIdentity),
                    const PxVec3 &scale = {1, 1, 1}, std::string const &name = "");

  /* when a.g1 & b.g2 != 0, the collision is ignored
   * by default g1 = g2 = 1
   */
  void setCollisionGroup(uint32_t g1, uint32_t g2, uint32_t g3);
  void addCollisionGroup(uint32_t g1, uint32_t g2, uint32_t g3);
  void resetCollisionGroup();

  // calling this function will overwrite the densities
  void setMassAndInertia(PxReal mass, PxTransform const &cMassPose, PxVec3 const &inertia);
  inline void setScene(SScene *scene) { mScene = scene; }

  // SActor *build(bool isStatic = false, bool isKinematic = false, std::string const &name = "")
  // const;
  SActor *build(bool isKinematic = false, std::string const &name = "") const;
  SActorStatic *buildStatic(std::string const &name = "") const;

  SActorStatic *buildGround(PxReal altitude, bool render, PxMaterial *material,
                            std::string const &name = "");

protected:
  Simulation *getSimulation() const;

  void buildShapes(std::vector<PxShape *> &shapes, std::vector<PxReal> &densities) const;
  void buildVisuals(std::vector<Renderer::IPxrRigidbody *> &renderBodies,
                    std::vector<physx_id_t> &renderIds) const;
};

} // namespace sapien
