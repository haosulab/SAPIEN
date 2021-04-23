#pragma once
#include "id_generator.h"
#include "render_interface.h"
#include "sapien_material.h"
#include <PxPhysicsAPI.h>
#include <memory>
#include <vector>

namespace sapien {
using namespace physx;

class SScene;
class Simulation;
class SActor;
class SActorStatic;
class SCollisionShape;

namespace Renderer {
class IPxrRididbody;
}

class ActorBuilder {
public:
  struct ShapeRecord {
    enum Type { SingleMesh, MultipleMeshes, Box, Capsule, Sphere } type;
    // mesh, scale also for box
    std::string filename;
    PxVec3 scale;

    // capsule, radius also for sphere
    PxReal radius;
    PxReal length;

    // common
    std::shared_ptr<SPhysicalMaterial> material;
    PxTransform pose;
    PxReal density;

    PxReal patchRadius;
    PxReal minPatchRadius;
    bool isTrigger;
  };

  struct VisualRecord {
    enum Type { Mesh, Box, Capsule, Sphere } type;

    std::string filename;
    PxVec3 scale;

    PxReal radius;
    PxReal length;

    std::shared_ptr<Renderer::IPxrMaterial> material;

    PxTransform pose;
    std::string name;
  };

protected:
  std::vector<ShapeRecord> mShapeRecord;
  std::vector<VisualRecord> mVisualRecord;

  SScene *mScene;

  bool mUseDensity = true;
  PxReal mMass = 1e-6;
  PxTransform mCMassPose = {{0, 0, 0}, PxIdentity};
  PxVec3 mInertia = {1e-6, 1e-6, 1e-6};

  struct {
    uint32_t w0 = 1, w1 = 1, w2 = 0, w3 = 0;
  } mCollisionGroup;

public:
  explicit ActorBuilder(SScene *scene = nullptr);
  ActorBuilder(ActorBuilder const &other) = delete;
  ActorBuilder &operator=(ActorBuilder const &other) = delete;

  void removeAllShapes();
  void removeAllVisuals();
  int getShapeCount() const;
  int getVisualCount() const;
  void removeShapeAt(uint32_t index);
  void removeVisualAt(uint32_t index);
  inline std::vector<ShapeRecord> const &getShapes() const { return mShapeRecord; }
  inline std::vector<VisualRecord> const &getVisuals() const { return mVisualRecord; }

  void addConvexShapeFromFile(const std::string &filename,
                              const PxTransform &pose = {{0, 0, 0}, PxIdentity},
                              const PxVec3 &scale = {1, 1, 1},
                              std::shared_ptr<SPhysicalMaterial> material = nullptr,
                              PxReal density = 1000.f, PxReal patchRadius = 0.f,
                              PxReal minPatchRadius = 0.f, bool isTrigger = false);

  void addMultipleConvexShapesFromFile(const std::string &filename,
                                       const PxTransform &pose = {{0, 0, 0}, PxIdentity},
                                       const PxVec3 &scale = {1, 1, 1},
                                       std::shared_ptr<SPhysicalMaterial> material = nullptr,
                                       PxReal density = 1000.f, PxReal patchRadius = 0.f,
                                       PxReal minPatchRadius = 0.f, bool isTrigger = false);

  void addBoxShape(const PxTransform &pose = {{0, 0, 0}, PxIdentity},
                   const PxVec3 &halfSize = {1, 1, 1},
                   std::shared_ptr<SPhysicalMaterial> material = nullptr, PxReal density = 1000.f,
                   PxReal patchRadius = 0.f, PxReal minPatchRadius = 0.f, bool isTrigger = false);

  void addCapsuleShape(const PxTransform &pose = {{0, 0, 0}, PxIdentity}, PxReal radius = 1,
                       PxReal halfLength = 1,
                       std::shared_ptr<SPhysicalMaterial> material = nullptr,
                       PxReal density = 1000.f, PxReal patchRadius = 0.f,
                       PxReal minPatchRadius = 0.f, bool isTrigger = false);

  void addSphereShape(const PxTransform &pose = {{0, 0, 0}, PxIdentity}, PxReal radius = 1,
                      std::shared_ptr<SPhysicalMaterial> material = nullptr,
                      PxReal density = 1000.f, PxReal patchRadius = 0.f,
                      PxReal minPatchRadius = 0.f, bool isTrigger = false);

  /* Visual functions */
  void addBoxVisualWithMaterial(const PxTransform &pose = {{0, 0, 0}, PxIdentity},
                                const PxVec3 &halfSize = {1, 1, 1},
                                std::shared_ptr<Renderer::IPxrMaterial> material = {},
                                std::string const &name = "");
  void addBoxVisual(const PxTransform &pose = {{0, 0, 0}, PxIdentity},
                    const PxVec3 &size = {1, 1, 1}, const PxVec3 &color = {1, 1, 1},
                    std::string const &name = "");

  void addCapsuleVisualWithMaterial(const PxTransform &pose = {{0, 0, 0}, PxIdentity},
                                    PxReal radius = 1, PxReal halfLength = 1,
                                    std::shared_ptr<Renderer::IPxrMaterial> material = {},
                                    std::string const &name = "");
  void addCapsuleVisual(const PxTransform &pose = {{0, 0, 0}, PxIdentity}, PxReal radius = 1,
                        PxReal halfLength = 1, const PxVec3 &color = {1, 1, 1},
                        std::string const &name = "");

  void addSphereVisualWithMaterial(const PxTransform &pose = {{0, 0, 0}, PxIdentity},
                                   PxReal radius = 1,
                                   std::shared_ptr<Renderer::IPxrMaterial> material = {},
                                   std::string const &name = "");
  void addSphereVisual(const PxTransform &pose = {{0, 0, 0}, PxIdentity}, PxReal radius = 1,
                       const PxVec3 &color = {1, 1, 1}, std::string const &name = "");

  void addVisualFromFile(const std::string &filename,
                         const PxTransform &pose = PxTransform({0, 0, 0}, PxIdentity),
                         const PxVec3 &scale = {1, 1, 1}, std::string const &name = "");

  /* when a.g1 & b.g2 != 0, the collision is ignored
   * by default g1 = g2 = 1
   */
  void setCollisionGroup(uint32_t g0, uint32_t g1, uint32_t g2, uint32_t g3);
  void addCollisionGroup(uint32_t g0, uint32_t g1, uint32_t g2, uint32_t g3);
  void resetCollisionGroup();

  // calling this function will overwrite the densities
  void setMassAndInertia(PxReal mass, PxTransform const &cMassPose, PxVec3 const &inertia);
  inline void setScene(SScene *scene) { mScene = scene; }

  SActor *build(bool isKinematic = false, std::string const &name = "") const;
  SActorStatic *buildStatic(std::string const &name = "") const;

  SActorStatic *buildGround(PxReal altitude, bool render,
                            std::shared_ptr<SPhysicalMaterial> material,
                            std::shared_ptr<Renderer::IPxrMaterial> renderMaterial = {},
                            std::string const &name = "");

protected:
  void buildShapes(std::vector<std::unique_ptr<SCollisionShape>> &shapes,
                   std::vector<PxReal> &densities) const;
  void buildVisuals(std::vector<Renderer::IPxrRigidbody *> &renderBodies,
                    std::vector<physx_id_t> &renderIds) const;
  void buildCollisionVisuals(std::vector<Renderer::IPxrRigidbody *> &collisionBodies,
                             std::vector<std::unique_ptr<SCollisionShape>> &shapes) const;
};

} // namespace sapien
