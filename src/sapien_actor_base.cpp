#include "sapien_actor_base.h"
#include "renderer/render_interface.h"
#include "sapien_scene.h"
#include <spdlog/spdlog.h>

namespace sapien {

std::vector<Renderer::IPxrRigidbody *> SActorBase::getRenderBodies() { return mRenderBodies; }
std::vector<Renderer::IPxrRigidbody *> SActorBase::getCollisionBodies() {
  return mCollisionBodies;
}

void SActorBase::renderCollisionBodies(bool collision) {
  collisionRender = collision;
  for (auto body : mRenderBodies) {
    body->setVisible(!collision);
  }
  for (auto body : mCollisionBodies) {
    body->setVisible(collision);
  }
}

bool SActorBase::isRenderingCollision() const { return collisionRender; }

void SActorBase::prestep() {
  EventActorStep s;
  s.actor = this;
  s.time = mParentScene->getTimestep();
  EventEmitter<EventActorStep>::emit(s);
}

void SActorBase::updateRender(PxTransform const &pose) {
  for (auto body : mRenderBodies) {
    body->update(pose);
  }
  for (auto body : mCollisionBodies) {
    body->update(pose);
  }
}

void SActorBase::addDrive(SDrive *drive) { mDrives.push_back(drive); }
void SActorBase::removeDrive(SDrive *drive) {
  mDrives.erase(std::remove(mDrives.begin(), mDrives.end(), drive), mDrives.end());
}

std::vector<SShape> SActorBase::getCollisionShapes() {
  std::vector<SShape> output;
  auto actor = getPxActor();
  std::vector<PxShape *> shapes(actor->getNbShapes());
  actor->getShapes(shapes.data(), shapes.size());
  for (PxShape *shape : shapes) {
    output.emplace_back();
    SShape &outputShape = output.back();
    outputShape.pose = shape->getLocalPose();
    switch (shape->getGeometryType()) {
    case PxGeometryType::eBOX: {
      outputShape.type = "box";
      PxBoxGeometry g;
      shape->getBoxGeometry(g);
      auto sg = std::make_unique<SBoxGeometry>();
      sg->halfLengths = g.halfExtents;
      outputShape.geometry = std::move(sg);
      break;
    }
    case PxGeometryType::eSPHERE: {
      outputShape.type = "sphere";
      PxSphereGeometry g;
      shape->getSphereGeometry(g);
      auto sg = std::make_unique<SSphereGeometry>();
      sg->radius = g.radius;
      outputShape.geometry = std::move(sg);
      break;
    }

    case PxGeometryType::eCAPSULE: {
      outputShape.type = "capsule";
      PxCapsuleGeometry g;
      shape->getCapsuleGeometry(g);
      auto sg = std::make_unique<SCapsuleGeometry>();
      sg->radius = g.radius;
      sg->halfLength = g.halfHeight;
      outputShape.geometry = std::move(sg);
      break;
    }

    case PxGeometryType::ePLANE: {
      outputShape.type = "plane";
      auto sg = std::make_unique<SPlaneGeometry>();
      outputShape.geometry = std::move(sg);
      break;
    }
    case PxGeometryType::eCONVEXMESH: {
      outputShape.type = "convex_mesh";
      PxConvexMeshGeometry g;
      shape->getConvexMeshGeometry(g);
      auto sg = std::make_unique<SConvexMeshGeometry>();
      sg->scale = g.scale.scale;
      sg->rotation = g.scale.rotation;

      // fill vertices
      sg->vertices.reserve(3 * g.convexMesh->getNbVertices());
      auto vertices = g.convexMesh->getVertices();
      for (uint32_t i = 0; i < g.convexMesh->getNbVertices(); ++i) {
        sg->vertices.push_back(vertices[i].x);
        sg->vertices.push_back(vertices[i].y);
        sg->vertices.push_back(vertices[i].z);
      }

      // fill indices
      sg->indices.reserve(3 * g.convexMesh->getNbPolygons());
      auto indices = g.convexMesh->getIndexBuffer();
      for (uint32_t i = 0; i < g.convexMesh->getNbPolygons(); ++i) {
        PxHullPolygon polygon;
        g.convexMesh->getPolygonData(i, polygon);
        for (int j = 0; j < int(polygon.mNbVerts) - 2; ++j) {
          sg->indices.push_back(indices[polygon.mIndexBase]);
          sg->indices.push_back(indices[polygon.mIndexBase + j + 1]);
          sg->indices.push_back(indices[polygon.mIndexBase + j + 2]);
        }
      }
      outputShape.geometry = std::move(sg);
      break;
    }
    default:
      spdlog::get("SAPIEN")->critical("Unrecognized geometry in getCollisionShapes");
      throw std::runtime_error("Unrecognized geometry");
    }
  }
  return output;
}

SActorBase::SActorBase(physx_id_t id, SScene *scene,
                       std::vector<Renderer::IPxrRigidbody *> renderBodies,
                       std::vector<Renderer::IPxrRigidbody *> collisionBodies)
    : mId(id), mParentScene(scene), mRenderBodies(renderBodies),
      mCollisionBodies(collisionBodies) {}

PxTransform SActorBase::getPose() { return getPxActor()->getGlobalPose(); }

PxVec3 SActorDynamicBase::getVelocity() { return getPxActor()->getLinearVelocity(); }

PxVec3 SActorDynamicBase::getAngularVelocity() { return getPxActor()->getAngularVelocity(); }

void SActorDynamicBase::addForceAtPoint(const PxVec3 &force, const PxVec3 &pos) {
  PxRigidBodyExt::addForceAtPos(*getPxActor(), force, pos);
}

void SActorDynamicBase::addForceTorque(const PxVec3 &force, const PxVec3 &torque) {
  getPxActor()->addForce(force);
  getPxActor()->addTorque(torque);
}

void SActorDynamicBase::setDamping(PxReal linear, PxReal angular) {
  auto actor = getPxActor();
  actor->setLinearDamping(linear);
  actor->setAngularDamping(angular);
}

PxReal SActorDynamicBase::getMass() { return getPxActor()->getMass(); }
PxVec3 SActorDynamicBase::getInertia() { return getPxActor()->getMassSpaceInertiaTensor(); }
PxTransform SActorDynamicBase::getCMassLocalPose() { return getPxActor()->getCMassLocalPose(); }

} // namespace sapien
