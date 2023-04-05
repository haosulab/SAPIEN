#include "sapien/sapien_camera.h"
#include "sapien/sapien_scene.h"
#include <glm/gtx/quaternion.hpp>

namespace sapien {

physx::PxTransform SLight::getPose() const { return getParentPose() * mLocalPose; };

void SLight::setPose(physx::PxTransform const &transform) {
  if (!mParent) {
    mLocalPose = transform;
    return;
  }
  throw std::runtime_error("failed to set pose: light has a parent, set local pose instead");
}

void SLight::setPosition(physx::PxVec3 position) {
  setPose(physx::PxTransform(position, getPose().q));
}
void SLight::setDirection(physx::PxVec3 direction) {
  auto x = direction.getNormalized();
  auto y = PxVec3{0.f, 1.f, 0.f};
  auto z = PxVec3{};
  if (std::abs(x.dot(y)) < 0.95) {
    z = x.cross(y).getNormalized();
  } else {
    z = x.cross({0.f, 0.f, 1.f}).getNormalized();
  }
  y = z.cross(x);
  setPose({getPose().p, PxQuat(PxMat33(x, y, z))});
}

PxTransform SLight::getParentPose() const {
  return mParent ? mParent->getPose() : physx::PxTransform{PxIdentity};
}

void SLight::setLocalPose(PxTransform const &pose) { mLocalPose = pose; }

void SLight::setParent(SActorBase *actor, bool keepPose) {
  PxTransform p2w{PxIdentity};
  mParent = actor;
  if (actor) {
    p2w = actor->getPose();
  }
  if (keepPose) {
    mLocalPose = p2w.getInverse() * getPose();
  }
}

void SLight::update() {
  static PxTransform gl2ros({0, 0, 0}, {-0.5, 0.5, 0.5, -0.5});
  getRendererLight()->setPose(getParentPose() * mLocalPose * gl2ros);
}

void SParallelogramLight::update() {
  PxVec3 edge0{0, mEdge0.x, mEdge0.y};
  PxVec3 edge1{0, mEdge1.x, mEdge1.y};
  PxTransform corner2center{-edge0 / 2.f - edge1 / 2.f, PxIdentity};

  getRendererLight()->setPose(getParentPose() * mLocalPose * corner2center);
}

} // namespace sapien
