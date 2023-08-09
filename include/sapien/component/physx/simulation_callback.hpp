#pragma once
#include "sapien/math/conversion.h"
#include <PxPhysicsAPI.h>
#include <map>
#include <memory>

namespace sapien {
namespace component {
using namespace physx;

class PhysxRigidBaseComponent;
class PhysxCollisionShape;

struct ContactPoint {
  Vec3 position;
  Vec3 normal;
  Vec3 impulse;
  float separation;
};

struct Contact {
  std::array<PhysxRigidBaseComponent *, 2> components;
  std::array<PhysxCollisionShape *, 2> shapes;
  std::vector<ContactPoint> points;
};

class DefaultEventCallback : public PxSimulationEventCallback {

public:
  void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs,
                 PxU32 nbPairs) override {
    for (uint32_t i = 0; i < nbPairs; ++i) {
      if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_LOST) {
        mContacts.erase({pairs[i].shapes[0], pairs[i].shapes[1]});
        continue;
      }

      if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_FOUND ||
          pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_PERSISTS) {
        void *a0 = pairHeader.actors[0]->userData;
        void *a1 = pairHeader.actors[1]->userData;
        if (!a0 || !a1) {
          continue;
        }
        std::unique_ptr<Contact> contact = std::make_unique<Contact>();
        contact->components[0] = static_cast<PhysxRigidBaseComponent *>(a0);
        contact->components[1] = static_cast<PhysxRigidBaseComponent *>(a1);
        contact->shapes[0] = static_cast<PhysxCollisionShape *>(pairs[i].shapes[0]->userData);
        contact->shapes[1] = static_cast<PhysxCollisionShape *>(pairs[i].shapes[1]->userData);

        std::vector<PxContactPairPoint> points(pairs[i].contactCount);
        pairs[i].extractContacts(points.data(), pairs[i].contactCount);

        for (auto &p : points) {
          contact->points.push_back({PxVec3ToVec3(p.position), PxVec3ToVec3(p.normal),
                                     PxVec3ToVec3(p.impulse), p.separation});
        }
        mContacts[{pairs[i].shapes[0], pairs[i].shapes[1]}] = std::move(contact);
      }
    }
  }

  void onAdvance(const PxRigidBody *const *bodyBuffer, const PxTransform *poseBuffer,
                 const PxU32 count) override {}
  void onWake(PxActor **actors, PxU32 count) override {}
  void onSleep(PxActor **actors, PxU32 count) override {}
  void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) override {}
  void onTrigger(PxTriggerPair *pairs, PxU32 count) override {}

  std::vector<Contact *> getContacts() const {
    std::vector<Contact *> contacts{};
    for (auto &it : mContacts) {
      contacts.push_back(it.second.get());
    }
    return contacts;
  }

private:
  std::map<std::pair<PxShape *, PxShape *>, std::unique_ptr<Contact>> mContacts;
};

} // namespace component
} // namespace sapien
