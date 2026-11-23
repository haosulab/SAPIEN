#include "sapien/physx/articulation_tendon.h"
#include "sapien/physx/articulation_link_component.h"

using namespace physx;

namespace sapien {
namespace physx {

PhysxArticulationSpatialTendon::PhysxArticulationSpatialTendon(
    std::shared_ptr<PhysxArticulation> articulation,
    std::vector<std::shared_ptr<PhysxArticulationLinkComponent>> const &links,
    std::vector<int> const &parents, std::vector<float> const &coefficients,
    std::vector<Vec3> const &linkOffsets, std::vector<float> const &restLengths,
    std::vector<std::tuple<float, float>> limits, float stiffness, float damping, float limitStiffness)
    : mArticulation(articulation) {

  for (auto &link : links) {
    if (link->getArticulation() != articulation) {
      throw std::runtime_error("failed to create tendon: the link does not belong to this articulation.");
    }
  }
  if (links.size() == 0) {
    throw std::runtime_error("failed to create tendon: no links provided");
  }
  if (links.size() != parents.size()) {
    throw std::runtime_error("failed to create tendon: link count does not match parent count");
  }
  if (links.size() != coefficients.size()) {
    throw std::runtime_error("failed to create tendon: link count does not match the coefficient count");
  }
  if (links.size() != linkOffsets.size()) {
    throw std::runtime_error("failed to create tendon: link count does not match link offset count");
  }
  if (links.size() != restLengths.size()) {
    throw std::runtime_error("failed to create tendon: link count does not match rest length count");
  }
  if (links.size() != limits.size()) {
    throw std::runtime_error("failed to create tendon: link count does not match limit count");
  }
  for (int i = 0; i < static_cast<int>(parents.size()); ++i) {
    if (parents.at(i) >= i) {
      throw std::runtime_error("tendon parent must have a smaller index");
    }
    if (i == 0 && parents.at(0) != -1) {
      throw std::runtime_error("parent for the first link must be -1");
    } else if (i != 0 && parents.at(i) < 0) {
      throw std::runtime_error("only parent of the first link can be -1");
    }
  }

  mLinkParents = parents;

  mLinks.clear();
  for (auto link : links) {
    mLinks.push_back(link);
  }

  std::vector<PxArticulationAttachment *> attachments;

  mTendon = articulation->getPxArticulation()->createSpatialTendon();
  mTendon->setStiffness(stiffness);
  mTendon->setDamping(damping);
  mTendon->setLimitStiffness(limitStiffness);
  // mTendon->setOffset(offset);

  for (uint32_t i = 0; i < links.size(); ++i) {
    auto parent = i != 0 ? attachments.at(parents.at(i)) : nullptr;
    auto attachment = mTendon->createAttachment(parent, coefficients[i],
                                                {linkOffsets[i].x, linkOffsets[i].y, linkOffsets[i].z},
                                                links[i]->getPxActor());
    // HACK: createAttachment seems to have a bug in relativeOffset, so we set again
    attachment->setRelativeOffset({linkOffsets[i].x, linkOffsets[i].y, linkOffsets[i].z});
    attachment->setRestLength(restLengths.at(i));
    auto [l, h] = limits.at(i);
    attachment->setLimitParameters({l, h});
    attachments.push_back(attachment);
  }
}

float PhysxArticulationSpatialTendon::getOffset() const { return mTendon->getOffset(); }
void PhysxArticulationSpatialTendon::setOffset(float offset) { mTendon->setOffset(offset); }

float PhysxArticulationSpatialTendon::getStiffness() const { return mTendon->getStiffness(); }
void PhysxArticulationSpatialTendon::setStiffness(float stiffness) { mTendon->setStiffness(stiffness); }

float PhysxArticulationSpatialTendon::getDamping() const { return mTendon->getDamping(); }
void PhysxArticulationSpatialTendon::setDamping(float damping) { mTendon->setDamping(damping); }

std::vector<std::shared_ptr<PhysxArticulationLinkComponent>>
PhysxArticulationSpatialTendon::getLinks() const {
  std::vector<std::shared_ptr<PhysxArticulationLinkComponent>> res;
  for (auto &link : mLinks) {
    res.push_back(link.lock());
  }
  return res;
}

std::vector<int> PhysxArticulationSpatialTendon::getLinkParents() const {
  return mLinkParents;
}

std::vector<float> PhysxArticulationSpatialTendon::getCoefficients() const {
  std::vector<PxArticulationAttachment *> attachments(mTendon->getNbAttachments());
  if (mTendon->getAttachments(attachments.data(), attachments.size()) != attachments.size()) {
    throw std::runtime_error("failed to get tendon attachments");
  }
  std::vector<float> res;
  for (auto &v : attachments) {
    res.push_back(v->getCoefficient());
  }
  return res;
}

std::vector<Vec3> PhysxArticulationSpatialTendon::getOffsets() const {
  std::vector<PxArticulationAttachment *> attachments(mTendon->getNbAttachments());
  if (mTendon->getAttachments(attachments.data(), attachments.size()) != attachments.size()) {
    throw std::runtime_error("failed to get tendon attachments");
  }
  std::vector<Vec3> res;
  for (auto &v : attachments) {
    auto offset = v->getRelativeOffset();
    res.push_back({offset.x, offset.y, offset.z});
  }
  return res;
}

} // namespace physx
} // namespace sapien
