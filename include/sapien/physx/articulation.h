#pragma once
#include "sapien/math/pose.h"
#include <Eigen/Eigen>
#include <PxPhysicsAPI.h>
#include <memory>
#include <vector>

#include <cereal/cereal.hpp>

namespace sapien {
class Entity;
class Scene;
namespace physx {
class PhysxArticulationLinkComponent;
class PhysxArticulationJoint;
class PhysxEngine;

class PhysxArticulation {
public:
  PhysxArticulation();

  void addLink(PhysxArticulationLinkComponent &link, PhysxArticulationLinkComponent *parent);
  void removeLink(PhysxArticulationLinkComponent &link);

  /** find descendants in order */
  std::vector<std::shared_ptr<PhysxArticulationLinkComponent>>
  findDescendants(std::shared_ptr<PhysxArticulationLinkComponent> link) const;

  ::physx::PxArticulationReducedCoordinate *getPxArticulation() const { return mPxArticulation; }

  uint32_t getDof();

  Eigen::VectorXf getQpos();
  void setQpos(Eigen::VectorXf const &q);
  Eigen::VectorXf getQvel();
  void setQvel(Eigen::VectorXf const &q);
  Eigen::VectorXf getQacc();
  void setQacc(Eigen::VectorXf const &q);
  Eigen::VectorXf getQf();
  void setQf(Eigen::VectorXf const &q);

  Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> getQLimit();

  Eigen::VectorXf computePassiveForce(bool gravity, bool coriolisAndCentrifugal);

  Pose getRootPose();
  Vec3 getRootLinearVelocity();
  Vec3 getRootAngularVelocity();

  void setRootPose(Pose const &pose);
  void setRootLinearVelocity(Vec3 const &v);
  void setRootAngularVelocity(Vec3 const &v);

  void setName(std::string const &name) { mName = name; }
  std::string getName() const { return mName; }

  std::shared_ptr<PhysxArticulationLinkComponent> getRoot() const;
  std::vector<std::shared_ptr<PhysxArticulationLinkComponent>> getLinks() const;
  std::vector<std::shared_ptr<PhysxArticulationJoint>> getJoints() const;
  std::vector<std::shared_ptr<PhysxArticulationJoint>> getActiveJoints() const;

  void internalNotifyAddToScene(PhysxArticulationLinkComponent *link, Scene &scene);
  void internalNotifyRemoveFromScene(PhysxArticulationLinkComponent *link, Scene &scene);
  void internalEnsureRemovedFromScene();
  void internalEnsureAddedToScene();
  void internalAddPxArticulationToScene(Scene &scene);

  void internalAddLinkAtIndex(PhysxArticulationLinkComponent &link,
                              PhysxArticulationLinkComponent *parent, uint32_t index);

  void createFixedTendon(std::vector<std::shared_ptr<PhysxArticulationLinkComponent>> const &chain,
                         std::vector<float> const &coefficients,
                         std::vector<float> const &recipCoefficients, float restLength,
                         float offset, float stiffness, float damping, float low, float high,
                         float limitStiffness);

  ~PhysxArticulation();

private:
  void updatePermutationMatrix();
  void checkDof(uint32_t n);
  void refreshLinkIndices();
  void syncPose();

  friend cereal::access;
  template <class Archive> void save(Archive &ar) const {
    uint32_t size = mLinks.size();
    ar(size);
  }
  template <class Archive> void load(Archive &ar) {
    uint32_t size;
    ar(size);
    mLinks.resize(size, 0);
  }

  std::shared_ptr<PhysxEngine> mEngine;

  ::physx::PxArticulationReducedCoordinate *mPxArticulation{};
  ::physx::PxArticulationCache *mCache{};

  // std::vector<std::weak_ptr<PhysxArticulationLinkComponent>> mLinks;

  /** links sorted in the added order, guaranteed to be topologically sorted */
  std::vector<PhysxArticulationLinkComponent *> mLinks;

  Eigen::PermutationMatrix<Eigen::Dynamic> mPermutationE2I;
  Eigen::PermutationMatrix<Eigen::Dynamic> mLinkPermutationE2I;

  Scene *mScene{};
  uint32_t mLinksAddedToScene{};
  std::string mName;
};

} // namespace physx
} // namespace sapien
