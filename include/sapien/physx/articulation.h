/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once
#include "sapien/math/pose.h"
#include <Eigen/Eigen>
#include <PxPhysicsAPI.h>
#include <memory>
#include <vector>

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

  Eigen::Matrix<float, Eigen::Dynamic, 6, Eigen::RowMajor> getLinkIncomingJointForces();


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
  std::vector<std::shared_ptr<PhysxArticulationLinkComponent>> getLinksAdditionOrder() const;
  std::vector<std::shared_ptr<PhysxArticulationLinkComponent>> getLinks() const;
  std::vector<std::shared_ptr<PhysxArticulationJoint>> getJoints() const;
  std::vector<std::shared_ptr<PhysxArticulationJoint>> getActiveJoints() const;

  void setSolverPositionIterations(uint32_t count);
  void setSolverVelocityIterations(uint32_t count);
  void setSleepThreshold(float threshold);
  uint32_t getSolverPositionIterations() const;
  uint32_t getSolverVelocityIterations() const;
  float getSleepThreshold() const;

  void internalNotifyAddToScene(PhysxArticulationLinkComponent *link, Scene &scene);
  void internalNotifyRemoveFromScene(PhysxArticulationLinkComponent *link, Scene &scene);
  void internalEnsureRemovedFromScene();
  void internalEnsureAddedToScene();
  void internalAddPxArticulationToScene(Scene &scene);

  void createFixedTendon(std::vector<std::shared_ptr<PhysxArticulationLinkComponent>> const &chain,
                         std::vector<float> const &coefficients,
                         std::vector<float> const &recipCoefficients, float restLength,
                         float offset, float stiffness, float damping, float low, float high,
                         float limitStiffness);

  int getGpuIndex() const;

  ~PhysxArticulation();

private:
  void checkDof(uint32_t n);
  void syncPose();

  std::shared_ptr<PhysxEngine> mEngine;

  ::physx::PxArticulationReducedCoordinate *mPxArticulation{};
  ::physx::PxArticulationCache *mCache{};

  // links sorted in the added order, guaranteed to be topologically sorted
  std::vector<PhysxArticulationLinkComponent *> mLinks;

  Scene *mScene{};
  uint32_t mLinksAddedToScene{};
  std::string mName;
};

} // namespace physx
} // namespace sapien
