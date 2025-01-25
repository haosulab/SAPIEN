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
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace sapien {

class PinocchioModel {
  pinocchio::Model model{};
  pinocchio::Data data{};

  /** pinocchio_qpos = indexS2P * sapien_qpos
   * Left multiplication permutes rows of SAPIEN order to Pinocchio order
   * Right multiplication permutes columns of Pinocchio order to SAPIEN order
   */
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> indexS2P;

  Eigen::VectorXi QIDX;
  Eigen::VectorXi NQ;
  Eigen::VectorXi NV;

  Eigen::VectorXd posS2P(const Eigen::VectorXd &qpos);
  Eigen::VectorXd posP2S(const Eigen::VectorXd &qpos);

  std::vector<int> linkIdx2FrameIdx;

public:
  static std::unique_ptr<PinocchioModel> fromURDFXML(std::string const &urdf,
                                                     Eigen::Vector3d gravity);

  PinocchioModel(PinocchioModel const &other) = delete;
  PinocchioModel &operator=(PinocchioModel const &other) = delete;
  ~PinocchioModel() = default;

  inline pinocchio::Model &getInternalModel() { return model; }
  inline pinocchio::Data &getInternalData() { return data; }

private:
  inline PinocchioModel(){};

public:
  /** initialize internal permutation matrices by providing joint name*/
  void setJointOrder(std::vector<std::string> names);
  void setLinkOrder(std::vector<std::string> names);

  /** generate a random qpos */
  Eigen::MatrixXd getRandomConfiguration();

  /** compute and cache the forward kinematics */
  void computeForwardKinematics(const Eigen::VectorXd &qpos);

  /** get link pose
   *
   *  must be called after computeForwardKinematics
   */
  Pose getLinkPose(uint32_t index);

  void computeFullJacobian(const Eigen::VectorXd &qpos);

  /** get Jacobian for a link
   *
   *  must be called after computeFullJacobian
   */
  Eigen::Matrix<double, 6, Eigen::Dynamic> getLinkJacobian(uint32_t index, bool local = false);

  /** compute the local Jacobian for a single link
   *
   */
  Eigen::Matrix<double, 6, Eigen::Dynamic>
  computeSingleLinkLocalJacobian(Eigen::VectorXd const &qpos, uint32_t index);

  /** M in Ma + Cv + g = t
   *
   * Composite rigid body algorithm
   * Note: only upper triangular part is computed
   */
  Eigen::MatrixXd computeGeneralizedMassMatrix(const Eigen::VectorXd &qpos);

  /** C in Ma + Cv + g = t
   *
   * Recursive Newton-Euler algorithm
   */
  Eigen::MatrixXd computeCoriolisMatrix(const Eigen::VectorXd &qpos, const Eigen::VectorXd &qvel);

  /** Ma + Cv + g = t
   *
   * Recursive Newton-Euler algorithm
   * Note: to compute g, call computeInverseDynamics(qpos, 0, 0)
   *       to compute all passive forces, call computeInverseDynamics(qpos, qvel, 0)
   */
  Eigen::VectorXd computeInverseDynamics(const Eigen::VectorXd &qpos, const Eigen::VectorXd &qvel,
                                         const Eigen::VectorXd &qacc);

  /** Ma + Cv + g = t
   *
   * Articulated-body algorithm
   */
  Eigen::VectorXd computeForwardDynamics(const Eigen::VectorXd &qpos, const Eigen::VectorXd &qvel,
                                         const Eigen::VectorXd &qf);

  /** Numerical IK clik algorithm
   *  computes the numerical IK for a given link
   *  https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_i-inverse-kinematics.html
   *
   */
  std::tuple<Eigen::VectorXd, bool, Eigen::Matrix<double, 6, 1>>
  computeInverseKinematics(uint32_t linkIdx, Pose const &pose,
                           Eigen::VectorXd const &initialQpos = {},
                           Eigen::VectorXi const &activeJointIndices = {}, double eps = 1e-4,
                           int maxIter = 1000, double dt = 1e-1, double damp = 1e-6);
};

}; // namespace sapien