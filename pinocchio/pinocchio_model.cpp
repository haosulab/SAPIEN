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
#include "pinocchio_model.h"
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <memory>

#define PYBIND11_USE_SMART_HOLDER_AS_DEFAULT 1
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#define ASSERT(exp, info)                                                                         \
  if (!(exp)) {                                                                                   \
    throw std::runtime_error((info));                                                             \
  }

namespace sapien {
std::unique_ptr<PinocchioModel> PinocchioModel::fromURDFXML(std::string const &urdf,
                                                            Eigen::Vector3d gravity) {
  auto m = std::unique_ptr<PinocchioModel>(new PinocchioModel);
  pinocchio::urdf::buildModelFromXML(urdf, m->model);
  m->model.gravity = {gravity, Eigen::Vector3d{0, 0, 0}};
  m->data = pinocchio::Data(m->model);
  m->indexS2P.setIdentity(m->model.nv);
  return m;
}

Eigen::VectorXd PinocchioModel::posS2P(const Eigen::VectorXd &qext) {
  Eigen::VectorXd qint(model.nq);
  uint32_t count = 0;
  for (Eigen::Index N = 0; N < QIDX.size(); ++N) {
    auto start_idx = QIDX[N];
    switch (NQ[N]) {
    case 0:
      break;
    case 1:
      qint[start_idx] = qext[count];
      break;
    case 2:
      qint[start_idx] = std::cos(qext[count]);
      qint[start_idx + 1] = std::sin(qext[count]);
      break;
    default:
      throw std::runtime_error(
          "Unsupported joint in computation. Currently support: fixed, revolute, prismatic");
    }
    count += NV[N];
  }
  ASSERT(count == qext.size(), "posS2P failed");
  return qint;
}

Eigen::VectorXd PinocchioModel::posP2S(const Eigen::VectorXd &qint) {
  Eigen::VectorXd qext(model.nv);

  int count = 0;
  for (Eigen::Index N = 0; N < QIDX.size(); ++N) {
    auto start_idx = QIDX[N];
    switch (NQ[N]) {
    case 0:
      break;
    case 1:
      qext[count] = qint[start_idx];
      break;
    case 2:
      qext[count] = std::atan2(qint[start_idx + 1], qint[start_idx]);
      break;
    default:
      throw std::runtime_error(
          "Unsupported joint in computation. Currently support: fixed, revolute, prismatic");
    }
    count += NV[N];
  }
  ASSERT(count == model.nv, "posP2S failed");
  return qext;
}

void PinocchioModel::setJointOrder(std::vector<std::string> names) {
  Eigen::VectorXi v(model.nv);
  int count = 0;
  for (auto &name : names) {
    auto i = model.getJointId(name);
    if (i == static_cast<pinocchio::JointIndex>(model.njoints)) {
      throw std::invalid_argument("invalid names in setJointOrder");
    }
    auto size = model.nvs[i];
    auto qi = model.idx_vs[i];
    for (int s = 0; s < size; ++s) {
      v[count++] = qi + s;
    }
  }
  ASSERT(count == model.nv, "setJointOrder failed");
  indexS2P = Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>(v);

  QIDX = Eigen::VectorXi(names.size());
  NQ = Eigen::VectorXi(names.size());
  NV = Eigen::VectorXi(names.size());
  for (size_t N = 0; N < names.size(); ++N) {
    auto i = model.getJointId(names[N]);
    if (i == static_cast<pinocchio::JointIndex>(model.njoints)) {
      throw std::invalid_argument("invalid names in setJointOrder");
    }
    NQ[N] = model.nqs[i];
    NV[N] = model.nvs[i];
    QIDX[N] = model.idx_qs[i];
  }
}

void PinocchioModel::setLinkOrder(std::vector<std::string> names) {
  linkIdx2FrameIdx = {};
  Eigen::VectorXi v(names.size());
  for (auto &name : names) {
    auto i = model.getFrameId(name, pinocchio::BODY);
    if (i == static_cast<pinocchio::FrameIndex>(model.nframes)) {
      throw std::invalid_argument("invalid names in setLinkOrder");
    }
    linkIdx2FrameIdx.push_back(i);
  }
}

Eigen::MatrixXd PinocchioModel::getRandomConfiguration() {
  return posP2S(pinocchio::randomConfiguration(model));
}

void PinocchioModel::computeForwardKinematics(const Eigen::VectorXd &qpos) {
  pinocchio::forwardKinematics(model, data, posS2P(qpos));
}

Pose PinocchioModel::getLinkPose(uint32_t index) {
  ASSERT(index < linkIdx2FrameIdx.size(), "link index out of bound");
  auto frame = linkIdx2FrameIdx[index];
  auto parentJoint = model.frames[frame].parent;
  auto link2joint = model.frames[frame].placement;
  auto joint2world = data.oMi[parentJoint];

  auto link2world = joint2world * link2joint;
  auto P = link2world.translation();
  auto Q = Eigen::Quaterniond(link2world.rotation());
  return {Vec3(P.x(), P.y(), P.z()), Quat(Q.w(), Q.x(), Q.y(), Q.z())};
}

void PinocchioModel::computeFullJacobian(const Eigen::VectorXd &qpos) {
  pinocchio::computeJointJacobians(model, data, posS2P(qpos));
}

Eigen::Matrix<double, 6, Eigen::Dynamic> PinocchioModel::getLinkJacobian(uint32_t index,
                                                                         bool local) {
  ASSERT(index < linkIdx2FrameIdx.size(), "link index out of bound");
  auto frameIdx = linkIdx2FrameIdx[index];
  auto jointIdx = model.frames[frameIdx].parent;

  auto link2joint = model.frames[frameIdx].placement;
  auto joint2world = data.oMi[jointIdx];
  auto link2world = joint2world * link2joint;

  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model.nv);
  J.fill(0);

  pinocchio::getJointJacobian(model, data, jointIdx, pinocchio::ReferenceFrame::WORLD, J);
  if (local) {
    J = link2world.toActionMatrixInverse() * J;
  }
  // permute Jacobin to SAPIEN format
  return J * indexS2P;
}

Eigen::Matrix<double, 6, Eigen::Dynamic>
PinocchioModel::computeSingleLinkLocalJacobian(Eigen::VectorXd const &qpos, uint32_t index) {
  ASSERT(index < linkIdx2FrameIdx.size(), "link index out of bound");
  auto frameIdx = linkIdx2FrameIdx[index];
  auto jointIdx = model.frames[frameIdx].parent;
  auto link2joint = model.frames[frameIdx].placement;

  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model.nv);
  J.fill(0);
  pinocchio::computeJointJacobian(model, data, posS2P(qpos), jointIdx, J);
  return link2joint.toActionMatrixInverse() * J * indexS2P;
}

Eigen::MatrixXd PinocchioModel::computeGeneralizedMassMatrix(const Eigen::VectorXd &qpos) {
  pinocchio::crba(model, data, posS2P(qpos));
  data.M.triangularView<Eigen::StrictlyLower>() =
      data.M.transpose().triangularView<Eigen::StrictlyLower>();
  return indexS2P.transpose() * data.M * indexS2P;
}

Eigen::MatrixXd PinocchioModel::computeCoriolisMatrix(const Eigen::VectorXd &qpos,
                                                      const Eigen::VectorXd &qvel) {
  return indexS2P.transpose() *
         pinocchio::computeCoriolisMatrix(model, data, posS2P(qpos), indexS2P * qvel) * indexS2P;
}

Eigen::VectorXd PinocchioModel::computeInverseDynamics(const Eigen::VectorXd &qpos,
                                                       const Eigen::VectorXd &qvel,
                                                       const Eigen::VectorXd &qacc) {
  return indexS2P.transpose() *
         pinocchio::rnea(model, data, posS2P(qpos), indexS2P * qvel, indexS2P * qacc);
}

Eigen::VectorXd PinocchioModel::computeForwardDynamics(const Eigen::VectorXd &qpos,
                                                       const Eigen::VectorXd &qvel,
                                                       const Eigen::VectorXd &qf) {
  return indexS2P.transpose() *
         pinocchio::aba(model, data, posS2P(qpos), indexS2P * qvel, indexS2P * qf);
}

std::tuple<Eigen::VectorXd, bool, Eigen::Matrix<double, 6, 1>>
PinocchioModel::computeInverseKinematics(uint32_t linkIdx, Pose const &pose,
                                         Eigen::VectorXd const &initialQpos,
                                         Eigen::VectorXi const &activeQMask, double eps,
                                         int maxIter, double dt, double damp) {
  ASSERT(linkIdx < linkIdx2FrameIdx.size(), "link index out of bound");
  Eigen::VectorXd q;
  if (initialQpos.size() == 0) {
    q = pinocchio::neutral(model);
  } else {
    q = posS2P(initialQpos);
  }

  Eigen::VectorXd mask;
  if (activeQMask.size() > 0) {
    mask = indexS2P * activeQMask.cast<double>();
  } else {
    mask = Eigen::VectorXd(model.nv);
    for (int i = 0; i < model.nv; ++i) {
      mask(i) = 1.0;
    }
  }

  auto frameIdx = linkIdx2FrameIdx[linkIdx];
  auto jointIdx = model.frames[frameIdx].parent;
  pinocchio::SE3 l2w;
  l2w.translation({pose.p.x, pose.p.y, pose.p.z});
  l2w.rotation(Eigen::Quaterniond(pose.q.w, pose.q.x, pose.q.y, pose.q.z).toRotationMatrix());
  auto l2j = model.frames[frameIdx].placement;
  pinocchio::SE3 oMdes = l2w * l2j.inverse();

  pinocchio::Data::Matrix6x J(6, model.nv);
  J.setZero();
  bool success = false;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d err;
  Eigen::VectorXd v(model.nv);

  double minError = 1e10;
  Eigen::VectorXd bestQ = q;
  Vector6d bestErr;
  for (int i = 0;; i++) {
    pinocchio::forwardKinematics(model, data, q);
    const pinocchio::SE3 iMd = data.oMi[jointIdx].actInv(oMdes);
    err = pinocchio::log6(iMd).toVector();
    double errNorm = err.norm();
    if (errNorm < minError) {
      minError = errNorm;
      bestQ = q;
      bestErr = err;
    }
    if (errNorm < eps) {
      success = true;
      break;
    }
    if (i >= maxIter) {
      success = false;
      break;
    }
    pinocchio::computeJointJacobian(model, data, q, jointIdx, J);
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(iMd.inverse(), Jlog);
    J = -Jlog * J;
    J = J * mask.asDiagonal();

    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model, q, v * dt);
  }
  return {posP2S(bestQ), success, bestErr};
}

} // namespace sapien

using namespace sapien;
namespace py = pybind11;
PYBIND11_MODULE(pysapien_pinocchio, m) {
  auto PyPinocchioModel =
      py::class_<PinocchioModel>(m, "PinocchioModel");
  PyPinocchioModel
      .def(py::init([](std::string urdf, Eigen::Vector3d gravity) {
        return PinocchioModel::fromURDFXML(urdf, gravity);
      }))
      .def("set_link_order", &PinocchioModel::setLinkOrder)
      .def("set_joint_order", &PinocchioModel::setJointOrder)
      .def("compute_forward_kinematics", &PinocchioModel::computeForwardKinematics,
           "Compute and cache forward kinematics. After computation, use get_link_pose to "
           "retrieve the computed pose for a specific link.",
           py::arg("qpos"))
      .def("get_link_pose", &PinocchioModel::getLinkPose,
           "Given link index, get link pose (in articulation base frame) from forward kinematics. "
           "Must be called after compute_forward_kinematics.",
           py::arg("link_index"))
      .def("compute_inverse_kinematics", &PinocchioModel::computeInverseKinematics,
           R"doc(
Compute inverse kinematics with CLIK algorithm.
Details see https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_i-inverse-kinematics.html
Args:
    link_index: index of the link
    pose: target pose of the link in articulation base frame
    initial_qpos: initial qpos to start CLIK
    active_qmask: dof sized integer array, 1 to indicate active joints and 0 for inactive joints, default to all 1s
    max_iterations: number of iterations steps
    dt: iteration step "speed"
    damp: iteration step "damping"
Returns:
    result: qpos from IK
    success: whether IK is successful
    error: se3 norm error
)doc",
           py::arg("link_index"), py::arg("pose"), py::arg("initial_qpos") = Eigen::VectorXd{},
           py::arg("active_qmask") = Eigen::VectorXi{}, py::arg("eps") = 1e-4,
           py::arg("max_iterations") = 1000, py::arg("dt") = 0.1, py::arg("damp") = 1e-6)
      .def("compute_forward_dynamics", &PinocchioModel::computeForwardDynamics, py::arg("qpos"),
           py::arg("qvel"), py::arg("qf"))
      .def("compute_inverse_dynamics", &PinocchioModel::computeInverseDynamics, py::arg("qpos"),
           py::arg("qvel"), py::arg("qacc"))
      .def("compute_generalized_mass_matrix", &PinocchioModel::computeGeneralizedMassMatrix,
           py::arg("qpos"))
      .def("compute_coriolis_matrix", &PinocchioModel::computeCoriolisMatrix, py::arg("qpos"),
           py::arg("qvel"))

      .def("compute_full_jacobian", &PinocchioModel::computeFullJacobian,
           "Compute and cache Jacobian for all links", py::arg("qpos"))
      .def("get_link_jacobian", &PinocchioModel::getLinkJacobian,
           R"doc(
Given link index, get the Jacobian. Must be called after compute_full_jacobian.

Args:
  link_index: index of the link
  local: True for world(spatial) frame; False for link(body) frame
)doc",
           py::arg("link_index"), py::arg("local") = false)
      .def("compute_single_link_local_jacobian", &PinocchioModel::computeSingleLinkLocalJacobian,
           "Compute the link(body) Jacobian for a single link. It is faster than "
           "compute_full_jacobian followed by get_link_jacobian",
           py::arg("qpos"), py::arg("link_index"));
}