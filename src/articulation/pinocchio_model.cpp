#ifdef _USE_PINOCCHIO
#include "pinocchio_model.h"
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace sapien {
std::unique_ptr<PinocchioModel> PinocchioModel::fromURDFXML(std::string const &urdf) {
  auto m = std::unique_ptr<PinocchioModel>(new PinocchioModel);
  pinocchio::urdf::buildModelFromXML(urdf, m->model);
  m->data = pinocchio::Data(m->model);
  m->indexS2P.setIdentity(m->model.nq);
  return m;
}

void PinocchioModel::setJointOrder(std::vector<std::string> names) {
  Eigen::VectorXi v(model.nq);
  size_t count = 0;
  for (auto &name : names) {
    auto i = model.getJointId(name);
    if (i == model.njoints) {
      throw std::invalid_argument("invalid names in setJointOrder");
    }
    auto size = model.nqs[i];
    auto qi = model.idx_qs[i];
    for (int s = 0; s < size; ++s) {
      v[count++] = qi + s;
    }
  }
  assert(count == model.nq);

  indexS2P = Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>(v);
}

void PinocchioModel::setLinkOrder(std::vector<std::string> names) {
  linkIdx2FrameIdx = {};
  Eigen::VectorXi v(names.size());
  for (auto &name : names) {
    auto i = model.getFrameId(name, pinocchio::BODY);
    if (i == model.nframes) {
      throw std::invalid_argument("invalid names in setLinkOrder");
    }
    linkIdx2FrameIdx.push_back(i);
  }
}

Eigen::MatrixXd PinocchioModel::getRandomConfiguration() {
  return indexS2P.transpose() * pinocchio::randomConfiguration(model);
}

void PinocchioModel::computeForwardKinematics(const Eigen::VectorXd &qpos) {
  pinocchio::forwardKinematics(model, data, indexS2P * qpos);
}

physx::PxTransform PinocchioModel::getLinkPose(uint32_t index) {
  assert(index < linkIdx2FrameIdx.size());
  auto frame = linkIdx2FrameIdx[index];
  auto parentJoint = model.frames[frame].parent;
  auto link2joint = model.frames[frame].placement;
  auto joint2world = data.oMi[parentJoint];

  auto link2world = joint2world * link2joint;
  auto P = link2world.translation();
  auto Q = Eigen::Quaterniond(link2world.rotation());
  return {physx::PxVec3(P.x(), P.y(), P.z()), physx::PxQuat(Q.x(), Q.y(), Q.z(), Q.w())};
}

void PinocchioModel::computeFullJacobian(const Eigen::VectorXd &qpos) {
  pinocchio::computeJointJacobians(model, data, indexS2P * qpos);
}

Eigen::Matrix<double, 6, Eigen::Dynamic> PinocchioModel::getLinkJacobian(uint32_t index,
                                                                         bool local) {
  assert(index < linkIdx2FrameIdx.size());
  auto frameIdx = linkIdx2FrameIdx[index];
  auto jointIdx = model.frames[frameIdx].parent;

  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model.nq);
  J.fill(0);
  pinocchio::getJointJacobian(
      model, data, jointIdx,
      local ? pinocchio::ReferenceFrame::LOCAL : pinocchio::ReferenceFrame::WORLD, J);
  // permute jacobian to SAPIEN format
  return J * indexS2P;
}

Eigen::MatrixXd PinocchioModel::computeGeneralizedMassMatrix(const Eigen::VectorXd &qpos) {
  pinocchio::crba(model, data, indexS2P * qpos);
  data.M.triangularView<Eigen::StrictlyLower>() =
      data.M.transpose().triangularView<Eigen::StrictlyLower>();
  return indexS2P.transpose() * data.M * indexS2P;
}

Eigen::MatrixXd PinocchioModel::computeCoriolisMatrix(const Eigen::VectorXd &qpos,
                                                      const Eigen::VectorXd &qvel) {
  return indexS2P.transpose() *
         pinocchio::computeCoriolisMatrix(model, data, indexS2P * qpos, indexS2P * qvel) *
         indexS2P;
}

Eigen::VectorXd PinocchioModel::computeInverseDynamics(const Eigen::VectorXd &qpos,
                                                       const Eigen::VectorXd &qvel,
                                                       const Eigen::VectorXd &qacc) {
  return indexS2P.transpose() *
         pinocchio::rnea(model, data, indexS2P * qpos, indexS2P * qvel, indexS2P * qacc);
}

Eigen::VectorXd PinocchioModel::computeForwardDynamics(const Eigen::VectorXd &qpos,
                                                       const Eigen::VectorXd &qvel,
                                                       const Eigen::VectorXd &qf) {
  return indexS2P.transpose() *
         pinocchio::aba(model, data, indexS2P * qpos, indexS2P * qvel, indexS2P * qf);
}

std::tuple<Eigen::VectorXd, bool, Eigen::Matrix<double, 6, 1>>
PinocchioModel::computeInverseKinematics(uint32_t linkIdx, physx::PxTransform const &pose,
                                         double eps, int maxIter, double dt, double damp) {
  assert(index < linkIdx2FrameIdx.size());
  auto frameIdx = linkIdx2FrameIdx[linkIdx];
  auto jointIdx = model.frames[frameIdx].parent;
  pinocchio::SE3 l2w;
  l2w.translation({pose.p.x, pose.p.y, pose.p.z});
  l2w.rotation(Eigen::Quaterniond(pose.q.w, pose.q.x, pose.q.y, pose.q.z).toRotationMatrix());
  auto l2j = model.frames[frameIdx].placement;
  pinocchio::SE3 oMdes = l2w * l2j.inverse();

  Eigen::VectorXd q = pinocchio::neutral(model);

  pinocchio::Data::Matrix6x J(6, model.nv);
  J.setZero();
  bool success = false;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d err;
  Eigen::VectorXd v(model.nv);

  for (int i = 0;; i++) {
    pinocchio::forwardKinematics(model, data, q);
    const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[jointIdx]);
    err = pinocchio::log6(dMi).toVector();
    if (err.norm() < eps) {
      success = true;
      break;
    }
    if (i >= maxIter) {
      success = false;
      break;
    }
    pinocchio::computeJointJacobian(model, data, q, jointIdx, J);
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model, q, v * dt);
  }
  return {q, success, err};
}

} // namespace sapien

#endif
