#ifdef _USE_PINOCCHIO
#include "pinocchio_model.h"
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#define ASSERT(exp, info)                                                                         \
  if (!(exp)) {                                                                                   \
    throw std::runtime_error((info));                                                             \
  }

namespace sapien {
std::unique_ptr<PinocchioModel> PinocchioModel::fromURDFXML(std::string const &urdf) {
  auto m = std::unique_ptr<PinocchioModel>(new PinocchioModel);
  pinocchio::urdf::buildModelFromXML(urdf, m->model);
  m->data = pinocchio::Data(m->model);
  m->indexS2P.setIdentity(m->model.nv);
  return m;
}

Eigen::VectorXd PinocchioModel::posS2P(const Eigen::VectorXd &qext) {
  Eigen::VectorXd qint(model.nq);
  uint32_t count = 0;
  for (size_t N = 0; N < QIDX.size(); ++N) {
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

  uint32_t count = 0;
  for (size_t N = 0; N < QIDX.size(); ++N) {
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
  size_t count = 0;
  for (auto &name : names) {
    auto i = model.getJointId(name);
    if (i == model.njoints) {
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
    if (i == model.njoints) {
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
    if (i == model.nframes) {
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

physx::PxTransform PinocchioModel::getLinkPose(uint32_t index) {
  ASSERT(index < linkIdx2FrameIdx.size(), "link index out of bound");
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
PinocchioModel::computeInverseKinematics(uint32_t linkIdx, physx::PxTransform const &pose,
                                         double eps, int maxIter, double dt, double damp) {
  ASSERT(linkIdx < linkIdx2FrameIdx.size(), "link index out of bound");
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
  return {posP2S(q), success, err};
}

} // namespace sapien

#endif
