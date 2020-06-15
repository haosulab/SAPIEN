#include "articulation/articulation_builder.h"
#include "articulation/sapien_articulation.h"
#include "articulation/sapien_kinematic_articulation.h"
#include "articulation/sapien_kinematic_joint.h"
#include "articulation/sapien_link.h"
#include "articulation/urdf_loader.h"
#include "renderer/optifuser_controller.h"
#include "renderer/optifuser_renderer.h"
#include "sapien_scene.h"
#include "simulation.h"
#include <iostream>

#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <chrono>

using namespace sapien;

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance =
                                   typename MatT::Scalar{1e-4}) // choose appropriately
{
  typedef typename MatT::Scalar Scalar;
  auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto &singularValues = svd.singularValues();
  Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(
      mat.cols(), mat.rows());
  singularValuesInv.setZero();
  for (unsigned int i = 0; i < singularValues.size(); ++i) {
    if (singularValues(i) > tolerance) {
      singularValuesInv(i, i) = Scalar{1} / singularValues(i);
    } else {
      singularValuesInv(i, i) = Scalar{0};
    }
  }
  return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

void printPose(physx::PxTransform const &T) {
  printf("p: %.4f %.4f %.4f, q: %.4f %.4f %.4f %.4f\n", T.p.x, T.p.y, T.p.z, T.q.w, T.q.x, T.q.y,
         T.q.z);
}

bool poseClose(physx::PxTransform const &T1, physx::PxTransform const &T2, float tol = 1e-3) {
  return std::abs(T1.p.x - T2.p.x) < tol && std::abs(T1.p.y - T2.p.y) < tol &&
         std::abs(T1.p.z - T2.p.z) < tol &&
         ((std::abs(T1.q.w - T2.q.w) < tol && std::abs(T1.q.x - T2.q.x) < tol &&
           std::abs(T1.q.y - T2.q.y) < tol && std::abs(T1.q.z - T2.q.z) < tol) ||
          (std::abs(T1.q.w + T2.q.w) < tol && std::abs(T1.q.x + T2.q.x) < tol &&
           std::abs(T1.q.y + T2.q.y) < tol && std::abs(T1.q.z + T2.q.z) < tol));
}

#define PI (3.141592653589793238462643383279502884f)
std::unique_ptr<ArticulationBuilder> createAntBuilder(SScene &scene) {
  Renderer::PxrMaterial copper = {};
  copper.base_color = {0.975, 0.453, 0.221, 1};
  copper.metallic = 1.f;
  copper.roughness = 0.7f;
  copper.specular = 0.5f;

  auto builder = scene.createArticulationBuilder();
  auto body = builder->createLinkBuilder();
  body->addSphereShape({{0, 0, 0}, PxIdentity}, 0.25);
  body->addSphereVisualWithMaterial({{0, 0, 0}, PxIdentity}, 0.25, copper);
  body->addCapsuleShape({{0.141, 0, 0}, PxIdentity}, 0.08, 0.141);
  body->addCapsuleVisualWithMaterial({{0.141, 0, 0}, PxIdentity}, 0.08, 0.141, copper);
  body->addCapsuleShape({{-0.141, 0, 0}, PxIdentity}, 0.08, 0.141);
  body->addCapsuleVisualWithMaterial({{-0.141, 0, 0}, PxIdentity}, 0.08, 0.141, copper);
  body->addCapsuleShape({{0, 0.141, 0}, physx::PxQuat(PI / 2, {0, 0, 1})}, 0.08, 0.141);
  body->addCapsuleVisualWithMaterial({{0, 0.141, 0}, physx::PxQuat(PI / 2, {0, 0, 1})}, 0.08,
                                     0.141, copper);
  body->addCapsuleShape({{0, -0.141, 0}, physx::PxQuat(PI / 2, {0, 0, 1})}, 0.08, 0.141);
  body->addCapsuleVisualWithMaterial({{0, -0.141, 0}, physx::PxQuat(PI / 2, {0, 0, 1})}, 0.08,
                                     0.141, copper);
  body->setName("body");

  auto l1 = builder->createLinkBuilder(body);
  l1->setName("l1");
  l1->setJointName("j1");
  l1->setJointProperties(PxArticulationJointType::eREVOLUTE, {{-0.5236, 0.5236}},
                         {{0.282, 0, 0}, {0, 0.7071068, 0, 0.7071068}},
                         {{0.141, 0, 0}, {0, -0.7071068, 0, 0.7071068}});
  l1->addCapsuleShape({{0, 0, 0}, PxIdentity}, 0.08, 0.141);
  l1->addCapsuleVisualWithMaterial({{0, 0, 0}, PxIdentity}, 0.08, 0.141, copper);

  auto l2 = builder->createLinkBuilder(body);
  l2->setName("l2");
  l2->setJointName("j2");
  l2->setJointProperties(PxArticulationJointType::eREVOLUTE, {{-0.5236, 0.5236}},
                         {{-0.282, 0, 0}, {0.7071068, 0, -0.7071068, 0}},
                         {{0.141, 0, 0}, {0, -0.7071068, 0, 0.7071068}});
  l2->addCapsuleShape({{0, 0, 0}, PxIdentity}, 0.08, 0.141);
  l2->addCapsuleVisualWithMaterial({{0, 0, 0}, PxIdentity}, 0.08, 0.141, copper);

  auto l3 = builder->createLinkBuilder(body);
  l3->setName("l3");
  l3->setJointName("j3");
  l3->setJointProperties(PxArticulationJointType::eREVOLUTE, {{-0.5236, 0.5236}},
                         {{0, 0.282, 0}, {-0.5, 0.5, 0.5, 0.5}},
                         {{0.141, 0, 0}, {0, -0.7071068, 0, 0.7071068}});
  l3->addCapsuleShape({{0, 0, 0}, PxIdentity}, 0.08, 0.141);
  l3->addCapsuleVisualWithMaterial({{0, 0, 0}, PxIdentity}, 0.08, 0.141, copper);

  auto l4 = builder->createLinkBuilder(body);
  l4->setName("l4");
  l4->setJointName("j4");
  l4->setJointProperties(PxArticulationJointType::eREVOLUTE, {{-0.5236, 0.5236}},
                         {{0, -0.282, 0}, {0.5, 0.5, -0.5, 0.5}},
                         {{0.141, 0, 0}, {0, -0.7071068, 0, 0.7071068}});
  l4->addCapsuleShape({{0, 0, 0}, PxIdentity}, 0.08, 0.141);
  l4->addCapsuleVisualWithMaterial({{0, 0, 0}, PxIdentity}, 0.08, 0.141, copper);

  auto f1 = builder->createLinkBuilder(l1);
  f1->setName("f1");
  f1->setJointName("j11");
  f1->setJointProperties(PxArticulationJointType::eREVOLUTE, {{0, 1.222}},
                         {{-0.141, 0, 0}, {0.7071068, 0.7071068, 0, 0}},
                         {{0.282, 0, 0}, {0.7071068, 0.7071068, 0, 0}});
  f1->addCapsuleShape({{0, 0, 0}, PxIdentity}, 0.08, 0.282);
  f1->addCapsuleVisualWithMaterial({{0, 0, 0}, PxIdentity}, 0.08, 0.282, copper);

  auto f2 = builder->createLinkBuilder(l2);
  f2->setName("f2");
  f2->setJointName("j21");
  f2->setJointProperties(PxArticulationJointType::eREVOLUTE, {{0, 1.222}},
                         {{-0.141, 0, 0}, {0.7071068, 0.7071068, 0, 0}},
                         {{0.282, 0, 0}, {0.7071068, 0.7071068, 0, 0}});
  f2->addCapsuleShape({{0, 0, 0}, PxIdentity}, 0.08, 0.282);
  f2->addCapsuleVisualWithMaterial({{0, 0, 0}, PxIdentity}, 0.08, 0.282, copper);

  auto f3 = builder->createLinkBuilder(l3);
  f3->setName("f3");
  f3->setJointName("j31");
  f3->setJointProperties(PxArticulationJointType::eREVOLUTE, {{0, 1.222}},
                         {{-0.141, 0, 0}, {0.7071068, 0.7071068, 0, 0}},
                         {{0.282, 0, 0}, {0.7071068, 0.7071068, 0, 0}});
  f3->addCapsuleShape({{0, 0, 0}, PxIdentity}, 0.08, 0.282);
  f3->addCapsuleVisualWithMaterial({{0, 0, 0}, PxIdentity}, 0.08, 0.282, copper);

  auto f4 = builder->createLinkBuilder(l4);
  f4->setName("f4");
  f4->setJointName("j41");
  f4->setJointProperties(PxArticulationJointType::eREVOLUTE, {{0, 1.222}},
                         {{-0.141, 0, 0}, {0.7071068, 0.7071068, 0, 0}},
                         {{0.282, 0, 0}, {0.7071068, 0.7071068, 0, 0}});
  f4->addCapsuleShape({{0, 0, 0}, PxIdentity}, 0.08, 0.282);
  f4->addCapsuleVisualWithMaterial({{0, 0, 0}, PxIdentity}, 0.08, 0.282, copper);

  return builder;
}

int main() {
  Simulation sim;
  Renderer::OptifuserRenderer renderer;
  sim.setRenderer(&renderer);
  Renderer::OptifuserController controller(&renderer);

  controller.showWindow();

  auto s0 = sim.createScene();
  s0->setTimestep(1 / 480.f);

  s0->addGround(-2);

  auto loader = s0->createURDFLoader();
  loader->fixRootLink = 1;
  auto a = loader->loadKinematic("../assets_local/robot/panda.urdf");
  // auto a = loader->load("../assets_local/robot/tmp.urdf");
  // auto a = loader->loadKinematic("../assets_local/old_kinova/old_kinova_parallel.urdf");
  // auto b = loader->loadKinematic("../assets/tmp.urdf");

  // std::cout << a->exportKinematicsChainAsURDF(true) << std::endl;

  // std::vector<float> q = {0, 0, 0, -1.5, 0, 1.5, 0.7, 0.01, 0.01};
  // a->setQpos(q);
  // s0->step();
  // q = a->getQpos();

  // Eigen::VectorXd q2(9);
  // for (uint32_t i = 0; i < 9; ++i) {
  //   q2(i) = q[i];
  // }
  // std::cout << q2 << std::endl;

  // std::cout << a->exportKinematicsChainAsURDF(true) << std::endl;

  auto pm = a->createPinocchioModel();
  auto q2 = pm->getRandomConfiguration();
  std::cout << q2 << std::endl;

  if (q2.size() != a->dof()) {
    throw std::runtime_error("DOF mismatch");
  }

  std::vector<float> q;
  for (uint32_t i = 0; i < q2.size(); ++i) {
    q.push_back(q2(i));
  }
  a->setQpos(q);
  s0->step();

  pm->computeForwardKinematics(q2);
  for (uint32_t i = 0; i < a->getBaseLinks().size(); ++i) {
    bool close = poseClose(pm->getLinkPose(i), a->getBaseLinks()[i]->getPose());
    if (!close) {
      throw std::runtime_error("pose not close");
    }
  }

  s0->setAmbientLight({0.3, 0.3, 0.3});
  s0->setShadowLight({1, -1, -1}, {.5, .5, 0.4});
  controller.setCurrentScene(s0.get());

  pm->computeFullJacobian(q2);
  
  // auto start = std::chrono::high_resolution_clock::now(); 
  // for (uint32_t i = 0; i < 1000; ++i) {
  //   pinocchio::computeJointKinematicHessians(model, data);
  //   Eigen::Tensor<double, 3> tensor;
  //   tensor.resize(9, 6, 9);
  //   pinocchio::getJointKinematicHessian(model, data, 5, pinocchio::ReferenceFrame::WORLD, tensor);
  // }
  // auto end = std::chrono::high_resolution_clock::now(); 
  // std::cout << "ms: " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << std::endl;

  while (!controller.shouldQuit()) {
    for (int i = 0; i < 8; ++i) {
      // auto qpos = a->getQpos();
      // auto qvel = a->getQvel();
      // Eigen::VectorXd qpos2(q.size());
      // Eigen::VectorXd qvel2(q.size());
      // Eigen::VectorXd qacc2(qpos.size());
      // qacc2.fill(0);
      // for (uint32_t i = 0; i < q.size(); ++i) {
      //   qpos2(i) = qpos[i];
      //   qvel2(i) = qvel[i];
      // }
      // auto qf2 = pm->computeInverseDynamics(qpos2, qvel2, qacc2);
      // std::vector<PxReal> qf;
      // for (uint32_t i = 0; i < q.size(); ++i) {
      //   qf.push_back(qf2[i]);
      // }

      // pm->computeFullJacobian(q2);

      // auto j = pm->computeSingleLinkLocalJacobian(q2, 9);
      // Eigen::VectorXd v(6);
      // v << 0, 0, 0, 0, 0, 1;
      // auto qvel = pseudoinverse(j) * v;
      // for (uint32_t i = 0; i < q.size(); ++i) {
      //   q[i] = qvel[i] / 10;
      // }
      // a->setQvel(q);

      // a->setQf(qf);

      s0->step();
    }
    s0->updateRender();
    controller.render();
  }

  return 0;
}
