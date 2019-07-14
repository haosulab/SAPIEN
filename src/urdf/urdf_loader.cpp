#include "urdf/urdf_loader.h"
#include "articulation_builder.h"
#include "mesh_registry.h"
#include "simulation.h"
#include <eigen3/Eigen/Eigenvalues>
#include <map>
#include <tinyxml2.h>

using namespace tinyxml2;
using namespace physx;
using namespace MeshUtil;

PxTransform poseFromOrigin(const Origin &origin) {
  // TODO: Check this conversion!
  PxQuat q = PxQuat(origin.rpy.x, {1, 0, 0}) * PxQuat(origin.rpy.y, {0, 1, 0}) *
             PxQuat(origin.rpy.z, {0, 0, 1});

  return PxTransform(origin.xyz, q);
}

URDFLoader::URDFLoader(PxSimulation &simulation) : mSimulation(simulation) {}

struct LinkTreeNode {
  Link *link;
  Joint *joint;
  LinkTreeNode *parent;
  PxTransform worldPose;
  std::vector<LinkTreeNode *> children;
};

void URDFLoader::load(const std::string &filename) {
  XMLDocument doc;
  doc.LoadFile(filename.c_str());
  XMLPrinter printer;
  if (strcmp("robot", doc.RootElement()->Name()) == 0) {
    robot = std::make_unique<Robot>(*doc.RootElement());
  } else {
    std::cerr << "Invalid URDF: root is not <robot/>" << std::endl;
    exit(1);
  }

  std::vector<std::unique_ptr<LinkTreeNode>> treeNodes;
  std::map<std::string, LinkTreeNode *> linkName2treeNode;
  std::map<std::string, LinkTreeNode *> jointName2treeNode;

  // process link
  for (const auto &link : robot->link_array) {
    std::string name = link->name;
    if (linkName2treeNode.find(name) != linkName2treeNode.end()) {
      std::cerr << "Duplicated link name: " << name << std::endl;
      exit(1);
    }
    std::unique_ptr<LinkTreeNode> treeNode = std::make_unique<LinkTreeNode>();
    treeNode->link = link.get();
    treeNode->joint = nullptr;
    treeNode->parent = nullptr;
    treeNode->worldPose = {{0, 0, 0}, PxIdentity};
    treeNode->children = std::vector<LinkTreeNode *>();
    linkName2treeNode[name] = treeNode.get();
    treeNodes.push_back(std::move(treeNode));
  }

  // process joint
  for (const auto &joint : robot->joint_array) {
    std::string name = joint->name;
    if (jointName2treeNode.find(name) != jointName2treeNode.end()) {
      std::cerr << "Duplicated joint name: " << name << std::endl;
      exit(1);
    }
    std::string parent = joint->parent->link;
    std::string child = joint->child->link;
    LinkTreeNode *parentNode = linkName2treeNode[parent];
    LinkTreeNode *childNode = linkName2treeNode[child];
    if (childNode->parent) {
      std::cerr << "Error: link with name " << child << " has multiple parents." << std::endl;
      exit(1);
    }
    childNode->joint = joint.get();
    childNode->parent = parentNode;
    parentNode->children.push_back(childNode);
    jointName2treeNode[name] = childNode;
  }

  // find root
  LinkTreeNode *root = nullptr;
  for (const auto &node : treeNodes) {
    if (!node->parent) {
      if (root) {
        std::cerr << "Error: multiple root nodes detected." << std::endl;
        exit(1);
      }
      root = node.get();
    }
  }

  // check tree property
  std::map<LinkTreeNode *, bool> link2visited;
  std::vector<LinkTreeNode *> stack = {root};
  while (!stack.empty()) {
    LinkTreeNode *current = stack.back();
    stack.pop_back();
    if (link2visited.find(current) != link2visited.end()) {
      std::cerr << "Error: kinetic loop detected." << std::endl;
      exit(1);
    }
    link2visited[current] = true;
    for (auto c : current->children) {
      stack.push_back(c);
    }
  }

  std::map<LinkTreeNode *, physx::PxArticulationLink *> treeNode2pLink;
  PxArticulationBuilder builder(&mSimulation);
  stack = {root};
  while (!stack.empty()) {
    LinkTreeNode *current = stack.back();
    stack.pop_back();

    // create link and set its parent
    physx::PxArticulationLink *pLink = current->parent ? treeNode2pLink[current->parent] : nullptr;
    treeNode2pLink[current] = builder.addLink(pLink);

    // inertial
    const Inertial &currentInertial = *current->link->inertial;
    const PxTransform tInertial2Link = poseFromOrigin(*currentInertial.origin);

    const Inertia &currentInertia = *currentInertial.inertia;

    Eigen::Matrix3f inertia;
    inertia(0, 0) = currentInertia.ixx;
    inertia(1, 1) = currentInertia.iyy;
    inertia(2, 2) = currentInertia.izz;
    inertia(0, 1) = currentInertia.ixy;
    inertia(1, 0) = currentInertia.ixy;
    inertia(0, 2) = currentInertia.ixz;
    inertia(2, 0) = currentInertia.ixz;
    inertia(1, 2) = currentInertia.iyz;
    inertia(2, 1) = currentInertia.iyz;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
    es.compute(inertia);
    auto eigs = es.eigenvalues();
    auto quat = Eigen::Quaternionf(es.eigenvectors());

    const PxTransform tInertia2Inertial =
        PxTransform(PxVec3(0), PxQuat(quat.x(), quat.y(), quat.z(), quat.w()));

    const PxTransform tInertia2Link = tInertial2Link * tInertia2Inertial;

    treeNode2pLink[current]->setCMassLocalPose(tInertia2Link);
    treeNode2pLink[current]->setMassSpaceInertiaTensor({eigs.x(), eigs.y(), eigs.z()});
    treeNode2pLink[current]->setMass(currentInertial.mass->value);

    // visual
    for (const auto &visual : current->link->visual_array) {
      const PxTransform tVisual2Link = poseFromOrigin(*visual->origin);
      // TODO: register the initial position of this visual and ...
      switch (visual->geometry->type) {
      case Geometry::BOX:
        break;
      case Geometry::CYLINDER:
        break;
      case Geometry::SPHERE:
        break;
      case Geometry::MESH:
        break;
        // TODO: finish
      }
      // TODO: material
    }

    // collision
    for (const auto &collision : current->link->collision_array) {
      const PxTransform tCollision2Link = poseFromOrigin(*collision->origin);
      PxShape *shape;
      // TODO: add physical material support
      PxMaterial *material = mSimulation.mDefaultMaterial;
      switch (collision->geometry->type) {
      case Geometry::BOX:
        shape = mSimulation.mPhysicsSDK->createShape(PxBoxGeometry(collision->geometry->size),
                                                     *material);
        break;
      case Geometry::CYLINDER:
        shape = mSimulation.mPhysicsSDK->createShape(
            PxCapsuleGeometry(collision->geometry->radius, collision->geometry->length),
            *material);
        // TODO: insert warning here
        break;
      case Geometry::SPHERE:
        shape = mSimulation.mPhysicsSDK->createShape(PxSphereGeometry(collision->geometry->radius),
                                                     *material);
        break;
      case Geometry::MESH:
        PxConvexMesh *mesh = loadObjMesh(filename, mSimulation.mPhysicsSDK, mSimulation.mCooking);
        PxConvexMeshGeometry(mesh, PxMeshScale(collision->geometry->scale));
        shape = mSimulation.mPhysicsSDK->createShape(PxConvexMeshGeometry(mesh), *material);
        break;
      }
      shape->setLocalPose(tCollision2Link);
      treeNode2pLink[current]->attachShape(*shape);
    }

    for (auto c : current->children) {
      stack.push_back(c);
    }
  }
}
