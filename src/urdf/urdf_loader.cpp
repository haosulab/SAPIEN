#include "urdf/urdf_loader.h"
#include "actor_builder.h"
#include "articulation_builder.h"
#include "articulation_wrapper.h"
#include "mesh_registry.h"
#include "simulation.h"
#include <eigen3/Eigen/Eigenvalues>
#include <experimental/filesystem>
#include <map>
#include <tinyxml2.h>

#include "kinematics_articulation_interface.h"

namespace URDF {

using namespace tinyxml2;
using namespace physx;
using namespace MeshUtil;
namespace fs = std::experimental::filesystem;
// TODO: to function
static const std::map<std::string, JointType> typeString2JointType = {
    {"revolute", JointType::REVOLUTE},
    {"continuous", JointType::CONTINUOUS},
    {"fixed", JointType::FIXED},
    {"prismatic", JointType::PRISMATIC}};

PxTransform poseFromOrigin(const Origin &origin) {
  PxQuat q = PxQuat(origin.rpy.z, {0, 0, 1}) * PxQuat(origin.rpy.y, {0, 1, 0}) *
             PxQuat(origin.rpy.x, {1, 0, 0});

  return PxTransform(origin.xyz, q);
}

std::string getAbsPath(const std::string &urdfPath, const std::string &filePath) {
  if (filePath.length() == 0) {
    fprintf(stderr, "Empty file path in URDF\n");
    exit(1);
  }
  if (filePath[0] == '/') {
    return filePath;
  }
  auto path = fs::path(urdfPath);
  return fs::absolute(path).remove_filename().string() + filePath;
}

URDFLoader::URDFLoader(PxSimulation &simulation) : mSimulation(simulation) {}

struct LinkTreeNode {
  Link *link;
  Joint *joint;
  LinkTreeNode *parent;
  std::vector<LinkTreeNode *> children;
};

PxArticulationWrapper *URDFLoader::load(const std::string &filename) {
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

    const PxTransform tJoint2Parent =
        current->joint ? poseFromOrigin(*current->joint->origin) : PxTransform(PxIdentity);

    // create link and set its parent
    treeNode2pLink[current] = builder.addLink(
        current->parent ? treeNode2pLink[current->parent] : nullptr, tJoint2Parent);
    treeNode2pLink[current]->setName(current->link->name.c_str());

    PxArticulationLink &currentPxLink = *treeNode2pLink[current];

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
    Eigen::Matrix3f m;

    auto eig_vecs = es.eigenvectors();
    PxVec3 col0 = {eig_vecs(0, 0), eig_vecs(1, 0), eig_vecs(2, 0)};
    PxVec3 col1 = {eig_vecs(0, 1), eig_vecs(1, 1), eig_vecs(2, 1)};
    PxVec3 col2 = {eig_vecs(0, 2), eig_vecs(1, 2), eig_vecs(2, 2)};
    PxMat33 mat = PxMat33(col0, col1, col2);

    const PxTransform tInertia2Inertial = PxTransform(PxVec3(0), PxQuat(mat).getNormalized());

    if (!tInertia2Inertial.isSane()) {
      printf("Wrong!\n");
      exit(1);
    }

    const PxTransform tInertia2Link = tInertial2Link * tInertia2Inertial;

    if (!tInertia2Link.isSane()) {
      printf("Wrong!\n");
      exit(1);
    }

    currentPxLink.setCMassLocalPose(tInertia2Link);
    currentPxLink.setMassSpaceInertiaTensor({eigs.x(), eigs.y(), eigs.z()});
    currentPxLink.setMass(currentInertial.mass->value);

    // visual
    for (const auto &visual : current->link->visual_array) {
      const PxTransform tVisual2Link = poseFromOrigin(*visual->origin);
      switch (visual->geometry->type) {
      case Geometry::BOX:
        builder.addBoxVisualToLink(currentPxLink, tVisual2Link, visual->geometry->size);
        break;
      case Geometry::CYLINDER:
        builder.addCylinderVisualToLink(currentPxLink, tVisual2Link, visual->geometry->radius,
                                        visual->geometry->length);
        break;
      case Geometry::SPHERE:
        builder.addSphereVisualToLink(currentPxLink, tVisual2Link, visual->geometry->radius);
        break;
      case Geometry::MESH:
        builder.addObjVisualToLink(currentPxLink, getAbsPath(filename, visual->geometry->filename),
                                   tVisual2Link, visual->geometry->scale);
        break;
      }
    }

    // collision
    for (const auto &collision : current->link->collision_array) {
      const PxTransform tCollision2Link = poseFromOrigin(*collision->origin);
      // TODO: add physical material support (may require URDF extension)
      switch (collision->geometry->type) {
      case Geometry::BOX:
        builder.addBoxShapeToLink(currentPxLink, tCollision2Link, collision->geometry->size);
        break;
      case Geometry::CYLINDER:
        builder.addCylinderShapeToLink(currentPxLink, tCollision2Link, collision->geometry->radius,
                                       collision->geometry->length);
        break;
      case Geometry::SPHERE:
        builder.addSphereShapeToLink(currentPxLink, tCollision2Link, collision->geometry->radius);
        break;
      case Geometry::MESH:
        builder.addConvexObjShapeToLink(
            currentPxLink, getAbsPath(filename, collision->geometry->filename), tCollision2Link);
        break;
      }
    }

    // joint
    if (current->joint) {
      auto joint = (PxArticulationJointReducedCoordinate *)currentPxLink.getInboundJoint();

#ifdef _VERBOSE
      printf("Joint: %s between %s and %s\n", current->joint->type.c_str(),
             current->parent->link->name.c_str(), current->link->name.c_str());
#endif
      if (current->joint->type == "revolute") {
        joint->setJointType(PxArticulationJointType::eREVOLUTE);
        joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
        joint->setLimit(PxArticulationAxis::eTWIST, current->joint->limit->lower,
                        current->joint->limit->upper);
      } else if (current->joint->type == "continuous") {
        joint->setJointType(PxArticulationJointType::eREVOLUTE);
        joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
      } else if (current->joint->type == "prismatic") {
        joint->setJointType(PxArticulationJointType::ePRISMATIC);
        joint->setMotion(PxArticulationAxis::eX, PxArticulationMotion::eLIMITED);
        joint->setLimit(PxArticulationAxis::eX, current->joint->limit->lower,
                        current->joint->limit->upper);
      } else if (current->joint->type == "fixed") {
        joint->setJointType(PxArticulationJointType::eFIX);
      } else if (current->joint->type == "floating") {
        std::cerr << "Currently not supported: " + current->joint->type << std::endl;
        exit(1);
      } else if (current->joint->type == "planar") {
        std::cerr << "Currently not supported type: " + current->joint->type << std::endl;
        exit(1);
      } else {
        std::cerr << "Unreconized joint type: " + current->joint->type << std::endl;
        exit(1);
      }

      PxVec3 axis1 = current->joint->axis->xyz.getNormalized();

      PxVec3 axis2;
      if (axis1.dot({1, 0, 0}) > 0.9) {
        axis2 = axis1.cross({0, 0, 1}).getNormalized();
      } else {
        axis2 = axis1.cross({1, 0, 0}).getNormalized();
      }
      PxVec3 axis3 = axis1.cross(axis2);
      const PxTransform tAxis2Joint = {PxVec3(0), PxQuat(PxMat33(axis1, axis2, axis3))};
      if (!tAxis2Joint.isSane()) {
        printf("WRONG!\n");
        exit(1);
      }
      const PxTransform tAxis2Parent = tJoint2Parent * tAxis2Joint;
      joint->setParentPose(tAxis2Parent);
      // Joint frame === Child frame
      joint->setChildPose(tAxis2Joint);
    }

    for (auto c : current->children) {
      stack.push_back(c);
    }
  }

  PxArticulationWrapper *wrapper = builder.build(true);

  for (auto &gazebo : robot->gazebo_array) {
    for (auto &sensor : gazebo->sensor_array) {
      switch (sensor->type) {
      case Sensor::Type::RAY:
        std::cerr << "Ray Sensor not supported yet" << std::endl;
        break;
      case Sensor::Type::CAMERA:
      case Sensor::Type::DEPTH:
        PxU32 nbLinks = wrapper->articulation->getNbLinks();
        std::vector<PxArticulationLink *> links(nbLinks);
        wrapper->articulation->getLinks(links.data(), nbLinks);
        uint32_t idx =
            std::find_if(links.begin(), links.end(),
                         [&](auto const &link) { return link->getName() == gazebo->reference; }) -
            links.begin();
        if (idx == nbLinks) {
          std::cerr << "Failed to find the link to mount camera: " << gazebo->reference
                    << std::endl;
          continue;
        }

        // TODO: mount the camera
        physx_id_t cameraId = IDGenerator::instance()->next();

        const PxVec3 up = {0, 0, 1};
        const PxVec3 forward = {1, 0, 0};
        const PxMat33 rot(forward.cross(up), up, -forward);

        mSimulation.mMountedCamera2MountedActor[cameraId] = links[idx];
        mSimulation.mCameraId2InitialPose[cameraId] =
            poseFromOrigin(*sensor->origin) * PxTransform(PxVec3(0), PxQuat(rot));

        mSimulation.mRenderer->addCamera(
            cameraId, sensor->name, sensor->camera->width, sensor->camera->height,
            sensor->camera->fovx, sensor->camera->fovy, sensor->camera->near, sensor->camera->far);
      }
    }
  }
  return wrapper;
}

std::unique_ptr<PxKinematicsArticulationWrapper>
URDFLoader::loadKinematic(const std::string &filename) {
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

  std::map<LinkTreeNode *, KJoint *> treeNode2KJoint;
  std::unique_ptr<PxKinematicsArticulationWrapper> wrapper =
      std::make_unique<PxKinematicsArticulationWrapper>();
  stack = {root};
  while (!stack.empty()) {
    LinkTreeNode *current = stack.back();
    stack.pop_back();

    PxActorBuilder actorBuilder(&mSimulation);
    // visual
    for (auto &visual : current->link->visual_array) {
      const PxTransform tVisual2Link = poseFromOrigin(*visual->origin);
      switch (visual->geometry->type) {
      case Geometry::BOX:
        actorBuilder.addBoxVisual(tVisual2Link, visual->geometry->size);
        break;
      case Geometry::CYLINDER:
        actorBuilder.addCylinderVisual(tVisual2Link, visual->geometry->radius,
                                       visual->geometry->length);
        break;
      case Geometry::SPHERE:
        actorBuilder.addSphereVisual(tVisual2Link, visual->geometry->radius);
        break;
      case Geometry::MESH:
        visual->geometry->filename = getAbsPath(filename, visual->geometry->filename);
        actorBuilder.addObjVisual(visual->geometry->filename, tVisual2Link,
                                  visual->geometry->scale);
        break;
      }
    }

    // collision
    for (auto &collision : current->link->collision_array) {
      const PxTransform tCollision2Link = poseFromOrigin(*collision->origin);
      // TODO: add physical material support (may require URDF extension)
      switch (collision->geometry->type) {
      case Geometry::BOX:
        actorBuilder.addBoxShape(tCollision2Link, collision->geometry->size);
        break;
      case Geometry::CYLINDER:
        actorBuilder.addCylinderShape(tCollision2Link, collision->geometry->radius,
                                      collision->geometry->length);
        break;
      case Geometry::SPHERE:
        actorBuilder.addSphereShape(tCollision2Link, collision->geometry->radius);
        break;
      case Geometry::MESH:
        collision->geometry->filename = getAbsPath(filename, collision->geometry->filename);
        actorBuilder.addConvexShapeFromObj(collision->geometry->filename, tCollision2Link);
        break;
      }
    }

    PxRigidDynamic *currentLink = static_cast<PxRigidDynamic *>(actorBuilder.build(false, true));

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
    Eigen::Matrix3f m;

    auto eig_vecs = es.eigenvectors();
    PxVec3 col0 = {eig_vecs(0, 0), eig_vecs(1, 0), eig_vecs(2, 0)};
    PxVec3 col1 = {eig_vecs(0, 1), eig_vecs(1, 1), eig_vecs(2, 1)};
    PxVec3 col2 = {eig_vecs(0, 2), eig_vecs(1, 2), eig_vecs(2, 2)};
    PxMat33 mat = PxMat33(col0, col1, col2);

    const PxTransform tInertia2Inertial = PxTransform(PxVec3(0), PxQuat(mat).getNormalized());

    if (!tInertia2Inertial.isSane()) {
      printf("Wrong!\n");
      exit(1);
    }

    const PxTransform tInertia2Link = tInertial2Link * tInertia2Inertial;

    if (!tInertia2Link.isSane()) {
      printf("Wrong!\n");
      exit(1);
    }

    currentLink->setCMassLocalPose(tInertia2Link);
    currentLink->setMassSpaceInertiaTensor({eigs.x(), eigs.y(), eigs.z()});
    currentLink->setMass(currentInertial.mass->value);

    const PxTransform tJoint2Parent =
        current->joint ? poseFromOrigin(*current->joint->origin) : PxTransform(PxIdentity);

    // // create link and set its parent
    // treeNode2pActor[current] = builder.addLink(
    //     current->parent ? treeNode2pActor[current->parent] : nullptr, tJoint2Parent);
    // treeNode2pActor[current]->setName(current->link->name.c_str());

    // PxArticulationLink &currentPxLink = *treeNode2pActor[current];

    bool hasAxis = false;
    // joint
    if (current->joint) {
      KJoint *joint = nullptr;

      // Save urdf string, robot interface will use it
      doc.Accept(&printer);
      mUrdfString = std::string(printer.CStr());

#ifdef _VERBOSE
      printf("Joint: %s between %s and %s\n", current->joint->type.c_str(),
             current->parent->link->name.c_str(), current->link->name.c_str());
#endif

      KJoint *parentJoint = current->parent ? treeNode2KJoint[current->parent] : nullptr;
      PxVec3 axis1 = current->joint->axis->xyz.getNormalized();
      PxVec3 axis2;
      if (axis1.dot({1, 0, 0}) > 0.9) {
        axis2 = axis1.cross({0, 0, 1}).getNormalized();
      } else {
        axis2 = axis1.cross({1, 0, 0}).getNormalized();
      }
      PxVec3 axis3 = axis1.cross(axis2);
      const PxTransform tAxis2Joint = {PxVec3(0), PxQuat(PxMat33(axis1, axis2, axis3))};
      if (!tAxis2Joint.isSane()) {
        printf("WRONG!\n");
        exit(1);
      }
      const PxTransform tAxis2Parent = tJoint2Parent * tAxis2Joint;

      JointType jointType;
      if (typeString2JointType.find(current->joint->type) != typeString2JointType.end()) {
        jointType = typeString2JointType.find(current->joint->type)->second;
      } else {
        jointType = JointType::UNDEFINED;
      }
      PxReal lower = current->joint->limit ? current->joint->limit->lower : 0;
      PxReal upper = current->joint->limit ? current->joint->limit->upper : 0;
      joint = wrapper->createJoint(jointType, parentJoint, currentLink, tAxis2Parent, tAxis2Joint,
                                   upper, lower, current->joint->name);
      treeNode2KJoint[current] = joint;
    } else {
      auto rootJoint = wrapper->createJoint(JointType::FIXED, nullptr, currentLink, tJoint2Parent,
                                            PxTransform(PxIdentity), 0, 0, "root");
      treeNode2KJoint[current] = rootJoint;
    }
    for (auto c : current->children) {
      stack.push_back(c);
    }
  }
  wrapper->buildCache();
  return wrapper;
}
} // namespace URDF
