#include "urdf_loader.h"
#include "actor_builder.h"
#include "articulation_builder.h"
#include "articulation_wrapper.h"
#include "common.h"
#include "joint_system.h"
#include "kinematics_articulation_wrapper.h"
#include "mesh_registry.h"
#include "simulation.h"
#include <eigen3/Eigen/Eigenvalues>
#include <experimental/filesystem>
#include <map>
#include <tinyxml2.h>

namespace sapien {
namespace URDF {

using namespace tinyxml2;
using namespace physx;
using namespace MeshUtil;
namespace fs = std::experimental::filesystem;

static PxTransform poseFromOrigin(const Origin &origin, float scale = 1.f) {
  PxQuat q = PxQuat(origin.rpy.z, {0, 0, 1}) * PxQuat(origin.rpy.y, {0, 1, 0}) *
             PxQuat(origin.rpy.x, {1, 0, 0});

  return PxTransform(origin.xyz * scale, q);
}

static JointType typeString2JointType(const std::string &type) {
  if (type == "revolute") {
    return JointType::REVOLUTE;
  } else if (type == "continuous") {
    return JointType::CONTINUOUS;
  } else if (type == "fixed") {
    return JointType::FIXED;
  } else if (type == "prismatic") {
    return JointType::PRISMATIC;
  } else {
    std::cerr << "Unknwon joint type: " << type << std::endl;
    exit(1);
  }
}

static std::string getAbsPath(const std::string &urdfPath, const std::string &filePath) {
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

std::optional<std::string> findSRDF(const std::string &urdfName) {
  std::string srdfName = urdfName.substr(0, urdfName.length() - 4) + "srdf";
  if (fs::is_regular_file(srdfName)) {
    return srdfName;
  }
  return {};
}

URDFLoader::URDFLoader(Simulation &simulation) : mSimulation(simulation) {}

struct LinkTreeNode {
  Link *link;
  Joint *joint;
  LinkTreeNode *parent;
  std::vector<LinkTreeNode *> children;
};

std::unique_ptr<SRDF::Robot> URDFLoader::loadSRDF(const std::string &filename) {
  XMLDocument doc;
  if (doc.LoadFile(filename.c_str())) {
    std::cerr << "Error loading " << filename << std::endl;
    return nullptr;
  }
  XMLPrinter printer;
  if (strcmp("robot", doc.RootElement()->Name()) == 0) {
    return std::make_unique<SRDF::Robot>(*doc.RootElement());
  } else {
    throw std::runtime_error("SRDF root is not <robot/> in " + filename);
  }
}

ArticulationWrapper *URDFLoader::load(const std::string &filename, PxMaterial *material) {
  if (filename.substr(filename.length() - 4) != std::string("urdf")) {
    throw std::invalid_argument("None URDF file passed to URDF loader");
  }
  auto srdfName = findSRDF(filename);
  std::unique_ptr<SRDF::Robot> srdf = srdfName ? loadSRDF(srdfName.value()) : nullptr;
#ifdef _VERBOSE
  if (srdf) {
    std::cout << "SRDF found " << srdfName.value() << std::endl;
  }
#endif

  if (scale <= 0.f) {
    throw std::runtime_error("Invalid URDF scale, valid scale should >= 0");
  }

  XMLDocument doc;
  if (doc.LoadFile(filename.c_str())) {
    std::cerr << "Error loading " << filename << std::endl;
    return nullptr;
  }
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

  if (robot->link_array.size() >= 64) {
    std::cerr << "cannot build articulation with more than 64 links: " << filename << std::endl;
    return nullptr;
  }

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

  std::map<LinkTreeNode *, physx::PxArticulationLink *> treeNode2pxLink;
  ArticulationBuilder builder(&mSimulation);
  stack = {root};
  while (!stack.empty()) {
    LinkTreeNode *current = stack.back();
    stack.pop_back();

    const PxTransform tJoint2Parent =
        current->joint ? poseFromOrigin(*current->joint->origin, scale) : PxTransform(PxIdentity);

    // create link and set its parent
    treeNode2pxLink[current] = builder.addLink(
        current->parent ? treeNode2pxLink[current->parent] : nullptr, tJoint2Parent,
        current->link->name, current->joint ? current->joint->name : "");

    PxArticulationLink &currentPxLink = *treeNode2pxLink[current];

    bool shouldComputeInertia = false;
    // inertial
    const Inertial &currentInertial = *current->link->inertial;
    const Inertia &currentInertia = *currentInertial.inertia;
    if (currentInertia.ixx == 0 && currentInertia.iyy == 0 && currentInertia.izz == 0 &&
        currentInertia.ixy == 0 && currentInertia.ixz == 0 && currentInertia.iyz == 0) {
      // in this case inertia is not correctly specified
      shouldComputeInertia = true;
    } else {
      // mass is specified
      const PxTransform tInertial2Link = poseFromOrigin(*currentInertial.origin, scale);

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
        printf("Got insane inertia-inertial pose!\n");
        exit(1);
      }

      const PxTransform tInertia2Link = tInertial2Link * tInertia2Inertial;

      if (!tInertia2Link.isSane()) {
        printf("Got insane inertial-link pose!\n");
        exit(1);
      }

      float scale3 = scale * scale * scale;
      currentPxLink.setCMassLocalPose(tInertia2Link);
      currentPxLink.setMassSpaceInertiaTensor(
          {scale3 * eigs.x(), scale3 * eigs.y(), scale * eigs.z()});
      currentPxLink.setMass(currentInertial.mass->value * scale3);
    }

    // visual
    for (const auto &visual : current->link->visual_array) {
      const PxTransform tVisual2Link = poseFromOrigin(*visual->origin, scale);
      switch (visual->geometry->type) {
      case Geometry::BOX:
        builder.addBoxVisualToLink(currentPxLink, tVisual2Link, visual->geometry->size * scale,
                                   {1, 1, 1}, visual->name);
        break;
      case Geometry::CYLINDER:
        builder.addCapsuleVisualToLink(
            currentPxLink, tVisual2Link * PxTransform({{0, 0, 0}, PxQuat(1.57, {0, 1, 0})}),
            visual->geometry->radius * scale, visual->geometry->length * scale / 2.f, {1, 1, 1},
            visual->name);
        break;
      case Geometry::SPHERE:
        builder.addSphereVisualToLink(currentPxLink, tVisual2Link,
                                      visual->geometry->radius * scale, {1, 1, 1}, visual->name);
        break;
      case Geometry::MESH:
        builder.addObjVisualToLink(currentPxLink, getAbsPath(filename, visual->geometry->filename),
                                   tVisual2Link, visual->geometry->scale * scale, visual->name);
        break;
      }
    }

    // collision
    for (const auto &collision : current->link->collision_array) {
      const PxTransform tCollision2Link = poseFromOrigin(*collision->origin, scale);
      // TODO: add physical material support (may require URDF extension)
      switch (collision->geometry->type) {
      case Geometry::BOX:
        builder.addBoxShapeToLink(currentPxLink, tCollision2Link,
                                  collision->geometry->size * scale);
        break;
      case Geometry::CYLINDER:
        builder.addCapsuleShapeToLink(
            currentPxLink, tCollision2Link * PxTransform({{0, 0, 0}, PxQuat(1.57, {0, 1, 0})}),
            collision->geometry->radius * scale, collision->geometry->length * scale / 2.0f);
        break;
      case Geometry::SPHERE:
        builder.addSphereShapeToLink(currentPxLink, tCollision2Link,
                                     collision->geometry->radius * scale);
        break;
      case Geometry::MESH:
        builder.addConvexObjShapeToLink(currentPxLink,
                                        getAbsPath(filename, collision->geometry->filename),
                                        tCollision2Link, PxVec3(1, 1, 1) * scale, material);
        break;
      }
    }
    if (shouldComputeInertia) {
      // TODO: check density
      PxRigidBodyExt::updateMassAndInertia(currentPxLink, defaultDensity);
    }

    // joint
    if (current->joint) {
      auto joint = (PxArticulationJointReducedCoordinate *)currentPxLink.getInboundJoint();
      // Load joint dynamics
      if (current->joint->dynamics) {
        joint->setFrictionCoefficient(current->joint->dynamics->friction);
      }

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
        joint->setLimit(PxArticulationAxis::eX, current->joint->limit->lower * scale,
                        current->joint->limit->upper * scale);
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
      if (std::abs(axis1.dot({1, 0, 0})) > 0.9) {
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
  // collision
  if (srdf) {
    for (auto &dc : srdf->disable_collisions_array) {
      if (dc->reason == std::string("default")) {
        if (linkName2treeNode.find(dc->link1) == linkName2treeNode.end()) {
          throw std::runtime_error("SRDF link name not found: " + dc->link1);
        }
        if (linkName2treeNode.find(dc->link2) == linkName2treeNode.end()) {
          throw std::runtime_error("SRDF link name not found: " + dc->link2);
        }
        auto l1 = linkName2treeNode[dc->link1];
        auto l2 = linkName2treeNode[dc->link2];
        auto link1 = treeNode2pxLink[l1];
        auto link2 = treeNode2pxLink[l2];
        builder.disableCollision(*link1, *link2);
      }
    }
  }

  ArticulationWrapper *wrapper = builder.build(fixLoadedObject, balancePassiveForce);

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
        uint32_t idx = std::find_if(links.begin(), links.end(),
                                    [&](auto const &link) {
                                      return std::string(link->getName()) == gazebo->reference;
                                    }) -
                       links.begin();
        if (idx == nbLinks) {
          std::cerr << "Failed to find the link to mount camera: " << gazebo->reference
                    << std::endl;
          continue;
        }

        mSimulation.addMountedCamera(sensor->name, links[idx],
                                     poseFromOrigin(*sensor->origin, scale), sensor->camera->width,
                                     sensor->camera->height, sensor->camera->fovx,
                                     sensor->camera->fovy);
      }
    }
  }
  return wrapper;
}

KinematicsArticulationWrapper *URDFLoader::loadKinematic(const std::string &filename) {
  if (scale != 1.f) {
    std::cerr
        << "Scaling is not yet supported for Kinematic Articulation Loader. Using scale=1 instead."
        << std::endl;
  }

  std::unique_ptr<KinematicsArticulationWrapper> wrapper =
      std::make_unique<KinematicsArticulationWrapper>(mSimulation);

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
  stack = {root};
  while (!stack.empty()) {
    LinkTreeNode *current = stack.back();
    stack.pop_back();

    ActorBuilder actorBuilder(&mSimulation);
    // visual
    for (auto &visual : current->link->visual_array) {
      const PxTransform tVisual2Link = poseFromOrigin(*visual->origin);
      physx_id_t linkId;
      switch (visual->geometry->type) {
      case Geometry::BOX:
        linkId = actorBuilder.addBoxVisual(tVisual2Link, visual->geometry->size);
        break;
        // TODO: Cylinder parameter is wrong now
      case Geometry::CYLINDER:
        linkId = actorBuilder.addCapsuleVisual(
            tVisual2Link * PxTransform({{0, 0, 0}, PxQuat(1.57, {0, 1, 0})}),
            visual->geometry->radius, visual->geometry->length / 2.f);
        break;
      case Geometry::SPHERE:
        linkId = actorBuilder.addSphereVisual(tVisual2Link, visual->geometry->radius);
        break;
      case Geometry::MESH:
        visual->geometry->filename = getAbsPath(filename, visual->geometry->filename);
        linkId = actorBuilder.addObjVisual(visual->geometry->filename, tVisual2Link,
                                           visual->geometry->scale);
        break;
      }
      mSimulation.mLinkId2Articulation[linkId] = wrapper.get();
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
        actorBuilder.addCapsuleShape(
            tCollision2Link * PxTransform({{0, 0, 0}, PxQuat(1.57, {0, 1, 0})}),
            collision->geometry->radius, collision->geometry->length / 2.f);
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

    PxRigidDynamic *currentLink =
        static_cast<PxRigidDynamic *>(actorBuilder.build(false, true, current->link->name));

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

      auto jointType = typeString2JointType(current->joint->type);
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
  for (auto &gazebo : robot->gazebo_array) {
    for (auto &sensor : gazebo->sensor_array) {
      switch (sensor->type) {
      case Sensor::Type::RAY:
        std::cerr << "Ray Sensor not supported yet" << std::endl;
        break;
      case Sensor::Type::CAMERA:
      case Sensor::Type::DEPTH:
        std::vector<PxRigidBody *> links = wrapper->get_links();
        uint32_t nbLinks = links.size();

        uint32_t idx = std::find_if(links.begin(), links.end(),
                                    [&](auto const &link) {
                                      return std::string(link->getName()) == gazebo->reference;
                                    }) -
                       links.begin();
        if (idx == nbLinks) {
          std::cerr << "Failed to find the link to mount camera: " << gazebo->reference
                    << std::endl;
          continue;
        }

        mSimulation.addMountedCamera(sensor->name, links[idx], poseFromOrigin(*sensor->origin),
                                     sensor->camera->width, sensor->camera->height,
                                     sensor->camera->fovx, sensor->camera->fovy);
      }
    }
  }
  auto wrapperPtr = wrapper.get();
  mSimulation.mKinematicArticulationWrappers.push_back(std::move(wrapper));
  return wrapperPtr;
}

JointSystem *URDFLoader::loadJointSystem(const std::string &filename) {
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

  // std::map<LinkTreeNode *, physx::PxRigidActor *> treeNode2pLink;
  // TODO: fix

  std::vector<std::unique_ptr<ActorBuilder>> builders;
  std::map<LinkTreeNode *, ActorBuilder *> treeNode2Builder;
  std::map<LinkTreeNode *, PxTransform> treeNode2Pose;
  std::map<LinkTreeNode *, PxTransform> treeNode2ActorPose;
  std::vector<physx_id_t> linkIds;
  auto newObject = std::make_unique<JointSystem>(&mSimulation);

  stack = {root};
  while (!stack.empty()) {
    LinkTreeNode *current = stack.back();
    stack.pop_back();

    const PxTransform tJoint2Parent =
        current->joint ? poseFromOrigin(*current->joint->origin) : PxTransform(PxIdentity);
    treeNode2Pose[current] =
        current->parent ? treeNode2Pose[current->parent] * tJoint2Parent : tJoint2Parent;

    if (current->joint && current->joint->type == "fixed") {
      treeNode2Builder[current] = treeNode2Builder[current->parent];
      treeNode2ActorPose[current] = treeNode2ActorPose[current->parent] * tJoint2Parent;
    } else {
      builders.push_back(mSimulation.createActorBuilder());
      treeNode2Builder[current] = builders.back().get();
      treeNode2ActorPose[current] = PxTransform(PxIdentity);
    }

    auto actorBuilder = treeNode2Builder[current];
    PxTransform interPose = treeNode2ActorPose[current];
    // visual
    for (const auto &visual : current->link->visual_array) {
      const PxTransform tVisual2Link = poseFromOrigin(*visual->origin);
      physx_id_t linkId = 0;
      switch (visual->geometry->type) {
      case Geometry::BOX:
        linkId = actorBuilder->addBoxVisual(interPose * tVisual2Link, visual->geometry->size);
        break;
        // TODO: Cylinder
      case Geometry::CYLINDER:
        linkId = actorBuilder->addCapsuleVisual(interPose * tVisual2Link, visual->geometry->radius,
                                                visual->geometry->length);
        break;
      case Geometry::SPHERE:
        linkId = actorBuilder->addSphereVisual(interPose * tVisual2Link, visual->geometry->radius);
        break;
      case Geometry::MESH:
        linkId = actorBuilder->addObjVisual(getAbsPath(filename, visual->geometry->filename),
                                            interPose * tVisual2Link, visual->geometry->scale);
        break;
      }
      linkIds.push_back(linkId);
    }

    // collision
    for (const auto &collision : current->link->collision_array) {
      const PxTransform tCollision2Link = poseFromOrigin(*collision->origin);
      // TODO: add physical material support (may require URDF extension)
      switch (collision->geometry->type) {
      case Geometry::BOX:
        actorBuilder->addBoxShape(interPose * tCollision2Link, collision->geometry->size);
        break;
      case Geometry::CYLINDER:
        actorBuilder->addCapsuleShape(interPose * tCollision2Link, collision->geometry->radius,
                                      collision->geometry->length);
        break;
      case Geometry::SPHERE:
        actorBuilder->addSphereShape(interPose * tCollision2Link, collision->geometry->radius);
        break;
      case Geometry::MESH:
        actorBuilder->addConvexShapeFromObj(getAbsPath(filename, collision->geometry->filename),
                                            interPose * tCollision2Link);
        break;
      }
    }

    // TODO: inertia loading
    std::cerr << "Inertia loading currently unsupported for Object Joint Loader" << std::endl;

    for (auto c : current->children) {
      stack.push_back(c);
    }
  }
  std::map<LinkTreeNode *, physx::PxRigidActor *> treeNode2pLink;
  stack = {root};
  while (!stack.empty()) {
    auto current = stack.back();
    stack.pop_back();
    for (auto c : current->children) {
      stack.push_back(c);
    }
    if (current->joint && current->joint->type == "fixed") {
      treeNode2pLink[current] = treeNode2pLink[current->parent];
      continue;
    }
    PxRigidActor *currentLink =
        current->parent ? treeNode2Builder[current]->build(false, false, "", false)
                        : treeNode2Builder[current]->build(false, fixLoadedObject, "", false);
    treeNode2pLink[current] = currentLink;
    currentLink->setGlobalPose(treeNode2Pose[current]);
    newObject->addLink(currentLink);
    if (current->joint && current->joint->type != "fixed") {
      PxJoint *newJoint = nullptr;
#ifdef _VERBOSE
      printf("Joint: %s between %s and %s\n", current->joint->type.c_str(),
             current->parent->link->name.c_str(), current->link->name.c_str());
#endif

      PxRigidActor *parentLink = current->parent ? treeNode2pLink[current->parent] : nullptr;
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
      const PxTransform tJoint2Parent =
          current->joint ? poseFromOrigin(*current->joint->origin) : PxTransform(PxIdentity);
      const PxTransform tAxis2Parent =
          treeNode2ActorPose[current->parent] * tJoint2Parent * tAxis2Joint;

      if (current->joint->type == "revolute") {
        auto newRJoint = PxRevoluteJointCreate(*mSimulation.mPhysicsSDK, parentLink, tAxis2Parent,
                                               currentLink, tAxis2Joint);
        newRJoint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
        newRJoint->setLimit(
            PxJointAngularLimitPair(current->joint->limit->lower, current->joint->limit->upper));
        newJoint = newRJoint;
        newJoint->setName(newNameFromString(""));
      } else if (current->joint->type == "continuous") {
        auto newCJoint = PxRevoluteJointCreate(*mSimulation.mPhysicsSDK, parentLink, tAxis2Parent,
                                               currentLink, tAxis2Joint);
        newJoint = newCJoint;
        newJoint->setName(newNameFromString(""));
      } else if (current->joint->type == "prismatic") {
        auto newPJoint = PxPrismaticJointCreate(*mSimulation.mPhysicsSDK, parentLink, tAxis2Parent,
                                                currentLink, tAxis2Joint);
        newPJoint->setPrismaticJointFlag(PxPrismaticJointFlag::eLIMIT_ENABLED, true);
        newPJoint->setLimit(PxJointLinearLimitPair(mSimulation.mPhysicsSDK->getTolerancesScale(),
                                                   current->joint->limit->lower,
                                                   current->joint->limit->upper));
        newJoint = newPJoint;
        newJoint->setName(newNameFromString(""));
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
      newObject->addJoint(newJoint, current->joint->name);
    }
  }

  for (auto id : linkIds) {
    mSimulation.mLinkId2Articulation[id] = newObject.get();
  }
  mSimulation.mJointSystemWrappers.push_back(std::move(newObject));
  return mSimulation.mJointSystemWrappers.back().get();
}

} // namespace URDF

} // namespace sapien
