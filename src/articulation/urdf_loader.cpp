#include "urdf_loader.h"
#include "articulation_builder.h"
#include "sapien_articulation.h"
#include "sapien_link.h"
#include "sapien_scene.h"
#include <eigen3/Eigen/Eigenvalues>
#include <experimental/filesystem>

namespace sapien {
namespace URDF {
using namespace tinyxml2;
using namespace physx;
namespace fs = std::experimental::filesystem;

static PxTransform poseFromOrigin(const Origin &origin, float scale = 1.f) {
  PxQuat q = PxQuat(origin.rpy.z, {0, 0, 1}) * PxQuat(origin.rpy.y, {0, 1, 0}) *
             PxQuat(origin.rpy.x, {1, 0, 0});

  return PxTransform(origin.xyz * scale, q);
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
  return fs::canonical(path).remove_filename().string() + filePath;
}

std::optional<std::string> findSRDF(const std::string &urdfName) {
  std::string srdfName = urdfName.substr(0, urdfName.length() - 4) + "srdf";
  if (fs::is_regular_file(srdfName)) {
    return srdfName;
  }
  return {};
}

URDFLoader::URDFLoader(SScene *scene) : mScene(scene) {}

struct LinkTreeNode {
  Link *link;
  Joint *joint;
  LinkTreeNode *parent;
  std::vector<LinkTreeNode *> children;
  LinkBuilder *linkBuilder = nullptr;
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

SArticulation *URDFLoader::load(const std::string &filename, PxMaterial *material) {
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
  std::unique_ptr<Robot> robot;
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

  // std::map<LinkTreeNode *, physx::PxArticulationLink *> treeNode2pxLink;

  auto builder = mScene->createArticulationBuilder();
  stack = {root};
  while (!stack.empty()) {
    LinkTreeNode *current = stack.back();
    stack.pop_back();

    const PxTransform tJoint2Parent =
        current->joint ? poseFromOrigin(*current->joint->origin, scale) : PxTransform(PxIdentity);

    current->linkBuilder =
        builder->createLinkBuilder(current->parent ? current->parent->linkBuilder : nullptr);
    current->linkBuilder->setName(current->link->name);
    current->linkBuilder->setJointName(current->joint ? current->joint->name : "");

    // create link and set its parent
    // treeNode2pxLink[current] = builder.addLink(
    //     current->parent ? treeNode2pxLink[current->parent] : nullptr, tJoint2Parent,
    //     current->link->name, current->joint ? current->joint->name : "");

    // PxArticulationLink &currentPxLink = *treeNode2pxLink[current];
    auto currentLinkBuilder = current->linkBuilder;

    // inertial
    const Inertial &currentInertial = *current->link->inertial;
    const Inertia &currentInertia = *currentInertial.inertia;
    if (currentInertia.ixx == 0 && currentInertia.iyy == 0 && currentInertia.izz == 0 &&
        currentInertia.ixy == 0 && currentInertia.ixz == 0 && currentInertia.iyz == 0) {
      // in this case inertia is not correctly specified
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
      currentLinkBuilder->setMassAndInertia(
          currentInertial.mass->value * scale3, tInertia2Link,
          {scale3 * eigs.x(), scale3 * eigs.y(), scale * eigs.z()});
      // currentPxLink.setCMassLocalPose(tInertia2Link);
      // currentPxLink.setMassSpaceInertiaTensor(
      //     {scale3 * eigs.x(), scale3 * eigs.y(), scale * eigs.z()});
      // currentPxLink.setMass(currentInertial.mass->value * scale3);
    }

    // visual
    for (const auto &visual : current->link->visual_array) {
      const PxTransform tVisual2Link = poseFromOrigin(*visual->origin, scale);
      switch (visual->geometry->type) {
      case Geometry::BOX:
        currentLinkBuilder->addBoxVisual(tVisual2Link, visual->geometry->size * scale, {1, 1, 1},
                                         visual->name);
        // builder.addBoxVisualToLink(currentPxLink, tVisual2Link, visual->geometry->size * scale,
        //                            {1, 1, 1}, visual->name);
        break;
      case Geometry::CYLINDER:
        currentLinkBuilder->addCapsuleVisual(
            tVisual2Link * PxTransform({{0, 0, 0}, PxQuat(1.57, {0, 1, 0})}),
            visual->geometry->radius * scale, visual->geometry->length * scale / 2.f, {1, 1, 1},
            visual->name);
        // builder.addCapsuleVisualToLink(
        //     currentPxLink, tVisual2Link * PxTransform({{0, 0, 0}, PxQuat(1.57, {0, 1, 0})}),
        //     visual->geometry->radius * scale, visual->geometry->length * scale / 2.f, {1, 1, 1},
        //     visual->name);
        break;
      case Geometry::SPHERE:
        currentLinkBuilder->addSphereVisual(tVisual2Link, visual->geometry->radius * scale,
                                            {1, 1, 1}, visual->name);
        // builder.addSphereVisualToLink(currentPxLink, tVisual2Link,
        //                               visual->geometry->radius * scale, {1, 1, 1},
        //                               visual->name);
        break;
      case Geometry::MESH:
        currentLinkBuilder->addObjVisual(getAbsPath(filename, visual->geometry->filename),
                                         tVisual2Link, visual->geometry->scale * scale,
                                         visual->name);
        // builder.addObjVisualToLink(currentPxLink, getAbsPath(filename,
        // visual->geometry->filename),
        //                            tVisual2Link, visual->geometry->scale * scale, visual->name);
        break;
      }
    }

    // collision
    for (const auto &collision : current->link->collision_array) {
      const PxTransform tCollision2Link = poseFromOrigin(*collision->origin, scale);
      // TODO: add physical material support (may require URDF extension)
      switch (collision->geometry->type) {
      case Geometry::BOX:
        // builder.addBoxShapeToLink(currentPxLink, tCollision2Link,
        //                           collision->geometry->size * scale);
        currentLinkBuilder->addBoxShape(tCollision2Link, collision->geometry->size * scale,
                                        material, defaultDensity);
        break;
      case Geometry::CYLINDER:
        // builder.addCapsuleShapeToLink(
        //     currentPxLink, tCollision2Link * PxTransform({{0, 0, 0}, PxQuat(1.57, {0, 1, 0})}),
        //     collision->geometry->radius * scale, collision->geometry->length * scale / 2.0f);
        currentLinkBuilder->addCapsuleShape(
            tCollision2Link * PxTransform({{0, 0, 0}, PxQuat(1.57, {0, 1, 0})}),
            collision->geometry->radius * scale, collision->geometry->length * scale / 2.0f,
            material, defaultDensity);
        break;
      case Geometry::SPHERE:
        // builder.addSphereShapeToLink(currentPxLink, tCollision2Link,
        //                              collision->geometry->radius * scale);
        currentLinkBuilder->addSphereShape(tCollision2Link, collision->geometry->radius * scale,
                                           material, defaultDensity);
        break;
      case Geometry::MESH:
        // builder.addConvexObjShapeToLink(currentPxLink,
        //                                 getAbsPath(filename, collision->geometry->filename),
        //                                 tCollision2Link, PxVec3(1, 1, 1) * scale, material);
        currentLinkBuilder->addConvexShapeFromObj(
            getAbsPath(filename, collision->geometry->filename), tCollision2Link,
            PxVec3(1, 1, 1) * scale, material, defaultDensity);
        break;
      }
    }

    // joint
    if (current->joint) {
      // auto joint = (PxArticulationJointReducedCoordinate *)currentPxLink.getInboundJoint();

      PxReal friction = 0;
      PxReal damping = 0;
      if (current->joint->dynamics) {
        friction = current->joint->dynamics->friction;
        damping = current->joint->dynamics->damping;
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

      if (current->joint->type == "revolute") {
        currentLinkBuilder->setJointProperties(
            PxArticulationJointType::eREVOLUTE,
            {{current->joint->limit->lower, current->joint->limit->upper}}, tAxis2Parent,
            tAxis2Joint, friction, damping);
      } else if (current->joint->type == "continuous") {
        currentLinkBuilder->setJointProperties(
            PxArticulationJointType::eREVOLUTE,
            {{-std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()}},
            tAxis2Parent, tAxis2Joint, friction, damping);
      } else if (current->joint->type == "prismatic") {
        currentLinkBuilder->setJointProperties(
            PxArticulationJointType::ePRISMATIC,
            {{current->joint->limit->lower, current->joint->limit->upper}}, tAxis2Parent,
            tAxis2Joint, friction, damping);
      } else if (current->joint->type == "fixed") {
        currentLinkBuilder->setJointProperties(PxArticulationJointType::eFIX, {}, tAxis2Parent,
                                               tAxis2Joint, friction, damping);
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
    }

    for (auto c : current->children) {
      stack.push_back(c);
    }
  }

  // TODO collision
  // if (srdf) {
  //   for (auto &dc : srdf->disable_collisions_array) {
  //     if (dc->reason == std::string("default")) {
  //       if (linkName2treeNode.find(dc->link1) == linkName2treeNode.end()) {
  //         throw std::runtime_error("SRDF link name not found: " + dc->link1);
  //       }
  //       if (linkName2treeNode.find(dc->link2) == linkName2treeNode.end()) {
  //         throw std::runtime_error("SRDF link name not found: " + dc->link2);
  //       }
  //       auto l1 = linkName2treeNode[dc->link1];
  //       auto l2 = linkName2treeNode[dc->link2];
  //       auto link1 = treeNode2pxLink[l1];
  //       auto link2 = treeNode2pxLink[l2];
  //       builder.disableCollision(*link1, *link2);
  //     }
  //   }
  // }

  SArticulation *articulation = builder->build(fixLoadedObject);

  for (auto &gazebo : robot->gazebo_array) {
    for (auto &sensor : gazebo->sensor_array) {
      switch (sensor->type) {
      case Sensor::Type::RAY:
        std::cerr << "Ray Sensor not supported yet" << std::endl;
        break;
      case Sensor::Type::CAMERA:
      case Sensor::Type::DEPTH:

        std::vector<SLinkBase *> links = articulation->getLinks();

        auto it = std::find_if(links.begin(), links.end(), [&](SLinkBase *link) {
          return link->getName() == gazebo->reference;
        });

        if (it == links.end()) {
          spdlog::error("Failed to find the link to mount camera: ", gazebo->reference);
          continue;
        }

        mScene->addMountedCamera(sensor->name, *it, poseFromOrigin(*sensor->origin, scale),
                                 sensor->camera->width, sensor->camera->height,
                                 sensor->camera->fovx, sensor->camera->fovy);
      }
    }
  }
  return articulation;
}

} // namespace URDF
} // namespace sapien
