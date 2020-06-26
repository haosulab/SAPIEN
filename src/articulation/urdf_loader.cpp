#include "urdf_loader.h"
#include "articulation_builder.h"
#include "sapien_articulation.h"
#include "sapien_kinematic_articulation.h"
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
  if (urdfPath.empty()) {
    return filePath;
  }
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

std::tuple<std::unique_ptr<ArticulationBuilder>, std::vector<SensorRecord>>
URDFLoader::parseRobotDescription(XMLDocument const &urdfDoc, XMLDocument const *srdfDoc,
                                  const std::string &urdfFilename, bool isKinematic,
                                  URDFConfig const &config) {
  std::optional<SRDF::Robot> srdf;
  if (srdfDoc) {
    srdf = SRDF::Robot(*srdfDoc->RootElement());
  }

  std::unique_ptr<Robot> robot;
  if (strcmp("robot", urdfDoc.RootElement()->Name()) == 0) {
    robot = std::make_unique<Robot>(*urdfDoc.RootElement());
  } else {
    std::cerr << "Invalid URDF: root is not <robot/>" << std::endl;
    exit(1);
  }

  std::vector<std::unique_ptr<LinkTreeNode>> treeNodes;
  std::map<std::string, LinkTreeNode *> linkName2treeNode;
  std::map<std::string, LinkTreeNode *> jointName2treeNode;

  if (robot->link_array.size() >= 64 && !isKinematic) {
    spdlog::get("SAPIEN")->error("cannot build dynamic articulation with more than 64 links");
    return {nullptr, std::vector<SensorRecord>()};
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
    if (linkName2treeNode.find(parent) == linkName2treeNode.end()) {
      spdlog::get("SAPIEN")->error(
          "Failed to load URDF: parent of joint {} does not exist", name);
      return {nullptr, std::vector<SensorRecord>()};
    }
    if (linkName2treeNode.find(child) == linkName2treeNode.end()) {
      spdlog::get("SAPIEN")->error(
          "Failed to load URDF: child of joint {} does not exist", name);
      return {nullptr, std::vector<SensorRecord>()};
    }

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

  std::vector<LinkTreeNode *> roots;
  // find root
  LinkTreeNode *root = nullptr;
  for (const auto &node : treeNodes) {
    if (!node->parent) {
      roots.push_back(node.get());
    }
  }
  if (roots.size() > 1) {
    spdlog::get("SAPIEN")->error(
        "Failed to load URDF: multiple root nodes detected in a single URDF");
    return {nullptr, std::vector<SensorRecord>()};
  }
  if (roots.size() == 0) {
    spdlog::get("SAPIEN")->error("Failed to load URDF: no root node found");
    return {nullptr, std::vector<SensorRecord>()};
  }
  root = roots[0];

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

      PxVec3 eigs;
      PxTransform tInertia2Inertial;
      if (currentInertia.ixy == 0 && currentInertia.ixz == 0 && currentInertia.iyz == 0) {
        tInertia2Inertial = PxTransform(PxVec3(0), PxIdentity);
        eigs = {currentInertia.ixx, currentInertia.iyy, currentInertia.izz};
      } else {
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
        eigs = {es.eigenvalues().x(), es.eigenvalues().y(), es.eigenvalues().z()};
        Eigen::Matrix3f m;

        auto eig_vecs = es.eigenvectors();
        PxVec3 col0 = {eig_vecs(0, 0), eig_vecs(1, 0), eig_vecs(2, 0)};
        PxVec3 col1 = {eig_vecs(0, 1), eig_vecs(1, 1), eig_vecs(2, 1)};
        PxVec3 col2 = {eig_vecs(0, 2), eig_vecs(1, 2), eig_vecs(2, 2)};
        PxMat33 mat = PxMat33(col0, col1, col2);

        tInertia2Inertial = PxTransform(PxVec3(0), PxQuat(mat).getNormalized());
      }

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
      currentLinkBuilder->setMassAndInertia(currentInertial.mass->value * scale3, tInertia2Link,
                                            {scale3 * eigs.x, scale3 * eigs.y, scale * eigs.z});
    }

    // visual
    for (const auto &visual : current->link->visual_array) {
      const PxTransform tVisual2Link = poseFromOrigin(*visual->origin, scale);
      switch (visual->geometry->type) {
      case Geometry::BOX:
        currentLinkBuilder->addBoxVisual(tVisual2Link, visual->geometry->size * scale,
                                         PxVec3{1, 1, 1}, visual->name);
        break;
      case Geometry::CYLINDER:
      case Geometry::CAPSULE:
        currentLinkBuilder->addCapsuleVisual(
            tVisual2Link * PxTransform({{0, 0, 0}, PxQuat(1.57079633, {0, 1, 0})}),
            visual->geometry->radius * scale, visual->geometry->length * scale / 2.f,
            PxVec3{1, 1, 1}, visual->name);
        break;
      case Geometry::SPHERE:
        currentLinkBuilder->addSphereVisual(tVisual2Link, visual->geometry->radius * scale,
                                            PxVec3{1, 1, 1}, visual->name);
        break;
      case Geometry::MESH:
        currentLinkBuilder->addVisualFromFile(getAbsPath(urdfFilename, visual->geometry->filename),
                                              tVisual2Link, visual->geometry->scale * scale,
                                              visual->name);
        break;
      }
    }

    // collision
    for (uint32_t collision_idx = 0; collision_idx < current->link->collision_array.size();
         ++collision_idx) {
      const auto &collision = current->link->collision_array[collision_idx];

      physx::PxMaterial *material;
      float patchRadius;
      float minPatchRadius;
      float density;
      auto it = config.link.find(current->link->name);
      if (it != config.link.end()) {
        auto it2 = it->second.shape.find(collision_idx);
        if (it2 != it->second.shape.end()) {
          material = it2->second.material;
          patchRadius = it2->second.patchRadius;
          minPatchRadius = it2->second.minPatchRadius;
          density = it2->second.density;
        } else {
          material = it->second.material;
          patchRadius = it->second.patchRadius;
          minPatchRadius = it->second.minPatchRadius;
          density = it->second.density;
        }
      } else {
        patchRadius = 0.f;
        minPatchRadius = 0.f;
        material = config.material;
        density = config.density;
      }

      const PxTransform tCollision2Link = poseFromOrigin(*collision->origin, scale);
      // TODO: add physical material support (may require URDF extension)
      switch (collision->geometry->type) {
      case Geometry::BOX:
        currentLinkBuilder->addBoxShape(tCollision2Link, collision->geometry->size * scale / 2.f,
                                        material, density, patchRadius, minPatchRadius);
        if (collisionIsVisual) {
          currentLinkBuilder->addBoxVisual(
              tCollision2Link, collision->geometry->size * scale / 2.f, PxVec3{1, 1, 1}, "");
        }
        break;
      case Geometry::CYLINDER:
      case Geometry::CAPSULE:
        currentLinkBuilder->addCapsuleShape(
            tCollision2Link * PxTransform({{0, 0, 0}, PxQuat(1.57079633, {0, 1, 0})}),
            collision->geometry->radius * scale, collision->geometry->length * scale / 2.0f,
            material, density, patchRadius, minPatchRadius);
        if (collisionIsVisual) {
          currentLinkBuilder->addCapsuleVisual(
              tCollision2Link * PxTransform({{0, 0, 0}, PxQuat(1.57079633, {0, 1, 0})}),
              collision->geometry->radius * scale, collision->geometry->length * scale / 2.f,
              PxVec3{1, 1, 1}, "");
        }
        break;
      case Geometry::SPHERE:
        currentLinkBuilder->addSphereShape(tCollision2Link, collision->geometry->radius * scale,
                                           material, density, patchRadius, minPatchRadius);
        if (collisionIsVisual) {
          currentLinkBuilder->addSphereVisual(tCollision2Link, collision->geometry->radius * scale,
                                              PxVec3{1, 1, 1}, "");
        }
        break;
      case Geometry::MESH:
        currentLinkBuilder->addConvexShapeFromFile(
            getAbsPath(urdfFilename, collision->geometry->filename), tCollision2Link,
            collision->geometry->scale * scale, material, density, patchRadius, minPatchRadius);
        if (collisionIsVisual) {
          currentLinkBuilder->addVisualFromFile(
              getAbsPath(urdfFilename, collision->geometry->filename), tCollision2Link,
              collision->geometry->scale * scale, "");
        }
        break;
      }
    }

    // joint
    if (current->joint) {
      PxReal friction = 0;
      PxReal damping = 0;
      if (current->joint->dynamics) {
        friction = current->joint->dynamics->friction;
        damping = current->joint->dynamics->damping;
      }

      if (current->joint->axis->xyz.magnitudeSquared() < 0.01) {
        current->joint->axis->xyz = {1, 0, 0};
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
        spdlog::get("SAPIEN")->critical("URDF loading failed: invalid joint pose");
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
            {{current->joint->limit->lower * scale, current->joint->limit->upper * scale}},
            tAxis2Parent, tAxis2Joint, friction, damping);
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

  if (srdf) {
    uint32_t groupCount = 0;
    for (auto &dc : srdf->disable_collisions_array) {
      if (dc->reason == std::string("default")) {
        if (linkName2treeNode.find(dc->link1) == linkName2treeNode.end()) {
          spdlog::get("SAPIEN")->error("SRDF link name not found: {}", dc->link1);
          continue;
        }
        if (linkName2treeNode.find(dc->link2) == linkName2treeNode.end()) {
          spdlog::get("SAPIEN")->error("SRDF link name not found: {}", dc->link2);
          continue;
        }
        LinkTreeNode *l1 = linkName2treeNode[dc->link1];
        LinkTreeNode *l2 = linkName2treeNode[dc->link2];
        if (l1->parent == l2 || l2->parent == l1) {
          continue; // ignored by default
        }
        groupCount += 1;
        if (groupCount == 32) {
          spdlog::get("SAPIEN")->critical("Collision group exhausted, please simplify the SRDF");
          throw std::runtime_error("Too many collision groups");
        }
        l1->linkBuilder->addCollisionGroup(0, 0, groupCount);
        l2->linkBuilder->addCollisionGroup(0, 0, groupCount);
      }
    }
    spdlog::get("SAPIEN")->info("SRDF: ignored {} pairs", groupCount);
  }

  // SArticulationBase *articulation;
  // if (isKinematic) {
  //   articulation = builder->buildKinematic();
  // } else {
  //   articulation = builder->build(fixRootLink);
  // }

  std::vector<SensorRecord> sensorRecords;

  for (auto &gazebo : robot->gazebo_array) {
    for (auto &sensor : gazebo->sensor_array) {
      switch (sensor->type) {
      case Sensor::Type::RAY:
      case Sensor::Type::UNKNOWN:
        break;
      case Sensor::Type::CAMERA:
      case Sensor::Type::DEPTH:

        // std::vector<SLinkBase *> links = articulation->getBaseLinks();

        // auto it = std::find_if(links.begin(), links.end(), [&](SLinkBase *link) {
        //   return link->getName() == gazebo->reference;
        // });

        // if (it == links.end()) {
        //   spdlog::get("SAPIEN")->error("Failed to find the link to mount camera: ",
        //                                gazebo->reference);
        //   continue;
        // }

        sensorRecords.push_back(SensorRecord{"camera", sensor->name, gazebo->reference,
                                             poseFromOrigin(*sensor->origin, scale),
                                             sensor->camera->width, sensor->camera->height,
                                             sensor->camera->fovx, sensor->camera->fovy});
        // mScene->addMountedCamera(sensor->name, *it, poseFromOrigin(*sensor->origin, scale),
        //                          sensor->camera->width, sensor->camera->height,
        //                          sensor->camera->fovx, sensor->camera->fovy);
      }
    }
  }
  return {std::move(builder), sensorRecords};
}

SArticulation *URDFLoader::load(const std::string &filename, URDFConfig const &config) {
  if (filename.substr(filename.length() - 4) != std::string("urdf")) {
    throw std::invalid_argument("Non-URDF file passed to URDF loader");
  }
  auto srdfName = findSRDF(filename);

  std::unique_ptr<XMLDocument> srdfDoc = nullptr;
  if (srdfName) {
    srdfDoc = std::make_unique<XMLDocument>();
    if (srdfDoc->LoadFile(srdfName.value().c_str())) {
      srdfDoc = nullptr;
      spdlog::get("SAPIEN")->error("SRDF loading faild for {}", filename);
    }
  }

  XMLDocument urdfDoc;
  if (urdfDoc.LoadFile(filename.c_str())) {
    spdlog::get("SAPIEN")->error("Failed to open URDF file: {}", filename);
    return nullptr;
  }

  auto [builder, records] = parseRobotDescription(urdfDoc, srdfDoc.get(), filename, false, config);
  auto articulation = builder->build(fixRootLink);

  for (auto &record : records) {
    if (record.type == "camera") {
      std::vector<SLinkBase *> links = articulation->getBaseLinks();

      auto it = std::find_if(links.begin(), links.end(),
                             [&](SLinkBase *link) { return link->getName() == record.linkName; });

      if (it == links.end()) {
        spdlog::get("SAPIEN")->error("Failed to find the link to mount camera: ", record.linkName);
        continue;
      }

      mScene->addMountedCamera(record.name, *it, record.localPose, record.width, record.height,
                               record.fovx, record.fovy);
    }
  }

  return articulation;
}

SKArticulation *URDFLoader::loadKinematic(const std::string &filename, URDFConfig const &config) {
  if (filename.substr(filename.length() - 4) != std::string("urdf")) {
    throw std::invalid_argument("Non-URDF file passed to URDF loader");
  }
  auto srdfName = findSRDF(filename);

  std::unique_ptr<XMLDocument> srdfDoc = nullptr;
  if (srdfName) {
    srdfDoc = std::make_unique<XMLDocument>();
    if (srdfDoc->LoadFile(filename.c_str())) {
      srdfDoc = nullptr;
      spdlog::get("SAPIEN")->error("SRDF loading faild for {}", filename);
    }
  }

  XMLDocument urdfDoc;
  if (urdfDoc.LoadFile(filename.c_str())) {
    spdlog::get("SAPIEN")->error("Failed to open URDF file: {}", filename);
    return nullptr;
  }

  auto [builder, records] = parseRobotDescription(urdfDoc, srdfDoc.get(), filename, false, config);
  auto articulation = builder->buildKinematic();

  for (auto &record : records) {
    if (record.type == "camera") {
      std::vector<SLinkBase *> links = articulation->getBaseLinks();

      auto it = std::find_if(links.begin(), links.end(),
                             [&](SLinkBase *link) { return link->getName() == record.linkName; });

      if (it == links.end()) {
        spdlog::get("SAPIEN")->error("Failed to find the link to mount camera: ", record.linkName);
        continue;
      }

      mScene->addMountedCamera(record.name, *it, record.localPose, record.width, record.height,
                               record.fovx, record.fovy);
    }
  }

  return articulation;
}

std::unique_ptr<ArticulationBuilder>
URDFLoader::loadFileAsArticulationBuilder(const std::string &filename, URDFConfig const &config) {
  if (filename.substr(filename.length() - 4) != std::string("urdf")) {
    throw std::invalid_argument("Non-URDF file passed to URDF loader");
  }
  auto srdfName = findSRDF(filename);

  std::unique_ptr<XMLDocument> srdfDoc = nullptr;
  if (srdfName) {
    srdfDoc = std::make_unique<XMLDocument>();
    if (srdfDoc->LoadFile(filename.c_str())) {
      srdfDoc = nullptr;
      spdlog::get("SAPIEN")->error("SRDF loading faild for {}", filename);
    }
  }

  XMLDocument urdfDoc;
  if (urdfDoc.LoadFile(filename.c_str())) {
    spdlog::get("SAPIEN")->error("Failed to open URDF file: {}", filename);
    return nullptr;
  }
  return std::move(
      std::get<0>(parseRobotDescription(urdfDoc, srdfDoc.get(), filename, false, config)));
}

SArticulation *URDFLoader::loadFromXML(const std::string &URDFString,
                                       const std::string &SRDFString, URDFConfig const &config) {

  XMLDocument urdfDoc;
  if (urdfDoc.Parse(URDFString.c_str(), URDFString.length())) {
    spdlog::get("SAPIEN")->error("Failed to parse URDF string");
    return nullptr;
  }

  std::unique_ptr<XMLDocument> srdfDoc = nullptr;
  if (!SRDFString.empty()) {
    srdfDoc = std::make_unique<XMLDocument>();
    if (srdfDoc->Parse(SRDFString.c_str(), SRDFString.length())) {
      spdlog::get("SAPIEN")->error("Failed parsing given SRDF string.");
    }
  }

  auto [builder, records] = parseRobotDescription(urdfDoc, srdfDoc.get(), "", false, config);
  auto articulation = builder->build(fixRootLink);

  for (auto &record : records) {
    if (record.type == "camera") {
      std::vector<SLinkBase *> links = articulation->getBaseLinks();

      auto it = std::find_if(links.begin(), links.end(),
                             [&](SLinkBase *link) { return link->getName() == record.linkName; });

      if (it == links.end()) {
        spdlog::get("SAPIEN")->error("Failed to find the link to mount camera: ", record.linkName);
        continue;
      }

      mScene->addMountedCamera(record.name, *it, record.localPose, record.width, record.height,
                               record.fovx, record.fovy);
    }
  }

  return articulation;
};

} // namespace URDF
} // namespace sapien
