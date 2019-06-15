#include "urdf/urdf_loader.h"
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <tinyxml2.h>

namespace URDFUtil {
using namespace tinyxml2;

// TODO: Test this file!

physx::PxVec3 readVec3(const std::string &s) {
  float x, y, z;
  std::istringstream iss(s);
  iss >> x >> y >> z;
  return {x, y, z};
}

physx::PxVec4 readVec4(const std::string &s) {
  float w, x, y, z;
  std::istringstream iss(s);
  iss >> w >> x >> y >> z;
  return {w, x, y, z};
}

bool tagEqual(const char *t1, const char *t2) { return strcmp(t1, t2) == 0; }

Origin parseOrigin(const XMLElement *elem) {
  Origin origin;
  for (const XMLAttribute *attr = elem->FirstAttribute(); attr; attr = attr->Next()) {
    if (tagEqual("xyz", attr->Name())) {
      origin.xyz = readVec3(attr->Value());
    } else if (tagEqual("rpy", attr->Name())) {
      origin.rpy = readVec3(attr->Value());
    }
  }
  return origin;
}

Inertia parseInertia(const XMLElement *elem) {
  Inertia inertia;
  for (const XMLAttribute *attr = elem->FirstAttribute(); attr; attr = attr->Next()) {
    auto name = attr->Name();
    auto value = attr->Value();
    if (tagEqual("ixx", name)) {
      inertia.ixx = std::stof(value);
    } else if (tagEqual("ixy", name)) {
      inertia.ixy = std::stof(value);
    } else if (tagEqual("ixz", name)) {
      inertia.ixz = std::stof(value);
    } else if (tagEqual("iyy", name)) {
      inertia.iyy = std::stof(value);
    } else if (tagEqual("iyz", name)) {
      inertia.iyz = std::stof(value);
    } else if (tagEqual("izz", name)) {
      inertia.izz = std::stof(value);
    }
  }
  return inertia;
}

Inertial parseInertial(const XMLElement *elem) {
  Inertial inertial;
  for (const XMLElement *child = elem->FirstChildElement(); child;
       child = child->NextSiblingElement()) {
    auto name = child->Name();
    if (tagEqual("origin", name)) {
      inertial.origin = parseOrigin(child);
    } else if (tagEqual("mass", name)) {
      auto v = child->Attribute("value");
      if (v) {
        inertial.mass = std::stof(v);
      }
    } else if (tagEqual("inertia", name)) {
      inertial.inertia = parseInertia(child);
    }
  }
  return inertial;
}

Geometry parseGeometry(const XMLElement *elem) {
  Geometry geom;
  auto child = elem->FirstChildElement();
  if (!child) {
    std::cerr << "Geometry has no child at line " << elem->GetLineNum() << std::endl;
    exit(1);
  }
  auto name = child->Name();
  if (tagEqual("box", name)) {
    geom.type = Geometry::Type::Box;
    if (auto attr = child->FindAttribute("size")) {
      geom.size = readVec3(attr->Value());
    }
  } else if (tagEqual("cylinder", name)) {
    geom.type = Geometry::Type::Cylinder;
    if (auto attr = child->FindAttribute("radius")) {
      geom.radius = std::stof(attr->Value());
    }
    if (auto attr = child->FindAttribute("length")) {
      geom.length = std::stof(attr->Value());
    }
  } else if (tagEqual("sphere", name)) {
    geom.type = Geometry::Type::Sphere;
    if (auto attr = child->FindAttribute("radius")) {
      geom.radius = std::stof(attr->Value());
    }
  } else if (tagEqual("mesh", name)) {
    geom.type = Geometry::Type::Mesh;
    if (auto attr = child->FindAttribute("filename")) {
      geom.filename = attr->Value();
    }
    if (auto attr = child->FindAttribute("scale")) {
      geom.scale = readVec3(attr->Value());
    }
  } else {
    std::cerr << "Warning: unsupported geometry child tag at line " << child->GetLineNum()
              << std::endl;
  }
  if (child->NextSibling()) {
    std::cerr << "Warning: geometry should only have one child at line " << elem->GetLineNum()
              << std::endl;
  }
  return geom;
}

Material parseMaterial(const XMLElement *elem) {
  Material mat;
  if (auto attr = elem->FindAttribute("name")) {
    mat.name = attr->Value();
  }
  for (const XMLElement *child = elem->FirstChildElement(); child;
       child = child->NextSiblingElement()) {
    auto name = child->Name();
    if (tagEqual("color", name)) {
      if (auto attr = child->FindAttribute("rgba")) {
        mat.color = readVec4(attr->Value());
      }
    } else if (tagEqual("texture", name)) {
      if (auto attr = child->FindAttribute("filename")) {
        mat.texture = attr->Value();
      }
    } else {
      std::cerr << "Warning: Unsupported URDF Tag <" << std::string(name) << "/>" << std::endl;
    }
  }
  return mat;
}

Visual parseVisual(const XMLElement *elem) {
  Visual visual;
  if (auto attr = elem->FindAttribute("name")) {
    visual.name = attr->Value();
  }

  for (const XMLElement *child = elem->FirstChildElement(); child;
       child = child->NextSiblingElement()) {
    auto name = child->Name();

    if (tagEqual("origin", name)) {
      visual.origin = parseOrigin(child);
    } else if (tagEqual("geometry", name)) {
      visual.geometry = parseGeometry(child);
    } else if (tagEqual("material", name)) {
      visual.material = parseMaterial(child);
    } else {
      std::cerr << "Warning: Unsupported URDF Tag <" << std::string(name) << "/>" << std::endl;
    }
  }
  return visual;
}

Collision parseCollision(const XMLElement *elem) {
  Collision col;
  if (auto attr = elem->FindAttribute("name")) {
    col.name = attr->Value();
  }

  for (const XMLElement *child = elem->FirstChildElement(); child;
       child = child->NextSiblingElement()) {
    auto name = child->Name();

    if (tagEqual("origin", name)) {
      col.origin = parseOrigin(child);
    } else if (tagEqual("geometry", name)) {
      col.geometry = parseGeometry(child);
    } else {
      std::cerr << "Warning: Unsupported URDF Tag <" << std::string(name) << "/>" << std::endl;
    }
  }
  return col;
}

Dynamics parseDynamics(const XMLElement *elem) {
  Dynamics dyn;
  if (auto attr = elem->FindAttribute("damping")) {
    dyn.damping = std::stof(attr->Value());
  }
  if (auto attr = elem->FindAttribute("friction")) {
    dyn.friction = std::stof(attr->Value());
  }
  return dyn;
}

Limit parseLimit(const XMLElement *elem) {
  Limit limit;
  if (auto attr = elem->FindAttribute("lower")) {
    limit.lower = std::stof(attr->Value());
  }
  if (auto attr = elem->FindAttribute("upper")) {
    limit.upper = std::stof(attr->Value());
  }
  if (auto attr = elem->FindAttribute("effort")) {
    limit.effort = std::stof(attr->Value());
  }
  if (auto attr = elem->FindAttribute("velocity")) {
    limit.velocity = std::stof(attr->Value());
  }
  return limit;
}

Link parseLink(const XMLElement *elem) {
  Link link;
  if (auto attr = elem->FindAttribute("name")) {
    link.name = attr->Value();
  } else {
    std::cerr << "Link does not have name at line " << elem->GetLineNum() << std::endl;
    exit(1);
  }

  for (const XMLElement *child = elem->FirstChildElement(); child;
       child = child->NextSiblingElement()) {
    auto name = child->Name();
    if (tagEqual("inertial", name)) {
      link.inertial = parseInertial(child);
    } else if (tagEqual("visual", name)) {
      link.visuals.push_back(parseVisual(child));
    } else if (tagEqual("collision", name)) {
      link.collisions.push_back(parseCollision(child));
    }
  }
  return link;
}

Joint parseJoint(const XMLElement *elem) {
  Joint joint;
  if (auto attr = elem->FindAttribute("name")) {
    joint.name = attr->Value();
  } else {
    std::cerr << "Joint has no name at line " << elem->GetLineNum() << std::endl;
    exit(1);
  }

  if (auto attr = elem->FindAttribute("type")) {
    if (tagEqual("revolute", attr->Value())) {
      joint.type = Joint::Type::Revolute;
    } else if (tagEqual("continuous", attr->Value())) {
      joint.type = Joint::Type::Continuous;
    } else if (tagEqual("prismatic", attr->Value())) {
      joint.type = Joint::Type::Prismatic;
    } else if (tagEqual("fixed", attr->Value())) {
      joint.type = Joint::Type::Fixed;
    } else if (tagEqual("floating", attr->Value())) {
      joint.type = Joint::Type::Floating;
    } else if (tagEqual("plannar", attr->Value())) {
      joint.type = Joint::Type::Plannar;
    }
  } else {
    std::cerr << "Joint has no type at line " << elem->GetLineNum() << std::endl;
    exit(1);
  }

  for (const XMLElement *child = elem->FirstChildElement(); child;
       child = child->NextSiblingElement()) {
    auto name = child->Name();
    if (tagEqual("origin", name)) {
      joint.origin = parseOrigin(child);
    } else if (tagEqual("axis", name)) {
      if (auto attr = child->FindAttribute("xyz")) {
        joint.axis = readVec3(attr->Value());
      }
    } else if (tagEqual("dynamics", name)) {
      joint.dynamics = parseDynamics(child);
    } else if (tagEqual("limit", name)) {
      joint.limit = parseLimit(child);
    }
  }
  if (auto child = elem->FirstChildElement("parent")) {
    if (auto attr = child->FindAttribute("link")) {
      joint.parent = attr->Value();
    } else {
      std::cerr << "Invalid parent at line " << child->GetLineNum() << std::endl;
      exit(1);
    }
  } else {
    std::cerr << "Joint has no parent at line " << elem->GetLineNum() << std::endl;
    exit(1);
  }
  if (auto child = elem->FirstChildElement("child")) {
    if (auto attr = child->FindAttribute("link")) {
      joint.child = attr->Value();
    } else {
      std::cerr << "Invalid child at line " << child->GetLineNum() << std::endl;
      exit(1);
    }
  } else {
    std::cerr << "Joint has no child at line " << elem->GetLineNum() << std::endl;
    exit(1);
  }
  return joint;
}

Robot ParseRobot(const XMLElement *elem) {
  Robot robot;
  if (auto attr = elem->FindAttribute("name")) {
    robot.name = attr->Value();
  }
  for (const XMLElement *child = elem->FirstChildElement(); child;
       child = child->NextSiblingElement()) {
    auto name = child->Name();
    if (tagEqual("link", name)) {
      robot.links.push_back(parseLink(child));
    } else if (tagEqual("joint", name)) {
      robot.joints.push_back(parseJoint(child));
    } else {
      std::cerr << "Warning: Unsupported URDF Tag <" << std::string(child->Name()) << "/>"
                << std::endl;
    }
  }
  return robot;
}

physx::PxTransform getPose(const Origin &origin) {
  physx::PxQuat rot = physx::PxQuat(origin.rpy.x, {1, 0, 0}) *
                      physx::PxQuat(origin.rpy.y, {0, 1, 0}) *
                      physx::PxQuat(origin.rpy.z, {0, 0, 1});
  return physx::PxTransform(origin.xyz, rot);
}

struct LinkTreeNode {
  std::string name;
  std::string joint;
  LinkTreeNode *parent; // weak pointer but no need to test validity
  std::vector<std::shared_ptr<LinkTreeNode>> children;
};

void URDFLoader::load(const std::string &filename) {
  XMLDocument doc;
  doc.LoadFile(filename.c_str());
  XMLPrinter printer;
  if (std::string("robot").compare(doc.RootElement()->Name()) == 0) {
    robot = ParseRobot(doc.RootElement());
  } else {
    std::cerr << "Invalid URDF: root is not <robot/>" << std::endl;
    exit(1);
  }

  std::map<std::string, Link *> name2link;
  std::map<std::string, std::shared_ptr<LinkTreeNode>> name2treeNode;
  std::map<std::string, Joint *> name2joint;

  // fill name2link
  for (auto link : robot.links) {
    if (!link.name.empty()) {
      if (name2link.find(link.name) != name2link.end()) {
        // name duplication
        std::cerr << "Duplicated link name: " << link.name << std::endl;
        exit(1);
      }
      name2link[link.name] = &link;
      name2treeNode[link.name] = std::make_shared<LinkTreeNode>();
    }
  }

  // fill name2joint
  for (auto joint : robot.joints) {
    if (name2joint.find(joint.name) != name2joint.end()) {
      // name duplication
      std::cerr << "Duplicated joint name: " << joint.name << std::endl;
      exit(1);
    }
    name2joint[joint.name] = &joint;
  }

  // fill name2treeNode
  for (auto joint : robot.joints) {
    if (name2treeNode.find(joint.parent) == name2treeNode.end()) {
      std::cerr << "Invalid joint parent: " << joint.parent << std::endl;
      exit(1);
    }
    if (name2treeNode.find(joint.child) == name2treeNode.end()) {
      std::cerr << "Invalid joint child: " << joint.child << std::endl;
      exit(1);
    }
    if (!name2treeNode[joint.child]->joint.empty()) {
      std::cerr << "Kinetic loop detected: forward loop" << std::endl;
      exit(1);
    }
    name2treeNode[joint.parent]->children.push_back(name2treeNode[joint.child]);
    name2treeNode[joint.child]->parent = name2treeNode[joint.parent].get();
    name2treeNode[joint.child]->joint = joint.name;
  }

  // find root
  LinkTreeNode *root = nullptr;
  for (auto nt : name2treeNode) {
    if (!nt.second->parent) {
      if (root) {
        std::cerr << "There appears to be 2 root links in a robot. This is not supported."
                  << std::endl;
        exit(1);
      }
      root = nt.second.get();
    }
  }

  // check the tree property
  std::map<LinkTreeNode *, bool> link2visited;
  std::vector<LinkTreeNode *> stack = {root};
  while (!stack.empty()) {
    LinkTreeNode *current = stack.back();
    stack.pop_back();
    if (link2visited.find(current) != link2visited.end()) {
      std::cerr << "Kinetic loop detected: backward loop" << std::endl;
      exit(1);
    }
    link2visited[current] = true;
    for (auto c : current->children) {
      stack.push_back(c.get());
    }
  }

  // traverse and build the tree
  articulation = physicsSDK->createArticulation();
  std::map<LinkTreeNode *, physx::PxArticulationLink *> link2physxLink;
  stack = {root};
  while (!stack.empty()) {
    LinkTreeNode *current = stack.back();
    stack.pop_back();
    physx::PxArticulationLink *link;
    if (!current->parent) {
      link = articulation->createLink(nullptr, physx::PxTransform(physx::PxIdentity));
    } else {
      link = articulation->createLink(link2physxLink[current->parent],
                                      physx::PxTransform(physx::PxIdentity));
    }
    link2physxLink[current] = link;
    for (auto c : current->children) {
      stack.push_back(c.get());
    }
  }

  // TODO: create shape, sync visual, add joint
}

} // namespace URDFUtil
