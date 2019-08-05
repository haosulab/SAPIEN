#pragma once
#include <PxPhysicsAPI.h>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <tinyxml2.h>
#include <vector>

using namespace tinyxml2;

#define DECLARE_ATTR(type, name) type name;

#define DECLARE_CHILD_UNIQUE(type, name) std::unique_ptr<type> name;

#define DECLARE_CHILD(type, name) std::vector<std::unique_ptr<type>> name##_array;

#define DECLARE_CONSTRUCTOR(type)                                                                 \
  type() {}                                                                                       \
  type(const XMLElement &elem) {                                                                  \
    for (const XMLElement *child = elem.FirstChildElement(); child;                               \
         child = child->NextSiblingElement()) {                                                   \
      if (child) {                                                                                \
        loadChild(*child);                                                                        \
      }                                                                                           \
    }                                                                                             \
    checkChildren();                                                                              \
    loadAttrs(elem);                                                                              \
  }

#define LOAD_CHILD_BEGIN()                                                                        \
  void loadChild(const XMLElement &child) {                                                       \
    const char *tag = child.Name();

#define LOAD_CHILD_END()                                                                          \
  std::cerr << "Unknown child type \"" << tag << "\". Ignored" << std::endl;                      \
  }

#define LOAD_CHILD_UNIQUE(type, name)                                                             \
  if (strcmp(tag, #name) == 0) {                                                                  \
    this->name = std::make_unique<type>(child);                                                   \
    return;                                                                                       \
  }

#define LOAD_CHILD(type, name)                                                                    \
  if (strcmp(tag, #name) == 0) {                                                                  \
    this->name##_array.push_back(std::make_unique<type>(child));                                  \
    return;                                                                                       \
  }

#define LOAD_ATTR_BEGIN() void loadAttrs(const XMLElement &elem) {

#define LOAD_ATTR_END() }

#define LOAD_ATTR(type, name) name = read_attr<type>(elem, #name);

#define LOAD_ATTR_OPTIONAL(type, name, deft) name = read_attr_optional<type>(elem, #name, deft);

#define CHECK_CHILD_BEGIN() void checkChildren() {

#define CHECK_CHILD_END() }

/* Error if a unique child is not set */
#define CHECK_CHILD_UNIQUE(name)                                                                  \
  if (!name) {                                                                                    \
    std::cerr << "Missing required child " << #name << std::endl;                                 \
    exit(1);                                                                                      \
  }

/* Error if a child array is empty */
#define CHECK_CHILD(name)                                                                         \
  if (name##_array.empty()) {                                                                     \
    std::cerr << "Missing required children " << #name << std::endl;                              \
    exit(1);                                                                                      \
  }

/* If a unique child is not set, run code */
#define CHECK_CHILD_UNIQUE_SET_DEFAULT(name, ...)                                                 \
  if (!name) {                                                                                    \
    __VA_ARGS__                                                                                   \
  }

struct DomBase {

  void loadAttrs(const XMLElement &elem) {}
  void loadChild(const XMLElement &child) {}
  void checkChildren() {}

  template <typename T> static inline T _read_attr(const std::string &str);

  template <typename T>
  static inline T read_attr(const XMLElement &elem, const std::string &name) {
    const char *result = elem.Attribute(name.c_str());
    if (result) {
      return _read_attr<T>(result);
    }
    throw "Attribute " + name + " does not exist.";
  }

  template <typename T>
  static inline T read_attr_optional(const XMLElement &elem, const std::string &name, const T &d) {
    const char *result = elem.Attribute(name.c_str());
    if (result) {
      return _read_attr<T>(result);
    }
    return d;
  }
};

template <> inline std::string DomBase::_read_attr<std::string>(const std::string &str) {
  return str;
}

template <> inline physx::PxVec3 DomBase::_read_attr<physx::PxVec3>(const std::string &str) {
  float x, y, z;
  std::istringstream iss(str);
  iss >> x >> y >> z;
  return {x, y, z};
}

template <> inline physx::PxReal DomBase::_read_attr<physx::PxReal>(const std::string &str) {
  return std::atof(str.c_str());
}

template <> inline int DomBase::_read_attr<int>(const std::string &str) {
  return std::atoi(str.c_str());
}

struct Origin : DomBase {
  DECLARE_CONSTRUCTOR(Origin)

  DECLARE_ATTR(physx::PxVec3, rpy)
  DECLARE_ATTR(physx::PxVec3, xyz)

  LOAD_ATTR_BEGIN()
  LOAD_ATTR_OPTIONAL(physx::PxVec3, rpy, physx::PxVec3(0, 0, 0))
  LOAD_ATTR_OPTIONAL(physx::PxVec3, xyz, physx::PxVec3(0, 0, 0))
  LOAD_ATTR_END()
};

struct Inertia : DomBase {
  DECLARE_CONSTRUCTOR(Inertia)
  DECLARE_ATTR(physx::PxReal, ixx)
  DECLARE_ATTR(physx::PxReal, ixy)
  DECLARE_ATTR(physx::PxReal, ixz)
  DECLARE_ATTR(physx::PxReal, iyy)
  DECLARE_ATTR(physx::PxReal, iyz)
  DECLARE_ATTR(physx::PxReal, izz)

  LOAD_ATTR_BEGIN()
  LOAD_ATTR(physx::PxReal, ixx)
  LOAD_ATTR(physx::PxReal, ixy)
  LOAD_ATTR(physx::PxReal, ixz)
  LOAD_ATTR(physx::PxReal, iyy)
  LOAD_ATTR(physx::PxReal, iyz)
  LOAD_ATTR(physx::PxReal, izz)
  LOAD_ATTR_END()
};

struct Mass : DomBase {
  DECLARE_CONSTRUCTOR(Mass)
  DECLARE_ATTR(physx::PxReal, value)

  LOAD_ATTR_BEGIN()
  LOAD_ATTR(physx::PxReal, value)
  LOAD_ATTR_END()
};

struct Inertial : DomBase {
  DECLARE_CONSTRUCTOR(Inertial)
  DECLARE_CHILD_UNIQUE(Origin, origin)
  DECLARE_CHILD_UNIQUE(Mass, mass)
  DECLARE_CHILD_UNIQUE(Inertia, inertia)

  LOAD_CHILD_BEGIN()
  LOAD_CHILD_UNIQUE(Origin, origin)
  LOAD_CHILD_UNIQUE(Mass, mass)
  LOAD_CHILD_UNIQUE(Inertia, inertia)
  LOAD_CHILD_END()

  CHECK_CHILD_BEGIN()
  CHECK_CHILD_UNIQUE_SET_DEFAULT(origin, {
    origin = std::make_unique<Origin>();
    origin->xyz = {0, 0, 0};
    origin->rpy = {0, 0, 0};
  })
  CHECK_CHILD_UNIQUE(mass)
  CHECK_CHILD_UNIQUE(inertia)
  CHECK_CHILD_END()
};

struct Box : DomBase {
  DECLARE_CONSTRUCTOR(Box)
  DECLARE_ATTR(physx::PxVec3, size)

  LOAD_ATTR_BEGIN()
  LOAD_ATTR(physx::PxVec3, size)
  LOAD_ATTR_END()
};

struct Cylinder : DomBase {
  DECLARE_CONSTRUCTOR(Cylinder)
  DECLARE_ATTR(physx::PxReal, radius)
  DECLARE_ATTR(physx::PxReal, length)

  LOAD_ATTR_BEGIN()
  LOAD_ATTR(physx::PxReal, radius)
  LOAD_ATTR(physx::PxReal, length)
  LOAD_ATTR_END()
};

struct Sphere : DomBase {
  DECLARE_CONSTRUCTOR(Sphere)
  DECLARE_ATTR(physx::PxReal, radius)

  LOAD_ATTR_BEGIN()
  LOAD_ATTR(physx::PxReal, radius)
  LOAD_ATTR_END()
};

struct Mesh : DomBase {
  DECLARE_CONSTRUCTOR(Mesh)
  DECLARE_ATTR(std::string, filename)
  DECLARE_ATTR(physx::PxVec3, scale)

  LOAD_ATTR_BEGIN()
  LOAD_ATTR(std::string, filename)
  LOAD_ATTR_OPTIONAL(physx::PxVec3, scale, physx::PxVec3(1.f, 1.f, 1.f))
  LOAD_ATTR_END()
};

struct Geometry : DomBase {
  enum Type { BOX, CYLINDER, SPHERE, MESH } type;
  physx::PxVec3 size;
  physx::PxReal radius;
  physx::PxReal length;
  std::string filename;
  physx::PxVec3 scale;

  Geometry() {}
  Geometry(const XMLElement &elem) {
    const XMLElement *child = elem.FirstChildElement();
    if (!child) {
      std::cerr << "gemoetry elements contains no child" << std::endl;
      exit(1);
    }
    if (child->NextSibling()) {
      std::cerr << "gemoetry elements contains more than 1 child" << std::endl;
      exit(1);
    }
    const char *childTag = child->Name();
    if (strcmp(childTag, "box") == 0) {
      type = BOX;
      auto g = Box(*child);
      size = g.size;
      return;
    }

    if (strcmp(childTag, "cylinder") == 0) {
      type = CYLINDER;
      auto g = Cylinder(*child);
      radius = g.radius;
      length = g.length;
      return;
    }

    if (strcmp(childTag, "sphere") == 0) {
      type = SPHERE;
      auto g = Sphere(*child);
      radius = g.radius;
      return;
    }

    if (strcmp(childTag, "mesh") == 0) {
      type = MESH;
      auto g = Mesh(*child);
      filename = g.filename;
      scale = g.scale;
      return;
    }
  }
};

// We do not care about material for now
struct Visual : DomBase {
  DECLARE_CONSTRUCTOR(Visual)
  DECLARE_ATTR(std::string, name)
  DECLARE_CHILD_UNIQUE(Origin, origin)
  DECLARE_CHILD_UNIQUE(Geometry, geometry)

  LOAD_CHILD_BEGIN()
  LOAD_CHILD_UNIQUE(Origin, origin)
  LOAD_CHILD_UNIQUE(Geometry, geometry)
  LOAD_CHILD_END()

  LOAD_ATTR_BEGIN()
  LOAD_ATTR_OPTIONAL(std::string, name, "")
  LOAD_ATTR_END()

  CHECK_CHILD_BEGIN()
  CHECK_CHILD_UNIQUE_SET_DEFAULT(origin, {
    origin = std::make_unique<Origin>();
    origin->xyz = {0, 0, 0};
    origin->rpy = {0, 0, 0};
  })
  CHECK_CHILD_UNIQUE(geometry)
  CHECK_CHILD_END()
};

struct Collision : DomBase {
  DECLARE_CONSTRUCTOR(Collision)
  DECLARE_ATTR(std::string, name)
  DECLARE_CHILD_UNIQUE(Origin, origin)
  DECLARE_CHILD_UNIQUE(Geometry, geometry)

  LOAD_CHILD_BEGIN()
  LOAD_CHILD_UNIQUE(Origin, origin)
  LOAD_CHILD_UNIQUE(Geometry, geometry)
  LOAD_CHILD_END()

  LOAD_ATTR_BEGIN()
  LOAD_ATTR_OPTIONAL(std::string, name, "")
  LOAD_ATTR_END()

  CHECK_CHILD_BEGIN()
  CHECK_CHILD_UNIQUE_SET_DEFAULT(origin, {
    origin = std::make_unique<Origin>();
    origin->xyz = {0, 0, 0};
    origin->rpy = {0, 0, 0};
  })
  CHECK_CHILD_UNIQUE(geometry)
  CHECK_CHILD_END()
};

struct Link : DomBase {
  DECLARE_CONSTRUCTOR(Link)
  DECLARE_ATTR(std::string, name)
  DECLARE_CHILD_UNIQUE(Inertial, inertial)
  DECLARE_CHILD(Visual, visual)
  DECLARE_CHILD(Collision, collision)

  LOAD_CHILD_BEGIN()
  LOAD_CHILD_UNIQUE(Inertial, inertial)
  LOAD_CHILD(Visual, visual)
  LOAD_CHILD(Collision, collision)
  LOAD_CHILD_END()

  LOAD_ATTR_BEGIN()
  LOAD_ATTR(std::string, name)
  CHECK_CHILD_UNIQUE_SET_DEFAULT(inertial, {
      inertial = std::make_unique<Inertial>();
      inertial->mass = std::make_unique<Mass>();
      inertial->mass->value = 1;

      inertial->origin = std::make_unique<Origin>();
      inertial->origin->xyz = {0, 0, 0};
      inertial->origin->rpy = {0, 0, 0};

      inertial->inertia = std::make_unique<Inertia>();
      inertial->inertia->ixx = 1;
      inertial->inertia->iyy = 1;
      inertial->inertia->izz = 1;
      inertial->inertia->ixy = 0;
      inertial->inertia->ixz = 0;
      inertial->inertia->iyz = 0;
    })
  LOAD_ATTR_END()

  // no need to check child
};

struct ParentChild : DomBase {
  DECLARE_CONSTRUCTOR(ParentChild)
  DECLARE_ATTR(std::string, link)

  LOAD_ATTR_BEGIN()
  LOAD_ATTR(std::string, link)
  LOAD_ATTR_END()
};

struct Dynamics : DomBase {
  DECLARE_CONSTRUCTOR(Dynamics)
  DECLARE_ATTR(physx::PxReal, damping)
  DECLARE_ATTR(physx::PxReal, friction)

  LOAD_ATTR_BEGIN()
  LOAD_ATTR(physx::PxReal, damping)
  LOAD_ATTR(physx::PxReal, friction)
  LOAD_ATTR_END()
};

struct Limit : DomBase {
  DECLARE_CONSTRUCTOR(Limit)
  DECLARE_ATTR(physx::PxReal, lower)
  DECLARE_ATTR(physx::PxReal, upper)
  DECLARE_ATTR(physx::PxReal, effort)
  DECLARE_ATTR(physx::PxReal, velocity)

  LOAD_ATTR_BEGIN()
  LOAD_ATTR_OPTIONAL(physx::PxReal, lower, 0)
  LOAD_ATTR_OPTIONAL(physx::PxReal, upper, 0)
  LOAD_ATTR(physx::PxReal, effort)
  LOAD_ATTR(physx::PxReal, velocity)
  LOAD_ATTR_END()
};

struct Axis : DomBase {
  DECLARE_CONSTRUCTOR(Axis)
  DECLARE_ATTR(physx::PxVec3, xyz)

  LOAD_ATTR_BEGIN()
  LOAD_ATTR(physx::PxVec3, xyz)
  LOAD_ATTR_END()
};

struct Joint : DomBase {
  DECLARE_CONSTRUCTOR(Joint)
  DECLARE_ATTR(std::string, name)
  DECLARE_ATTR(std::string, type)

  DECLARE_CHILD_UNIQUE(Origin, origin)
  DECLARE_CHILD_UNIQUE(ParentChild, parent)
  DECLARE_CHILD_UNIQUE(ParentChild, child)
  DECLARE_CHILD_UNIQUE(Dynamics, dynamics)
  DECLARE_CHILD_UNIQUE(Axis, axis)
  DECLARE_CHILD_UNIQUE(Limit, limit)

  LOAD_CHILD_BEGIN()
  LOAD_CHILD_UNIQUE(Origin, origin)
  LOAD_CHILD_UNIQUE(ParentChild, parent)
  LOAD_CHILD_UNIQUE(ParentChild, child)
  LOAD_CHILD_UNIQUE(Dynamics, dynamics)
  LOAD_CHILD_UNIQUE(Axis, axis)
  LOAD_CHILD_UNIQUE(Limit, limit)
  LOAD_CHILD_END()

  LOAD_ATTR_BEGIN()
  LOAD_ATTR(std::string, name)
  LOAD_ATTR(std::string, type)
  LOAD_ATTR_END()

  CHECK_CHILD_BEGIN()
  CHECK_CHILD_UNIQUE_SET_DEFAULT(origin, {
    origin = std::make_unique<Origin>();
    origin->xyz = {0, 0, 0};
    origin->rpy = {0, 0, 0};
  })
  CHECK_CHILD_UNIQUE_SET_DEFAULT(axis, {
    axis = std::make_unique<Axis>();
    axis->xyz = {1.f, 0.f, 0.f};
  })
  CHECK_CHILD_UNIQUE(parent)
  CHECK_CHILD_UNIQUE(child)
  CHECK_CHILD_UNIQUE_SET_DEFAULT(dynamics,
                                 {
                                   dynamics = std::make_unique<Dynamics>();
                                   dynamics->friction = 0.f;
                                   dynamics->damping = 0.f;
                                 })

  // custom code for child checking
  {
    if (type == "revolute" || type == "prismatic") {
      if (!limit) {
        std::cerr << "Limit is required for " << type << std::endl;
        exit(1);
      }
    }
  }
  CHECK_CHILD_END()
};

struct Robot : DomBase {
  DECLARE_CONSTRUCTOR(Robot)

  DECLARE_CHILD(Link, link)
  DECLARE_CHILD(Joint, joint)

  LOAD_CHILD_BEGIN()
  LOAD_CHILD(Link, link);
  LOAD_CHILD(Joint, joint);
  LOAD_CHILD_END()
};

class URDFLoader {
  std::unique_ptr<Robot> robot;
  physx::PxArticulationReducedCoordinate *articulation;
  class PxSimulation &mSimulation;

public:
  URDFLoader(class PxSimulation &simulation);
  class PxArticulationInterface load(const std::string &filename);
};
