#pragma once
#include <PxPhysicsAPI.h>
#include <iostream>
#include <map>
#include <memory>
#include <spdlog/spdlog.h>
#include <sstream>
#include <string>
#include <tinyxml2.h>
#include <vector>

namespace sapien {
class SScene;
class SArticulation;
class SKArticulation;
class SArticulationBase;
class ArticulationBuilder;
class SPhysicalMaterial;

namespace URDF {
using namespace tinyxml2;

#define DECLARE_ATTR(type, name) type name;

#define DECLARE_CHILD_UNIQUE(type, name) std::unique_ptr<type> name;

#define DECLARE_CHILD(type, name) std::vector<std::unique_ptr<type>> name##_array;

#define DECLARE_CONSTRUCTOR(type)                                                                 \
  type() {}                                                                                       \
  explicit type(const XMLElement &elem) {                                                         \
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

#define LOAD_CHILD_END() }

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
    spdlog::get("SAPIEN")->critical("Missing required child <{}>", #name);                        \
    throw std::runtime_error("Missing required child");                                           \
  }

/* Error if a child array is empty */
#define CHECK_CHILD(name)                                                                         \
  if (name##_array.empty()) {                                                                     \
    spdlog::get("SAPIEN")->critical("Missing required children <{}>", #name);                     \
    throw std::runtime_error("Missing required child");                                           \
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
    spdlog::get("SAPIEN")->critical("Attribute {} does not exist on {}, at line {}.", name,
                                    elem.Name(), elem.GetLineNum());
    throw std::runtime_error("Missing attribute");
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
  std::string str2 = str;
  std::replace(str2.begin(), str2.end(), ',', ' ');

  std::string sx;
  std::string sy;
  std::string sz;
  std::istringstream iss(str2);
  iss >> sx >> sy >> sz;
  float x = 0, y = 0, z = 0;
  try {
    x = std::stof(sx);
  } catch (std::invalid_argument const &) {
  }
  try {
    y = std::stof(sy);
  } catch (std::invalid_argument const &) {
  }
  try {
    z = std::stof(sz);
  } catch (std::invalid_argument const &) {
  }
  return {x, y, z};
}

template <> inline physx::PxVec4 DomBase::_read_attr<physx::PxVec4>(const std::string &str) {
  std::string str2 = str;
  std::replace(str2.begin(), str2.end(), ',', ' ');

  std::string sx;
  std::string sy;
  std::string sz;
  std::string sw;
  std::istringstream iss(str2);
  iss >> sx >> sy >> sz >> sw;
  float x = 0, y = 0, z = 0, w = 0;
  try {
    x = std::stof(sx);
  } catch (std::invalid_argument const &) {
  }
  try {
    y = std::stof(sy);
  } catch (std::invalid_argument const &) {
  }
  try {
    z = std::stof(sz);
  } catch (std::invalid_argument const &) {
  }
  try {
    w = std::stof(sw);
  } catch (std::invalid_argument const &) {
  }
  return {x, y, z, w};
}

template <> inline physx::PxReal DomBase::_read_attr<physx::PxReal>(const std::string &str) {
  return std::stof(str);
}

template <> inline int DomBase::_read_attr<int>(const std::string &str) { return std::stoi(str); }

namespace SRDF {

struct DisableCollisions : DomBase {
  DECLARE_CONSTRUCTOR(DisableCollisions)

  DECLARE_ATTR(std::string, link1)
  DECLARE_ATTR(std::string, link2)
  DECLARE_ATTR(std::string, reason)

  LOAD_ATTR_BEGIN()
  LOAD_ATTR(std::string, link1)
  LOAD_ATTR(std::string, link2)
  LOAD_ATTR(std::string, reason)
  LOAD_ATTR_END()
};

struct Robot : DomBase {
  DECLARE_CONSTRUCTOR(Robot)

  DECLARE_CHILD(DisableCollisions, disable_collisions)

  LOAD_CHILD_BEGIN()
  LOAD_CHILD(DisableCollisions, disable_collisions);
  LOAD_CHILD_END()
};

} // namespace SRDF

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

typedef Cylinder Capsule;

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
  enum Type { BOX, CYLINDER, SPHERE, MESH, CAPSULE } type;
  physx::PxVec3 size;
  physx::PxReal radius;
  physx::PxReal length;
  std::string filename;
  physx::PxVec3 scale;

  Geometry() {}
  Geometry(const XMLElement &elem) {
    const XMLElement *child = elem.FirstChildElement();
    if (!child) {
      spdlog::get("SAPIEN")->critical("<geometry> contains no child, at line {}",
                                      elem.GetLineNum());
      throw std::runtime_error("<geometry> contains no child");
    }
    if (child->NextSiblingElement()) {
      spdlog::get("SAPIEN")->critical("<geometry> contains more than 1 child, at line {}",
                                      elem.GetLineNum());
      throw std::runtime_error("<geometry> contains more than 1 child");
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

    if (strcmp(childTag, "capsule") == 0) {
      type = CAPSULE;
      auto g = Capsule(*child);
      radius = g.radius;
      length = g.length;
      return;
    }

    spdlog::get("SAPIEN")->critical("Unrecognized geometry tag <{}>", childTag);
    exit(1);
  }
};

struct MaterialColor : DomBase {
  DECLARE_CONSTRUCTOR(MaterialColor)
  DECLARE_ATTR(physx::PxVec4, rgba)

  LOAD_ATTR_BEGIN()
  LOAD_ATTR_OPTIONAL(physx::PxVec4, rgba, physx::PxVec4(1, 1, 1, 1))
  LOAD_ATTR_END()
};

// texture not supported
struct RenderMaterial : DomBase {
  DECLARE_CONSTRUCTOR(RenderMaterial)
  DECLARE_ATTR(std::string, name)
  DECLARE_CHILD_UNIQUE(MaterialColor, color)

  LOAD_ATTR_BEGIN()
  LOAD_ATTR(std::string, name)
  LOAD_ATTR_END()

  LOAD_CHILD_BEGIN()
  LOAD_CHILD_UNIQUE(MaterialColor, color)
  LOAD_CHILD_END()

  CHECK_CHILD_BEGIN()
  CHECK_CHILD_UNIQUE_SET_DEFAULT(color, { color = nullptr; })
  CHECK_CHILD_END()
};

// We do not care about material for now
struct Visual : DomBase {
  DECLARE_CONSTRUCTOR(Visual)
  DECLARE_ATTR(std::string, name)
  DECLARE_CHILD_UNIQUE(Origin, origin)
  DECLARE_CHILD_UNIQUE(Geometry, geometry)
  DECLARE_CHILD_UNIQUE(RenderMaterial, material)

  LOAD_CHILD_BEGIN()
  LOAD_CHILD_UNIQUE(Origin, origin)
  LOAD_CHILD_UNIQUE(Geometry, geometry)
  LOAD_CHILD_UNIQUE(RenderMaterial, material)
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
  CHECK_CHILD_UNIQUE_SET_DEFAULT(material, { material = nullptr; });
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
    inertial->inertia->ixx = 0;
    inertial->inertia->iyy = 0;
    inertial->inertia->izz = 0;
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
  LOAD_ATTR_OPTIONAL(physx::PxReal, damping, 0)
  LOAD_ATTR_OPTIONAL(physx::PxReal, friction, 0)
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
  LOAD_ATTR_OPTIONAL(physx::PxReal, effort, 0)
  LOAD_ATTR_OPTIONAL(physx::PxReal, velocity, 0)
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
        spdlog::get("SAPIEN")->critical("Missing required attribute [limit] on <{}>", type);
        throw std::runtime_error("Missing required attribute");
      }
    }
  }
  CHECK_CHILD_END()
};

struct Camera {
  float near;
  float far;
  uint32_t width;
  uint32_t height;
  float fovx;
  float fovy;
};

struct Sensor : DomBase {
  enum Type { CAMERA, DEPTH, RAY, UNKNOWN } type;
  std::string name;
  std::unique_ptr<Origin> origin;
  std::unique_ptr<Camera> camera;

  Sensor() {}
  Sensor(const XMLElement &elem) {
    const char *name_ = elem.Attribute("name");
    if (!name_) {
      name_ = "";
    }
    name = name_;

    const char *type_ = elem.Attribute("type");
    if (!type_) {
      spdlog::get("SAPIEN")->critical("Missing attribute [type] on <sensor>, at line {}",
                                      elem.GetLineNum());
      throw std::runtime_error("Missing attribute [type] on <sensor>");
    }
    std::string type_string = type_;
    if (type_string == "camera") {
      type = CAMERA;
    } else if (type_string == "depth") {
      type = DEPTH;
    } else if (type_string == "ray") {
      type = RAY;
    } else {
      type = UNKNOWN;
      spdlog::get("SAPIEN")->warn("Sensor type " + type_string + " is not supported");
      return;
    }
    auto pose = elem.FirstChildElement("pose");
    origin = std::make_unique<Origin>();
    if (!pose) {
      origin->xyz = {0, 0, 0};
      origin->rpy = {0, 0, 0};
    } else {
      float x, y, z, roll, pitch, yaw;
      std::istringstream iss(pose->GetText());
      iss >> x >> y >> z >> roll >> pitch >> yaw;
      origin->xyz = {x, y, z};
      origin->rpy = {roll, pitch, yaw};
    }
    if (type == CAMERA || type == DEPTH) {
      loadCamera(elem);
    }
  }

  void loadCamera(const XMLElement &elem) {
    this->camera = std::make_unique<Camera>();
    auto camera = elem.FirstChildElement("camera");
    if (!camera) {
      spdlog::get("SAPIEN")->critical(
          "Missing <camera> child on color or depth camera sensor, at line {}", elem.GetLineNum());
      throw std::runtime_error("Missing <camera> child on color or depth camera sensor");
    }
    auto fovx = camera->FirstChildElement("horizontal_fov");
    auto fovy = camera->FirstChildElement("vertical_fov");
    if (!fovx && !fovy) {
      spdlog::get("SAPIEN")->critical("Missing horizontal_fov/vertical_fov on camera, at line {}",
                                      elem.GetLineNum());
      throw std::runtime_error("Missing horizontal_fov/vertical_fov on camera");
    }
    auto clip = camera->FirstChildElement("clip");
    auto image = camera->FirstChildElement("image");
    if (!clip || !image) {
      spdlog::get("SAPIEN")->critical("Missing <clip> or <image> on camera {}", elem.GetLineNum());
      throw std::runtime_error("Missing <clip> or <image> on camera");
    }
    auto near = clip->FirstChildElement("near");
    auto far = clip->FirstChildElement("far");
    if (!near || !far) {
      spdlog::get("SAPIEN")->critical("Missing near/far on clip {}", elem.GetLineNum());
      throw std::runtime_error("Missing near/far on clip");
    }
    float nearValue = std::atof(near->GetText());
    float farValue = std::atof(far->GetText());

    if (image->FirstChildElement("format")) {
      spdlog::get("SAPIEN")->warn("Ignored <format> on camera");
    }
    auto width = image->FirstChildElement("width");
    auto height = image->FirstChildElement("height");
    if (!width || !height) {
      spdlog::get("SAPIEN")->critical("Missing <width> or <height> on image {}",
                                      elem.GetLineNum());
      throw std::runtime_error("Missing <width> or <height> on image");
    }
    float widthValue = std::atoi(width->GetText());
    float heightValue = std::atoi(height->GetText());
    float fovxValue, fovyValue;
    if (fovx && fovy) {
      fovxValue = std::atof(fovx->GetText());
      fovyValue = std::atof(fovy->GetText());
    } else if (fovx) {
      fovxValue = std::atof(fovx->GetText());
      fovyValue = std::atan(std::tan(fovxValue / 2) / widthValue * heightValue) * 2;
    } else { // fovy
      fovyValue = std::atof(fovy->GetText());
      fovxValue = std::atan(std::tan(fovyValue / 2) / heightValue * widthValue) * 2;
    }
    this->camera->fovx = fovxValue;
    this->camera->fovy = fovyValue;
    this->camera->near = nearValue;
    this->camera->far = farValue;
    this->camera->width = widthValue;
    this->camera->height = heightValue;
  }
};

struct Gazebo : DomBase {
  DECLARE_CONSTRUCTOR(Gazebo)
  DECLARE_ATTR(std::string, reference)
  DECLARE_CHILD(Sensor, sensor)

  LOAD_ATTR_BEGIN()
  LOAD_ATTR_OPTIONAL(std::string, reference, "")
  LOAD_ATTR_END()

  LOAD_CHILD_BEGIN()
  LOAD_CHILD(Sensor, sensor);
  LOAD_CHILD_END()
};

struct Robot : DomBase {
  DECLARE_CONSTRUCTOR(Robot)
  DECLARE_CHILD(Link, link)
  DECLARE_CHILD(Joint, joint)
  DECLARE_CHILD(RenderMaterial, material)
  DECLARE_CHILD(Gazebo, gazebo)
  LOAD_CHILD_BEGIN()
  LOAD_CHILD(Link, link);
  LOAD_CHILD(Joint, joint);
  LOAD_CHILD(RenderMaterial, material)
  LOAD_CHILD(Gazebo, gazebo)
  LOAD_CHILD_END()
};

struct URDFConfig {
  struct ShapeConfig {
    std::shared_ptr<SPhysicalMaterial> material = nullptr;
    float patchRadius = 0.f;
    float minPatchRadius = 0.f;
    float density = 1000.f;
  };
  struct LinkConfig {
    // default material for a link
    std::shared_ptr<SPhysicalMaterial> material = nullptr;
    std::map<int, ShapeConfig> shape;
    // patch radius for the whole link
    float patchRadius = 0.f;
    float minPatchRadius = 0.f;
    // density for the whole link
    float density = 1000.f;
  };
  std::map<std::string, LinkConfig> link = {};

  std::shared_ptr<SPhysicalMaterial> material = nullptr;
  float density = 1000.f;
};

struct SensorRecord {
  std::string type;
  std::string name;
  std::string linkName;
  physx::PxTransform localPose;
  uint32_t width;
  uint32_t height;
  float fovx;
  float fovy;
  float near;
  float far;
};

class URDFLoader {
  SScene *mScene;
  std::string mUrdfString;

public:
  /* The base of the loaded articulation will be fixed */
  bool fixRootLink = true;

  bool multipleMeshesInOneFile = false;

  /* Load the articulation at a different scale.
   * It will scale mass and inertia accordingly
   */
  float scale = 1.f;

  /* collision will be rendered along with visual */
  bool collisionIsVisual = false;

  /* directory for package:// */
  std::string packageDir = "";

  explicit URDFLoader(SScene *scene);

  SArticulation *load(const std::string &filename, URDFConfig const &config = {});

  SKArticulation *loadKinematic(const std::string &filename, URDFConfig const &config = {});

  /* Directly load robot model from string, srdf is optional */
  /* Using this mode, the path in URDF should be absolute path, rather than relative one*/
  SArticulation *loadFromXML(const std::string &URDFString, const std::string &SRDFString = "",
                             URDFConfig const &config = {});

  std::shared_ptr<ArticulationBuilder>
  loadFileAsArticulationBuilder(const std::string &filename, URDFConfig const &config = {});

private:
  std::tuple<std::shared_ptr<ArticulationBuilder>, std::vector<SensorRecord>>
  parseRobotDescription(XMLDocument const &urdfDoc, XMLDocument const *srdfDoc,
                        const std::string &urdfFilename, bool isKinematic,
                        URDFConfig const &config);
};

} // namespace URDF
} // namespace sapien
