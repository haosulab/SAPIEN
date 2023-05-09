#include "sapien/articulation/urdf_loader.h"
#include "sapien/articulation/articulation_builder.h"
#include "sapien/articulation/sapien_articulation.h"
#include "sapien/articulation/sapien_kinematic_articulation.h"
#include "sapien/articulation/sapien_link.h"
#include "sapien/sapien_scene.h"
#include <Eigen/Eigenvalues>
#include <filesystem>
#include <optional>
#include <spdlog/spdlog.h>
#include <tinyxml2.h>

namespace sapien {
namespace URDF {
using namespace tinyxml2;
using namespace physx;
namespace fs = std::filesystem;

// --------------------- DOM Parser ---------------------

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

// --------------------- End DOM Parser ---------------------

static PxTransform poseFromOrigin(const Origin &origin, float scale = 1.f) {
  PxQuat q = PxQuat(origin.rpy.z, {0, 0, 1}) * PxQuat(origin.rpy.y, {0, 1, 0}) *
             PxQuat(origin.rpy.x, {1, 0, 0});

  return PxTransform(origin.xyz * scale, q);
}

static std::string getAbsPath(const std::string &urdfPath, const std::string &filePath,
                              const std::string &packagePath = "") {

  std::string filename = filePath;
  if (filePath.starts_with("package://")) {
    filename = filePath.substr(10);
  }
  if (filename.starts_with("/")) {
    return filename;
  }
  if (filename.empty()) {
    throw std::runtime_error("Empty file path in URDF");
  }

  std::string basePath = "";
  if (filePath.starts_with("package://") && (!packagePath.empty())) {
    basePath = packagePath;
  } else if (!urdfPath.empty()) {
    basePath = fs::canonical(fs::path(urdfPath)).remove_filename().string();
  }
  if (basePath.empty()) {
    basePath = ".";
  }
  if (!basePath.ends_with('/')) {
    basePath += "/";
  }
  return basePath + filename;
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
  std::shared_ptr<LinkBuilder> linkBuilder = nullptr;
};

std::tuple<std::shared_ptr<ArticulationBuilder>, std::vector<SensorRecord>>
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
      spdlog::get("SAPIEN")->error("Failed to load URDF: parent of joint {} does not exist", name);
      return {nullptr, std::vector<SensorRecord>()};
    }
    if (linkName2treeNode.find(child) == linkName2treeNode.end()) {
      spdlog::get("SAPIEN")->error("Failed to load URDF: child of joint {} does not exist", name);
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
      float scale5 = scale3 * scale * scale;
      currentLinkBuilder->setMassAndInertia(currentInertial.mass->value * scale3, tInertia2Link,
                                            {scale5 * eigs.x, scale5 * eigs.y, scale5 * eigs.z});
    }

    // visual
    for (const auto &visual : current->link->visual_array) {
      PxVec3 color = {1, 1, 1};
      if (visual->material) {
        if (visual->material->color) {
          color = {visual->material->color->rgba.x, visual->material->color->rgba.y,
                   visual->material->color->rgba.z};
        } else {
          for (auto &m : robot->material_array) {
            if (m->name == visual->material->name) {
              color = {m->color->rgba.x, m->color->rgba.y, m->color->rgba.z};
              break;
            }
          }
        }
      }
      const PxTransform tVisual2Link = poseFromOrigin(*visual->origin, scale);
      switch (visual->geometry->type) {
      case Geometry::BOX:
        currentLinkBuilder->addBoxVisual(tVisual2Link, visual->geometry->size * scale / 2.f, color,
                                         visual->name);
        break;
      case Geometry::CYLINDER:
        spdlog::get("SAPIEN")->error("Cylinder visual is not supported. Replaced with a capsule");
        currentLinkBuilder->addCapsuleVisual(
            tVisual2Link * PxTransform({{0, 0, 0}, PxQuat(1.57079633, {0, 1, 0})}),
            visual->geometry->radius * scale,
            std::max(0.f,
                     visual->geometry->length * scale / 2.f - visual->geometry->radius * scale),
            color, visual->name);
        break;
      case Geometry::CAPSULE:
        currentLinkBuilder->addCapsuleVisual(
            tVisual2Link * PxTransform({{0, 0, 0}, PxQuat(1.57079633, {0, 1, 0})}),
            visual->geometry->radius * scale, visual->geometry->length * scale / 2.f, color,
            visual->name);
        break;
      case Geometry::SPHERE:
        currentLinkBuilder->addSphereVisual(tVisual2Link, visual->geometry->radius * scale, color,
                                            visual->name);
        break;
      case Geometry::MESH:
        currentLinkBuilder->addVisualFromFile(
            getAbsPath(urdfFilename, visual->geometry->filename, packageDir), tVisual2Link,
            visual->geometry->scale * scale, nullptr, visual->name);
        break;
      }
    }

    // collision
    for (uint32_t collision_idx = 0; collision_idx < current->link->collision_array.size();
         ++collision_idx) {
      const auto &collision = current->link->collision_array[collision_idx];

      std::shared_ptr<SPhysicalMaterial> material;
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
        if (collision->geometry->length / 2.0f - collision->geometry->radius < 1e-4) {
          spdlog::get("SAPIEN")->error(
              "Cylinder collision is not supported. Replaced with a sphere");
          currentLinkBuilder->addSphereShape(
              tCollision2Link * PxTransform({{0, 0, 0}, PxQuat(1.57079633, {0, 1, 0})}),
              collision->geometry->radius * scale, material, density, patchRadius, minPatchRadius);
        } else {
          spdlog::get("SAPIEN")->error(
              "Cylinder collision is not supported. Replaced with a capsule");
          currentLinkBuilder->addCapsuleShape(
              tCollision2Link * PxTransform({{0, 0, 0}, PxQuat(1.57079633, {0, 1, 0})}),
              collision->geometry->radius * scale,
              std::max(0.f, collision->geometry->length * scale / 2.0f -
                                collision->geometry->radius * scale),
              material, density, patchRadius, minPatchRadius);
        }
        if (collisionIsVisual) {
          currentLinkBuilder->addCapsuleVisual(
              tCollision2Link * PxTransform({{0, 0, 0}, PxQuat(1.57079633, {0, 1, 0})}),
              collision->geometry->radius * scale,
              std::max(0.f, collision->geometry->length * scale / 2.0f -
                                collision->geometry->radius * scale),
              PxVec3{1, 1, 1}, "");
        }

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
        if (multipleMeshesInOneFile) {
          currentLinkBuilder->addMultipleConvexShapesFromFile(
              getAbsPath(urdfFilename, collision->geometry->filename, packageDir), tCollision2Link,
              collision->geometry->scale * scale, material, density, patchRadius, minPatchRadius);
        } else {
          currentLinkBuilder->addConvexShapeFromFile(
              getAbsPath(urdfFilename, collision->geometry->filename, packageDir), tCollision2Link,
              collision->geometry->scale * scale, material, density, patchRadius, minPatchRadius);
        }
        if (collisionIsVisual) {
          currentLinkBuilder->addVisualFromFile(
              getAbsPath(urdfFilename, collision->geometry->filename, packageDir), tCollision2Link,
              collision->geometry->scale * scale, nullptr, "");
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

      PxArticulationJointType::Enum revoluteType =
          revoluteUnwrapped ? PxArticulationJointType::eREVOLUTE_UNWRAPPED
                            : PxArticulationJointType::eREVOLUTE;

      if (current->joint->type == "revolute") {
        currentLinkBuilder->setJointProperties(
            revoluteType, {{current->joint->limit->lower, current->joint->limit->upper}},
            tAxis2Parent, tAxis2Joint, friction, damping);
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

    // Reverse iterate over children and push into the stack
    for (auto c = current->children.rbegin(); c != current->children.rend(); c++) {
      stack.push_back(*c);
    }
  }

  if (srdf) {
    uint32_t groupCount = 0;
    for (auto &dc : srdf->disable_collisions_array) {
      if (dc->reason == std::string("Default")) {
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
        l1->linkBuilder->addCollisionGroup(0, 0, groupCount, 0);
        l2->linkBuilder->addCollisionGroup(0, 0, groupCount, 0);
      }
    }
    spdlog::get("SAPIEN")->info("SRDF: ignored {} pairs", groupCount);
  }

  std::vector<SensorRecord> sensorRecords;

  for (auto &gazebo : robot->gazebo_array) {
    for (auto &sensor : gazebo->sensor_array) {
      switch (sensor->type) {
      case Sensor::Type::RAY:
      case Sensor::Type::UNKNOWN:
        break;
      case Sensor::Type::CAMERA:
      case Sensor::Type::DEPTH:

        sensorRecords.push_back(SensorRecord{
            "camera", sensor->name, gazebo->reference, poseFromOrigin(*sensor->origin, scale),
            sensor->camera->width, sensor->camera->height, sensor->camera->fovx,
            sensor->camera->fovy, sensor->camera->near, sensor->camera->far});
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

      auto cam = mScene->addCamera(record.name, record.width, record.height, record.fovy,
                                   record.near, record.far);
      cam->setParent(*it);
      cam->setLocalPose(record.localPose);
      // mScene->addMountedCamera(record.name, *it, record.localPose, record.width, record.height,
      //                          record.fovx, record.fovy, record.near, record.far);
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

  auto [builder, records] = parseRobotDescription(urdfDoc, srdfDoc.get(), filename, true, config);
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

      auto cam = mScene->addCamera(record.name, record.width, record.height, record.fovy,
                                   record.near, record.far);
      cam->setParent(*it);
      cam->setLocalPose(record.localPose);
    }
  }

  return articulation;
}

std::shared_ptr<ArticulationBuilder>
URDFLoader::loadFileAsArticulationBuilder(const std::string &filename, URDFConfig const &config) {
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
  return std::move(
      std::get<0>(parseRobotDescription(urdfDoc, srdfDoc.get(), filename, true, config)));
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

      auto cam = mScene->addCamera(record.name, record.width, record.height, record.fovy,
                                   record.near, record.far);
      cam->setParent(*it);
      cam->setLocalPose(record.localPose);
    }
  }

  return articulation;
};

} // namespace URDF
} // namespace sapien
