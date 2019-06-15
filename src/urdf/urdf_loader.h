#include <PxPhysicsAPI.h>
#include <string>
#include <vector>

namespace URDFUtil {

struct Origin {
  physx::PxVec3 xyz = {0, 0, 0}; // x, y, z offset
  physx::PxVec3 rpy = {0, 0, 0}; // roll, pitch, yaw in radian
};

struct Inertia {
  physx::PxReal ixx = 1;
  physx::PxReal ixy = 0;
  physx::PxReal ixz = 0;
  physx::PxReal iyy = 1;
  physx::PxReal iyz = 0;
  physx::PxReal izz = 1;
};

struct Inertial {
  Origin origin;
  physx::PxReal mass;
  Inertia inertia;
};

struct Geometry {
  enum Type { Box, Cylinder, Sphere, Mesh } type = Box;
  physx::PxVec3 size = {1.f, 1.f, 1.f};
  physx::PxReal radius = 1.f;
  physx::PxReal length = 1.f;
  std::string filename;
  physx::PxVec3 scale = {1.f, 1.f, 1.f};
};

struct Material {
  std::string name;
  physx::PxVec4 color;
  std::string texture;
};

struct Visual {
  std::string name;
  Origin origin;
  Geometry geometry;
  Material material;
};

struct Collision {
  std::string name;
  Origin origin;
  Geometry geometry;
};

struct Link {
  std::string name;
  Inertial inertial;
  std::vector<Visual> visuals;
  std::vector<Collision> collisions;
};

struct Dynamics {
  physx::PxReal damping;
  physx::PxReal friction;
};

struct Limit {
  physx::PxReal lower;
  physx::PxReal upper;
  physx::PxReal effort;
  physx::PxReal velocity;
};

struct Joint {
  std::string name;
  enum Type { Revolute, Continuous, Prismatic, Fixed, Floating, Plannar } type;
  Origin origin;
  std::string parent;
  std::string child;
  physx::PxVec3 axis = {1, 0, 0};
  Dynamics dynamics;
  Limit limit;
};

struct Robot {
  std::string name;
  std::vector<Link> links;
  std::vector<Joint> joints;
};

class URDFLoader {
  Robot robot;
  physx::PxArticulation *articulation;

  physx::PxPhysics *physicsSDK;

public:
  URDFLoader(physx::PxPhysics *physics) : physicsSDK(physics) {}
  void load(const std::string &filename);
};

} // namespace URDFUtil
