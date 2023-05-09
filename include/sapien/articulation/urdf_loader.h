#pragma once
#include <PxPhysicsAPI.h>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace tinyxml2 {
class XMLDocument;
}

namespace sapien {
class SScene;
class SArticulation;
class SKArticulation;
class SArticulationBase;
class ArticulationBuilder;
class SPhysicalMaterial;

namespace URDF {

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

  /* load mesh files as separate convex components */
  bool multipleMeshesInOneFile = false;

  /* Load the articulation at a different scale.
   * It will scale mass and inertia accordingly
   */
  float scale = 1.f;

  /* collision will be rendered along with visual */
  bool collisionIsVisual = false;

  /* directory for package:// */
  std::string packageDir = "";

  /* build revolute as unwrapped */
  bool revoluteUnwrapped = false;

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
  parseRobotDescription(tinyxml2::XMLDocument const &urdfDoc, tinyxml2::XMLDocument const *srdfDoc,
                        const std::string &urdfFilename, bool isKinematic,
                        URDFConfig const &config);
};

} // namespace URDF
} // namespace sapien
