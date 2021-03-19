#pragma once
#include <PxPhysicsAPI.h>
#include <array>
#include <foundation/PxTransform.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace sapien {
namespace Renderer {

class ISensor;
class ICamera;
class IPxrScene;
class IPxrRigidbody;
class IPxrRenderer;

struct RenderMeshGeometry {
  std::vector<float> vertices;
  std::vector<float> normals;
  std::vector<float> uvs;
  std::vector<float> tangents;
  std::vector<float> bitangents;
  std::vector<uint32_t> indices;
};

class IPxrMaterial {
public:
  virtual void setBaseColor(std::array<float, 4> color) = 0;
  virtual void setRoughness(float roughness) = 0;
  virtual void setSpecular(float specular) = 0;
  virtual void setMetallic(float metallic) = 0;
  virtual ~IPxrMaterial() = default;
};

struct RenderShape {
  std::string type = "invalid";
  physx::PxVec3 scale = {0, 0, 0};
  physx::PxTransform pose = physx::PxTransform(physx::PxIdentity);
  std::unique_ptr<RenderMeshGeometry> geometry = nullptr;
  std::shared_ptr<IPxrMaterial> material;
  uint32_t objId = 0;

  inline RenderShape(){};
  RenderShape(RenderShape const &) = delete;
  RenderShape(RenderShape &&) = default;
  RenderShape &operator=(RenderShape const &) = delete;
  RenderShape &operator=(RenderShape &&) = default;
  ~RenderShape() = default;
};

class PxrMaterial : public IPxrMaterial {
public:
  std::array<float, 4> base_color = {1, 1, 1, 1};
  float specular = 0.f;
  float roughness = 0.85f;
  float metallic = 0.f;
  std::string color_texture = "";
  std::string specular_texture = "";
  std::string normal_texture = "";

  inline void setBaseColor(std::array<float, 4> color) override { base_color = color; }
  inline void setRoughness(float value) override { roughness = value; }
  inline void setSpecular(float value) override { specular = value; }
  inline void setMetallic(float value) override { metallic = value; }
};

class ISensor {
public:
  virtual void setInitialPose(physx::PxTransform const &pose) = 0;
  virtual physx::PxTransform getPose() const = 0;
  virtual void setPose(physx::PxTransform const &pose) = 0;
  virtual IPxrScene *getScene() = 0;

  virtual ~ISensor() = default;
};

class ICamera : public ISensor {
public:
  virtual const std::string &getName() const = 0;
  virtual uint32_t getWidth() const = 0;
  virtual uint32_t getHeight() const = 0;
  virtual float getFovy() const = 0;
  virtual float getNear() const = 0;
  virtual float getFar() const = 0;

  virtual void takePicture() = 0;
  virtual std::vector<float> getColorRGBA() = 0;
  virtual std::vector<float> getAlbedoRGBA() = 0;
  virtual std::vector<float> getNormalRGBA() = 0;
  virtual std::vector<float> getDepth() = 0;
  virtual std::vector<int> getSegmentation() = 0;
  virtual std::vector<int> getObjSegmentation() = 0;
};

class IPxrRigidbody {
public:
  virtual void setName(std::string const &name) = 0;
  virtual std::string getName() const = 0;

  virtual void setUniqueId(uint32_t uniqueId) = 0;
  virtual uint32_t getUniqueId() const = 0;
  virtual void setSegmentationId(uint32_t segmentationId) = 0;
  virtual uint32_t getSegmentationId() const = 0;
  virtual void setSegmentationCustomData(std::vector<float> const &customData) = 0;
  virtual void setInitialPose(const physx::PxTransform &transform) = 0;
  virtual void update(const physx::PxTransform &transform) = 0;

  virtual void setVisibility(float visibility) = 0;
  virtual void setVisible(bool visible) = 0;
  virtual void setRenderMode(uint32_t mode) = 0;

  virtual void destroy() = 0;

  virtual std::vector<std::unique_ptr<RenderShape>> getRenderShapes() const = 0;

  virtual ~IPxrRigidbody() = default;
};

class IPxrScene {

public:
  virtual IPxrRigidbody *addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale) = 0;

  virtual IPxrRigidbody *addRigidbody(physx::PxGeometryType::Enum type, const physx::PxVec3 &scale,
                                      std::shared_ptr<IPxrMaterial> material) = 0;
  inline virtual IPxrRigidbody *addRigidbody(physx::PxGeometryType::Enum type,
                                             const physx::PxVec3 &scale,
                                             const physx::PxVec3 &color) {
    auto mat = std::make_shared<PxrMaterial>();
    mat->setBaseColor({color.x, color.y, color.z, 1.f});
    return addRigidbody(type, scale, mat);
  };

  virtual IPxrRigidbody *addRigidbody(std::vector<physx::PxVec3> const &vertices,
                                      std::vector<physx::PxVec3> const &normals,
                                      std::vector<uint32_t> const &indices,
                                      const physx::PxVec3 &scale,
                                      std::shared_ptr<IPxrMaterial> material) = 0;
  inline virtual IPxrRigidbody *addRigidbody(std::vector<physx::PxVec3> const &vertices,
                                             std::vector<physx::PxVec3> const &normals,
                                             std::vector<uint32_t> const &indices,
                                             const physx::PxVec3 &scale,
                                             const physx::PxVec3 &color) {
    auto mat = std::make_shared<PxrMaterial>();
    mat->setBaseColor({color.x, color.y, color.z, 1.f});
    return addRigidbody(vertices, normals, indices, scale, mat);
  }

  virtual void removeRigidbody(IPxrRigidbody *body) = 0;

  virtual ICamera *addCamera(std::string const &name, uint32_t width, uint32_t height, float fovx,
                             float fovy, float near, float far,
                             std::string const &shaderDir = "") = 0;
  virtual void removeCamera(ICamera *camera) = 0;

  virtual std::vector<ICamera *> getCameras() = 0;

  virtual void setShadowLight(std::array<float, 3> const &direction,
                              std::array<float, 3> const &color) = 0;
  virtual void addPointLight(std::array<float, 3> const &position,
                             std::array<float, 3> const &color) = 0;
  virtual void setAmbientLight(std::array<float, 3> const &color) = 0;
  virtual void addDirectionalLight(std::array<float, 3> const &direction,
                                   std::array<float, 3> const &color) = 0;

  /** call this function before every rendering time frame */
  inline virtual void updateRender(){};

  virtual void destroy() = 0;

  virtual ~IPxrScene() = default;
};

class IPxrRenderer {
public:
  virtual IPxrScene *createScene(std::string const &name = "") = 0;
  virtual void removeScene(IPxrScene *scene) = 0;
  virtual std::shared_ptr<IPxrMaterial> createMaterial() = 0;

  virtual ~IPxrRenderer() = default;
};

} // namespace Renderer
} // namespace sapien
