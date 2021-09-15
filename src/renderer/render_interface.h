#pragma once
#include <PxPhysicsAPI.h>
#include <array>
#include <foundation/PxTransform.h>
#include <functional>
#include <memory>
#include <spdlog/spdlog.h>
#include <string>
#include <vector>

namespace sapien {
namespace Renderer {

class ISensor;
class ICamera;
class IPxrScene;
class IPxrRigidbody;
class IPxrRenderer;

class ILight;
class IPointLight;
class ISpotLight;
class IDirectionalLight;

static inline void _warn_mat_func_not_supported(std::string_view func_name) {
  // gcc9 do not have source_location
  spdlog::get("SAPIEN")->warn("{} is not supported for the renderer", func_name);
}

static inline void _warn_texture_func_not_supported(std::string_view func_name) {
  // gcc9 do not have source_location
  spdlog::get("SAPIEN")->warn("{} is not supported for the renderer", func_name);
}

struct RenderMeshGeometry {
  std::vector<float> vertices;
  std::vector<float> normals;
  std::vector<float> uvs;
  std::vector<float> tangents;
  std::vector<float> bitangents;
  std::vector<uint32_t> indices;
};

class IPxrTexture {
public:
  enum FilterMode { eNEAREST, eLINEAR };
  enum AddressMode { eREPEAT, eBORDER, eEDGE, eMIRROR };
  enum Type { eBYTE, eINT, eHALF, eFLOAT, eOTHER };

  [[nodiscard]] virtual int getWidth() const = 0;
  [[nodiscard]] virtual int getHeight() const = 0;
  [[nodiscard]] virtual int getChannels() const = 0;

  [[nodiscard]] virtual int getMipmapLevels() const {
    _warn_texture_func_not_supported(__func__);
    return 0;
  };

  [[nodiscard]] virtual Type getType() const {
    _warn_texture_func_not_supported(__func__);
    return eOTHER;
  }
  [[nodiscard]] virtual AddressMode getAddressMode() const {
    _warn_texture_func_not_supported(__func__);
    return eREPEAT;
  };
  [[nodiscard]] virtual FilterMode getFilterMode() const {
    _warn_texture_func_not_supported(__func__);
    return eNEAREST;
  };
  [[nodiscard]] virtual std::string getFilename() const { return ""; };
  virtual ~IPxrTexture() = default;
};

class IPxrMaterial {
public:
  virtual void setBaseColor(std::array<float, 4> color) = 0;
  [[nodiscard]] virtual std::array<float, 4> getBaseColor() const = 0;
  virtual void setRoughness(float roughness) = 0;
  [[nodiscard]] virtual float getRoughness() const = 0;
  virtual void setSpecular(float specular) = 0;
  [[nodiscard]] virtual float getSpecular() const = 0;
  virtual void setMetallic(float metallic) = 0;
  [[nodiscard]] virtual float getMetallic() const = 0;

  virtual void setEmission(std::array<float, 4> color) { _warn_mat_func_not_supported(__func__); };
  [[nodiscard]] virtual std::array<float, 4> getEmission() const {
    _warn_mat_func_not_supported(__func__);
    return {};
  };
  virtual void setIOR(float ior) { _warn_mat_func_not_supported(__func__); };
  [[nodiscard]] virtual float getIOR() const {
    _warn_mat_func_not_supported(__func__);
    return 0.0;
  };
  virtual void setTransmission(float transmission) { _warn_mat_func_not_supported(__func__); };
  [[nodiscard]] virtual float getTransmission() const {
    _warn_mat_func_not_supported(__func__);
    return 0.0;
  };

  // texture functions
  virtual void setEmissionTexture(std::shared_ptr<IPxrTexture> texture) {
    _warn_mat_func_not_supported(__func__);
  }
  [[nodiscard]] virtual std::shared_ptr<IPxrTexture> getEmissionTexture() const {
    _warn_mat_func_not_supported(__func__);
    return nullptr;
  }
  virtual void setDiffuseTexture(std::shared_ptr<IPxrTexture> texture) {
    _warn_mat_func_not_supported(__func__);
  }
  [[nodiscard]] virtual std::shared_ptr<IPxrTexture> getDiffuseTexture() const {
    _warn_mat_func_not_supported(__func__);
    return nullptr;
  }
  virtual void setMetallicTexture(std::shared_ptr<IPxrTexture> texture) {
    _warn_mat_func_not_supported(__func__);
  }
  [[nodiscard]] virtual std::shared_ptr<IPxrTexture> getMetallicTexture() const {
    _warn_mat_func_not_supported(__func__);
    return nullptr;
  }
  virtual void setRoughnessTexture(std::shared_ptr<IPxrTexture> texture) {
    _warn_mat_func_not_supported(__func__);
  }
  [[nodiscard]] virtual std::shared_ptr<IPxrTexture> getRoughnessTexture() const {
    _warn_mat_func_not_supported(__func__);
    return nullptr;
  }
  virtual void setNormalTexture(std::shared_ptr<IPxrTexture> texture) {
    _warn_mat_func_not_supported(__func__);
  }
  [[nodiscard]] virtual std::shared_ptr<IPxrTexture> getNormalTexture() const {
    _warn_mat_func_not_supported(__func__);
    return nullptr;
  }
  virtual void setTransmissionTexture(std::shared_ptr<IPxrTexture> texture) {
    _warn_mat_func_not_supported(__func__);
  }
  [[nodiscard]] virtual std::shared_ptr<IPxrTexture> getTransmissionTexture() const {
    _warn_mat_func_not_supported(__func__);
    return nullptr;
  }

  // texture filename functions
  virtual void setEmissionTextureFromFilename(std::string_view path) {
    _warn_mat_func_not_supported(__func__);
  };
  [[nodiscard]] virtual std::string getEmissionTextureFilename() const {
    _warn_mat_func_not_supported(__func__);
    return "";
  };
  virtual void setDiffuseTextureFromFilename(std::string_view path) {
    _warn_mat_func_not_supported(__func__);
  };
  [[nodiscard]] virtual std::string getDiffuseTextureFilename() const {
    _warn_mat_func_not_supported(__func__);
    return "";
  };
  virtual void setMetallicTextureFromFilename(std::string_view path) {
    _warn_mat_func_not_supported(__func__);
  };
  [[nodiscard]] virtual std::string getMetallicTextureFilename() const {
    _warn_mat_func_not_supported(__func__);
    return "";
  };
  virtual void setRoughnessTextureFromFilename(std::string_view path) {
    _warn_mat_func_not_supported(__func__);
  };
  [[nodiscard]] virtual std::string getRoughnessTextureFilename() const {
    _warn_mat_func_not_supported(__func__);
    return "";
  };
  virtual void setNormalTextureFromFilename(std::string_view path) {
    _warn_mat_func_not_supported(__func__);
  };
  [[nodiscard]] virtual std::string getNormalTextureFilename() const {
    _warn_mat_func_not_supported(__func__);
    return "";
  };
  virtual void setTransmissionTextureFromFilename(std::string_view path) {
    _warn_mat_func_not_supported(__func__);
  };
  [[nodiscard]] virtual std::string getTransmissionTextureFilename() const {
    _warn_mat_func_not_supported(__func__);
    return "";
  };

  virtual ~IPxrMaterial() = default;
};

// struct RenderShape {
//   std::string type = "invalid";
//   physx::PxVec3 scale = {0, 0, 0};
//   physx::PxTransform pose = physx::PxTransform(physx::PxIdentity);
//   std::unique_ptr<RenderMeshGeometry> geometry = nullptr;
//   std::shared_ptr<IPxrMaterial> material;
//   uint32_t objId = 0;

//   inline RenderShape() = default;
//   RenderShape(RenderShape const &) = delete;
//   RenderShape(RenderShape &&) = default;
//   RenderShape &operator=(RenderShape const &) = delete;
//   RenderShape &operator=(RenderShape &&) = default;
//   ~RenderShape() = default;
// };

class IPxrRenderShape {
public:
  [[nodiscard]] virtual std::shared_ptr<RenderMeshGeometry> getGeometry() const { return {}; }
  [[nodiscard]] virtual std::shared_ptr<IPxrMaterial> getMaterial() const { return nullptr; }
  virtual ~IPxrRenderShape() = default;

  // TODO: these are deprecated
  // [[nodiscard]] virtual std::string getType() const { return "invalid"; }
  // [[nodiscard]] virtual physx::PxVec3 getScale() const { return {0, 0, 0}; }
  // [[nodiscard]] virtual physx::PxTransform getPose() const {
  //   return physx::PxTransform{physx::PxIdentity};
  // }
  // [[nodiscard]] virtual uint32_t getId() const { return 0; }
};

class PxrMaterial : public IPxrMaterial {
public:
  std::array<float, 4> base_color = {1, 1, 1, 1};
  float specular = 0.f;
  float roughness = 0.85f;
  float metallic = 0.f;
  std::string color_texture;
  std::string specular_texture;
  std::string normal_texture;

  inline void setBaseColor(std::array<float, 4> color) override { base_color = color; }
  [[nodiscard]] inline std::array<float, 4> getBaseColor() const override { return base_color; }
  inline void setRoughness(float value) override { roughness = value; }
  [[nodiscard]] inline float getRoughness() const override { return roughness; }
  inline void setSpecular(float value) override { specular = value; }
  [[nodiscard]] inline float getSpecular() const override { return specular; }
  inline void setMetallic(float value) override { metallic = value; }
  [[nodiscard]] inline float getMetallic() const override { return metallic; }
};

class ISensor {
public:
  virtual void setInitialPose(physx::PxTransform const &pose) = 0;
  [[nodiscard]] virtual physx::PxTransform getPose() const = 0;
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

class ILight {
public:
  virtual physx::PxTransform getPose() const = 0;
  virtual void setPose(physx::PxTransform const &transform) = 0;
  virtual physx::PxVec3 getColor() const = 0;
  virtual void setColor(physx::PxVec3 color) = 0;
  virtual bool getShadowEnabled() const = 0;
  virtual void setShadowEnabled(bool enabled) = 0;
  virtual ~ILight() = default;
};

class IPointLight : public ILight {
public:
  virtual physx::PxVec3 getPosition() const = 0;
  virtual void setPosition(physx::PxVec3 position) = 0;
  virtual void setShadowParameters(float near, float far) = 0;

  virtual float getShadowNear() const = 0;
  virtual float getShadowFar() const = 0;
};

class IDirectionalLight : public ILight {
public:
  virtual physx::PxVec3 getDirection() const = 0;
  virtual void setDirection(physx::PxVec3 direction) = 0;
  virtual void setShadowParameters(float halfSize, float near, float far) = 0;

  virtual float getShadowHalfSize() const = 0;
  virtual float getShadowNear() const = 0;
  virtual float getShadowFar() const = 0;
};

class ISpotLight : public ILight {
public:
  virtual physx::PxVec3 getPosition() const = 0;
  virtual void setPosition(physx::PxVec3 position) = 0;
  virtual physx::PxVec3 getDirection() const = 0;
  virtual void setDirection(physx::PxVec3 direction) = 0;
  virtual void setShadowParameters(float near, float far) = 0;

  virtual void setFov(float fov) = 0;
  virtual float getFov() const = 0;

  virtual float getShadowNear() const = 0;
  virtual float getShadowFar() const = 0;
};

class IActiveLight : public ILight {
public:
  virtual physx::PxVec3 getPosition() const = 0;
  virtual void setPosition(physx::PxVec3 position) = 0;
  virtual void setFov(float fov) = 0;
  virtual float getFov() const = 0;

  virtual void setTexture(std::string_view path) = 0;
  virtual std::string_view getTexture() = 0;
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

  // virtual std::vector<std::unique_ptr<RenderShape>> getRenderShapes() const = 0;

  // virtual std::vector<std::shared_ptr<IPxrRenderShape>> getRenderShapes() const = 0;

  virtual ~IPxrRigidbody() = default;

  // TODO: implement
  // return one of "box", "sphere", "capsule", "mesh"
  virtual physx::PxGeometryType::Enum getType() const {
    throw std::runtime_error("getType is not implemented");
  }

  // TODO: implement
  virtual physx::PxTransform getInitialPose() const {
    throw std::runtime_error("getInitialPose is not implemented");
  }

  // TODO: implement
  virtual std::vector<std::shared_ptr<IPxrRenderShape>> getRenderShapes() const {
    throw std::runtime_error("getRenderShapes is not implemented");
  }

  // TODO: implement
  // for capsule, it should return [half_length, radius, radius]
  virtual physx::PxVec3 getScale() const {
    throw std::runtime_error("getScale is not implemented");
  }
};

class IPxrScene {

public:
  virtual IPxrRigidbody *addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale) = 0;
  inline virtual IPxrRigidbody *
  addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale,
               std::shared_ptr<IPxrMaterial> material) { // add and replace material
    if (material)
      spdlog::get("SAPIEN")->warn("Add rigid body and substitute material is "
                                  "not supported on this rendering backend. Material in the mesh "
                                  "file will be used!");
    return addRigidbody(meshFile, scale);
  }

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

  virtual void setAmbientLight(std::array<float, 3> const &color) = 0;
  virtual std::array<float, 3> getAmbientLight() const = 0;

  virtual IPointLight *addPointLight(std::array<float, 3> const &position,
                                     std::array<float, 3> const &color, bool enableShadow,
                                     float shadowNear, float shadowFar) = 0;

  virtual IDirectionalLight *
  addDirectionalLight(std::array<float, 3> const &direction, std::array<float, 3> const &color,
                      bool enableShadow, std::array<float, 3> const &position, float shadowScale,
                      float shadowNear, float shadowFar) = 0;

  virtual ISpotLight *addSpotLight(std::array<float, 3> const &position,
                                   std::array<float, 3> const &direction, float fovInner,
                                   float fovOuter, std::array<float, 3> const &color,
                                   bool enableShadow, float shadowNear, float shadowFar) = 0;

  virtual IActiveLight *addActiveLight(physx::PxTransform const &pose,
                                       std::array<float, 3> const &color, float fov,
                                       std::string_view texPath) {
    spdlog::get("SAPIEN")->warn("Active light not supported!");
    return nullptr;
  };

  virtual void removeLight(ILight *light) = 0;

  /** call this function before every rendering time frame */
  inline virtual void updateRender(){};

  virtual void destroy() = 0;

  virtual ~IPxrScene() = default;
};

class IPxrRenderer {
public:
  virtual IPxrScene *createScene(std::string const &name) = 0;
  virtual void removeScene(IPxrScene *scene) = 0;
  virtual std::shared_ptr<IPxrMaterial> createMaterial() = 0;
  virtual std::shared_ptr<IPxrTexture> createTexture(std::string_view filename,
                                                     uint32_t mipLevels = 1,
                                                     IPxrTexture::FilterMode filterMode = {},
                                                     IPxrTexture::AddressMode addressMode = {}) {
    throw std::runtime_error("Texture creation is not supported.");
  }

  virtual ~IPxrRenderer() = default;
};

} // namespace Renderer
} // namespace sapien
