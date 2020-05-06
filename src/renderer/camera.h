#pragma once
#include "config.h"
#include "render_interface.h"
#include <camera_spec.h>
#include <memory>
#include <optifuser.h>

namespace sapien {
namespace Renderer {
struct OptifuserConfig;

class OptifuserScene;

class OptifuserCamera : public ICamera {
  enum Mode { PERSPECTIVE, ORTHOGRAPHIC } mMode;

public:
  uint32_t mWidth, mHeight;
  std::unique_ptr<Optifuser::OffscreenRenderContext> mRenderContext;
  OptifuserScene *mScene;
  physx::PxTransform mInitialPose;

  std::unique_ptr<Optifuser::CameraSpec> mCameraSpec;

public:
  OptifuserCamera(std::string const &name, uint32_t width, uint32_t height, float fovy,
                  OptifuserScene *scene, std::string const &shaderDir,
                  OptifuserConfig const &config = {});

  // ICamera
  const std::string &getName() const override;
  uint32_t getWidth() const override;
  uint32_t getHeight() const override;
  float getFovy() const override;
  float getNear() const override;
  float getFar() const override;
  void takePicture() override;
  std::vector<float> getColorRGBA() override;
  std::vector<float> getAlbedoRGBA() override;
  std::vector<float> getNormalRGBA() override;
  std::vector<float> getDepth() override;
  std::vector<int> getSegmentation() override;
  std::vector<int> getObjSegmentation() override;

  IPxrScene *getScene() override;

  // ISensor
  physx::PxTransform getPose() const override;
  void setInitialPose(physx::PxTransform const &pose) override;
  void setPose(physx::PxTransform const &pose) override;

  void changeModeToOrthographic(float scaling);
  void changeModeToPerspective(float fovy);

  inline Mode getMode() { return mMode; }

#ifdef _USE_OPTIX
  std::vector<float> takeRaytracedPicture(uint32_t samplesPerPixel = 65536,
                                          uint32_t reflectionCount = 4);
#endif

  // Camera intrinsic
  glm::mat4 getCameraMatrix();
};

} // namespace Renderer
} // namespace sapien
