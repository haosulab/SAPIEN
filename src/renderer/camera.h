#pragma once
#include "render_interface.h"
#include <camera_spec.h>
#include <memory>
#include <optifuser.h>

namespace sapien {
namespace Renderer {
class MountedCamera : public Optifuser::CameraSpec, public ICamera {
public:
  uint32_t mWidth, mHeight;
  std::unique_ptr<Optifuser::OffscreenRenderContext> mRenderContext;
  Optifuser::Scene *mScene;

public:
  MountedCamera(std::string const &name, uint32_t width, uint32_t height, float fovy,
                Optifuser::Scene *scene);

  // ICamera
  virtual const std::string &getName() const override;
  virtual uint32_t getWidth() const override;
  virtual uint32_t getHeight() const override;
  virtual float getFovy() const override;
  virtual void takePicture() override;
  virtual std::vector<float> getColorRGBA() override;
  virtual std::vector<float> getAlbedoRGBA() override;
  virtual std::vector<float> getNormalRGBA() override;
  virtual std::vector<float> getDepth() override;
  virtual std::vector<int> getSegmentation() override;

  // ISensor
  virtual SensorPose getSensorPose() const override;
  virtual void setSensorPose(const SensorPose &pose) override;

  // Camera intrinsic
  glm::mat4 getCameraMatrix();
};
} // namespace Renderer

}
