#include "camera.h"
#include <optifuser.h>

namespace sapien {
namespace Renderer {

MountedCamera::MountedCamera(std::string const &name_, uint32_t width, uint32_t height,
                             float fovy_, Optifuser::Scene *scene)
    : mWidth(width), mHeight(height), mScene(scene) {
  name = name_;
  fovy = fovy_;
  position = glm::vec3(0, 0, 0);
  rotation = glm::quat(1, 0, 0, 0);

  // initialize render context
  mRenderContext = Optifuser::OffscreenRenderContext::Create(width, height);
  mRenderContext->renderer.setShadowShader("glsl_shader/shadow.vsh",
                                           "glsl_shader/shadow.fsh");
  mRenderContext->renderer.setGBufferShader("glsl_shader/gbuffer.vsh",
                                            "glsl_shader/gbuffer_segmentation.fsh");
  mRenderContext->renderer.setDeferredShader("glsl_shader/deferred.vsh",
                                             "glsl_shader/deferred.fsh");
  mRenderContext->renderer.setAxisShader("glsl_shader/axes.vsh", "glsl_shader/axes.fsh");
}

uint32_t MountedCamera::getWidth() const { return mWidth; }
uint32_t MountedCamera::getHeight() const { return mHeight; }
float MountedCamera::getFovy() const { return fovy; }

void MountedCamera::takePicture() { mRenderContext->renderer.renderScene(*mScene, *this); }

std::vector<float> MountedCamera::getColorRGBA() { return mRenderContext->renderer.getLighting(); }
std::vector<float> MountedCamera::getAlbedoRGBA() { return mRenderContext->renderer.getAlbedo(); }
std::vector<float> MountedCamera::getNormalRGBA() { return mRenderContext->renderer.getNormal(); }
std::vector<float> MountedCamera::getDepth() { return mRenderContext->renderer.getDepth(); }
std::vector<int> MountedCamera::getSegmentation() {
  return mRenderContext->renderer.getSegmentation();
}

SensorPose MountedCamera::getSensorPose() const {
  return {{position.x, position.y, position.z}, {rotation.w, rotation.x, rotation.y, rotation.z}};
}
void MountedCamera::setSensorPose(const SensorPose &pose) {
  {
    auto [x, y, z] = pose.positionXYZ;
    position = {x, y, z};
  }
  {
    auto [w, x, y, z] = pose.rotationWXYZ;
    // TODO: needs testing
    rotation = {w, x, y, z};
  }
}
const std::string &MountedCamera::getName() const { return name; }
glm::mat4 MountedCamera::getCameraMatrix() {
  float f = static_cast<float>(mHeight) / std::tan(fovy / 2) / 2;
  auto matrix = glm::mat3(1.0);
  matrix[0][0] = f;
  matrix[2][0] = static_cast<float>(mWidth) / 2;
  matrix[1][1] = f;
  matrix[2][1] = static_cast<float>(mHeight) / 2;
  return matrix;
}

} // namespace Renderer

} // namespace sapien
