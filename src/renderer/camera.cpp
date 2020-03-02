#include "camera.h"
#include "optifuser_renderer.h"
#include <optifuser.h>

namespace sapien {
namespace Renderer {

OptifuserCamera::OptifuserCamera(std::string const &name_, uint32_t width, uint32_t height,
                             float fovy_, OptifuserScene *scene, std::string const &shaderDir)
    : mWidth(width), mHeight(height), mScene(scene) {
  name = name_;
  fovy = fovy_;
  aspect = static_cast<float>(width) / height;
  position = glm::vec3(0, 0, 0);
  rotation = glm::quat(1, 0, 0, 0);

  // initialize render context
  mRenderContext = Optifuser::OffscreenRenderContext::Create(width, height);
  mRenderContext->renderer.setShadowShader(shaderDir + "/shadow.vsh", shaderDir + "/shadow.fsh");
  mRenderContext->renderer.setGBufferShader(shaderDir + "/gbuffer.vsh",
                                            shaderDir + "/gbuffer_segmentation.fsh");
  mRenderContext->renderer.setDeferredShader(shaderDir + "/deferred.vsh",
                                             shaderDir + "/deferred.fsh");
  mRenderContext->renderer.setAxisShader(shaderDir + "/axes.vsh", shaderDir + "/axes.fsh");
}

IPxrScene *OptifuserCamera::getScene() { return mScene; };

uint32_t OptifuserCamera::getWidth() const { return mWidth; }
uint32_t OptifuserCamera::getHeight() const { return mHeight; }
float OptifuserCamera::getFovy() const { return fovy; }

void OptifuserCamera::takePicture() {
  mRenderContext->renderer.renderScene(*mScene->getScene(), *this);
}

std::vector<float> OptifuserCamera::getColorRGBA() { return mRenderContext->renderer.getLighting(); }
std::vector<float> OptifuserCamera::getAlbedoRGBA() { return mRenderContext->renderer.getAlbedo(); }
std::vector<float> OptifuserCamera::getNormalRGBA() { return mRenderContext->renderer.getNormal(); }
std::vector<float> OptifuserCamera::getDepth() { return mRenderContext->renderer.getDepth(); }
std::vector<int> OptifuserCamera::getSegmentation() {
  return mRenderContext->renderer.getSegmentation();
}
std::vector<int> OptifuserCamera::getObjSegmentation() {
  return mRenderContext->renderer.getSegmentation2();
}

physx::PxTransform OptifuserCamera::getPose() const {
  return physx::PxTransform({position.x, position.y, position.z},
                            physx::PxQuat(rotation.x, rotation.y, rotation.z, rotation.w));
}

void OptifuserCamera::setInitialPose(physx::PxTransform const &pose) {
  mInitialPose = pose;
  position = {pose.p.x, pose.p.y, pose.p.z};
  rotation = {pose.q.w, pose.q.x, pose.q.y, pose.q.z};
}

void OptifuserCamera::setPose(physx::PxTransform const &pose) {
  auto p = pose * mInitialPose;
  position = {p.p.x, p.p.y, p.p.z};
  rotation = {p.q.w, p.q.x, p.q.y, p.q.z};
}

const std::string &OptifuserCamera::getName() const { return name; }

glm::mat4 OptifuserCamera::getCameraMatrix() {
  float f = static_cast<float>(mHeight) / std::tan(fovy / 2) / 2;
  auto matrix = glm::mat3(1.0);
  matrix[0][0] = f;
  matrix[2][0] = static_cast<float>(mWidth) / 2;
  matrix[1][1] = f;
  matrix[2][1] = static_cast<float>(mHeight) / 2;
  return matrix;
}

#ifdef _USE_OPTIX

std::vector<float> OptifuserCamera::takeRaytracedPicture(uint32_t samplesPerPixel,
                                                       uint32_t reflectionCount) {
  auto pathTracer = new Optifuser::OptixRenderer(OptifuserRenderer::gPtxDir);
  pathTracer->init(mWidth, mHeight);
  pathTracer->numRays = reflectionCount;
  pathTracer->max_iterations = samplesPerPixel;
  pathTracer->invalidateCamera();

  for (uint32_t i = 0; i < samplesPerPixel; ++i) {
    pathTracer->renderScene(*mScene->getScene(), *this);
  }
  auto result = pathTracer->getResult();
  delete pathTracer;
  return result;
}
#endif

} // namespace Renderer
} // namespace sapien
