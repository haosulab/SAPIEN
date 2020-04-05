#include "camera.h"
#include "optifuser_renderer.h"
#include <optifuser.h>
#include <spdlog/spdlog.h>

namespace sapien {
namespace Renderer {

OptifuserCamera::OptifuserCamera(std::string const &name_, uint32_t width, uint32_t height,
                                 float fovy_, OptifuserScene *scene, std::string const &shaderDir)
    : mWidth(width), mHeight(height), mScene(scene) {

  auto cam = std::make_unique<Optifuser::PerspectiveCameraSpec>();
  cam->name = name_;
  cam->fovy = fovy_;
  cam->aspect = static_cast<float>(width) / height;
  cam->position = {0, 0, 0};
  cam->setRotation({1, 0, 0, 0});

  mCameraSpec = std::move(cam);

  // initialize render context
  mRenderContext = Optifuser::OffscreenRenderContext::Create(width, height);
  mRenderContext->renderer.setShadowShader(shaderDir + "/shadow.vsh", shaderDir + "/shadow.fsh");
  mRenderContext->renderer.setGBufferShader(shaderDir + "/gbuffer.vsh",
                                            shaderDir + "/gbuffer_segmentation.fsh");
  mRenderContext->renderer.setDeferredShader(shaderDir + "/deferred.vsh",
                                             shaderDir + "/deferred.fsh");
  // mRenderContext->renderer.setAxisShader(shaderDir + "/axes.vsh", shaderDir + "/axes.fsh");
  mRenderContext->renderer.setTransparencyShader(shaderDir + "/transparency.vsh",
                                                 shaderDir + "/transparency.fsh");
  mRenderContext->renderer.enableDisplayPass(false);
  mRenderContext->renderer.enableAxisPass(false);
}

IPxrScene *OptifuserCamera::getScene() { return mScene; };

uint32_t OptifuserCamera::getWidth() const { return mWidth; }
uint32_t OptifuserCamera::getHeight() const { return mHeight; }
float OptifuserCamera::getFovy() const {
  if (mMode == ORTHOGRAPHIC) {
    return 0;
  }
  return static_cast<Optifuser::PerspectiveCameraSpec *>(mCameraSpec.get())->fovy;
}

float OptifuserCamera::getNear() const { return mCameraSpec->near; }

float OptifuserCamera::getFar() const { return mCameraSpec->far; }

void OptifuserCamera::takePicture() {
  mRenderContext->renderer.renderScene(*mScene->getScene(), *this->mCameraSpec);
}

std::vector<float> OptifuserCamera::getColorRGBA() {
  return mRenderContext->renderer.getLighting();
}
std::vector<float> OptifuserCamera::getAlbedoRGBA() {
  return mRenderContext->renderer.getAlbedo();
}
std::vector<float> OptifuserCamera::getNormalRGBA() {
  return mRenderContext->renderer.getNormal();
}
std::vector<float> OptifuserCamera::getDepth() { return mRenderContext->renderer.getDepth(); }
std::vector<int> OptifuserCamera::getSegmentation() {
  return mRenderContext->renderer.getSegmentation();
}
std::vector<int> OptifuserCamera::getObjSegmentation() {
  return mRenderContext->renderer.getSegmentation2();
}

physx::PxTransform OptifuserCamera::getPose() const {
  auto p = mCameraSpec->position;
  auto q = mCameraSpec->getRotation();
  return physx::PxTransform({p.x, p.y, p.z}, physx::PxQuat(q.x, q.y, q.z, q.w));
}

void OptifuserCamera::setInitialPose(physx::PxTransform const &pose) {
  mInitialPose = pose;
  mCameraSpec->position = {pose.p.x, pose.p.y, pose.p.z};
  mCameraSpec->setRotation({pose.q.w, pose.q.x, pose.q.y, pose.q.z});
}

void OptifuserCamera::setPose(physx::PxTransform const &pose) {
  auto p = pose * mInitialPose;
  mCameraSpec->position = {p.p.x, p.p.y, p.p.z};
  mCameraSpec->setRotation({p.q.w, p.q.x, p.q.y, p.q.z});
}

const std::string &OptifuserCamera::getName() const { return mCameraSpec->name; }

glm::mat4 OptifuserCamera::getCameraMatrix() {
  if (mMode == ORTHOGRAPHIC) {
    spdlog::error("Orthographic Camera Matrix is not implemented yet");
    return glm::mat4(1);
  } else {
    float fovy = static_cast<Optifuser::PerspectiveCameraSpec *>(mCameraSpec.get())->fovy;
    float f = static_cast<float>(mHeight) / std::tan(fovy / 2) / 2;
    auto matrix = glm::mat3(1.0);
    matrix[0][0] = f;
    matrix[2][0] = static_cast<float>(mWidth) / 2;
    matrix[1][1] = f;
    matrix[2][1] = static_cast<float>(mHeight) / 2;
    return matrix;
  }
}

void OptifuserCamera::changeModeToOrthographic(float scaling) {
  auto cam = std::make_unique<Optifuser::OrthographicCameraSpec>();
  cam->name = mCameraSpec->name;
  cam->position = mCameraSpec->position;
  cam->setRotation(mCameraSpec->getRotation());
  cam->aspect = mCameraSpec->aspect;
  cam->scaling = scaling;
  mCameraSpec = std::move(cam);
  mMode = ORTHOGRAPHIC;
}

void OptifuserCamera::changeModeToPerspective(float fovy) {
  auto cam = std::make_unique<Optifuser::PerspectiveCameraSpec>();
  cam->name = mCameraSpec->name;
  cam->position = mCameraSpec->position;
  cam->setRotation(mCameraSpec->getRotation());
  cam->aspect = mCameraSpec->aspect;
  cam->fovy = fovy;
  mCameraSpec = std::move(cam);
  mMode = PERSPECTIVE;
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
    pathTracer->renderScene(*mScene->getScene(), *mCameraSpec);
  }
  auto result = pathTracer->getResult();
  delete pathTracer;
  return result;
}
#endif

} // namespace Renderer
} // namespace sapien
