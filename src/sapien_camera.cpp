#include "sapien_camera.h"
#include "sapien_scene.h"
#include <glm/gtx/quaternion.hpp>

namespace sapien {

SCamera::SCamera(SScene *scene, uint32_t width, uint32_t height)
    : SEntity(scene), mWidth(width), mHeight(height) {
  if (!scene || !scene->getRendererScene()) {
    throw std::runtime_error("failed to create camera: renderer is not attached");
  }
  mCamera = scene->getRendererScene()->addCamera(width, height, 1, 0.01, 100);
}

void SCamera::setLocalPose(PxTransform const &pose) {
  mLocalPose = pose;
  mPose = getParentPose() * mLocalPose;
}

void SCamera::setParent(SActorBase *actor, bool keepPose) {
  PxTransform p2w{PxIdentity};
  mParent = actor;
  if (actor) {
    p2w = actor->getPose();
  }
  if (keepPose) {
    mLocalPose = p2w.getInverse() * mPose;
  } else {
    mPose = p2w * mLocalPose;
  }
}

void SCamera::update() {
  mPose = getParentPose() * mLocalPose;
  if (mCamera) {
    static PxTransform gl2ros({0, 0, 0}, {-0.5, 0.5, 0.5, -0.5});
    mCamera->setPose(mPose * gl2ros);
  }
}

uint32_t SCamera::getWidth() const { return mWidth; }
uint32_t SCamera::getHeight() const { return mHeight; }

float SCamera::getFocalLengthX() const { return mCamera->getFocalX(); }
float SCamera::getFocalLengthY() const { return mCamera->getFocalY(); }
float SCamera::getFovX() const { return mCamera->getFovX(); }
float SCamera::getFovY() const { return mCamera->getFovY(); }
float SCamera::getNear() const { return mCamera->getNear(); }
float SCamera::getFar() const { return mCamera->getFar(); }
float SCamera::getPrincipalPointX() const { return mCamera->getPrincipalPointX(); }
float SCamera::getPrincipalPointY() const { return mCamera->getPrincipalPointY(); }
float SCamera::getSkew() const { return mCamera->getSkew(); }

void SCamera::setFocalLengths(float fx, float fy) {
  mCamera->setPerspectiveCameraParameters(mCamera->getNear(), mCamera->getFar(), fx, fy,
                                          mCamera->getPrincipalPointX(),
                                          mCamera->getPrincipalPointY(), mCamera->getSkew());
}

void SCamera::setFovX(float fovx, bool computeY) {
  float fx = getWidth() / 2.f / std::tan(fovx / 2);
  float fy = computeY ? fx : mCamera->getFocalY();
  setFocalLengths(fx, fy);
}

void SCamera::setFovY(float fovy, bool computeX) {
  float fy = getHeight() / 2.f / std::tan(fovy / 2);
  float fx = computeX ? fy : mCamera->getFocalX();
  setFocalLengths(fx, fy);
}

void SCamera::setNear(float near) {
  mCamera->setPerspectiveCameraParameters(near, mCamera->getFar(), mCamera->getFocalX(),
                                          mCamera->getFocalY(), mCamera->getPrincipalPointX(),
                                          mCamera->getPrincipalPointY(), mCamera->getSkew());
}

void SCamera::setFar(float far) {
  mCamera->setPerspectiveCameraParameters(mCamera->getNear(), far, mCamera->getFocalX(),
                                          mCamera->getFocalY(), mCamera->getPrincipalPointX(),
                                          mCamera->getPrincipalPointY(), mCamera->getSkew());
}

void SCamera::setPrincipalPoint(float cx, float cy) {
  mCamera->setPerspectiveCameraParameters(mCamera->getNear(), mCamera->getFar(),
                                          mCamera->getFocalX(), mCamera->getFocalY(), cx, cy,
                                          mCamera->getSkew());
}

void SCamera::setSkew(float s) {
  mCamera->setPerspectiveCameraParameters(
      mCamera->getNear(), mCamera->getFar(), mCamera->getFocalX(), mCamera->getFocalY(),
      mCamera->getPrincipalPointX(), mCamera->getPrincipalPointY(), s);
}

void SCamera::setPerspectiveParameters(float near, float far, float fx, float fy, float cx,
                                       float cy, float skew) {
  mCamera->setPerspectiveCameraParameters(near, far, fx, fy, cx, cy, skew);
}

SCamera::~SCamera() {
  if (mCamera) {
    mParentScene->getRendererScene()->removeCamera(mCamera);
  }
}

PxTransform SCamera::getParentPose() const {
  return mParent ? mParent->getPose() : PxTransform(PxIdentity);
}

// TODO: test
glm::mat4 SCamera::getProjectionMatrix() const {
  glm::mat4 mat(1.f);
  float fx = getFocalLengthX();
  float fy = getFocalLengthY();
  float width = getWidth();
  float height = getHeight();
  float far = getFar();
  float near = getNear();
  float cx = getPrincipalPointX();
  float cy = getPrincipalPointY();
  float skew = getSkew();

  mat[0][0] = (2.f * fx) / width;
  mat[1][1] = -(2.f * fy) / height;
  mat[2][2] = -far / (far - near);
  mat[3][2] = -far * near / (far - near);
  mat[2][3] = -1.f;
  mat[2][0] = -2.f * cx / width + 1;
  mat[2][1] = -2.f * cy / height + 1;
  mat[3][3] = 0.f;
  mat[1][0] = -2 * skew / width;
  return mat;
}

// TODO: test
glm::mat4 SCamera::getCameraMatrix() const {
  auto matrix = glm::mat4(1.f);
  matrix[0][0] = getFocalLengthX();
  matrix[1][1] = getFocalLengthY();
  matrix[2][0] = getPrincipalPointX();
  matrix[2][1] = getPrincipalPointY();
  matrix[1][0] = getSkew();
  return matrix;
}

// TODO: test
glm::mat4 SCamera::getModelMatrix() const {
  auto pose = mCamera->getPose();
  glm::quat q(pose.q.w, pose.q.x, pose.q.y, pose.q.z);
  glm::vec3 p(pose.p.x, pose.p.y, pose.p.z);
  return glm::translate(glm::mat4(1.0f), p) * glm::toMat4(q);
}

void SCamera::takePicture() { mCamera->takePicture(); }

} // namespace sapien
