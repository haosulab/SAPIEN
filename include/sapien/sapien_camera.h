#pragma once
#include "awaitable.hpp"
#include "renderer/render_interface.h"
#include "sapien_actor_base.h"
#include "sapien_entity.h"
#include <glm/glm.hpp>

namespace sapien {

class SCamera : public SEntity {
public:
  SCamera(SScene *scene, uint32_t width, uint32_t height, float fovy = 1.5708, float near = 0.01f,
          float far = 100.f);

  /** call update to sync camera pose to renderer */
  void update();

  inline physx::PxTransform getPose() const override { return mPose; }
  inline physx::PxTransform getLocalPose(PxTransform const &pose) { return mLocalPose; }
  inline SActorBase *getParent() const { return mParent; };

  void setLocalPose(PxTransform const &pose);
  void setParent(SActorBase *actor, bool keepPose = false);

  uint32_t getWidth() const;
  uint32_t getHeight() const;

  float getFocalLengthX() const;
  float getFocalLengthY() const;
  float getFovX() const;
  float getFovY() const;
  float getNear() const;
  float getFar() const;
  float getPrincipalPointX() const;
  float getPrincipalPointY() const;
  float getSkew() const;

  void setFocalLengths(float fx, float fy);
  void setFovX(float fovx, bool computeY = true);
  void setFovY(float fovy, bool computeX = true);
  void setNear(float near);
  void setFar(float far);
  void setPrincipalPoint(float cx, float cy);
  void setSkew(float s);

  void setPerspectiveParameters(float near, float far, float fx, float fy, float cx, float cy,
                                float skew);

  glm::mat4 getProjectionMatrix() const;
  glm::mat4 getCameraMatrix() const;
  glm::mat4 getModelMatrix() const;

  glm::mat3 getIntrinsicMatrix() const;
  glm::mat4 getExtrinsicMatrix() const;

  void takePicture();

#ifdef SAPIEN_DLPACK
  std::shared_ptr<IAwaitable<std::vector<DLManagedTensor *>>>
  takePictureAndGetDLTensorsAsync(std::vector<std::string> const &names);
#endif

  Renderer::ICamera *getRendererCamera() const { return mCamera; }

  ~SCamera();

private:
  float mWidth{};
  float mHeight{};
  PxTransform mPose{PxIdentity};
  PxTransform mLocalPose{PxIdentity};
  SActorBase *mParent{};
  Renderer::ICamera *mCamera{};

  PxTransform getParentPose() const;
};

} // namespace sapien
