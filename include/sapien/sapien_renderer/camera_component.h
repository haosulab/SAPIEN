#pragma once
#include "../component.h"
#include "image.h"
#include "sapien/math/mat.h"
#include "sapien/math/pose.h"
#include <svulkan2/renderer/renderer_base.h>
#include <svulkan2/scene/camera.h>
#include <variant>
#include <map>

namespace sapien {
class Entity;
namespace sapien_renderer {
struct SapienRenderCameraInternal;
class SapienRenderTexture;

class SapienRenderCameraComponent : public Component {
public:
  SapienRenderCameraComponent(uint32_t width, uint32_t height, std::string const &shaderDir);

  virtual void onAddToScene(Scene &scene) override;
  virtual void onRemoveFromScene(Scene &scene) override;

  void setLocalPose(Pose const &);
  Pose getLocalPose() const;
  Pose getGlobalPose() const;

  // getters
  inline uint32_t getWidth() const { return mWidth; }
  inline uint32_t getHeight() const { return mHeight; }
  inline float getFocalLengthX() const { return mFx; }
  inline float getFocalLengthY() const { return mFy; }
  inline float getFovX() const { return std::atan(mWidth / 2.f / mFx) * 2.f; }
  inline float getFovY() const { return std::atan(mHeight / 2.f / mFy) * 2.f; }
  inline float getNear() const { return mNear; }
  inline float getFar() const { return mFar; }
  inline float getPrincipalPointX() const { return mCx; }
  inline float getPrincipalPointY() const { return mCy; }
  inline float getSkew() const { return mSkew; }

  Mat4 getModelMatrix() const;
  Mat4 getProjectionMatrix() const;
  Mat3 getIntrinsicMatrix() const;
  Mat34 getExtrinsicMatrix() const;

  // setters
  void setPerspectiveParameters(float near, float far, float fx, float fy, float cx, float cy,
                                float skew);
  void setFocalLengths(float fx, float fy);
  void setFovX(float fovx, bool computeY = true);
  void setFovY(float fovy, bool computeX = true);
  void setNear(float near);
  void setFar(float far);
  void setPrincipalPoint(float cx, float cy);
  void setSkew(float s);

  void takePicture();
  std::vector<std::string> getImageNames() const;
  SapienRenderImageCpu getImage(std::string const &name);
  SapienRenderImageCuda getImageCuda(std::string const &name);

  // TODO: make the following serializable
  void setProperty(std::string const &name, int property);
  void setProperty(std::string const &name, float property);
  void setTexture(std::string const &name, std::shared_ptr<SapienRenderTexture> texture);
  void setTextureArray(std::string const &name,
                       std::vector<std::shared_ptr<SapienRenderTexture>> texture);

  void internalUpdate();

  // GPU apis
  void gpuInit();
  CudaArrayHandle getCudaBuffer();
  void setGpuBatchedPoseIndex(int);
  int getGpuBatchedPoseIndex() const;
  void setAutoUpload(bool enable);
  svulkan2::core::Image &getInternalImage(std::string const &name);
  svulkan2::renderer::RendererBase &getInternalRenderer();
  svulkan2::scene::Camera &getInternalCamera();

  ~SapienRenderCameraComponent();
  SapienRenderCameraComponent(SapienRenderCameraComponent const &) = delete;
  SapienRenderCameraComponent &operator=(SapienRenderCameraComponent const &) = delete;
  SapienRenderCameraComponent(SapienRenderCameraComponent const &&) = delete;
  SapienRenderCameraComponent &operator=(SapienRenderCameraComponent const &&) = delete;

private:
  uint32_t mWidth{};
  uint32_t mHeight{};
  float mFx{};
  float mFy{};
  float mCx{};
  float mCy{};
  float mNear{};
  float mFar{};
  float mSkew{};
  std::string mShaderDir;

  std::map<std::string, std::variant<int, float>> mProperties;

  std::unique_ptr<SapienRenderCameraInternal> mCamera;
  Pose mLocalPose;

  // this is set to true when the camera is updated but take picture has not
  // been called to produce a warning for get picture
  bool mUpdatedWithoutTakingPicture{true};

  // this is set to true when GPU resources is available
  bool mGpuInitialized{false};
  int mGpuPoseIndex{-1};
};

} // namespace sapien_renderer
} // namespace sapien

