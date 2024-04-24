#pragma once
#include "material.h"
#include "sapien/array.h"
#include "sapien/math/bounding_box.h"
#include "sapien/math/pose.h"
#include "sapien_renderer_system.h"
#include <svulkan2/resource/model.h>

namespace sapien {
namespace sapien_renderer {
class SapienRenderBodyComponent;

class RenderShapeTriangleMeshPart {
public:
  RenderShapeTriangleMeshPart(std::shared_ptr<svulkan2::resource::SVShape> shape);

  Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> getVertices() const;
  Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> getTriangles() const;
  std::shared_ptr<SapienRenderMaterial> getMaterial() const;

  void setUV(Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> const &);
  void setNormal(Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &);

  Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> getUV() const;
  Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> getNormal() const;

  CudaArrayHandle getVertexBufferCudaArray() const;
  CudaArrayHandle getIndexBufferCudaArray() const;

  std::shared_ptr<svulkan2::resource::SVShape> getShape() const { return mShape; };

private:
  std::shared_ptr<svulkan2::resource::SVShape> mShape;
};

class RenderShape {
public:
  RenderShape();

  void setLocalPose(Pose const &pose);
  Pose getLocalPose() const;

  svulkan2::scene::Transform getLocalTransform() const;

  inline std::shared_ptr<svulkan2::resource::SVModel> const &getModel() const { return mModel; }

  virtual AABB getLocalAABB() = 0;
  virtual AABB getGlobalAABBFast();
  virtual AABB computeGlobalAABBTight();

  RenderShape(RenderShape const &) = delete;
  RenderShape &operator=(RenderShape const &) = delete;
  RenderShape(RenderShape const &&) = delete;
  RenderShape &operator=(RenderShape const &&) = delete;

  void internalSetParent(SapienRenderBodyComponent *parent) { mParent = parent; }
  std::shared_ptr<SapienRenderBodyComponent> getParent() const;

  void internalSetRenderObject(svulkan2::scene::Object *object) { mObject = object; }

  virtual std::shared_ptr<SapienRenderMaterial> getMaterial() const = 0;
  virtual ~RenderShape();

  uint64_t getRenderId() const { return mRenderId; }
  void internalSetRenderId(uint64_t id) { mRenderId = id; }

  void setName(std::string const &name) { mName = name; }
  std::string getName() const { return mName; }

  vk::FrontFace getFrontFace() const;
  void setFrontFace(vk::FrontFace);

  virtual std::vector<std::shared_ptr<RenderShapeTriangleMeshPart>> getParts() {
    return {std::make_shared<RenderShapeTriangleMeshPart>(mModel->getShapes().at(0))};
  }

  virtual std::shared_ptr<RenderShape> clone() const = 0;

  /** batched rendering only, sets the index to look up for GPU pose */
  void setGpuBatchedPoseIndex(int);
  int getGpuBatchedPoseIndex() const;

  /** Get the index of this shape in the transform array of the render scene
   *  The index will change when any object is removed from the scene. */
  int getInternalGpuTransformIndex();

  /** Get the internal mesh scale
   *  For mesh, this is the same as getScale
   *  For other shapes, it is implementation dependent
   *  This should be used with the API that directly updates poses on the GPU
   * */
  Vec3 getGpuScale();

protected:
  uint64_t mRenderId{0};
  std::string mName;
  std::shared_ptr<SapienRenderEngine> mEngine;
  Pose mLocalPose{};
  std::shared_ptr<svulkan2::resource::SVModel> mModel;

  // this corresponds to the actual scale in the renderer
  Vec3 mScale{1.f};

  vk::FrontFace mFrontFace{vk::FrontFace::eCounterClockwise};

  SapienRenderBodyComponent *mParent{nullptr};
  svulkan2::scene::Object *mObject{nullptr};

  int mBatchedPoseIndex{-1};
};

class RenderShapePrimitive : public RenderShape {
public:
  Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> getVertices() const;
  Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> getTriangles() const;
  Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> getNormal() const;
  Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> getUV() const;
};

class RenderShapePlane : public RenderShapePrimitive {
public:
  RenderShapePlane(Vec3 scale, std::shared_ptr<SapienRenderMaterial> material);
  Vec3 getScale() const;
  AABB getLocalAABB() override;

  std::shared_ptr<SapienRenderMaterial> getMaterial() const override { return mMaterial; }

  std::shared_ptr<RenderShape> clone() const override;

private:
  std::shared_ptr<SapienRenderMaterial> mMaterial;
};

class RenderShapeBox : public RenderShapePrimitive {
public:
  RenderShapeBox(Vec3 halfLengths, std::shared_ptr<SapienRenderMaterial> material);
  Vec3 getHalfLengths() const;
  AABB getLocalAABB() override;

  std::shared_ptr<SapienRenderMaterial> getMaterial() const override { return mMaterial; }

  std::shared_ptr<RenderShape> clone() const override;

private:
  std::shared_ptr<SapienRenderMaterial> mMaterial;
};

class RenderShapeCapsule : public RenderShapePrimitive {
public:
  RenderShapeCapsule(float radius, float halfLength,
                     std::shared_ptr<SapienRenderMaterial> material);
  float getRadius() const;
  float getHalfLength() const;
  AABB getLocalAABB() override;

  std::shared_ptr<SapienRenderMaterial> getMaterial() const override { return mMaterial; }

  std::shared_ptr<RenderShape> clone() const override;

private:
  float mRadius{};
  float mHalfLength{};
  std::shared_ptr<SapienRenderMaterial> mMaterial;
};

class RenderShapeCylinder : public RenderShapePrimitive {
public:
  RenderShapeCylinder(float radius, float halfLength,
                      std::shared_ptr<SapienRenderMaterial> material);
  float getRadius() const;
  float getHalfLength() const;
  AABB getLocalAABB() override;

  std::shared_ptr<SapienRenderMaterial> getMaterial() const override { return mMaterial; }

  std::shared_ptr<RenderShape> clone() const override;

private:
  float mRadius{};
  float mHalfLength{};
  std::shared_ptr<SapienRenderMaterial> mMaterial;
};

class RenderShapeSphere : public RenderShapePrimitive {
public:
  RenderShapeSphere(float radius, std::shared_ptr<SapienRenderMaterial> material);
  float getRadius() const;
  AABB getLocalAABB() override;
  std::shared_ptr<SapienRenderMaterial> getMaterial() const override { return mMaterial; }

  std::shared_ptr<RenderShape> clone() const override;

private:
  std::shared_ptr<SapienRenderMaterial> mMaterial;
};

class RenderShapeTriangleMesh : public RenderShape {
public:
  RenderShapeTriangleMesh(std::string const &filename, Vec3 scale,
                          std::shared_ptr<SapienRenderMaterial> material = nullptr);
  RenderShapeTriangleMesh(
      Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &vertices,
      Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> const &triangles,
      Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &normals,
      Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> const &uvs,
      std::shared_ptr<SapienRenderMaterial> material);

  RenderShapeTriangleMesh(std::vector<std::shared_ptr<RenderShapeTriangleMeshPart>> parts);

  AABB getLocalAABB() override;
  AABB computeGlobalAABBTight() override;

  std::string getFilename() const;
  Vec3 getScale() const;
  void setScale(Vec3 const &scale);

  std::vector<std::shared_ptr<RenderShapeTriangleMeshPart>> getParts() override;

  std::shared_ptr<SapienRenderMaterial> getMaterial() const override;

  std::shared_ptr<RenderShape> clone() const override;

private:
  std::string mFilename{};
  std::shared_ptr<SapienRenderMaterial> mMaterial;

  std::optional<AABB> mAABB;
};

} // namespace sapien_renderer
} // namespace sapien
