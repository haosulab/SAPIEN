#pragma once
#include "material.h"
#include "sapien/array.h"
#include "sapien/math/bounding_box.h"
#include "sapien/math/pose.h"
#include "sapien/serialize.h"
#include "sapien_renderer_system.h"
#include <svulkan2/resource/model.h>

namespace sapien {
namespace sapien_renderer {
class SapienRenderBodyComponent;

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

  virtual ~RenderShape();

  template <class Archive> void save(Archive &archive) const {
    throw std::runtime_error("cereal workaround. should never be called.");
  }
  template <class Archive> void load(Archive &archive) {
    throw std::runtime_error("cereal workaround. should never be called.");
  }

  uint64_t getRenderId() const { return mRenderId; }
  void internalSetRenderId(uint64_t id) { mRenderId = id; }

  void setName(std::string const &name) { mName = name; }
  std::string getName() const { return mName; }

  vk::CullModeFlagBits getCulling() const;
  void setCulling(vk::CullModeFlagBits);

protected:
  uint64_t mRenderId{0};
  std::string mName;
  std::shared_ptr<SapienRenderEngine> mEngine;
  Pose mLocalPose{};
  std::shared_ptr<svulkan2::resource::SVModel> mModel;
  Vec3 mScale{1.f};
  vk::CullModeFlagBits mCulling{vk::CullModeFlagBits::eBack};

  SapienRenderBodyComponent *mParent{nullptr};
};

class RenderShapePlane : public RenderShape {
public:
  RenderShapePlane(Vec3 scale, std::shared_ptr<SapienRenderMaterial> material);
  Vec3 getScale() const;
  AABB getLocalAABB() override;

  template <class Archive> void save(Archive &ar) const { ar(mScale, mLocalPose, mMaterial); }
  template <class Archive>
  static void load_and_construct(Archive &ar, cereal::construct<RenderShapePlane> &construct) {
    Vec3 scale;
    std::shared_ptr<SapienRenderMaterial> material;
    Pose pose;
    ar(scale, pose, material);
    construct(scale, material);
    construct->setLocalPose(pose);
  }

private:
  std::shared_ptr<SapienRenderMaterial> mMaterial;
};

class RenderShapeBox : public RenderShape {
public:
  RenderShapeBox(Vec3 halfLengths, std::shared_ptr<SapienRenderMaterial> material);
  Vec3 getHalfLengths() const;
  AABB getLocalAABB() override;

  template <class Archive> void save(Archive &ar) const {
    ar(getHalfLengths(), mLocalPose, mMaterial);
  }
  template <class Archive>
  static void load_and_construct(Archive &ar, cereal::construct<RenderShapeBox> &construct) {
    Vec3 halfLengths;
    std::shared_ptr<SapienRenderMaterial> material;
    Pose pose;
    ar(halfLengths, pose, material);
    construct(halfLengths, material);
    construct->setLocalPose(pose);
  }

private:
  std::shared_ptr<SapienRenderMaterial> mMaterial;
};

class RenderShapeCapsule : public RenderShape {
public:
  RenderShapeCapsule(float radius, float halfLength,
                     std::shared_ptr<SapienRenderMaterial> material);
  float getRadius() const;
  float getHalfLength() const;
  AABB getLocalAABB() override;

  template <class Archive> void save(Archive &ar) const {
    ar(getRadius(), getHalfLength(), mLocalPose, mMaterial);
  }
  template <class Archive>
  static void load_and_construct(Archive &ar, cereal::construct<RenderShapeCapsule> &construct) {
    float radius, halfLength;
    std::shared_ptr<SapienRenderMaterial> material;
    Pose pose;
    ar(radius, halfLength, pose, material);
    construct(radius, halfLength, material);
    construct->setLocalPose(pose);
  }

private:
  float mRadius{};
  float mHalfLength{};
  std::shared_ptr<SapienRenderMaterial> mMaterial;
};

class RenderShapeCylinder : public RenderShape {
public:
  RenderShapeCylinder(float radius, float halfLength,
                      std::shared_ptr<SapienRenderMaterial> material);
  float getRadius() const;
  float getHalfLength() const;
  AABB getLocalAABB() override;

  template <class Archive> void save(Archive &ar) const {
    ar(getRadius(), getHalfLength(), mLocalPose, mMaterial);
  }
  template <class Archive>
  static void load_and_construct(Archive &ar, cereal::construct<RenderShapeCylinder> &construct) {
    float radius, halfLength;
    std::shared_ptr<SapienRenderMaterial> material;
    Pose pose;
    ar(radius, halfLength, pose, material);
    construct(radius, halfLength, material);
    construct->setLocalPose(pose);
  }

private:
  float mRadius{};
  float mHalfLength{};
  std::shared_ptr<SapienRenderMaterial> mMaterial;
};

class RenderShapeSphere : public RenderShape {
public:
  RenderShapeSphere(float radius, std::shared_ptr<SapienRenderMaterial> material);
  float getRadius() const;
  AABB getLocalAABB() override;

  template <class Archive> void save(Archive &ar) const { ar(getRadius(), mLocalPose, mMaterial); }
  template <class Archive>
  static void load_and_construct(Archive &ar, cereal::construct<RenderShapeSphere> &construct) {
    float radius;
    std::shared_ptr<SapienRenderMaterial> material;
    Pose pose;
    ar(radius, pose, material);
    construct(radius, material);
    construct->setLocalPose(pose);
  }

private:
  std::shared_ptr<SapienRenderMaterial> mMaterial;
};

class RenderShapeTriangleMeshPart {
public:
  RenderShapeTriangleMeshPart(std::shared_ptr<svulkan2::resource::SVShape> shape);

  Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> getVertices() const;
  Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> getTriangles() const;
  std::shared_ptr<SapienRenderMaterial> getMaterial() const;

  CudaArrayHandle getVertexBufferCudaArray() const;
  CudaArrayHandle getIndexBufferCudaArray() const;

  std::shared_ptr<svulkan2::resource::SVShape> getShape() const { return mShape; };

private:
  std::shared_ptr<svulkan2::resource::SVShape> mShape;
};

class RenderShapeTriangleMesh : public RenderShape {
public:
  RenderShapeTriangleMesh(std::string const &filename, Vec3 scale,
                          std::shared_ptr<SapienRenderMaterial> material = nullptr);
  RenderShapeTriangleMesh(
      Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &vertices,
      Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> const &triangles,
      Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &normals,
      std::shared_ptr<SapienRenderMaterial> material);

  RenderShapeTriangleMesh(std::vector<std::shared_ptr<RenderShapeTriangleMeshPart>> parts);

  template <class Archive> void save(Archive &ar) const {
    if (mFilename.empty()) {
      throw std::runtime_error("visual mesh not loaded from a file is not currently serializable");
    }
    ar(mFilename, mScale, mLocalPose, mMaterial);
  }
  template <class Archive>
  static void load_and_construct(Archive &ar,
                                 cereal::construct<RenderShapeTriangleMesh> &construct) {
    std::string filename;
    Vec3 scale;
    std::shared_ptr<SapienRenderMaterial> material;
    Pose pose;
    ar(filename, scale, pose, material);
    construct(filename, scale, material);
    construct->setLocalPose(pose);
  }
  AABB getLocalAABB() override;
  AABB computeGlobalAABBTight() override;

  std::string getFilename() const;
  Vec3 getScale() const;
  void setScale(Vec3 const &scale);

  std::vector<RenderShapeTriangleMeshPart> getParts();

  std::shared_ptr<SapienRenderMaterial> getMaterial();

private:
  std::string mFilename{};
  std::shared_ptr<SapienRenderMaterial> mMaterial;

  std::optional<AABB> mAABB;
};

} // namespace sapien_renderer
} // namespace sapien

CEREAL_REGISTER_TYPE(sapien::sapien_renderer::RenderShapePlane);
CEREAL_REGISTER_TYPE(sapien::sapien_renderer::RenderShapeBox);
CEREAL_REGISTER_TYPE(sapien::sapien_renderer::RenderShapeSphere);
CEREAL_REGISTER_TYPE(sapien::sapien_renderer::RenderShapeCapsule);
CEREAL_REGISTER_TYPE(sapien::sapien_renderer::RenderShapeCylinder);
CEREAL_REGISTER_TYPE(sapien::sapien_renderer::RenderShapeTriangleMesh);

CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::sapien_renderer::RenderShape,
                                     sapien::sapien_renderer::RenderShapePlane);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::sapien_renderer::RenderShape,
                                     sapien::sapien_renderer::RenderShapeBox);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::sapien_renderer::RenderShape,
                                     sapien::sapien_renderer::RenderShapeSphere);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::sapien_renderer::RenderShape,
                                     sapien::sapien_renderer::RenderShapeCapsule);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::sapien_renderer::RenderShape,
                                     sapien::sapien_renderer::RenderShapeCylinder);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::sapien_renderer::RenderShape,
                                     sapien::sapien_renderer::RenderShapeTriangleMesh);
