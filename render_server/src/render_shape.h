#pragma once
#include "sapien/math/pose.h"
#include <cstdint>

namespace sapien {
namespace render_server {

class ClientRenderShape {
public:
  void setLocalPose(Pose const &pose) { mLocalPose = pose; }
  Pose getLocalPose() const { return mLocalPose; }

  void internalSetRenderId(uint64_t id) { mRenderId = id; }
  void internalSetServerId(uint64_t id) { mServerId = id; }

  uint64_t getRenderId() const { return mRenderId; }
  uint64_t getServerId() const { return mServerId; }

  virtual ~ClientRenderShape(){};

protected:
  uint64_t mRenderId{0};
  uint64_t mServerId{0};
  Pose mLocalPose{};
};

class ClientRenderShapeTriangleMesh : public ClientRenderShape {
public:
  ClientRenderShapeTriangleMesh(std::string const &filename, Vec3 const &scale) {
    mFilename = filename;
    mScale = scale;
  }

  std::string getFilename() const { return mFilename; }
  Vec3 getScale() const { return mScale; }

private:
  std::string mFilename;
  Vec3 mScale{1.f};
};

} // namespace render_server
} // namespace sapien
