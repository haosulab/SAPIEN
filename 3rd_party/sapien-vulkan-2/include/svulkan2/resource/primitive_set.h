/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once
#include "svulkan2/common/layout.h"
#include "svulkan2/common/vk.h"
#include "svulkan2/core/as.h"
#include "svulkan2/core/buffer.h"
#include <mutex>

namespace svulkan2 {
namespace resource {

class SVPrimitiveSet {

public:
  SVPrimitiveSet(uint32_t capacity = 0);
  void setVertexAttribute(std::string const &name, std::vector<float> const &attrib);
  std::vector<float> const &getVertexAttribute(std::string const &name) const;
  core::Buffer &getVertexBuffer();

  uint32_t getVertexCapacity() const { return mVertexCapacity; }

  size_t getVertexSize();
  uint32_t getVertexCount() const { return mVertexCount; }

  void uploadToDevice();
  void removeFromDevice();

protected:
  uint32_t mVertexCapacity;

  std::unordered_map<std::string, std::vector<float>> mAttributes;

  bool mOnDevice{false};
  bool mDirty{true};

  uint32_t mVertexCount{};
  std::unique_ptr<core::Buffer> mVertexBuffer;

  std::mutex mUploadingMutex;
};

class SVLineSet : public SVPrimitiveSet {
public:
  using SVPrimitiveSet::SVPrimitiveSet;
};
class SVPointSet : public SVPrimitiveSet {
public:
  using SVPrimitiveSet::SVPrimitiveSet;
  void buildBLAS(bool update);
  core::BLAS *getBLAS() { return mBLAS.get(); }
  core::Buffer *getAabbBuffer() const { return mAabbBuffer.get(); }
  void recordUpdateBLAS(vk::CommandBuffer commandBuffer);

private:
  std::unique_ptr<core::BLAS> mBLAS;
  std::unique_ptr<core::Buffer> mAabbBuffer;
};

} // namespace resource
} // namespace svulkan2