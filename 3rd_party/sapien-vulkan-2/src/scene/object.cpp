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
#include "svulkan2/scene/object.h"
#include "svulkan2/scene/scene.h"
#include "svulkan2/common/profiler.h"

namespace svulkan2 {
namespace scene {

Object::Object(std::shared_ptr<resource::SVModel> model, std::string const &name)
    : Node(name), mModel(model) {}

void Object::uploadToDevice(core::Buffer &objectBuffer, uint32_t offset,
                            StructDataLayout const &objectLayout) {
  // FIXME: this function is deprecated, remove it
  SVULKAN2_PROFILE_BLOCK("allocate");
  std::vector<char> buffer(objectLayout.size);
  SVULKAN2_PROFILE_BLOCK_END;

  SVULKAN2_PROFILE_BLOCK("copy matrix");
  std::memcpy(buffer.data() + objectLayout.elements.at("segmentation").offset, &mSegmentation[0],
              16);

  SVULKAN2_PROFILE_BLOCK_END;

  SVULKAN2_PROFILE_BLOCK("check other data");
  for (auto &[name, value] : mCustomData) {
    if (objectLayout.elements.find(name) != objectLayout.elements.end()) {
      auto &elem = objectLayout.elements.at(name);
      if (elem.dtype != value.dtype) {
        throw std::runtime_error("Upload object failed: object attribute \"" + name +
                                 "\" does not match declared type.");
        std::memcpy(buffer.data() + elem.offset, &value.floatValue, elem.size);
      }
    }
  }
  if (objectLayout.elements.find("transparency") != objectLayout.elements.end()) {
    auto &elem = objectLayout.elements.at("transparency");
    if (elem.dtype != DataType::FLOAT()) {
      throw std::runtime_error("Upload object failed: object attribute "
                               "\"transparency\" must be a float");
    }
    std::memcpy(buffer.data() + elem.offset, &mTransparency, 4);
  }

  if (objectLayout.elements.find("shadeFlat") != objectLayout.elements.end()) {
    auto &elem = objectLayout.elements.at("shadeFlat");
    if (elem.dtype != DataType::INT()) {
      throw std::runtime_error("Upload object failed: object attribute "
                               "\"shadeFlat\" must be an int");
    }
    int shadeFlat = mShadeFlat;
    std::memcpy(buffer.data() + elem.offset, &shadeFlat, 4);
  }
  SVULKAN2_PROFILE_BLOCK_END;
  objectBuffer.upload(buffer.data(), objectLayout.size, offset);
}

void Object::setSegmentation(glm::uvec4 const &segmentation) {
  mSegmentation = segmentation;
  mScene->updateRenderVersion();
}

void Object::setCustomDataFloat(std::string const &name, float x) {
  mCustomData[name] = CustomData{.dtype = DataType::FLOAT(), .floatValue = x};
  mScene->updateRenderVersion();
}
void Object::setCustomDataFloat2(std::string const &name, glm::vec2 x) {
  mCustomData[name] = CustomData{.dtype = DataType::FLOAT2(), .float2Value = x};
  mScene->updateRenderVersion();
}
void Object::setCustomDataFloat3(std::string const &name, glm::vec3 x) {
  mCustomData[name] = CustomData{.dtype = DataType::FLOAT3(), .float3Value = x};
  mScene->updateRenderVersion();
}
void Object::setCustomDataFloat4(std::string const &name, glm::vec4 x) {
  mCustomData[name] = CustomData{.dtype = DataType::FLOAT4(), .float4Value = x};
  mScene->updateRenderVersion();
}
void Object::setCustomDataFloat44(std::string const &name, glm::mat4 x) {
  mCustomData[name] = CustomData{.dtype = DataType::FLOAT44(), .float44Value = x};
  mScene->updateRenderVersion();
}
void Object::setCustomDataInt(std::string const &name, int x) {
  mCustomData[name] = CustomData{.dtype = DataType::INT(), .intValue = x};
  mScene->updateRenderVersion();
}
void Object::setCustomDataInt2(std::string const &name, glm::ivec2 x) {
  mCustomData[name] = CustomData{.dtype = DataType::INT2(), .int2Value = x};
  mScene->updateRenderVersion();
}
void Object::setCustomDataInt3(std::string const &name, glm::ivec3 x) {
  mCustomData[name] = CustomData{.dtype = DataType::INT3(), .int3Value = x};
  mScene->updateRenderVersion();
}
void Object::setCustomDataInt4(std::string const &name, glm::ivec4 x) {
  mCustomData[name] = CustomData{.dtype = DataType::INT4(), .int4Value = x};
  mScene->updateRenderVersion();
}

void Object::setCustomTexture(std::string const &name,
                              std::shared_ptr<resource::SVTexture> texture) {
  mCustomTexture[name] = texture;
}
std::shared_ptr<resource::SVTexture> const
Object::getCustomTexture(std::string const &name) const {
  auto it = mCustomTexture.find(name);
  if (it != mCustomTexture.end()) {
    return it->second;
  }
  return nullptr;
}

void Object::setCustomTextureArray(std::string const &name,
                                   std::vector<std::shared_ptr<resource::SVTexture>> textures) {
  mCustomTextureArray[name] = textures;
}
std::vector<std::shared_ptr<resource::SVTexture>>
Object::getCustomTextureArray(std::string const &name) const {
  auto it = mCustomTextureArray.find(name);
  if (it != mCustomTextureArray.end()) {
    return it->second;
  }
  return {};
}

void Object::setTransparency(float transparency) {
  if (transparency >= 1.f && mTransparency < 1.f) {
    mScene->updateVersion();
  } else if (transparency < 1.f && mTransparency >= 1.f) {
    mScene->updateVersion();
  } else if (transparency <= 0.f && mTransparency > 0.f) {
    mScene->updateVersion();
  } else if (transparency > 0.f && mTransparency <= 0.f) {
    mScene->updateVersion();
  }
  mTransparency = transparency;
}

void Object::setCastShadow(bool castShadow) {
  mCastShadow = castShadow;
  mScene->updateVersion();
}

LineObject::LineObject(std::shared_ptr<resource::SVLineSet> lineSet, std::string const &name)
    : Node(name), mLineSet(lineSet) {}

void LineObject::uploadToDevice(core::Buffer &objectBuffer, uint32_t offset,
                                StructDataLayout const &objectLayout) {
  std::vector<char> buffer(objectLayout.size);
  std::memcpy(buffer.data() + objectLayout.elements.at("segmentation").offset, &mSegmentation[0],
              16);
  if (objectLayout.elements.find("transparency") != objectLayout.elements.end()) {
    auto &elem = objectLayout.elements.at("transparency");
    if (elem.dtype != DataType::FLOAT()) {
      throw std::runtime_error("Upload object failed: object attribute "
                               "\"transparency\" must be a float");
    }
    std::memcpy(buffer.data() + elem.offset, &mTransparency, 4);
  }
  // objectBuffer.upload(buffer);

  objectBuffer.upload(buffer.data(), objectLayout.size, offset);
}

void LineObject::setSegmentation(glm::uvec4 const &segmentation) { mSegmentation = segmentation; }

void LineObject::setTransparency(float transparency) { mTransparency = transparency; }

PointObject::PointObject(std::shared_ptr<resource::SVPointSet> pointSet, std::string const &name)
    : Node(name), mPointSet(pointSet), mVertexCount(pointSet->getVertexCount()) {}

void PointObject::uploadToDevice(core::Buffer &objectBuffer, uint32_t offset,
                                 StructDataLayout const &objectLayout) {
  std::vector<char> buffer(objectLayout.size);
  std::memcpy(buffer.data() + objectLayout.elements.at("segmentation").offset, &mSegmentation[0],
              16);
  if (objectLayout.elements.find("transparency") != objectLayout.elements.end()) {
    auto &elem = objectLayout.elements.at("transparency");
    if (elem.dtype != DataType::FLOAT()) {
      throw std::runtime_error("Upload object failed: object attribute "
                               "\"transparency\" must be a float");
    }
    std::memcpy(buffer.data() + elem.offset, &mTransparency, 4);
  }
  objectBuffer.upload(buffer.data(), objectLayout.size, offset);
}

void PointObject::setSegmentation(glm::uvec4 const &segmentation) { mSegmentation = segmentation; }

void PointObject::setTransparency(float transparency) { mTransparency = transparency; }

void PointObject::setVertexCount(uint32_t count) {
  mScene->updateVersion();
  mVertexCount = count;
}

void Object::setInternalGpuIndex(int index) { mGpuIndex = index; }
int Object::getInternalGpuIndex() const { return mGpuIndex; }
void LineObject::setInternalGpuIndex(int index) { mGpuIndex = index; }
int LineObject::getInternalGpuIndex() const { return mGpuIndex; }
void PointObject::setInternalGpuIndex(int index) { mGpuIndex = index; }
int PointObject::getInternalGpuIndex() const { return mGpuIndex; }

} // namespace scene
} // namespace svulkan2