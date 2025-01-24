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
#include "./image.h"
#include <filesystem>
#include <svulkan2/resource/material.h>

namespace sapien {
namespace sapien_renderer {
class SapienRenderEngine;

class SapienRenderTexture {
public:
  enum class FilterMode { eNEAREST, eLINEAR };
  enum class AddressMode { eREPEAT, eBORDER, eEDGE, eMIRROR };

  SapienRenderTexture(uint32_t width, uint32_t height, uint32_t depth, vk::Format format,
                      std::vector<char> const &rawData, int dim, uint32_t mipLevels = 1,
                      FilterMode filterMode = FilterMode::eLINEAR,
                      AddressMode addressMode = AddressMode::eREPEAT, bool srgb = false);

  vk::Format getFormat() const;
  int getDim() const;
  int getWidth() const;
  int getHeight() const;
  int getDepth() const;

  FilterMode getFilterMode() const { return getMinFilterMode(); }
  FilterMode getMinFilterMode() const;
  FilterMode getMagFilterMode() const;

  AddressMode getAddressMode() const { return getAddressModeU(); }
  AddressMode getAddressModeU() const;
  AddressMode getAddressModeV() const;
  AddressMode getAddressModeW() const;
  int getMipmapLevels() const;
  bool getIsSrgb() const;
  int getChannels() const;

  void download(void *data);
  void upload(void *data);

  std::shared_ptr<svulkan2::resource::SVTexture> getTexture() const { return mTexture; }

protected:
  SapienRenderTexture() {}

  std::shared_ptr<SapienRenderEngine> mEngine;
  std::shared_ptr<svulkan2::resource::SVTexture> mTexture;
};

class SapienRenderTexture2D : public SapienRenderTexture {

public:
  SapienRenderTexture2D(uint32_t width, uint32_t height, vk::Format format,
                        std::vector<char> const &rawData, uint32_t mipLevels = 1,
                        FilterMode filterMode = FilterMode::eLINEAR,
                        AddressMode addressMode = AddressMode::eREPEAT, bool srgb = false);
  SapienRenderTexture2D(std::string_view filename, uint32_t mipLevels, FilterMode filterMode,
                        AddressMode addressMode, bool srgb);
  explicit SapienRenderTexture2D(std::shared_ptr<svulkan2::resource::SVTexture> tex);

  std::string getFilename() const;
};

} // namespace sapien_renderer
} // namespace sapien
