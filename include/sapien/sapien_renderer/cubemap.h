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
#include <svulkan2/resource/cubemap.h>

namespace sapien {
namespace sapien_renderer {
class SapienRenderEngine;

class SapienRenderCubemap {

public:
  explicit SapienRenderCubemap(std::string const &filename);
  explicit SapienRenderCubemap(std::string const &px, std::string const &nx, std::string const &py,
                               std::string const &ny, std::string const &pz,
                               std::string const &nz);

  [[nodiscard]] inline std::shared_ptr<svulkan2::resource::SVCubemap> getCubemap() const {
    return mCubemap;
  }

  void exportKtx(std::string const &filename);

private:
  std::shared_ptr<SapienRenderEngine> mEngine;
  std::shared_ptr<svulkan2::resource::SVCubemap> mCubemap;
};

} // namespace sapien_renderer
} // namespace sapien
