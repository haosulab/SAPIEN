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
#include "sapien/sapien_renderer/cubemap.h"
#include "sapien/sapien_renderer/sapien_renderer_system.h"

namespace sapien {
namespace sapien_renderer {

SapienRenderCubemap::SapienRenderCubemap(std::string const &filename) {
  mEngine = SapienRenderEngine::Get();
  mCubemap = mEngine->getContext()->getResourceManager()->CreateCubemapFromFile(filename, 5);
}

SapienRenderCubemap::SapienRenderCubemap(std::string const &px, std::string const &nx,
                                         std::string const &py, std::string const &ny,
                                         std::string const &pz, std::string const &nz) {
  mEngine = SapienRenderEngine::Get();
  mCubemap = mEngine->getContext()->getResourceManager()->CreateCubemapFromFiles(
      {px, nx, py, ny, pz, nz}, 5);
}

void SapienRenderCubemap::exportKtx(std::string const &filename) {
  mCubemap->load();
  mCubemap->uploadToDevice();
  mCubemap->exportKTX(filename);
}

} // namespace sapien_renderer
} // namespace sapien
