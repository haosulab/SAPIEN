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
#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

namespace svulkan2 {

void exportTriangleMesh(std::string const &filename,
                        std::vector<float> const &vertices,
                        std::vector<uint32_t> const &indices,
                        std::vector<float> const &normals,
                        std::vector<float> const &uvs);
}; // namespace svulkan2