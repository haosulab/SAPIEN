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
#include <string>
#include <vector>
#include <vulkan/vulkan.hpp>

namespace svulkan2 {

/**
 * load image from file.
 * valid values for desiredChannels are {0, 1, 4}
 * When desiredChannels == 0, channels is detected from the file
 * */
std::vector<uint8_t> loadImage(std::string const &filename, int &width, int &height, int &channels,
                               int desiredChannels);

std::vector<uint8_t> loadImageFromMemory(unsigned char *buffer, int len, int &width, int &height,
                                         int &channels, int desiredChannels);

std::vector<uint8_t> loadKTXImage(std::string const &filename, int &width, int &height,
                                  int &levels, int &faces, int &layers, vk::Format &format);
std::vector<uint8_t> loadKTXImageFromMemory(unsigned char *buffer, size_t size, int &width,
                                            int &height, int &levels, int &faces, int &layers,
                                            vk::Format &format);

// the loaded image is always rgba float16
std::vector<std::byte> loadExrImage(std::string const &filename, int &width, int &height);

template <typename T> std::vector<char> toRawBytes(std::vector<T> const &data) {
  std::vector<char> raw(data.size() * sizeof(T));
  std::memcpy(raw.data(), data.data(), raw.size());
  return raw;
}

uint32_t computeMipLevelSize(vk::Extent3D extent, uint32_t level);
vk::Extent3D computeMipLevelExtent(vk::Extent3D extent, uint32_t level);

} // namespace svulkan2