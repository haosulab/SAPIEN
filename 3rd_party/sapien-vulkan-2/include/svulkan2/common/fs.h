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
#include <filesystem>
#include <string>
#include <vector>

namespace svulkan2 {

namespace fs = std::filesystem;

std::vector<char> readFile(std::filesystem::path const &filename);

bool inline check_file(fs::path const &path) {
  return fs::is_regular_file(path);
}

bool inline check_dir(fs::path const &path) { return fs::is_directory(path); }

void inline check_file_required(fs::path const &path, std::string const &err) {
  if (!check_file(path)) {
    throw std::runtime_error(err);
  }
}

void inline check_dir_required(fs::path const &path, std::string const &err) {
  if (!check_dir(path)) {
    throw std::runtime_error(err);
  }
}

} // namespace svulkan2