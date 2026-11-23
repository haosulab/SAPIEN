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
#include "widget.h"
#include <functional>

namespace svulkan2 {
namespace ui {

UI_CLASS(FileChooser) {
  UI_DECLARE_LABEL(FileChooser);
  UI_ATTRIBUTE(FileChooser, std::string, Title);
  UI_ATTRIBUTE(FileChooser, std::string, Filter);
  UI_ATTRIBUTE(FileChooser, std::string, Path);

  UI_ATTRIBUTE(
      FileChooser,
      std::function<void(std::shared_ptr<FileChooser>, std::string name, std::string path)>,
      Callback);

public:
  FileChooser();

  void build() override;
  void open();
  void close();
};

} // namespace ui
} // namespace svulkan2