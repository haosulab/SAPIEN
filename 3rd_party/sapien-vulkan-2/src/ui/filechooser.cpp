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
#include "svulkan2/ui/filechooser.h"
#include <ImGuiFileDialog.h>

namespace svulkan2 {
namespace ui {

FileChooser::FileChooser() : mTitle("Choose File"), mPath(".") {}

void FileChooser::build() {
  if (ImGuiFileDialog::Instance()->Display(mLabel)) {
    if (ImGuiFileDialog::Instance()->IsOk()) {
      std::string filePathName = ImGuiFileDialog::Instance()->GetFilePathName();
      std::string filePath = ImGuiFileDialog::Instance()->GetCurrentPath();
      if (mCallback) {
        mCallback(std::static_pointer_cast<FileChooser>(shared_from_this()), filePathName,
                  filePath);
      }
    }
    ImGuiFileDialog::Instance()->Close();
  }
}

void FileChooser::open() {
  ImGuiFileDialog::Instance()->OpenDialog(mLabel, mTitle, mFilter.c_str(), mPath);
}

void FileChooser::close() { ImGuiFileDialog::Instance()->Close(); }

} // namespace ui
} // namespace svulkan2