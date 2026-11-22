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
#include "svulkan2/ui/widget.h"
#include <algorithm>
#include <imgui.h>

namespace svulkan2 {
namespace ui {

void Widget::setParent(std::weak_ptr<Widget> parent) { mParent = parent; }

void Widget::remove() {
  if (!mParent.expired()) {
    auto p = mParent.lock();
    auto it =
        std::find(p->mChildren.begin(), p->mChildren.end(), shared_from_this());
    p->mChildren.erase(it);
  }
}

void Widget::removeChildren() { mChildren.clear(); }

} // namespace ui
} // namespace svulkan2