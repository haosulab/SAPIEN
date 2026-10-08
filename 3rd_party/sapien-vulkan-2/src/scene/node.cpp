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
#include "svulkan2/scene/node.h"
#include "svulkan2/scene/object.h"
#include <algorithm>

namespace svulkan2 {
namespace scene {

Node::Node(std::string const &name) : mName(name) {}

void Node::removeChild(Node &child) {
  auto it = std::find(mChildren.begin(), mChildren.end(), &child);
  if (it == mChildren.end()) {
    throw std::invalid_argument("child node does not exist");
  }
  mChildren.erase(it);
}

void Node::clearChild() { mChildren = {}; }

void Node::setTransform(Transform const &transform) { mTransform = transform; }

void Node::setPosition(glm::vec3 const &pos) {
  mTransform.position = pos;
  updateGlobalModelMatrixRecursive();
}
void Node::setRotation(glm::quat const &rot) {
  mTransform.rotation = rot;
  updateGlobalModelMatrixRecursive();
}
void Node::setScale(glm::vec3 const &scale) {
  mTransform.scale = scale;
  updateGlobalModelMatrixRecursive();
}

void Node::updateGlobalModelMatrixRecursive() {
  glm::mat4 localMatrix = glm::translate(glm::mat4(1), mTransform.position) *
                          glm::toMat4(mTransform.rotation) *
                          glm::scale(glm::mat4(1), mTransform.scale);
  mTransform.prevWorldModelMatrix = mTransform.worldModelMatrix;
  if (!mParent) {
    mTransform.worldModelMatrix = localMatrix;
  } else {
    mTransform.worldModelMatrix = mParent->mTransform.worldModelMatrix * localMatrix;
  }
  for (auto c : mChildren) {
    c->updateGlobalModelMatrixRecursive();
  }
}

std::vector<Object *> Node::getObjectsRecursive() const {
  std::vector<Object *> result;
  for (auto c : mChildren) {
    if (auto object = dynamic_cast<Object *>(c))
      if (object) {
        result.push_back(object);
      }
    auto childObjects = c->getObjectsRecursive();
    result.insert(result.end(), childObjects.begin(), childObjects.end());
  }
  return result;
}

glm::mat4 Node::computeWorldModelMatrix() const {
  glm::mat4 localMatrix = glm::translate(glm::mat4(1), mTransform.position) *
                          glm::toMat4(mTransform.rotation) *
                          glm::scale(glm::mat4(1), mTransform.scale);
  if (!mParent) {
    return localMatrix;
  }
  return mParent->computeWorldModelMatrix() * localMatrix;
}

void Node::markRemovedRecursive() {
  mRemoved = true;
  for (auto c : mChildren) {
    c->markRemovedRecursive();
  }
}

} // namespace scene
} // namespace svulkan2