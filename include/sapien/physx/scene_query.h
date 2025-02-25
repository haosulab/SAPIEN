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
#include "sapien/math/math.h"
namespace sapien {
namespace physx {

class PhysxCollisionShape;
class PhysxRigidBaseComponent;

struct PhysxHitInfo {
  Vec3 position;
  Vec3 normal;
  float distance;
  PhysxCollisionShape *shape;
  PhysxRigidBaseComponent *component;

  PhysxHitInfo(Vec3 position_, Vec3 normal_, float distance_, PhysxCollisionShape *shape_,
               PhysxRigidBaseComponent *component_)
      : position(position_), normal(normal_), distance(distance_), shape(shape_),
        component(component_) {}
};

} // namespace physx
} // namespace sapien
