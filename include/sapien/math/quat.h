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
#include "./vec3.h"
#include <cmath>

namespace sapien {

struct Quat {
  inline CUDA_CALLABLE Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
  inline CUDA_CALLABLE Quat() : Quat(1.f, 0.f, 0.f, 0.f) {}
  inline CUDA_CALLABLE Quat(Vec3 const &axis, float angle) {
    float s = std::sin(angle / 2.f);
    w = std::cos(angle / 2.f);
    x = axis.x * s;
    y = axis.y * s;
    z = axis.z * s;
  }

  inline CUDA_CALLABLE float lengthSqr() const { return w * w + x * x + y * y + z * z; }
  inline CUDA_CALLABLE float length() const { return std::sqrt(lengthSqr()); }
  inline CUDA_CALLABLE void normalize() {
    float l = length();
    if (l != 0.f) {
      float il = 1.f / l;
      w *= il;
      x *= il;
      y *= il;
      z *= il;
    }
  }
  inline CUDA_CALLABLE Quat getNormalized() const {
    Quat v = *this;
    v.normalize();
    return v;
  }

  // clang-format off
  inline CUDA_CALLABLE bool isInf() const { return std::isinf(w) || std::isinf(x) || std::isinf(y) || std::isinf(z); }
  inline CUDA_CALLABLE bool isNan() const { return std::isnan(w) || std::isnan(x) || std::isnan(y) || std::isnan(z); }
  // clang-format on

  inline CUDA_CALLABLE bool isUnit(float tol = 1e-3) const {
    return !isInf() && std::abs(length() - 1.f) < tol;
  }
  inline CUDA_CALLABLE bool isSane() const { return isUnit(1e-2); }

  inline CUDA_CALLABLE Quat getConjugate() const { return {w, -x, -y, -z}; }

  inline CUDA_CALLABLE Quat &operator*=(Quat const &q) {
    w = w * q.w - x * q.x - y * q.y - z * q.z;
    x = w * q.x + q.w * x + y * q.z - q.y * z;
    y = w * q.y + q.w * y + z * q.x - q.z * x;
    z = w * q.z + q.w * z + x * q.y - q.x * y;
    return *this;
  }
  inline CUDA_CALLABLE Quat operator*(Quat const &q) const {
    return {w * q.w - x * q.x - y * q.y - z * q.z, w * q.x + q.w * x + y * q.z - q.y * z,
            w * q.y + q.w * y + z * q.x - q.z * x, w * q.z + q.w * z + x * q.y - q.x * y};
  }

  inline CUDA_CALLABLE Vec3 rotate(Vec3 const &v) const {
    Vec3 u(x, y, z);
    return 2.f * u.dot(v) * u + (w * w - u.dot(u)) * v + 2.f * w * u.cross(v);
  }

  float w{1.f}, x{0.f}, y{0.f}, z{0.f};
};

} // namespace sapien
