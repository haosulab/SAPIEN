#pragma once
#include "vec3.h"
#include <cmath>
#include <istream>
#include <ostream>

namespace sapien {

struct Quat {
  inline Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
  inline Quat() : Quat(1.f, 0.f, 0.f, 0.f) {}
  inline Quat(Vec3 const &axis, float angle) {
    float s = std::sin(angle / 2.f);
    w = std::cos(angle / 2.f);
    x = axis.x * s;
    y = axis.y * s;
    z = axis.z * s;
  }

  inline float lengthSqr() const { return w * w + x * x + y * y + z * z; }
  inline float length() const { return std::sqrt(lengthSqr()); }
  inline void normalize() {
    float l = length();
    if (l != 0.f) {
      float il = 1.f / l;
      w *= il;
      x *= il;
      y *= il;
      z *= il;
    }
  }
  inline Quat getNormalized() const {
    Quat v = *this;
    v.normalize();
    return v;
  }

  // clang-format off
  inline bool isInf() const { return std::isinf(w) || std::isinf(x) || std::isinf(y) || std::isinf(z); }
  inline bool isNan() const { return std::isnan(w) || std::isnan(x) || std::isnan(y) || std::isnan(z); }
  // clang-format on

  inline bool isUnit(float tol = 1e-3) const { return !isInf() && std::abs(length() - 1.f) < tol; }
  inline bool isSane() const { return isUnit(1e-2); }

  inline Quat getConjugate() const { return {w, -x, -y, -z}; }

  Quat &operator*=(Quat const &q) {
    w = w * q.w - x * q.x - y * q.y - z * q.z;
    x = w * q.x + q.w * x + y * q.z - q.y * z;
    y = w * q.y + q.w * y + z * q.x - q.z * x;
    z = w * q.z + q.w * z + x * q.y - q.x * y;
    return *this;
  }
  Quat operator*(Quat const &q) const {
    return {w * q.w - x * q.x - y * q.y - z * q.z, w * q.x + q.w * x + y * q.z - q.y * z,
            w * q.y + q.w * y + z * q.x - q.z * x, w * q.z + q.w * z + x * q.y - q.x * y};
  }

  Vec3 rotate(Vec3 const &v) const {
    Vec3 u(x, y, z);
    return 2.f * u.dot(v) * u + (w * w - u.dot(u)) * v + 2.f * w * u.cross(v);
  }

  float w{1.f}, x{0.f}, y{0.f}, z{0.f};
  template <class Archive> void serialize(Archive &ar) { ar(w, x, y, z); }
};

} // namespace sapien
