#pragma once
#include <cmath>
#include <cstring>
#include <istream>
#include <ostream>

namespace sapien {

struct Vec3 {
  inline Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
  inline explicit Vec3(float s_ = 0.f) : x(s_), y(s_), z(s_) {}

  // clang-format off
  inline Vec3 operator-() const { return {-x, -y, -z}; }

  inline Vec3 operator+(Vec3 const &other) const { return {x + other.x, y + other.y, z + other.z}; }
  inline Vec3 operator-(Vec3 const &other) const { return {x - other.x, y - other.y, z - other.z}; }
  inline Vec3 operator*(Vec3 const &other) const { return {x * other.x, y * other.y, z * other.z}; }
  inline Vec3 operator/(Vec3 const &other) const { return {x / other.x, y / other.y, z / other.z}; }

  inline Vec3 operator+(float other) const { return {x + other, y + other, z + other}; }
  inline Vec3 operator-(float other) const { return {x - other, y - other, z - other}; }
  inline Vec3 operator*(float other) const { return {x * other, y * other, z * other}; }
  inline Vec3 operator/(float other) const { return {x / other, y / other, z / other}; }

  inline Vec3 &operator+=(Vec3 const &other) {  x += other.x; y += other.y; z += other.z; return *this; }
  inline Vec3 &operator-=(Vec3 const &other) {  x -= other.x; y -= other.y; z -= other.z; return *this; }
  inline Vec3 &operator*=(Vec3 const &other) {  x *= other.x; y *= other.y; z *= other.z; return *this; }
  inline Vec3 &operator/=(Vec3 const &other) {  x /= other.x; y /= other.y; z /= other.z; return *this; }

  inline Vec3 &operator+=(float other) {  x += other; y += other; z += other; return *this; }
  inline Vec3 &operator-=(float other) {  x -= other; y -= other; z -= other; return *this; }
  inline Vec3 &operator*=(float other) {  x *= other; y *= other; z *= other; return *this; }
  inline Vec3 &operator/=(float other) {  x /= other; y /= other; z /= other; return *this; }

  inline float dot(Vec3 const &v) const { return x * v.x + y * v.y + z * v.z; }
  inline Vec3 cross(Vec3 const &v) const { return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x}; }
  // clang-format on

  inline float lengthSqr() const { return dot(*this); }
  inline float length() const { return std::sqrt(lengthSqr()); }
  inline void normalize() {
    float l = length();
    if (l != 0.f) {
      float il = 1.f / l;
      x *= il;
      y *= il;
      z *= il;
    }
  }
  inline Vec3 getNormalized() const {
    Vec3 v = *this;
    v.normalize();
    return v;
  }

  inline bool isInf() const { return std::isinf(x) || std::isinf(y) || std::isinf(z); }
  inline bool isNan() const { return std::isnan(x) || std::isnan(y) || std::isnan(z); }
  inline bool isSane() const { return !isInf() && !isNan(); }

  float x{0.f}, y{0.f}, z{0.f};

  template <class Archive> void serialize(Archive &ar) { ar(x, y, z); }
};

inline Vec3 operator+(float s, Vec3 const &v) { return v + s; }
inline Vec3 operator-(float s, Vec3 const &v) { return -v + s; }
inline Vec3 operator*(float s, Vec3 const &v) { return v * s; }
inline Vec3 operator/(float s, Vec3 const &v) { return {s / v.x, s / v.y, s / v.z}; }

} // namespace sapien
