#pragma once
#include <cmath>

#ifdef __CUDA_ARCH__
#define CUDA_CALLABLE __host__ __device__
#else
#define CUDA_CALLABLE
#endif

namespace sapien {

struct Vec3 {
  inline CUDA_CALLABLE Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
  inline CUDA_CALLABLE explicit Vec3(float s_ = 0.f) : x(s_), y(s_), z(s_) {}

  // clang-format off
  inline CUDA_CALLABLE Vec3 operator-() const { return {-x, -y, -z}; }

  inline CUDA_CALLABLE Vec3 operator+(Vec3 const &other) const { return {x + other.x, y + other.y, z + other.z}; }
  inline CUDA_CALLABLE Vec3 operator-(Vec3 const &other) const { return {x - other.x, y - other.y, z - other.z}; }
  inline CUDA_CALLABLE Vec3 operator*(Vec3 const &other) const { return {x * other.x, y * other.y, z * other.z}; }
  inline CUDA_CALLABLE Vec3 operator/(Vec3 const &other) const { return {x / other.x, y / other.y, z / other.z}; }

  inline CUDA_CALLABLE Vec3 operator+(float other) const { return {x + other, y + other, z + other}; }
  inline CUDA_CALLABLE Vec3 operator-(float other) const { return {x - other, y - other, z - other}; }
  inline CUDA_CALLABLE Vec3 operator*(float other) const { return {x * other, y * other, z * other}; }
  inline CUDA_CALLABLE Vec3 operator/(float other) const { return {x / other, y / other, z / other}; }

  inline CUDA_CALLABLE Vec3 &operator+=(Vec3 const &other) {  x += other.x; y += other.y; z += other.z; return *this; }
  inline CUDA_CALLABLE Vec3 &operator-=(Vec3 const &other) {  x -= other.x; y -= other.y; z -= other.z; return *this; }
  inline CUDA_CALLABLE Vec3 &operator*=(Vec3 const &other) {  x *= other.x; y *= other.y; z *= other.z; return *this; }
  inline CUDA_CALLABLE Vec3 &operator/=(Vec3 const &other) {  x /= other.x; y /= other.y; z /= other.z; return *this; }

  inline CUDA_CALLABLE Vec3 &operator+=(float other) {  x += other; y += other; z += other; return *this; }
  inline CUDA_CALLABLE Vec3 &operator-=(float other) {  x -= other; y -= other; z -= other; return *this; }
  inline CUDA_CALLABLE Vec3 &operator*=(float other) {  x *= other; y *= other; z *= other; return *this; }
  inline CUDA_CALLABLE Vec3 &operator/=(float other) {  x /= other; y /= other; z /= other; return *this; }

  inline CUDA_CALLABLE float dot(Vec3 const &v) const { return x * v.x + y * v.y + z * v.z; }
  inline CUDA_CALLABLE Vec3 cross(Vec3 const &v) const { return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x}; }
  // clang-format on

  inline CUDA_CALLABLE float lengthSqr() const { return dot(*this); }
  inline CUDA_CALLABLE float length() const { return std::sqrt(lengthSqr()); }
  inline CUDA_CALLABLE void normalize() {
    float l = length();
    if (l != 0.f) {
      float il = 1.f / l;
      x *= il;
      y *= il;
      z *= il;
    }
  }
  inline CUDA_CALLABLE Vec3 getNormalized() const {
    Vec3 v = *this;
    v.normalize();
    return v;
  }

  inline CUDA_CALLABLE bool isInf() const {
    return std::isinf(x) || std::isinf(y) || std::isinf(z);
  }
  inline CUDA_CALLABLE bool isNan() const {
    return std::isnan(x) || std::isnan(y) || std::isnan(z);
  }
  inline CUDA_CALLABLE bool isSane() const { return !isInf() && !isNan(); }

  float x{0.f}, y{0.f}, z{0.f};
};

inline CUDA_CALLABLE Vec3 operator+(float s, Vec3 const &v) { return v + s; }
inline CUDA_CALLABLE Vec3 operator-(float s, Vec3 const &v) { return -v + s; }
inline CUDA_CALLABLE Vec3 operator*(float s, Vec3 const &v) { return v * s; }
inline CUDA_CALLABLE Vec3 operator/(float s, Vec3 const &v) { return {s / v.x, s / v.y, s / v.z}; }

} // namespace sapien
