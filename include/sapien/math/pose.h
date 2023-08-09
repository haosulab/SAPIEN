#pragma once
#include "quat.h"
#include "vec3.h"
#include <cstring>
#include <ostream>

namespace sapien {

// TODO: unittests
struct Pose {
  Quat q{1.f, 0.f, 0.f, 0.f};
  Vec3 p{0.f, 0.f, 0.f};

  Pose() : q(1.f, 0.f, 0.f, 0.f), p(0.f, 0.f, 0.f) {}
  Pose(Vec3 p_, Quat q_) : q(q_), p(p_) {}
  explicit Pose(Vec3 p_) : q({1.f, 0.f, 0.f, 0.f}), p(p_) {}
  explicit Pose(Quat q_) : q(q_), p({0.f, 0.f, 0.f}) {}

  Pose getInverse() const {
    Quat q2 = q.getConjugate();
    return {q2.rotate(-p), q2};
  }
  Vec3 operator*(Vec3 const &v) const { return q.rotate(v) + p; }
  Pose operator*(Pose const &other) const { return {q.rotate(other.p) + p, q * other.q}; }
  Pose &operator*=(Pose const &other) {
    *this = *this * other;
    return *this;
  }

  bool isSane() const { return p.isSane() && q.isSane(); }
  template <class Archive> void serialize(Archive &ar) { ar(q, p); }
};

static Pose const POSE_GL_TO_ROS({0, 0, 0}, {-0.5, -0.5, 0.5, 0.5});
static Pose const POSE_ROS_TO_GL({0, 0, 0}, {-0.5, 0.5, -0.5, -0.5});

} // namespace sapien
