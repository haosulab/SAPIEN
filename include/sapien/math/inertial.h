#pragma once
#include "conversion.h"
#include "pose.h"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace sapien {

struct MassProperties {
  /* mass */
  float mass{1.f};

  /* center of mass */
  Eigen::Vector3f cm{0.f, 0.f, 0.f};

  /* moment of inertia at origin */
  float xx{1.f}, yy{1.f}, zz{1.f}, xy{0.f}, yz{0.f}, xz{0.f};

  Eigen::Matrix<float, 3, 3, Eigen::RowMajor> getCMInertia() const {
    auto xx_ = xx - mass * (cm.y() * cm.y() + cm.z() * cm.z());
    auto yy_ = yy - mass * (cm.z() * cm.z() + cm.x() * cm.x());
    auto zz_ = zz - mass * (cm.x() * cm.x() + cm.y() * cm.y());
    auto xy_ = xy + mass * cm.x() * cm.y();
    auto yz_ = yz + mass * cm.y() * cm.z();
    auto xz_ = xz + mass * cm.x() * cm.z();
    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> res;
    res << xx_, xy_, xz_, xy_, yy_, yz_, xz_, yz_, zz_;
    return res;
  }

  void setCMInertia(Eigen::Matrix<float, 3, 3, Eigen::RowMajor> const &inertia) {
    float xx_ = inertia(0, 0);
    float yy_ = inertia(1, 1);
    float zz_ = inertia(2, 2);
    float xy_ = inertia(0, 1);
    float yz_ = inertia(1, 2);
    float xz_ = inertia(0, 2);

    xx = xx_ + mass * (cm.y() * cm.y() + cm.z() * cm.z());
    yy = yy_ + mass * (cm.z() * cm.z() + cm.x() * cm.x());
    zz = zz_ + mass * (cm.x() * cm.x() + cm.y() * cm.y());
    xy = xy_ - mass * cm.x() * cm.y();
    yz = yz_ - mass * cm.y() * cm.z();
    xz = xz_ - mass * cm.x() * cm.z();
  }

  Eigen::Matrix<float, 3, 3, Eigen::RowMajor> getOriginInertia() const {
    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> res;
    res << xx, xy, xz, xy, yy, yz, xz, yz, zz;
    return res;
  }

  void setOriginInertia(Eigen::Matrix<float, 3, 3, Eigen::RowMajor> const &inertia) {
    float xx_ = inertia(0, 0);
    float yy_ = inertia(1, 1);
    float zz_ = inertia(2, 2);
    float xy_ = inertia(0, 1);
    float yz_ = inertia(1, 2);
    float xz_ = inertia(0, 2);

    xx = xx_;
    yy = yy_;
    zz = zz_;
    xy = xy_;
    yz = yz_;
    xz = xz_;
  }

  MassProperties operator+(MassProperties const &other) const {
    float newMass = mass + other.mass;
    Eigen::Vector3f newCm = (cm * mass + other.cm * other.mass) / newMass;
    return {.mass = newMass,
            .cm = newCm,
            .xx = xx + other.xx,
            .yy = yy + other.yy,
            .zz = zz + other.zz,
            .xy = xy + other.xy,
            .yz = yz + other.yz,
            .xz = xz + other.xz};
  }

  MassProperties scaleSize(Vec3 const &s) const {
    float s3 = s.x * s.y * s.z;
    Vec3 sxyz2 = (0.5 * (xx + yy + zz) - Vec3(xx, yy, zz)) * s * s;
    float xx_ = sxyz2.y + sxyz2.z;
    float yy_ = sxyz2.z + sxyz2.x;
    float zz_ = sxyz2.x + sxyz2.y;
    float xy_ = xy * s.x * s.y;
    float xz_ = xz * s.x * s.z;
    float yz_ = yz * s.y * s.z;

    return {
        .mass = mass * s3,
        .cm = cm.cwiseProduct(Eigen::Vector3f{s.x, s.y, s.z}),
        .xx = xx_ * s3,
        .yy = yy_ * s3,
        .zz = zz_ * s3,
        .xy = xy_ * s3,
        .yz = yz_ * s3,
        .xz = xz_ * s3,
    };
  }

  MassProperties scaleMass(float s) const {
    return {
        .mass = mass * s,
        .cm = cm,
        .xx = xx * s,
        .yy = yy * s,
        .zz = zz * s,
        .xy = xy * s,
        .yz = yz * s,
        .xz = xz * s,
    };
  }

  auto decompose() const {
    auto inertia = getCMInertia();

    Eigen::SelfAdjointEigenSolver<decltype(inertia)> solver(inertia);
    Eigen::Vector3f vs = solver.eigenvalues();
    Eigen::Matrix3f v = solver.eigenvectors();
    if (v.determinant() < 0) {
      v = -v;
    }
    Eigen::Quaternionf quat = Eigen::Quaternionf(v).normalized();
    return std::tuple{
        mass,
        Pose(Vec3(cm.x(), cm.y(), cm.z()), Quat(quat.w(), quat.x(), quat.y(), quat.z())),
        Vec3(vs.x(), vs.y(), vs.z()),
    };
  }

  inline MassProperties transform(Pose const &pose) const {
    auto transform = PoseToEigenMat4(pose);
    auto R = transform.block<3, 3>(0, 0);
    auto newCm = R * cm + transform.block<3, 1>(0, 3);

    auto I = R * getCMInertia() * R.transpose();
    auto xx_ = I(0, 0) + mass * (newCm.y() * newCm.y() + newCm.z() * newCm.z());
    auto yy_ = I(1, 1) + mass * (newCm.z() * newCm.z() + newCm.x() * newCm.x());
    auto zz_ = I(2, 2) + mass * (newCm.x() * newCm.x() + newCm.y() * newCm.y());
    auto xy_ = I(0, 1) - mass * newCm.x() * newCm.y();
    auto yz_ = I(1, 2) - mass * newCm.y() * newCm.z();
    auto xz_ = I(0, 2) - mass * newCm.x() * newCm.z();

    return {
        .mass = mass,
        .cm = newCm,
        .xx = xx_,
        .yy = yy_,
        .zz = zz_,
        .xy = xy_,
        .yz = yz_,
        .xz = xz_,
    };
  }

  static inline MassProperties
  FromMassInertia(float mass, Eigen::Vector3f const &cm,
                  Eigen::Matrix<float, 3, 3, Eigen::RowMajor> const &inertia) {
    MassProperties mp;
    mp.mass = mass;
    mp.cm = cm;
    mp.setCMInertia(inertia);
    return mp;
  }

  static inline MassProperties FromCMLocalPose(float mass, Pose const &pose, Vec3 const &inertia) {
    MassProperties mp;
    mp.mass = mass;
    mp.cm = {pose.p.x, pose.p.y, pose.p.z};

    auto R = Eigen::Quaternionf(pose.q.w, pose.q.x, pose.q.y, pose.q.z).toRotationMatrix();
    mp.setCMInertia(R * Eigen::Vector3f(inertia.x, inertia.y, inertia.z).asDiagonal() *
                    R.transpose());
    return mp;
  }

  static inline MassProperties
  FromMesh(Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &vertices,
           Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> const &triangles);

  static inline MassProperties FromBox(Vec3 const &halfExtents) {
    float x = halfExtents.x;
    float y = halfExtents.y;
    float z = halfExtents.z;
    float mass = x * y * z * 8.f;
    return {
        .mass = mass,
        .cm = {0.f, 0.f, 0.f},
        .xx = (1.f / 3.f) * mass * (y * y + z * z),
        .yy = (1.f / 3.f) * mass * (x * x + z * z),
        .zz = (1.f / 3.f) * mass * (x * x + y * y),
        .xy = 0.f,
        .yz = 0.f,
        .xz = 0.f,
    };
  }

  static inline MassProperties FromSphere(float r) {
    float mass = 4.f / 3.f * std::numbers::pi_v<float> * r * r * r;
    float I = 2.f / 5.f * mass * r * r;
    return {
        .mass = mass,
        .cm = {0.f, 0.f, 0.f},
        .xx = I,
        .yy = I,
        .zz = I,
        .xy = 0.f,
        .yz = 0.f,
        .xz = 0.f,
    };
  }

  static inline MassProperties FromCylinder(float r, float halfHeight) {
    float mass = std::numbers::pi_v<float> * r * r * halfHeight * 2.f;
    float Iy = (1.f / 12.f) * mass * (3.f * r * r + 4.f * halfHeight * halfHeight);
    float Ix = 0.5f * mass * r * r;
    return {
        .mass = mass,
        .cm = {0.f, 0.f, 0.f},
        .xx = Ix,
        .yy = Iy,
        .zz = Iy,
        .xy = 0.f,
        .yz = 0.f,
        .xz = 0.f,
    };
  }

  static inline MassProperties FromCapsule(float r, float halfHeight) {
    float ms = 4.f / 3.f * std::numbers::pi_v<float> * r * r * r;
    float mc = std::numbers::pi_v<float> * r * r * halfHeight * 2.f;

    float Iy = mc * (0.25 * r * r + (1.0 / 3.0) * halfHeight * halfHeight) +
               ms * (0.4 * r * r + 0.75 * r * halfHeight + halfHeight * halfHeight);
    float Ix = (mc * 0.5 + ms * 0.4) * r * r;

    return {
        .mass = ms + mc,
        .cm = {0.f, 0.f, 0.f},
        .xx = Ix,
        .yy = Iy,
        .zz = Iy,
        .xy = 0.f,
        .yz = 0.f,
        .xz = 0.f,
    };
  }
};

static inline void subexpressions(Eigen::Vector3f w0, Eigen::Vector3f w1, Eigen::Vector3f w2,
                                  Eigen::Vector3f &f1, Eigen::Vector3f &f2, Eigen::Vector3f &f3,
                                  Eigen::Vector3f &g0, Eigen::Vector3f &g1, Eigen::Vector3f &g2) {
  Eigen::Vector3f t0 = w0 + w1;
  f1 = t0 + w2;
  Eigen::Vector3f t1 = w0.cwiseProduct(w0);
  Eigen::Vector3f t2 = t1 + w1.cwiseProduct(t0);
  f2 = t2 + w2.cwiseProduct(f1);
  f3 = w0.cwiseProduct(t1) + w1.cwiseProduct(t2) + w2.cwiseProduct(f2);
  g0 = f2 + w0.cwiseProduct((f1 + w0));
  g1 = f2 + w1.cwiseProduct((f1 + w1));
  g2 = f2 + w2.cwiseProduct((f1 + w2));
}

// https://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
MassProperties MassProperties::FromMesh(
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &vertices,
    Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> const &triangles) {

  float I0 = 0.f;
  Eigen::Vector3f I1{0.f, 0.f, 0.f};
  Eigen::Vector3f I2{0.f, 0.f, 0.f};
  Eigen::Vector3f I3{0.f, 0.f, 0.f};
  for (auto &r : triangles.rowwise()) {
    Eigen::Vector3f v0 = vertices.row(r.x());
    Eigen::Vector3f v1 = vertices.row(r.y());
    Eigen::Vector3f v2 = vertices.row(r.z());
    Eigen::Vector3f e0 = v1 - v0;
    Eigen::Vector3f e1 = v2 - v0;
    Eigen::Vector3f d = e0.cross(e1);

    Eigen::Vector3f f1, f2, f3;
    Eigen::Vector3f g0, g1, g2;
    subexpressions(v0, v1, v2, f1, f2, f3, g0, g1, g2);
    I0 += d.x() * f1.x();
    I1 += d.cwiseProduct(f2);
    I2 += d.cwiseProduct(f3);
    I3 += d.cwiseProduct(Eigen::Vector3f{v0.y(), v0.z(), v0.x()}.cwiseProduct(g0) +
                         Eigen::Vector3f{v1.y(), v1.z(), v1.x()}.cwiseProduct(g1) +
                         Eigen::Vector3f{v2.y(), v2.z(), v2.x()}.cwiseProduct(g2));
  }

  float mass = I0 / 6.f;
  Eigen::Vector3f cm = I1 / 24.f / mass;

  Eigen::Vector3f cm_yzx{cm.y(), cm.z(), cm.x()};
  Eigen::Vector3f cm_zxy{cm.z(), cm.x(), cm.y()};

  // Eigen::Vector3f diag =
  //     Eigen::Vector3f{I2.y() + I2.z(), I2.x() + I2.z(), I2.x() + I2.y()} / 60.f -
  //     mass * (cm_yzx.cwiseProduct(cm_yzx) + cm_zxy.cwiseProduct(cm_zxy));
  // Eigen::Vector3f offdiag = mass * cm.cwiseProduct(cm_yzx) - I3 / 120.f;

  Eigen::Vector3f diag = Eigen::Vector3f{I2.y() + I2.z(), I2.x() + I2.z(), I2.x() + I2.y()} / 60.f;
  Eigen::Vector3f offdiag = -I3 / 120.f;

  MassProperties mp;
  mp.mass = mass;
  mp.cm = cm;
  mp.xx = diag.x();
  mp.yy = diag.y();
  mp.zz = diag.z();
  mp.xy = offdiag.x();
  mp.yz = offdiag.y();
  mp.xz = offdiag.z();

  // mp.inertiaOrigin << diag.x(), offdiag.x(), offdiag.z(), offdiag.x(), diag.y(), offdiag.y(),
  //     offdiag.z(), offdiag.y(), diag.z();
  return mp;
}

} // namespace sapien
