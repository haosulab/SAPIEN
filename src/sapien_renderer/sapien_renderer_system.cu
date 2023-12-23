#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#define HD __host__ __device__

namespace {

struct Vec3 {
  HD inline Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
  HD inline explicit Vec3(float s_ = 0.f) : x(s_), y(s_), z(s_) {}

  // clang-format off
  HD inline Vec3 operator-() const { return {-x, -y, -z}; }

  HD inline Vec3 operator+(Vec3 const &other) const { return {x + other.x, y + other.y, z + other.z}; }
  HD inline Vec3 operator-(Vec3 const &other) const { return {x - other.x, y - other.y, z - other.z}; }
  HD inline Vec3 operator*(Vec3 const &other) const { return {x * other.x, y * other.y, z * other.z}; }
  HD inline Vec3 operator/(Vec3 const &other) const { return {x / other.x, y / other.y, z / other.z}; }

  HD inline Vec3 operator+(float other) const { return {x + other, y + other, z + other}; }
  HD inline Vec3 operator-(float other) const { return {x - other, y - other, z - other}; }
  HD inline Vec3 operator*(float other) const { return {x * other, y * other, z * other}; }
  HD inline Vec3 operator/(float other) const { return {x / other, y / other, z / other}; }

  HD inline Vec3 &operator+=(Vec3 const &other) {  x += other.x; y += other.y; z += other.z; return *this; }
  HD inline Vec3 &operator-=(Vec3 const &other) {  x -= other.x; y -= other.y; z -= other.z; return *this; }
  HD inline Vec3 &operator*=(Vec3 const &other) {  x *= other.x; y *= other.y; z *= other.z; return *this; }
  HD inline Vec3 &operator/=(Vec3 const &other) {  x /= other.x; y /= other.y; z /= other.z; return *this; }

  HD inline Vec3 &operator+=(float other) {  x += other; y += other; z += other; return *this; }
  HD inline Vec3 &operator-=(float other) {  x -= other; y -= other; z -= other; return *this; }
  HD inline Vec3 &operator*=(float other) {  x *= other; y *= other; z *= other; return *this; }
  HD inline Vec3 &operator/=(float other) {  x /= other; y /= other; z /= other; return *this; }

  HD inline float dot(Vec3 const &v) const { return x * v.x + y * v.y + z * v.z; }
  HD inline Vec3 cross(Vec3 const &v) const { return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x}; }
  // clang-format on

  float x{0.f}, y{0.f}, z{0.f};
};

HD inline Vec3 operator+(float s, Vec3 const &v) { return v + s; }
HD inline Vec3 operator-(float s, Vec3 const &v) { return -v + s; }
HD inline Vec3 operator*(float s, Vec3 const &v) { return v * s; }
HD inline Vec3 operator/(float s, Vec3 const &v) { return {s / v.x, s / v.y, s / v.z}; }

struct Quat {
  HD inline Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
  HD inline Quat() : Quat(1.f, 0.f, 0.f, 0.f) {}

  HD inline float lengthSqr() const { return w * w + x * x + y * y + z * z; }

  HD inline Quat &operator*=(Quat const &q) {
    w = w * q.w - x * q.x - y * q.y - z * q.z;
    x = w * q.x + q.w * x + y * q.z - q.y * z;
    y = w * q.y + q.w * y + z * q.x - q.z * x;
    z = w * q.z + q.w * z + x * q.y - q.x * y;
    return *this;
  }
  HD inline Quat operator*(Quat const &q) const {
    return {w * q.w - x * q.x - y * q.y - z * q.z, w * q.x + q.w * x + y * q.z - q.y * z,
            w * q.y + q.w * y + z * q.x - q.z * x, w * q.z + q.w * z + x * q.y - q.x * y};
  }

  HD inline Quat getConjugate() const { return {w, -x, -y, -z}; }

  HD inline Vec3 rotate(Vec3 const &v) const {
    Vec3 u(x, y, z);
    return 2.f * u.dot(v) * u + (w * w - u.dot(u)) * v + 2.f * w * u.cross(v);
  }

  float w{1.f}, x{0.f}, y{0.f}, z{0.f};
};

struct Pose {
  Quat q{1.f, 0.f, 0.f, 0.f};
  Vec3 p{0.f, 0.f, 0.f};

  HD inline Pose() : q(1.f, 0.f, 0.f, 0.f), p(0.f, 0.f, 0.f) {}
  HD inline Pose(Vec3 p_, Quat q_) : q(q_), p(p_) {}
  HD inline explicit Pose(Vec3 p_) : q({1.f, 0.f, 0.f, 0.f}), p(p_) {}
  HD inline explicit Pose(Quat q_) : q(q_), p({0.f, 0.f, 0.f}) {}

  HD inline Pose getInverse() const {
    Quat q2 = q.getConjugate();
    return {q2.rotate(-p), q2};
  }
  HD inline Vec3 operator*(Vec3 const &v) const { return q.rotate(v) + p; }
  HD inline Pose operator*(Pose const &other) const {
    return {q.rotate(other.p) + p, q * other.q};
  }
  HD inline Pose &operator*=(Pose const &other) {
    *this = *this * other;
    return *this;
  }

  HD inline void toMatrix(float *result, Vec3 const &scale) {
    Vec3 c0 = q.rotate(Vec3(1, 0, 0));
    Vec3 c1 = q.rotate(Vec3(0, 1, 0));
    Vec3 c2 = q.rotate(Vec3(0, 0, 1));

    result[0] = c0.x * scale.x;
    result[1] = c0.y * scale.x;
    result[2] = c0.z * scale.x;
    result[3] = 0.f;

    result[4] = c1.x * scale.y;
    result[5] = c1.y * scale.y;
    result[6] = c1.z * scale.y;
    result[7] = 0.f;

    result[8] = c2.x * scale.z;
    result[9] = c2.y * scale.z;
    result[10] = c2.z * scale.z;
    result[11] = 0.f;

    result[12] = p.x;
    result[13] = p.y;
    result[14] = p.z;
    result[15] = 1.f;
  }
};

} // namespace

__global__ void transform_sapien_to_render(
    float *__restrict__ *__restrict__ scene_transform_buffers, // output buffers
    int *__restrict__ scene_indices,      // the scene that each shape belongs to
    int *__restrict__ transform_indices,  // the scene transform index for each shape
    float *__restrict__ local_transforms, // local pose for each shape
    float *__restrict__ local_scale,      // local scale for each shape

    int *__restrict__ parent_indices,      // index for the parent of each pose
    float *__restrict__ parent_transforms, // parent pose array
    int parent_transform_stride,
    int count // total shape count
) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  Vec3 scale = Vec3(local_scale[3 * g], local_scale[3 * g + 1], local_scale[3 * g + 2]);

  Pose pose_s2b =
      Pose(Vec3(local_transforms[7 * g], local_transforms[7 * g + 1], local_transforms[7 * g + 2]),
           Quat(local_transforms[7 * g + 3], local_transforms[7 * g + 4],
                local_transforms[7 * g + 5], local_transforms[7 * g + 6]));

  int parent_index = parent_indices[g];
  Pose pose_b2w = Pose(Vec3(parent_transforms[parent_transform_stride * parent_index],
                            parent_transforms[parent_transform_stride * parent_index + 1],
                            parent_transforms[parent_transform_stride * parent_index + 2]),
                       Quat(parent_transforms[parent_transform_stride * parent_index + 3],
                            parent_transforms[parent_transform_stride * parent_index + 4],
                            parent_transforms[parent_transform_stride * parent_index + 5],
                            parent_transforms[parent_transform_stride * parent_index + 6]));
  Pose p = pose_b2w * pose_s2b;

  int scene_index = scene_indices[g];
  int transform_index = transform_indices[g];

  p.toMatrix(scene_transform_buffers[scene_index] + transform_index * 16, scale);
}

__global__ void transform_sapien_to_camera_view_model(
    float *__restrict__ *__restrict__ camera_buffers, float *__restrict__ local_transforms,
    int *__restrict__ parent_indices, float *__restrict__ parent_transforms,
    int parent_transform_stride, int count) {
  int g = blockIdx.x * blockDim.x + threadIdx.x;
  if (g >= count) {
    return;
  }

  Pose pose_s2b =
      Pose(Vec3(local_transforms[7 * g], local_transforms[7 * g + 1], local_transforms[7 * g + 2]),
           Quat(local_transforms[7 * g + 3], local_transforms[7 * g + 4],
                local_transforms[7 * g + 5], local_transforms[7 * g + 6]));

  int parent_index = parent_indices[g];
  Pose pose_b2w = Pose(Vec3(parent_transforms[parent_transform_stride * parent_index],
                            parent_transforms[parent_transform_stride * parent_index + 1],
                            parent_transforms[parent_transform_stride * parent_index + 2]),
                       Quat(parent_transforms[parent_transform_stride * parent_index + 3],
                            parent_transforms[parent_transform_stride * parent_index + 4],
                            parent_transforms[parent_transform_stride * parent_index + 5],
                            parent_transforms[parent_transform_stride * parent_index + 6]));
  Pose p = pose_b2w * pose_s2b;

  p.getInverse().toMatrix(camera_buffers[g], Vec3(1));
  p.toMatrix(camera_buffers[g] + 16, Vec3(1));
}

namespace sapien {

constexpr int BLOCK_SIZE = 128;
void transform_sapien_to_render(float **scene_transform_buffers, int *scene_indices,
                                int *transform_indices, float *local_transforms,
                                float *local_scale, int *parent_indices, float *parent_transforms,
                                int parent_transform_stride, int count, void *stream) {
  ::transform_sapien_to_render<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                 (cudaStream_t)stream>>>(
      scene_transform_buffers, scene_indices, transform_indices, local_transforms, local_scale,
      parent_indices, parent_transforms, parent_transform_stride, count);
}

void transform_sapien_to_camera_view_model(float **camera_buffers, float *local_transforms,
                                           int *parent_indices, float *parent_transforms,
                                           int parent_transform_stride, int count, void *stream) {
  ::transform_sapien_to_camera_view_model<<<(count + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE, 0,
                                            (cudaStream_t)stream>>>(
      camera_buffers, local_transforms, parent_indices, parent_transforms, parent_transform_stride,
      count);
}

} // namespace sapien
