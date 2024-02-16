#pragma once
#include <string>
#include <vector>
#include <vulkan/vulkan.hpp>

struct DLManagedTensorVersioned;
struct DLManagedTensor;

namespace sapien {

std::vector<int> ShapeToStrides(std::vector<int> const &shape, int elemSize);

struct CudaArrayHandle {
  std::vector<int> shape;
  std::vector<int> strides;
  std::string type;
  int cudaId{-1};
  void *ptr{nullptr};

  bool isContiguous() const;
  DLManagedTensor *toDLPack() const;

  void checkCongiguous() const;
  void checkShape(std::vector<int> const &expected) const;
  void checkStride(std::vector<int> const &expected) const;

  int bytes() const;
};

// template <int... Shape> static bool checkShape(std::vector<int> const &shape);
// template <> bool checkShape(std::vector<int> const &shape) { return shape.empty(); }
// template <int Head, int... Tail> bool checkShape(std::vector<int> const &shape) {
//   return !shape.empty() && (Head == -1 || shape[0] == Head) && checkShape<Tail...>(shape);
// }

// template <typename T> static bool checkType(std::string const &type) {
//   if constexpr (std::is_floating_point<T>::value) {
//     return type == "f" + std::to_string(sizeof(T));
//   }
//   if constexpr (std::is_integral<T>::value && std::is_unsigned<T>::value) {
//     return type == "u" + std::to_string(sizeof(T));
//   }
//   if constexpr (std::is_integral<T>::value && std::is_signed<T>::value) {
//     return type == "i" + std::to_string(sizeof(T));
//   }
//   return false;
// }

// template <typename T, int... Shape> struct CudaArrayHandleT : public CudaArrayHandle {
//   CudaArrayHandleT(CudaArrayHandle const &other) {
//     if (!checkType<T>(other.type)) {
//       throw std::runtime_error("incompatble types");
//     }
//     if (!checkShape<Shape...>(other.type)) {
//       throw std::runtime_error("incompatble shapes");
//     }

//     shape = other.shape;
//     strides = other.strides;
//     type = other.type;
//     cudaId = other.cudaId;
//     ptr = other.ptr;
//   }
// };

struct CudaArray {
  /** Create uninitialized empty CudaArray */
  CudaArray() {}

  /** Create cuda array on the current cuda runtime device
   * @param shape shape of the array
   * @param typestr in numpy typestr format
   */
  CudaArray(std::vector<int> shape_, std::string type_);

  /** Create uint8 cuda array on the current cuda runtime device
   * @param data CPU raw data buffer
   * @param size of the raw data buffer
   * @return cuda array filled with data buffer
   */
  static CudaArray FromData(void *data, int size);

  template <typename T> static CudaArray FromData(std::vector<T> data) {
    return CudaArray::FromData(data.data(), data.size() * sizeof(T));
  }

  CudaArrayHandle handle() const;

  /** @return byte size of the array in bytes */
  int bytes() const;

  DLManagedTensor *moveToDLPack();

  CudaArray(CudaArray const &) = delete;
  CudaArray &operator=(CudaArray const &) = delete;
  CudaArray(CudaArray &&other);
  CudaArray &operator=(CudaArray &&other);
  ~CudaArray();

  std::vector<int> shape;
  std::string type;
  int cudaId{-1};
  void *ptr{nullptr};
};

struct CudaHostArray {
  CudaHostArray(){};
  CudaHostArray(std::vector<int> shape_, std::string type_);

  void copyFrom(CudaArray const &array);

  CudaHostArray(CudaHostArray const &) = delete;
  CudaHostArray &operator=(CudaHostArray const &) = delete;
  CudaHostArray(CudaHostArray &&other);
  CudaHostArray &operator=(CudaHostArray &&other);
  ~CudaHostArray();

  std::vector<int> shape;
  std::string type;
  void *ptr{nullptr};
};

struct CpuArrayHandle {
  std::vector<int> shape;
  std::vector<int> strides;
  std::string type;
  void *ptr{};
};

struct CpuArray {
  std::vector<int> shape;
  std::string type;

  std::vector<char> data;
};

} // namespace sapien
