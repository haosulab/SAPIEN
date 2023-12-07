#pragma once
#include "sapien/array.h"
#include <dlpack/dlpack.h>
#include <pybind11/numpy.h>
#include <pybind11/smart_holder.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace sapien;

namespace sapien {

inline py::capsule DLPackToCapsule(DLManagedTensor *tensor) {
  return py::capsule(tensor, "dltensor", [](PyObject *data) {
    auto tensor = static_cast<DLManagedTensor *>(PyCapsule_GetPointer(data, "dltensor"));
    if (tensor) {
      tensor->deleter(tensor);
    } else {
      PyErr_Clear();
    }
  });
}

struct PythonCudaArrayHandle {
  std::vector<int> shape;
  std::vector<int> strides;
  std::string type;
  int cudaId;
  void *ptr;
};

void setPythonCudaBackend(std::string const &backend);
std::string getPythonCudaBackend();

} // namespace sapien

namespace pybind11::detail {

template <> struct type_caster<CudaArray> {
  PYBIND11_TYPE_CASTER(CudaArray, _("CudaArray"));

  bool load(py::handle src, bool convert) { return false; }

  static py::handle cast(CudaArray &&src, py::return_value_policy policy, py::handle parent) {
    auto backend = getPythonCudaBackend();
    auto capsule = DLPackToCapsule(src.moveToDLPack());
    if (backend == "torch") {
      auto from_dlpack =
          py::module_::import("torch").attr("utils").attr("dlpack").attr("from_dlpack");
      return from_dlpack(capsule).release();
    } else if (backend == "jax") {
      auto from_dlpack = py::module_::import("jax").attr("dlpack").attr("from_dlpack");
      return from_dlpack(capsule).release();
    }
    throw std::runtime_error(
        "A cuda backend must be specified. See sapien.set_cuda_tensor_backend for more info.");
  }
};

template <> struct type_caster<CudaArrayHandle> {
  PYBIND11_TYPE_CASTER(CudaArrayHandle, _("CudaArrayHandle"));

  bool load(py::handle src, bool convert) {
    auto obj = py::cast<py::object>(src);
    auto interface = src.cast<py::object>().attr("__cuda_array_interface__").cast<py::dict>();

    auto shape = interface["shape"].cast<py::tuple>().cast<std::vector<int>>();
    auto type = interface["typestr"].cast<std::string>();
    py::dtype dtype(type);

    std::vector<int> strides;
    if (interface.contains("strides") && !interface["strides"].is_none()) {
      strides = interface["strides"].cast<py::tuple>().cast<std::vector<int>>();
    } else {
      int acc = dtype.itemsize();
      strides.push_back(acc);
      for (uint32_t i = shape.size() - 1; i >= 1; --i) {
        acc *= shape.at(i);
        strides.push_back(acc);
      }
      std::ranges::reverse(strides);
    }

    auto data = interface["data"].cast<py::tuple>();
    void *ptr = reinterpret_cast<void *>(data[0].cast<uintptr_t>());

    value = CudaArrayHandle{.shape = shape,
                            .strides = strides,
                            .type = type,
                            .cudaId = 0, // TODO: do we need cuda id?
                            .ptr = ptr};
    return true;
  }

  static py::handle cast(CudaArrayHandle const &src, py::return_value_policy policy,
                         py::handle parent) {
    auto backend = getPythonCudaBackend();

    // fallback
    PythonCudaArrayHandle result{.shape = src.shape,
                                 .strides = src.strides,
                                 .type = src.type,
                                 .cudaId = src.cudaId,
                                 .ptr = src.ptr};
    py::object obj = py::cast(result);

    if (backend == "torch") {
      auto as_tensor = py::module_::import("torch").attr("as_tensor");
      return as_tensor("data"_a = obj, "device"_a = "cuda").release();
    } else if (backend == "jax") {
      auto from_dlpack = py::module_::import("jax").attr("dlpack").attr("from_dlpack");
      auto capsule = DLPackToCapsule(src.toDLPack());
      return from_dlpack(capsule).release();
    }

    return obj.release();
  }
};

template <> struct type_caster<CpuArrayHandle> {
  PYBIND11_TYPE_CASTER(CpuArrayHandle, _("numpy.ndarray"));

  bool load(py::handle src, bool convert) { return false; }

  static py::handle cast(CpuArrayHandle const &src, py::return_value_policy policy,
                         py::handle parent) {

    std::vector<py::ssize_t> shape;
    std::vector<py::ssize_t> strides;
    for (auto s : src.shape) {
      shape.push_back(s);
    }
    for (auto s : src.strides) {
      strides.push_back(s);
    }

    // this makes a copy
    auto array = py::array(py::dtype(src.type), shape, strides, src.ptr);

    // this makes array in-place
    // assert(static_cast<const void *>(array.request().ptr) == static_cast<const void
    // *>(src.ptr));

    return array.release();
  }
};

template <> struct type_caster<CpuArray> {
  PYBIND11_TYPE_CASTER(CpuArray, _("numpy.ndarray"));

  bool load(py::handle src, bool convert) { return false; }

  static py::handle cast(CpuArray const &src, py::return_value_policy policy, py::handle parent) {

    std::vector<py::ssize_t> shape;
    for (auto s : src.shape) {
      shape.push_back(s);
    }

    auto array = py::array(py::dtype(src.type), shape, src.data.data());
    return array.release();
  }
};

} // namespace pybind11::detail
