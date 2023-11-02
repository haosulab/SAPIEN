#ifdef SAPIEN_CUDA

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <sapien/array.h>
#include <simsense/core.h>
#include <simsense/util.h>
#include <stdint.h>

namespace py = pybind11;

using namespace sapien;

template <class T> static simsense::Mat2d<T> ndarray2Mat2d(py::array_t<T> arr) {
  py::buffer_info buf = arr.request();
  auto ptr = static_cast<T *>(buf.ptr);
  simsense::Mat2d<T> new_arr(buf.shape[0], buf.shape[1], ptr);
  return new_arr;
}

template <class T> static py::array_t<T> Mat2d2ndarray(simsense::Mat2d<T> arr) {
  py::array_t<T> new_arr = py::array({arr.rows(), arr.cols()}, arr.data());
  return new_arr;
}

// static void DLCapsuleDeleter(PyObject *data) {
//   DLManagedTensor *tensor = (DLManagedTensor *)PyCapsule_GetPointer(data, "dltensor");
//   if (tensor) {
//     tensor->deleter(const_cast<DLManagedTensor *>(tensor));
//   } else {
//     PyErr_Clear();
//   }
// }

class DepthSensorEnginePython : public simsense::DepthSensorEngine {
public:
  DepthSensorEnginePython(uint32_t _rows, uint32_t _cols, uint32_t _rgbRows, uint32_t _rgbCols,
                          float _focalLen, float _baselineLen, float _minDepth, float _maxDepth,
                          uint64_t infraredNoiseSeed, float _speckleShape, float _speckleScale,
                          float _gaussianMu, float _gaussianSigma, bool _rectified,
                          uint8_t _censusWidth, uint8_t _censusHeight, uint32_t _maxDisp,
                          uint8_t _bfWidth, uint8_t _bfHeight, uint8_t _p1, uint8_t _p2,
                          uint8_t _uniqRatio, uint8_t _lrMaxDiff, uint8_t _mfSize,
                          py::array_t<float> mapLx, py::array_t<float> mapLy,
                          py::array_t<float> mapRx, py::array_t<float> mapRy,
                          py::array_t<float> a1, py::array_t<float> a2, py::array_t<float> a3,
                          float _b1, float _b2, float _b3, bool _dilation, float _mainFx,
                          float _mainFy, float _mainSkew, float _mainCx, float _mainCy)
      : DepthSensorEngine(_rows, _cols, _rgbRows, _rgbCols, _focalLen, _baselineLen, _minDepth,
                          _maxDepth, infraredNoiseSeed, _speckleShape, _speckleScale, _gaussianMu,
                          _gaussianSigma, _rectified, _censusWidth, _censusHeight, _maxDisp,
                          _bfWidth, _bfHeight, _p1, _p2, _uniqRatio, _lrMaxDiff, _mfSize,
                          ndarray2Mat2d(mapLx), ndarray2Mat2d(mapLy), ndarray2Mat2d(mapRx),
                          ndarray2Mat2d(mapRy), ndarray2Mat2d(a1), ndarray2Mat2d(a2),
                          ndarray2Mat2d(a3), _b1, _b2, _b3, _dilation, _mainFx, _mainFy, _mainSkew,
                          _mainCx, _mainCy) {}

  void compute(py::array_t<uint8_t> left_array, py::array_t<uint8_t> right_array) {
    simsense::Mat2d<uint8_t> left = ndarray2Mat2d<uint8_t>(left_array);
    simsense::Mat2d<uint8_t> right = ndarray2Mat2d<uint8_t>(right_array);
    DepthSensorEngine::compute(left, right);
  }

  // void compute(py::capsule left_capsule, py::capsule right_capsule) {
  //   // Open left capsule
  //   auto leftCapsule = left_capsule.ptr();
  //   DLManagedTensor *leftDLMTensor =
  //       (DLManagedTensor *)PyCapsule_GetPointer(leftCapsule, "dltensor");
  //   if (leftDLMTensor) {
  //     PyCapsule_SetName(leftCapsule,
  //                       "used_dltensor"); // Successfully obtained content, rename capsule
  //   } else {
  //     throw std::runtime_error(
  //         "Left capsule is invalid. Note that DLTensor capsules can only be consumed once");
  //   }

  //   // Open right capsule
  //   auto *rightCapsule = right_capsule.ptr();
  //   DLManagedTensor *rightDLMTensor =
  //       (DLManagedTensor *)PyCapsule_GetPointer(rightCapsule, "dltensor");
  //   if (rightDLMTensor) {
  //     PyCapsule_SetName(rightCapsule,
  //                       "used_dltensor"); // Successfully obtained content, rename capsule
  //   } else {
  //     throw std::runtime_error(
  //         "Right capsule is invalid. Note that DLTensor capsules can only be consumed once");
  //   }

  //   DepthSensorEngine::compute(leftDLMTensor, rightDLMTensor);

  //   leftDLMTensor->deleter(const_cast<DLManagedTensor *>(leftDLMTensor));
  //   rightDLMTensor->deleter(const_cast<DLManagedTensor *>(rightDLMTensor));
  // }

  void compute(CudaArrayHandle leftCuda, CudaArrayHandle rightCuda) {
    // Instance check
    if (leftCuda.shape[0] != rightCuda.shape[0] || leftCuda.shape[1] != rightCuda.shape[1]) {
      throw std::runtime_error("Both images must have the same size");
    }
    if (getInputRows() != leftCuda.shape[0] || getInputCols() != leftCuda.shape[1]) {
      throw std::runtime_error("Input image size different from initiated");
    }
    if (leftCuda.type != "f4" or rightCuda.type != "f4") {
      throw std::runtime_error("Input data type must be float");
    }

    DepthSensorEngine::compute(leftCuda.ptr, rightCuda.ptr);
  }

  py::array_t<float> getNdarray() { return Mat2d2ndarray(getMat2d()); }

  // py::capsule getDLTensorCapsule() {
  //   return py::capsule(getDLTensor(), "dltensor", DLCapsuleDeleter);
  // }

  // py::capsule getPointCloudDLTensorCapsule() {
  //   return py::capsule(getPointCloudDLTensor(), "dltensor", DLCapsuleDeleter);
  // }

  // py::array_t<float> getRgbPointCloudNdarray(py::capsule rgba_capsule) {
  //   // Open rgb capsule
  //   auto rgbaCapsule = rgba_capsule.ptr();
  //   DLManagedTensor *rgbaDLMTensor =
  //       (DLManagedTensor *)PyCapsule_GetPointer(rgbaCapsule, "dltensor");
  //   if (rgbaDLMTensor) {
  //     PyCapsule_SetName(rgbaCapsule,
  //                       "used_dltensor"); // Successfully obtained content, rename capsule
  //   } else {
  //     throw std::runtime_error(
  //         "RGBA capsule is invalid. Note that DLTensor capsules can only be consumed once");
  //   }
  //   return Mat2d2ndarray(getRgbPointCloudMat2d(rgbaDLMTensor));
  // }

  // py::capsule getRgbPointCloudDLTensorCapsule(py::capsule rgba_capsule) {
  //   // Open rgb capsule
  //   auto rgbaCapsule = rgba_capsule.ptr();
  //   DLManagedTensor *rgbaDLMTensor =
  //       (DLManagedTensor *)PyCapsule_GetPointer(rgbaCapsule, "dltensor");
  //   if (rgbaDLMTensor) {
  //     PyCapsule_SetName(rgbaCapsule,
  //                       "used_dltensor"); // Successfully obtained content, rename capsule
  //   } else {
  //     throw std::runtime_error(
  //         "RGBA capsule is invalid. Note that DLTensor capsules can only be consumed once");
  //   }
  //   return py::capsule(getRgbPointCloudDLTensor(rgbaDLMTensor), "dltensor", DLCapsuleDeleter);
  // }

  CudaArrayHandle getCuda() {
    CudaArrayHandle arr{{(int)getOutputRows(), (int)getOutputCols()}, // shape
                        {4 * (int)getOutputCols(), 4},                // strides (in bytes)
                        "f4",
                        getCudaId(),
                        getCudaPtr()};

    return arr;
  }

  CudaArrayHandle getPointCloudCuda() {
    int size = (int)(getOutputRows() * getOutputCols());
    CudaArrayHandle arr{{size, 3},  // shape
                        {4 * 3, 4}, // strides (in bytes)
                        "f4",
                        getCudaId(),
                        getPointCloudCudaPtr()};

    return arr;
  }

  py::array_t<float> getPointCloudNdarray() { return Mat2d2ndarray(getPointCloudMat2d()); }

  py::array_t<float> getRgbPointCloudNdarray(CudaArrayHandle rgba_cuda) {
    return Mat2d2ndarray(getRgbPointCloudMat2d(rgba_cuda.ptr));
  }

  CudaArrayHandle getRgbPointCloudCuda(CudaArrayHandle rgba_cuda) {
    int size = (int)(getOutputRows() * getOutputCols());
    CudaArrayHandle arr{{size, 6},  // shape
                        {4 * 6, 4}, // strides (in bytes)
                        "f4",
                        getCudaId(),
                        getRgbPointCloudCudaPtr(rgba_cuda.ptr)};

    return arr;
  }
};

void init_simsense(py::module &sapien) {
  auto m = sapien.def_submodule("simsense");

  auto PySimsense = py::class_<DepthSensorEnginePython>(m, "DepthSensorEngine");

  PySimsense.def(
      py::init<uint32_t, uint32_t, uint32_t, uint32_t, float, float, float, float, uint64_t, float,
               float, float, float, bool, uint8_t, uint8_t, uint32_t, uint8_t, uint8_t, uint8_t,
               uint8_t, uint8_t, uint8_t, uint8_t, py::array_t<float>, py::array_t<float>,
               py::array_t<float>, py::array_t<float>, py::array_t<float>, py::array_t<float>,
               py::array_t<float>, float, float, float, bool, float, float, float, float,
               float>());
  PySimsense.def("compute", py::overload_cast<py::array_t<uint8_t>, py::array_t<uint8_t>>(
                                &DepthSensorEnginePython::compute));
  // PySimsense.def("compute",
  //                py::overload_cast<py::capsule,
  //                py::capsule>(&DepthSensorEnginePython::compute));
  PySimsense.def("compute", py::overload_cast<CudaArrayHandle, CudaArrayHandle>(
                                &DepthSensorEnginePython::compute));
  PySimsense.def("get_ndarray", &DepthSensorEnginePython::getNdarray);
  // PySimsense.def("get_dl_tensor", &DepthSensorEnginePython::getDLTensorCapsule);
  // PySimsense.def("get_point_cloud_dl_tensor",
  //                &DepthSensorEnginePython::getPointCloudDLTensorCapsule);
  // PySimsense.def("get_rgb_point_cloud_ndarray",
  // &DepthSensorEnginePython::getRgbPointCloudNdarray);
  // PySimsense.def("get_rgb_point_cloud_dl_tensor",
  //                &DepthSensorEnginePython::getRgbPointCloudDLTensorCapsule);
  PySimsense.def("get_cuda", &DepthSensorEnginePython::getCuda);
  PySimsense.def("get_point_cloud_cuda", &DepthSensorEnginePython::getPointCloudCuda);
  PySimsense.def("get_point_cloud_ndarray", &DepthSensorEnginePython::getPointCloudNdarray);
  PySimsense.def("get_rgb_point_cloud_ndarray", &DepthSensorEnginePython::getRgbPointCloudNdarray);
  PySimsense.def("get_rgb_point_cloud_cuda", &DepthSensorEnginePython::getRgbPointCloudCuda);
  PySimsense.def("set_ir_noise_parameters", &DepthSensorEnginePython::setInfraredNoiseParameters);
  PySimsense.def("set_census_window_size", &DepthSensorEnginePython::setCensusWindowSize);
  PySimsense.def("set_matching_block_size", &DepthSensorEnginePython::setMatchingBlockSize);
  PySimsense.def("set_penalties", &DepthSensorEnginePython::setPenalties);
  PySimsense.def("set_uniqueness_ratio", &DepthSensorEnginePython::setUniquenessRatio);
  PySimsense.def("set_lr_max_diff", &DepthSensorEnginePython::setLrMaxDiff);
}

#endif
