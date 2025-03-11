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
#ifdef SAPIEN_CUDA

#include "./array.hpp"
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

  void compute(py::array_t<uint8_t> left_array, py::array_t<uint8_t> right_array,
               bool bbox = false, uint32_t bboxStartX = 0, uint32_t bboxStartY = 0,
               uint32_t bboxWidth = 0, uint32_t bboxHeight = 0) {
    simsense::Mat2d<uint8_t> left = ndarray2Mat2d<uint8_t>(left_array);
    simsense::Mat2d<uint8_t> right = ndarray2Mat2d<uint8_t>(right_array);
    DepthSensorEngine::compute(left, right, bbox, bboxStartX, bboxStartY, bboxWidth, bboxHeight);
  }

  void compute(CudaArrayHandle leftCuda, CudaArrayHandle rightCuda, bool bbox = false,
               uint32_t bboxStartX = 0, uint32_t bboxStartY = 0, uint32_t bboxWidth = 0,
               uint32_t bboxHeight = 0) {
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

    DepthSensorEngine::compute(leftCuda.ptr, rightCuda.ptr, bbox, bboxStartX, bboxStartY,
                               bboxWidth, bboxHeight);
  }

  py::array_t<float> getNdarray() { return Mat2d2ndarray(getMat2d()); }

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

  auto PySimsense = py::classh<DepthSensorEnginePython>(m, "DepthSensorEngine");

  PySimsense.def(
      py::init<uint32_t, uint32_t, uint32_t, uint32_t, float, float, float, float, uint64_t, float,
               float, float, float, bool, uint8_t, uint8_t, uint32_t, uint8_t, uint8_t, uint8_t,
               uint8_t, uint8_t, uint8_t, uint8_t, py::array_t<float>, py::array_t<float>,
               py::array_t<float>, py::array_t<float>, py::array_t<float>, py::array_t<float>,
               py::array_t<float>, float, float, float, bool, float, float, float, float,
               float>());
  PySimsense.def(
      "compute",
      py::overload_cast<py::array_t<uint8_t>, py::array_t<uint8_t>, bool, uint32_t, uint32_t,
                        uint32_t, uint32_t>(&DepthSensorEnginePython::compute),
      py::arg("left_array"), py::arg("right_array"), py::arg("bbox") = false,
      py::arg("bbox_start_x") = 0, py::arg("bbox_start_y") = 0, py::arg("bbox_width") = 0,
      py::arg("bbox_height") = 0);
  PySimsense.def("compute",
                 py::overload_cast<CudaArrayHandle, CudaArrayHandle, bool, uint32_t, uint32_t,
                                   uint32_t, uint32_t>(&DepthSensorEnginePython::compute),
                 py::arg("left_cuda"), py::arg("right_cuda"), py::arg("bbox") = false,
                 py::arg("bbox_start_x") = 0, py::arg("bbox_start_y") = 0,
                 py::arg("bbox_width") = 0, py::arg("bbox_height") = 0);
  PySimsense.def("get_ndarray", &DepthSensorEnginePython::getNdarray);
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
