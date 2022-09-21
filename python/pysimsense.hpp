#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "core.h"

namespace py = pybind11;

using namespace simsense;

template <class T>
Mat2d<T> ndarray2Mat2d(py::array_t<T> arr) {
    py::buffer_info buf = arr.request();
    auto ptr = static_cast<T *>(buf.ptr);
    Mat2d<T> new_arr(buf.shape[0], buf.shape[1], ptr);
    return new_arr;
}

template <class T>
py::array_t<T> Mat2d2ndarray(Mat2d<T> arr) {
    py::str NO_COPY; // Magic to let pybind create array without copying
    py::array_t<T> new_arr = py::array({arr.rows(), arr.cols()}, arr.data(), NO_COPY);
    return new_arr;
}

void initWithReg(uint32_t img_h, uint32_t img_w, uint32_t rgb_h, uint32_t rgb_w, float focal_len, float baseline_len, float min_depth, float max_depth, bool rectified,
                    uint8_t census_width, uint8_t census_height, uint32_t max_disp, uint8_t block_width, uint8_t block_height, uint8_t p1, uint8_t p2,
                    uint8_t uniqueness_ratio, uint8_t lr_max_diff, uint8_t median_filter_size,
                    py::array_t<float> map_lx, py::array_t<float> map_ly, py::array_t<float> map_rx, py::array_t<float> map_ry,
                    py::array_t<float> _a1, py::array_t<float> _a2, py::array_t<float> _a3, float b1, float b2, float b3, bool depth_dilation) {
    Mat2d<float> mapLx = ndarray2Mat2d<float>(map_lx);
    Mat2d<float> mapLy = ndarray2Mat2d<float>(map_ly);
    Mat2d<float> mapRx = ndarray2Mat2d<float>(map_rx);
    Mat2d<float> mapRy = ndarray2Mat2d<float>(map_ry);
    Mat2d<float> a1 = ndarray2Mat2d<float>(_a1);
    Mat2d<float> a2 = ndarray2Mat2d<float>(_a2);
    Mat2d<float> a3 = ndarray2Mat2d<float>(_a3);
    coreInitWithReg(img_h, img_w, focal_len, baseline_len, min_depth, max_depth, rectified,
                    census_width, census_height, max_disp, block_width, block_height, p1, p2,
                    uniqueness_ratio, lr_max_diff, median_filter_size, mapLx, mapLy, mapRx, mapRy,
                    rgb_h, rgb_w, a1, a2, a3, b1, b2, b3, depth_dilation);
}

void initWithoutReg(uint32_t img_h, uint32_t img_w, float focal_len, float baseline_len, float min_depth, float max_depth, bool rectified,
                    uint8_t census_width, uint8_t census_height, uint32_t max_disp, uint8_t block_width, uint8_t block_height, uint8_t p1, uint8_t p2,
                    uint8_t uniqueness_ratio, uint8_t lr_max_diff, uint8_t median_filter_size,
                    py::array_t<float> map_lx, py::array_t<float> map_ly, py::array_t<float> map_rx, py::array_t<float> map_ry) {
    Mat2d<float> mapLx = ndarray2Mat2d<float>(map_lx);
    Mat2d<float> mapLy = ndarray2Mat2d<float>(map_ly);
    Mat2d<float> mapRx = ndarray2Mat2d<float>(map_rx);
    Mat2d<float> mapRy = ndarray2Mat2d<float>(map_ry);
    coreInitWithoutReg(img_h, img_w, focal_len, baseline_len, min_depth, max_depth, rectified,
                        census_width, census_height, max_disp, block_width, block_height, p1, p2,
                        uniqueness_ratio, lr_max_diff, median_filter_size, mapLx, mapLy, mapRx, mapRy);
}

py::array_t<float> compute(py::array_t<uint8_t> left_ndarray, py::array_t<uint8_t> right_ndarray) {
    Mat2d<uint8_t> left = ndarray2Mat2d<uint8_t>(left_ndarray);
    Mat2d<uint8_t> right = ndarray2Mat2d<uint8_t>(right_ndarray);
    Mat2d<float> result = coreCompute(left, right);
    py::array_t<float> result_ndarray = Mat2d2ndarray<float>(result);
    return result_ndarray;
}

void buildSimsense(py::module &parent) {
    py::module m = parent.def_submodule("simsense");

    m.def("init_with_reg", &initWithReg);
    m.def("init_without_reg", &initWithoutReg);
    m.def("compute", &compute, py::return_value_policy::take_ownership);
    m.def("close", &coreClose);
}
