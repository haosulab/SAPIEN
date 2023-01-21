#pragma once

#include <stdint.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <simsense/core.h>

namespace py = pybind11;

void buildSimsense(py::module &parent) {
    py::module m = parent.def_submodule("simsense");

    auto PySimsense = py::class_<simsense::DepthSensorEngine>(m, "DepthSensorEngine");

    PySimsense.def(
        py::init<
            uint32_t, uint32_t, float, float, float, float, uint64_t,
            float, float, float, float, bool, uint8_t, uint8_t, uint32_t,
            uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t,
            py::array_t<float>, py::array_t<float>, py::array_t<float>, py::array_t<float>
        >()
    );
    PySimsense.def(
        py::init<
            uint32_t, uint32_t, uint32_t, uint32_t, float, float, float,
            float, uint64_t, float, float, float, float,  bool, uint8_t,
            uint8_t, uint32_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t,
            uint8_t, uint8_t, py::array_t<float>, py::array_t<float>,
            py::array_t<float>, py::array_t<float>, py::array_t<float>,
            py::array_t<float>, py::array_t<float>, float, float, float, bool
        >()
    );
    PySimsense.def("compute", py::overload_cast<py::capsule, py::capsule>(&simsense::DepthSensorEngine::compute));
    PySimsense.def("compute", py::overload_cast<py::array_t<uint8_t>, py::array_t<uint8_t>>(&simsense::DepthSensorEngine::compute));
    PySimsense.def("get_ndarray", &simsense::DepthSensorEngine::getNdarray);
    PySimsense.def("get_dl_tensor", &simsense::DepthSensorEngine::getDLTensor);
    PySimsense.def("set_ir_noise_parameters", &simsense::DepthSensorEngine::setInfraredNoiseParameters);
    PySimsense.def("set_census_window_size", &simsense::DepthSensorEngine::setCensusWindowSize);
    PySimsense.def("set_matching_block_size", &simsense::DepthSensorEngine::setMatchingBlockSize);
    PySimsense.def("set_penalties", &simsense::DepthSensorEngine::setPenalties);
    PySimsense.def("set_uniqueness_ratio", &simsense::DepthSensorEngine::setUniquenessRatio);
    PySimsense.def("set_lr_max_diff", &simsense::DepthSensorEngine::setLrMaxDiff);
}
