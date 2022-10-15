#pragma once

#include <stdint.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <simsense/core.h>

namespace py = pybind11;

using namespace simsense;

void buildSimsense(py::module &parent) {
    py::module m = parent.def_submodule("simsense");

    auto PySimsense = py::class_<DepthSensorEngine>(m, "DepthSensorEngine");

    PySimsense.def(
        py::init<
            uint32_t, uint32_t, float, float, float, float, bool,
            uint8_t, uint8_t, uint32_t, uint8_t, uint8_t, uint8_t, uint8_t,
            uint8_t, uint8_t, uint8_t, py::array_t<float>, py::array_t<float>,
            py::array_t<float>, py::array_t<float>
        >()
    );
    PySimsense.def(
        py::init<
            uint32_t, uint32_t, uint32_t, uint32_t, float, float, float,
            float, bool, uint8_t, uint8_t, uint32_t, uint8_t, uint8_t,
            uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, py::array_t<float>,
            py::array_t<float>, py::array_t<float>, py::array_t<float>,
            py::array_t<float>, py::array_t<float>, py::array_t<float>,
            float, float, float, bool
        >()
    );
    PySimsense.def("compute", &DepthSensorEngine::compute);
    PySimsense.def("_set_penalties", &DepthSensorEngine::setPenalties);
    PySimsense.def("_set_census_window_size", &DepthSensorEngine::setCensusWindowSize);
    PySimsense.def("_set_matching_block_size", &DepthSensorEngine::setMatchingBlockSize);
    PySimsense.def("_set_uniqueness_ratio", &DepthSensorEngine::setUniquenessRatio);
    PySimsense.def("_set_lr_max_diff", &DepthSensorEngine::setLrMaxDiff);
}
