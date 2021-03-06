#include <pybind11/pybind11.h>

#include "pysapien_content.hpp"

#include "pysapien-renderer.hpp"

using namespace sapien;
namespace py = pybind11;

PYBIND11_MODULE(pysapien_renderer, m) { buildRenderer(m); }
PYBIND11_MODULE(pysapien, m) { buildSapien(m); }
