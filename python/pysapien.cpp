#include <pybind11/pybind11.h>

#include "pysapien_content.hpp"

using namespace sapien;
namespace py = pybind11;

PYBIND11_MODULE(pysapien, m) { init_ex1(m); }
