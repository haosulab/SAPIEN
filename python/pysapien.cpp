#include <pybind11/pybind11.h>
#include "pysapien_content.hpp"
#include "pysapien_renderer.hpp"

#ifdef SAPIEN_SIMSENSE
#include "pysimsense.hpp"
#endif

using namespace sapien;
namespace py = pybind11;

PYBIND11_MODULE(pysapien, m) {
  buildRenderer(m);
  buildSapien(m);

#ifdef SAPIEN_SIMSENSE
  buildSimsense(m);
#endif
}
