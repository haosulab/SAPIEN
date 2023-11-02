#include "generator.hpp"
#include <pybind11/smart_holder.h>
namespace py = pybind11;

Generator<int> init_sapien(py::module &m);
Generator<int> init_physx(py::module &m);
void init_sapien_renderer_internal(py::module &m);
void init_sapien_renderer(py::module &m);
void init_math(py::module &m);
void init_serialization(py::module &m);

#ifdef SAPIEN_CUDA
void init_simsense(py::module &m);
#endif

PYBIND11_MODULE(pysapien, m) {
  auto sapien_gen = init_sapien(m);
  auto physx_gen = init_physx(m);

  sapien_gen.next();
  physx_gen.next();

  init_sapien_renderer_internal(m);
  init_sapien_renderer(m);
  init_math(m);
  init_serialization(m);

  sapien_gen.next();
  physx_gen.next();

#ifdef SAPIEN_CUDA
  init_simsense(m);
#endif
}
