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
#include "generator.hpp"
#include <pybind11/pybind11.h>
namespace py = pybind11;

Generator<int> init_sapien(py::module &m);
Generator<int> init_physx(py::module &m);
void init_sapien_renderer_internal(py::module &m);
void init_sapien_renderer(py::module &m);
void init_math(py::module &m);

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

  sapien_gen.next();
  physx_gen.next();

#ifdef SAPIEN_CUDA
  init_simsense(m);
#endif
}
