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
#ifndef SIMSENSE_UTIL_H
#define SIMSENSE_UTIL_H

// #include <dlpack/dlpack.h>
#include "config.h"
#include <memory>

namespace simsense {

template <class T> class Mat2d {
public:
  Mat2d(size_t rows, size_t cols, T *data) {
    m_rows = rows;
    m_cols = cols;
    m_data = data;
  }
  T *data() { return m_data; }
  size_t rows() const { return m_rows; }
  size_t cols() const { return m_cols; }

private:
  size_t m_rows, m_cols;
  T *m_data;
};

// template <class T> void DLMTensorDeleter(DLManagedTensor *self) {
//   delete[] self->dl_tensor.shape;
//   delete static_cast<std::shared_ptr<T> *>(self->manager_ctx);
//   delete self;
// }

} // namespace simsense

#endif