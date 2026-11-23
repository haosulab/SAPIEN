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
#pragma once

#include <exception>
#include <spdlog/spdlog.h>
#include <string_view>

namespace svulkan2 {
namespace logger {

std::shared_ptr<spdlog::logger> getLogger();

template <typename... Args>
inline void debug(spdlog::string_view_t fmt, const Args &...args) {
  getLogger()->debug(fmt, args...);
};

template <typename... Args>
inline void info(spdlog::string_view_t fmt, const Args &...args) {
  getLogger()->info(fmt, args...);
};

template <typename... Args>
inline void warn(spdlog::string_view_t fmt, const Args &...args) {
  getLogger()->warn(fmt, args...);
};

template <typename... Args>
inline void error(spdlog::string_view_t fmt, const Args &...args) {
  getLogger()->error(fmt, args...);
};

template <typename... Args>
inline void critical(spdlog::string_view_t fmt, const Args &...args) {
  getLogger()->critical(fmt, args...);
};

} // namespace log
} // namespace svulkan2