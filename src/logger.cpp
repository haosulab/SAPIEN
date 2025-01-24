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
#include "./logger.h"

namespace sapien {
namespace logger {

std::shared_ptr<spdlog::logger> getLogger() {
  static std::shared_ptr<spdlog::logger> logger;
  if (!logger) {
    logger = spdlog::stderr_color_mt("SAPIEN");
    logger->set_level(spdlog::level::warn);
  }
  return logger;
}

void setLogLevel(std::string_view level) {
  if (level == "off") {
    return getLogger()->set_level(spdlog::level::off);
  }
  if (level == "info") {
    return getLogger()->set_level(spdlog::level::info);
  }
  if (level == "warn" || level == "warning") {
    return getLogger()->set_level(spdlog::level::warn);
  }
  if (level == "err" || level == "error") {
    return getLogger()->set_level(spdlog::level::err);
  }
  if (level == "critical") {
    return getLogger()->set_level(spdlog::level::critical);
  }
}

} // namespace logger
} // namespace sapien
