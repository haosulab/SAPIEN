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
#include "logger.h"
#include <spdlog/sinks/stdout_color_sinks.h>

namespace svulkan2 {
namespace logger {

std::shared_ptr<spdlog::logger> getLogger() {
  static std::shared_ptr<spdlog::logger> logger;
  if (!logger) {
    logger = spdlog::stderr_color_mt("svulkan2");
    logger->set_level(spdlog::level::warn);
  }
  return logger;
}

void setLogLevel(std::string_view level) {
  if (level == "off") {
    getLogger()->set_level(spdlog::level::off);
  } else if (level == "trace") {
    getLogger()->set_level(spdlog::level::trace);
  } else if (level == "debug") {
    getLogger()->set_level(spdlog::level::debug);
  } else if (level == "info") {
    getLogger()->set_level(spdlog::level::info);
  } else if (level == "warn" || level == "warning") {
    getLogger()->set_level(spdlog::level::warn);
  } else if (level == "error" || level == "err") {
    getLogger()->set_level(spdlog::level::err);
  } else if (level == "critical") {
    getLogger()->set_level(spdlog::level::critical);
  } else {
    throw std::runtime_error("unknown log level " + std::string(level));
  }
}

std::string getLogLevel() {
  switch (getLogger()->level()) {
  case spdlog::level::off:
    return "off";
  case spdlog::level::trace:
    return "trace";
  case spdlog::level::debug:
    return "debug";
  case spdlog::level::info:
    return "info";
  case spdlog::level::warn:
    return "warn";
  case spdlog::level::err:
    return "err";
  case spdlog::level::critical:
    return "critical";
  default:
    return "";
  }
}

} // namespace logger
} // namespace svulkan2