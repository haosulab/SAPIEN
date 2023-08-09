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
