#pragma once

namespace sapien {

struct Event {
  bool cancellable{false};
  bool cancelled{false};
};

} // namespace sapien
