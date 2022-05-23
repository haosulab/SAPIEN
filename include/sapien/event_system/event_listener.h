#pragma once

namespace sapien {

template <typename T> class IEventListener {
public:
  virtual void onEvent(T &event) = 0;

  virtual ~IEventListener() = default;
};

} // namespace sapien
