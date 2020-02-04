#pragma once
#include "event.h"
#include "event_listener.h"
#include <vector>
#include <algorithm>

namespace sapien {
template <typename T> class EventEmitter {
  std::vector<IEventListener<T> *> mListeners;

public:
  void registerListener(IEventListener<T> &listener) {
    if (std::find(mListeners.begin(), mListeners.end(), &listener) != mListeners.end()) {
      // spdlog::error("Listener already registered");
      return;
    }
    mListeners.push_back(&listener);
  }
  void unregisterListener(IEventListener<T> &listener) {
    auto it = std::find(mListeners.begin(), mListeners.end(), &listener);
    if (it != mListeners.end()) {
      mListeners.erase(it);
    }
  }
  void emit(T &event) {
    for (auto l : mListeners) {
      l->onEvent(event);
    }
  }
};
} // namespace sapien
