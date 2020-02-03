#pragma once
#include <algorithm>
#include <spdlog/spdlog.h>
#include <vector>

namespace sapien {
class SActorBase;

struct Event {};

struct ActorPreDestroyEvent : public Event {
  SActorBase *actor;
};

struct ArticulationPreDestroyEvent : public Event {};

template <typename T> class IEventListener {
public:
  virtual void onEvent(T &event) = 0;
};

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

}; // namespace sapien
