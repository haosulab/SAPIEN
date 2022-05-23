#pragma once
#include "event.h"
#include "event_listener.h"
#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

namespace sapien {
template <typename T> class EventEmitter;

class Subscription {
public:
  virtual void unsubscribe() = 0;
  virtual ~Subscription() = default;
};

template <typename T> class ListenerSubscription : public Subscription {
  EventEmitter<T> *mEmitter;
  IEventListener<T> *mListener;

public:
  ListenerSubscription(EventEmitter<T> &emitter, IEventListener<T> &listener)
      : mEmitter(&emitter), mListener(&listener){};
  void disable() { mEmitter = nullptr; };
  void unsubscribe() override;

  friend EventEmitter<T>;
};

template <typename T> class CallbackSubscription : public Subscription {
  EventEmitter<T> *mEmitter;
  std::function<void(T &)> mCallback;

public:
  CallbackSubscription(EventEmitter<T> &emitter, std::function<void(T &)> callback)
      : mEmitter(&emitter), mCallback(callback){};
  void disable() { mEmitter = nullptr; };
  void unsubscribe() override;

  friend EventEmitter<T>;
};

template <typename T> class EventEmitter {
  std::vector<std::shared_ptr<ListenerSubscription<T>>> mListenerSubscriptions;
  std::vector<std::shared_ptr<CallbackSubscription<T>>> mCallbackSubscriptions;

public:
  std::shared_ptr<Subscription> registerListener(IEventListener<T> &listener) {
    auto it = std::find_if(mListenerSubscriptions.begin(), mListenerSubscriptions.end(),
                           [&](auto &sub) { return sub->mListener == &listener; });
    if (it != mListenerSubscriptions.end()) {
      return *it;
    }
    auto sub = std::make_shared<ListenerSubscription<T>>(*this, listener);
    mListenerSubscriptions.push_back(sub);
    return sub;
  }

  std::shared_ptr<Subscription> registerCallback(std::function<void(T &)> callback) {
    auto sub = std::make_shared<CallbackSubscription<T>>(*this, callback);
    mCallbackSubscriptions.push_back(sub);
    return sub;
  }

  void unregisterListener(IEventListener<T> &listener) {
    auto it = std::find_if(mListenerSubscriptions.begin(), mListenerSubscriptions.end(),
                           [&](auto &sub) { return sub->mListener == &listener; });
    if (it != mListenerSubscriptions.end()) {
      mListenerSubscriptions.erase(it);
    }
  }

  void unregisterCallback(Subscription &subscription) {
    auto it = std::find_if(mCallbackSubscriptions.begin(), mCallbackSubscriptions.end(),
                        [&](auto &sub) { return sub.get() == &subscription; });
    if (it != mCallbackSubscriptions.end()) {
      mCallbackSubscriptions.erase(it);
    }
  }

  void emit(T &event) {
    for (auto &l : mListenerSubscriptions) {
      l->mListener->onEvent(event);
    }
    for (auto &l : mCallbackSubscriptions) {
      l->mCallback(event);
    }
  }

  ~EventEmitter() {
    for (auto &l : mListenerSubscriptions) {
      l->disable();
    }
    for (auto &l : mCallbackSubscriptions) {
      l->disable();
    }
  }
};

template <typename T> void ListenerSubscription<T>::unsubscribe() {
  if (mEmitter) {
    mEmitter->unregisterListener(*mListener);
  }
}

template <typename T> void CallbackSubscription<T>::unsubscribe() {
  if (mEmitter) {
    mEmitter->unregisterCallback(*this);
  }
}

} // namespace sapien
