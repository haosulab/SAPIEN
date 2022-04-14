#pragma once
#include <future>
#include <memory>

namespace sapien {
template <typename Res> class IAwaitable : public std::enable_shared_from_this<IAwaitable<Res>> {
public:
  virtual Res wait() = 0;
  virtual bool ready() = 0;
  virtual ~IAwaitable() = default;
};

template <typename Res> class AwaitableFuture : public IAwaitable<Res> {
  std::future<Res> mFuture;

public:
  AwaitableFuture(std::future<Res> &&future) : mFuture(std::move(future)) {}
  Res wait() { return mFuture.get(); }
  bool ready() { return mFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready; }
};

} // namespace sapien
