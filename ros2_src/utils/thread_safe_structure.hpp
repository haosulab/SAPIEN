#include <mutex>
#include <queue>
#include <spdlog/spdlog.h>
#include <vector>

template <typename T> class ThreadSafeVector {
public:
  explicit ThreadSafeVector(uint32_t num) : mVec(num){};

  void write(const std::vector<T> &input) {
    std::lock_guard<std::mutex> guard(mLock);
    if (input.size() != mVec.size()) {
      spdlog::warn("Attempt to write a thread safe vector with a vector of differernt size!");
      return;
    }
    for (int i = 0; i < mVec.size(); ++i) {
      mVec[i] = input[i];
    }
  };
  std::vector<T> read() {
    std::lock_guard<std::mutex> guard(mLock);
    std::vector<T> result(mVec);
    return result;
  };

private:
  std::vector<T> mVec;
  std::mutex mLock;
};

template <typename T> class ThreadSafeQueue {
public:
  ThreadSafeQueue() : mQueue(){};
  bool empty() {
    std::lock_guard<std::mutex> guard(mLock);
    return mQueue.empty();
  };
  void clear() {
    std::lock_guard<std::mutex> guard(mLock);
    std::queue<T> emptyQueue;
    std::swap(mQueue, emptyQueue);
  };
  T pop() {
    std::lock_guard<std::mutex> guard(mLock);
    T topElement = mQueue.front();
    mQueue.pop();
    return topElement;
  };
  T front() {
    std::lock_guard<std::mutex> guard(mLock);
    T topElement = mQueue.front();
    return topElement;
  };
  void push(const T &element) {
    std::lock_guard<std::mutex> guard(mLock);
    mQueue.push(element);
  };

private:
  std::mutex mLock;
  std::queue<T> mQueue;
};
