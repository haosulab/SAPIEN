#include <mutex>
#include <queue>
#include <spdlog/spdlog.h>
#include <vector>

template <typename T> class ThreadSafeVector {
public:
  explicit ThreadSafeVector(uint32_t num) : mVec(num){};

  void write(const std::vector<T> &input);
  std::vector<T> read();

private:
  std::vector<T> mVec;
  std::mutex mLock;
};

template <typename T> class ThreadSafeQueue {
public:
  ThreadSafeQueue() : mQueue(){};
  bool empty();
  void clear();
  T pop();
  T front();
  void push(const T &element);

private:
  std::mutex mLock;
  std::queue<T> mQueue;
};
