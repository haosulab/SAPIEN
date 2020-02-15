#include "thread_safe_structure.h"

// Thread Safe Vector
template <typename T> void ThreadSafeVector<T>::write(const std::vector<T> &input) {
  std::lock_guard<std::mutex> guard(mLock);
  if (input.size() != mVec.size()) {
    spdlog::warn("Attempt to write a thread safe vector with a vector of differernt size!");
    return;
  }
  for (int i = 0; i < mVec.size(); ++i) {
    mVec[i] = input[i];
  }
}
template <typename T> std::vector<T> ThreadSafeVector<T>::read() {
  std::lock_guard<std::mutex> guard(mLock);
  std::vector<T> result(mVec);
  return result;
}

// Thread Safe Queue
template <typename T> bool ThreadSafeQueue<T>::empty() {
  std::lock_guard<std::mutex> guard(mLock);
  return mQueue.empty();
}
template <typename T> void ThreadSafeQueue<T>::clear() {
  std::lock_guard<std::mutex> guard(mLock);
  std::queue<T> emptyQueue;
  std::swap(mQueue, emptyQueue);
}
template <typename T> T ThreadSafeQueue<T>::pop() {
  std::lock_guard<std::mutex> guard(mLock);
  T topElement = mQueue.front();
  mQueue.pop();
  return topElement;
}
template <typename T> void ThreadSafeQueue<T>::push(const T &element) {
  std::lock_guard<std::mutex> guard(mLock);
  mQueue.push(element);
}
template <typename T> T ThreadSafeQueue<T>::front() {
  std::lock_guard<std::mutex> guard(mLock);
  T topElement = mQueue.front();
  return topElement;
}
