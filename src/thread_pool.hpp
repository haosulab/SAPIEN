/**
MIT License

Copyright (c) 2016 Mariano Trebino

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// taken from https://github.com/mtrebi/thread-pool
#pragma once

#include <mutex>
#include <queue>

#include <functional>
#include <future>
#include <thread>
#include <utility>
#include <vector>

namespace sapien {

// Thread safe implementation of a Queue using an std::queue
template <typename T> class SafeQueue {
private:
  std::queue<T> m_queue;
  std::mutex m_mutex;

public:
  SafeQueue() {}

  SafeQueue(SafeQueue &other) {
    // TODO:
  }

  ~SafeQueue() {}

  bool empty() {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_queue.empty();
  }

  int size() {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_queue.size();
  }

  void enqueue(T &t) {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_queue.push(t);
  }

  bool dequeue(T &t) {
    std::unique_lock<std::mutex> lock(m_mutex);

    if (m_queue.empty()) {
      return false;
    }
    t = std::move(m_queue.front());

    m_queue.pop();
    return true;
  }
};

class ThreadPool {
private:
  class ThreadWorker {
  private:
    int m_id;
    ThreadPool *m_pool;

  public:
    ThreadWorker(ThreadPool *pool, const int id) : m_pool(pool), m_id(id) {}

    void operator()() {
      std::function<void()> func;
      bool dequeued;
      while (!m_pool->m_shutdown) {
        {
          std::unique_lock<std::mutex> lock(m_pool->m_conditional_mutex);
          if (m_pool->m_queue.empty()) {
            m_pool->m_conditional_lock.wait(lock);
          }
          dequeued = m_pool->m_queue.dequeue(func);
        }
        if (dequeued) {
          func();
        }
      }
    }
  };

  bool m_init;
  bool m_shutdown;
  SafeQueue<std::function<void()>> m_queue;
  std::vector<std::thread> m_threads;
  std::mutex m_conditional_mutex;
  std::condition_variable m_conditional_lock;

public:
  ThreadPool(const int n_threads)
      : m_threads(std::vector<std::thread>(n_threads)), m_shutdown(false), m_init(false) {}

  ThreadPool(const ThreadPool &) = delete;
  ThreadPool(ThreadPool &&) = delete;

  ThreadPool &operator=(const ThreadPool &) = delete;
  ThreadPool &operator=(ThreadPool &&) = delete;

  ~ThreadPool() {
    shutdown();
  }

  // Inits thread pool
  void init() {
    m_init = true;
    for (int i = 0; i < m_threads.size(); ++i) {
      m_threads[i] = std::thread(ThreadWorker(this, i));
    }
  }

  // Waits until threads finish their current task and shutdowns the pool
  void shutdown() {
    m_shutdown = true;
    m_conditional_lock.notify_all();

    for (int i = 0; i < m_threads.size(); ++i) {
      if (m_threads[i].joinable()) {
        m_threads[i].join();
      }
    }
  }

  // Submit a function to be executed asynchronously by the pool
  template <typename F, typename... Args>
  auto submit(F &&f, Args &&... args) -> std::future<decltype(f(args...))> {
    // Create a function with bounded parameters ready to execute
    std::function<decltype(f(args...))()> func =
        std::bind(std::forward<F>(f), std::forward<Args>(args)...);
    // Encapsulate it into a shared ptr in order to be able to copy construct / assign
    auto task_ptr = std::make_shared<std::packaged_task<decltype(f(args...))()>>(func);

    // Wrap packaged task into void function
    std::function<void()> wrapper_func = [task_ptr]() { (*task_ptr)(); };

    // Enqueue generic wrapper function
    m_queue.enqueue(wrapper_func);

    // Wake up one thread if its waiting
    m_conditional_lock.notify_one();

    // Return future from promise
    return task_ptr->get_future();
  }

  bool running() const { return m_init; }
};
} // namespace sapien
