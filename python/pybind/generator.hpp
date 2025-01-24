/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <coroutine>
#include <iostream>
#include <optional>

template <std::movable T> class Generator {
public:
  struct promise_type {
    Generator<T> get_return_object() { return Generator{Handle::from_promise(*this)}; }
    static std::suspend_always initial_suspend() noexcept { return {}; }
    static std::suspend_always final_suspend() noexcept { return {}; }
    std::suspend_always yield_value(T value) noexcept {
      current_value = std::move(value);
      return {};
    }
    // Disallow co_await in generator coroutines.
    void await_transform() = delete;
    [[noreturn]] static void unhandled_exception() { throw; }

    std::optional<T> current_value;

    void return_void() noexcept {}
  };

  using Handle = std::coroutine_handle<promise_type>;

  explicit Generator(const Handle coroutine) : m_coroutine{coroutine} {}

  Generator() = default;
  ~Generator() {
    if (m_coroutine)
      m_coroutine.destroy();
  }

  Generator(const Generator &) = delete;
  Generator &operator=(const Generator &) = delete;

  Generator(Generator &&other) noexcept : m_coroutine{other.m_coroutine} {
    other.m_coroutine = {};
  }
  Generator &operator=(Generator &&other) noexcept {
    if (this != &other) {
      if (m_coroutine)
        m_coroutine.destroy();
      m_coroutine = other.m_coroutine;
      other.m_coroutine = {};
    }
    return *this;
  }

  bool done() const { return !m_coroutine || m_coroutine.done(); }

  const T &next() const {
    if (done()) {
      throw std::runtime_error("end of generator");
    }
    m_coroutine.resume();
    return *m_coroutine.promise().current_value;
  }

private:
  Handle m_coroutine;
};
