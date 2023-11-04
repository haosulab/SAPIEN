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
