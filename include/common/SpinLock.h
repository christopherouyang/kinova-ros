#pragma once

#include <atomic>
#include <mutex>

namespace COMMON {
class SpinLock {
 public:
  SpinLock() : m_flag(false) {
  }

  bool try_lock() {
    bool expect = false;
    return m_flag.compare_exchange_weak(expect, true);
  }

  void lock() {
    bool expect = false;
    while (!m_flag.compare_exchange_weak(expect, true)) {
      //这里一定要将expect复原，执行失败时expect结果是未定的
      expect = false;
    }
  }

  void unlock() {
    m_flag.store(false);
  }

 private:
  std::atomic<bool> m_flag;
};
}  // namespace COMMON