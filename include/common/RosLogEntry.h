#pragma once

#include "TimeUtil.h"
#include "Singleton.h"
#include <unordered_map>
#include <string>
#include <mutex>
#include <memory>
#include <ros/ros.h>

namespace COMMON {

class ScopedFuncLogger {
  std::string _functionName;
  unsigned long long _timestamp;

 public:
  ScopedFuncLogger(const std::string& functionName) : _functionName(functionName) {
#if 0
        _timestamp = COMMON::TimeUtil::GetCurrentTimestampMicro();
#else
    _timestamp = COMMON::TimeUtil::GetCurrentTimestamp();
#endif
    std::string info = "entry " + functionName;
    ROS_INFO_STREAM(info);
  }
  ~ScopedFuncLogger() {
#if 0
        unsigned long long current = COMMON::TimeUtil::GetCurrentTimestampMicro();
#else
    unsigned long long current = COMMON::TimeUtil::GetCurrentTimestamp();
#endif
    unsigned long long cost = current - _timestamp;
    std::string info = "exit " + _functionName + ", cost : " + std::to_string(cost);
    ROS_INFO_STREAM(info);
  }
};

class FuncAccumulatedTimer {
 private:
  std::string _functionName;
  unsigned long long _interval = 0x7fffffffffffffffLL;
  unsigned long long _count = 0;
  unsigned long long _total_cost = 0;
  unsigned long long _entry_timestamp = 0;

 public:
  FuncAccumulatedTimer(const std::string& functionName, unsigned long long interval = 0) : _functionName(functionName) {
    if (0 != interval) {
      _interval = interval;
    }
  }
  ~FuncAccumulatedTimer() {
    unsigned long long avg = 0;
    if (0 != _count) {
      avg = _total_cost / _count;
    }
    std::string info = "Function " + _functionName + "has been executed" + std::to_string(_total_cost) + "us, avg " +
                       std::to_string(avg) + "us";
    ROS_INFO_STREAM(info);
  }

 public:
  inline void Entry() {
    _entry_timestamp = COMMON::TimeUtil::GetCurrentTimestampMicro();
  }
  inline void Exit() {
    ++_count;
    unsigned long long current = COMMON::TimeUtil::GetCurrentTimestampMicro();
    _total_cost += current - _entry_timestamp;
  }
};

class TimeoutUtil {
 public:
  TimeoutUtil(uint64_t interval) : m_interval(interval) {
  }

  bool Timeout() {
    unsigned long long now = COMMON::TimeUtil::GetCurrentTimestamp();
    if (now - m_lastTimestamp < m_interval) {
      return false;
    }
    m_lastTimestamp = now;
    return true;
  }

 private:
  unsigned long long m_lastTimestamp = 0;
  unsigned long long m_interval = 5000;
};

}  // namespace COMMON
