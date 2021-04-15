#pragma once
#include <ros/ros.h>
#include "TimeUtil.h"
#include "RosLogEntry.h"

#define TRACEF() COMMON::ScopedFuncLogger _funcLogger##__COUNTER__(__PRETTY_FUNCTION__);
#define DEBUG_TIMER()                                                           \
  static COOMON::FuncAccumulatedTimer _timer##__COUNTER__(__PRETTY_FUNCTION__); \
  COMMON::ScopedObject<COMMON::FuncAccumulatedTimer> __scoped(_timer##__COUNTER__);

#define TRACED(...) ROS_DEBUG_STREAM(__VA_ARGS__)
#define TRACEI(...) ROS_INFO_STREAM(__VA_ARGS__)
#define TRACEW(...) ROS_WARN_STREAM(__VA_ARGS__)
#define TRACEE(...) ROS_ERROR_STREAM(__VA_ARGS__)

#define ASSERT_LOGW(x, ...)         \
  do {                              \
    if (!(x)) {                     \
      ROS_WARN_STREAM(__VA_ARGS__); \
    }                               \
  } while (0);

#define ASSERT_LOGE(x, ...)          \
  do {                               \
    if (!(x)) {                      \
      ROS_ERROR_STREAM(__VA_ARGS__); \
    }                                \
  } while (0);

#define BASSERT(x) \
  if (1) {         \
    if (!(x)) {    \
      break;       \
    }              \
  } else {         \
  }

#define BASSERT_LOGW(x, ...)        \
  if (1) {                          \
    if (!(x)) {                     \
      ROS_WARN_STREAM(__VA_ARGS__); \
      break;                        \
    }                               \
  } else {                          \
  }

#define BASSERT_LOGE(x, ...)         \
  if (1) {                           \
    if (!(x)) {                      \
      ROS_ERROR_STREAM(__VA_ARGS__); \
      break;                         \
    }                                \
  } else {                           \
  }

#define CASSERT(x) \
  if (1) {         \
    if (!(x)) {    \
      continue;    \
    }              \
  } else {         \
  }

#define CASSERT_LOGW(x, ...)        \
  if (1) {                          \
    if (!(x)) {                     \
      ROS_WARN_STREAM(__VA_ARGS__); \
      continue;                     \
    }                               \
  } else {                          \
  }

#define CASSERT_LOGE(x, ...)         \
  if (1) {                           \
    if (!(x)) {                      \
      ROS_ERROR_STREAM(__VA_ARGS__); \
      continue;                      \
    }                                \
  } else {                           \
  }

#define RASSERT(x, r) \
  do {                \
    if (!(x)) {       \
      return (r);     \
    }                 \
  } while (0);

#define RASSERT_LOGI(x, r, ...)     \
  do {                              \
    if (!(x)) {                     \
      ROS_INFO_STREAM(__VA_ARGS__); \
      return (r);                   \
    }                               \
  } while (0);

#define RASSERT_LOGW(x, r, ...)     \
  do {                              \
    if (!(x)) {                     \
      ROS_WARN_STREAM(__VA_ARGS__); \
      return (r);                   \
    }                               \
  } while (0);

#define RASSERT_LOGE(x, r, ...)      \
  do {                               \
    if (!(x)) {                      \
      ROS_ERROR_STREAM(__VA_ARGS__); \
      return (r);                    \
    }                                \
  } while (0);

#define RASSERTV(x) \
  do {              \
    if (!(x)) {     \
      return;       \
    }               \
  } while (0);

#define RASSERTV_LOGI(x, ...)       \
  do {                              \
    if (!(x)) {                     \
      ROS_INFO_STREAM(__VA_ARGS__); \
      return;                       \
    }                               \
  } while (0);

#define RASSERTV_LOGW(x, ...)       \
  do {                              \
    if (!(x)) {                     \
      ROS_WARN_STREAM(__VA_ARGS__); \
      return;                       \
    }                               \
  } while (0);

#define RASSERTV_LOGE(x, ...)        \
  do {                               \
    if (!(x)) {                      \
      ROS_ERROR_STREAM(__VA_ARGS__); \
      return;                        \
    }                                \
  } while (0);

#define EASSERT_LOGE(x, ...)         \
  do {                               \
    if (!(x)) {                      \
      ROS_ERROR_STREAM(__VA_ARGS__); \
      abort();                       \
    }                                \
  } while (0);

#define TM_TRACEF(interval)                               \
  do {                                                    \
    static COMMON::TimeoutUtil s_frameTraceVar(interval); \
    if (s_frameTraceVar.Timeout()) {                      \
      TRACEF();                                           \
    }                                                     \
  } while (0);

#define TM_TRACED(interval, ...)                          \
  do {                                                    \
    static COMMON::TimeoutUtil s_frameTraceVar(interval); \
    if (s_frameTraceVar.Timeout()) {                      \
      ROS_DEBUG_STREAM(__VA_ARGS__);                      \
    }                                                     \
  } while (0);

#define TM_TRACEI(interval, ...)                          \
  do {                                                    \
    static COMMON::TimeoutUtil s_frameTraceVar(interval); \
    if (s_frameTraceVar.Timeout()) {                      \
      ROS_INFO_STREAM(__VA_ARGS__);                       \
    }                                                     \
  } while (0);

#define TM_TRACEW(interval, ...)                          \
  do {                                                    \
    static COMMON::TimeoutUtil s_frameTraceVar(interval); \
    if (s_frameTraceVar.Timeout()) {                      \
      ROS_WARN_STREAM(__VA_ARGS__);                       \
    }                                                     \
  } while (0);

#define TM_TRACEE(interval, ...)                          \
  do {                                                    \
    static COMMON::TimeoutUtil s_frameTraceVar(interval); \
    if (s_frameTraceVar.Timeout()) {                      \
      ROS_ERROR_STREAM(__VA_ARGS__);                      \
    }                                                     \
  } while (0);

#define TM_TRACEC(interval, code)                         \
  do {                                                    \
    static COMMON::TimeoutUtil s_frameTraceVar(interval); \
    if (s_frameTraceVar.Timeout()) {                      \
      code;                                               \
    }                                                     \
  } while (0);

/************ default interval is 5 seconds **************/
#define TM_DTRACEF()                                  \
  do {                                                \
    static COMMON::TimeoutUtil s_frameTraceVar(5000); \
    if (s_frameTraceVar.Timeout()) {                  \
      TRACEF();                                       \
    }                                                 \
  } while (0);

#define TM_DTRACED(...)                               \
  do {                                                \
    static COMMON::TimeoutUtil s_frameTraceVar(5000); \
    if (s_frameTraceVar.Timeout()) {                  \
      ROS_DEBUG_STREAM(__VA_ARGS__);                  \
    }                                                 \
  } while (0);

#define TM_DTRACEI(...)                               \
  do {                                                \
    static COMMON::TimeoutUtil s_frameTraceVar(5000); \
    if (s_frameTraceVar.Timeout()) {                  \
      ROS_INFO_STREAM(__VA_ARGS__);                   \
    }                                                 \
  } while (0);

#define TM_DTRACEW(...)                               \
  do {                                                \
    static COMMON::TimeoutUtil s_frameTraceVar(5000); \
    if (s_frameTraceVar.Timeout()) {                  \
      ROS_WARN_STREAM(__VA_ARGS__);                   \
    }                                                 \
  } while (0);

#define TM_DTRACEE(...)                               \
  do {                                                \
    static COMMON::TimeoutUtil s_frameTraceVar(5000); \
    if (s_frameTraceVar.Timeout()) {                  \
      ROS_ERROR_STREAM(__VA_ARGS__);                  \
    }                                                 \
  } while (0);

#define TM_DTRACEC(code)                              \
  do {                                                \
    static COMMON::TimeoutUtil s_frameTraceVar(5000); \
    if (s_frameTraceVar.Timeout()) {                  \
      code;                                           \
    }                                                 \
  } while (0);

#define TM_CASSERT_LOGW(interval, x, ...)       \
  if (1) {                                      \
    if (!(x)) {                                 \
      ROS_WARN_THROTTLE(interval, __VA_ARGS__); \
      continue;                                 \
    }                                           \
  } else {                                      \
  }
