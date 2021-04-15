#ifndef TIMEUTIL_H_
#define TIMEUTIL_H_

#include <chrono>
#include <string>
#include <sys/time.h>

namespace COMMON {
;

class TimeUtil {
  TimeUtil(const TimeUtil&) = delete;
  TimeUtil& operator=(const TimeUtil&) = delete;

 private:
  TimeUtil() = default;
  ~TimeUtil() = default;

 public:
  static unsigned long long GetCurrentTimestamp() {
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp =
        std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    return static_cast<unsigned long long>(tp.time_since_epoch().count());
  }
  static unsigned long long GetCurrentTimestampMicro() {
    std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds> tp =
        std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
    return static_cast<unsigned long long>(tp.time_since_epoch().count());
  }

  static unsigned long long GetTimestampMs() {
    timeval tv{};
    gettimeofday(&tv, nullptr);
    return static_cast<unsigned long long>(tv.tv_sec * 1000) + static_cast<unsigned long long>(tv.tv_usec / 1000);
  }

  static unsigned long long GetTimestampUs() {
    timeval tv{};
    gettimeofday(&tv, nullptr);
    return static_cast<unsigned long long>(tv.tv_sec * 1000000) + static_cast<unsigned long long>(tv.tv_usec);
  }

  static double GetTimestampS() {
    timeval tv{};
    gettimeofday(&tv, nullptr);
    return static_cast<double>(tv.tv_sec) + static_cast<double>(tv.tv_usec) / 1e6;
  }

  static double TimeSSince(unsigned long long last_time_ms) {
    return static_cast<double>(GetTimestampMs() - last_time_ms) / 1e3;
  }

  static std::string GetDateStr() {
    time_t rawtime;
    struct tm* timeinfo;
    char buffer[256] = {0};

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
    return buffer;
  }
};

}  // namespace COMMON

#endif
