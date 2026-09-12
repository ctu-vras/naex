#pragma once

#include <chrono>

namespace naex {

class Timer {
private:
  typedef std::chrono::steady_clock Clock;
  typedef Clock::time_point Time;
  typedef std::chrono::duration<double> Duration;
  Time start;

public:
  Timer() : start(Clock::now()) {}
  explicit Timer(const Time &s) : start(s) {}
  Timer(const Timer &s) : start(s.start) {}
  Timer &operator=(const Timer &s) {
    start = s.start;
    return *this;
  }
  void reset() { start = Clock::now(); }
  double seconds_elapsed() const {
    return std::chrono::duration_cast<Duration>(Clock::now() - start).count();
  }
};

} // namespace naex
