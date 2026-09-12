#pragma once

#include <stdexcept>
#include <string>

namespace naex {

/**
 * Runtime error with an optional diagnostic trace.
 *
 * The ROS 1 version captured a boost::stacktrace here. Boost.Stacktrace needs
 * libbacktrace/libdl to be linked explicitly, which the package does not do,
 * so the trace is left empty and the accessor kept for source compatibility.
 */
class Exception : public std::runtime_error {
public:
  explicit Exception(const char *what) : std::runtime_error(what) {}
  explicit Exception(const std::string &what) : std::runtime_error(what) {}

  const std::string &stacktrace() const { return stacktrace_; }

private:
  std::string stacktrace_;
};

class NotInitialized : public Exception {
public:
  explicit NotInitialized(const char *what) : Exception(what) {}
};

} // namespace naex
