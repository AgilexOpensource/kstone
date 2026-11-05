#pragma once

#include "kstone/core/export.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string_view>

namespace kstone {

class Clock;

enum class LogLevel : int { trace = 0, debug = 1, info = 2, warn = 3, error = 4, off = 5 };

KSTONE_EXPORT const char* to_string(LogLevel level) noexcept;

struct LogRecord {
    LogLevel level;
    std::string_view component;  ///< e.g. "model.urdf"; valid only during write().
    std::string_view message;    ///< valid only during write().
    std::int64_t time_ns;        ///< from the logger's clock.
};

/// Destination for log records. Implementations must be thread-safe and must not throw.
class KSTONE_EXPORT LogSink {
  public:
    virtual ~LogSink();
    virtual void write(const LogRecord& record) noexcept = 0;
};

KSTONE_EXPORT std::shared_ptr<LogSink> make_stderr_sink();
KSTONE_EXPORT std::shared_ptr<LogSink> make_null_sink();
/// Adapts any callable; exceptions escaping the callable are swallowed.
KSTONE_EXPORT std::shared_ptr<LogSink> make_callback_sink(std::function<void(const LogRecord&)> fn);

/// Cheap, copyable front-end: level filter + sink + clock.
class KSTONE_EXPORT Logger {
  public:
    Logger();  ///< null sink, level off.
    Logger(std::shared_ptr<LogSink> sink, LogLevel level, std::shared_ptr<const Clock> clock = nullptr);

    bool enabled(LogLevel level) const noexcept { return level >= level_ && level_ != LogLevel::off; }
    void log(LogLevel level, std::string_view component, std::string_view message) const noexcept;

    LogLevel level() const noexcept { return level_; }
    void set_level(LogLevel level) noexcept { level_ = level; }
    const std::shared_ptr<LogSink>& sink() const noexcept { return sink_; }

  private:
    std::shared_ptr<LogSink> sink_;
    std::shared_ptr<const Clock> clock_;
    LogLevel level_;
};

}  // namespace kstone
