#include "kstone/core/log.hpp"

#include "kstone/core/clock.hpp"

#include <cstdio>
#include <mutex>
#include <utility>

namespace kstone {

const char* to_string(LogLevel level) noexcept {
    switch (level) {
        case LogLevel::trace:
            return "trace";
        case LogLevel::debug:
            return "debug";
        case LogLevel::info:
            return "info";
        case LogLevel::warn:
            return "warn";
        case LogLevel::error:
            return "error";
        case LogLevel::off:
            return "off";
    }
    return "unknown";
}

LogSink::~LogSink() = default;

namespace {

class StderrSink final : public LogSink {
  public:
    void write(const LogRecord& r) noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        std::fprintf(stderr, "[kstone %s] %.*s: %.*s\n", to_string(r.level),
                     static_cast<int>(r.component.size()), r.component.data(),
                     static_cast<int>(r.message.size()), r.message.data());
    }

  private:
    std::mutex mutex_;
};

class NullSink final : public LogSink {
  public:
    void write(const LogRecord&) noexcept override {}
};

class CallbackSink final : public LogSink {
  public:
    explicit CallbackSink(std::function<void(const LogRecord&)> fn) : fn_(std::move(fn)) {}
    void write(const LogRecord& r) noexcept override {
        if (!fn_) return;
        try {
            fn_(r);
        } catch (...) {
        }
    }

  private:
    std::function<void(const LogRecord&)> fn_;
};

}  // namespace

std::shared_ptr<LogSink> make_stderr_sink() { return std::make_shared<StderrSink>(); }
std::shared_ptr<LogSink> make_null_sink() { return std::make_shared<NullSink>(); }
std::shared_ptr<LogSink> make_callback_sink(std::function<void(const LogRecord&)> fn) {
    return std::make_shared<CallbackSink>(std::move(fn));
}

Logger::Logger() : sink_(make_null_sink()), level_(LogLevel::off) {}

Logger::Logger(std::shared_ptr<LogSink> sink, LogLevel level, std::shared_ptr<const Clock> clock)
    : sink_(sink ? std::move(sink) : make_null_sink()), clock_(std::move(clock)), level_(level) {}

void Logger::log(LogLevel level, std::string_view component, std::string_view message) const noexcept {
    if (!enabled(level)) return;
    sink_->write(LogRecord{level, component, message, clock_ ? clock_->now_ns() : 0});
}

}  // namespace kstone
