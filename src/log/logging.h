#ifndef __LOG_LOGGING_H_
#define __LOG_LOGGING_H_

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/spdlog.h>

#include <iostream>

#define LOG_DEBUG(...) SPDLOG_LOGGER_DEBUG(spdlog::default_logger_raw(), __VA_ARGS__)
#define LOG_INFO(...) SPDLOG_LOGGER_INFO(spdlog::default_logger_raw(), __VA_ARGS__)
#define LOG_WARN(...) SPDLOG_LOGGER_WARN(spdlog::default_logger_raw(), __VA_ARGS__)
#define LOG_ERROR(...) SPDLOG_LOGGER_ERROR(spdlog::default_logger_raw(), __VA_ARGS__)
#define LOG_CRITICAL(...) SPDLOG_LOGGER_CRITICAL(spdlog::default_logger_raw(), __VA_ARGS__)

class Logger
{
public:
    static Logger& instance();

    template <typename... Args>
    void debug(fmt::format_string<Args...> fmt, Args&&... args)
    {
        try {
            spdlog::debug(fmt, std::forward<Args>(args)...);
        } catch (const spdlog::spdlog_ex& ex) {
            std::cerr << "Write Log Exception: " << ex.what() << std::endl;
        }
    }

    template <typename... Args>
    void info(fmt::format_string<Args...> fmt, Args&&... args)
    {
        try {
            spdlog::info(fmt, std::forward<Args>(args)...);
        } catch (const spdlog::spdlog_ex& ex) {
            std::cerr << "Write Log Exception: " << ex.what() << std::endl;
        }
    }

    template <typename... Args>
    void warn(fmt::format_string<Args...> fmt, Args&&... args)
    {
        try {
            spdlog::warn(fmt, std::forward<Args>(args)...);
        } catch (const spdlog::spdlog_ex& ex) {
            std::cerr << "Write Log Exception: " << ex.what() << std::endl;
        }
    }

    template <typename... Args>
    void error(fmt::format_string<Args...> fmt, Args&&... args)
    {
        try {
            spdlog::error(fmt, std::forward<Args>(args)...);
        } catch (const spdlog::spdlog_ex& ex) {
            std::cerr << "Write Log Exception: " << ex.what() << std::endl;
        }
    }

private:
    Logger();
};

class Logging
{
public:
    template <typename... Args>
    static void debug(fmt::format_string<Args...> fmt, Args&&... args)
    {
        Logger::instance().debug(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void info(fmt::format_string<Args...> fmt, Args&&... args)
    {
        Logger::instance().info(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void warn(fmt::format_string<Args...> fmt, Args&&... args)
    {
        Logger::instance().warn(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void error(fmt::format_string<Args...> fmt, Args&&... args)
    {
        Logger::instance().error(fmt, std::forward<Args>(args)...);
    }
};

#endif  // __LOG_LOGGING_H_
