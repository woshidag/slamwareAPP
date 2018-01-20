/*
* log.h
* RPOS Log System
*
* Created By Tony Huang (tony@slamtec.com) at 2014-12-29
* Copyright 2014 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/rpos_core_config.h>
#include <rpos/system/target_info.h>
#include <stdarg.h>
#include <string>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem/path.hpp>
#include <list>

#ifdef RPOS_TARGET_64BIT
#   define RPOS_LOG_POINTER_FMT "%016llx"
#else
#   define RPOS_LOG_POINTER_FMT "%08x"
#endif

namespace rpos { namespace system { namespace util {

    enum LogLevel {
        LogLevelDebug,
        LogLevelInfo,
        LogLevelWarn,
        LogLevelError,
        LogLevelFatal
    };
	
	namespace diagnosis {
		struct LogData
		{
			std::string logSource;
			rpos::system::util::LogLevel logLevel;
			std::string logMessage;
		};
	}

    struct LogAppenderConfig {
        LogLevel logLevel;
        std::vector<std::string> excludeLogSources;
        std::vector<std::string> includeLogSources;
    };

    struct FileLogAppenderConfig : public LogAppenderConfig {
        std::string filename;
    };

    struct DiagnosisPublisherLogAppenderConfig: public LogAppenderConfig {
        std::string topic;
    };

    struct LogConfig {
        LogAppenderConfig global;
        LogAppenderConfig console;
        std::vector<FileLogAppenderConfig> files;
        std::vector<DiagnosisPublisherLogAppenderConfig> diagnosis_publishers;
    };

    RPOS_CORE_API void vlog(const char* source, LogLevel level, const char* msg, va_list args);
    RPOS_CORE_API void log(const char* source, LogLevel level, const char* msg, ...);

    RPOS_CORE_API void vdebug_out(const char* source, const char* msg, va_list args);
    RPOS_CORE_API void  debug_out(const char* source, const char* msg, ...);

    RPOS_CORE_API void vinfo_out(const char* source, const char* msg, va_list args);
    RPOS_CORE_API void  info_out(const char* source, const char* msg, ...);

    RPOS_CORE_API void vwarn_out(const char* source, const char* msg, va_list args);
    RPOS_CORE_API void  warn_out(const char* source, const char* msg, ...);

    RPOS_CORE_API void verror_out(const char* source, const char* msg, va_list args);
    RPOS_CORE_API void  error_out(const char* source, const char* msg, ...);

    RPOS_CORE_API void vfatal_out(const char* source, const char* msg, va_list args);
    RPOS_CORE_API void  fatal_out(const char* source, const char* msg, ...);

    class RPOS_CORE_API LogScope : private boost::noncopyable {
    public:
        LogScope(const std::string& source);
        ~LogScope();

    public:
        void vlog(LogLevel level, const char* msg, va_list args);
        void log(LogLevel level, const char* msg, ...);

        void vdebug_out(const char* msg, va_list args);
        void  debug_out(const char* msg, ...);

        void vinfo_out(const char* msg, va_list args);
        void  info_out(const char* msg, ...);

        void vwarn_out(const char* msg, va_list args);
        void  warn_out(const char* msg, ...);

        void verror_out(const char* msg, va_list args);
        void  error_out(const char* msg, ...);

        void vfatal_out(const char* msg, va_list args);
        void  fatal_out(const char* msg, ...);

    public:
        std::string getSource() const;
        void setSource(const std::string& source);

    private:
        mutable boost::mutex lock_;
        std::string source_;
    };

    class RPOS_CORE_API LogAppender : private boost::noncopyable {
    public:
        typedef boost::shared_ptr<LogAppender> Pointer;

    protected:
        LogAppender(LogLevel logLevel = LogLevelDebug);

    public:
        virtual ~LogAppender();

    public:
        LogLevel getLogLevel() const;
        void setLogLevel(LogLevel logLevel);

        void append(const std::string& logSource, LogLevel logLevel, const std::string& message);

        bool isExcluded(const std::string& logSource, LogLevel logLevel);

        bool isIncluded(const std::string& logSource);

        std::vector<std::string>& excludeSources();

        std::vector<std::string>& includeSources();

    protected:
        virtual void append_(const std::string& logSource, LogLevel logLevel, const std::string& message) = 0;

    private:
        LogLevel logLevel_;
        std::vector<std::string> excludeSources_;
        std::vector<std::string> includeSources_;
    };

    class RPOS_CORE_API LogManager : private boost::noncopyable {
    public:
        typedef boost::shared_ptr<LogAppender> AppenderPointer;
        typedef boost::shared_ptr<LogManager> ManagerPointer;

    private:
        LogManager();

    public:
        ~LogManager();

        void append(const std::string& logSource, LogLevel logLevel, const std::string& message);

        bool isExcluded(const std::string& logSource, LogLevel logLevel);
        bool isIncluded(const std::string& logSource);

        void addAppender(AppenderPointer appender);
        void removeAppender(AppenderPointer appender);
        void clearAppenders();

    public:
        // Helper functions
        void addConsoleAppender();
        void addFileAppender(const std::string& filename);

        void configWith(const LogConfig& config);

    public:
        static ManagerPointer defaultManager();

    private:
        static void createManager_();

    private:
        boost::mutex lock_;
        std::list<AppenderPointer> appenders_;
        LogAppenderConfig globalLogSettings_;
    };

    class RPOS_CORE_API ConsoleLogAppender : public LogAppender {
    public:
        ConsoleLogAppender(LogLevel logLevel = LogLevelDebug);
        ~ConsoleLogAppender();

    protected:
        virtual void append_(const std::string& logSource, LogLevel logLevel, const std::string& message);

    private:
        boost::mutex lock_;
    };

    class RPOS_CORE_API FileLogAppender : public LogAppender {
    public:
        FileLogAppender(const std::string& filename, LogLevel logLevel = LogLevelDebug);
        explicit FileLogAppender(const std::wstring& wfilename, LogLevel logLevel = LogLevelDebug, bool addUtf8Bom = false);
        ~FileLogAppender();

    public:
        bool isAvailable() const;

    protected:
        virtual void append_(const std::string& logSource, LogLevel logLevel, const std::string& message);

    private:
        boost::filesystem::path filename_;
        FILE* file_;
        mutable boost::mutex lock_;
    };

    class RPOS_CORE_API DiagnosisPublisherLogAppender: public LogAppender
    {
    public:
        explicit DiagnosisPublisherLogAppender(const std::string& topic, LogLevel logLevel = LogLevelDebug);
        virtual ~DiagnosisPublisherLogAppender();

    public:
        const std::string& getTopic() const { return topic_; }

    protected:
        virtual void append_(const std::string& logSource, LogLevel logLevel, const std::string& message);

    private:
        const std::string topic_;
    };

} } }
