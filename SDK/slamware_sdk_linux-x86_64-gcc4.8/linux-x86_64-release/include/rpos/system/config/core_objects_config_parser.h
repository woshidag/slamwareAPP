#pragma once

#include "config_parser.h"
#include <rpos/core/pose.h>
#include <rpos/core/geometry.h>
#include <rpos/system/util/log.h>

namespace rpos { namespace system { namespace config {

    template <>
    struct RPOS_CORE_API ConfigParser < core::Pose > {
        static bool parse(const Json::Value& config, core::Pose& that);
    };

    template <>
    struct RPOS_CORE_API ConfigParser < core::RectangleF > {
        static bool parse(const Json::Value& config, core::RectangleF& that);
    };

    template <>
    struct RPOS_CORE_API ConfigParser < util::LogConfig > {
        static bool parse(const Json::Value& config, util::LogConfig& that);
    };

    template <>
    struct RPOS_CORE_API ConfigParser < util::LogAppenderConfig > {
        static bool parse(const Json::Value& config, util::LogAppenderConfig& that);
    };

    template <>
    struct RPOS_CORE_API ConfigParser < util::FileLogAppenderConfig > {
        static bool parse(const Json::Value& config, util::FileLogAppenderConfig& that);
    };

    template <>
    struct RPOS_CORE_API ConfigParser < util::DiagnosisPublisherLogAppenderConfig > {
        static bool parse(const Json::Value& config, util::DiagnosisPublisherLogAppenderConfig& that);
    };

    template <>
    struct RPOS_CORE_API ConfigParser < util::LogLevel > {
        static bool parse(const Json::Value& config, util::LogLevel& that);
    };

} } }
