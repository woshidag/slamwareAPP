/*
* base_messages.h
* Base messages
*
* Was in i_base_device.h
* Created by Tony Huang (tony@slamtec.com) at 2016-11-15
* Copyright 2016 (c) Shanghai Slamtec Co., Ltd.
*/


#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <rpos/core/geometry.h>

namespace rpos { namespace message { namespace base {

    enum BaseMotionModel
    {
        BaseMotionModelTwoWheelDifferential,
        BaseMotionModelThreeOmniWheel,
        // BaseMotionModelFourMcNameeWheel,
        // BaseMotionModelSyncTurnWheel
    };

    struct BaseInfo
    {
        std::uint32_t manufactureId;
        std::uint32_t productId;
        std::string manufactureName;
        std::string productName;
        std::string firmwareVersion;
        std::string hardwareVersion;
        std::string serialNumber;
    };

    struct MotionRequest
    {
        rpos::core::Vector2f linearSpeed;       // the unit is: m/s
        float angularSpeed;                     // the unit is: rad/s

                                                // both in right hand system, the head direction of the robot is x axis
                                                // the y axis point at the left of the robot
    };

    struct MovementEstimation
    {
        rpos::core::Vector2f positionDifference;    // the unit is: m
        float angularDifference;                    // the unit is: rad
#if 0
        rpos::core::Vector2f linearSpeed;           // the unit is: m/s
        float angularSpeed;                         // the unit is: rad/s
        rpos::core::Vector2f linearAcc;             // the unit is: m/s^2
        float angularAcc;                           // the unit is: rad/s^2
#endif
    };

    enum BaseErrorLevel
    {
        BaseErrorLevelWarn,
        BaseErrorLevelError,
        BaseErrorLevelFatal,
        BaseErrorLevelUnknown = 255
    };

    enum BaseErrorComponent
    {
        BaseErrorComponentUser,
        BaseErrorComponentSystem,
        BaseErrorComponentPower,
        BaseErrorComponentMotion,
        BaseErrorComponentSensor,
        BaseErrorComponentUnknown = 255
    };

    struct BaseError
    {
        BaseError()
            : id(0), errorCode(0), level(BaseErrorLevelWarn)
            , component(BaseErrorComponentSystem), componentErrorCode(0)
        {
        }

        int id;
        std::uint32_t errorCode;
        BaseErrorLevel level;
        BaseErrorComponent component;
        std::uint16_t componentErrorCode;
        std::string message;
    };

    struct BaseHealthInfo
    {
        BaseHealthInfo()
            : hasWarning(false), hasError(false), hasFatal(false)
        {}

        bool hasWarning;
        bool hasError;
        bool hasFatal;

        std::vector<BaseError> errors;
    };

    struct AuxiliaryAnchorInfo
    {
        AuxiliaryAnchorInfo() {
            id = 0;
            distance_mm = 0;
            max_error_mm = 0;
        };
        uint16_t id;
        uint16_t distance_mm;
        uint8_t max_error_mm; // OPTIONAL, only for devices support error measurement
    };

    enum AuxiliaryAnchorType {
        AuxiliaryAnchorTypeUWB = 0,
    };

    struct AuxiliaryAnchor
    {
        AuxiliaryAnchor() {
            type = AuxiliaryAnchorTypeUWB;
            anchors_data.clear();
        };
        AuxiliaryAnchorType type;
        std::vector<AuxiliaryAnchorInfo> anchors_data;
    };


} } }
