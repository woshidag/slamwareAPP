/**
* feature.h
* The System Resource feature
*
* Created By Jacky Li @ 2014-7-1
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <map>

#include <rpos/rpos_config.h>
#include <rpos/core/feature.h>
#include <rpos/core/pose.h>

#include "system_parameters.h"
#include "laser_scan.h"
#include "device_info.h"
#include "device_health.h"
#include "power_management.h"
#include "../../message/depth_camera_messages.h"

namespace rpos { namespace features {

    namespace system_resource {

        enum RestartMode
        {
            RestartModeSoft,
            RestartModeHard
        };
        
        enum NetworkMode {
            NetworkModeAP,
            NetworkModeStation,
            NetworkModeWifiDisabled,
            NetworkModeDHCPDisabled,
            NetworkModeDHCPEnabled
        };

        struct NetworkStatus
        {
            std::map<std::string, std::string> options;
        };
        typedef std::uint32_t HeartBeatToken;
        const std::uint32_t MAX_CONCURRENT_HEART_BEAT = 1;
    }

    namespace detail {
        class SystemResourceImpl;
    }

    class RPOS_CORE_API SystemResource : public rpos::core::Feature{
    public:
        typedef detail::SystemResourceImpl impl_t;

        RPOS_OBJECT_CTORS_WITH_BASE(SystemResource, rpos::core::Feature);
        SystemResource(boost::shared_ptr<detail::SystemResourceImpl> impl);
        ~SystemResource();

    public:
        int getBatteryPercentage();
        bool getBatteryIsCharging();
        bool getDCIsConnected();
        system_resource::PowerStatus getPowerStatus();
        void wakeUp();

        int getBoardTemperature();
        std::string getSDPVersion();
        system_resource::LaserScan getLaserScan();
        bool restartModule(system_resource::RestartMode mode = system_resource::RestartModeSoft);
        bool setSystemParameter(const std::string& param, const std::string& value);
        std::string getSystemParameter(const std::string& param);
        system_resource::DeviceInfo getDeviceInfo();
        rpos::features::system_resource::BaseHealthInfo getRobotHealth();
        void clearRobotHealth(int errorCode);
        bool configurateNetwork(rpos::features::system_resource::NetworkMode mode, const std::map<std::string, std::string>& options);
        std::map<std::string, std::string> getNetworkStatus();
        system_resource::HeartBeatToken startHeartBeat(int heartBeatTimeoutInSeconds);
        void refreshHeartBeat(system_resource::HeartBeatToken token);
        void stopHeartBeat(system_resource::HeartBeatToken token);
        void voiceRespond();
        void startFirmwareUpgrade(const std::string& filename);
        void startFirmwareUpgrade(const std::vector<uint8_t>& firmwareContent);
        void publishDepthCamFrame(int sensorId, const rpos::message::depth_camera::DepthCameraFrame& frame);
    };

} }
