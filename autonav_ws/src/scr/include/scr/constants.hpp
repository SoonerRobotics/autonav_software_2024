#pragma once
#include <string>

namespace SCR
{
    namespace Constants
    {
        namespace Topics
        {
            const std::string SYSTEM_STATE = "/scr/system_state";
            const std::string DEVICE_STATE = "/scr/device_state";
            const std::string CONFIG_UPDATE = "/scr/config_updated";
            const std::string PERFORMANCE_TRACK = "/scr/performance";
        }

        namespace Services
        {
            const std::string SYSTEM_STATE = "/scr/system_state_client";
            const std::string DEVICE_STATE = "/scr/device_state_client";
            const std::string CONFIG_UPDATE = "/scr/update_config_client";
        }
    }

    namespace DZ
    {
        namespace Topics
        {
            const std::string POSITION = "/autonav/position";
            const std::string MOTOR_FEEDBACK = "/autonav/MotorFeedback";
            const std::string MOTOR_INPUT = "/autonav/MotorInput";
            const std::string GPS_FEEDBACK = "/autonav/GPSFeedback";
            const std::string IMU_DATA = "/autonav/imu";
            const std::string CONBUS_DATA = "/autonav/conbus/data";
            const std::string CONBUS_CONTROL = "/autonav/conbus/instruction";
            const std::string CAMERA_RAW = "/autonav/camera/compressed";
            const std::string CAMERA_FILTERED = "/autonav/cfg_space/raw/image";
            const std::string CAMERA_ASTAR_DEBUG = "/autonav/debug/astar/image";
        }
    }
}