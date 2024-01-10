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
}