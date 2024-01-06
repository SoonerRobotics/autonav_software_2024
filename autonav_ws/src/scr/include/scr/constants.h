#pragma once
#include <string>

namespace SCR
{
    namespace Constants
    {
        namespace Topics
        {
            const std::string SYSTEM_MODE_SUBSCRIPTION_NAME = "/scr/system_mode";
            const std::string SYSTEM_STATE_SUBSCRIPTION_NAME = "/scr/system_state";
            const std::string DEVICE_STATE_SUBSCRIPTION_NAME = "/scr/device_state";
            const std::string SYSTEM_MODE_CLIENT_NAME = "/scr/system_mode_client";
            const std::string SYSTEM_STATE_CLIENT_NAME = "/scr/system_state_client";
            const std::string DEVICE_STATE_CLIENT_NAME = "/scr/device_state_client";
        }
    }
}