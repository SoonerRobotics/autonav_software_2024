#pragma once

namespace SCR
{
    namespace Structs
    {
        struct ChangeDeviceStateRequest
        {
            char device_name[32];
            int32_t device_state;
        };

        struct ChangeDeviceStateResponse
        {
            bool success;
        };

        struct ChangeSystemStateRequest
        {
            int32_t system_state;
        };

        struct ChangeSystemStateResponse
        {
            bool success;
        };

        struct ChangeSystemModeRequest
        {
            int32_t system_mode;
        };

        struct ChangeSystemModeResponse
        {
            bool success;
        };
    }
}