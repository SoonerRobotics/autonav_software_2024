#pragma once
#include <cstdint>
#include <string>

namespace SCR
{
    enum DeviceState : uint8_t
    {
        OFF = 0,
        BOOTING = 1,
        STANDBY = 2,
        READY = 3,
        OPERATING = 4,
        ERRORED = 5
    };

    std::string toString(DeviceState state)
    {
        switch (state)
        {
        case DeviceState::OFF:
            return "Off";
        case DeviceState::BOOTING:
            return "Booting";
        case DeviceState::STANDBY:
            return "Standby";
        case DeviceState::READY:
            return "Ready";
        case DeviceState::OPERATING:
            return "Operating";
        case DeviceState::ERRORED:
            return "Errored";
        }
        return "Unknown";
    }

    enum SystemState : uint8_t
    {
        DISABLED = 0,
        AUTONOMOUS = 1,
        MANUAL = 2,
        SHUTDOWN = 3
    };

    std::string toString(SystemState state)
    {
        switch (state)
        {
        case SystemState::DISABLED:
            return "Disabled";
        case SystemState::AUTONOMOUS:
            return "Autonomous";
        case SystemState::MANUAL:
            return "Manual";
        case SystemState::SHUTDOWN:
            return "Shutdown";
        }
        return "Unknown";
    }

    enum SystemMode : uint8_t
    {
        COMPETITION = 0,
        SIMULATION = 1,
        PRACTICE = 2,
    };

    std::string toString(SystemMode state)
    {
        switch (state)
        {
        case SystemMode::COMPETITION:
            return "Competition";
        case SystemMode::SIMULATION:
            return "Simulation";
        case SystemMode::PRACTICE:
            return "Practice";
        }
        return "Unknown";
    }
}