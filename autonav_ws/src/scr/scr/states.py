from enum import IntEnum


class DeviceStateEnum(IntEnum):
    OFF = 0
    BOOTING = 1
    STANDBY = 2
    READY = 3
    OPERATING = 4
    ERRORED = 5


class SystemStateEnum(IntEnum):
    DISABLED = 0
    AUTONOMOUS = 1
    MANUAL = 2
    SHUTDOWN = 3


class SystemModeEnum(IntEnum):
    COMPETITION = 0
    SIMULATION = 1
    PRACTICE = 2