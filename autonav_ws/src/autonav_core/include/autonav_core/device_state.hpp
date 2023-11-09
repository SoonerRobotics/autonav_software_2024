namespace SCR {
    enum DeviceState {
        OFF = 0,
        STANDBY = 1,
        READY = 2,
        OPERATING = 3
    };

    struct Device {
        std::string name;
        DeviceState state;
    };
}