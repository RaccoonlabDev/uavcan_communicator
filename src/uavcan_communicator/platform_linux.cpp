/**
* @file platform_linux.cpp
* @note from uavcan demo
*/

#include <uavcan_linux/uavcan_linux.hpp>

uavcan::ISystemClock& getSystemClock() {
    static uavcan_linux::SystemClock clock;
    return clock;
}

uavcan::ICanDriver& getCanDriver() {
    static uavcan_linux::SocketCanDriver driver(dynamic_cast<const uavcan_linux::SystemClock&>(getSystemClock()));
    if (driver.getNumIfaces() == 0) {
        if (driver.addIface("slcan0") < 0) {
            throw std::runtime_error("Failed to add iface");
        }
    }
    return driver;
}
