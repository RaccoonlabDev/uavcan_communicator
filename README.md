# Uavcan communicator

This package suggests a bridge between the PX4 drone on the UAVCAN bus on one side and the ROS topics on the other side.

It has minimal sensors set. Topics are:

| № | Direction   | ROS topic                  | ROS msg                               | UAVCAN msg                                     |
| - | ----------- | -------------------------- | ------------------------------------- |----------------------------------------------- |
| 1 | UAVCAN->ROS | /uav/actuators             | sensor_msgs/Joy                       | uavcan::equipment::esc::RawCommand             |
| 2 | ROS->UAVCAN | /uav/static_temperature    | drone_communicators/StaticTemperature | uavcan::equipment::air_data::StaticTemperature |
| 3 | ROS->UAVCAN | /uav/static_pressure       | drone_communicators/StaticPressure    | uavcan::equipment::air_data::StaticPressure    |
| 4 | ROS->UAVCAN | /uav/raw_air_data          | drone_communicators/RawAirData        | uavcan::equipment::air_data::RawAirData        |
| 5 | ROS->UAVCAN | /uav/gps_position          | drone_communicators/Fix               | uavcan::equipment::gnss::Fix                   |
| 6 | ROS->UAVCAN | /uav/imu                   | sensor_msgs/Imu                       | uavcan::equipment::ahrs::RawIMU                |
| 7 | ROS->UAVCAN | /uav/mag                   | sensor_msgs/MagneticField             | uavcan::equipment::ahrs::MagneticFieldStrength |

# Preparation

1. Install libuavcan v0.1

2. Clone this repository using recursive. Update submodules every time you pull this repo:

```
git clone --recursive https://github.com/InnopolisAero/innopolis_vtol_dynamics.git .
git submodule init
git submodule update --recursive
```

# Running

If you has Innopolis sniffer, use scipt `./scripts/create_slcan.sh`.
Otherwise you should use `./scripts/create_slcan.sh /dev/ttyACMx`, where `x` is index of your tty device.