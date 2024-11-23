# DroneCAN communicator [![Build Status](https://build.ros.org/job/Ndev__uavcan_communicator__ubuntu_focal_amd64/badge/icon)](https://build.ros.org/job/Ndev__uavcan_communicator__ubuntu_focal_amd64/) [![Build Status](https://build.ros.org/job/Ndev_db__uavcan_communicator__debian_buster_amd64/badge/icon)](https://build.ros.org/job/Ndev_db__uavcan_communicator__debian_buster_amd64/) [![Build Status](https://build.ros.org/job/Mdev__uavcan_communicator__ubuntu_bionic_amd64/badge/icon)](https://build.ros.org/job/Mdev__uavcan_communicator__ubuntu_bionic_amd64/)

> It is recommended to use [Cyphal communicator](https://github.com/RaccoonlabDev/cyphal_communicator) in new designs.

DroneCAN communicator converts DroneCAN messages to ROS and vice versa.

It covers a minimal set of sensors required for such applications as Ardupilot/PX4 DroneCAN HITL simulation. This communicator can be used for other purposes as well.

## 1. Conversions

The tables below represent the supported conversions:

**DroneCAN->ROS**

| № | Brige name |ROS msg                               | DroneCAN msg                                     |
| - | ---------- | ------------------------------------ |----------------------------------------------- |
| 1 | RawCommandUavcanToRos | [sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html)                       | [esc::RawCommand](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#rawcommand)             |
| 2 | ArrayCommandUavcanToRos | [sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html)                       | [actuator::ArrayCommand](https://dronecan.github.io/Specification/7._List_of_standard_data_types/#arraycommand)             |
| 3 | ArmUavcanToRos         | [std_msgs::Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html)             | [safety::ArmingStatus](https://dronecan.github.io/Specification/7._List_of_standard_data_types/#armingstatus) |
| 4 | AhrsSolutionUavcanToRos | [sensor_msgs::Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)             | [ahrs::AhrsSolution](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#solution) |
| 5 | EscStatusUavcanToRos | [mavros_msgs::ESCTelemetryItem](http://docs.ros.org/en/api/mavros_msgs/html/msg/ESCTelemetryItem.html) | [esc::Status](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#status-2) |
| 6 | CircuitStatusUavcanToRos | [mavros_msgs::BatteryStatus](http://docs.ros.org/en/api/mavros_msgs/html/msg/BatteryStatus.html) | [power::CircuitStatus](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#circuitstatus) |

**ROS->DroneCAN**

| № | Brige name | ROS msg                               | DroneCAN msg                                     |
| - | ---------- | ------------------------------------- |----------------------------------------------- |
| 1 | BaroStaticTemperatureRosToUavcan | [std_msgs/Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html) | [air_data::StaticTemperature](https://dronecan.github.io/Specification/7._List_of_standard_data_types/#statictemperature) |
| 2 | BaroStaticPressureRosToUavcan | [std_msgs/Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html) | [air_data::StaticPressure](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#staticpressure)    |
| 3 | DiffPressureRosToUavcan | [std_msgs/Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html) | [air_data::RawAirData](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#rawairdata)        |
| 4 | GpsRosToUavcan | [sensor_msgs/NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html) | [gnss::Fix2](https://dronecan.github.io/Specification/7._List_of_standard_data_types/#fix2)                   |
| | | [geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) | [gnss::Fix2](https://dronecan.github.io/Specification/7._List_of_standard_data_types/#fix2)                   |
| 5 | ImuRosToUavcan | [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)                       | [ahrs::RawIMU](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#rawimu)                |
| 6 | MagnetometerRosToUavcan | [sensor_msgs/MagneticField](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/MagneticField.html)             | [ahrs::MagneticFieldStrength](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#magneticfieldstrength) |
| 7 | EscStatusRosToUavcan | [mavros_msgs::ESCTelemetryItem](http://docs.ros.org/en/api/mavros_msgs/html/msg/ESCTelemetryItem.html) | [esc::Status](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#status-2) |
| 8 | IceReciprocatingStatusRosToUavcan | [mavros_msgs::ESCStatusItem](http://docs.ros.org/en/api/mavros_msgs/html/msg/ESCStatusItem.html) | [ice::reciprocating::Status](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#status-4) |
| 9 | IceFuelTankStatusRosToUavcan | [std_msgs/UInt8](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/UInt8.html) | [ice::FuelTankStatus](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#fueltankstatus) |
| 10 | BatteryInfoRosToUavcan | [sensor_msgs/BatteryState](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/BatteryState.html)             | [power::BatteryInfo](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#batteryinfo) |

## 2. Preparation

You need to perform the following steps to use this package:

1. Clone this repository using recursive. Update submodules every time you pull this repo
2. Install required packages using `install_requirements.sh` script
3. Build [libuavcan v0.1](https://github.com/UAVCAN/libuavcan/tree/legacy-v0#using-in-a-gnulinux-application) as a static library and install it on the system globally
4. (optionally) Build DSDL if you want to use custom messages

```
cd catkin_ws/src
git clone --recursive git@github.com:RaccoonlabDev/uavcan_communicator.git
cd uavcan_communicator
git submodule update --init --recursive
./scripts/install.sh
```

## 3. Running

1. At first, you need to create a virtual can port
2. Then specify in `config/params.yaml` which conversions do you need to use
3. Then launch communicator typing:

Example:
```
roslaunch uavcan_communicator example_communicator.launch
```

## 4. Usage example

Below you can see an example of using the uavcan_communicator in conjunction with a VTOL dynamics simulator.

[![uavcan vtol dynamics simulator](https://img.youtube.com/vi/JmElAwgAoSc/0.jpg)](https://youtu.be/JmElAwgAoSc)
