# DroneCAN communicator

DroneCAN communicator converts DroneCAN messages to ROS and vice versa.

It covers a minimal set of sensors required for such applications as Ardupilot/PX4 DroneCAN HITL simulation. This communicator can be used for other purposes as well.

> It is recommended to use [Cyphal communicator](https://github.com/RaccoonlabDev/cyphal_communicator) in new designs.

## 1. Conversions

The tables below represent the supported conversions:

**DroneCAN->ROS**

| № | Brige name |ROS msg                               | DroneCAN msg                                     |
| - | ---------- | ------------------------------------ |----------------------------------------------- |
| 1 | ActuatorsUavcanToRos  | [sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html)                       | [esc::RawCommand](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#rawcommand)             |
| 2 | ArmUavcanToRos         | [std_msgs::Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html)             | [esc::RawCommand](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#rawcommand) |
| 3 | AhrsSolutionUavcanToRos | [sensor_msgs::Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)             | [ahrs::AhrsSolution](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#solution) |
| 4 | EscStatusUavcanToRos | uavcan_msgs::EscStatus | [esc::Status](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#status-2) |
| 5 | CircuitStatusUavcanToRos | uavcan_msgs::CircuitStatus | [power::CircuitStatus](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#circuitstatus) |

**ROS->DroneCAN**

| № | Brige name | ROS msg                               | DroneCAN msg                                     |
| - | ---------- | ------------------------------------- |----------------------------------------------- |
| 1 | BaroStaticTemperatureRosToUavcan | uavcan_msgs/StaticTemperature | [air_data::StaticTemperature](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#statictemperature) |
| 2 | BaroStaticPressureRosToUavcan | uavcan_msgs/StaticPressure    | [air_data::StaticPressure](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#staticpressure)    |
| 3 | DiffPressureRosToUavcan | uavcan_msgs/RawAirData        | [air_data::RawAirData](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#rawairdata)        |
| 4 | GpsRosToUavcan | uavcan_msgs/Fix               | [gnss::Fix](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#fix)                   |
| 5 | ImuRosToUavcan | [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)                       | [ahrs::RawIMU](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#rawimu)                |
| 6 | MagnetometerRosToUavcan | [sensor_msgs/MagneticField](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/MagneticField.html)             | [ahrs::MagneticFieldStrength](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#magneticfieldstrength) |
| 7 | EscStatusRosToUavcan | uavcan_msgs/EscStatus             | [esc::Status](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#status-2) |
| 8 | IceReciprocatingStatusRosToUavcan | uavcan_msgs/IceReciprocatingStatus             | [ice::reciprocating::Status](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#status-4) |
| 9 | IceFuelTankStatusRosToUavcan | uavcan_msgs/IceFuelTankStatus             | [ice::FuelTankStatus](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#fueltankstatus) |
| 10 | BatteryInfoRosToUavcan | [sensor_msgs/BatteryState](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/BatteryState.html)             | [power::BatteryInfo](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#batteryinfo) |

Here we try to use default ROS messages as much as possible, but sometimes we need to define our own messages `uavcan_msgs`.

## 2. Preparation

You need to perform the following steps to use this package:

1. Clone this repository using recursive. Update submodules every time you pull this repo
2. Install required packages using `install_requirements.sh` script
3. Build [libuavcan v0.1](https://github.com/UAVCAN/libuavcan/tree/legacy-v0#using-in-a-gnulinux-application) as a static library and install it on the system globally
4. (optionally) Build dsds if you want to use custom messages

```
cd catkin_ws/src
git clone --recursive git@github.com:InnopolisAero/uavcan_communicator.git
cd uavcan_communicator
git submodule update --init --recursive
./scripts/install_requirements.sh
./scripts/install_libuavcan.sh
./scripts/compile_dsdl.sh
```

## 3. Running

1. At first, you need to create a virtual can port
2. Then specify in `config/params.yaml` which conversions do you need to use
3. Then launch communicator typing:

Example:
```
roslaunch uavcan_communicator inno_dynamics_communicator.launch
```

## 4. Usage example

Below you can see an example of using the uavcan_communicator in conjunction with a VTOL dynamics simulator.

[![uavcan vtol dynamics simulator](https://img.youtube.com/vi/JmElAwgAoSc/0.jpg)](https://youtu.be/JmElAwgAoSc)