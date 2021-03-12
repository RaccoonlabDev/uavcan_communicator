# Uavcan communicator

Uavcan communicator offers a bridge between UAVCAN and ROS.

It covers minimal set of sensors required for such applications as PX4 UAVCAN HITL simulation and even a few additional ones. This communicator can be used for other purposes as well.

Example of design of the UAVCAN HITL simulator mentioned above using this communicator is shown on the figure below.

![scheme](img/scheme.png?raw=true "scheme")

Supported convertions are shown below in the table:

UAVCAN->ROS

| № | ROS msg                               | UAVCAN msg                                     |
| - | ------------------------------------- |----------------------------------------------- |
| 1 |[sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html)                       | [esc::RawCommand](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#rawcommand)             |
| 2 | [sensor_msgs::Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)             | [ahrs::AhrsSolution](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#solution) |

ROS->UAVCAN

| № | ROS msg                               | UAVCAN msg                                     |
| - | ------------------------------------- |----------------------------------------------- |
| 1 | uavcan_msgs/StaticTemperature | [air_data::StaticTemperature](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#statictemperature) |
| 2 | uavcan_msgs/StaticPressure    | [air_data::StaticPressure](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#staticpressure)    |
| 3 | uavcan_msgs/RawAirData        | [air_data::RawAirData](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#rawairdata)        |
| 4 | uavcan_msgs/Fix               | [gnss::Fix](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#fix)                   |
| 5 | [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)                       | [ahrs::RawIMU](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#rawimu)                |
| 6 | [sensor_msgs/MagneticField](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/MagneticField.html)             | [ahrs::MagneticFieldStrength](https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/#magneticfieldstrength) |

Here we try to use default ROS messages as much as possible, but they are not enough.

# Preparation

1. Clone this repository using recursive. Update submodules every time you pull this repo:

```
cd catkin_ws/src
git clone --recursive https://github.com/InnopolisAero/innopolis_vtol_dynamics.git .
git submodule update --init --recursive
```

2. Install required packages

```
./install_requirements.sh
```

3. Build libuavcan v0.1 as a static library and install it on the system globally. Use [official instuction](https://github.com/UAVCAN/libuavcan/tree/legacy-v0#using-in-a-gnulinux-application) or this script:

```
./install_libuavcan.sh.sh
```

# Running

1. At first, you need to create virtual can port
If you has [Innopolis sniffer](), just use scipt `./scripts/create_slcan.sh`.
Otherwise you should use `./scripts/create_slcan.sh /dev/ttyACMx`, where `x` is index of your tty device.

2. Then specify in `config/params.yaml` which convertions do you need to use

3. Then launch communicator typing:
```
roslaunch drone_communicators uavcan_communicator.launch
```
