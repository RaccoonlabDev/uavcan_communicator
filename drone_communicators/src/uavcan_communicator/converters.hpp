/**
* @file converters.hpp
* @author Dmitry Ponomarev  <ponomarevda96@gmail.com>
*/

#ifndef DRONE_COMMUNICATORS_SRC_UAVCAN_COMMUNICATOR_CONVERTERS_HPP_
#define DRONE_COMMUNICATORS_SRC_UAVCAN_COMMUNICATOR_CONVERTERS_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>
#include <uavcan_msgs/StaticPressure.h>
#include <uavcan_msgs/StaticTemperature.h>
#include <uavcan_msgs/RawAirData.h>
#include <uavcan_msgs/Fix.h>
#include <uavcan_msgs/CircuitStatus.h>
#include <uavcan_msgs/EscStatus.h>

#include <iostream>
#include <memory>
#include <string>

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/ahrs/MagneticFieldStrength.hpp>
#include <uavcan/equipment/ahrs/RawIMU.hpp>
#include <uavcan/equipment/ahrs/Solution.hpp>
#include <uavcan/equipment/air_data/RawAirData.hpp>
#include <uavcan/equipment/air_data/StaticPressure.hpp>
#include <uavcan/equipment/air_data/StaticTemperature.hpp>
#include <uavcan/equipment/esc/RawCommand.hpp>
#include <uavcan/equipment/esc/Status.hpp>
#include <uavcan/equipment/gnss/Fix.hpp>
#include <uavcan/equipment/power/CircuitStatus.hpp>


constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> UavcanNode;


class Converter {};

template<typename IN_UAVCAN, typename OUT_ROS>
class UavcanToRosConverter: public Converter {
protected:
    typedef OUT_ROS OUT_ROS_MSG;
    typedef IN_UAVCAN IN_UAVCAN_MSG;
    ros::Publisher ros_pub_;
    uavcan::Subscriber<IN_UAVCAN_MSG> uavcan_sub_;
    OUT_ROS_MSG ros_msg_;

    virtual void uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) = 0;
    UavcanToRosConverter(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic) :
                         uavcan_sub_(uavcan_node) {
        ros_pub_ = ros_node.advertise<OUT_ROS_MSG>(ros_topic, 1);
        uavcan_sub_.start(std::bind(&UavcanToRosConverter::uavcan_callback, this, std::placeholders::_1));
    }
};

template<typename IN_ROS, typename OUT_UAVCAN>
class RosToUavcanConverter: public Converter {
protected:
    typedef IN_ROS IN_ROS_MSG;
    typedef typename IN_ROS::Ptr IN_ROS_MSG_PTR;
    typedef OUT_UAVCAN OUT_UAVCAN_MSG;

    uavcan::Publisher<OUT_UAVCAN_MSG> uavcan_pub_;
    ros::Subscriber ros_sub_;
    OUT_UAVCAN_MSG out_uavcan_msg_;

    virtual void ros_callback(IN_ROS_MSG_PTR in_ros_msg) = 0;
    RosToUavcanConverter(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
                         uavcan_pub_(uavcan_node) {
        ros_sub_ = ros_node.subscribe(ros_topic, 1, &RosToUavcanConverter::ros_callback, this);
    }
};


class Actuators: public UavcanToRosConverter<uavcan::equipment::esc::RawCommand,
                                             sensor_msgs::Joy> {
    void uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) override;
public:
    Actuators(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic);
};

class AhrsSolution: public UavcanToRosConverter<uavcan::equipment::ahrs::Solution,
                                                sensor_msgs::Imu> {
    void uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) override;
public:
    AhrsSolution(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
              UavcanToRosConverter(ros_node, uavcan_node, ros_topic) { }
};


class Arm: public UavcanToRosConverter<uavcan::equipment::esc::RawCommand,
                                       std_msgs::Bool> {
    void uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) override;

public:
    Arm(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        UavcanToRosConverter(ros_node, uavcan_node, ros_topic) {}
};


class EscStatusUavcanToRos: public UavcanToRosConverter<uavcan::equipment::esc::Status,
                                                        uavcan_msgs::EscStatus> {
    void uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) override;

public:
    EscStatusUavcanToRos(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
                         UavcanToRosConverter(ros_node, uavcan_node, ros_topic) {}
};


class CircuitStatusUavcanToRos: public UavcanToRosConverter<uavcan::equipment::power::CircuitStatus,
                                                            uavcan_msgs::CircuitStatus> {
    void uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) override;

public:
    CircuitStatusUavcanToRos(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
                             UavcanToRosConverter(ros_node, uavcan_node, ros_topic) {}
};


class BaroStaticPressure: public RosToUavcanConverter<uavcan_msgs::StaticPressure,
                                                      uavcan::equipment::air_data::StaticPressure> {
    void ros_callback(IN_ROS_MSG_PTR in_ros_msg) override;
public:
    BaroStaticPressure(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        RosToUavcanConverter(ros_node, uavcan_node, ros_topic) {}
};


class BaroStaticTemperature: public RosToUavcanConverter<uavcan_msgs::StaticTemperature,
                                                         uavcan::equipment::air_data::StaticTemperature> {
    void ros_callback(IN_ROS_MSG_PTR in_ros_msg) override;
public:
    BaroStaticTemperature(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        RosToUavcanConverter(ros_node, uavcan_node, ros_topic) {}
};


class DiffPressure: public RosToUavcanConverter<uavcan_msgs::RawAirData,
                                                uavcan::equipment::air_data::RawAirData> {
    void ros_callback(IN_ROS_MSG_PTR in_ros_msg) override;
public:
    DiffPressure(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        RosToUavcanConverter(ros_node, uavcan_node, ros_topic) {}
};


class GPS: public RosToUavcanConverter<uavcan_msgs::Fix,
                                       uavcan::equipment::gnss::Fix> {
    void ros_callback(IN_ROS_MSG_PTR in_ros_msg) override;
public:
    GPS(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        RosToUavcanConverter(ros_node, uavcan_node, ros_topic) {}
};


class IMU: public RosToUavcanConverter<sensor_msgs::Imu,
                                       uavcan::equipment::ahrs::RawIMU> {
    void ros_callback(IN_ROS_MSG_PTR in_ros_msg) override;
public:
    IMU(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        RosToUavcanConverter(ros_node, uavcan_node, ros_topic) {}
};


class Magnetometer: public RosToUavcanConverter<sensor_msgs::MagneticField,
                           uavcan::equipment::ahrs::MagneticFieldStrength> {
    void ros_callback(IN_ROS_MSG_PTR in_ros_msg) override;
public:
    Magnetometer(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        RosToUavcanConverter(ros_node, uavcan_node, ros_topic) {}
};


std::unique_ptr<Converter> instantiate_converter(std::string converter_name,
                                                 ros::NodeHandle& ros_node,
                                                 UavcanNode& uavcan_node,
                                                 const char* ros_topic);

#endif  // DRONE_COMMUNICATORS_SRC_UAVCAN_COMMUNICATOR_CONVERTERS_HPP_
