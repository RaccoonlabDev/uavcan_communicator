/*
 * Copyright (c) 2020-2023 RaccoonLab.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

#ifndef SRC_UAVCAN_COMMUNICATOR_CONVERTERS_HPP_
#define SRC_UAVCAN_COMMUNICATOR_CONVERTERS_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/BatteryStatus.h>
#include <mavros_msgs/ESCTelemetryItem.h>
#include <mavros_msgs/ESCStatusItem.h>

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
#include <uavcan/equipment/gnss/Fix2.hpp>
#include <uavcan/equipment/ice/FuelTankStatus.hpp>
#include <uavcan/equipment/ice/reciprocating/Status.hpp>
#include <uavcan/equipment/power/CircuitStatus.hpp>
#include <uavcan/equipment/power/BatteryInfo.hpp>

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
        ros_pub_ = ros_node.advertise<OUT_ROS_MSG>(ros_topic, 5);
        uavcan_sub_.start(std::bind(&UavcanToRosConverter::uavcan_callback, this, std::placeholders::_1));
    }
};

template<typename IN_ROS, typename OUT_UAVCAN>
class RosToUavcanConverter: public Converter {
public:
    typedef OUT_UAVCAN OUT_UAVCAN_MSG;
    OUT_UAVCAN_MSG out_uavcan_msg_;
    bool enabled{true};
protected:
    typedef IN_ROS IN_ROS_MSG;
    typedef typename IN_ROS::Ptr IN_ROS_MSG_PTR;

    uavcan::Publisher<OUT_UAVCAN_MSG> uavcan_pub_;
    ros::Subscriber ros_sub_;
    std::string _name;

    virtual void ros_callback(IN_ROS_MSG_PTR in_ros_msg) = 0;
    RosToUavcanConverter(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic, std::string name):
        uavcan_pub_(uavcan_node), _name(name) {
        ros_sub_ = ros_node.subscribe(ros_topic, 1, &RosToUavcanConverter::ros_callback, this);
    }

    void broadcast() {
        if (enabled == false) {
            return;
        }
        int pub_res = uavcan_pub_.broadcast(out_uavcan_msg_);
        if (pub_res < 0) {
            std::cerr << _name << " publication failure: " << pub_res << std::endl;
        }
    }
};


class ActuatorsUavcanToRos: public UavcanToRosConverter<
    uavcan::equipment::esc::RawCommand,
    sensor_msgs::Joy> {
    void uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) override;
public:
    ActuatorsUavcanToRos(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        UavcanToRosConverter(ros_node, uavcan_node, ros_topic) { }
};

class AhrsSolutionUavcanToRos: public UavcanToRosConverter<
    uavcan::equipment::ahrs::Solution,
    sensor_msgs::Imu> {
    void uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) override;
public:
    AhrsSolutionUavcanToRos(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        UavcanToRosConverter(ros_node, uavcan_node, ros_topic) { }
};


class ArmUavcanToRos: public UavcanToRosConverter<
    uavcan::equipment::esc::RawCommand,
    std_msgs::Bool> {
    void uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) override;

public:
    ArmUavcanToRos(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        UavcanToRosConverter(ros_node, uavcan_node, ros_topic) {}
};


class EscStatusUavcanToRos: public UavcanToRosConverter<
    uavcan::equipment::esc::Status,
    mavros_msgs::ESCTelemetryItem> {
    void uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) override;

public:
    EscStatusUavcanToRos(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        UavcanToRosConverter(ros_node, uavcan_node, ros_topic) {}
};


class CircuitStatusUavcanToRos: public UavcanToRosConverter<
    uavcan::equipment::power::CircuitStatus,
    mavros_msgs::BatteryStatus> {
    void uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) override;

public:
    CircuitStatusUavcanToRos(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        UavcanToRosConverter(ros_node, uavcan_node, ros_topic) {}
};


class BaroStaticPressureRosToUavcan: public RosToUavcanConverter<
    std_msgs::Float32,
    uavcan::equipment::air_data::StaticPressure> {
    void ros_callback(IN_ROS_MSG_PTR in_ros_msg) override;
public:
    BaroStaticPressureRosToUavcan(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        RosToUavcanConverter(ros_node, uavcan_node, ros_topic, __FUNCTION__) {}
};


class BaroStaticTemperatureRosToUavcan: public RosToUavcanConverter<
    std_msgs::Float32,
    uavcan::equipment::air_data::StaticTemperature> {
    void ros_callback(IN_ROS_MSG_PTR in_ros_msg) override;
public:
    BaroStaticTemperatureRosToUavcan(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        RosToUavcanConverter(ros_node, uavcan_node, ros_topic, __FUNCTION__) {}
};


class DiffPressureRosToUavcan: public RosToUavcanConverter<
    std_msgs::Float32,
    uavcan::equipment::air_data::RawAirData> {
    void ros_callback(IN_ROS_MSG_PTR in_ros_msg) override;
    BaroStaticPressureRosToUavcan _pressure;
    BaroStaticTemperatureRosToUavcan _temperature;
public:
    DiffPressureRosToUavcan(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        RosToUavcanConverter(ros_node, uavcan_node, ros_topic, __FUNCTION__),
        _pressure(ros_node, uavcan_node, "/uav/static_pressure"),
        _temperature(ros_node, uavcan_node, "/uav/static_temperature") {
            _pressure.enabled = false;
            _temperature.enabled = false;
        }
};


class GpsRosToUavcan: public RosToUavcanConverter<
    sensor_msgs::NavSatFix,
    uavcan::equipment::gnss::Fix2> {
    void ros_callback(IN_ROS_MSG_PTR in_ros_msg) override;
    void ros_velocity_callback(geometry_msgs::Twist::Ptr in_ros_msg);
    ros::Subscriber ros_velocity_sub_;
public:
    GpsRosToUavcan(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        RosToUavcanConverter(ros_node, uavcan_node, ros_topic, __FUNCTION__) {
        ros_velocity_sub_ = ros_node.subscribe("/uav/velocity", 1, &GpsRosToUavcan::ros_velocity_callback, this);
        out_uavcan_msg_.sats_used = 10.0;
        out_uavcan_msg_.status = 3;
        out_uavcan_msg_.pdop = 1.0;
    }
};


class ImuRosToUavcan: public RosToUavcanConverter<
    sensor_msgs::Imu,
    uavcan::equipment::ahrs::RawIMU> {
    void ros_callback(IN_ROS_MSG_PTR in_ros_msg) override;
public:
    ImuRosToUavcan(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        RosToUavcanConverter(ros_node, uavcan_node, ros_topic, __FUNCTION__) {}
};


class MagnetometerRosToUavcan: public RosToUavcanConverter<
    sensor_msgs::MagneticField,
    uavcan::equipment::ahrs::MagneticFieldStrength> {
    void ros_callback(IN_ROS_MSG_PTR in_ros_msg) override;
public:
    MagnetometerRosToUavcan(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        RosToUavcanConverter(ros_node, uavcan_node, ros_topic, __FUNCTION__) {}
};

class EscStatusRosToUavcan: public RosToUavcanConverter<
    mavros_msgs::ESCTelemetryItem,
    uavcan::equipment::esc::Status> {
    void ros_callback(IN_ROS_MSG_PTR in_ros_msg) override;

public:
    EscStatusRosToUavcan(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        RosToUavcanConverter(ros_node, uavcan_node, ros_topic, __FUNCTION__) {}
};

class IceReciprocatingStatusRosToUavcan: public RosToUavcanConverter<
    mavros_msgs::ESCStatusItem,
    uavcan::equipment::ice::reciprocating::Status> {
    void ros_callback(IN_ROS_MSG_PTR in_ros_msg) override;
    void ros_status_callback(std_msgs::UInt8 in_ros_msg);
    ros::Subscriber ros_status_sub_;
public:
    IceReciprocatingStatusRosToUavcan(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        RosToUavcanConverter(ros_node, uavcan_node, ros_topic, __FUNCTION__) {
        ros_status_sub_ = ros_node.subscribe("/uav/ice_status", 1, &IceReciprocatingStatusRosToUavcan::ros_status_callback, this);
    }
};

class IceFuelTankStatusRosToUavcan: public RosToUavcanConverter<
    std_msgs::UInt8,
    uavcan::equipment::ice::FuelTankStatus> {
    void ros_callback(IN_ROS_MSG_PTR in_ros_msg) override;

public:
    IceFuelTankStatusRosToUavcan(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        RosToUavcanConverter(ros_node, uavcan_node, ros_topic, __FUNCTION__) {}
};

class BatteryInfoRosToUavcan: public RosToUavcanConverter<
    sensor_msgs::BatteryState,
    uavcan::equipment::power::BatteryInfo> {
    void ros_callback(IN_ROS_MSG_PTR in_ros_msg) override;

public:
    BatteryInfoRosToUavcan(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
        RosToUavcanConverter(ros_node, uavcan_node, ros_topic, __FUNCTION__) {}
};


std::unique_ptr<Converter> instantiate_converter(std::string converter_name,
                                                 ros::NodeHandle& ros_node,
                                                 UavcanNode& uavcan_node,
                                                 const char* ros_topic);

#endif  // SRC_UAVCAN_COMMUNICATOR_CONVERTERS_HPP_
