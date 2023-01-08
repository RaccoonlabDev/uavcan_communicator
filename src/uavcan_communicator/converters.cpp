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

#include "converters.hpp"


void ActuatorsUavcanToRos::uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) {
    if (uavcan_msg.cmd.size() > 0 && uavcan_msg.cmd.size() <= 20) {
        sensor_msgs::Joy ros_msg;
        ros_msg.header.stamp = ros::Time::now();
        for (auto cmd : uavcan_msg.cmd) {
            if (cmd >= 0) {
                ros_msg.axes.push_back(cmd / 8091.0);
            } else {
                ros_msg.axes.push_back(cmd / 8092.0);
            }
        }
        ros_pub_.publish(ros_msg);
    }
}


void AhrsSolutionUavcanToRos::uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) {
    ros_msg_.header.stamp = ros::Time::now();

    ros_msg_.orientation.x = uavcan_msg.orientation_xyzw[0];
    ros_msg_.orientation.y = uavcan_msg.orientation_xyzw[1];
    ros_msg_.orientation.z = uavcan_msg.orientation_xyzw[2];
    ros_msg_.orientation.w = uavcan_msg.orientation_xyzw[3];

    ros_msg_.angular_velocity.x = uavcan_msg.angular_velocity[0];
    ros_msg_.angular_velocity.y = uavcan_msg.angular_velocity[1];
    ros_msg_.angular_velocity.z = uavcan_msg.angular_velocity[2];

    ros_msg_.linear_acceleration.x = uavcan_msg.linear_acceleration[0];
    ros_msg_.linear_acceleration.y = uavcan_msg.linear_acceleration[1];
    ros_msg_.linear_acceleration.z = uavcan_msg.linear_acceleration[2];

    ros_pub_.publish(ros_msg_);
}


void ArmUavcanToRos::uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) {
    ros_msg_.data = false;
    if (uavcan_msg.cmd.size() != 0) {
        for (auto raw_cmd : uavcan_msg.cmd) {
            if (raw_cmd != -1) {
                ros_msg_.data = true;
            }
        }
    }
    ros_pub_.publish(ros_msg_);
}


void CircuitStatusUavcanToRos::uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) {
    ros_msg_.circuit_id = uavcan_msg.circuit_id;
    ros_msg_.voltage = uavcan_msg.voltage;
    ros_msg_.current = uavcan_msg.current;
    ros_msg_.error_flags = uavcan_msg.error_flags;
    ros_pub_.publish(ros_msg_);
}


void EscStatusUavcanToRos::uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) {
    ros_msg_.error_count = uavcan_msg.error_count;
    ros_msg_.voltage = uavcan_msg.voltage;
    ros_msg_.current = uavcan_msg.current;
    ros_msg_.temperature = uavcan_msg.temperature;
    ros_msg_.rpm = uavcan_msg.rpm;
    ros_msg_.power_rating_pct = uavcan_msg.power_rating_pct;
    ros_msg_.esc_index = uavcan_msg.esc_index;
    ros_pub_.publish(ros_msg_);
}


void BaroStaticPressureRosToUavcan::ros_callback(IN_ROS_MSG_PTR in_ros_msg) {
    out_uavcan_msg_.static_pressure = in_ros_msg->data;
    out_uavcan_msg_.static_pressure_variance = 1;
    broadcast();
}


void BaroStaticTemperatureRosToUavcan::ros_callback(IN_ROS_MSG_PTR in_ros_msg) {
    out_uavcan_msg_.static_temperature = in_ros_msg->data;
    out_uavcan_msg_.static_temperature_variance = 1;
    broadcast();
}


void DiffPressureRosToUavcan::ros_callback(IN_ROS_MSG_PTR in_ros_msg) {
    out_uavcan_msg_.static_air_temperature = _temperature.out_uavcan_msg_.static_temperature;
    out_uavcan_msg_.static_pressure = _pressure.out_uavcan_msg_.static_pressure;
    out_uavcan_msg_.differential_pressure = in_ros_msg->data;
    broadcast();
}


void GpsRosToUavcan::ros_callback(IN_ROS_MSG_PTR in_ros_msg) {
    out_uavcan_msg_.latitude_deg_1e8 = in_ros_msg->latitude * 1e8;
    out_uavcan_msg_.longitude_deg_1e8 = in_ros_msg->longitude * 1e8;
    out_uavcan_msg_.height_msl_mm = in_ros_msg->altitude * 1e3;
    broadcast();
}

void GpsRosToUavcan::ros_velocity_callback(geometry_msgs::Twist::Ptr in_ros_msg) {
    out_uavcan_msg_.ned_velocity[0] = in_ros_msg->linear.x;
    out_uavcan_msg_.ned_velocity[1] = in_ros_msg->linear.y;
    out_uavcan_msg_.ned_velocity[2] = in_ros_msg->linear.z;
}


void ImuRosToUavcan::ros_callback(IN_ROS_MSG_PTR in_ros_msg) {
    out_uavcan_msg_.rate_gyro_latest[0] = in_ros_msg->angular_velocity.x;
    out_uavcan_msg_.rate_gyro_latest[1] = in_ros_msg->angular_velocity.y;
    out_uavcan_msg_.rate_gyro_latest[2] = in_ros_msg->angular_velocity.z;

    out_uavcan_msg_.accelerometer_latest[0] = in_ros_msg->linear_acceleration.x;
    out_uavcan_msg_.accelerometer_latest[1] = in_ros_msg->linear_acceleration.y;
    out_uavcan_msg_.accelerometer_latest[2] = in_ros_msg->linear_acceleration.z;

    broadcast();
}


void MagnetometerRosToUavcan::ros_callback(IN_ROS_MSG_PTR in_ros_msg) {
    out_uavcan_msg_.magnetic_field_ga[0] = in_ros_msg->magnetic_field.x;
    out_uavcan_msg_.magnetic_field_ga[1] = in_ros_msg->magnetic_field.y;
    out_uavcan_msg_.magnetic_field_ga[2] = in_ros_msg->magnetic_field.z;
    broadcast();
}

void EscStatusRosToUavcan::ros_callback(IN_ROS_MSG_PTR in_ros_msg) {
    out_uavcan_msg_.error_count = in_ros_msg->error_count;
    out_uavcan_msg_.voltage = in_ros_msg->voltage;
    out_uavcan_msg_.current = in_ros_msg->current;
    out_uavcan_msg_.temperature = in_ros_msg->temperature;
    out_uavcan_msg_.rpm = in_ros_msg->rpm;
    out_uavcan_msg_.power_rating_pct = in_ros_msg->power_rating_pct;
    out_uavcan_msg_.esc_index = in_ros_msg->esc_index;
    broadcast();
}

void IceReciprocatingStatusRosToUavcan::ros_callback(IN_ROS_MSG_PTR in_ros_msg) {
    out_uavcan_msg_.state = in_ros_msg->state;
    out_uavcan_msg_.flags = in_ros_msg->flags;
    out_uavcan_msg_.engine_load_percent = in_ros_msg->engine_load_percent;
    out_uavcan_msg_.engine_speed_rpm = in_ros_msg->engine_speed_rpm;

    out_uavcan_msg_.spark_dwell_time_ms = in_ros_msg->spark_dwell_time_ms;
    out_uavcan_msg_.atmospheric_pressure_kpa = in_ros_msg->atmospheric_pressure_kpa;
    out_uavcan_msg_.intake_manifold_pressure_kpa = in_ros_msg->intake_manifold_pressure_kpa;
    out_uavcan_msg_.intake_manifold_temperature = in_ros_msg->intake_manifold_temperature;
    out_uavcan_msg_.coolant_temperature = in_ros_msg->coolant_temperature;
    out_uavcan_msg_.oil_pressure = in_ros_msg->oil_pressure;
    out_uavcan_msg_.oil_temperature = in_ros_msg->oil_temperature;
    out_uavcan_msg_.fuel_pressure = in_ros_msg->fuel_pressure;
    out_uavcan_msg_.fuel_consumption_rate_cm3pm = in_ros_msg->fuel_consumption_rate_cm3pm;
    out_uavcan_msg_.estimated_consumed_fuel_volume_cm3 = in_ros_msg->estimated_consumed_fuel_volume_cm3;

    out_uavcan_msg_.throttle_position_percent = in_ros_msg->throttle_position_percent;
    out_uavcan_msg_.ecu_index = in_ros_msg->ecu_index;
    out_uavcan_msg_.spark_plug_usage = in_ros_msg->spark_plug_usage;

    broadcast();
}

void IceFuelTankStatusRosToUavcan::ros_callback(IN_ROS_MSG_PTR in_ros_msg) {
    out_uavcan_msg_.available_fuel_volume_percent = in_ros_msg->data;
    broadcast();
}

void BatteryInfoRosToUavcan::ros_callback(IN_ROS_MSG_PTR in_ros_msg) {
    ///< Required by PX4
    out_uavcan_msg_.voltage = in_ros_msg->voltage;
    out_uavcan_msg_.current = in_ros_msg->current;
    out_uavcan_msg_.full_charge_capacity_wh = in_ros_msg->design_capacity;
    out_uavcan_msg_.state_of_charge_pct = in_ros_msg->percentage * 100;
    out_uavcan_msg_.status_flags = 1;               ///< STATUS_FLAG_IN_USE
    out_uavcan_msg_.model_instance_id = 1;          ///< Set to zero if not applicable
    out_uavcan_msg_.temperature = 300;              ///< Some fields can be set to NAN if their values are unknown.

    ///< Unused by PX4
    out_uavcan_msg_.remaining_capacity_wh = in_ros_msg->capacity;
    out_uavcan_msg_.average_power_10sec = 0;        ///< Some fields can be set to NAN if their values are unknown.
    out_uavcan_msg_.hours_to_full_charge = 0;       ///< Some fields can be set to NAN if their values are unknown.
    out_uavcan_msg_.state_of_health_pct = 127;      ///< STATE_OF_HEALTH_UNKNOWN
    out_uavcan_msg_.state_of_charge_pct_stdev = 0;  ///< use best guess if unknown
    out_uavcan_msg_.battery_id = 0;                 ///< 0 - primary battery
    out_uavcan_msg_.model_name = "simulated_battery";

    broadcast();
}

std::unique_ptr<Converter> instantiate_converter(std::string converter_name,
                                                 ros::NodeHandle& ros_node,
                                                 UavcanNode& uavcan_node,
                                                 const char* ros_topic) {
    std::unique_ptr<Converter> converter(nullptr);
    if (converter_name.compare("ActuatorsUavcanToRos") == 0) {
        converter = std::unique_ptr<Converter>(new ActuatorsUavcanToRos(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("AhrsSolutionUavcanToRos") == 0) {
        converter = std::unique_ptr<Converter>(new AhrsSolutionUavcanToRos(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("ArmUavcanToRos") == 0) {
        converter = std::unique_ptr<Converter>(new ArmUavcanToRos(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("CircuitStatusUavcanToRos") == 0) {
        converter = std::unique_ptr<Converter>(new CircuitStatusUavcanToRos(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("EscStatusUavcanToRos") == 0) {
        converter = std::unique_ptr<Converter>(new EscStatusUavcanToRos(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("BaroStaticPressureRosToUavcan") == 0) {
        converter = std::unique_ptr<Converter>(new BaroStaticPressureRosToUavcan(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("BaroStaticTemperatureRosToUavcan") == 0) {
        converter = std::unique_ptr<Converter>(new BaroStaticTemperatureRosToUavcan(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("DiffPressureRosToUavcan") == 0) {
        converter = std::unique_ptr<Converter>(new DiffPressureRosToUavcan(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("GpsRosToUavcan") == 0) {
        converter = std::unique_ptr<Converter>(new GpsRosToUavcan(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("ImuRosToUavcan") == 0) {
        converter = std::unique_ptr<Converter>(new ImuRosToUavcan(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("MagnetometerRosToUavcan") == 0) {
        converter = std::unique_ptr<Converter>(new MagnetometerRosToUavcan(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("EscStatusRosToUavcan") == 0) {
        converter = std::unique_ptr<Converter>(new EscStatusRosToUavcan(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("IceReciprocatingStatusRosToUavcan") == 0) {
        converter = std::unique_ptr<Converter>(new IceReciprocatingStatusRosToUavcan(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("IceFuelTankStatusRosToUavcan") == 0) {
        converter = std::unique_ptr<Converter>(new IceFuelTankStatusRosToUavcan(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("BatteryInfoRosToUavcan") == 0) {
        converter = std::unique_ptr<Converter>(new BatteryInfoRosToUavcan(ros_node, uavcan_node, ros_topic));
    } else {
        std::cout << "ERROR: instantiate_converter, wrong converter name" << std::endl;
    }

    return converter;
}
