/**
* @file converters.cpp
* @author Dmitry Ponomarev  <ponomarevda96@gmail.com>
*/


#include "converters.hpp"


void Actuators::uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) {
    ros_msg_.header.stamp = ros::Time::now();

    if (uavcan_msg.cmd.size() == 8) {
        ros_msg_.axes[0] = uavcan_msg.cmd[0] / 8191.0;
        ros_msg_.axes[1] = uavcan_msg.cmd[1] / 8191.0;
        ros_msg_.axes[2] = uavcan_msg.cmd[2] / 8191.0;
        ros_msg_.axes[3] = uavcan_msg.cmd[3] / 8191.0;

        ros_msg_.axes[4] = 0.5 + (uavcan_msg.cmd[4] - 4096.0) / 8191.0;
        ros_msg_.axes[5] = (uavcan_msg.cmd[5] == -1) ? 0 : uavcan_msg.cmd[5] / 4096.0 - 1.0;
        ros_msg_.axes[6] = (uavcan_msg.cmd[6] == -1) ? 0 : uavcan_msg.cmd[6] / 4096.0 - 1.0;

        ros_msg_.axes[7] = uavcan_msg.cmd[7] / 8191.0 / 0.75;
    } else {
        ros_msg_.axes[4] = 0.5;
        ros_msg_.axes[5] = 0.0;
        ros_msg_.axes[6] = 0.0;
        ros_msg_.axes[7] = -1.0;
    }

    ros_pub_.publish(ros_msg_);
}

Actuators::Actuators(ros::NodeHandle& ros_node, UavcanNode& uavcan_node, const char* ros_topic):
            UavcanToRosConverter(ros_node, uavcan_node, ros_topic) {
    for (size_t idx = 0; idx < 8; idx++) {
        ros_msg_.axes.push_back(0);
    }
}


void AhrsSolution::uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) {
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


void Arm::uavcan_callback(const uavcan::ReceivedDataStructure<IN_UAVCAN_MSG>& uavcan_msg) {
    ros_msg_.data = false;
    if (uavcan_msg.cmd.size() == 8) {
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


void BaroStaticPressure::ros_callback(IN_ROS_MSG_PTR in_ros_msg) {
    out_uavcan_msg_.static_pressure = in_ros_msg->static_pressure;
    out_uavcan_msg_.static_pressure_variance = 1;
    int pub_res = uavcan_pub_.broadcast(out_uavcan_msg_);
    if (pub_res < 0) {
        std::cerr << "BaroStaticPressure publication failure: " << pub_res << std::endl;
    }
}


void BaroStaticTemperature::ros_callback(IN_ROS_MSG_PTR in_ros_msg) {
    out_uavcan_msg_.static_temperature = in_ros_msg->static_temperature;
    out_uavcan_msg_.static_temperature_variance = 1;
    int pub_res = uavcan_pub_.broadcast(out_uavcan_msg_);
    if (pub_res < 0) {
        std::cerr << "BaroStaticTemperature publication failure: " << pub_res << std::endl;
    }
}


void DiffPressure::ros_callback(IN_ROS_MSG_PTR in_ros_msg) {
    out_uavcan_msg_.static_air_temperature = in_ros_msg->static_air_temperature;
    out_uavcan_msg_.static_pressure = in_ros_msg->static_pressure;
    out_uavcan_msg_.differential_pressure = in_ros_msg->differential_pressure;

    int pub_res = uavcan_pub_.broadcast(out_uavcan_msg_);
    if (pub_res < 0) {
        std::cerr << "DiffPressure publication failure: " << pub_res << std::endl;
    }
}


void GPS::ros_callback(IN_ROS_MSG_PTR in_ros_msg) {
    const double PUB_PERIOD = 0.2;
    static double prev_time_sec = 0;
    double crnt_time_sec = ros::Time::now().toSec();
    if (crnt_time_sec - prev_time_sec > PUB_PERIOD) {
        out_uavcan_msg_.latitude_deg_1e8 = in_ros_msg->latitude_deg_1e8;
        out_uavcan_msg_.longitude_deg_1e8 = in_ros_msg->longitude_deg_1e8;
        out_uavcan_msg_.height_msl_mm = in_ros_msg->height_msl_mm;
        out_uavcan_msg_.ned_velocity[0] = in_ros_msg->ned_velocity.x;
        out_uavcan_msg_.ned_velocity[1] = in_ros_msg->ned_velocity.y;
        out_uavcan_msg_.ned_velocity[2] = in_ros_msg->ned_velocity.z;
        out_uavcan_msg_.sats_used = in_ros_msg->sats_used;
        out_uavcan_msg_.status = in_ros_msg->status;
        out_uavcan_msg_.pdop = in_ros_msg->pdop;
        int pub_res = uavcan_pub_.broadcast(out_uavcan_msg_);
        if (pub_res < 0) {
            std::cerr << "GPS publication failure: " << pub_res << std::endl;
        }
        prev_time_sec = crnt_time_sec;
    }
}


void IMU::ros_callback(IN_ROS_MSG_PTR in_ros_msg) {
    out_uavcan_msg_.rate_gyro_latest[0] = in_ros_msg->angular_velocity.x;
    out_uavcan_msg_.rate_gyro_latest[1] = in_ros_msg->angular_velocity.y;
    out_uavcan_msg_.rate_gyro_latest[2] = in_ros_msg->angular_velocity.z;

    out_uavcan_msg_.accelerometer_latest[0] = in_ros_msg->linear_acceleration.x;
    out_uavcan_msg_.accelerometer_latest[1] = in_ros_msg->linear_acceleration.y;
    out_uavcan_msg_.accelerometer_latest[2] = in_ros_msg->linear_acceleration.z;

    int pub_res = uavcan_pub_.broadcast(out_uavcan_msg_);
    if (pub_res < 0) {
        std::cerr << "IMU publication failure: " << pub_res << std::endl;
    }
}


void Magnetometer::ros_callback(IN_ROS_MSG_PTR in_ros_msg) {
    out_uavcan_msg_.magnetic_field_ga[0] = in_ros_msg->magnetic_field.x;
    out_uavcan_msg_.magnetic_field_ga[1] = in_ros_msg->magnetic_field.y;
    out_uavcan_msg_.magnetic_field_ga[2] = in_ros_msg->magnetic_field.z;

    int pub_res = uavcan_pub_.broadcast(out_uavcan_msg_);
    if (pub_res < 0) {
        std::cerr << "Magnetometer publication failure: " << pub_res << std::endl;
    }
}


std::unique_ptr<Converter> instantiate_converter(std::string converter_name,
                                                 ros::NodeHandle& ros_node,
                                                 UavcanNode& uavcan_node,
                                                 const char* ros_topic) {
    std::unique_ptr<Converter> converter(nullptr);
    if (converter_name.compare("Actuators") == 0) {
        converter = std::unique_ptr<Converter>(new Actuators(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("AhrsSolution") == 0) {
        converter = std::unique_ptr<Converter>(new AhrsSolution(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("Arm") == 0) {
        converter = std::unique_ptr<Converter>(new Arm(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("CircuitStatusUavcanToRos") == 0) {
        converter = std::unique_ptr<Converter>(new CircuitStatusUavcanToRos(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("EscStatusUavcanToRos") == 0) {
        converter = std::unique_ptr<Converter>(new EscStatusUavcanToRos(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("BaroStaticPressure") == 0) {
        converter = std::unique_ptr<Converter>(new BaroStaticPressure(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("BaroStaticTemperature") == 0) {
        converter = std::unique_ptr<Converter>(new BaroStaticTemperature(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("DiffPressure") == 0) {
        converter = std::unique_ptr<Converter>(new DiffPressure(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("GPS") == 0) {
        converter = std::unique_ptr<Converter>(new GPS(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("IMU") == 0) {
        converter = std::unique_ptr<Converter>(new IMU(ros_node, uavcan_node, ros_topic));
    } else if (converter_name.compare("Magnetometer") == 0) {
        converter = std::unique_ptr<Converter>(new Magnetometer(ros_node, uavcan_node, ros_topic));
    } else {
        std::cout << "ERROR: instantiate_converter, wrong converter name" << std::endl;
    }

    return converter;
}
