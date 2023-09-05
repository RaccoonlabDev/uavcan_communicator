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

#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <string>
#include <map>

#include "converters.hpp"


extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();
static UavcanNode& getUavcanNode() {
    static UavcanNode uavcan_node(getCanDriver(), getSystemClock());
    return uavcan_node;
}


int main(int argc, char** argv) {
    ///< 1. Init ros node
    ros::init(argc, argv, "dronecan_communicator");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::NodeHandle ros_node;


    ///< 2. Init DroneCAN node
    int node_id;
    std::string node_id_param = "uavcan_node_id";
    if (ros_node.getParam(node_id_param.c_str(), node_id)) {
        std::cout << "Param " << node_id_param << ": " << node_id << std::endl;
    } else {
        std::cout << "Param " << node_id_param << ": is missing in your config file." << std::endl;
        return -1;
    }

    std::string uavcan_node_name;
    std::string node_name_param = "uavcan_node_name";
    if (ros_node.getParam(node_name_param.c_str(), uavcan_node_name)) {
        std::cout << "Param " << node_name_param << ": " << uavcan_node_name << std::endl;
    } else {
        std::cout << "Param " << node_name_param << ": is missing in your config file." << std::endl;
        return -1;
    }

    auto& uavcan_node = getUavcanNode();
    const int self_node_id = node_id;
    uavcan_node.setNodeID(self_node_id);
    uavcan_node.setName(uavcan_node_name.c_str());
    uavcan::protocol::SoftwareVersion sw_version;
    sw_version.major = 1;
    uavcan_node.setSoftwareVersion(sw_version);
    uavcan::protocol::HardwareVersion hw_version;
    hw_version.major = 1;
    uavcan_node.setHardwareVersion(hw_version);
    const int node_start_res = uavcan_node.start();
    if (node_start_res < 0) {
        throw std::runtime_error("ERROR: Uavcan_node: " + std::to_string(node_start_res));
    }


    ///< 3. Instantiate converters
    std::vector<std::string> bridges;
    std::vector<std::unique_ptr<Converter>> converters;
    if (ros_node.getParam("bridges",   bridges)) {
        if (bridges.size() == 0 || bridges.size() % 2 == 1) {
            std::cout << "ERROR. Param problem: The size of `bridges` must be even." << std::endl;
            return -1;
        }
        for (size_t idx = 0; idx < bridges.size(); idx += 2) {
            auto bridge_name = bridges[idx];
            auto topic_name = bridges[idx + 1].c_str();
            auto converter = instantiate_converter(bridge_name, ros_node, uavcan_node, topic_name);
            std::cout << idx / 2 << ". Creation of converter with name `" << bridge_name << "` "
                      << "and topic `" << topic_name << "` has been ";
            if (converter.get() == nullptr) {
                std::cout << "\033[1;31m" << "failed: wrong converter name. Exit." << "\033[0m" << std::endl;
                return -1;
            } else {
                converters.push_back(std::move(converter));
                std::cout << "successful." << std::endl;
            }
        }
    } else {
        std::cout << "Param problem: you should specify bridges in your config file." << std::endl;
    }

    // 4. Spinner
    uavcan_node.setModeOperational();
    uavcan_node.setHealthOk();
    while (ros::ok()) {
        const int res = uavcan_node.spin(uavcan::MonotonicDuration::fromMSec(2));
        ros::spinOnce();
        if (res < 0) {
            std::cerr << "Transient failure: " << res << std::endl;
        }
    }
}
