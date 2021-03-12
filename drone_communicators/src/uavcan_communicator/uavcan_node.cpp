/**
* @file uavcan_node.cpp
* @author Dmitry Ponomarev  <ponomarevda96@gmail.com>
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
    ros::init(argc, argv, "uavcan_communicator");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::NodeHandle ros_node;


    ///< 2. Init uavcan node
    int node_id;
    std::string uavcan_node_name;
    if (ros::param::get("/uavcan/uavcan_node_id", node_id)) {
        std::cout << "Param node_id: " << node_id << std::endl;
    } else {
        std::cout << "Param problem: You should specify node_id in your config file." << std::endl;
        return -1;
    }
    if (ros::param::get("/uavcan/uavcan_node_name", uavcan_node_name)) {
        std::cout << "Param node_id: " << uavcan_node_name << std::endl;
    } else {
        std::cout << "Param problem: You should specify node_id in your config file." << std::endl;
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
    if (ros::param::get("/uavcan/bridges",   bridges)) {
        if (bridges.size() == 0 || bridges.size() % 2 == 1) {
            std::cout << "ERROR. Param problem: The size of `bridges` must be even." << std::endl;
            return -1;
        }
        for (size_t idx = 0; idx < bridges.size(); idx += 2) {
            std::string converter_name = bridges[idx];
            std::string converter_topic = bridges[idx + 1];
            std::unique_ptr<Converter> converter = instantiate_converter(converter_name,
                                                                         ros_node,
                                                                         uavcan_node,
                                                                         converter_topic.c_str());
            std::cout << idx / 2 << ". Creation of converter with name `" << converter_name << "` "
                                 << "and topic `" << converter_topic << "` has been ";
            if (converter.get() == nullptr) {
                std::cout << "failed: wrong converter name. Finish." << std::endl;
                return -1;
            } else {
                converters.push_back(std::move(converter));
                std::cout << "successful." << std::endl;
            }
        }
    } else {
        std::cout << "Param problem: you should specify bridges in your config file." << std::endl;
    }

    // 2. Spinner
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
