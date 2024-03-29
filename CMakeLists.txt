cmake_minimum_required(VERSION 3.0.2)
project(uavcan_communicator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  mavros_msgs
)

find_library(UAVCAN_LIB uavcan REQUIRED)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -pedantic -Werror -std=c++11")

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ../dsdlc_generated
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/platform_specific_components/linux/libuavcan/include
)

add_executable(${PROJECT_NAME}_uavcan_communicator src/uavcan_communicator/platform_linux.cpp
                                                   src/uavcan_communicator/converters.cpp
                                                   src/uavcan_communicator/uavcan_node.cpp)
target_include_directories(${PROJECT_NAME}_uavcan_communicator
    BEFORE
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/libs/platform_specific_components/linux/libuavcan/uavcan_linux
)
set_target_properties(${PROJECT_NAME}_uavcan_communicator PROPERTIES OUTPUT_NAME uavcan_communicator PREFIX "")
add_dependencies(${PROJECT_NAME}_uavcan_communicator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_uavcan_communicator
    ${catkin_LIBRARIES}
    ${UAVCAN_LIB}
    rt
)

install(TARGETS ${PROJECT_NAME}_uavcan_communicator
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
