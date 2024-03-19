cmake_minimum_required(VERSION 3.5)
project(fkie_measurement_sensor_simulator)
#---- Enable C++17 ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
else()
    add_definitions(-Wpedantic)
endif()

# Update the policy setting to avoid an error when loading the ament_cmake package
# at the current cmake version level
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(fkie_measurement_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)



add_definitions( -DUSING_ROS2 )

set(DEPS
        fkie_measurement_msgs
        visualization_msgs
        geometry_msgs
        tf2_ros
        rclcpp
    )

include_directories(include)

add_executable(sensor-simulator
        src/SensorSimulator.cpp
        src/SensorSimulatorNode.cpp
    )
ament_target_dependencies(sensor-simulator ${DEPS})

if(MSVC)
else()
    target_compile_options(sensor-simulator PRIVATE
        -Wall -Wextra -Werror=return-type)
endif()

ament_export_libraries(${PROJECT_NAME})
ament_package()
install(TARGETS
    sensor-simulator
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)
