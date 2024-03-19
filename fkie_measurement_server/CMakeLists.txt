cmake_minimum_required(VERSION 3.5)
project(fkie_measurement_server)

find_package(ament_cmake QUIET)

if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()


find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()

# Install Python executables
install(PROGRAMS
nodes/measurement_server_node.py
nodes/simple_collector_node.py
DESTINATION lib/${PROJECT_NAME}
)

