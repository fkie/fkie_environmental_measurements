cmake_minimum_required(VERSION 3.5)

project(fkie_measurement_msgs)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
    action/RequestBoundaryPolygons.action 
    action/RequestGaussianInterpolation.action
    action/ExplorePolygon3D.action
    msg/Measurement.msg
    msg/MeasurementValue.msg
    msg/MeasurementLocated.msg
    msg/MeasurementArray.msg
    msg/MeasurementGaussianStatistics.msg
    msg/BoundaryPolygon.msg 
    msg/MeasurementEstimation.msg
    msg/MeasurementEstimationValue.msg
     DEPENDENCIES std_msgs actionlib_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs)
ament_package()
