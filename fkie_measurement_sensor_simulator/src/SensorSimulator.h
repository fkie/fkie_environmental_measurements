// Copyright 2024 Fraunhofer FKIE - All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _SENSOR_SIMULATOR_H_
#define _SENSOR_SIMULATOR_H_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "tf2_ros/buffer.h"
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include "visualization_msgs/msg/marker_array.hpp"

#include "fkie_measurement_sensor_simulator/SourceDescription.hpp"
#include <fkie_measurement_msgs/msg/measurement.hpp>
#include <fkie_measurement_msgs/msg/measurement_array.hpp>
#include <fkie_measurement_msgs/msg/measurement_value.hpp>

class SensorSimulator
{
public:
  std::vector<SourceDescription> sources;

  std::string global_frame = "";
  std::string sensor_frame = "";
  std::string topic_measurement = "measurement";
  std::string topic_measurement_array = "measurement_array";
  std::string topic_sensor_array = "sensor_array";
  double rate = 1.0;
  double marker_size = 0.2;
  int utm_zone_number = 0;
  std::string utm_zone_letter = "";

  // measurement properties
  std::string unique_serial_id = "";
  std::string manufacturer_device_name = "";
  std::string device_classification = "";

  // measurement value properties
  std::string sensor_name = "";
  std::string sensor_source_type = "";
  std::string sensor_unit = "";
  double random_factor = 0.1;
  double random_pos_factor = 0.0;
  double xPos = 0;
  double yPos = 0;
  double xDir = 1.0;
  double yDir = 1.0;

protected:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  geometry_msgs::msg::TransformStamped tf_world_sensor;
  std::unique_ptr<tf2_ros::Buffer> p_tf_buffer;
  rclcpp::Publisher<fkie_measurement_msgs::msg::Measurement>::SharedPtr measurement_pub;
  rclcpp::Publisher<fkie_measurement_msgs::msg::MeasurementArray>::SharedPtr measurement_array_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_location_pub;
  visualization_msgs::msg::MarkerArray marker_locations;

  PositionGrid current_sensor_position;

public:
  SensorSimulator();

  ~SensorSimulator();

  /**
   * @brief Spin sensor simulation methods
   */
  void spin();

  /**
   * @brief Compute current robot position based on TF between [global_frame] and [sensor_frame]
   */
  void updateCurrentSensorPosition();

  /**
   * @brief Computes the accumulated sensor measurement assuming that sensor is located on [current_sensor_position]
   */
  double computeMeasurementFromSources() const;

  /**
   * @brief Publish  current measurements using [measurement_msgs_fkie]
   */
  void publishMeasurement(const double measurement) const;

  /**
   * @brief Publish the location of sensors using RVIZ markers
   */
  void publishSourceLocations();

  /**
   * @brief Computes euler distance between points [p1] and [p2]
   */
  double euclideanDistance(const PositionGrid &p1, const PositionGrid &p2) const;
};

#endif /* _SENSOR_SIMULATOR_H_ */
