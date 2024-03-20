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

#include "SensorSimulator.h"

SensorSimulator::SensorSimulator()
{
  node = std::make_shared<rclcpp::Node>("sensor_simulator", rclcpp::NodeOptions().allow_undeclared_parameters(false));
  p_tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*p_tf_buffer);

  // declare params
  node->declare_parameter("global_frame", global_frame);
  node->declare_parameter("sensor_frame", sensor_frame);
  node->declare_parameter("rate", rate);
  node->declare_parameter("topic_measurement", topic_measurement);
  node->declare_parameter("topic_measurement_array", topic_measurement_array);
  node->declare_parameter("topic_sensor_array", topic_sensor_array);
  node->declare_parameter("marker_size", marker_size);

  node->declare_parameter("unique_serial_id", unique_serial_id);
  node->declare_parameter("manufacturer_device_name", manufacturer_device_name);
  node->declare_parameter("device_classification", device_classification);

  node->declare_parameter("sensor_name", sensor_name);
  node->declare_parameter("sensor_source_type", sensor_source_type);
  node->declare_parameter("sensor_unit", sensor_unit);

  node->declare_parameter("random_factor", random_factor);
  node->declare_parameter("random_pos_factor", random_pos_factor);
  node->declare_parameter("utm_zone_number", utm_zone_number);
  node->declare_parameter("utm_zone_letter", utm_zone_letter);

  // Simulator properties
  global_frame = node->get_parameter("global_frame").as_string();
  sensor_frame = node->get_parameter("sensor_frame").as_string();
  rate = node->get_parameter("rate").as_double();
  topic_measurement = node->get_parameter("topic_measurement").as_string();
  marker_size = node->get_parameter("marker_size").as_double();

  // Measurement properties
  unique_serial_id = node->get_parameter("unique_serial_id").as_string();
  manufacturer_device_name = node->get_parameter("manufacturer_device_name").as_string();
  device_classification = node->get_parameter("device_classification").as_string();

  // Measurement value properties
  sensor_name = node->get_parameter("sensor_name").as_string();
  sensor_source_type = node->get_parameter("sensor_source_type").as_string();
  sensor_unit = node->get_parameter("sensor_unit").as_string();

  // Measurement properties
  random_factor = node->get_parameter("random_factor").as_double();
  random_pos_factor = node->get_parameter("random_pos_factor").as_double();
  utm_zone_number = node->get_parameter("utm_zone_number").as_int();
  utm_zone_letter = node->get_parameter("utm_zone_letter").as_string();

  if (global_frame.empty() || sensor_frame.empty() || unique_serial_id.empty() || manufacturer_device_name.empty() ||
      device_classification.empty() || sensor_name.empty() || sensor_source_type.empty() || sensor_unit.empty())
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Invalid empty parameter, please check input parameters!");
    // return;
  }

  // update parameters
  if (!SourceDescription::updateSourceDescriptions(node, sources))
  {

    RCLCPP_ERROR_STREAM(node->get_logger(), "Could not update source descriptions!");
    // return;
  }
  // initialize publishers

  measurement_pub = node->create_publisher<fkie_measurement_msgs::msg::Measurement>(topic_measurement, 10);
  measurement_array_pub = node->create_publisher<fkie_measurement_msgs::msg::MeasurementArray>(topic_measurement_array, 10);
  sensor_array_pub = node->create_publisher<fkie_measurement_msgs::msg::MeasurementArray>(topic_sensor_array, 10);
  marker_location_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("sensor_locations", 10);
  publishSourceLocations();
  spin();
}

SensorSimulator::~SensorSimulator()
{
  measurement_pub.reset();
  measurement_array_pub.reset();
  sensor_array_pub.reset();
  marker_location_pub.reset();
}

void SensorSimulator::spin()
{
  rclcpp::Rate loop_rate(rate);
  while (rclcpp::ok())
  {
    // Get current sensor position
    updateCurrentSensorPosition();

    // compute accumulated_intensity from all
    double accumulated_intensity = computeMeasurementFromSources();

    // publish message
    publishMeasurement(accumulated_intensity);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
}

void SensorSimulator::updateCurrentSensorPosition()
{

  // get tf between world_frame and path_frame

  try
  {
    auto ts = p_tf_buffer->lookupTransform(global_frame, sensor_frame, node->get_clock()->now(), rclcpp::Duration::from_seconds(4));
  }
  catch (tf2::TransformException &ex)
  {
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(node->get_logger(), steady_clock, 10000, "Could not lookup transform from %s to %s: %s", global_frame.c_str(), sensor_frame.c_str(), ex.what());
  }

  rclcpp::Time time_zero;
  auto tf_world_sensor = p_tf_buffer->lookupTransform(global_frame, sensor_frame, time_zero);

  // convert point to world frame
  if (random_pos_factor != 0.0)
  {
    float rFactor = static_cast<float>(rand()) / static_cast<float>(RAND_MAX / random_pos_factor);
    xPos += rFactor * xDir;
    if (rFactor < random_pos_factor / 10.0)
    {
      xDir *= -1.0;
    }
  }
  if (random_pos_factor != 0.0)
  {
    float rFactor = static_cast<float>(rand()) / static_cast<float>(RAND_MAX / random_pos_factor);
    yPos += rFactor * yDir;
    if (rFactor < random_pos_factor / 10.0)
    {
      yDir *= -1.0;
    }
  }
  current_sensor_position = PositionGrid(tf_world_sensor.transform.translation.x + xPos, tf_world_sensor.transform.translation.y + yPos, tf_world_sensor.transform.translation.z);
}

double SensorSimulator::computeMeasurementFromSources() const
{
  double accumulated_intensity = 0.0;
  for (SourceDescription s : sources)
  {
    double distance = euclideanDistance(s.position, current_sensor_position);
    double m = 0.0;

    if (s.function == "linear" && std::abs(distance) > 0.0)
    {
      m = s.linear_alpha * (s.intensity / distance);
    }
    else if (s.function == "exponential" && std::abs(distance) > 0.0)
    {
      m = s.intensity * s.linear_alpha * std::exp(-s.exponential_decay_rate * distance);
    }
    else if (s.function == "inverse_squared")
    {
      m = s.intensity * (1.0 / std::pow(distance, 2));
    }
    else
    {
      RCLCPP_WARN_STREAM(node->get_logger(), "Unsupported function: [" << s.function << "]");
    }

    accumulated_intensity += m;
  }

  return accumulated_intensity;
}

void SensorSimulator::publishMeasurement(const double measurement) const
{
  fkie_measurement_msgs::msg::Measurement m;

  // header:
  //   frame_id: TF frame to which the sensor is attached
  //   stamp: Time when this message was generated
  m.header.frame_id = sensor_frame;
  m.header.stamp = rclcpp::Clock().now();

  // Unique ID that identifies the device
  m.unique_serial_id = unique_serial_id;

  // Generic name assigned by the device manufacturer
  m.manufacturer_device_name = manufacturer_device_name;

  // classification that groups what the device is able to measure:
  //   e.g. chemical (C), biological (B), radiological (R), meteorologic (M), (W) WiFi etc...
  m.device_classification = device_classification;

  fkie_measurement_msgs::msg::MeasurementValue v;
  v.begin = rclcpp::Clock().now();
  v.end = rclcpp::Clock().now();
  v.sensor = sensor_name;
  v.source_type = sensor_source_type;
  v.unit = sensor_unit;
  v.value_single = measurement;

  m.values.push_back(v);

  measurement_pub->publish(m);

  // publish measurement array
  fkie_measurement_msgs::msg::MeasurementLocated m_located;
  m_located.measurement = m;
  m_located.pose = current_sensor_position.toPoseStamped(global_frame);
  m_located.utm_zone_number = utm_zone_number;
  m_located.utm_zone_letter = utm_zone_letter;

  fkie_measurement_msgs::msg::MeasurementArray m_array;
  m_array.header.stamp = rclcpp::Clock().now();
  m_array.full_history = false;
  m_array.located_measurements.push_back(m_located);
  measurement_array_pub->publish(m_array);
}

void SensorSimulator::publishSourceLocations()
{
  marker_locations.markers.clear();

  visualization_msgs::msg::Marker m;
  m.header.frame_id = global_frame;
  m.header.stamp = rclcpp::Clock().now();
  m.id = 0;
  m.ns = "sources";
  m.action = visualization_msgs::msg::Marker::ADD;
  m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  m.scale.x = marker_size;
  m.scale.y = marker_size;
  m.scale.z = marker_size;
  m.lifetime = rclcpp::Duration::from_nanoseconds(0);
  m.pose.orientation.w = 1.0;

  fkie_measurement_msgs::msg::MeasurementArray m_array;
  m_array.header.stamp = rclcpp::Clock().now();
  m_array.full_history = true;

  int counter_id = 1;
  for (SourceDescription s : sources)
  {
    geometry_msgs::msg::Point pp;
    pp.x = s.position.x;
    pp.y = s.position.y;
    pp.z = s.position.z;
    m.points.push_back(pp);
    m.colors.push_back(s.color);

    // add text description of source
    // std::string description = s.name + " (" + std::to_string((int)s.intensity) + ")";
    std::string description = s.name;

    visualization_msgs::msg::Marker m_text;
    m_text.header.frame_id = global_frame;
    m_text.header.stamp = rclcpp::Clock().now();
    m_text.id = counter_id++;
    m_text.ns = "description";
    m_text.action = visualization_msgs::msg::Marker::ADD;
    m_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m_text.scale.z = marker_size;
    m_text.lifetime = rclcpp::Duration::from_nanoseconds(0);
    m_text.pose.orientation.w = 1.0;
    m_text.text = description;
    m_text.pose.position.x = s.position.x;
    m_text.pose.position.y = s.position.y;
    m_text.pose.position.z = s.position.z + marker_size;
    m_text.color = s.color_text;
    marker_locations.markers.push_back(m_text);

    visualization_msgs::msg::Marker m_line;
    m_line.header.frame_id = global_frame;
    m_line.header.stamp = rclcpp::Clock().now();
    m_line.id = counter_id++;
    m_line.ns = "ref_line";
    m_line.action = visualization_msgs::msg::Marker::ADD;
    m_line.type = visualization_msgs::msg::Marker::ARROW;
    m_line.scale.x = 0.2 * marker_size;
    m_line.scale.y = 0.2 * marker_size;
    m_line.scale.z = 0.0;
    m_line.lifetime = rclcpp::Duration::from_nanoseconds(0);
    m_line.pose.orientation.w = 1.0;

    pp.x = s.position.x;
    pp.y = s.position.y;
    pp.z = 0.0;
    m_line.points.push_back(pp);

    pp.x = s.position.x;
    pp.y = s.position.y;
    pp.z = s.position.z;
    m_line.points.push_back(pp);

    m_line.color = s.color_text;
    marker_locations.markers.push_back(m_line);

    // create measurement array
    fkie_measurement_msgs::msg::MeasurementLocated m_located;
    m_located.measurement.header.frame_id = global_frame;
    m_located.measurement.header.stamp = rclcpp::Clock().now();
    // Unique ID that identifies the device
    m_located.measurement.unique_serial_id = s.name;
    // Generic name assigned by the device manufacturer
    m_located.measurement.manufacturer_device_name = s.name;
    // classification that groups what the device is able to measure:
    //   e.g. chemical (C), biological (B), radiological (R), meteorologic (M), (W) WiFi etc...
    m_located.measurement.device_classification = device_classification;

    fkie_measurement_msgs::msg::MeasurementValue v;
    v.begin = rclcpp::Clock().now();
    v.end = v.begin;
    v.sensor = s.name;
    v.source_type = sensor_source_type;
    v.unit = sensor_unit;
    v.value_single = s.intensity;
    m_located.measurement.values.push_back(v);

    // publish measurement array
    m_located.pose.header.frame_id = global_frame;
    m_located.pose.header.stamp = rclcpp::Clock().now();
    m_located.pose.pose.position.x = s.position.x;
    m_located.pose.pose.position.y = s.position.y;
    m_located.pose.pose.position.z = s.position.z + marker_size;
    m_located.utm_zone_number = utm_zone_number;
    m_located.utm_zone_letter = utm_zone_letter;
    m_array.located_measurements.push_back(m_located);
  }
  marker_locations.markers.push_back(m);
  marker_location_pub->publish(marker_locations);
  sensor_array_pub->publish(m_array);
}

double SensorSimulator::euclideanDistance(const PositionGrid &p1, const PositionGrid &p2) const
{
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}