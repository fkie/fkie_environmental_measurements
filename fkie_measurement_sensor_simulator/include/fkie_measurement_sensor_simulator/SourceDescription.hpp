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

#ifndef SURFACE_DESCRIPTION_H_
#define SURFACE_DESCRIPTION_H_

#include <iostream>
#include <unordered_map>
#include <map>
#include <deque>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "fkie_measurement_sensor_simulator/structs.hpp"

class SourceDescription
{
public:
  std::string name;
  std::string function;
  double intensity;
  PositionGrid position;

  double linear_alpha;
  double exponential_decay_rate;
  std_msgs::msg::ColorRGBA color;
  std_msgs::msg::ColorRGBA color_text;

  SourceDescription(){};

  [[nodiscard]] static bool updateSourceDescriptions(rclcpp::Node::SharedPtr node, std::vector<SourceDescription> &sources)
  {
    sources.clear();
    int counter = 0;
    bool valid = true;
    while (valid)
    {
      std::string param_name = "sources/source_" + std::to_string(counter);
      RCLCPP_INFO_STREAM(node->get_logger(), "check for source: " << param_name);
      SourceDescription sd;
      std::vector<double> color_rgba;
      node->declare_parameter(param_name + "/name", sd.name);
      node->declare_parameter(param_name + "/intensity", sd.intensity);
      node->declare_parameter(param_name + "/x", sd.position.x);
      node->declare_parameter(param_name + "/y", sd.position.y);
      node->declare_parameter(param_name + "/z", sd.position.z);
      node->declare_parameter(param_name + "/function", sd.function);
      node->declare_parameter(param_name + "/linear_alpha", sd.linear_alpha);
      node->declare_parameter(param_name + "/exponential_decay_rate", sd.exponential_decay_rate);
      node->declare_parameter<std::vector<double>>(param_name + "/color_rgba", color_rgba);
      node->declare_parameter(param_name + "/text_color_rgba", color_rgba);

      sd.name = node->get_parameter(param_name + "/name").as_string();
      // Simulator properties
      if (!sd.name.empty())
      {
        sd.intensity = node->get_parameter(param_name + "/intensity").as_double();
        sd.position.x = node->get_parameter(param_name + "/x").as_double();
        sd.position.y = node->get_parameter(param_name + "/y").as_double();
        sd.position.z = node->get_parameter(param_name + "/z").as_double();

        sd.function = node->get_parameter(param_name + "/function").as_string();
        sd.linear_alpha = node->get_parameter(param_name + "/linear_alpha").as_double();
        sd.exponential_decay_rate = node->get_parameter(param_name + "/exponential_decay_rate").as_double();

        color_rgba = node->get_parameter(param_name + "/color_rgba").as_double_array();
        if (color_rgba.size() > 0)
        {
          sd.color.r = color_rgba[0];
          sd.color.g = color_rgba[1];
          sd.color.b = color_rgba[2];
          sd.color.a = color_rgba[3];
        }

        color_rgba = node->get_parameter(param_name + "/text_color_rgba").as_double_array();

        if (color_rgba.size() > 0)
        {
          sd.color_text.r = color_rgba[0];
          sd.color_text.g = color_rgba[1];
          sd.color_text.b = color_rgba[2];
          sd.color_text.a = color_rgba[3];
        }

        sources.push_back(sd);
        counter++;
      } else {
        valid = false;
      }
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Total sources: " << sources.size());
    return (counter > 0);
  }
};

#endif /* SURFACE_DESCRIPTION_H_ */
