// Copyright 2023 Yuma Matsumura All rights reserved.
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

#pragma once

#include <octomap_msgs/conversions.h>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>

#include "navista_map_msgs/srv/load_map.hpp"
#include <octomap_msgs/msg/octomap.hpp>

#include <octomap_ros/conversions.hpp>
#include <rclcpp/rclcpp.hpp>

namespace navista_map_loader
{

class MapLoader : public rclcpp::Node
{
public:
  explicit MapLoader(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MapLoader();

private:
  void loadMapCallback(
    [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<navista_map_msgs::srv::LoadMap::Request> req,
    std::shared_ptr<navista_map_msgs::srv::LoadMap::Response> res);
  void loadMap(const std::string & map_yaml_file);

  // ROS 2 publisher and service server
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_octomap_;
  rclcpp::Service<navista_map_msgs::srv::LoadMap>::SharedPtr srv_load_map_;
  // Parameters
  std::string global_frame_id_;
};

}  // namespace navista_map_loader
