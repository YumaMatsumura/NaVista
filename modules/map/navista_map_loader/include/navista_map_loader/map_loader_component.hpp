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
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>

#include "navista_map_msgs/srv/load_map.hpp"
#include <octomap_msgs/msg/octomap.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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
  void loadPCDMap(
    const std::string & map_file, const std::string & map_type,
    std::unique_ptr<sensor_msgs::msg::PointCloud2> & pcd_msg_ptr);
  void loadOctomap(
    const std::string & map_file, const double resolution,
    std::unique_ptr<octomap_msgs::msg::Octomap> & octomap_msg_ptr);
  template <typename T>
  bool loadPCDFile(
    const std::string & map_file, std::unique_ptr<sensor_msgs::msg::PointCloud2> & pcd_msg_ptr);

  // ROS 2 publisher and service server
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcd_map_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_octomap_;
  rclcpp::Service<navista_map_msgs::srv::LoadMap>::SharedPtr srv_load_map_;
  // Parameters
  std::string global_frame_id_;
};

}  // namespace navista_map_loader
