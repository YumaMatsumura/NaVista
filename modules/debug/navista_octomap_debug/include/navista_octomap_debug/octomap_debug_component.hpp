// Copyright 2024 Yuma Matsumura All rights reserved.
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

#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <memory>
#include <vector>

#include <octomap_msgs/msg/octomap.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/rclcpp.hpp>

namespace navista_octomap_debug
{

class OctomapDebug : public rclcpp::Node
{
public:
  explicit OctomapDebug(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~OctomapDebug();

private:
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_octomap_;

  double min_x_;
  double min_y_;
  double min_z_;
  double max_x_;
  double max_y_;
  double max_z_;
};

}  // namespace navista_octomap_debug
