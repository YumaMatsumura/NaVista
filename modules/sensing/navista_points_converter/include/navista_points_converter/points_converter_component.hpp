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

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>

#include <octomap_msgs/msg/octomap.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>

namespace navista_points_converter
{

class PointsConverter : public rclcpp::Node
{
public:
  explicit PointsConverter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PointsConverter();

private:
  void pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg_ptr);
  template <typename T>
  void pointsConversion(
    const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg_ptr,
    std::unique_ptr<octomap_msgs::msg::Octomap> & octomap_msg_ptr);

  // ROS 2 publisher and service server
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_octomap_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcd_;
  // Parameters
  double resolution_;
  std::string pcd_type_;
};

}  // namespace navista_points_converter
