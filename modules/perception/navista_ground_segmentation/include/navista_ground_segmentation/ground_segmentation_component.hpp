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

#ifndef NAVISTA_GROUND_SEGMENTATION__GROUND_SEGMENTATION_COMPONENT_HPP_
#define NAVISTA_GROUND_SEGMENTATION__GROUND_SEGMENTATION_COMPONENT_HPP_

#include <memory>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>

namespace navista_ground_segmentation
{

enum class Label : int
{
  Ground = 0,
  Obstacle = 1,
};

template <typename T>
struct LidarPoint
{
  T point;
  int m;
  int n;
  navista_ground_segmentation::Label label;
};

class GroundSegmentation : public rclcpp::Node
{
public:
  explicit GroundSegmentation(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GroundSegmentation();

private:
  void pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  template <typename T>
  void groundFilter(
    const std::shared_ptr<sensor_msgs::msg::PointCloud2> pc_msg_ptr,
    std::unique_ptr<sensor_msgs::msg::PointCloud2> & ground_pc_msg_ptr,
    std::unique_ptr<sensor_msgs::msg::PointCloud2> & obstacle_pc_msg_ptr);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;

  int angle_resolution_;
  float ground_height_threshold_;
  float grid_ring_space_;
  float max_slope_;
  std::string pcl_type_;
};

}  // namespace navista_ground_segmentation

#endif  // NAVISTA_GROUND_SEGMENTATION__GROUND_SEGMENTATION_COMPONENT_HPP_
