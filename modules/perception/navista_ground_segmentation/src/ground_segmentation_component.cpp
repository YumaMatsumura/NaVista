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

#include "navista_ground_segmentation/ground_segmentation_component.hpp"

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace navista_ground_segmentation
{

GroundSegmentation::GroundSegmentation(const rclcpp::NodeOptions & options)
: Node("ground_segmentation_node", options)
{
  this->declare_parameter<int>("angle_resolution", 100);
  this->declare_parameter<float>("ground_height_threshold", 0.1);
  this->declare_parameter<float>("grid_ring_space", 0.1);
  this->declare_parameter<float>("max_slope", 1.0);
  this->declare_parameter<std::string>("pcl_type", "XYZI");
  this->get_parameter("angle_resolution", angle_resolution_);
  this->get_parameter("ground_height_threshold", ground_height_threshold_);
  this->get_parameter("grid_ring_space", grid_ring_space_);
  this->get_parameter("max_slope", max_slope_);
  this->get_parameter("pcl_type", pcl_type_);

  using std::placeholders::_1;
  pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/ground_points", rclcpp::SensorDataQoS());
  pub_obstacle_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/obstacle_points", rclcpp::SensorDataQoS());
  sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/input_points", rclcpp::SensorDataQoS(),
    std::bind(&GroundSegmentation::pointsCallback, this, _1));
}

GroundSegmentation::~GroundSegmentation()
{
}

void GroundSegmentation::pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto ground_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto obstacle_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();

  if (pcl_type_ == "XYZ") {
    groundFilter<pcl::PointXYZ>(msg, ground_pc_msg_ptr, obstacle_pc_msg_ptr);
  } else if (pcl_type_ == "XYZRGB") {
    groundFilter<pcl::PointXYZRGB>(msg, ground_pc_msg_ptr, obstacle_pc_msg_ptr);
  } else if (pcl_type_ == "XYZI") {
    groundFilter<pcl::PointXYZI>(msg, ground_pc_msg_ptr, obstacle_pc_msg_ptr);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid pcl_type.");
  }

  pub_ground_->publish(std::move(ground_pc_msg_ptr));
  pub_obstacle_->publish(std::move(obstacle_pc_msg_ptr));
}

template <typename T>
void GroundSegmentation::groundFilter(
  const std::shared_ptr<sensor_msgs::msg::PointCloud2> pc_msg_ptr,
  std::unique_ptr<sensor_msgs::msg::PointCloud2> & ground_pc_msg_ptr,
  std::unique_ptr<sensor_msgs::msg::PointCloud2> & obstacle_pc_msg_ptr)
{
  auto ground_cloud_ptr = std::make_shared<pcl::PointCloud<T>>();
  auto obstacle_cloud_ptr = std::make_shared<pcl::PointCloud<T>>();
  auto cloud_ptr = std::make_shared<pcl::PointCloud<T>>();
  pcl::fromROSMsg(*pc_msg_ptr, *cloud_ptr);

  // NとMを計算
  float max_distance = 0.0;
  for (const auto & point : cloud_ptr->points) {
    float d = std::hypot(point.x, point.y);
    max_distance = std::max(max_distance, d);
  }
  int M = std::ceil(max_distance / grid_ring_space_);
  int N = angle_resolution_;

  std::vector<navista_ground_segmentation::LidarPoint<T>> lidar_points(cloud_ptr->points.size());
  std::vector<float> E(N * M, std::numeric_limits<float>::max());

  for (const auto & point : cloud_ptr->points) {
    navista_ground_segmentation::LidarPoint<T> lidar_point;

    lidar_point.label = Label::Ground;
    lidar_point.point = point;

    // gridに分割
    float beta = std::atan2(point.y, point.x);
    if (beta < 0) {
      beta += 2.0 * M_PI;
    }
    int m = std::floor(std::hypot(point.x, point.y) / grid_ring_space_);
    int n = std::floor(beta * N / (2.0 * M_PI));
    int index = M * n + m;
    lidar_point.m = m;
    lidar_point.n = n;
    E[index] = std::min(E[index], point.z);  // gridの中で最も低い高さの値をE(m, n)とする。
    lidar_points.push_back(lidar_point);
  }

  for (int n = 0; n < N; ++n) {
    for (int m = 0; m < M; ++m) {
      if (m - 1 < 0) {
        continue;
      }
      int index = M * n + m;
      int pre_index = M * n + (m - 1);
      E[index] = std::min(E[index], E[pre_index] + grid_ring_space_ * std::tan(max_slope_));
    }
  }

  for (auto & lidar_point : lidar_points) {
    int index = M * lidar_point.n + lidar_point.m;
    if (lidar_point.point.z >= E[index] + ground_height_threshold_) {
      lidar_point.label = Label::Obstacle;
    }
    // 点群を格納
    if (lidar_point.label == Label::Ground) {
      ground_cloud_ptr->points.push_back(lidar_point.point);
    } else if (lidar_point.label == Label::Obstacle) {
      obstacle_cloud_ptr->points.push_back(lidar_point.point);
    }
  }

  pcl::toROSMsg(*ground_cloud_ptr, *ground_pc_msg_ptr);
  pcl::toROSMsg(*obstacle_cloud_ptr, *obstacle_pc_msg_ptr);
  ground_pc_msg_ptr->header = pc_msg_ptr->header;
  obstacle_pc_msg_ptr->header = pc_msg_ptr->header;
}

}  // namespace navista_ground_segmentation
