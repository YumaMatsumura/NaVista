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

#ifndef NAVISTA_EUCLIDEAN_CLUSTER__EUCLIDEAN_CLUSTER_COMPONENT_HPP_
#define NAVISTA_EUCLIDEAN_CLUSTER__EUCLIDEAN_CLUSTER_COMPONENT_HPP_

#include <memory>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>

namespace navista_euclidean_cluster
{

class EuclideanCluster : public rclcpp::Node
{
public:
  explicit EuclideanCluster(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~EuclideanCluster();

private:
  void pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;

  int max_iteration_;
  int min_cluster_size_;
  int max_cluster_size_;
  double distance_threshold_;
  double cluster_tolerance_;
};

}  // namespace navista_euclidean_cluster

#endif  // NAVISTA_EUCLIDEAN_CLUSTER__EUCLIDEAN_CLUSTER_COMPONENT_HPP_
