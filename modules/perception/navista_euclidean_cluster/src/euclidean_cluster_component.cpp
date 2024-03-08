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

#include "navista_euclidean_cluster/euclidean_cluster_component.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <vector>

namespace navista_euclidean_cluster
{

EuclideanCluster::EuclideanCluster(const rclcpp::NodeOptions & options)
: Node("euclidean_cluster_node", options)
{
  // Declare ROS 2 params
  this->declare_parameter<int>("seg_max_iteration", 100);
  this->declare_parameter<int>("min_cluster_size", 100);
  this->declare_parameter<int>("max_cluster_size", 25000);
  this->declare_parameter<double>("seg_distance_threshold", 0.02);
  this->declare_parameter<double>("cluster_tolerance", 0.3);

  // Get ROS 2 params
  this->get_parameter("seg_max_iteration", max_iteration_);
  this->get_parameter("min_cluster_size", min_cluster_size_);
  this->get_parameter("max_cluster_size", max_cluster_size_);
  this->get_parameter("seg_distance_threshold", distance_threshold_);
  this->get_parameter("cluster_tolerance", cluster_tolerance_);

  // Create publisher and subscriber
  using std::placeholders::_1;
  pub_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/cluster_points", rclcpp::SensorDataQoS());
  sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/input_points", rclcpp::SensorDataQoS(),
    std::bind(&EuclideanCluster::pointsCallback, this, _1));
}

EuclideanCluster::~EuclideanCluster()
{
}

void EuclideanCluster::pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto tmp_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *cloud_ptr);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(max_iteration_);
  seg.setDistanceThreshold(distance_threshold_);

  int nr_points = static_cast<int>(cloud_ptr->size());
  while (cloud_ptr->size() > 0.3 * nr_points) {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_ptr);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Could not estimate a planar model for the given dataset.");
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*tmp_cloud_ptr);
    *cloud_ptr = *tmp_cloud_ptr;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_ptr);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_ptr);
  ec.extract(cluster_indices);

  int j = 1;
  int cluster_size = static_cast<int>(cluster_indices.size());
  auto cloud_cluster_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  RCLCPP_INFO(this->get_logger(), "cluster_size: %d", cluster_size);
  for (const auto & cluster : cluster_indices) {
    for (const auto & idx : cluster.indices) {
      pcl::PointXYZRGB point;
      point.x = cloud_ptr->points[idx].x;
      point.y = cloud_ptr->points[idx].y;
      point.z = cloud_ptr->points[idx].z;
      point.r = static_cast<std::uint8_t>((j * 37) % 255);
      point.g = static_cast<std::uint8_t>((j * 67) % 255);
      point.b = static_cast<std::uint8_t>((j * 97) % 255);
      cloud_cluster_ptr->points.push_back(point);
    }
    cloud_cluster_ptr->width = cloud_cluster_ptr->size();
    cloud_cluster_ptr->height = 1;
    cloud_cluster_ptr->is_dense = true;

    j++;
  }

  auto output_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*cloud_cluster_ptr, *output_msg_ptr);
  output_msg_ptr->header = msg->header;
  pub_pc_->publish(std::move(output_msg_ptr));
}

}  // namespace navista_euclidean_cluster

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(navista_euclidean_cluster::EuclideanCluster)
