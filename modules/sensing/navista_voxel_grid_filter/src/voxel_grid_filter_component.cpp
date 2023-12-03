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

#include "navista_voxel_grid_filter/voxel_grid_filter_component.hpp"

namespace navista_voxel_grid_filter
{

VoxelGridFilter::VoxelGridFilter(const rclcpp::NodeOptions & options)
: Node("voxel_grid_filter", options)
{
  voxel_leaf_x_ = this->declare_parameter<double>("voxel_leaf_x", 0.1);
  voxel_leaf_y_ = this->declare_parameter<double>("voxel_leaf_y", 0.1);
  voxel_leaf_z_ = this->declare_parameter<double>("voxel_leaf_z", 0.1);
  pcd_type_ = this->declare_parameter<std::string>("pcd_type", "XYZ");

  // Create publisher and subscription
  using std::placeholders::_1;
  pub_pcd_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("/output_pcd", rclcpp::SensorDataQoS());
  sub_pcd_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/input_pcd", rclcpp::SensorDataQoS(), std::bind(&VoxelGridFilter::pcdCallback, this, _1));
}

VoxelGridFilter::~VoxelGridFilter()
{
}

void VoxelGridFilter::pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg_ptr)
{
  auto filtered_pcd_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();

  if (pcd_type_ == "XYZ") {
    voxelGridFilter<pcl::PointXYZ>(pcd_msg_ptr, filtered_pcd_msg_ptr);
  } else if (pcd_type_ == "XYZRGB") {
    voxelGridFilter<pcl::PointXYZRGB>(pcd_msg_ptr, filtered_pcd_msg_ptr);
  } else if (pcd_type_ == "XYZI") {
    voxelGridFilter<pcl::PointXYZI>(pcd_msg_ptr, filtered_pcd_msg_ptr);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid pcd_type.");
  }

  filtered_pcd_msg_ptr->header = pcd_msg_ptr->header;
  pub_pcd_->publish(std::move(filtered_pcd_msg_ptr));
}

template <typename T>
void VoxelGridFilter::voxelGridFilter(
  const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg_ptr,
  std::unique_ptr<sensor_msgs::msg::PointCloud2> & filtered_pcd_msg_ptr)
{
  auto pcd_cloud_ptr = std::make_shared<pcl::PointCloud<T>>();
  auto filtered_pcd_cloud_ptr = std::make_shared<pcl::PointCloud<T>>();

  pcl::VoxelGrid<T> voxel_grid_filter;

  pcl::fromROSMsg(*pcd_msg_ptr, *pcd_cloud_ptr);
  voxel_grid_filter.setLeafSize(voxel_leaf_x_, voxel_leaf_y_, voxel_leaf_z_);
  voxel_grid_filter.setInputCloud(pcd_cloud_ptr);
  voxel_grid_filter.filter(*filtered_pcd_cloud_ptr);
  pcl::toROSMsg(*filtered_pcd_cloud_ptr, *filtered_pcd_msg_ptr);
}

}  // namespace navista_voxel_grid_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(navista_voxel_grid_filter::VoxelGridFilter)
