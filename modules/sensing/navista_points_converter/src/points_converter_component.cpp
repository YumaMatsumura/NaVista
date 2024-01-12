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

#include "navista_points_converter/points_converter_component.hpp"

namespace navista_points_converter
{

PointsConverter::PointsConverter(const rclcpp::NodeOptions & options)
: Node("points_converter", options)
{
  // Declare ROS 2 parameters
  resolution_ = this->declare_parameter<double>("octomap_resolution", 0.1);
  pcd_type_ = this->declare_parameter<std::string>("pcd_type", "XYZ");

  // Create publisher and subscription
  using std::placeholders::_1;
  pub_octomap_ =
    this->create_publisher<octomap_msgs::msg::Octomap>("/octomap", rclcpp::SensorDataQoS());
  sub_pcd_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/pcd", rclcpp::SensorDataQoS(), std::bind(&PointsConverter::pcdCallback, this, _1));
}

PointsConverter::~PointsConverter()
{
}

void PointsConverter::pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg_ptr)
{
  auto octomap_msg_ptr = std::make_unique<octomap_msgs::msg::Octomap>();

  if (pcd_type_ == "XYZ") {
    pointsConversion<pcl::PointXYZ>(pcd_msg_ptr, octomap_msg_ptr);
  } else if (pcd_type_ == "XYZRGB") {
    pointsConversion<pcl::PointXYZRGB>(pcd_msg_ptr, octomap_msg_ptr);
  } else if (pcd_type_ == "XYZI") {
    pointsConversion<pcl::PointXYZI>(pcd_msg_ptr, octomap_msg_ptr);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid map type for pcd map.");
    return;
  }

  octomap_msg_ptr->header = pcd_msg_ptr->header;
  pub_octomap_->publish(std::move(octomap_msg_ptr));
}

template <typename T>
void PointsConverter::pointsConversion(
  const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg_ptr,
  std::unique_ptr<octomap_msgs::msg::Octomap> & octomap_msg_ptr)
{
  pcl::PCLPointCloud2 pcl_pcd;
  pcl_conversions::toPCL(*pcd_msg_ptr, pcl_pcd);
  typename pcl::PointCloud<T>::Ptr pcl_cloud_ptr(new pcl::PointCloud<T>);
  pcl::fromPCLPointCloud2(pcl_pcd, *pcl_cloud_ptr);

  octomap::OcTree tree(resolution_);
  for (const auto & point : pcl_cloud_ptr->points) {
    tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
  }
  tree.updateInnerOccupancy();

  octomap_msgs::fullMapToMsg(tree, *octomap_msg_ptr);
}

}  // namespace navista_points_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(navista_points_converter::PointsConverter)
