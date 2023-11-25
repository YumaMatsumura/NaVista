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

#include "navista_map_loader/map_loader_component.hpp"

namespace navista_map_loader
{

MapLoader::MapLoader(const rclcpp::NodeOptions & options)
: Node("map_loader_node", options)
{
  // Set parameters
  global_frame_id_ = this->declare_parameter<std::string>("global_frame_id", "map");

  // Create publisher and subscriber
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  pub_pcd_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/pcd_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_octomap_ = this->create_publisher<octomap_msgs::msg::Octomap>(
    "/octomap", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  srv_load_map_ = this->create_service<navista_map_msgs::srv::LoadMap>(
    "/load_map", std::bind(&MapLoader::loadMapCallback, this, _1, _2, _3),
    rclcpp::QoS(rclcpp::ServicesQoS()));

  // Load octomap
  const auto map_yaml_file = this->declare_parameter<std::string>("map_yaml_file", "");
  if (map_yaml_file != "") {
    loadMap(map_yaml_file);
  }
}

MapLoader::~MapLoader()
{
}

void MapLoader::loadMapCallback(
  [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<navista_map_msgs::srv::LoadMap::Request> req,
  std::shared_ptr<navista_map_msgs::srv::LoadMap::Response> res)
{
  try {
    loadMap(req->map_yaml_file);
    res->result = true;
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    res->result = false;
  }
}

void MapLoader::loadMap(const std::string & map_yaml_file)
{
  YAML::Node map_yaml_node = YAML::LoadFile(map_yaml_file);

  const auto pcd_map_file = map_yaml_node["pcd"]["map_file"].as<std::string>();
  const auto pcd_map_type = map_yaml_node["pcd"]["map_type"].as<std::string>();
  const auto octomap_file = map_yaml_node["octomap"]["map_file"].as<std::string>();
  const auto octomap_resolution = map_yaml_node["octomap"]["resolution"].as<double>();
  auto pcd_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto octomap_msg_ptr = std::make_unique<octomap_msgs::msg::Octomap>();

  try {
    loadPCDMap(pcd_map_file, pcd_map_type, pcd_msg_ptr);
    loadOctomap(octomap_file, octomap_resolution, octomap_msg_ptr);
    pub_pcd_map_->publish(std::move(pcd_msg_ptr));
    pub_octomap_->publish(std::move(octomap_msg_ptr));
  } catch (const std::runtime_error & e) {
    throw e;
  }
}

void MapLoader::loadPCDMap(
  const std::string & map_file, const std::string & map_type,
  std::unique_ptr<sensor_msgs::msg::PointCloud2> & pcd_msg_ptr)
{
  if (map_type == "XYZ") {
    if (this->loadPCDFile<pcl::PointXYZ>(map_file, pcd_msg_ptr)) {
      RCLCPP_INFO(this->get_logger(), "[PCDMap] Loaded pcd_map with PointXYZ type.");
    } else {
      throw std::runtime_error("[PCDMap] Failed to load pcd file.");
    }
  } else if (map_type == "XYZRGB") {
    if (this->loadPCDFile<pcl::PointXYZRGB>(map_file, pcd_msg_ptr)) {
      RCLCPP_INFO(this->get_logger(), "[PCDMap] Loaded pcd_map with PointXYZRGB type.");
    } else {
      throw std::runtime_error("[PCDMap] Failed to load pcd file.");
    }
  } else if (map_type == "XYZI") {
    if (this->loadPCDFile<pcl::PointXYZI>(map_file, pcd_msg_ptr)) {
      RCLCPP_INFO(this->get_logger(), "[PCDMap] Loaded pcd_map with PointXYZI type.");
    } else {
      throw std::runtime_error("[PCDMap] Failed to load pcd file.");
    }
  } else {
    throw std::runtime_error("[PCDMap] Invalid map type for pcd map.");
  }
}

void MapLoader::loadOctomap(
  const std::string & map_file, const double resolution,
  std::unique_ptr<octomap_msgs::msg::Octomap> & octomap_msg_ptr)
{
  auto octree = std::make_unique<octomap::OcTree>(resolution);

  if (map_file.length() <= 3) {
    throw std::runtime_error("[Octomap] Invalid name for octomap: too short.");
  }

  std::string suffix = map_file.substr(map_file.length() - 3, 3);
  if (suffix == ".bt") {
    if (octree->readBinary(map_file)) {
      RCLCPP_INFO(this->get_logger(), "[Octomap] Loaded octomap.");
    } else {
      throw std::runtime_error("[Octomap] Failed to load binary octomap file.");
    }
  } else if (suffix == ".ot") {
    std::unique_ptr<octomap::AbstractOcTree> tree{octomap::AbstractOcTree::read(map_file)};
    if (!tree) {
      throw std::runtime_error("[Octomap] Failed to load octomap file.");
    }
    octree = std::unique_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(tree.release()));
    if (octree) {
      RCLCPP_INFO(this->get_logger(), "[Octomap] Loaded octomap.");
    } else {
      throw std::runtime_error("[Octomap] Failed to load file.");
    }
  } else {
    throw std::runtime_error("[Octomap] Extension not supported.");
  }

  if (!octomap_msgs::fullMapToMsg(*octree, *octomap_msg_ptr)) {
    throw std::runtime_error("Error serializing Octomap.");
  }

  octomap_msg_ptr->header.stamp = this->now();
  octomap_msg_ptr->header.frame_id = global_frame_id_;
}

template <typename T>
bool MapLoader::loadPCDFile(
  const std::string & map_file, std::unique_ptr<sensor_msgs::msg::PointCloud2> & pcd_msg_ptr)
{
  auto map_cloud_ptr = std::make_shared<pcl::PointCloud<T>>();

  if (pcl::io::loadPCDFile<T>(map_file, *map_cloud_ptr) == -1) {
    return false;
  }

  pcl::toROSMsg(*map_cloud_ptr, *pcd_msg_ptr);
  pcd_msg_ptr->header.stamp = this->now();
  pcd_msg_ptr->header.frame_id = global_frame_id_;
  return true;
}

}  // namespace navista_map_loader

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(navista_map_loader::MapLoader)
