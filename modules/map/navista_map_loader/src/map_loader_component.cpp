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
  pub_octomap_ = this->create_publisher<octomap_msgs::msg::Octomap>(
    "/octomap", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  srv_load_map_ = this->create_service<navista_map_msgs::srv::LoadMap>(
    "load_map", std::bind(&MapLoader::loadMapCallback, this, _1, _2, _3),
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

  const auto map_file = map_yaml_node["octomap_file"].as<std::string>();
  const auto resolution = map_yaml_node["resolution"].as<double>();

  auto octree = std::make_unique<octomap::OcTree>(resolution);

  // Load octomap file
  {
    if (map_file.length() <= 3) {
      throw std::runtime_error("Invalid name for octomap: too short.");
    }

    std::string suffix = map_file.substr(map_file.length() - 3, 3);
    if (suffix == ".bt") {
      if (!octree->readBinary(map_file)) {
        throw std::runtime_error("Could not open binary octomap.");
      }
    } else if (suffix == ".ot") {
      std::unique_ptr<octomap::AbstractOcTree> tree{octomap::AbstractOcTree::read(map_file)};
      if (!tree) {
        throw std::runtime_error("Could not read octomap file.");
      }
      octree = std::unique_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(tree.release()));
      if (!octree) {
        throw std::runtime_error("Could not read file.");
      }
    } else {
      throw std::runtime_error("Extension not supported.");
    }
  }

  // Publish octomap
  {
    octomap_msgs::msg::Octomap octomap_msg;
    if (!octomap_msgs::fullMapToMsg(*octree, octomap_msg)) {
      RCLCPP_ERROR(this->get_logger(), "Error serializing Octomap.");
      return;
    }

    int octomap_subscription_count =
      pub_octomap_->get_subscription_count() + pub_octomap_->get_intra_process_subscription_count();
    if (octomap_subscription_count > 0) {
      octomap_msg.header.stamp = this->now();
      octomap_msg.header.frame_id = global_frame_id_;
      pub_octomap_->publish(octomap_msg);
    }
  }
}

}  // namespace navista_map_loader

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(navista_map_loader::MapLoader)
