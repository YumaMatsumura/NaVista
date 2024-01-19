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

#include "navista_octomap_debug/octomap_debug_component.hpp"

namespace navista_octomap_debug
{

OctomapDebug::OctomapDebug(const rclcpp::NodeOptions & options)
: Node("octomap_debug", options)
{
  min_x_ = this->declare_parameter<double>("min_x", -0.1);
  min_y_ = this->declare_parameter<double>("min_y", -0.1);
  min_z_ = this->declare_parameter<double>("min_z", -0.1);
  max_x_ = this->declare_parameter<double>("max_x", 0.1);
  max_y_ = this->declare_parameter<double>("max_y", 0.1);
  max_z_ = this->declare_parameter<double>("max_z", 0.1);

  using std::placeholders::_1;
  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/octomap_debug", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  sub_octomap_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    "/octomap", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&OctomapDebug::octomapCallback, this, _1));
}

OctomapDebug::~OctomapDebug()
{
}

void OctomapDebug::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  octomap::OcTree * octree_cast = dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg));
  std::unique_ptr<octomap::OcTree> octree(octree_cast);

  auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
  int id = 0;

  octomap::point3d min(min_x_, min_y_, min_z_);
  octomap::point3d max(max_x_, max_y_, max_z_);

  for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min, max);
       it != octree->end_leafs(); ++it) {
    const octomap::point3d center_query = it.getCoordinate();
    {
      visualization_msgs::msg::Marker marker;
      marker.header = msg->header;
      marker.ns = "sphere";
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = center_query.x();
      marker.pose.position.y = center_query.y();
      marker.pose.position.z = center_query.z();
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      if (octree->isNodeOccupied(*it)) {
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
      } else if (!octree->isNodeOccupied(*it)) {
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1;
      }
      marker.color.a = 1.0f;
      marker.frame_locked = false;
      marker_array->markers.push_back(marker);
    }
    {
      visualization_msgs::msg::Marker marker;
      marker.header = msg->header;
      marker.ns = "text";
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = center_query.x();
      marker.pose.position.y = center_query.y();
      marker.pose.position.z = center_query.z() + msg->resolution * 0.25;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.05;
      marker.text = std::to_string(id);
      if (octree->isNodeOccupied(*it)) {
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
      } else if (!octree->isNodeOccupied(*it)) {
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1;
      }
      marker.color.a = 1.0f;
      marker.frame_locked = false;
      marker_array->markers.push_back(marker);
    }

    id++;
  }

  /*
  for (double x = min_x_; x <= max_x_; x += octree->getResolution()) {
    for (double y = min_y_; y <= max_y_; y += octree->getResolution()) {
      for (double z = min_z_; z <= max_z_; z += octree->getResolution()) {
        octomap::point3d query(x, y, z);
        octomap::OcTreeNode* node = octree->search(query);

        if (node) {
          const octomap::point3d center_query = octree->keyToCoord(octree->coordToKey(query));

          visualization_msgs::msg::Marker marker;
          marker.header = msg->header;
          marker.id = id;
          marker.type = visualization_msgs::msg::Marker::SPHERE;
          marker.action = visualization_msgs::msg::Marker::ADD;
          marker.pose.position.x = center_query.x();
          marker.pose.position.y = center_query.y();
          marker.pose.position.z = center_query.z();
          marker.scale.x = 0.05;
          marker.scale.y = 0.05;
          marker.scale.z = 0.05;
          if (octree->isNodeOccupied(node)) {
            marker.color.r = 1;
            marker.color.g = 0;
            marker.color.b = 0;
          } else if (!octree->isNodeOccupied(node)) {
            marker.color.r = 0;
            marker.color.g = 0;
            marker.color.b = 1;
          }
          marker.color.a = 1.0f;
          marker.frame_locked = false;
          marker_array->markers.push_back(marker);

          id++;
        }
      }
    }
  }
  */

  if (marker_array->markers.size() != 0) {
    pub_markers_->publish(std::move(marker_array));
  }
}

}  // namespace navista_octomap_debug

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(navista_octomap_debug::OctomapDebug)
