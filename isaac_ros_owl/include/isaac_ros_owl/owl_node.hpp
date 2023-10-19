// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ISAAC_ROS_OWL__OWL_NODE_HPP_
#define ISAAC_ROS_OWL__OWL_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_nitros/nitros_node.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"
#include "isaac_ros_argus_camera/argus_camera_node.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace owl
{

class OwlNode : public nvidia::isaac_ros::argus::ArgusCameraNode
{
public:
  explicit OwlNode(const rclcpp::NodeOptions & options);
  ~OwlNode();

  // The callback to be implemented by users for any required initialization
  void preLoadGraphCallback() override;
  void postLoadGraphCallback() override;

private:
  std::string optical_frame_name_;

  std::string camera_info_url_;

  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
};

}  // namespace owl
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_OWL__OWL_NODE_HPP_
