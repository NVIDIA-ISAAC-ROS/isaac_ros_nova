// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <functional>

#include "isaac_ros_nova_recorder/camera_multiplexer_node.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace isaac_ros_nova_recorder
{

CameraMultiplexerNode::CameraMultiplexerNode(const rclcpp::NodeOptions options)
    : rclcpp::Node("camera_multiplexer", options)
{
  cameras_ = declare_parameter<std::vector<std::string>>("cameras");
  select_ = declare_parameter<std::string>("select");

  for (size_t i = 0; i < cameras_.size(); i++)
  {
    const std::string& camera = cameras_[i];
    sub_image_.emplace_back(
        std::make_shared<nitros::ManagedNitrosSubscriber<nitros::NitrosImageView>>(
            this, camera + "/image_raw", nitros::nitros_image_nv12_t::supported_type_name,
            std::bind(
                &CameraMultiplexerNode::ImageCallback, this, std::placeholders::_1, camera)));
    sub_camera_info_.emplace_back(
        std::make_shared<nitros::ManagedNitrosSubscriber<nitros::NitrosCameraInfoView>>(
            this, camera + "/camera_info", nitros::nitros_camera_info_t::supported_type_name,
            std::bind(
                &CameraMultiplexerNode::CameraInfoCallback, this, std::placeholders::_1, camera)));
  }

  pub_image_ = std::make_shared<nitros::ManagedNitrosPublisher<nitros::NitrosImage>>(
      this, "image_raw", nitros::nitros_image_nv12_t::supported_type_name);

  pub_camera_info_ = std::make_shared<nitros::ManagedNitrosPublisher<nitros::NitrosCameraInfo>>(
      this, "camera_info", nitros::nitros_camera_info_t::supported_type_name);
}

CameraMultiplexerNode::~CameraMultiplexerNode() = default;

void CameraMultiplexerNode::ImageCallback(const nitros::NitrosImageView& image,
                                          const std::string& camera)
{
  if (get_parameter<std::string>("select", select_) && (camera == select_))
  {
    pub_image_->publish(image.GetMessage());
  }
}

void CameraMultiplexerNode::CameraInfoCallback(const nitros::NitrosCameraInfoView& camera_info,
                                               const std::string& camera)
{
  if (get_parameter<std::string>("select", select_) && (camera == select_))
  {
    pub_camera_info_->publish(camera_info.GetMessage());
  }
}

}  // namespace isaac_ros_nova_recorder
}  // namespace isaac_ros
}  // namespace nvidia

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::isaac_ros_nova_recorder::CameraMultiplexerNode)
