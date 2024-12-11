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

#ifndef ISAAC_ROS_NOVA_RECORDER__CAMERA_MULTIPLEXER_NODE_HPP_
#define ISAAC_ROS_NOVA_RECORDER__CAMERA_MULTIPLEXER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "isaac_ros_managed_nitros/managed_nitros_publisher.hpp"
#include "isaac_ros_managed_nitros/managed_nitros_subscriber.hpp"
#include "isaac_ros_nitros_camera_info_type/nitros_camera_info.hpp"
#include "isaac_ros_nitros_camera_info_type/nitros_camera_info_view.hpp"
#include "isaac_ros_nitros_image_type/nitros_image.hpp"
#include "isaac_ros_nitros_image_type/nitros_image_view.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace isaac_ros_nova_recorder
{

class CameraMultiplexerNode : public rclcpp::Node
{
public:
  explicit CameraMultiplexerNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions());

  ~CameraMultiplexerNode();

private:
  void ImageCallback(const nitros::NitrosImageView& image, const std::string& camera);
  void CameraInfoCallback(const nitros::NitrosCameraInfoView& camera_info,
                          const std::string& camera);

  std::vector<std::shared_ptr<nitros::ManagedNitrosSubscriber<nitros::NitrosImageView>>>
      sub_image_;
  std::vector<std::shared_ptr<nitros::ManagedNitrosSubscriber<nitros::NitrosCameraInfoView>>>
      sub_camera_info_;

  std::shared_ptr<nitros::ManagedNitrosPublisher<nitros::NitrosImage>> pub_image_;
  std::shared_ptr<nitros::ManagedNitrosPublisher<nitros::NitrosCameraInfo>> pub_camera_info_;

  std::vector<std::string> cameras_;
  std::string select_;
  std::string image_topic_;
  std::string camera_info_topic_;
};

}  // namespace isaac_ros_nova_recorder
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NOVA_RECORDER__CAMERA_MULTIPLEXER_NODE_HPP_
