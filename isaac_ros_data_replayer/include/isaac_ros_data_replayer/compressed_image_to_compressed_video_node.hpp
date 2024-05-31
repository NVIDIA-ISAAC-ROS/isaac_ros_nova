// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_DATA_REPLAYER__COMPRESSED_IMAGE_TO_COMPRESSED_VIDEO_NODE_HPP_
#define ISAAC_ROS_DATA_REPLAYER__COMPRESSED_IMAGE_TO_COMPRESSED_VIDEO_NODE_HPP_

#include "foxglove_msgs/msg/compressed_video.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace data_replayer
{

class CompressedImageToCompressedVideoNode : public rclcpp::Node
{
public:
  explicit CompressedImageToCompressedVideoNode(
    const rclcpp::NodeOptions options = rclcpp::NodeOptions());

private:
  rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr
    compressed_video_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
    compressed_image_sub_;
};

}  // namespace data_replayer
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_DATA_REPLAYER__COMPRESSED_IMAGE_TO_COMPRESSED_VIDEO_NODE_HPP_
