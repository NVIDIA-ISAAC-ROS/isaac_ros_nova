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

#ifndef ISAAC_ROS_CORRELATED_TIMESTAMP_DRIVER__CORRELATED_TIMESTAMP_DRIVER_NODE_HPP_
#define ISAAC_ROS_CORRELATED_TIMESTAMP_DRIVER__CORRELATED_TIMESTAMP_DRIVER_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_nitros/nitros_node.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace correlated_timestamp_driver
{

class CorrelatedTimestampDriverNode : public nitros::NitrosNode
{
public:
  explicit CorrelatedTimestampDriverNode(const rclcpp::NodeOptions &);

  ~CorrelatedTimestampDriverNode();

  void postLoadGraphCallback() override;

  CorrelatedTimestampDriverNode(const CorrelatedTimestampDriverNode &) = delete;

  CorrelatedTimestampDriverNode & operator=(const CorrelatedTimestampDriverNode &) = delete;

private:
  const bool use_time_since_epoch_;
  const std::string nvpps_dev_name_;
};

}  // namespace correlated_timestamp_driver
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_CORRELATED_TIMESTAMP_DRIVER__CORRELATED_TIMESTAMP_DRIVER_NODE_HPP_
