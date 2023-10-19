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

#ifndef ISAAC_ROS_IMU_BMI088__IMU_BMI088_NODE_HPP_
#define ISAAC_ROS_IMU_BMI088__IMU_BMI088_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_nitros/nitros_node.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace imu_bmi088
{

class Bmi088Node : public nitros::NitrosNode
{
public:
  explicit Bmi088Node(const rclcpp::NodeOptions &);

  ~Bmi088Node();

  // Custom postLoadGraphCallback to populate gxf parameters from ROS parameters
  void postLoadGraphCallback() override;

  Bmi088Node(const Bmi088Node &) = delete;

  Bmi088Node & operator=(const Bmi088Node &) = delete;

private:
  const int imu_frequency_;  // Imu Update Frequency (Hz). Supported values are 100 and 200
  const int bmi_id_;  // Id for BMI088 IMU
};

}  // namespace imu_bmi088
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_IMU_BMI088__IMU_BMI088_NODE_HPP_
