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

#ifndef ISAAC_ROS_NOVA__NOVA_ANALYZER_HPP_
#define ISAAC_ROS_NOVA__NOVA_ANALYZER_HPP_

#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nova_monitor
{
class NovaAnalyzer
{
public:
  NovaAnalyzer();
  ~NovaAnalyzer();
  bool init(
    const std::string & base_path, const std::string & sensor_name,
    rclcpp::Node::SharedPtr n);
  bool init(
    const std::string & base_path, const std::string & sensor_name,
    rclcpp::Clock::SharedPtr clk);
  bool match(const std::string & name);
  bool
  analyze(const std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> status);
  std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> report();
  std::string getPath() const {return path_;}

private:
  std::string base_path_, id_string_, path_;
  bool has_initialized_;
  std::map<std::string,
    std::tuple<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>,
    rclcpp::Time>>
  items_;
  float timeout_;
  rclcpp::Clock::SharedPtr clock_;
};
}  // namespace nova_monitor
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NOVA__NOVA_ANALYZER_HPP_
