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

#ifndef ISAAC_ROS_NOVA__NOVA_MONITOR_NODE_HPP_
#define ISAAC_ROS_NOVA__NOVA_MONITOR_NODE_HPP_

#include <memory>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "isaac_ros_nova/nova_analyzer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nova_monitor
{
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;

class MonitorNode : public rclcpp::Node
{
public:
  explicit MonitorNode(
    const rclcpp::NodeOptions options = rclcpp::NodeOptions());
  ~MonitorNode();
  MonitorNode(const MonitorNode &) = delete;
  MonitorNode & operator=(const MonitorNode &) = delete;

private:
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr aggregate_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  void CallbackDiagnostics(const DiagnosticArray::SharedPtr msg);
  void PublishSummary();
  std::vector<std::shared_ptr<nova_monitor::NovaAnalyzer>> analyzers_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<nova_monitor::NovaAnalyzer> misc_analyzer_;
  int64_t aggregate_pd_;
};

}  //  namespace nova_monitor
}  //  namespace isaac_ros
}  //  namespace nvidia

#endif  // ISAAC_ROS_NOVA__NOVA_MONITOR_NODE_HPP_
