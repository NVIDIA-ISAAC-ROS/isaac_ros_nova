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

#include "isaac_ros_nova/nova_monitor_node.hpp"

#include <chrono>
#include <memory>
#include <vector>

namespace nvidia
{
namespace isaac_ros
{
namespace nova_monitor
{

using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

MonitorNode::MonitorNode(rclcpp::NodeOptions options)
: Node("system_monitor", options)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(100));
  RCLCPP_INFO(this->get_logger(), "Initialising node");
  diagnostics_sub_ = this->create_subscription<DiagnosticArray>(
    "/diagnostics", qos, [this](const DiagnosticArray::SharedPtr msg) {
      this->CallbackDiagnostics(msg);
    });

  aggregate_pub_ =
    this->create_publisher<DiagnosticArray>("/diagnostics_agg", qos);
  float publish_rate = declare_parameter<float>("publish_rate", 1.0);
  if (publish_rate > 0) {
    aggregate_pd_ = 1000 / publish_rate;
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "[NOVA MONITOR] Invalid publish_rate found! Defaulting to 1 Hz."
    );
    aggregate_pd_ = 1000;
  }

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(aggregate_pd_),
    [this]() {this->PublishSummary();});

  clock_ = this->get_clock();

  std::vector<std::string> sensor_names{"hawk", "owl", "bmi"};

  for (const auto & sensor : sensor_names) {
    auto analyzer = std::make_shared<NovaAnalyzer>();
    if (!analyzer->init("/sensors/", sensor, this->get_clock())) {
      RCLCPP_ERROR(
        this->get_logger(), "%s analyzer uninitialized",
        sensor.c_str());
    }
    analyzers_.push_back(analyzer);
  }
  misc_analyzer_ = std::make_shared<NovaAnalyzer>();
  if (!misc_analyzer_->init("/", "misc", this->get_clock())) {
    RCLCPP_ERROR(this->get_logger(), "misc analyzer uninitialized");
  }
}

void MonitorNode::CallbackDiagnostics(const DiagnosticArray::SharedPtr msg)
{
  if ((msg->status).size() == 0) {
    RCLCPP_WARN(this->get_logger(), "Empty DiagnosticArray Message found");
    return;
  }
  for (auto & status : msg->status) {
    auto status_reported = std::make_shared<DiagnosticStatus>(status);
    bool group_found{false};
    for (auto it = analyzers_.begin(); it < analyzers_.end(); it++) {
      auto analyzer = *it;
      if (analyzer->match(status_reported->name)) {
        analyzer->analyze(status_reported);
        group_found = true;
      }
    }
    if (!group_found) {
      misc_analyzer_->analyze(status_reported);
    }
  }
}

void MonitorNode::PublishSummary()
{
  for (const auto & analyzer : analyzers_) {
    DiagnosticArray aggregated_msg;
    std::vector<std::shared_ptr<DiagnosticStatus>> report = analyzer->report();
    for (const auto & msg : report) {
      aggregated_msg.status.push_back(*msg);
    }
    aggregated_msg.header.stamp = clock_->now();
    aggregate_pub_->publish(aggregated_msg);
  }
  DiagnosticArray aggregated_msg;
  std::vector<std::shared_ptr<DiagnosticStatus>> report =
    misc_analyzer_->report();
  for (const auto & msg : report) {
    aggregated_msg.status.push_back(*msg);
  }
  aggregated_msg.header.stamp = clock_->now();
  aggregate_pub_->publish(aggregated_msg);
}

MonitorNode::~MonitorNode() {}

}  // namespace nova_monitor
}  // namespace isaac_ros
}  // namespace nvidia

//  Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::nova_monitor::MonitorNode)
