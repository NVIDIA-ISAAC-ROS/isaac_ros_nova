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

#include "isaac_ros_nova/nova_analyzer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace nvidia
{
namespace isaac_ros
{
namespace nova_monitor
{

using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

NovaAnalyzer::NovaAnalyzer()
: base_path_(""), timeout_(20.0) {}

NovaAnalyzer::~NovaAnalyzer() {}

bool NovaAnalyzer::init(
  const std::string & base_path,
  const std::string & sensor_name,
  rclcpp::Node::SharedPtr n)
{
  base_path_ = base_path;
  path_ = base_path_ + "/sensors/" + sensor_name;
  id_string_ = sensor_name;
  has_initialized_ = true;
  clock_ = n->get_clock();
  return true;
}

bool NovaAnalyzer::init(
  const std::string & base_path,
  const std::string & sensor_name,
  rclcpp::Clock::SharedPtr clk)
{
  base_path_ = base_path;
  path_ = base_path_ + sensor_name;
  id_string_ = sensor_name;
  has_initialized_ = true;
  clock_ = clk;
  return true;
}

bool NovaAnalyzer::analyze(const std::shared_ptr<DiagnosticStatus> status)
{
  if (!has_initialized_) {
    return false;
  }

  items_[status->name] = std::make_tuple(status, clock_->now());
  return has_initialized_;
}

bool NovaAnalyzer::match(const std::string & name)
{
  if (name.find(id_string_) != std::string::npos) {
    return true;
  } else {
    return false;
  }
}

std::string rqt_compatible_name(std::string output_name)
{
  std::string slash_str = "/";
  std::string::size_type pos = 0;
  while ((pos = output_name.find(slash_str, pos)) != std::string::npos) {
    output_name.replace(pos, slash_str.size(), " ");
    pos++;
  }
  return output_name;
}

std::vector<std::shared_ptr<DiagnosticStatus>> NovaAnalyzer::report()
{
  auto header_status = std::make_shared<DiagnosticStatus>();
  header_status->name = path_;
  header_status->level = DiagnosticStatus::OK;
  header_status->message = "OK";
  std::vector<std::shared_ptr<DiagnosticStatus>> processed;
  processed.push_back(header_status);
  auto it = items_.begin();
  bool all_stale = true;

  while (it != items_.end()) {
    auto name = it->first;
    auto item = it->second;
    std::shared_ptr<DiagnosticStatus> status = std::get<0>(item);
    rclcpp::Time update_time = std::get<1>(item);

    bool stale = false;
    stale = (clock_->now() - update_time).seconds() > timeout_;

    auto level = status->level;
    if (header_status->level < level) {
      header_status->message = status->message;
    }
    header_status->level = std::max(header_status->level, level);

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = name;
    kv.value = status->message;

    header_status->values.push_back(kv);

    all_stale = all_stale && ((level == DiagnosticStatus::STALE) || stale);

    std::shared_ptr<DiagnosticStatus> status_msg(new DiagnosticStatus());
    status_msg->name = path_ + "/" + rqt_compatible_name(status->name);
    status_msg->level = (stale) ? DiagnosticStatus::STALE : status->level;
    status_msg->message = status->message;
    status_msg->hardware_id = status->hardware_id;
    status_msg->values = status->values;

    processed.push_back(status_msg);

    ++it;
  }

  // Header is not stale unless all subs are
  if (all_stale) {
    header_status->level = DiagnosticStatus::STALE;
    header_status->message = "ALL TOPICS STALE";
  }
  if (items_.empty()) {
    header_status->level = DiagnosticStatus::STALE;
    header_status->message = "NO PUBLISHED TOPICS";
  }

  return processed;
}

}  // namespace nova_monitor
}  // namespace isaac_ros
}  // namespace nvidia
