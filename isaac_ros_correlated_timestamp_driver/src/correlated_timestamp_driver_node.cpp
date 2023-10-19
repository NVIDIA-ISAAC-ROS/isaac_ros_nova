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

#include "isaac_ros_correlated_timestamp_driver/correlated_timestamp_driver_node.hpp"
#include "isaac_ros_nitros_correlated_timestamp_type/nitros_correlated_timestamp.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace correlated_timestamp_driver
{

using nvidia::gxf::optimizer::GraphIOGroupSupportedDataTypesInfoList;

#define OUTPUT_COMPONENT_KEY_CORRELATED_TIMESTAMP         "sink/sink"
#define OUTPUT_DEFAULT_TENSOR_FORMAT_CORRELATED_TIMESTAMP "nitros_correlated_timestamp"
#define OUTPUT_TOPIC_NAME_CORRELATED_TIMESTAMP            "correlated_timestamp"

constexpr char APP_YAML_FILENAME[] = "config/correlated_timestamp_driver_node.yaml";
constexpr char PACKAGE_NAME[] = "isaac_ros_correlated_timestamp_driver";

const std::vector<std::pair<std::string, std::string>> EXTENSIONS = {
  {"isaac_ros_gxf", "gxf/lib/libgxf_timestamp_correlator.so"}  // timestamp correlator
};
const std::vector<std::string> PRESET_EXTENSION_SPEC_NAMES = {
  "isaac_ros_correlated_timestamp_driver",
};
const std::vector<std::string> EXTENSION_SPEC_FILENAMES = {};
const std::vector<std::string> GENERATOR_RULE_FILENAMES = {};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
const nitros::NitrosPublisherSubscriberConfigMap CONFIG_MAP = {
  {OUTPUT_COMPONENT_KEY_CORRELATED_TIMESTAMP,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = OUTPUT_DEFAULT_TENSOR_FORMAT_CORRELATED_TIMESTAMP,
      .topic_name = OUTPUT_TOPIC_NAME_CORRELATED_TIMESTAMP,
    }
  }
};
#pragma GCC diagnostic pop

CorrelatedTimestampDriverNode::CorrelatedTimestampDriverNode(const rclcpp::NodeOptions & options)
: nitros::NitrosNode(options,
    APP_YAML_FILENAME,
    CONFIG_MAP,
    PRESET_EXTENSION_SPEC_NAMES,
    EXTENSION_SPEC_FILENAMES,
    GENERATOR_RULE_FILENAMES,
    EXTENSIONS,
    PACKAGE_NAME),
  use_time_since_epoch_(declare_parameter<bool>("use_time_since_epoch", false)),
  nvpps_dev_name_(declare_parameter<std::string>("nvpps_dev_name", "/dev/nvpps0"))
{
  RCLCPP_DEBUG(get_logger(), "[CorrelatedTimestampDriverNode] Constructor");

  registerSupportedType<nvidia::isaac_ros::nitros::NitrosCorrelatedTimestamp>();

  startNitrosNode();
}

void CorrelatedTimestampDriverNode::postLoadGraphCallback()
{
  getNitrosContext().setParameterBool(
    "correlator", "nvidia::isaac::CorrelatedTimestampDriver", "use_time_since_epoch",
    use_time_since_epoch_);
  getNitrosContext().setParameterStr(
    "correlator", "nvidia::isaac::CorrelatedTimestampDriver", "nvpps_dev_name", nvpps_dev_name_);
}


CorrelatedTimestampDriverNode::~CorrelatedTimestampDriverNode() {}

}  // namespace correlated_timestamp_driver
}  // namespace isaac_ros
}  // namespace nvidia

RCLCPP_COMPONENTS_REGISTER_NODE(
  nvidia::isaac_ros::correlated_timestamp_driver::CorrelatedTimestampDriverNode)
