// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "isaac_ros_common/qos.hpp"
#include "isaac_ros_hesai/hesai_node.hpp"
#include "isaac_ros_nitros_point_cloud_type/nitros_point_cloud.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/std/timestamp.hpp"
#pragma GCC diagnostic pop


namespace nvidia
{
namespace isaac_ros
{
namespace hesai
{

using nvidia::gxf::optimizer::GraphIOGroupSupportedDataTypesInfoList;

#define OUTPUT_COMPONENT_KEY_POINT_CLOUD         "sink/sink"
#define OUTPUT_DEFAULT_TENSOR_FORMAT_POINT_CLOUD "nitros_point_cloud"
#define OUTPUT_TOPIC_NAME_POINT_CLOUD            "pointcloud"
#define OUTPUT_FRAME_ID_MAP_KEY                  "hesai_frame"

constexpr char APP_YAML_FILENAME[] = "config/hesai_node.yaml";
constexpr char PACKAGE_NAME[] = "isaac_ros_hesai";

const std::vector<std::pair<std::string, std::string>> EXTENSIONS = {
  {"isaac_ros_gxf", "gxf/lib/serialization/libgxf_serialization.so"},
  // rangescan info
  {"gxf_isaac_messages", "gxf/lib/libgxf_isaac_messages.so"},
  {"gxf_isaac_message_compositor", "gxf/lib/libgxf_isaac_message_compositor.so"},
  {"gxf_isaac_hesai", "gxf/lib/libgxf_isaac_hesai.so"},
  // UdpRecevier
  {"gxf_isaac_utils", "gxf/lib/libgxf_isaac_utils.so"},
  {"gxf_isaac_range_scan_processing", "gxf/lib/libgxf_isaac_range_scan_processing.so"},
  // rangescan to pointcloud
  {"gxf_isaac_point_cloud", "gxf/lib/libgxf_isaac_point_cloud.so"},
  // poitncloud info
  {"gxf_isaac_ros_messages", "gxf/lib/libgxf_isaac_ros_messages.so"}
};
const std::vector<std::string> PRESET_EXTENSION_SPEC_NAMES = {
  "isaac_ros_hesai",
};
const std::vector<std::string> EXTENSION_SPEC_FILENAMES = {};
const std::vector<std::string> GENERATOR_RULE_FILENAMES = {
  "config/namespace_injector_rule_hesai.yaml"
};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
const nitros::NitrosPublisherSubscriberConfigMap CONFIG_MAP = {
  {OUTPUT_COMPONENT_KEY_POINT_CLOUD,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = OUTPUT_DEFAULT_TENSOR_FORMAT_POINT_CLOUD,
      .topic_name = OUTPUT_TOPIC_NAME_POINT_CLOUD,
    }
  }
};
#pragma GCC diagnostic pop

HesaiNode::HesaiNode(const rclcpp::NodeOptions & options)
: nitros::NitrosNode(options,
    APP_YAML_FILENAME,
    CONFIG_MAP,
    PRESET_EXTENSION_SPEC_NAMES,
    EXTENSION_SPEC_FILENAMES,
    GENERATOR_RULE_FILENAMES,
    EXTENSIONS,
    PACKAGE_NAME)
{
  RCLCPP_DEBUG(get_logger(), "[HesaiNode] Constructor");

  // This function sets the QoS parameter for publishers and subscribers in this NITROS node
  rclcpp::QoS output_qos_ = ::isaac_ros::common::AddQosParameter(*this, "DEFAULT", "output_qos");
  for (auto & config : config_map_) {
    config.second.qos = output_qos_;
  }

  registerSupportedType<nvidia::isaac_ros::nitros::NitrosPointCloud>();

  startNitrosNode();
}

HesaiNode::~HesaiNode() {}

}  // namespace hesai
}  // namespace isaac_ros
}  // namespace nvidia

RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::hesai::HesaiNode)
