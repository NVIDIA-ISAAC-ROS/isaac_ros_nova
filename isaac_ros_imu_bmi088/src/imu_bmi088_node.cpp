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

#include "isaac_ros_imu_bmi088/imu_bmi088_node.hpp"
#include "isaac_ros_nitros_imu_type/nitros_imu.hpp"
#include "isaac_ros_nitros_correlated_timestamp_type/nitros_correlated_timestamp.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace imu_bmi088
{

using nvidia::gxf::optimizer::GraphIOGroupSupportedDataTypesInfoList;

constexpr char OUTPUT_COMPONENT_KEY_IMU[] = "sink/sink";
constexpr char OUTPUT_DEFAULT_TENSOR_FORMAT_IMU[] = "nitros_imu";
constexpr char OUTPUT_TOPIC_NAME_IMU[] = "imu";

constexpr char INPUT_COMPONENT_KEY_CORRELATED_TIMESTAMP[] =
  "correlator_broadcast/rx_correlated_timestamps";
constexpr char INPUT_DEFAULT_TENSOR_FORMAT_CORRELATED_TIMESTAMP[] =
  "nitros_correlated_timestamp";
constexpr char INPUT_TOPIC_NAME_CORRELATED_TIMESTAMP[] =
  "correlated_timestamp";

constexpr char APP_YAML_FILENAME[] = "config/bmi088_node.yaml";
constexpr char PACKAGE_NAME[] = "isaac_ros_imu_bmi088";

const std::vector<std::pair<std::string, std::string>> EXTENSIONS = {
  {"isaac_ros_gxf", "gxf/lib/serialization/libgxf_serialization.so"},
  {"isaac_ros_imu_bmi088", "gxf/lib/bmi088_imu/libgxf_bmi088_imu.so"},  // bmi088_driver
  {"isaac_ros_imu_bmi088", "gxf/lib/imu_utils/libgxf_imu_utils.so"},  // imu_combiner
  {"isaac_ros_gxf", "gxf/lib/libgxf_timestamp_correlator.so"}  // timestamp_translator
};
const std::vector<std::string> PRESET_EXTENSION_SPEC_NAMES = {
  "isaac_ros_imu_bmi088",
};
const std::vector<std::string> EXTENSION_SPEC_FILENAMES = {};
const std::vector<std::string> GENERATOR_RULE_FILENAMES = {
  "config/namespace_injector_rule_bmi088.yaml"
};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
const nitros::NitrosPublisherSubscriberConfigMap CONFIG_MAP = {
  {OUTPUT_COMPONENT_KEY_IMU,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = OUTPUT_DEFAULT_TENSOR_FORMAT_IMU,
      .topic_name = OUTPUT_TOPIC_NAME_IMU,
    }
  },
  {INPUT_COMPONENT_KEY_CORRELATED_TIMESTAMP,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = INPUT_DEFAULT_TENSOR_FORMAT_CORRELATED_TIMESTAMP,
      .topic_name = INPUT_TOPIC_NAME_CORRELATED_TIMESTAMP,
    }
  }
};
#pragma GCC diagnostic pop

Bmi088Node::Bmi088Node(const rclcpp::NodeOptions & options)
: nitros::NitrosNode(options,
    APP_YAML_FILENAME,
    CONFIG_MAP,
    PRESET_EXTENSION_SPEC_NAMES,
    EXTENSION_SPEC_FILENAMES,
    GENERATOR_RULE_FILENAMES,
    EXTENSIONS,
    PACKAGE_NAME),
  imu_frequency_(declare_parameter<int>("imu_frequency", 100)),
  bmi_id_(declare_parameter<int>("bmi_id", 69))
{
  RCLCPP_DEBUG(get_logger(), "[Bmi088Node] Constructor");

  registerSupportedType<nvidia::isaac_ros::nitros::NitrosImu>();
  registerSupportedType<nvidia::isaac_ros::nitros::NitrosCorrelatedTimestamp>();

  startNitrosNode();
}

void Bmi088Node::postLoadGraphCallback()
{
  getNitrosContext().setParameterInt32(
    "bmi088_driver", "nvidia::isaac::Bmi088Driver", "accel_frequency", imu_frequency_);
  getNitrosContext().setParameterInt32(
    "bmi088_driver", "nvidia::isaac::Bmi088Driver", "gyro_frequency", imu_frequency_);
  getNitrosContext().setParameterInt32(
    "bmi088_driver", "nvidia::isaac::Bmi088Driver", "bmi_id", bmi_id_);
}

Bmi088Node::~Bmi088Node() {}

}  // namespace imu_bmi088
}  // namespace isaac_ros
}  // namespace nvidia

RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::imu_bmi088::Bmi088Node)
