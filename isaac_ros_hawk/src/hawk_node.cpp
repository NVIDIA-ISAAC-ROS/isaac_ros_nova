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

#include "isaac_ros_hawk/hawk_node.hpp"
#include "isaac_ros_nitros_imu_type/nitros_imu.hpp"
#include "isaac_ros_nitros_correlated_timestamp_type/nitros_correlated_timestamp.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace hawk
{

using nvidia::gxf::optimizer::GraphIOGroupSupportedDataTypesInfoList;

constexpr char OUTPUT_COMPONENT_KEY_CAM_LEFT[] = "sink_left_image/sink";
constexpr char OUTPUT_DEFAULT_TENSOR_FORMAT_CAM_LEFT[] = "nitros_image_rgb8";
constexpr char OUTPUT_TOPIC_NAME_CAM_LEFT[] = "left/image_raw";

constexpr char OUTPUT_COMPONENT_KEY_CAM_INFO_LEFT[] = "sink_left_camerainfo/sink";
constexpr char OUTPUT_DEFAULT_TENSOR_FORMAT_CAM_INFO_LEFT[] = "nitros_camera_info";
constexpr char OUTPUT_TOPIC_NAME_CAM_INFO_LEFT[] = "left/camerainfo";

constexpr char OUTPUT_COMPONENT_KEY_CAM_RIGHT[] = "sink_right_image/sink";
constexpr char OUTPUT_DEFAULT_TENSOR_FORMAT_CAM_RIGHT[] = "nitros_image_rgb8";
constexpr char OUTPUT_TOPIC_NAME_CAM_RIGHT[] = "right/image_raw";

constexpr char OUTPUT_COMPONENT_KEY_CAM_INFO_RIGHT[] = "sink_right_camerainfo/sink";
constexpr char OUTPUT_DEFAULT_TENSOR_FORMAT_CAM_INFO_RIGHT[] = "nitros_camera_info";
constexpr char OUTPUT_TOPIC_NAME_CAM_INFO_RIGHT[] = "right/camerainfo";

constexpr char APP_YAML_FILENAME[] = "config/hawk_node.yaml";
constexpr char PACKAGE_NAME[] = "isaac_ros_hawk";

constexpr char INPUT_COMPONENT_KEY_CORRELATED_TIMESTAMP[] =
  "correlator_broadcast/rx_correlated_timestamps";
constexpr char INPUT_DEFAULT_TENSOR_FORMAT_CORRELATED_TIMESTAMP[] =
  "nitros_correlated_timestamp";
constexpr char INPUT_TOPIC_NAME_CORRELATED_TIMESTAMP[] =
  "correlated_timestamp";

const std::vector<std::pair<std::string, std::string>> EXTENSIONS = {
  {"isaac_ros_gxf", "gxf/lib/std/libgxf_std.so"},
  {"isaac_ros_gxf", "gxf/lib/cuda/libgxf_cuda.so"},
  {"isaac_ros_gxf", "gxf/lib/serialization/libgxf_serialization.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_gxf_helpers.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_sight.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_atlas.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_isaac_messages.so"},
  {"isaac_ros_gxf", "gxf/lib/multimedia/libgxf_multimedia.so"},
  {"isaac_ros_image_proc", "gxf/lib/image_proc/libgxf_tensorops.so"},
  {"isaac_ros_image_proc", "gxf/lib/image_proc/libgxf_rectify_params_generator.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_timestamp_correlator.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_argus.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_message_compositor.so"},
  {"isaac_ros_argus_camera", "gxf/lib/utils/libgxf_utils.so"}
};
const std::vector<std::string> PRESET_EXTENSION_SPEC_NAMES = {
  "isaac_ros_hawk",
};
const std::vector<std::string> EXTENSION_SPEC_FILENAMES = {};
const std::vector<std::string> GENERATOR_RULE_FILENAMES = {
  "config/namespace_injector_rule_hawk.yaml"
};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
const nitros::NitrosPublisherSubscriberConfigMap CONFIG_MAP = {
  {OUTPUT_COMPONENT_KEY_CAM_LEFT,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = OUTPUT_DEFAULT_TENSOR_FORMAT_CAM_LEFT,
      .topic_name = OUTPUT_TOPIC_NAME_CAM_LEFT,
    }
  },
  {OUTPUT_COMPONENT_KEY_CAM_INFO_LEFT,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = OUTPUT_DEFAULT_TENSOR_FORMAT_CAM_INFO_LEFT,
      .topic_name = OUTPUT_TOPIC_NAME_CAM_INFO_LEFT,
    }
  },
  {OUTPUT_COMPONENT_KEY_CAM_RIGHT,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = OUTPUT_DEFAULT_TENSOR_FORMAT_CAM_RIGHT,
      .topic_name = OUTPUT_TOPIC_NAME_CAM_RIGHT,
    }
  },
  {OUTPUT_COMPONENT_KEY_CAM_INFO_RIGHT,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = OUTPUT_DEFAULT_TENSOR_FORMAT_CAM_INFO_RIGHT,
      .topic_name = OUTPUT_TOPIC_NAME_CAM_INFO_RIGHT,
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

HawkNode::HawkNode(const rclcpp::NodeOptions & options)
: ArgusCameraNode(
    options,
    APP_YAML_FILENAME,
    CONFIG_MAP,
    PRESET_EXTENSION_SPEC_NAMES,
    EXTENSION_SPEC_FILENAMES,
    GENERATOR_RULE_FILENAMES,
    EXTENSIONS,
    PACKAGE_NAME)
{
  camera_id_ = declare_parameter<int>("camera_id", 0);
  module_id_ = declare_parameter<int>("module_id", 2);
  mode_ = declare_parameter<int>("mode", 0);
  camera_type_ = declare_parameter<int>("camera_type", 1);
  fsync_type_ = declare_parameter<int>("fsync_type", 1);
  camera_link_frame_name_ = declare_parameter<std::string>("camera_link_frame_name", "camera");
  left_optical_frame_name_ = declare_parameter<std::string>("left_optical_frame_name", "left_cam");
  right_optical_frame_name_ =
    declare_parameter<std::string>("right_optical_frame_name", "right_cam");
  left_camera_info_url_ =
    declare_parameter<std::string>("left_camera_info_url", "");
  right_camera_info_url_ =
    declare_parameter<std::string>("right_camera_info_url", "");
  imu_frequency_ = declare_parameter<int>("imu_frequency", 200);
  bmi_id_ = declare_parameter<int>("bmi_id", 69);

  // Load camera info from files if provided
  if (!left_camera_info_url_.empty()) {
    left_camera_info_ = loadCameraInfoFromFile(left_camera_info_url_);
    RCLCPP_INFO(
      get_logger(), "[ArgusStereoNode] Loaded left camera info from \"%s\"",
      left_camera_info_url_.c_str());
  }
  if (!right_camera_info_url_.empty()) {
    right_camera_info_ = loadCameraInfoFromFile(right_camera_info_url_);
    RCLCPP_INFO(
      get_logger(), "[ArgusStereoNode] Loaded right camera info from \"%s\"",
      right_camera_info_url_.c_str());
  }

  // Adding callback for left image
  config_map_[OUTPUT_COMPONENT_KEY_CAM_LEFT].callback =
    std::bind(
    &ArgusCameraNode::ArgusImageCallback, this,
    std::placeholders::_1, std::placeholders::_2, left_optical_frame_name_);

  // Adding callback for right image
  config_map_[OUTPUT_COMPONENT_KEY_CAM_RIGHT].callback =
    std::bind(
    &ArgusCameraNode::ArgusImageCallback, this,
    std::placeholders::_1, std::placeholders::_2, right_optical_frame_name_);

  // Adding callback for left camerainfo
  config_map_[OUTPUT_COMPONENT_KEY_CAM_INFO_LEFT].callback =
    std::bind(
    &ArgusCameraNode::ArgusCameraInfoCallback, this,
    std::placeholders::_1, std::placeholders::_2, camera_link_frame_name_,
    left_optical_frame_name_, left_camera_info_);

  // Adding callback for right camerainfo
  config_map_[OUTPUT_COMPONENT_KEY_CAM_INFO_RIGHT].callback =
    std::bind(
    &ArgusCameraNode::ArgusCameraInfoCallback, this,
    std::placeholders::_1, std::placeholders::_2, camera_link_frame_name_,
    right_optical_frame_name_, right_camera_info_);
  registerSupportedType<nvidia::isaac_ros::nitros::NitrosImu>();
  registerSupportedType<nvidia::isaac_ros::nitros::NitrosCorrelatedTimestamp>();
  startNitrosNode();
}

HawkNode::~HawkNode() = default;

void HawkNode::preLoadGraphCallback()
{
  nvidia::isaac_ros::argus::ArgusCameraNode::preLoadGraphCallback();
  RCLCPP_INFO(get_logger(), "[HawkNode] preLoadGraphCallback().");
}

void HawkNode::postLoadGraphCallback()
{
  nvidia::isaac_ros::argus::ArgusCameraNode::postLoadGraphCallback();
  RCLCPP_INFO(get_logger(), "[HawkNode] postLoadGraphCallback().");
}

}  // namespace hawk
}  // namespace isaac_ros
}  // namespace nvidia

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::hawk::HawkNode)
