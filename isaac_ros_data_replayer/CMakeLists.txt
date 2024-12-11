# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.5)
project(isaac_ros_data_replayer LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# compressed_image_to_compressed_video
ament_auto_add_library(
  compressed_image_to_compressed_video SHARED src/compressed_image_to_compressed_video_node.cpp)
rclcpp_components_register_nodes(
  compressed_image_to_compressed_video "nvidia::isaac_ros::data_replayer::CompressedImageToCompressedVideoNode")
ament_target_dependencies(compressed_image_to_compressed_video foxglove_msgs)

set(
  node_plugins
  "${node_plugins}nvidia::isaac_ros::data_replayer::CompressedImageToCompressedVideoNode;$<TARGET_FILE:compressed_image_to_compressed_video>\n")

install(PROGRAMS scripts/camera_converter.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/camera_info_publisher.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/camera_info_writer.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/data_extraction.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/data_processor.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/foxglove_converter.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/image_writer.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/rosbag_converter.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/stereo_camera_info_creator.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # The FindPythonInterp and FindPythonLibs modules are removed
  if(POLICY CMP0148)
    cmake_policy(SET CMP0148 OLD)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/isaac_ros_compressed_image_to_compressed_video_pol_test.py)

  find_package(Python REQUIRED)
  install(PROGRAMS test/test_node.py DESTINATION lib/${PROJECT_NAME})
  add_test(
    NAME TestDataProcessor
    COMMAND ${PYTHON_EXECUTABLE} -m unittest test/test_data_processor.py
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  )
endif()


# Embed versioning information into installed files
ament_index_get_resource(ISAAC_ROS_COMMON_CMAKE_PATH isaac_ros_common_cmake_path isaac_ros_common)
include("${ISAAC_ROS_COMMON_CMAKE_PATH}/isaac_ros_common-version-info.cmake")
generate_version_info(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE launch)
