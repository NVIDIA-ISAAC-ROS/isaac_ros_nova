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

cmake_minimum_required(VERSION 3.22.1)
project(isaac_ros_nova_recorder LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(PROGRAMS scripts/record_data.sh DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/event_trigger_node.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/jpeg_encoder_node.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/rosbag_manager_node.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/rosbag_uploader_node.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/web_server.py DESTINATION lib/${PROJECT_NAME})

find_package(ament_cmake_auto REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/UploadRosbag.action"
  "msg/DirectoryInfo.msg"
  "msg/RosbagInfo.msg"
  "msg/TransferStatus.msg"
  "srv/DeleteRosbag.srv"
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

ament_auto_find_build_dependencies()
ament_export_dependencies(rosidl_default_runtime)

# Camera Multiplexer Node
ament_auto_add_library(camera_multiplexer_node SHARED src/camera_multiplexer_node.cpp)
rclcpp_components_register_nodes(camera_multiplexer_node "nvidia::isaac_ros::isaac_ros_nova_recorder::CameraMultiplexerNode")
set(node_plugins "${node_plugins}nvidia::isaac_ros::isaac_ros_nova_recorder::CameraMultiplexerNode;$<TARGET_FILE:camera_multiplexer_node>\n")
set_target_properties(camera_multiplexer_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE)

# Find OpenSSL executable
find_program(OPENSSL_EXECUTABLE openssl)
if(NOT OPENSSL_EXECUTABLE)
  message(FATAL_ERROR "OpenSSL executable not found. Please install OpenSSL.")
endif()

# Specify the output directory for the PEM files
set(CERTS_DIR "${CMAKE_CURRENT_BINARY_DIR}/certs")

# Add a custom command to generate the cert.pem and key.pem
add_custom_command(
  OUTPUT "${CERTS_DIR}/cert.pem" "${CERTS_DIR}/key.pem"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CERTS_DIR}"
  COMMAND ${OPENSSL_EXECUTABLE}
          req
          -x509
          -newkey
          rsa:4096
          -keyout
          "${CERTS_DIR}/key.pem"
          -out
          "${CERTS_DIR}/cert.pem"
          -days
          365
          -nodes
          -subj
          "/C=US/ST=State/L=City/O=Organization/OU=Unit/CN=localhost"
  COMMAND chmod 600 "${CERTS_DIR}/key.pem"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt"
  COMMENT "Generating self-signed SSL certificate and key"
)

# Add a custom target that depends on the PEM files
add_custom_target(generate_certs ALL DEPENDS "${CERTS_DIR}/cert.pem" "${CERTS_DIR}/key.pem")

# Install the PEM files to the share directory
install(
  FILES "${CERTS_DIR}/cert.pem" "${CERTS_DIR}/key.pem"
  DESTINATION share/${PROJECT_NAME}/certs
)

include(FetchContent)

# Find NodeJS and Yarn
find_program(NODEJS_EXECUTABLE node)
if(NOT NODEJS_EXECUTABLE)
  message(FATAL_ERROR "NodeJS not found. Please install NodeJS to build foxsight.")
endif()

find_program(YARN_EXECUTABLE yarn)
if(NOT YARN_EXECUTABLE)
  message(FATAL_ERROR "Yarn not found. Please install Yarn to build foxsight.")
endif()

# Fetch the foxsight project
FetchContent_Declare(
  foxsight
  GIT_REPOSITORY https://github.com/nvidia-isaac/lichtblick
  GIT_TAG        main_internal
)

FetchContent_MakeAvailable(foxsight)

# Define the path to the foxsight directory
set(FOXSIGHT_DIR ${foxsight_SOURCE_DIR})

# Add custom target to build foxsight
add_custom_target(build_foxsight ALL
  COMMAND ${YARN_EXECUTABLE} install
  COMMAND ${YARN_EXECUTABLE} run web:build:prod >> foxsight_install.log 2>&1
  WORKING_DIRECTORY ${FOXSIGHT_DIR}
  COMMENT "Building foxsight using FetchContent"
)

# Install the built foxsight artifacts
install(DIRECTORY ${FOXSIGHT_DIR}/web/.webpack/
  DESTINATION share/${PROJECT_NAME}/foxsight
  PATTERN "node_modules" EXCLUDE
)


# Embed versioning information into installed files
ament_index_get_resource(ISAAC_ROS_COMMON_CMAKE_PATH isaac_ros_common_cmake_path isaac_ros_common)
include("${ISAAC_ROS_COMMON_CMAKE_PATH}/isaac_ros_common-version-info.cmake")
generate_version_info(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE launch)
