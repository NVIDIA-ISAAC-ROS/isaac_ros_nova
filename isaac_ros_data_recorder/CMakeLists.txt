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
project(isaac_ros_data_recorder LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(PROGRAMS
  scripts/record_data.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_package(INSTALL_TO_SHARE launch)
