# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

project(gxf_bmi088_imu LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-fPIC -w)
endif()

# Dependencies
find_package(CUDAToolkit)
find_package(vpi REQUIRED)
find_package(GXF ${ISAAC_ROS_GXF_VERSION} MODULE REQUIRED
    core
    cuda
    multimedia
    isaac_messages
)
find_package(yaml-cpp)

# Create extension
add_library(gxf_bmi088_imu SHARED
  extensions/bmi088_imu/bmi088_imu.cpp
  extensions/bmi088_imu/bmi088_driver.cpp
  extensions/bmi088_imu/bmi088_driver.hpp
)

target_include_directories(gxf_bmi088_imu PRIVATE  ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(gxf_bmi088_imu
  PUBLIC
    GXF::cuda
    GXF::multimedia
    yaml-cpp
  PRIVATE
    GXF::isaac_messages
)
