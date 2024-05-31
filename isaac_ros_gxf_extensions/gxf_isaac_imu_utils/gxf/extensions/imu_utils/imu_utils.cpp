// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include "extensions/imu_utils/imu_message_combiner.hpp"
#include "gxf/std/extension_factory_helper.hpp"

GXF_EXT_FACTORY_BEGIN()

GXF_EXT_FACTORY_SET_INFO(
    0x22e02942a64c11ed, 0x8b3007d6a63b38ab, "NvIsaacImuUtilsExtension",
    "Extension containing utilities for IMU data", "Isaac SDK", "2.0.0", "LICENSE");

GXF_EXT_FACTORY_ADD(
    0x3e7daf9a9cfa11ed, 0x997fefae88173117, nvidia::isaac::ImuMessageCombiner, nvidia::gxf::Codelet,
    "Combines accel and gyro messages into a single IMU message");

GXF_EXT_FACTORY_END()
