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
#include "extensions/bmi088_imu/bmi088_driver.hpp"
#include "gxf/std/extension_factory_helper.hpp"

GXF_EXT_FACTORY_BEGIN()

GXF_EXT_FACTORY_SET_INFO(
    0xa87c3d4f3f2d2520, 0x9a82df070e64dd20, "bmi088_imu",
    "Extension containing bmi088 IMU components", "Isaac SDK", "2.0.0", "LICENSE");

GXF_EXT_FACTORY_ADD(
    0x1d9c42a09cfa11ed, 0xa5d8c342776c0af6, nvidia::isaac::Bmi088Driver, nvidia::gxf::Codelet,
    "GXF driver for BMI088 IIO driver");

GXF_EXT_FACTORY_END()
