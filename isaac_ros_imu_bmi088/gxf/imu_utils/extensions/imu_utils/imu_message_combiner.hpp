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

#pragma once

#include "gxf/std/codelet.hpp"
#include "gxf/std/receiver.hpp"
#include "gxf/std/transmitter.hpp"

#include "extensions/messages/accelerometer_message.hpp"
#include "extensions/messages/gyroscope_message.hpp"

namespace nvidia {
namespace isaac {

class ImuMessageCombiner : public gxf::Codelet {
 public:
  gxf_result_t registerInterface(gxf::Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t start() override { return GXF_SUCCESS; }
  gxf_result_t tick() override;
  gxf_result_t stop() override;

 private:
  gxf::Expected<void> syncImuDataCopy();
  gxf::Expected<void> updateGyroMessage();
  gxf::Expected<void> updateAccelMessage();

  size_t gyros_dropped_;
  size_t accels_dropped_;
  size_t total_published_;
  bool suppress_warnings_;

  AccelerometerMessageParts accel_msg_parts_;
  GyroscopeMessageParts gyro_msg_parts_;

  bool accel_updated_;
  bool gyro_updated_;

  gxf::Parameter<gxf::Handle<gxf::Receiver>> rx_accelerometer_;
  gxf::Parameter<gxf::Handle<gxf::Receiver>> rx_gyroscope_;
  gxf::Parameter<gxf::Handle<gxf::Transmitter>> tx_imu_;
  gxf::Parameter<int64_t> time_sync_tol_ns_;
};

}  // namespace isaac
}  // namespace nvidia
