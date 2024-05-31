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

#include <cmath>
#include <string>

#include "extensions/messages/imu_message.hpp"
#include "gems/gxf_helpers/expected_macro_gxf.hpp"

namespace nvidia {
namespace isaac {

namespace {
size_t kMaxNumWarnings = 25;  // maximum number of warnings to print before suppressing them
}

gxf_result_t ImuMessageCombiner::registerInterface(gxf::Registrar* registrar) {
  gxf::Expected<void> result;
  result &= registrar->parameter(
      rx_accelerometer_, "rx_accelerometer", "Accelerometer Receiver",
      "Input for messages containing accelerometer component");
  result &= registrar->parameter(
      rx_gyroscope_, "rx_gyroscope", "Gyroscope Receiver",
      "Input for messages containing gyroscope component");
  result &= registrar->parameter(
      tx_imu_, "tx_imu", "IMU Transmitter",
      "Output IMUMessage with components combined from inputs");
  result &= registrar->parameter(
      time_sync_tol_ns_, "time_sync_tol_ns", "Time Sync Tolerance (ns)",
      "Time tolerance for an accelerometer and gyroscope message to be considered synchronized",
      static_cast<int64_t>(5.2e6));
  result &= registrar->parameter(
      combine_method_, "combine_method", "Combination method",
      "Which method to use for combining data streams, options are interpolate or copy",
      std::string("copy"));
  result &= registrar->parameter(
      apply_delay_offsets_, "apply_delay_offsets", "Apply Offsets?",
      "Boolean determining if compensation for group delays are applied", false);
  result &= registrar->parameter(
      accelerometer_offset_ns_, "accelerometer_delay_ns", "Accelerometer Group Delay (nanoseconds)",
      "Delay in accelerometer measurements, added as an offset to the timestamp", 10000000L);
  result &= registrar->parameter(
      gyroscope_offset_ns_, "gyroscope_delay_ns", "Gyroscope Group Delay (nanoseconds)",
      "Delay in gyroscope measurements, added as an offset to the timestamp", 34500000L);

  return gxf::ToResultCode(result);
}

gxf_result_t ImuMessageCombiner::initialize() {
  gyros_dropped_ = 0;
  accels_dropped_ = 0;
  total_published_ = 0;
  suppress_warnings_ = false;

  accel_updated_ = false;
  gyro_updated_ = false;

  return GXF_SUCCESS;
}

gxf_result_t ImuMessageCombiner::tick() {
  if (combine_method_.get() == "interpolate") {
    return gxf::ToResultCode(syncImuDataInterpolate());
  } else if (combine_method_.get() == "copy") {
    return gxf::ToResultCode(syncImuDataCopy());
  } else {
    GXF_LOG_ERROR("Unsupported combine_method %s", combine_method_.get().c_str());
    return GXF_FAILURE;
  }
}

gxf::Expected<void> ImuMessageCombiner::updateGyroMessage() {
  auto gyro_msg_entity = UNWRAP_OR_RETURN(rx_gyroscope_->receive());
  curr_gyro_msg_parts_ = UNWRAP_OR_RETURN(GetGyroscopeMessage(gyro_msg_entity));
  if (apply_delay_offsets_.get()) {
    curr_gyro_msg_parts_.timestamp->acqtime -= gyroscope_offset_ns_.get();
  }
  gyro_updated_ = true;
  return gxf::Success;
}

gxf::Expected<void> ImuMessageCombiner::updateAccelMessage() {
  prev_accel_msg_parts_ = curr_accel_msg_parts_;
  auto accel_msg_entity = UNWRAP_OR_RETURN(rx_accelerometer_->receive());
  curr_accel_msg_parts_ = UNWRAP_OR_RETURN(GetAccelerometerMessage(accel_msg_entity));
  if (apply_delay_offsets_.get()) {
    curr_accel_msg_parts_.timestamp->acqtime -= accelerometer_offset_ns_.get();
  }
  accel_updated_ = true;
  return gxf::Success;
}

gxf::Expected<void> ImuMessageCombiner::syncImuDataInterpolate() {
  if (!gyro_updated_) {
    RETURN_IF_ERROR(updateGyroMessage());
  }
  if (!accel_updated_) {
    RETURN_IF_ERROR(updateAccelMessage());
  }

  if (prev_accel_msg_parts_.message.is_null()) {
    accel_updated_ = false;
    return gxf::Success;
  }

  if (prev_accel_msg_parts_.timestamp->acqtime < curr_gyro_msg_parts_.timestamp->acqtime &&
      curr_accel_msg_parts_.timestamp->acqtime > curr_gyro_msg_parts_.timestamp->acqtime) {
    auto imu_msg_parts = UNWRAP_OR_RETURN(CreateImuMessage(context()));

    const double alpha =
        static_cast<double>(
            curr_gyro_msg_parts_.timestamp->acqtime - prev_accel_msg_parts_.timestamp->acqtime) /
        static_cast<double>(
            curr_accel_msg_parts_.timestamp->acqtime - prev_accel_msg_parts_.timestamp->acqtime);

    imu_msg_parts.imu->linear_acceleration_x =
        (1 - alpha) * prev_accel_msg_parts_.accelerometer->linear_acceleration_x +
        alpha * curr_accel_msg_parts_.accelerometer->linear_acceleration_x;

    imu_msg_parts.imu->linear_acceleration_y =
        (1 - alpha) * prev_accel_msg_parts_.accelerometer->linear_acceleration_y +
        alpha * curr_accel_msg_parts_.accelerometer->linear_acceleration_y;

    imu_msg_parts.imu->linear_acceleration_z =
        (1 - alpha) * prev_accel_msg_parts_.accelerometer->linear_acceleration_z +
        alpha * curr_accel_msg_parts_.accelerometer->linear_acceleration_z;

    imu_msg_parts.imu->angular_velocity_x = curr_gyro_msg_parts_.gyroscope->angular_velocity_x;
    imu_msg_parts.imu->angular_velocity_y = curr_gyro_msg_parts_.gyroscope->angular_velocity_y;
    imu_msg_parts.imu->angular_velocity_z = curr_gyro_msg_parts_.gyroscope->angular_velocity_z;

    // Forward the frame id of the gyroscope message
    imu_msg_parts.pose_frame_uid->uid = curr_gyro_msg_parts_.pose_frame_uid->uid;

    // For now just take the gyroscope timestamp, this is what the realsense GXF driver does
    // RMPLite driver essentially uses execution time when publishing, which is worse
    imu_msg_parts.timestamp->acqtime = curr_gyro_msg_parts_.timestamp->acqtime;
    imu_msg_parts.timestamp->pubtime = getExecutionTimestamp();

    tx_imu_->publish(imu_msg_parts.message);

    total_published_++;
    gyro_updated_ = false;
    accel_updated_ = false;

  } else if (curr_accel_msg_parts_.timestamp->acqtime < curr_gyro_msg_parts_.timestamp->acqtime) {
    accel_updated_ = false;
    accels_dropped_++;
    return gxf::Success;  // wait for more data
  } else if (curr_gyro_msg_parts_.timestamp->acqtime < prev_accel_msg_parts_.timestamp->acqtime) {
    gyro_updated_ = false;
    gyros_dropped_++;
    return gxf::Success;  // wait for more data
  }

  return gxf::Success;
}

gxf::Expected<void> ImuMessageCombiner::syncImuDataCopy() {
  if (!gyro_updated_) {
    RETURN_IF_ERROR(updateGyroMessage());
  }
  if (!accel_updated_) {
    RETURN_IF_ERROR(updateAccelMessage());
  }
  int64_t timestamp_diff =
      std::abs(curr_gyro_msg_parts_.timestamp->acqtime - curr_accel_msg_parts_.timestamp->acqtime);
  if (timestamp_diff > time_sync_tol_ns_.get()) {
    if (!suppress_warnings_) {
      if (accels_dropped_ + gyros_dropped_ < kMaxNumWarnings) {
        GXF_LOG_WARNING(
            "Gyroscope and accelerometer timestamps are %zd ns apart, more than time_tol = %zd "
            "apart, dropping message",
            timestamp_diff, time_sync_tol_ns_.get());
      } else {
        GXF_LOG_WARNING("Too many messages dropped, suppressing further warnings");
        suppress_warnings_ = true;
      }
    }
    if (curr_gyro_msg_parts_.timestamp->acqtime < curr_accel_msg_parts_.timestamp->acqtime) {
      gyro_updated_ = false;
      gyros_dropped_++;
      return gxf::Success;  // wait for more data
    } else {
      accel_updated_ = false;
      accels_dropped_++;
      return gxf::Success;  // wait for more data
    }
  }

  if (gyro_updated_ && accel_updated_) {
    auto imu_msg_parts = UNWRAP_OR_RETURN(CreateImuMessage(context()));
    imu_msg_parts.imu->linear_acceleration_x =
        curr_accel_msg_parts_.accelerometer->linear_acceleration_x;
    imu_msg_parts.imu->linear_acceleration_y =
        curr_accel_msg_parts_.accelerometer->linear_acceleration_y;
    imu_msg_parts.imu->linear_acceleration_z =
        curr_accel_msg_parts_.accelerometer->linear_acceleration_z;
    imu_msg_parts.imu->angular_velocity_x = curr_gyro_msg_parts_.gyroscope->angular_velocity_x;
    imu_msg_parts.imu->angular_velocity_y = curr_gyro_msg_parts_.gyroscope->angular_velocity_y;
    imu_msg_parts.imu->angular_velocity_z = curr_gyro_msg_parts_.gyroscope->angular_velocity_z;

    // Forward the frame id of the gyroscope message
    imu_msg_parts.pose_frame_uid->uid = curr_gyro_msg_parts_.pose_frame_uid->uid;

    // For now just take the gyroscope timestamp, this is what the realsense GXF driver does
    // RMPLite driver essentially uses execution time when publishing, which is worse
    imu_msg_parts.timestamp->acqtime = curr_gyro_msg_parts_.timestamp->acqtime;
    imu_msg_parts.timestamp->pubtime = getExecutionTimestamp();

    tx_imu_->publish(imu_msg_parts.message);

    total_published_++;
    gyro_updated_ = false;
    accel_updated_ = false;
  }
  return gxf::Success;
}

gxf_result_t ImuMessageCombiner::stop() {
  float total_dropped = static_cast<float>(gyros_dropped_ + accels_dropped_);
  float total_packets = total_dropped + static_cast<float>(total_published_);
  GXF_LOG_INFO("Dropped %lu gyros and %lu accels", gyros_dropped_, accels_dropped_);
  GXF_LOG_INFO("Published %lu IMU messages", total_published_);
  GXF_LOG_INFO("percent dropped: %f %%", 100 * total_dropped / total_packets);

  curr_gyro_msg_parts_.message = gxf::Entity();
  curr_accel_msg_parts_.message = gxf::Entity();
  prev_accel_msg_parts_.message = gxf::Entity();

  return GXF_SUCCESS;
}

}  // namespace isaac
}  // namespace nvidia
