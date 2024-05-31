// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <sys/poll.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "extensions/atlas/pose_tree_frame.hpp"
#include "extensions/messages/accelerometer_message.hpp"
#include "extensions/messages/gyroscope_message.hpp"
#include "gxf/std/codelet.hpp"
#include "gxf/std/scheduling_terms.hpp"
#include "gxf/std/transmitter.hpp"

namespace nvidia {
namespace isaac {

class Bmi088Driver : public gxf::Codelet {
 public:
  gxf_result_t registerInterface(gxf::Registrar* registrar) override;
  gxf_result_t start() override;
  gxf_result_t tick() override;
  gxf_result_t stop() override;

 private:
  static constexpr uint32_t kQueueSize = 64;

  // Just holds IMU data and a timestamp
  struct TimestampedImuChannelData {
    std::array<double, 3> data;
    int64_t hw_timestamp;
    int64_t sw_timestamp;
  };

  // for communication between main and async threads
  enum class ThreadStates {
    kOk = 0,        // Should continue execution
    kStopRequested  //  should exit as soon as possible
  };

  enum class DeviceTypes {
    kAccel = 0,
    kGyro,
  };

  // Simple Ring buffer implementation of a queue
  // Not thread safe, only meant to be used here but could be adopted for use elsewhere if needed
  // Overflows are handled by dropping the oldest value
  // Based on std::array rather than fixedVector to avoid needing to call buffer.delete on old
  // elements everytime
  // TODO(sgillen) this should ideally be brought out and tested
  template <typename T, uint32_t capacity>
  class RingBufferQueue {
   public:
    RingBufferQueue() : capacity_(capacity) {}
    gxf::Expected<void> push(T value) {
      buffer_[head_] = value;
      head_ = (head_ + 1) % capacity_;
      size_ += 1;
      if (head_ == tail_) {
        frames_dropped_ += 1;
        // get current time in seconds, can't use getExecutionTime since this is not written as part
        // of a codelet
        int current_warning_time = std::chrono::duration_cast<std::chrono::seconds>(
                                       std::chrono::system_clock::now().time_since_epoch())
                                       .count();

        if (current_warning_time - last_warning_time_ >= 1) {
          GXF_LOG_DEBUG("RingBufferQueue overflowed! Dropped %i values", frames_dropped_);
          last_warning_time_ = current_warning_time;
          frames_dropped_ = 0;
        }
        tail_ = (tail_ + 1) % capacity_;
        size_ -= 1;
      }
      return gxf::Success;
    }
    gxf::Expected<T> pop() {
      if (head_ == tail_) {
        GXF_LOG_ERROR("Popped from empty RingBufferQueue!");
        return gxf::Unexpected{GXF_FAILURE};
      }

      T value = buffer_[tail_];
      tail_ = (tail_ + 1) % capacity_;
      size_ -= 1;
      return value;
    }

    int size() { return size_; }

   private:
    std::array<T, capacity> buffer_;  // underlying buffer
    uint32_t capacity_;               // max number of elements in the queue
    std::atomic<uint32_t> size_ = 0;  // number of elements currently in the queue
    uint32_t head_ = 0;               // index of the head of the queue
    uint32_t tail_ = 0;               // index of the tail of the queue
    uint32_t frames_dropped_ = 0;     // number of frames dropped due to overflow
    int last_warning_time_ = 0;       // time of last warning
  };

  gxf::Expected<void> findDevices();       // find the accelerometer and gyroscope devices
  gxf::Expected<int> getBmiId(const std::string dev_path);  // Get the bmi ID from a sysfs path
  gxf::Expected<void> publishAccelData();  // publish accelerometer data
  gxf::Expected<void> publishGyroData();   // publish gyroscope data

  // Thread for reading data from the devices,
  // We have one thread  for the accelerometer and one for the gyroscope
  void asyncDeviceThread(DeviceTypes device_type);
  // Initialize the sensor buffer
  gxf::Expected<void> initializeDevice(int device_index, float& scale, int& buf_fd);
  // Blocking call to read raw data from the device
  gxf::Expected<int> readRawData(struct pollfd pfd, const int toread, char* data, int& read_size);
  // write an int to a sysfs file and make sure it worked
  gxf::Expected<void> writeAndVerifyInt(const std::string& filename, int val);
  // enable or disable all channels
  gxf::Expected<void> enableDisableAll(const std::string& directoryPath, bool enable);
  // find and read in the scaling factor
  gxf::Expected<float> readScaleFile(const std::string& directoryPath);
  // utility checking if a file name has a certain suffix
  bool endsWith(const std::string& str, const std::string& suffix);
  // convert raw uint16 data to a floating point number in SI units
  float convertRawData(uint16_t input, float scale);
  // clean up resources used by the device
  void cleanupDevice(int device_index, int buf_fd);

  gxf::Parameter<gxf::Handle<gxf::Transmitter>> tx_accelerometer_;
  gxf::Parameter<gxf::Handle<gxf::Transmitter>> tx_gyroscope_;
  gxf::Parameter<int> accel_frequency_;
  gxf::Parameter<int> gyro_frequency_;
  gxf::Parameter<gxf::Handle<PoseTreeFrame>> imu_frame_;
  gxf::Parameter<gxf::Handle<gxf::AsynchronousSchedulingTerm>> async_scheduling_term_;
  gxf::Parameter<int> bmi_id_;

  int accel_index_;  // index of the accelerometer device, populated automatically
  int gyro_index_;   // index of the gyroscope device, populated automatically

  PoseFrameUid imu_frame_uid_;  // IMU frame uid  (from imu_frame_ if present)
  std::thread accel_thread_;    // thread for getting accel data when ready
  std::thread gyro_thread_;     // thread for getting gyro data when ready

  // Used the Imu message for both here so I can still visualize and serialize without additonal
  // changes It does feel a little janky though, and potentially confusing
  RingBufferQueue<TimestampedImuChannelData, kQueueSize> accel_queue_;
  RingBufferQueue<TimestampedImuChannelData, kQueueSize> gyro_queue_;

  std::mutex accel_mutex_;                         // protects accel_data
  std::mutex gyro_mutex_;                          // protects gyro_data
  std::atomic<ThreadStates> main_thread_state_;    // state of the codelet
  std::atomic<ThreadStates> worker_thread_state_;  // state of the codelet

  // The bmi has a startup period, during which it will publish data that is invalid, so we drop
  // the first N samples to account for this
  int accel_samples_to_drop_;
  int accel_samples_dropped_;
  int gyro_samples_to_drop_;
  int gyro_samples_dropped_;
};

}  // namespace isaac
}  // namespace nvidia
