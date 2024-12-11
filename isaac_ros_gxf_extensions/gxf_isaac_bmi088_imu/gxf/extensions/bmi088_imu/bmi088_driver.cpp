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
#include "extensions/bmi088_imu/bmi088_driver.hpp"

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <filesystem>
#include <algorithm>
#include <memory>
#include <string>

#include "gems/gxf_helpers/expected_macro_gxf.hpp"

namespace nvidia {
namespace isaac {

namespace {
// Prefix path to IIO device attributes
constexpr char kIioSysfsPath[] = "/sys/bus/iio/devices/";
constexpr char kAccelPartName[] = "bmi088 accelerometer\n";
constexpr char kGyroPartName[] = "bmi088 gyroscope\n";
constexpr char kIioSysfsDevPath[] = "/sys/bus/iio/devices/iio:device";
constexpr char kIoDevPath[] = "/dev/iio:device";
constexpr int kPollTimeoutSamples = 2;  // timeout if poll takes longer than X samples
constexpr int kPollTimeoutAfterInitMs = 1000;  // timeout after initializing device
constexpr int kIioBufSize = 64;
constexpr int kScanSize = 16;
constexpr int kXIndex = 0;  // channel index for x axis (both accel and gyro)
constexpr int kYIndex = 1;  // channel index for y axis (both accel and gyro)
constexpr int kZIndex = 2;  // channel index for z axis (both accel and gyro)
constexpr int kTIndex = 4;  // channel index for timestamp (both accel and gyro)
constexpr double kStartupTimeS = 0.1;  // Time after startup to wait until data is valid

}  // namespace

gxf_result_t Bmi088Driver::registerInterface(gxf::Registrar* registrar) {
  gxf::Expected<void> result;
  result &= registrar->parameter(
      tx_accelerometer_, "tx_accelerometer", "Accelerometer Output",
      "Transmitter for accelerometer data");
  result &= registrar->parameter(
      tx_gyroscope_, "tx_gyroscope", "Gyroscope Output", "Transmitter for gyroscope data");
  result &= registrar->parameter(
      async_scheduling_term_, "async_scheduling_term", "Asynchronous Scheduling Term",
      "Schedules execution when new data is ready to publish");
  result &= registrar->parameter(
      imu_frame_, "imu_frame", "IMU Frame", "Optional PoseTreeID for IMU data, defaults to zero");
  result &= registrar->parameter(
      accel_frequency_, "accel_frequency", "Accelerometer Update Frequency (Hz)",
      "Frequency to set the accelerometer to.", 100);
  result &= registrar->parameter(
      gyro_frequency_, "gyro_frequency", "Gyroscope Update Frequency (Hz)",
      "Frequency to set the gyroscope to.", 100);
  result &= registrar->parameter(
      bmi_id_, "bmi_id", "BMI088 ID",
      "ID selecting which BMI088 IMU to use. On Carter, there is one IMU on the front hawk with ID "
      "69, And one IMU mounted on the chassis with ID 68. On Carter_v2.3 only IMU 69 is available, "
      "and on Carter_v2.4 both IMUs are available",
      69);
  result &= registrar->parameter(
    recovery_samples_trigger_, "recovery_samples_trigger", "Recovery Samples Trigger",
    "Number of failed samples to trigger recovery.",
    10);

  return gxf::ToResultCode(result);
}

gxf_result_t Bmi088Driver::start() {
  GXF_LOG_DEBUG("Starting bmi088 imu driver");
  imu_frame_uid_.uid = imu_frame_->frame_uid();
  gxf::Expected<void> result = findDevices();
  if (!result) {
    GXF_LOG_ERROR("Failed to find BMI088 devices");
    return GXF_FAILURE;
  }

  accel_samples_to_drop_ = static_cast<int>(std::ceil(accel_frequency_.get() * kStartupTimeS));
  gyro_samples_to_drop_ = static_cast<int>(std::ceil(gyro_frequency_.get() * kStartupTimeS));
  accel_samples_dropped_ = 0;
  gyro_samples_dropped_ = 0;

  worker_thread_state_ = ThreadStates::kOk;
  main_thread_state_ = ThreadStates::kOk;
  async_scheduling_term_->setEventState(nvidia::gxf::AsynchronousEventState::EVENT_WAITING);
  accel_thread_ = std::thread([this] { asyncDeviceThread(DeviceTypes::kAccel); });
  gyro_thread_ = std::thread([this] { asyncDeviceThread(DeviceTypes::kGyro); });

  return GXF_SUCCESS;
}

gxf_result_t Bmi088Driver::tick() {
  if (worker_thread_state_ == ThreadStates::kStopRequested) {
    // This implies and error occurred, so let's stop the graph
    GXF_LOG_ERROR("Error occurred in worker iio thread, shutting down");
    main_thread_state_ = ThreadStates::kStopRequested;
    accel_thread_.join();
    gyro_thread_.join();
    return GXF_FAILURE;
  }

  auto res = gxf::Success;

  if (accel_queue_.size()) {
    res &= publishAccelData();
  }
  if (gyro_queue_.size()) {
    res &= publishGyroData();
  }

  if (!accel_queue_.size() && !gyro_queue_.size()) {
    async_scheduling_term_.get()->setEventState(nvidia::gxf::AsynchronousEventState::EVENT_WAITING);
  }
  return gxf::ToResultCode(res);
}

gxf_result_t Bmi088Driver::stop() {
  main_thread_state_ = ThreadStates::kStopRequested;
  accel_thread_.join();
  gyro_thread_.join();
  return GXF_SUCCESS;
}

gxf::Expected<int> Bmi088Driver::getBmiId(const std::string dev_path) {
  if (!std::filesystem::exists(dev_path + "/of_node")) {
    GXF_LOG_WARNING("Found iio device without of_node at %s", dev_path.c_str());
    return gxf::Unexpected{GXF_FAILURE};
  }
  const std::string of_node = std::filesystem::canonical(dev_path + "/of_node");
  const size_t bmi_id_idx = of_node.rfind("@");
  if (bmi_id_idx == std::string::npos) {
    GXF_LOG_WARNING("Found iio device with unexpected name %s", of_node.c_str());
    return gxf::Unexpected{GXF_FAILURE};
  }
  int bmi_id;
  try {
    bmi_id = std::stoi(of_node.substr(bmi_id_idx + 1));
  } catch (const std::exception&) {
    GXF_LOG_WARNING("Error while extracting bmi_id fron %s", of_node.c_str());
    return gxf::Unexpected{GXF_FAILURE};
  }
  GXF_LOG_DEBUG("Found device at %s", dev_path.c_str());
  return bmi_id;
}

gxf::Expected<void> Bmi088Driver::findDevices() {
  const std::filesystem::path& device_path(kIioSysfsPath);
  int num_accels = 0;
  int num_gyros = 0;
  if (!std::filesystem::exists(device_path) || !std::filesystem::is_directory(device_path)) {
    GXF_LOG_ERROR("Path is not a directory or does not exist: %s", device_path.c_str());
    return gxf::Unexpected{GXF_FAILURE};
  }
  for (const auto& entry : std::filesystem::directory_iterator(device_path)) {
    const std::string entry_path_string = entry.path().string();
    const std::filesystem::path part_path(entry_path_string + "/part");
    if (!std::filesystem::is_regular_file(part_path)) {
      continue;
    }
    std::ifstream part_file(part_path.string());
    if (!part_file.is_open()) {
      GXF_LOG_ERROR("Could not open file: %s", part_path.c_str());
      return gxf::Unexpected{GXF_FAILURE};
    }

    const std::string file_contents(
        (std::istreambuf_iterator<char>(part_file)), std::istreambuf_iterator<char>());

    if (file_contents == std::string(kGyroPartName)) {
      gxf::Expected<int> maybe_id = getBmiId(entry_path_string);
      if (!maybe_id || maybe_id.value() != bmi_id_) {
        continue;
      }
      gyro_index_ = std::stoi(entry_path_string.substr(entry_path_string.size() - 1));
      num_gyros++;
    } else if (file_contents == std::string(kAccelPartName)) {
      gxf::Expected<int> maybe_id = getBmiId(entry_path_string);
      if (!maybe_id || maybe_id.value() != bmi_id_) {
        continue;
      }
      accel_index_ = std::stoi(entry_path_string.substr(entry_path_string.size() - 1));
      num_accels++;
    }
    part_file.close();
  }

  if (num_accels != 1) {
    GXF_LOG_ERROR("Found %d accelerometers for bmi %d expected 1", num_accels, bmi_id_.get());
    return gxf::Unexpected{GXF_FAILURE};
  }
  if (num_gyros != 1) {
    GXF_LOG_ERROR("Found %d gyroscopes for bmi %d expected 1", num_accels, bmi_id_.get());
    return gxf::Unexpected{GXF_FAILURE};
  }

  return gxf::Success;
}

void Bmi088Driver::asyncDeviceThread(DeviceTypes device_type) {
  // Construct local variables based on the device type
  // this is a little awkward but it allows us to use the minimum number of copies and locks
  std::mutex& local_mutex = device_type == DeviceTypes::kAccel ? accel_mutex_ : gyro_mutex_;
  auto& local_queue = device_type == DeviceTypes::kAccel ? accel_queue_ : gyro_queue_;
  const int device_index = device_type == DeviceTypes::kAccel ? accel_index_ : gyro_index_;
  const int device_timeout_ms = std::max(1000 * kPollTimeoutSamples /
      (device_type == DeviceTypes::kAccel ? accel_frequency_.get() : gyro_frequency_.get()), 1);
  int consecutive_poll_fails = 0;
  int timeout_ms = kPollTimeoutAfterInitMs;  // start off waiting longer for a poll

  float scale = 0.0;
  int buf_fd = 0;

  // Initialize the device and get scale info and the buffer fd
  gxf::Expected<void> ret = initializeDevice(device_index, scale, buf_fd);
  if (!ret) {
    GXF_LOG_ERROR("Error intializing device  %i", device_index);
    // Try to cleanup, even though not everything was initialized
    // This keeps everything in a clean state
    cleanupDevice(device_index, buf_fd);
    worker_thread_state_ = ThreadStates::kStopRequested;
    async_scheduling_term_.get()->setEventState(nvidia::gxf::AsynchronousEventState::EVENT_DONE);
    return;
  }

  constexpr int toread = kScanSize * kIioBufSize;
  char data[toread] = {0};
  int read_size = -1;

  struct pollfd pfd = {
      .fd = buf_fd,
      .events = POLLIN,
  };

  while (true) {
    // check if main thread is shutting down
    if (main_thread_state_ == ThreadStates::kStopRequested) {
      GXF_LOG_DEBUG("Stop requested, shutting down");
      break;
    }

    gxf::Expected<int> ret2 = readRawData(pfd, toread, data, read_size, timeout_ms);
    if (!ret2) {
      GXF_LOG_ERROR("Error reading raw data from device %i", device_index);
      worker_thread_state_ = ThreadStates::kStopRequested;
      async_scheduling_term_.get()->setEventState(nvidia::gxf::AsynchronousEventState::EVENT_DONE);
      break;
    } else if (ret2.value() == 0) {
      consecutive_poll_fails++;
      // recover device after a certain amount of expected data is missed because of poll timeout
      if (kPollTimeoutSamples * consecutive_poll_fails >= recovery_samples_trigger_) {
        GXF_LOG_WARNING("Cleaning up device %i and reinitializing because of poll timeout",
            device_index);
        cleanupDevice(device_index, buf_fd);
        ret = initializeDevice(device_index, scale, buf_fd);
        if (!ret) {
          GXF_LOG_ERROR("Error reintializing device  %i", device_index);
          worker_thread_state_ = ThreadStates::kStopRequested;
          async_scheduling_term_.get()->setEventState(
              nvidia::gxf::AsynchronousEventState::EVENT_DONE);
          break;
        }
        pfd.fd = buf_fd;
        timeout_ms = kPollTimeoutAfterInitMs;  // wait longer for poll because of reinitialization
      }
      continue;  // Timeout, just keep waiting
    } else if (timeout_ms != device_timeout_ms) {
      // after successfull poll use smaller timeout proportial to device sampling rate
      timeout_ms = device_timeout_ms;
    }

    // Iterate through all new data, we may have more than one new data point
    for (int i = 0; i < read_size / kScanSize; i++) {
      // for each new data, copy x,y,and z to the imu_data struct
      // minor optimization, we can only copy the most recent data
      // but after startup in almost all cases there is only one new point anyway
      {  // Grab the lock so we can ensure the data is consistent
        std::lock_guard<std::mutex> lock(local_mutex);
        std::array<double, 3> new_data;

        // nominal case for data
        uint16_t x_input =
            *reinterpret_cast<int64_t*>(data + kScanSize * i + kXIndex * sizeof(int16_t));
        new_data[kXIndex] = convertRawData(x_input, scale);

        uint16_t y_input =
            *reinterpret_cast<int64_t*>(data + kScanSize * i + kYIndex * sizeof(int16_t));
        new_data[kYIndex] = convertRawData(y_input, scale);

        uint16_t z_input =
            *reinterpret_cast<int64_t*>(data + kScanSize * i + kZIndex * sizeof(int16_t));
        new_data[kZIndex] = convertRawData(z_input, scale);

        int64_t sw_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::system_clock::now().time_since_epoch())
                                   .count();

        int64_t hw_timestamp =
            *reinterpret_cast<int64_t*>(data + kScanSize * i + kTIndex * sizeof(int16_t));

        local_queue.push({new_data, hw_timestamp, sw_timestamp});
      }  // mutex lock scope
    }    // end data for loop
    // now we have the data, set the event to done so main thread can publish
    async_scheduling_term_.get()->setEventState(nvidia::gxf::AsynchronousEventState::EVENT_DONE);
    consecutive_poll_fails = 0;
  }  // end main while loop

  cleanupDevice(device_index, buf_fd);
  return;
}

gxf::Expected<void> Bmi088Driver::initializeDevice(int device_index, float& scale, int& buf_fd) {
  GXF_LOG_DEBUG("Initializing iio device %i", device_index);

  const std::string device_name = device_index == accel_index_ ?
      "accelerometer" : "gyroscope";

  // Set the sampling frequency for the accelerometer and gyroscope
  const std::string freq_device = device_index == accel_index_ ?
      "/in_accel_sampling_frequency" : "/in_anglvel_sampling_frequency";
  const std::string available_freqs = device_index == accel_index_ ?
      "25, 50, 100, 200, 400, 800, 1600" : "100, 200, 400, 1000, 2000";
  const std::string freq_path = kIioSysfsDevPath + std::to_string(device_index) + freq_device;
  const int frequency =
      device_index == accel_index_ ? accel_frequency_.get() : gyro_frequency_.get();
  RETURN_IF_ERROR(
      writeAndVerifyInt(freq_path, frequency),
      nvidia::Severity::ERROR,
      "Failed to set " + device_name + " frequency, available frequencies are: " + available_freqs);

  const std::string dev_buf_path = kIoDevPath + std::to_string(device_index);
  const std::string sysfs_path = kIioSysfsDevPath + std::to_string(device_index);
  const std::string sysfs_buf_path = kIioSysfsDevPath + std::to_string(device_index) + "/buffer/";

  RETURN_IF_ERROR(
      enableDisableAll(sysfs_path + "/scan_elements", 1), nvidia::Severity::ERROR,
      "Failed to initialize device");

  scale = UNWRAP_OR_RETURN(
      readScaleFile(sysfs_path), nvidia::Severity::ERROR, "Failed to read scale file!");

  RETURN_IF_ERROR(
      writeAndVerifyInt(sysfs_buf_path + "length", kIioBufSize), nvidia::Severity::ERROR,
      "Failed to set length");

  RETURN_IF_ERROR(
      writeAndVerifyInt(sysfs_buf_path + "enable", 1), nvidia::Severity::ERROR,
      "Failed to enable syfs buffer");

  // Attempt to open non blocking the access dev
  buf_fd = open(const_cast<char*>(dev_buf_path.c_str()), O_RDONLY | O_NONBLOCK);
  if (buf_fd < 0) {
    GXF_LOG_ERROR("Failed to open buffer , return code: %s", strerror(-buf_fd));
    return gxf::Unexpected{GXF_FAILURE};
  }

  GXF_LOG_INFO("iio device %i init succesful", device_index);
  return gxf::Success;
}

// return 0 on timeout or no data
// return read_size on read
// return unexpected on failure
gxf::Expected<int> Bmi088Driver::readRawData(
    struct pollfd pfd, const int toread, char data[], int& read_size, const int timeout_ms) {
  int ret = poll(&pfd, 1, timeout_ms);
  if (ret < 0) {
    // poll uses errno to communicate errors rather than return
    GXF_LOG_ERROR("Failed to poll IIO buffer %s", strerror(errno));
    return gxf::Unexpected{GXF_FAILURE};
  } else if (ret == 0) {
    GXF_LOG_WARNING("Poll iio buffer timeout, retrying");
    return 0;
  }

  read_size = read(pfd.fd, data, toread);
  if (read_size < 0) {
    if (errno == EAGAIN) {
      GXF_LOG_WARNING("nothing available\n");
      return 0;  // just continue
    } else {
      GXF_LOG_ERROR("Error reading from buffer, errno: %s", strerror(errno));
      return read_size;
    }
  }
  return read_size;
}

// Convert raw data to SI units
float Bmi088Driver::convertRawData(uint16_t input, float scale) {
  return (static_cast<float>(static_cast<int16_t>(input))) * scale;
}

void Bmi088Driver::cleanupDevice(int device_index, int buf_fd) {
  const std::string sysfs_path = kIioSysfsDevPath + std::to_string(device_index);
  const std::string sysfs_buf_path = kIioSysfsDevPath + std::to_string(device_index) + "/buffer/";

  GXF_LOG_DEBUG("Cleaning up device %i", device_index);
  gxf::Expected<void> ret = writeAndVerifyInt(sysfs_buf_path + "enable", 0);
  if (!ret) {
    GXF_LOG_ERROR("Failed to disable buffer");
  }

  ret = enableDisableAll(sysfs_path + "/scan_elements", 0);
  if (!ret) {
    GXF_LOG_ERROR("Failed to disable channels");
  }

  if (close(buf_fd) == -1) {
    // just carry on, nothing we can do except message user, we are shutting down
    GXF_LOG_ERROR("Failed to close IIO buffer %s", strerror(errno));
  }

  return;
}

gxf::Expected<void> Bmi088Driver::publishAccelData() {
  AccelerometerMessageParts message = UNWRAP_OR_RETURN(CreateAccelerometerMessage(context()));
  {
    std::lock_guard<std::mutex> lock(accel_mutex_);
    gxf::Expected<TimestampedImuChannelData> maybe_data = accel_queue_.pop();
    if (!maybe_data) {
      GXF_LOG_ERROR("Failed to pop data from queue");
      return gxf::ForwardError(maybe_data);
    }
    TimestampedImuChannelData data = maybe_data.value();
    message.accelerometer->linear_acceleration_z = data.data[kZIndex];
    message.accelerometer->linear_acceleration_y = data.data[kYIndex];
    message.accelerometer->linear_acceleration_x = data.data[kXIndex];
    message.timestamp->acqtime = data.hw_timestamp;
    auto sw_timestamp_entity = UNWRAP_OR_RETURN(message.message.add<int64_t>("sw_timestamp"));
    *sw_timestamp_entity = data.sw_timestamp;
  }

  *message.pose_frame_uid = imu_frame_uid_;
  message.timestamp->pubtime = getExecutionTimestamp();

  if (accel_samples_dropped_ < accel_samples_to_drop_) {
    accel_samples_dropped_++;
    return gxf::Success;
  } else {
    return tx_accelerometer_->publish(message.message, message.timestamp->acqtime);
  }
}

gxf::Expected<void> Bmi088Driver::publishGyroData() {
  GyroscopeMessageParts message = UNWRAP_OR_RETURN(CreateGyroscopeMessage(context()));
  {
    std::lock_guard<std::mutex> lock(gyro_mutex_);
    gxf::Expected<TimestampedImuChannelData> maybe_data = gyro_queue_.pop();
    if (!maybe_data) {
      GXF_LOG_ERROR("Failed to pop data from queue");
      return gxf::ForwardError(maybe_data);
    }
    TimestampedImuChannelData data = maybe_data.value();
    message.gyroscope->angular_velocity_z = data.data[kZIndex];
    message.gyroscope->angular_velocity_y = data.data[kYIndex];
    message.gyroscope->angular_velocity_x = data.data[kXIndex];
    message.timestamp->acqtime = data.hw_timestamp;
    auto sw_timestamp_entity = UNWRAP_OR_RETURN(message.message.add<int64_t>("sw_timestamp"));
    *sw_timestamp_entity = data.sw_timestamp;
  }

  *message.pose_frame_uid = imu_frame_uid_;
  message.timestamp->pubtime = getExecutionTimestamp();
  if (gyro_samples_dropped_ < gyro_samples_to_drop_) {
    gyro_samples_dropped_++;
    return gxf::Success;
  } else {
    return tx_gyroscope_->publish(message.message, message.timestamp->acqtime);
  }
}

bool Bmi088Driver::endsWith(const std::string& str, const std::string& suffix) {
  if (str.length() >= suffix.length()) {
    return (0 == str.compare(str.length() - suffix.length(), suffix.length(), suffix));
  }
  return false;
}

gxf::Expected<void> Bmi088Driver::enableDisableAll(const std::string& directory_path, bool enable) {
  for (const auto& entry : std::filesystem::directory_iterator(directory_path)) {
    if (entry.is_regular_file() && endsWith(entry.path().stem().string(), "_en")) {
      const std::string filename = entry.path().string();
      RETURN_IF_ERROR(writeAndVerifyInt(filename, enable));
    }
  }
  return gxf::Success;
}

gxf::Expected<float> Bmi088Driver::readScaleFile(const std::string& directory_path) {
  for (const auto& entry : std::filesystem::directory_iterator(directory_path)) {
    std::string file_path = entry.path().string();
    if (endsWith(file_path, "_scale")) {
      std::ifstream file_stream(file_path);
      float scale;
      file_stream >> scale;
      return scale;
    }
  }

  GXF_LOG_ERROR("No scale file found!");
  return gxf::Unexpected{GXF_FAILURE};
}

gxf::Expected<void> Bmi088Driver::writeAndVerifyInt(const std::string& filename, int value) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    GXF_LOG_ERROR("Error opening %s", filename.c_str());
    return gxf::Unexpected{GXF_FAILURE};
  }
  file << value;
  file.close();

  // Read the file to check if the value was written
  std::ifstream input_file(filename);
  int read_value;
  if (!(input_file >> read_value)) {
    GXF_LOG_ERROR("Error reading back from %s", filename.c_str());
    return gxf::Unexpected{GXF_FAILURE};
  }

  if (read_value != value) {
    GXF_LOG_ERROR(
        "value read back from %s does not match the expected value %d", filename.c_str(), value);
    return gxf::Unexpected{GXF_FAILURE};
  }

  return gxf::Success;
}

}  // namespace isaac
}  // namespace nvidia
