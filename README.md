# Isaac ROS Nova

Optimized Isaac ROS packages for [Isaac Nova Orin](https://developer.nvidia.com/isaac/nova-orin).

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nova/Nova_Carter_Isaac_KV_540p_01_v002_DM.png/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nova/Nova_Carter_Isaac_KV_540p_01_v002_DM.png/" width="800px"/></a></div>

## Overview

[Isaac ROS Nova](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nova) provides a set of optimized packages and tools to interface with the Nova sensor suite.
These packages integrate with hardware timestamp synchronization on Jetson Orin platforms to enable high-quality sensor fusion.
Sensor data streams through Isaac ROS graphs using [NITROS](https://nvidia-isaac-ros.github.io/concepts/nitros/index.html) for NVIDIA-accelerated processing.

> [!Note]
> Isaac ROS Nova requires having run `nova-orin-init` [here](https://nvidia-isaac-ros.github.io/nova/nova_init/index.html#install) on your compute node before using any of the included packages.
* [Hesai Pandar XT32 3D LiDAR](https://www.hesaitech.com/product/xt32/)
* [Leopard Imaging HAWK stereo camera](https://leopardimaging.com/leopard-imaging-hawk-stereo-camera/)
* [Leopard Imaging OWL monocular camera](https://leopardimaging.com/product/automotive-cameras/cameras-by-interface/maxim-gmsl-2-cameras/li-ar0234cs-gmsl2-owl/li-ar0234cs-gmsl2-owl/)
* [Bosch BMI088 IMU](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)

## Quickstart

It is recommended to use Nova through the [Isaac ROS Nova meta-package](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nova/blob/main/isaac_ros_nova). This package will launch all the sensor drivers for your defined Nova system. Guidelines and instructions on its use can be found [here](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_nova/index.html).

## Isaac ROS NITROS Acceleration

This package is powered by [NVIDIA Isaac Transport for ROS (NITROS)](https://developer.nvidia.com/blog/improve-perception-performance-for-ros-2-applications-with-nvidia-isaac-transport-for-ros/), which leverages type adaptation and negotiation to optimize message formats and dramatically accelerate communication between participating nodes.

## Performance

| Sample Graph<br/><br/>                                                                                                                                                                                                           | Input Size<br/><br/>      | Nova Carter<br/><br/>                                                                                                                                                                                     |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [Data Recorder Live Graph](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/benchmarks/isaac_ros_data_recorder_nova_benchmark/scripts/isaac_ros_data_recorder_4_hawk.py)<br/><br/><br/>4 Hawk Cameras<br/><br/> | 1200p<br/><br/><br/><br/> | [22.0 fps (per stream avg)](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_data_recorder_4_hawk-carter-v2.4-jp6.json)<br/><br/><br/>0 dropped frames (avg)<br/><br/> |

---

## Documentation

Please visit the [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/index.html) to learn how to use this repository.

---

## Packages

* [`isaac_ros_correlated_timestamp_driver`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_correlated_timestamp_driver/index.html)
  * [Overview](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_correlated_timestamp_driver/index.html#overview)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_correlated_timestamp_driver/index.html#quickstart)
  * [Troubleshooting](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_correlated_timestamp_driver/index.html#troubleshooting)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_correlated_timestamp_driver/index.html#api)
* [`isaac_ros_data_recorder`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_data_recorder/index.html)
  * [Overview](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_data_recorder/index.html#overview)
  * [Tutorials](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_data_recorder/index.html#tutorials)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_data_recorder/index.html#api)
* [`isaac_ros_data_replayer`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_data_replayer/index.html)
  * [Overview](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_data_replayer/index.html#overview)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_data_replayer/index.html#quickstart)
  * [Troubleshooting](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_data_replayer/index.html#troubleshooting)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_data_replayer/index.html#api)
* [`isaac_ros_data_validation`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_data_validation/index.html)
  * [Overview](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_data_validation/index.html#overview)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_data_validation/index.html#quickstart)
  * [Troubleshooting](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_data_validation/index.html#troubleshooting)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_data_validation/index.html#api)
* [`isaac_ros_ground_calibration`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_ground_calibration/index.html)
  * [Overview](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_ground_calibration/index.html#overview)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_ground_calibration/index.html#quickstart)
  * [Calibration Target](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_ground_calibration/index.html#calibration-target)
  * [Data Recording](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_ground_calibration/index.html#data-recording)
  * [Troubleshooting](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_ground_calibration/index.html#troubleshooting)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_ground_calibration/index.html#api)
* [`isaac_ros_hawk`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_hawk/index.html)
  * [Overview](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_hawk/index.html#overview)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_hawk/index.html#quickstart)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_hawk/index.html#api)
  * [Troubleshooting](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_hawk/index.html#troubleshooting)
* [`isaac_ros_hesai`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_hesai/index.html)
  * [Overview](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_hesai/index.html#overview)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_hesai/index.html#quickstart)
  * [Troubleshooting](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_hesai/index.html#troubleshooting)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_hesai/index.html#api)
* [`isaac_ros_imu_bmi088`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_imu_bmi088/index.html)
  * [Overview](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_imu_bmi088/index.html#overview)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_imu_bmi088/index.html#quickstart)
  * [Troubleshooting](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_imu_bmi088/index.html#troubleshooting)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_imu_bmi088/index.html#api)
* [`isaac_ros_nova`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_nova/index.html)
  * [Overview](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_nova/index.html#overview)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_nova/index.html#quickstart)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_nova/index.html#api)
* [`isaac_ros_nova_recorder`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_nova_recorder/index.html)
  * [Overview](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_nova_recorder/index.html#overview)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_nova_recorder/index.html#quickstart)
  * [Try More Examples](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_nova_recorder/index.html#try-more-examples)
  * [Troubleshooting](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_nova_recorder/index.html#troubleshooting)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_nova_recorder/index.html#api)
* [`isaac_ros_owl`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_owl/index.html)
  * [Overview](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_owl/index.html#overview)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_owl/index.html#quickstart)
  * [Troubleshooting](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_owl/index.html#troubleshooting)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_owl/index.html#api)

## Latest

Update 2024-12-10: Added ground calibration support
