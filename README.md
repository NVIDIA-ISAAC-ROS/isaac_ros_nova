# Isaac ROS Nova

Optimized Isaac ROS packages for [Isaac Nova Orin](https://developer.nvidia.com/isaac/nova-orin).

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nova/Nova_Carter_Isaac_KV_540p_01_v002_DM.png/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_nova/Nova_Carter_Isaac_KV_540p_01_v002_DM.png/" width="800px"/></a></div>

## Overview

[Isaac ROS Nova](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nova) provides a set of optimized packages to interface with the [Isaac Nova Orin](https://developer.nvidia.com/isaac/nova-orin) sensor suite.
These packages integrate with hardware timestamp synchronization on Jetson Orin platforms to enable high-quality sensor fusion.
Sensor data streams through Isaac ROS graphs using [NITROS](https://nvidia-isaac-ros.github.io/concepts/nitros/index.html) for hardware-accelerated processing.

* [Hesai Pandar XT32 3D LiDAR](https://www.hesaitech.com/product/xt32/)
* [Leopard Imaging HAWK stereo camera](https://leopardimaging.com/leopard-imaging-hawk-stereo-camera/)
* [Leopard Imaging OWL monocular camera](https://leopardimaging.com/product/automotive-cameras/cameras-by-interface/maxim-gmsl-2-cameras/li-ar0234cs-gmsl2-owl/li-ar0234cs-gmsl2-owl/)
* [Bosch BMI088 IMU](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)

## Isaac ROS NITROS Acceleration

This package is powered by [NVIDIA Isaac Transport for ROS (NITROS)](https://developer.nvidia.com/blog/improve-perception-performance-for-ros-2-applications-with-nvidia-isaac-transport-for-ros/), which leverages type adaptation and negotiation to optimize message formats and dramatically accelerate communication between participating nodes.

---

## Documentation

Please visit the [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/index.html) to learn how to use this repository.

---

## Packages

* [`isaac_ros_correlated_timestamp_driver`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_correlated_timestamp_driver/index.html)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_correlated_timestamp_driver/index.html#api)
* [`isaac_ros_hawk`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_hawk/index.html)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_hawk/index.html#api)
* [`isaac_ros_hesai`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_hesai/index.html)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_hesai/index.html#api)
* [`isaac_ros_imu_bmi088`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_imu_bmi088/index.html)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_imu_bmi088/index.html#api)
* [`isaac_ros_owl`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_owl/index.html)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nova/isaac_ros_owl/index.html#api)

## Latest

Update 2023-10-18: Initial release
