#!/bin/bash

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

# Restarts the dev container, retarts argus, runs record_data.sh, saves the output and extra logs
# from argus and the kernel. Meant to be run outside of the dev container


RECORDING_DIR="/mnt/nova_ssd/recordings"

if [ "$EUID" -ne 0 ]; then
    echo "This script must be run as root. Use sudo."
    exit 1
fi

show_help() {
    echo "Usage: $0 [options]"
    echo
    echo "Options:"
    echo "  -h                   Display this help message."
    echo "  -y | --yaml PATH     Path to the YAML spec file. If not provided, 4-hawk.yaml is used."
    echo "  -o | --output STRING String to prefix the output bag file. If not provided, defaults to 'rosbag2'."
    echo "  --skip_validation    Skip the validation process."
    echo
    echo "Example:"
    echo "  $0 -y /path/to/yaml_file.yaml -o unique_name"
    echo "  $0 --yaml /path/to/yaml_file.yaml --output unique_name"
    echo "  $0 -h"
    echo
}

restart_and_run_recorder() {
    local config_file="$1"
    local bag_base_name="$2"
    local skip_validation="$3"

    current_date_time=$(date +"%Y_%m_%d-%H_%M_%S")
    bag_file_name="${bag_base_name}_${current_date_time}"
    output_dir="${RECORDING_DIR}/${bag_file_name}"

    docker stop isaac_ros_dev-aarch64-container || true
    wait

    start_time=$(date +"%Y-%m-%d %H:%M:%S")

    systemctl restart nvargus-daemon.service

    # Taken from carter_dev's run_dev.sh
    docker run -d --rm \
           --privileged \
           --network host \
           -v /tmp/.X11-unix:/tmp/.X11-unixx -v /home/nvidia/.Xauthority:/home/admin/.Xauthority:rw -e DISPLAY -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml -e ROS_DOMAIN_ID -e USER -v /usr/bin/tegrastats:/usr/bin/tegrastats -v /tmp/argus_socket:/tmp/argus_socket -v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11 -v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11 -v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10 -v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcufft.so.10:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcufft.so.10 -v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so -v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4 -v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1 -v /usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h:/usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra -v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api -v /opt/nvidia/nsight-systems-cli:/opt/nvidia/nsight-systems-cli --pid=host -v /opt/nvidia/vpi2:/opt/nvidia/vpi2 -v /usr/share/vpi2:/usr/share/vpi2 -v /run/jtop.sock:/run/jtop.sock:ro --group-add 1001 -v /etc/nova/:/etc/nova/ -v /etc/nova/calibration:/etc/nova/calibration/ -v /mnt/nova_ssd/recordings:/mnt/nova_ssd/recordings \
           -v /mnt/nova_ssd/workspaces/carter-dev/:/workspaces/isaac_ros-dev/ \
           -v /dev/*:/dev/* \
           -v /etc/localtime:/etc/localtime:ro \
           --name "isaac_ros_dev-aarch64-container" \
           --runtime nvidia \
           --user="admin" \
           --workdir /workspaces/isaac_ros-dev \
           isaac_ros_dev-aarch64 \
           /bin/bash -c 'tail -f /dev/null'

    docker exec -it isaac_ros_dev-aarch64-container bash -c "source ros_ws/install/setup.bash &&
                                                             /workspaces/isaac_ros-dev/ros_ws/src/isaac_ros_data_recorder/isaac_ros_data_recorder/scripts/record_data.sh -y ${config_file} -o ${bag_file_name} --override_name$( [ "$skip_validation" = "true" ] && echo ' --skip_validation')"


    journalctl -u nvargus-daemon.service --since "$start_time"  > "${output_dir}/argus_log.txt"
    dmesg > "${output_dir}/dmesg.txt"

}


# Default paths
app_config="/mnt/nova_ssd/workspaces/carter-dev/ros_ws/src/isaac_ros_data_recorder/isaac_ros_data_recorder/config/nova-carter_hawk-4.yaml"
bag_base_name="rosbag2"
skip_validation=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -y|--yaml)
            app_config="$2"
            shift # past argument
            shift # past value
            ;;
        -o|--output)
            bag_base_name="$2"
            shift # past argument
            shift # past value
            ;;
        --skip_validation)
            skip_validation=true
            shift # past argument
            ;;
        *)
            # Unknown option
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Replace part of the path
app_config=$(readlink -f "$app_config")
new_root="/workspaces/isaac_ros-dev"
old_root="/mnt/nova_ssd/workspaces/carter-dev"
app_config="${app_config/$old_root/$new_root}"

echo $app_config

restart_and_run_recorder "${app_config}" "${bag_base_name}" "${skip_validation}"

