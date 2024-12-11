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

# Runs nova_recorder.launch.py with a specified yaml file, then summarizes the outputs automatically
# Meant to be run inside of the dev container.

# TODO(bmchale): find way to get argus and syslog from inside container

set -m

RECORDING_DIR="/mnt/nova_ssd/recordings"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Default paths
app_config="nova-carter_hawk-4.yaml"
bag_file_base="rosbag2"
skip_validation=false
verbosity="warning"
extra_launch_arguments=""

show_help() {
    echo "Usage: $0 [options]"
    echo
    echo "Options:"
    echo "  -h                   Display this help message."
    echo "  -y | --yaml PATH     Path to the YAML spec file. If not provided, $app_config is used."
    echo "  -o | --output STRING String to prefix the output bag file. If not provided, defaults to '$bag_file_base'."
    echo "  -d | --directory     Directory to save the recording. If not provided, defaults to '$RECORDING_DIR'."
    echo "  -v | --verbosity     Verbosity level for data validation. can be dump, error, info, compact, or warning, If not provided, defaults to 'warning'."
    echo "  --skip_validation    Skip the validation process."
    echo "  --                   Everything after this is forwarded to the ros application"
    echo
    echo "Example:"
    echo "  $0 -y /path/to/yaml_file.yaml -o unique_name"
    echo "  $0 --yaml /path/to/yaml_file.yaml --output unique_name"
    echo "  $0 -h"
    echo
}

handle_sigint() {
    # Forward the SIGINT to the last process
    echo "Signint caught, stopping recorder"
    kill -SIGINT "$ros2_launch_pid"
    wait "$ros2_launch_pid"
}

generate_git_summary() {
    local output_file=$1
    local separator="----------------------------------------------"

    error_handler() {
        echo "Git info not available" > "$output_file"
        exit 1
    }
    trap 'error_handler' ERR

    # For now, just cd into the script dir, we will probabaly want to add summaries for
    # all relavent repos eventually
    cd "$SCRIPT_DIR"
    # Check if we're in a Git repository
    if ! git -C "$git_repo_dir" rev-parse --is-inside-work-tree > /dev/null 2>&1; then
        error_handler
    fi

    echo "$separator" >> "$output_file"
    echo "Status" >> "$output_file"
    echo "$separator" >> "$output_file"
    git status >> "$output_file" || error_handler

    # Write the recent commit history to the file
    echo "$separator" >> "$output_file"
    echo "Recent commit history:" >> "$output_file"
    echo "$separator" >> "$output_file"
    git log --oneline -5 >> "$output_file" || error_handler
    echo >> "$output_file"

    # Check the Git status and append to the file
    status=$(git status --porcelain) || error_handler
    if [ -z "$status" ]; then
        echo "$separator" >> "$output_file"
        echo "No uncommitted changes." >> "$output_file"
    else
        echo "$separator" >> "$output_file"
        echo "Uncommitted changes:" >> "$output_file"
        echo "$separator" >> "$output_file"
        git status >> "$output_file" || error_handler
        echo $separator >> "$output_file"
        echo "Diff of uncommitted changes:" >> "$output_file"
        echo "$separator" >> "$output_file"
        git diff >> "$output_file" || error_handler
    fi

    echo "$separator" >> "$output_file"
}

trap handle_sigint SIGINT

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
            bag_file_base="$2"
            shift # past argument
            shift # past value
            ;;
        -d|--directory)
            RECORDING_DIR="$2"
            shift # past argument
            shift # past value
            ;;
        -v|--verbosity)
            verbosity="$2"
            shift # past argument
            shift # past value
            ;;
         --skip_validation)
            skip_validation=true
            shift # past argument
            ;;
         --)
            shift # past argument
            extra_launch_arguments="$@"
            break
            ;;
        *)
            # Unknown option
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done


if ! python -c "import isaac_ros_data_validation" &> /dev/null; then
    echo "isaac_ros_data_validation is not installed. Exiting."
    exit 1
fi

config_arg="config:=${app_config}"
rosbag_arg="recording_name:=${bag_file_base}"
headless_arg="headless:=True"
recording_dir_arg="recording_directory:=${RECORDING_DIR}"
ros2_launch_command="ros2 launch isaac_ros_nova_recorder nova_recorder.launch.py $config_arg $rosbag_arg $recording_dir_arg $headless_arg $extra_launch_arguments"
echo "Running the recorder with '$ros2_launch_command'"

# Run the data recorder and grab the output
start=$(date +%s)
$ros2_launch_command &
ros2_launch_pid=$!

wait

output_dir=$(ls -td ${RECORDING_DIR}/*/ | head -1)
timestamp=$(stat -c %Y ${output_dir})

# Only run validation if a new rosbag was created
if [[ $timestamp -gt $start ]]; then
    echo "Recorded the following bag:"
    ros2 bag info $output_dir

    if [ "$skip_validation" = false ]; then
        SECONDS=0
        bag_check_command="python -m isaac_ros_data_validation.summarize_bag -v $verbosity $output_dir"
        echo "Checking the bag with '$bag_check_command'"
        eval $bag_check_command | tee "${output_dir}/data_validation.txt"
        bag_check_duration=$SECONDS
        bag_check_minutes=$((bag_check_duration / 60))
        bag_check_seconds=$((bag_check_duration % 60))
    fi

    # Check the time and size the output directory
    bag_check_time="${bag_check_minutes}m ${bag_check_seconds}s"

    echo "Data validation took $bag_check_time."
    echo "Bag was saved in $output_dir"

    # Now add some extra info to the bag file
    generate_git_summary "${output_dir}/git_summary.txt"
else
    echo "Rosbag not found"
fi
