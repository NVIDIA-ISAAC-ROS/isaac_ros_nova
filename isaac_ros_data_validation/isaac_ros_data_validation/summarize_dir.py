# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import argparse
import glob
import os

from isaac_ros_data_validation.bag_tools import do_validation, VERBOSITY_MAP

"""
Analyze directory with ROS bag files, e.g.
python analyze_dir.py /workspaces/isaac_ros-dev/rosbag_output/cuvslam2/
"""

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process a directory of ROS bag files.')
    parser.add_argument('directory', type=str, help='Directory containing ROS bag files')
    parser.add_argument(
        '-v',
        '--verbosity',
        type=str,
        choices=VERBOSITY_MAP.keys(),
        default='compact',
        help='Verbosity level (default: compact)',
    )

    args = parser.parse_args()

    all_stats = {}
    all_errors = {}

    for subdir in os.listdir(args.directory):
        full_path = os.path.join(args.directory, subdir)
        if os.path.isdir(full_path):
            mcap_files = glob.glob(os.path.join(full_path, '*.mcap'))
            for input_file in mcap_files:
                try:
                    do_validation(input_file, verbose=VERBOSITY_MAP[args.verbosity])
                    print('\n')
                except Exception as e:
                    print(f'Caught exception: {e}')
                    print('Continuing anyway ... ')
