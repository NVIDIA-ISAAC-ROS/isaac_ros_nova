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
import json
import os

from isaac_ros_data_validation.bag_tools import do_validation, VERBOSITY_MAP

"""
Analyze single ROS bag file, e.g.
python -m isaac_ros_data_validation.summarize_dir /some/bag/file.mcap
"""

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Rosbag input file path.')
    parser.add_argument('input_file', type=str, help='Path to the input file')
    parser.add_argument(
        '-v',
        '--verbosity',
        type=str,
        default='warning',
        help='Verbosity level (default: warning)',
    )

    args = parser.parse_args()

    if args.verbosity not in VERBOSITY_MAP:
        print(
            f'Verbosity level {args.verbosity} not recognized. '
            'Options are: {VERBOSITY_MAP.keys()}, defaulting to warning')
        args.verbosity = 'warning'

    _, _, _, q_scores = do_validation(args.input_file, verbose=VERBOSITY_MAP[args.verbosity])

    # Check if the input is a directory
    if os.path.isdir(args.input_file):
        output_directory = args.input_file
    else:
        output_directory = os.path.dirname(args.input_file)

    with open(os.path.join(output_directory, 'q_scores.json'), 'w') as f:
        json.dump(q_scores, f)
