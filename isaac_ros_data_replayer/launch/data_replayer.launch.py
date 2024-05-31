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

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as all_types


def generate_launch_description():
    args = lu.ArgumentContainer()
    args.add_arg('rosbag', cli=True)
    args.add_arg('enabled_stereo_cameras',
                 'front_stereo_camera,back_stereo_camera,left_stereo_camera,right_stereo_camera',
                 cli=True)
    args.add_arg(
        'enabled_fisheye_cameras',
        'front_fisheye_camera,back_fisheye_camera,left_fisheye_camera,right_fisheye_camera',
        cli=True)
    args.add_arg('enable_3d_lidar', True, cli=True)
    args.add_arg('replay_loop', 'False', cli=True)
    args.add_arg('replay_rate', '1.0', cli=True)
    args.add_arg('replay_delay', 'None', cli=True)
    args.add_arg('replay_additional_args', '--disable-keyboard-controls', cli=True)

    args.add_arg('type_negotiation_duration_s', 2, cli=True)

    actions = args.get_launch_actions()
    actions.append(lu.component_container('nova_container'))
    actions.append(
        all_types.SetParameter('type_negotiation_duration_s', args.type_negotiation_duration_s))
    actions.append(
        lu.include('isaac_ros_data_replayer', 'launch/include/data_replayer_include.launch.py'))
    return all_types.LaunchDescription(actions)
