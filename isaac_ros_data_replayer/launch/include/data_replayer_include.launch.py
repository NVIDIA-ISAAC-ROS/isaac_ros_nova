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

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as all_types


def generate_launch_description():
    args = lu.ArgumentContainer()
    args.add_arg('rosbag')
    args.add_arg('enabled_stereo_cameras')
    args.add_arg('enabled_fisheye_cameras')
    args.add_arg('enable_3d_lidar')
    args.add_arg('replay_clock', 'True')
    args.add_arg('replay_loop', 'False')
    args.add_arg('replay_rate', '1.0')
    args.add_arg('replay_delay', 'None')
    args.add_arg('replay_additional_args', '--disable-keyboard-controls')
    args.add_arg('container_name', 'nova_container')

    actions = args.get_launch_actions()
    actions.append(
        lu.play_rosbag(args.rosbag,
                       clock=args.replay_clock,
                       loop=args.replay_loop,
                       rate=args.replay_rate,
                       delay=args.replay_delay,
                       additional_bag_play_args=args.replay_additional_args))
    actions.append(
        lu.include(
            'isaac_ros_data_replayer',
            'launch/include/hawks_decoding.launch.py',
            launch_arguments={
                'enabled_stereo_cameras': args.enabled_stereo_cameras,
            },
            condition=all_types.IfCondition(lu.is_valid(args.enabled_stereo_cameras)),
        ))
    actions.append(
        lu.include(
            'isaac_ros_data_replayer',
            'launch/include/owls_decoding.launch.py',
            launch_arguments={
                'enabled_fisheye_cameras': args.enabled_fisheye_cameras,
            },
            condition=all_types.IfCondition(lu.is_valid(args.enabled_fisheye_cameras)),
        ))
    actions.append(
        lu.include('isaac_ros_hesai',
                   'launch/hesai.launch.py',
                   launch_arguments={
                       'namespace': 'front_3d_lidar',
                       'replay': True,
                   },
                   condition=all_types.IfCondition(args.enable_3d_lidar)))

    return all_types.LaunchDescription(actions)
