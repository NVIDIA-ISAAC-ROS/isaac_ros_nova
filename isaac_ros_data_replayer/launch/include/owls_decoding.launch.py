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


def create_decoder(fisheye_camera_name: str, identifier: str,
                   args: lu.ArgumentContainer) -> all_types.Action:
    composable_node = all_types.ComposableNode(
        name='decoder_node',
        package='isaac_ros_h264_decoder',
        plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
        namespace=f'{fisheye_camera_name}/{identifier}',
        remappings=[
            ('image_uncompressed', 'image_raw'),
        ],
    )
    condition = all_types.IfCondition(
        lu.has_substring(args.enabled_fisheye_cameras, fisheye_camera_name))
    action = lu.load_composable_nodes(args.container_name, [composable_node], condition=condition)
    return action


def generate_launch_description() -> all_types.LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('container_name', 'nova_container')
    args.add_arg('enabled_fisheye_cameras')

    actions = args.get_launch_actions()
    actions.append(create_decoder('front_fisheye_camera', 'left', args))
    actions.append(create_decoder('back_fisheye_camera', 'left', args))
    actions.append(create_decoder('left_fisheye_camera', 'left', args))
    actions.append(create_decoder('right_fisheye_camera', 'left', args))

    return all_types.LaunchDescription(actions)
