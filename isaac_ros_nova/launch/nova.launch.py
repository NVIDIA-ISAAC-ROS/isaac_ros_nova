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

from pathlib import Path
from typing import List

import isaac_ros_launch_utils as lu
from launch import Action, LaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
import yaml


def monitor_node_description(frequency: float):
    monitor_node = lu.ComposableNode(
        name='monitor_node',
        package='isaac_ros_nova',
        plugin='nvidia::isaac_ros::nova_monitor::MonitorNode',
        parameters=[{
            'publish_rate': frequency
        }]
    )
    return monitor_node


def resize_node_description(namespace: str, lens_name: str, output_width: int, output_height: int):
    """Generate resize node given a ROS namespace and lens name."""
    remappings = []
    # Allow for any lens names
    if lens_name:
        remappings.append(('image', f'{lens_name}/image_raw'))
        remappings.append(('camera_info', f'{lens_name}/camera_info'))
        remappings.append(('resize/image', f'{lens_name}/resize/image_raw'))
        remappings.append(('resize/camera_info', f'{lens_name}/resize/camera_info'))
    else:  # single lens case
        remappings.append(('image', 'image_raw'))
        remappings.append(('camera_info', 'camera_info'))
        remappings.append(('resize/image', 'resize/image_raw'))
        remappings.append(('resize/camera_info', 'resize/camera_info'))
    resize_node = lu.ComposableNode(
        name=f'{lens_name}_resize_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        namespace=namespace,
        parameters=[{
            'output_width': output_width,
            'output_height': output_height,
            'type_negotiation_duration_s': 10,
            'input_qos': 'SENSOR_DATA',
            'input_qos_depth': 1,
            'output_qos': 'SENSOR_DATA',
            'output_qos_depth': 1,
        }],
        remappings=remappings,
    )
    return resize_node


def preview_image(namespace: str, lens_name: str, resize_width: int, resize_height: int):
    """Pipeline for previewing image efficiently."""
    composable_nodes = []
    preview_resize_node = resize_node_description(namespace, lens_name, resize_width,
                                                  resize_height)
    composable_nodes.append(preview_resize_node)
    return composable_nodes


def preview_camera(namespace: str, camera_type: str, image_size: List[int]):
    """Pipeline for previewing camera efficiently."""
    if len(image_size) != 2:
        raise ValueError(f'"visual_image" launch argument must be array of two integers, not '
                         f'{image_size}')
    resize_width = image_size[0]
    resize_height = image_size[1]
    composable_nodes = []
    if camera_type == 'stereo':
        composable_nodes.extend(preview_image(namespace, 'left', resize_width, resize_height))
        composable_nodes.extend(preview_image(namespace, 'right', resize_width, resize_height))
    elif camera_type == 'fisheye':
        composable_nodes.extend(preview_image(namespace, 'left', resize_width, resize_height))
    return composable_nodes


def load_sensors(args: lu.ArgumentContainer, app_config, system_config, file, systeminfo) -> \
        List[Action]:

    target_container = args.target_container
    actions = []
    composable_nodes = []

    correlated_timestamp_launch = lu.include(
        'isaac_ros_correlated_timestamp_driver',
        'launch/correlated_timestamp_driver.launch.py',
        launch_arguments={'target_container': target_container},
    )
    actions.append(correlated_timestamp_launch)

    for sensor in app_config['sensors']:
        if sensor not in system_config['sensors']:
            raise KeyError(f'Sensor {sensor} in {file.name} not found in {systeminfo.name}')
        sensor_config = system_config['sensors'][sensor]
        actions.append(
            lu.log_info(['Sensor ', sensor, ' added with type ', sensor_config['type']]))
        if sensor_config['type'] == 'hawk':
            if 'module_id' in sensor_config:
                if args.visualize:
                    composable_nodes.extend(
                        preview_camera(sensor, 'stereo', args.visual_image))
                hawk_launch = lu.include(
                    'isaac_ros_hawk',
                    'launch/hawk.launch.py',
                    launch_arguments={
                        'target_container': target_container,
                        'namespace': sensor,
                        'module_id': str(sensor_config['module_id']),
                    },
                )
                actions.append(hawk_launch)
            else:
                bmi088_launch = lu.include(
                    'isaac_ros_imu_bmi088',
                    'launch/bmi088.launch.py',
                    launch_arguments={
                        'target_container': target_container,
                        'namespace': sensor,
                    }
                )
                actions.append(bmi088_launch)
        elif sensor_config['type'] == 'owl':
            if args.visualize:
                composable_nodes.extend(preview_camera(sensor, 'fisheye', args.visual_image))
            owl_launch = lu.include(
                'isaac_ros_owl',
                'launch/owl.launch.py',
                launch_arguments={
                    'target_container': target_container,
                    'namespace': sensor,
                    'camera_id': str(sensor_config['camera_id']),
                }
            )
            actions.append(owl_launch)
        elif sensor_config['type'] == 'rplidar':
            rplidar_launch = lu.include(
                'isaac_ros_nova',
                'launch/include/rplidar.launch.py',
                launch_arguments={
                    'name': sensor,
                    'udp_ip': sensor_config['ip'],
                }
            )
            actions.append(rplidar_launch)
        elif sensor_config['type'] == 'hesai':
            hesai_launch = lu.include(
                'isaac_ros_hesai',
                'launch/hesai.launch.py',
                launch_arguments={
                    'namespace': sensor,
                    'ip': sensor_config['ip']
                }
            )
            actions.append(hesai_launch)
        elif sensor_config['type'] == 'bmi088':
            bmi088_launch = lu.include(
                'isaac_ros_imu_bmi088',
                'launch/bmi088.launch.py',
                launch_arguments={
                    'target_container': target_container,
                    'namespace': sensor,
                }
            )
            actions.append(bmi088_launch)
        else:
            actions.append(
                lu.log_info(
                    ['ERROR Sensor ', sensor, ' with type ', sensor_config['type'],
                     ' could not be added']))
    if args.diagnostics:
        composable_nodes.append(monitor_node_description(float(args.publish_rate)))
    load_composable_action = lu.load_composable_nodes(target_container, composable_nodes)
    actions.append(load_composable_action)
    return actions


def load_config(args: lu.ArgumentContainer) -> List[Action]:
    # Look for YAML in isaac_ros_nova config folder if it does not exist
    config_path = Path(args.config)
    if not config_path.is_file():
        default_dir = lu.get_path('isaac_ros_nova', 'config')
        default_path = default_dir.joinpath(config_path)
        # append .yaml if there is no suffix
        if not default_path.suffix:
            default_path = default_path.with_suffix('.yaml')
        if not default_path.is_file():
            ls_configs = [entry.stem for entry in default_dir.iterdir() if entry.is_file()]
            ls_configs_str = ' '.join(ls_configs)
            raise ValueError(
                f'Tried to find YAML {args.config} but failed. Use a valid file '
                f'path or one of the available YAMLs: {ls_configs_str}'
            )
        config_path = default_path
    actions = []
    # Component container
    component_container = lu.component_container(
        args.target_container,
        condition=LaunchConfigurationEquals('target_container', 'nova')
    )
    actions.append(component_container)
    # Visualization
    # NOTE: the max_qos_depth setting affects /tf_static and will cause frames to disappear if
    # if it is less than the total number of static broadcasters
    foxglove_bridge_node = lu.Node(
        name='foxglove_bridge_node',
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{
            'send_buffer_limit': 10000000,
            'max_qos_depth': 30,
            'use_compression': True,
            'capabilities': ['clientPublish', 'connectionGraph', 'assets'],
        }],
        arguments=['--ros-args', '--log-level', 'ERROR'],
        condition=IfCondition(LaunchConfiguration('visualize'))
    )
    actions.append(foxglove_bridge_node)
    # Process user specified YAML and systeminfo YAML created by Nova Init
    with (open(config_path, 'r') as file,
          open('/etc/nova/systeminfo.yaml', 'r') as systeminfo):
        app_config = yaml.safe_load(file)
        system_config = yaml.safe_load(systeminfo)

        for module in app_config:
            if module == 'sensors':
                actions.extend(load_sensors(args, app_config, system_config, file, systeminfo))
            elif module == 'calibration':
                actions.append(
                    lu.log_info(['Module ', module, ' added']))
                description_launch = lu.include(
                    'isaac_ros_nova',
                    'launch/include/description.launch.py',
                )
                actions.append(description_launch)
            elif module == 'jtop':
                actions.append(
                    lu.log_info(['Module ', module, ' added']))
                jtop_launch = lu.include(
                    'isaac_ros_jetson_stats',
                    'launch/jtop.launch.py',
                )
                actions.append(jtop_launch)

    return actions


def generate_launch_description():
    actions = []
    args = lu.ArgumentContainer()
    args.add_arg('target_container', 'nova', 'Target Container', cli=True)
    args.add_arg('config', '/etc/nova/systeminfo.yaml', 'Path to Nova YAML file or selection from '
                 'examples in config/ (nova-carter, hawk-4, etc.)', cli=True)
    args.add_arg('diagnostics', 'True', 'Turn on diagnostics', cli=True)
    args.add_arg('visualize', 'False', 'Starts foxglove and other nodes for visualization.',
                 cli=True)
    args.add_arg('visual_image', '[96,60]', 'Image size for all cameras when visualizing. Note, '
                 'larger images can slow performance.', cli=True)
    args.add_arg('publish_rate', '1.0',
                 'Frequency in hz for system monitor updates. Default 1Hz', cli=True)
    args.add_opaque_function(load_config)
    actions.extend(args.get_launch_actions())
    return LaunchDescription(actions)
