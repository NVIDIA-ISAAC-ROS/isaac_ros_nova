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

# %%

from isaac_ros_data_validation import TEST_BAG_DIR
from isaac_ros_data_validation.bag_tools import read_rosbag
import matplotlib.pyplot as plt

INPUT_FILE = f'{TEST_BAG_DIR}/vslam_camera_sync'

dfs = read_rosbag(INPUT_FILE, bagtype='db3')
print(dfs.keys())

# %%

for topic in ['/scan', '/pointcloud',
                       '/front/scan', '/back/scan',
                       '/hawk_front/nvblox_node/static_esdf_pointcloud']:
    plt.plot(dfs[topic]['acqtime'])
    plt.title(topic)
    print(f'len({topic}): {len(dfs[topic]["acqtime"])}')
    plt.grid()
    plt.figure()

# %%

plt.plot(dfs['/pointcloud']['acqtime'])
plt.figure()
plt.plot(dfs['/hawk_front/nvblox_node/static_esdf_pointcloud']['acqtime'])

# %%
plt.plot((dfs['/hawk_front/nvblox_node/static_esdf_pointcloud']
         ['acqtime'] - dfs['/pointcloud']['acqtime'])/1e9)
plt.title('nvblox_pointcloud - /pointcloud')
# %%

topic1 = '/hawk_front/nvblox_node/static_esdf_pointcloud'
topic2 = '/scan'
plt.plot((dfs[topic1]['acqtime'] - dfs[topic2]['acqtime'])/1e9)
plt.title(f'{topic1} - {topic2}')
plt.xlabel('index')
plt.ylabel('diff (seconds)')
plt.grid()

# %%

topic1 = '/hawk_front/nvblox_node/static_esdf_pointcloud'
topic2 = '/front/scan'
plt.plot((dfs[topic1]['acqtime'][::1] - dfs[topic2]['acqtime'][::1])/1e9)
plt.title(f'{topic1} - {topic2}')
plt.xlabel('index')
plt.ylabel('diff (seconds)')
plt.grid()

# Reversing the sequences
reversed_topic1 = dfs[topic1]['acqtime'][::-1].reset_index(drop=True) / 1e9
reversed_topic2 = dfs[topic2]['acqtime'][::-1].reset_index(drop=True) / 1e9

# Plotting
plt.figure()
plt.plot(reversed_topic1 - reversed_topic2)
plt.title(f'Reversed: {topic1} - {topic2}')
plt.xlabel('index')
plt.ylabel('diff (seconds)')
plt.grid()
plt.show()

# %%

topic1 = '/scan'
topic2 = '/front/scan'
plt.plot((dfs[topic1]['acqtime'] - dfs[topic2]['acqtime'])/1e9)
plt.title(f'{topic1} - {topic2}')
plt.xlabel('index')
plt.ylabel('diff (seconds)')
plt.grid()

# %%
