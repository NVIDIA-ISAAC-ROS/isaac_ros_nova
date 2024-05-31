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

"""Read rosbag with raw image data of an LED array and visualizing it to verify camera sync."""


# In[1]:

# coding: utf-8
from isaac_ros_data_validation import EXAMPLE_BAG_DIR
from isaac_ros_data_validation.bag_tools import read_rosbag
import matplotlib.pyplot as plt
import numpy as np


INPUT_FILE = f'{EXAMPLE_BAG_DIR}/rosbag2_raw_example'

dfs = read_rosbag(INPUT_FILE, verbose=False, store_data=True)

# In[2]
print(dfs.keys())

# TODO this only works for raw but no reason it shouldn't after we decode
# (or before??)
topic1 = '/hawk/left/image_raw'
topic2 = '/owl/left/image_raw'

plt.plot((dfs[topic1]['acqtime'] - dfs[topic2]['acqtime'][1:]) / 1e3)
plt.grid()
plt.title(f'{topic1} - {topic2} us)')

# In[2]:

idx1 = 0

width = dfs[topic1]['data'][idx1].width
height = dfs[topic1]['data'][idx1].height
image = np.empty((width * height * 3), np.uint8)
image[:] = dfs[topic1]['data'][idx1].data
image = image.reshape(height, width, 3)

fig, ax = plt.subplots(figsize=(32, 40))
ax.imshow(image, interpolation='nearest')

# %%
idx2 = 0

width = dfs[topic2]['data'][idx2].width
height = dfs[topic2]['data'][idx2].height
image = np.empty((width * height * 3), np.uint8)
image[:] = dfs[topic2]['data'][idx2].data
image = image.reshape(height, width, 3)

fig, ax = plt.subplots(figsize=(32, 40))
ax.imshow(image, interpolation='nearest')

print((dfs[topic1]['acqtime'][idx1] - dfs[topic2]['acqtime'][idx2]) / 1e6)

# %%
