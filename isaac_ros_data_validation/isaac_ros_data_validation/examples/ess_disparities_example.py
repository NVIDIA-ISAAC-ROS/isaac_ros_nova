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

"""Read in a db3 rosbag and plotting some topics."""

# In[1]:

# coding: utf-8
from isaac_ros_data_validation import EXAMPLE_BAG_DIR
from isaac_ros_data_validation.bag_tools import read_rosbag
import matplotlib.pyplot as plt

# For db3, you must specify the directory
INPUT_FILE = f'{EXAMPLE_BAG_DIR}/ess_rosbag'

# if you don't actually need data (just timestamps) set store_data = False for performance
dfs = read_rosbag(INPUT_FILE, verbose=False, store_data=True, bagtype='db3')
print(dfs.keys())

# In[2]

topic1 = '/left/image_rect'
topic2 = '/right/image_rect'

# Noticing very weird timestamps beyond idx=300
cutoff = 300

plt.plot((dfs[topic1]['acqtime'] - dfs[topic2]['acqtime'])[0:cutoff] / 1e6)
plt.grid()
plt.title(f'{topic1} - {topic2} (ms)')
plt.show()
# In[3]:

plt.plot(dfs['/disparity']['acqtime'][:cutoff])
plt.plot(dfs['/left/image_rect']['acqtime'][:cutoff])
plt.plot(dfs['/right/image_rect']['acqtime'][:cutoff])
plt.title('Acqtimes')
plt.grid()
plt.legend(['disparity', 'left', 'right'])
plt.show()

# %%

plt.plot(dfs['/disparity']['timestamp'][:cutoff])
plt.plot(dfs['/left/image_rect']['timestamp'][:cutoff])
plt.plot(dfs['/right/image_rect']['timestamp'][:cutoff])
plt.title('timestamps')
plt.grid()
plt.legend(['disparity', 'left', 'right'])
plt.show()

# %%
