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

"""Example of reading in a rosbag and plotting some topics."""


# In[1]:

import pprint

from isaac_ros_data_validation import TEST_BAG_DIR
from isaac_ros_data_validation.bag_tools import do_validation
import matplotlib.pyplot as plt

INPUT_FILE = f'{TEST_BAG_DIR}/owl-3/test.mcap'

stats, errors, dfs, q_scores = do_validation(INPUT_FILE, verbose=False)

# In[2]:
# Print all stats/errors
pprint.pprint(errors)
pprint.pprint(stats)

# In[3]:
# Or do operatations on things
print(stats['/left_owl/left/image_compressed']['max_absolute_error_all_ms'] < 0.001)

# In[4]:
# Or plot things / operate on the actual ros data

topic1 = '/left_owl/left/image_compressed'
topic2 = '/right_owl/left/image_compressed'

plt.plot((dfs[topic1]['acqtime'] - dfs[topic2]['acqtime']) / 1e3)
plt.grid()
plt.title(f'{topic1} - {topic2} us)')
