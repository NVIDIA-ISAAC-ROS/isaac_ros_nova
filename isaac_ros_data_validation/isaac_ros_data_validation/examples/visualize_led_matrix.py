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

"""
Load a multi cam rosbag and visualize individual frames.

Visualizes a scene looking at an LED array, accounts for frame drops to get a ground truth
check of frame times. Requires setting up a jupyter server or other environment with matplotlib
"""

# %%

import glob
import subprocess
import tempfile

import cv2
from isaac_ros_data_validation import EXAMPLE_BAG_DIR, REPLAYER_SCRIPT_DIR
from isaac_ros_data_validation.bag_tools import do_validation
import matplotlib.pyplot as plt


OUT_DIR = tempfile.mkdtemp()
MP4_SCRIPT = f'{REPLAYER_SCRIPT_DIR}/mp4_converter.py'
CAMERA_POSITIONS = ['front', 'right', 'left', 'back']
INPUT_FILE = f'{EXAMPLE_BAG_DIR}/led_matrix'

# %%

stats, errors, dfs, q_scores = do_validation(INPUT_FILE)

# %%

for position in CAMERA_POSITIONS:
    camera_topic = f'{position}_stereo_camera'
    print(f'Converting {position} camera recordings...')
    subprocess.run(['python', MP4_SCRIPT, INPUT_FILE, camera_topic, OUT_DIR], check=True)

# %%


def visualize_specific_frames_of_mp4_files(directory, frame_offsets):
    """Visualizes specific frames of mp4 files in a directory."""
    video_files = sorted(glob.glob(f'{directory}/*_stereo_camera_*.mp4'),
                         key=lambda x: (x.split('_stereo_')[0], 'right' in x))

    # Calculate rows based on the number of video files found
    cols = 2
    if len(video_files) % cols == 0:
        rows = len(video_files) // cols
    else:
        rows = (len(video_files) // cols) + 1

    fig, axs = plt.subplots(rows, cols, figsize=(15, rows * 5))
    plt.subplots_adjust(wspace=0.1, hspace=0.2)

    if rows * cols != 1:
        axs = axs.flatten()
    else:
        axs = [axs]

    for i, file_path in enumerate(video_files):
        file_name = file_path.split('/')[-1].split('.')[0]
        frame_num = frame_offsets.get(file_name, 0)  # Use default frame 0 if not specified
        cap = cv2.VideoCapture(file_path)
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_num)
        ret, frame = cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            axs[i].imshow(frame)
            axs[i].set_title(f'{file_name}: Frame {frame_num}')
            axs[i].axis('off')
        else:
            print(f'Failed to retrieve frame {frame_num} from {file_path}')
        cap.release()

    # Hide unused axes
    for j in range(i + 1, len(axs)):
        axs[j].axis('off')

    plt.show()


# N controls which frame to visualize
n = 200
frame_offsets = {}

# Extract frame offsets from stats and match it to topic names from the mp4 files
for topic, frame_number in stats['inter_camera_sync']['offsets'].items():
    if 'image_compressed' in topic:
        # Extract the camera name and side from the topic
        parts = topic.split('/')
        camera_name = parts[1]  # Get the camera name part
        camera_side = parts[2]  # Get the side (left/right)
        simplified_name = f'{camera_name}_{camera_side}'  # Create the simplified name

        # Map the simplified name to the frame number
        frame_offsets[simplified_name] = n - int(stats['inter_camera_sync']['offsets'][topic][n])

visualize_specific_frames_of_mp4_files(OUT_DIR, frame_offsets)
