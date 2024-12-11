#!/usr/bin/env python3

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

from isaac_ros_data_recorder.srv import Event
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class EventTriggerNode(Node):

    def __init__(self):
        super().__init__('event_trigger')

        self.create_subscription(Joy, 'joy', self.callback, 10)

        self.event_start = self.create_client(Event, 'event_start')
        self.event_end = self.create_client(Event, 'event_end')

        self.event_start.wait_for_service()
        self.event_end.wait_for_service()

        self.create_timer(0.1, self.tick)

        self.start_button = False
        self.end_button = False
        self.future = None

    def tick(self):
        if self.future and self.future.done():
            response = self.future.result()
            if not response.success:
                self.get_logger().warning('Service call was not successful')
            self.start_button = False
            self.end_button = False
            self.future = None
        elif self.start_button:
            self.get_logger().info('Starting event')
            self.future = self.event_start.call_async(Event.Request())
        elif self.end_button:
            self.get_logger().info('Ending event')
            self.future = self.event_end.call_async(Event.Request())

    def callback(self, msg):
        if not self.start_button and not self.end_button:
            if msg.buttons[1]:
                self.start_button = True
            elif msg.buttons[0]:
                self.end_button = True


def main():
    rclpy.init()
    rclpy.spin(EventTriggerNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
