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

import threading
import rclpy
from rclpy.node import Node
import http.server
import socketserver
import ssl
import time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from functools import partial
import os
import socket


class WebServerNode(Node):

    def __init__(self):
        super().__init__('web_server')

        self.declare_parameter('webserver_port', 8080)
        self.declare_parameter('retry_count', 10)
        self.declare_parameter('retry_delay', 2.0)

        self.port = self.get_parameter('webserver_port').get_parameter_value().integer_value
        self.retry_count = self.get_parameter('retry_count').get_parameter_value().integer_value
        self.retry_delay = self.get_parameter('retry_delay').get_parameter_value().double_value

        # Validate the retry_count
        if self.retry_count < 0:
            self.get_logger().warn(
                f"Invalid retry_count: {self.retry_count}. Setting retry_count to 10.")
            self.retry_count = 10

        # Validate the retry_delay
        if self.retry_delay < 0:
            self.get_logger().warn(
                f"Invalid retry_delay: {self.retry_delay}. Setting retry_delay to 2.0.")
            self.retry_delay = 2.0

        # Dynamically find the package share directory
        package_share_dir = Path(get_package_share_directory('isaac_ros_nova_recorder'))
        certs_dir = os.path.join(package_share_dir, 'certs')
        self.cert_file = os.path.join(certs_dir, 'cert.pem')
        self.key_file = os.path.join(certs_dir, 'key.pem')
        self.directory = package_share_dir / 'foxsight'

        # Check if the directory exists
        if not self.directory.exists():
            self.get_logger().error(f"The directory {self.directory} does not exist.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Serving files from: {self.directory}")

        self.server = None

        # Create a custom server class that sets allow_reuse_address
        class ThreadingHTTPServer(socketserver.ThreadingTCPServer):
            allow_reuse_address = True
            daemon_threads = True

        self.server_class = ThreadingHTTPServer

        # Register the shutdown callback
        rclpy.get_default_context().on_shutdown(self.shutdown_server)

    def start_server(self):
        handler_class = partial(
            http.server.SimpleHTTPRequestHandler, directory=str(self.directory))

        attempt = 0
        while attempt < self.retry_count:
            try:
                self.server = self.server_class(("", self.port), handler_class)
                # Create SSL context
                ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
                ssl_context.load_cert_chain(certfile=self.cert_file, keyfile=self.key_file)

                # Wrap the server's socket with SSL
                self.server.socket = ssl_context.wrap_socket(self.server.socket, server_side=True)

                # Get server IP address
                hostname = socket.gethostname()
                ip_address = socket.gethostbyname(hostname)
                self.get_logger().info(f"Serving at https://{ip_address}:{self.port}")

                # Use a shorter poll_interval to allow for quicker shutdown
                self.server.serve_forever(poll_interval=0.5)
                break
            except OSError as e:
                attempt += 1
                self.get_logger().error(f"Failed to bind to port {self.port}: {e}")
                if attempt < self.retry_count:
                    self.get_logger().info(
                        f"Retrying in {self.retry_delay} seconds... "
                        f"(Attempt {attempt}/{self.retry_count})"
                    )
                    time.sleep(self.retry_delay)
                else:
                    self.get_logger().error(
                        "Maximum retry attempts reached, shutting down the node."
                    )
                    break
            except Exception as e:
                attempt += 1
                self.get_logger().error(f"An error occurred: {e}")
                if attempt < self.retry_count:
                    self.get_logger().info(
                        f"Retrying in {self.retry_delay} seconds... "
                        f"(Attempt {attempt}/{self.retry_count})"
                    )
                    time.sleep(self.retry_delay)
                else:
                    self.get_logger().error(
                        "Maximum retry attempts reached, shutting down the node."
                    )
                    break

    def shutdown_server(self):
        if self.server:
            self.get_logger().info("Shutting down server")
            self.server.shutdown()
            self.server.server_close()

    # Move server thread initialization inside WebserverNode class for better encapsulation
    def run(self):
        self.server_thread = threading.Thread(target=self.start_server)
        self.server_thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()

    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        node.server_thread.join()
        node.get_logger().info("Server thread has finished")


if __name__ == '__main__':
    main()
