#!/usr/bin/env python3

# Copyright 2024 Yuma Matsumura All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import subprocess

from navista_system_msgs.msg import MemoryStatus
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


class MemoryMonitor(Node):
    def __init__(self):
        super().__init__('memory_monitor_node')
        timer_period = 0.25
        self.pub = self.create_publisher(MemoryStatus, 'memory_status', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # メモリー量を取得
        memory_usage = []
        for i in range(4, 12, 2):
            command = 'top -b -n1 | grep "MiB Mem" | awk \'{print $' + str(i) + '}\''
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                shell=True,
            )
            data = process.communicate()[0]
            if process.returncode != 0:
                self.get_logger().error(f'command: {command} failed: {process.returncode}')
                return
            memory_usage.append(float(data.decode()))

        msg = MemoryStatus()
        msg.total_usage = memory_usage[0]
        msg.free_usage = memory_usage[1]
        msg.used_usage = memory_usage[2]
        msg.buff_and_cache_usage = memory_usage[3]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        memory_monitor = MemoryMonitor()
        executor = SingleThreadedExecutor()
        executor.add_node(memory_monitor)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            memory_monitor.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
