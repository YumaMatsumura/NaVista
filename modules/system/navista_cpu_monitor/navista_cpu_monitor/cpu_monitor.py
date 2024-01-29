#!/usr/bin/env python3

import os
import subprocess

from navista_system_msgs.msg import CpuStatus
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


class CPUMonitor(Node):
    def __init__(self):
        super().__init__('cpu_monitor_node')
        timer_period = 0.25
        self.pub = self.create_publisher(CpuStatus, 'cpu_status', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # CPU使用率を取得
        cpu_usage = []
        for i in range(2, 10, 2):
            command = 'top -b -n1 | grep "Cpu(s)" | awk \'{print $' + str(i) + '}\''
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
            cpu_usage.append(float(data.decode()))

        msg = CpuStatus()
        msg.total_usage = cpu_usage[0] + cpu_usage[1]
        msg.usr_usage = cpu_usage[0]
        msg.sys_usage = cpu_usage[1]
        msg.nice_usage = cpu_usage[2]
        msg.idle_usage = cpu_usage[3]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        cpu_monitor = CPUMonitor()
        executor = SingleThreadedExecutor()
        executor.add_node(cpu_monitor)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            cpu_monitor.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
