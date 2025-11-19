import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import time
import os
import ctypes
import sys
import threading
from pathlib import Path

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import UInt64MultiArray

NODE_PRIORITY = 60
WARMING_UP = 10
EXPERIMENT_COUNT = 100

VELODYNE_TOPIC = 'velodyne_points'
TIME_TOPIC = 'time_packets'
RECEIVE_LOG_FILE = "receive_log.csv"
POINTCLOUD_LOG_FILE = "POINTCLOUD_log.csv"

class LiDARReceiveNode(Node):
    def __init__(self):
        super().__init__('lidar_node')

        self.running_count = 0
        self.receive_log = []
        self.pointcloud_log = []

        # basic profile (velodyne_driver)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            VELODYNE_TOPIC,
            self.pointcloud_callback,
            qos_profile
        )

        self.time_subscription = self.create_subscription(
            UInt64MultiArray,
            TIME_TOPIC,
            self.receive_time,
            qos_profile
        )

    def receive_time(self, msg):
        self.receive_log.push((msg.data[0], msg.data[1]))

    def pointcloud_callback(self, msg):
        self.running_count += 1

        if self.running_count <= WARMING_UP:
            return

        if self.running_count > EXPERIMENT_COUNT:
            rclpy.shutdown()
            with open(RECEIVE_LOG_FILE, 'w') as f:
                f.write("frame_index,receive_ns\n")
                self.receive_log.sort(key=lambda x: x[0])
                f.write("\n".join(f"{frame_index},{time}" for frame_index, time in self.receive_log))

            with open(POINTCLOUD_LOG_FILE, 'w') as f:
                f.write("frame_index,pointcloud_ns\n")
                self.pointcloud_log.sort(key=lambda x: x[0])
                f.write("\n".join(f"{frame_index},{time}" for frame_index, time in self.pointcloud_log))
            return

        self.pointcloud_log.push((msg.header.frame_id, time.monotonic_ns()))

def main(args=None):
    param = os.sched_param(NODE_PRIORITY)
    os.sched_setscheduler(0, os.SCHED_FIFO, param)

    lib_c = ctypes.CDLL("libc.so.6")
    lib_c.mlockall.argtypes = [ctypes.c_int]
    lib_c.mlockall.restype = ctypes.c_int

    result = lib_c.mlockall(1 | 2) # MCL_FUTURE and MCL_CURRENT both

    if result != 0:
        print("mlockall failed")
        return

    rclpy.init(args=args)
    
    lidar_node = LiDARReceiveNode()
    
    rclpy.spin(lidar_node)

if __name__ == '__main__':
    main()