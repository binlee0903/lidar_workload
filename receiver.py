import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import time
import os
import ctypes
import sys
import threading
from pathlib import Path

PUBLISH_HZ = 10
NODE_PRIORITY = 60
WARMING_UP = 100

IMAGE_TOPIC = 'kitti_image'
RESULT_TOPIC = 'frame_index'
SEND_LOG_FILE = "publish_log.csv"
RECEIVE_LOG_FILE = "receive_log.csv"


class SLAMSendNode(Node):
    def __init__(self):
        super().__init__('slam_send_node')
        
        self.bridge = CvBridge()
        self.rate = self.create_rate(PUBLISH_HZ)
        
        self.frame_index = 0

        self.publish_log = []
        self.receive_log = []

        image_dir = Path(DATASET_PATH)
        image_paths = sorted(list(image_dir.glob('*.png')))

        self.kitti_images = []

        for i in image_paths:
            kitti_images.push(cv2.imread(str(i)))

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.publisher = self.create_publisher(Image, IMAGE_TOPIC, qos_profile)
        self.subscription = self.create_subscription(
            String,
            RESULT_TOPIC,
            self.result_callback,
            qos_profile
        )

    self.publish_thread = threading.Thread(target=self.publish)
    self.publish_thread.start()

    def publish(self):
        while rclpy.ok():
            frame_start_time_ns = time.time_ns()

            msg = self.bridge.cv2_to_imgmsg(kitti_images[self.frame_index], encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_index = str(self.frame_index)

            start = time.time_ns()

            if self.frame_index >= WARMING_UP:
                self.publish_log.append((self.frame_index, start))
            
            self.publisher.publish(msg)
            
            self.frame_index += 1
            if self.frame_index >= len(self.image_paths):
                print("published all")
                break

            self.rate.sleep()

    def result_callback(self, msg: String):
        end = time.time_ns()
        frame_index = int(msg.data)

        if frame_index >= WARMING_UP:
            self.receive_log.append((frame_index, end))

        if self.frame_index >= len(self.image_paths):
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

                with open(SEND_LOG_FILE, 'w') as f:
                    f.write("frame_index,publish_ns\n")
                    f.write("\n".join(f"{frame_index},{start}" for frame_index, start in self.publish_log))

                with open(RECEIVE_LOG_FILE, 'w') as f:
                    f.write("frame_index,receive_ns\n")
                    self.t2_receive_log.sort(key=lambda x: x[0])
                    f.write("\n".join(f"{frame_index},{end}" for frame_index, end in self.receive_log))

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
    
    slam_node = SLAMSendNode()
    
    rclpy.spin(slam_node)

if __name__ == '__main__':
    main()