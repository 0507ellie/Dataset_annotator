import cv2
import queue
import rclpy
import numpy as np
from typing import *
from pathlib import Path
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from vision_msgs.msg import Detection2DArray,  Detection2D, ObjectHypothesisWithPose, Detection3DArray
from std_msgs.msg import String

from utils import initialize_ros, ros_numpy, ros_dict, ros_image

ISAAC_SIM = True

def hex_to_rgb(value):
	value = value.lstrip('#')
	lv = len(value)
	return tuple(int(value[i:i + lv // 3], 16) for i in range(0, lv, lv // 3))

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.rgb_image_pub = self.create_publisher(
            CompressedImage, "/InferEngine/RosStream/rgb", 10)
        self.create_subscription(
            Image, "/InferEngine/RosStream/isaac/rgb", self._callback_rgb_subscriber, 10)
        self._rgb_queue = queue.Queue(maxsize=5)

    def _callback_rgb_subscriber(self, data: Union[Image, CompressedImage]):
        if self._rgb_queue.full():
            self._rgb_queue.get(timeout=0.001)
        self._rgb_queue.put(data, timeout=0.001)

    def get_rgb_results(self):
        if not self.has_rgb():
            return None
        data = self._rgb_queue.get(timeout=0.05)

        return ros_image.imgify(data, "bgr8" if ISAAC_SIM else None)
    
    def has_rgb(self):
        return self._rgb_queue.qsize() > 0

def main(args=None):
    rclpy.init(args=args)
    subscriber = ImageSubscriber()
    try:
        color_image = None
        depth_image = None
        semantic_image = None
        while rclpy.ok():
            rclpy.spin_once(subscriber, timeout_sec=0.1)
            
            if subscriber.has_rgb():
                color_image = subscriber.get_rgb_results()
                image_msg = ros_image.msgify(subscriber.rgb_image_pub.msg_type, color_image, quality=60)
                subscriber.rgb_image_pub.publish(image_msg)

            if cv2.waitKey(1) & 0xFF == ord('q'):  # 按 'q' 退出
                break

        cv2.destroyAllWindows()

    except KeyboardInterrupt:
        subscriber.get_logger().info("Shutting down node gracefully.")
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
