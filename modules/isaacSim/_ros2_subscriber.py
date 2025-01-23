import cv2
import queue
import rclpy
import sys
import random
import logging
import numpy as np
from typing import *
from pathlib import Path
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from vision_msgs.msg import Detection2DArray,  Detection2D, ObjectHypothesisWithPose, Detection3DArray
from std_msgs.msg import String

from ros import initialize_ros, ros_numpy, ros_dict, ros_image

ISAAC_SIM = True

def hex_to_rgb(value):
	value = value.lstrip('#')
	lv = len(value)
	return tuple(int(value[i:i + lv // 3], 16) for i in range(0, lv, lv // 3))

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.create_subscription(
            CompressedImage, "/InferEngine/RosStream/rgb", self._callback_rgb_subscriber, 10)
        self.create_subscription(
            Image, "/InferEngine/RosStream/rgb", self._callback_rgb_subscriber, 10)
        self._rgb_queue = queue.Queue(maxsize=5)

        self.create_subscription(
            String, "/InferEngine/RosStream/class_name", self._callback_msg_results, 10)

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

    # -----------------------------------------------------------------------
    def _callback_semantic_subscriber(self, data: Union[Image, CompressedImage]):
        if self._semantic_queue.full():
            self._semantic_queue.get(timeout=0.001)
        self._semantic_queue.put(data, timeout=0.001)

    def get_semantic_results(self):
        if not self.has_semantic():
            return None
        data = self._semantic_queue.get(timeout=0.05)
        
        # TODO: semantic label = {0: background, 1: unknown, ....} -> labe = {0: unknown, ...}
        _image = ros_image.imgify(data)
        return np.where(_image <= 1, 0, _image-1)
    
    def has_semantic(self):
        return self._semantic_queue.qsize() > 0
    
    def _callback_msg_results(self, data: String):
        if data.data:
            msg = ros_dict.dictify(data)
            self.get_logger().info(f"Received message: {msg}")
        else:
            self.get_logger().warn("Received empty data or invalid message.")

        for subscription in self.subscriptions:
            if subscription.topic_name == "/InferEngine/RosStream/class_name":
                for idx, _label in msg.items():
                    if idx.isalnum():
                        self.class_names.update({int(idx)+1 : _label['class']}) 
                get_colors = list(map(lambda i: hex_to_rgb("#" +"%06x" % random.randint(0, 0xFFFFFF)), range(len(self.class_names)) ))
                self.class_colors = dict(zip(list(self.class_names), get_colors))
                subscription.destroy()
                
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
                
                cv2.imshow("ROS2 RGB Subscriber", cv2.resize(color_image, (640, 360)))
            
            if subscriber.has_depth():
                depth_image = subscriber.get_depth_results()
                
                if ISAAC_SIM: # unit: m -> mm
                    depth_image = np.array(depth_image*100*10, dtype=np.uint16)

                depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                depth_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET) # convert Gray -> BGR
                
                cv2.imshow("ROS2 Depth Subscriber", cv2.resize(depth_image, (640, 360)))

            if subscriber.has_semantic():
                semantic_image = subscriber.get_semantic_results()
                
                if subscriber.class_names:
                    # Create an RGB image initialized to zeros (black background)
                    h, w = semantic_image.shape
                    semantic_rgb_image = np.zeros((h, w, 3), dtype=np.uint8)

                    # Apply colors based on `subscriber.class_colors` (a dictionary mapping ID to color)
                    for class_id, color in subscriber.class_colors.items():
                        semantic_rgb_image[semantic_image == class_id] = color  # Map grayscale ID to RGB color

                    for id, xmin, ymin, xmax, ymax in subscriber.objs_info:
                        cv2.rectangle(semantic_rgb_image, (xmin, ymin), (xmax, ymax), subscriber.class_colors[id], 2)
                        cv2.putText(semantic_rgb_image, subscriber.class_names[id], 
                                    (int((xmin + xmax)//2), int((ymin+ymax)//2)), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255), 2)
                        
                    for id, label in subscriber.class_names.items():
                        text = f"{id}: {label}"
                        DrawTextOnFrame(semantic_rgb_image, text, (10, (id+1)*25), 0.8, subscriber.class_colors[id], 2)
                cv2.imshow("ROS2 Semantic/Detect Label Subscriber", cv2.resize(semantic_rgb_image, (640, 360)))
  
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