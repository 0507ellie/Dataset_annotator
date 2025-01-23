import time
import cv2
import sys
import rclpy
from rclpy.node import Node
from typing import *
from pathlib import Path
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

from ultralytics import YOLO
from ros import initialize_ros, ros_numpy, ros_dict, ros_image

PRED_SOURCE = "./face_video.mp4"
detection_model = YOLO("yolov8n.pt")
segmentation_model = YOLO("yolov8n-seg.pt")


try:
    det_result = detection_model(source=PRED_SOURCE, stream=True)
    seg_result = segmentation_model(source=PRED_SOURCE, stream=True)
except Exception as e:
    exit(1)
    
class InferEngineNode(Node):
    def __init__(self):
        super().__init__('infer_engine')
        self.msg_pub = self.create_publisher(String, '/ultralytics/msg', 10)
        self.det_image_pub = self.create_publisher(CompressedImage, "/ultralytics/detection/image", 10)
        self.seg_image_pub = self.create_publisher(CompressedImage, "/ultralytics/segmentation/image", 10)
        self.rate = self.create_rate(30)

    def run(self):
        try:
            while rclpy.ok():
                try:
                    det_annotated = next(det_result)
                    
                    if self.msg_pub.get_subscription_count() > 0:
                        boxes = det_annotated.boxes.xywh
                        confs = det_annotated.boxes.conf

                        json_dict = {}
                        for idx, (conf, box) in enumerate(zip(confs, boxes)):
                            json_dict.update({idx: {'conf': round(conf.cpu().numpy().item(), 2), 
                                                    'box': box.cpu().numpy()}})
                        self.msg_pub.publish(ros_dict.msgify(String, json_dict))
                        
                    if self.det_image_pub.get_subscription_count() > 0:
                        det_annotated = det_annotated.plot(show=False)
                        image_msg = ros_image.msgify(self.det_image_pub.msg_type, det_annotated, quality=60)
                        self.det_image_pub.publish(image_msg)


                    key = cv2.waitKey(1)
                    if key & 0xFF == 27:  # ESC key
                        break
                except StopIteration:
                    self.get_logger().warn("Detection stream ended.")
                    break
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down node gracefully.")



if __name__ == '__main__':
    initialize_ros()
    node = InferEngineNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()