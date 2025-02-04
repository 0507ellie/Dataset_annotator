from turtle import shapesize
import cv2
import queue

import rclpy
import codecs, os
import random
import numpy as np
from typing import *
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray,  Detection2D, ObjectHypothesisWithPose, Detection3DArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import String
from pathlib import Path

from modules.labeling.libs.create_ml_io import JSON_EXT
from modules.labeling.libs.pascal_voc_io import XML_EXT
from modules.labeling.libs.yolo_io import TXT_EXT
from modules.labeling.libs.labelFile import LabelFileFormat, LabelFile
from modules.labeling.libs.utils import generate_color_by_text
from modules.labeling.libs.shape import Shape
from modules.isaacSim.ros import initialize_ros, ros_numpy, ros_dict, ros_image

ISAAC_SIM = True

IMAGE_TAG = "images"
LABEL_TAG = "labels"

RGB_TOPIC = "/InferEngine/RosStream/rgb"
DETECT_TOPIC = "/InferEngine/RosStream/detect"
SEMANTIC_TOPIC = "/InferEngine/RosStream/segment"

class InfoSubscriber(Node):
    def __init__(self, default_prefdef_class_file=None):
        super().__init__('image_subscriber')
        self.label_hist = self.load_predefined_classes(default_prefdef_class_file)
        self.class_colors = {idx: generate_color_by_text(label).getRgb()[:3] for idx, label in enumerate(self.label_hist)} #dict(zip(range(len(self.label_hist)), get_colors))
        self.class_colors.update({len(self.label_hist) : (0, 0, 0)})
        
        self.label_file_format = LabelFileFormat.YOLO
        self.images_dir_path = None
        self.labels_dir_path = None
        
        base_dir = Path("temp")
        self.images_dir_path = base_dir.joinpath(IMAGE_TAG) 
        self.images_dir_path.mkdir(parents=True, exist_ok=True)
        self.labels_dir_path = base_dir.joinpath(LABEL_TAG)
        self.labels_dir_path.mkdir(parents=True, exist_ok=True)
        
        self._semantic_index_mapping = {}
        self._detect_index_mapping = {}
        self.__temp_semantic_label = {}
        self.__temp_detect_label = {}
        
        # ----------------------------- rgb image ------------------------------
        self.create_subscription(
            CompressedImage, RGB_TOPIC, self._callback_rgb_subscriber, 10)
        self.create_subscription(
            Image, RGB_TOPIC, self._callback_rgb_subscriber, 10)
        self._rgb_queue = queue.Queue(maxsize=5)

        # Labeling Part
        # ----------------------------- semantic image ------------------------------
        self.create_subscription(
            CompressedImage, SEMANTIC_TOPIC, self._callback_semantic_subscriber, 10)
        self.create_subscription(
            Image, SEMANTIC_TOPIC, self._callback_semantic_subscriber, 10)
        self._semantic_queue = queue.Queue(maxsize=5)
        
        self.create_subscription(
            String, SEMANTIC_TOPIC + "/class_name", self._callback_semantic_labels, 10)

        # ----------------------------- 2d bbox ------------------------------
        self.create_subscription(
            Detection2DArray, DETECT_TOPIC, self._callback_detect_subscriber, 10)
        
        self.create_subscription(
            String, DETECT_TOPIC + "/class_name", self._callback_detect_labels, 10)
        self._bbox_queue = queue.Queue(maxsize=5)
        
    def load_predefined_classes(self, predef_classes_file):
        label_hist = None
        if os.path.exists(predef_classes_file) is True:
            with codecs.open(predef_classes_file, 'r', 'utf8') as f:
                for line in f:
                    line = line.strip()
                    if label_hist is None:
                        label_hist = [line]
                    else:
                        label_hist.append(line)
            self.classes_file = predef_classes_file
        else :
            self.classes_file = None
        return label_hist

    def save_file(self, frame_id: int, image: np.ndarray, objs_info: list):
        image_file_name = str(frame_id).rjust(5, '0') + '.jpg'
        label_file_name = str(frame_id).rjust(5, '0')
        cv2.imwrite(str(self.images_dir_path.joinpath(image_file_name)), image)

        def format_shape(s):
            return dict(label=s.label,
                        line_color=s.line_color,
                        fill_color=s.fill_color,
                        points=[(p[0], p[1]) for p in s.points],
                        type=s.shape_type,
                        # add chris
                        difficult=s.difficult)
        shapes = [format_shape(shape) for shape in objs_info]

        if shapes:
            label_file = LabelFile()
            if self.label_file_format == LabelFileFormat.YOLO:
                label_file_name += TXT_EXT
                label_file.save_yolo_format(str(self.labels_dir_path.joinpath(label_file_name)),
                                            shapes, 
                                            str(self.images_dir_path.joinpath(image_file_name)),
                                            image,
                                            self.label_hist)
            elif self.label_file_format == LabelFileFormat.PASCAL_VOC:
                label_file_name += XML_EXT
                label_file.save_pascal_voc_format(str(self.labels_dir_path.joinpath(label_file_name)),
                                                    shapes, 
                                                    str(self.images_dir_path.joinpath(image_file_name)), 
                                                    image)
            elif self.label_file_format == LabelFileFormat.CREATE_ML:
                label_file_name += JSON_EXT
                label_file.save_create_ml_format(str(self.labels_dir_path.joinpath(label_file_name)),
                                                    shapes, 
                                                    str(self.images_dir_path.joinpath(image_file_name)),
                                                    image,
                                                    self.label_hist)

    # -----------------------------------------------------------------------
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
        
        # 将 ROS 的影像转换成 NumPy 图像
        _image = ros_image.imgify(data)
        # semantic label = {0: background, 1: unknown, ....} -> labe = {...}
        remapped_image = np.vectorize(self._semantic_index_mapping.get)(_image, -1)

        # 提取多边形轮廓
        unique_classes = np.unique(remapped_image)
        
        shapes = []
        for class_id in unique_classes:
            if class_id < 0:
                continue  # 忽略无效值 (-1)

            # 创建二值掩膜图像
            mask = (remapped_image == class_id).astype(np.uint8)
            # 找到边界轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 提取多边形顶点
            for cnt in contours:
                points = cv2.approxPolyDP(cnt, epsilon=2, closed=True).reshape(-1, 2).tolist()
                if len(points) > 2:
                    label = self.label_hist[class_id]
                    shape = Shape(label=label, shape_type="polygon")
                    shape.points = points
                    shape.line_color = self.class_colors[class_id]
                    shapes.append(shape)
        return remapped_image, shapes
    
    def has_semantic(self):
        return self._semantic_queue.qsize() > 0

    def _callback_semantic_labels(self, data: String):
        if data.data:
            msg = ros_dict.dictify(data)
            timestamp = msg.pop('time_stamp')
            if self.__temp_semantic_label != msg:
                self.__temp_semantic_label = msg
            else:
                return
            self.get_logger().info(f"Received semantic label message [{timestamp}]: {msg}")
        else:
            self.get_logger().warn("Received empty data or invalid message.")

        for subscription in self.subscriptions:
            if subscription.topic_name == SEMANTIC_TOPIC + "/class_name":
                for idx, _label in msg.items():
                    if idx.isalnum():
                        class_name = _label['class']
                        if class_name in self.label_hist:
                            self._semantic_index_mapping[int(idx)] = self.label_hist.index(class_name)
                        else:
                            self._semantic_index_mapping[int(idx)] = -1  # Unknown label mapping is -1
                # subscription.destroy()
                
                
    # -----------------------------------------------------------------------
    def _callback_detect_subscriber(self, data: Detection2DArray):
        if self._bbox_queue.full():
            self._bbox_queue.get(timeout=0.001)
        self._bbox_queue.put(data.detections, timeout=0.001)
        
    def get_bbox_results(self):
        if not self.has_bbox():
            return None
        data = self._bbox_queue.get(timeout=0.05)
        shapes = []
        for info in data:
            info: Detection2D
            for _info in info.results:
                _info: ObjectHypothesisWithPose
                id = self._detect_index_mapping.get(int(_info.id), None)
            
            if id is not None:
                label = self.label_hist[id]
                sx = int(info.bbox.center.x - (info.bbox.size_x/2))
                sy = int(info.bbox.center.y - (info.bbox.size_y/2))
                ex = int(info.bbox.center.x + (info.bbox.size_x/2))
                ey = int(info.bbox.center.y + (info.bbox.size_y/2))
                shape = Shape(label=label, shape_type="rectangle")
                shape.add_point((sx, sy))
                shape.add_point((ex, sy))
                shape.add_point((ex, ey))
                shape.add_point((sx, ey))
                shape.line_color = self.class_colors[id]
                shapes.append(shape)
        return shapes
    
    def has_bbox(self):
        return self._bbox_queue.qsize() > 0
              
    def _callback_detect_labels(self, data: String):
        if data.data:
            msg = ros_dict.dictify(data)
            timestamp = msg.pop('time_stamp')
            if self.__temp_detect_label != msg:
                self.__temp_detect_label = msg
            else:
                return
            self.get_logger().info(f"Received detect label message [{timestamp}]: {msg}")
        else:
            self.get_logger().warn("Received empty data or invalid message.")

        for subscription in self.subscriptions:
            if subscription.topic_name == DETECT_TOPIC + "/class_name":
                for idx, _label in msg.items():
                    if idx.isalnum():
                        class_name = _label['class']
                        if class_name in self.label_hist:
                            self._detect_index_mapping[int(idx)] = self.label_hist.index(class_name)
                        # else:
                        #     self._detect_index_mapping[int(idx)] = -1  # Unknown label mapping is -1
                # subscription.destroy()
                            
def main(args=None):
    initialize_ros()
    subscriber = InfoSubscriber("default_classes.txt")
    try:
        color_image = None
        semantic_image = None
        shapes = []
        # temp testing save label
        i = 0
        while rclpy.ok():
            shapes.clear()
            rclpy.spin_once(subscriber, timeout_sec=0.1)
            
            if subscriber.has_rgb():
                color_image = subscriber.get_rgb_results()
                display_image = color_image.copy()
            
            if subscriber.has_bbox():
                b_shapes = subscriber.get_bbox_results()

                for shape in b_shapes:
                    _shape: Shape = shape
                    cv2.rectangle(display_image, _shape.points[0], _shape.points[2], _shape.line_color, int(_shape.line_width))
                    cv2.putText(display_image, _shape.label, 
                                _shape.points[0], cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255), 2)
                shapes.extend(b_shapes)
            
            if subscriber.has_semantic():
                semantic_image, s_shapes = subscriber.get_semantic_results()

                if subscriber.label_hist:
                    # Create an RGB image initialized to zeros (black background)
                    h, w = semantic_image.shape
                    semantic_rgb_image = np.zeros((h, w, 3), dtype=np.uint8)

                    # Apply colors based on `subscriber.class_colors` (a dictionary mapping ID to color)

                    for class_id, color in subscriber.class_colors.items():
                        semantic_rgb_image[semantic_image == class_id] = color  # Map grayscale ID to RGB color

                    for id, label in enumerate(subscriber.label_hist):
                        text = f"{id}: {label}"
                        cv2.putText(semantic_rgb_image, text, (10, (id+1)*25), cv2.FONT_HERSHEY_TRIPLEX, 0.8, subscriber.class_colors[id], 2)
                cv2.imshow("ROS2 Semantic/Detect Label Subscriber", semantic_rgb_image)
                shapes.extend(s_shapes)
                
            if color_image is not None and shapes:
                subscriber.save_file(i, color_image, shapes)
                cv2.imshow("ROS2 RGB Subscriber", display_image)
            i += 1
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