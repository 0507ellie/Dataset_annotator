import time
import cv2

import rospy
from ros import ros_numpy, ros_dict
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ultralytics import YOLO

PRED_SOURCE = "./face_video.mp4"
detection_model = YOLO("yolov8n.pt")
segmentation_model = YOLO("yolov8n-seg.pt")
rospy.init_node("ultralytics", anonymous=True)
time.sleep(1)

msg_pub = rospy.Publisher("/ultralytics/msg", String, queue_size=10)
det_image_pub = rospy.Publisher("/ultralytics/detection/image", Image, queue_size=5)
seg_image_pub = rospy.Publisher("/ultralytics/segmentation/image", Image, queue_size=5)
rate = rospy.Rate(30)


try:
    det_result = detection_model(source=PRED_SOURCE, stream=True)
    seg_result = segmentation_model(source=PRED_SOURCE, stream=True)
except Exception as e:
    rospy.logerr(f"Failed to initialize models with video source: {e}")
    exit(1)
    
try:
    while not rospy.is_shutdown():
        try:
            det_annotated = next(det_result)
            boxes = det_annotated.boxes.xywh
            confs = det_annotated.boxes.conf

            json_dict = {}
            for idx, (conf, box) in enumerate(zip(confs, boxes)):
                json_dict.update({idx: {'conf': round(conf.cpu().numpy().item(), 2), 
                                        'box': box.cpu().numpy()}})

            msg_pub.publish(ros_dict.msgify(String, json_dict))

            if det_annotated:
                det_annotated = det_annotated.plot(show=False)
                if det_image_pub.get_num_connections():
                    det_image_pub.publish(ros_numpy.msgify(Image, det_annotated, encoding="rgb8"))
        except StopIteration:
            rospy.logwarn("Detection stream ended.")
            break

        try:
            seg_annotated = next(seg_result)
            if seg_annotated:
                seg_annotated = seg_annotated.plot(show=False)
                if seg_image_pub.get_num_connections():
                    seg_image_pub.publish(ros_numpy.msgify(Image, seg_annotated, encoding="rgb8"))
        except StopIteration:
            rospy.logwarn("Segmentation stream ended.")
            break


        if det_annotated is None and seg_annotated is None:
            break
        rate.sleep()

except KeyboardInterrupt:
    rospy.loginfo("Shutting down node gracefully.")
finally:
    cv2.destroyAllWindows()