import cv2
import json
import rospy
import queue

from ros import ros_numpy, ros_dict
from sensor_msgs.msg import Image
from std_msgs.msg import String

class ImageSubscriber:
    def __init__(self):
        rospy.init_node(self.__class__.__name__, anonymous=True)

        rospy.Subscriber(
            "/ultralytics/detection/image", Image, self._callback_det_subscriber)
        self._det_queue = queue.Queue(maxsize=5)
        
        rospy.Subscriber(
            "/ultralytics/segmentation/image", Image, self._callback_seg_subscriber)
        self._seg_queue = queue.Queue(maxsize=5)
        rospy.Subscriber(
            "/ultralytics/msg", String, self._callback_msg_results)
        self._msg_queue = queue.Queue(maxsize=5)
        
    def _callback_msg_results(self, data: String):
        if data:
            msg = ros_dict.dictify(data)
            print(msg)
        else:
            rospy.logwarn("Received empty data or invalid message.")

    # 
    def _callback_det_subscriber(self, data: Image):
        if self._det_queue.full():  # If queue is full, pop one.
            self._det_queue.get(timeout=0.001)
        self._det_queue.put(data, timeout=0.001)  # Push image to queue

    def get_det_results(self):
        if not self.has_det():
            return None
        data = self._det_queue.get(timeout=0.05)
        image = ros_numpy.numpify(data)
        return image

    def has_det(self):
        return self._det_queue.qsize() > 0

    # 
    def _callback_seg_subscriber(self, data: Image):
        if self._seg_queue.full():  # If queue is full, pop one.
            self._seg_queue.get(timeout=0.001)
        self._seg_queue.put(data, timeout=0.001)  # Push image to queue

    def get_seg_results(self):
        if not self.has_seg():
            return None
        data = self._seg_queue.get(timeout=0.05)
        image = ros_numpy.numpify(data)
        return image

    def has_seg(self):
        return self._seg_queue.qsize() > 0


if __name__ == '__main__':
    try:
        
        subscriber = ImageSubscriber()
        while not rospy.is_shutdown():
            if subscriber.has_det() and subscriber.has_seg():
                det_image = subscriber.get_det_results()
                seg_image = subscriber.get_seg_results()
                combined_image = cv2.hconcat([det_image, seg_image])
                cv2.imshow("ROS sub", cv2.resize(combined_image, (1280, 480)))
            elif subscriber.has_det():
                det_image = subscriber.get_det_results()
                cv2.imshow("ROS sub", cv2.resize(det_image, (720, 480)))
            elif subscriber.has_seg():
                seg_image = subscriber.get_seg_results()
                cv2.imshow("ROS sub", cv2.resize(seg_image, (720, 480)))
            else:
                continue
            
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
                break

        cv2.destroyAllWindows()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down node gracefully.")
