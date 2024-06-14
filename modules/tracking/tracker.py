import cv2
import codecs, os 
import numpy as np
from typing import *
from concurrent.futures import ThreadPoolExecutor

from .libs.painterDialog import PainterDialog

class TrackerDetector(object):
    def __init__(self, debug, model: str = 'csrt', cpu_workers: int = 1):
        self.debug = debug
        self.cv2_version = ''.join(cv2.__version__.split('.'))
        self.cpu_workers = cpu_workers
        
        self.track_model = model
        self.track_init = False
        self.label_hist = None
        self.bbox_list = []
        
    def _init_model(self, model: str) -> None:
        if int(self.cv2_version) >= 454:
            OPENCV_OBJECT_TRACKERS = {
                "csrt": cv2.legacy.TrackerCSRT_create,
                "kcf": cv2.legacy.TrackerKCF_create,
                "boosting": cv2.legacy.TrackerBoosting_create,
                "mil": cv2.legacy.TrackerMIL_create,
                "tld": cv2.legacy.TrackerTLD_create,
                "medianflow": cv2.legacy.TrackerMedianFlow_create,
                "mosse": cv2.legacy.TrackerMOSSE_create,
            }
            if self.cpu_workers == 1:
                self.multiTracker = cv2.legacy.MultiTracker_create()
            else :
                self.multiTracker = []
        else :
            OPENCV_OBJECT_TRACKERS = {
                "csrt": cv2.TrackerCSRT_create,
                "kcf": cv2.TrackerKCF_create,
                "boosting": cv2.TrackerBoosting_create,
                "mil": cv2.TrackerMIL_create,
                "tld": cv2.TrackerTLD_create,
                "medianflow": cv2.TrackerMedianFlow_create,
                "mosse": cv2.TrackerMOSSE_create
            }
            if self.cpu_workers == 1:
                self.multiTracker = cv2.MultiTracker_create()
            else :
                self.multiTracker = []
        self.tracker = OPENCV_OBJECT_TRACKERS[model]
      
    @staticmethod
    def _iou(box1: Union[tuple, list], box2: Union[tuple, list]) -> float:
        """
        Calculate the Intersection over Union (IOU) between two bounding boxes.
        
        Args:
            box1: Tuple (x1, y1, w1, h1) representing the coordinates and size of the first bounding box.
            box2: Tuple (x2, y2, w2, h2) representing the coordinates and size of the second bounding box.
        
        Returns:
            float: The IOU value between the two bounding boxes.
        """
        
        # Calculate the upper left and lower right coordinates of the intersection
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[0] + box1[2], box2[0] + box2[2])
        y2 = min(box1[1] + box1[3], box2[1] + box2[3])
        
        # Calculate intersection area
        intersection_area = max(0, x2 - x1 + 1) * max(0, y2 - y1 + 1)
        
        # Calculate the area of two boxes
        box1_area = (box1[2] + 1) * (box1[3] + 1)
        box2_area = (box2[2] + 1) * (box2[3] + 1)
        
        return intersection_area / float(box1_area + box2_area - intersection_area)

    def loadClasses(self, predef_classes_file: str) -> None:
        """
        Load predefined classes from a file.

        Args:
            predef_classes_file (str): Path to the file containing predefined classes.

        Returns:
            None
        """
        if os.path.exists(predef_classes_file) is True:
            with codecs.open(predef_classes_file, 'r', 'utf8') as f:
                for line in f:
                    line = line.strip()
                    if self.label_hist is None:
                        self.label_hist = [line]
                    else:
                        self.label_hist.append(line)
            self.classes_file = predef_classes_file
        else :
            self.classes_file = None

    def initBox(self, src_frame: np.ndarray) -> None:
        """
        Initialize tracking with a selected bounding box from the first frame.

        Args:
            src_frame (np.ndarray): The source frame as a NumPy array.

        Returns:
            None
        """
        if (not self.track_init) :
            self.track_init = True
            window = PainterDialog(src_frame.copy(), self.classes_file, debug=self.debug)
            window.set_qdarkstyle()
            window.load_bbox_by_list(self.bbox_list)
            window.exec()
            self.bbox_list = window.get_labels()

            if self.bbox_list:
                self._init_model(self.track_model)
                if self.cpu_workers == 1:
                    [self.multiTracker.add(self.tracker(), src_frame, tuple(box)) for label, box in self.bbox_list]
                else:
                    for label, box in self.bbox_list:
                        _tracker = self.tracker()
                        _tracker.init(src_frame, tuple(box))
                        self.multiTracker.append(_tracker)
        self.track_init = False

    def update(self, src_frame: np.ndarray) -> list:
        """
        Update the tracker with the current frame and return the updated bounding box list.

        Args:
            src_frame (np.ndarray): The current frame as a NumPy array.

        Returns:
            list: The updated bounding box list.
        """

        if self.bbox_list:
            if self.cpu_workers == 1:
                ret, boxes = self.multiTracker.update(src_frame)
                for idx, (label, box) in enumerate(self.bbox_list):
                    box[:] = boxes[idx]
            else:
                with ThreadPoolExecutor(max_workers= min(len(self.bbox_list), self.cpu_workers)) as executor:
                    futures = [executor.submit(tracker.update, src_frame) for tracker in self.multiTracker]
                    results = [future.result() for future in futures]
                for (label, box), (ret, _box) in zip(self.bbox_list, results):
                    xmin = int(max(0, _box[0]))
                    ymin = int(max(0, _box[1]))
                    xmax = int(min(_box[0] + _box[2], src_frame.shape[1]-1))
                    ymax = int(min(_box[1] + _box[3], src_frame.shape[0]-1))
                    box[:] = (xmin, ymin, xmax-xmin, ymax-ymin)
        return self.bbox_list