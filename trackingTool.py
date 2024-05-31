#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Purpose: Easy video labeling using tracking
'''
import cv2 
import os 
import time
import logging
import argparse
import codecs, sys
import numpy as np
import multiprocessing as mp
from typing import *
from pathlib import Path
from PyQt5 import Qt
from PyQt5 import QtCore, QtGui, QtWidgets
from concurrent.futures import ThreadPoolExecutor

from modules.logger import Logger
from modules.resources.resources  import *
from modules.labeling.libs.create_ml_io import JSON_EXT
from modules.labeling.libs.labelFile import LabelFileFormat, LabelFile
from modules.labeling.libs.pascal_voc_io import XML_EXT
from modules.labeling.libs.utils import generate_color_by_text
from modules.labeling.libs.yolo_io import TXT_EXT
from modules.tracking.libs.fileDialog import FileDialog
from modules.tracking.libs.painterDialog import PainterDialog
from modules.tracking.motion import MotionDetector

debug = Logger(None, logging.INFO, logging.INFO )

argparser = argparse.ArgumentParser(description='Multitracker for labeling in the video')
argparser.add_argument('-i','--video_dir',
                        help='Path to the input video directory.')
argparser.add_argument('-c','--class_file', default= os.path.join(os.path.dirname(__file__), 'default_classes.txt'),
                        help='Path to the file containing class names. Default is "default_classes.txt".')
argparser.add_argument('-o','--save_folder', default="temp", nargs="?",
                        help='Folder to save the labeled frames. Default is "temp".')

IMAGE_TAG = "images"
LABEL_TAG = "labels"
# #############################################

class ObjectTack(object):
    def __init__(self, model: str = 'csrt', cpu_workers: int = 1):
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
            window = PainterDialog(src_frame.copy(), self.classes_file, debug=debug)
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

if __name__ == '__main__':
    args = argparser.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    
    DISPLAT_RATE = 0.66
    desktop = QtWidgets.QApplication.desktop()
    display_size = (int(desktop.width() * DISPLAT_RATE), 
                    int(desktop.height() * DISPLAT_RATE))
    
    if args.video_dir:
        interval_frame = 10
        ROOT_DIRS = args.video_dir
        if not Path(ROOT_DIRS).is_dir(): 
            debug.error("Must be a folder path.")
            exit()
        OUTPUT_DIR = Path(ROOT_DIRS).joinpath(args.save_folder)
        video_paths = []
        for item in Path(ROOT_DIRS).iterdir():
            if item.is_file():
                video_paths.append(item)   
    else:
        dlg = FileDialog("File Dialog", args.class_file)
        dlg.show()  # Show the dialog
        app.exec_()
        video_paths = dlg.getVideoList()
        interval_frame = dlg.getInterval()
        OUTPUT_DIR = dlg.getOutputPath()

    debug.info('Video Paths List: ',video_paths)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    debug.info("Created {} directory\n".format(OUTPUT_DIR))
    debug.info("Check OpenCV Version: {}".format(cv2.__version__))
    
    exit = False
    for video_path in video_paths:
        debug.info('Start prossing [%s] video ...' % video_path.stem)
        # just make sure
        if exit:
            reply = QtWidgets.QMessageBox.question(None, 'Warning', 'Do you want to continue with the next source?', QtWidgets.QMessageBox.Yes, QtWidgets.QMessageBox.No)
            if reply == QtWidgets.QMessageBox.Yes:
                exit = False
            else:
                break
            
        frame_dir = OUTPUT_DIR.joinpath(video_path.stem)
        frame_dir.mkdir(parents=True, exist_ok=True)
        images_dir_path = frame_dir.joinpath(IMAGE_TAG) 
        images_dir_path.mkdir(parents=True, exist_ok=True)
        labels_dir_path = frame_dir.joinpath(LABEL_TAG)
        labels_dir_path.mkdir(parents=True, exist_ok=True)

        # open video handle
        cap = cv2.VideoCapture(str(video_path), cv2.CAP_FFMPEG) 
        start_frame = 0
        end_frame = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) - 1)
        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
        ret, image = cap.retrieve()

        tracker = ObjectTack(cpu_workers=mp.cpu_count() - 1)
        tracker.loadClasses(args.class_file)
        tracker.initBox(image)
        motion = MotionDetector()
        for frame_index in range(start_frame, end_frame):
            s = time.time()
            save_status = False
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            if cap.grab():
                ret, image = cap.retrieve()
                
            if ret:
                key = cv2.waitKey(5) 
                # press 'Esc' to exit
                if key == 27:
                    exit = True
                    break
                #  press 'Tab' or 'Enter' to reinitialize tracker (+ bboxs and labels)
                elif key == 9 or key==13:
                    tracker.initBox(image)

                # update tracker
                tracker_list = tracker.update(image)

                # write .jpg and .xml/txt to disk
                if frame_index % interval_frame == 0 : #TODO: #or motion.update(image):
                    save_status = True
                    image_file_name = str(frame_index).rjust(5, '0') + '.jpg'
                    label_file_name = str(frame_index).rjust(5, '0') + TXT_EXT
                    cv2.imwrite(str(images_dir_path.joinpath(image_file_name)), image)

                    shapes = []
                    for label, bbox in tracker_list:
                        p1 = (int(bbox[0]), int(bbox[1]))
                        p2 = (int(bbox[0] + bbox[2]), int(bbox[1]))
                        p3 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                        p4 = (int(bbox[0]), int(bbox[1] + bbox[3]))
                        shapes.append(dict(label=label, points=[p1, p2, p3, p4], difficult=False))
                        
                    label_file = LabelFile()
                    label_file.save_yolo_format(str(labels_dir_path.joinpath(label_file_name)),
                                                shapes, 
                                                str(images_dir_path.joinpath(image_file_name)),
                                                image,
                                                tracker.label_hist)
                    # label_file.save_pascal_voc_format(str(labels_dir_path.joinpath(label_file_name)),
                    #                                     shapes, 
                    #                                     str(images_dir_path.joinpath(image_file_name)), 
                    #                                     image)
                    
                # TODO: add motion UI
                # motion.DrawMotionOnFrame(image)
                # motion.DrawMotionHeatmap()
                # draw tracker bboxs on image
                for label, bbox in tracker_list:
                    sxsy = (int(bbox[0]), int(bbox[1]))
                    exey = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                    color = generate_color_by_text(label, alpha=255)

                    cv2.putText(image, label, (sxsy[0], sxsy[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (color.blue(), color.red(), color.green()), 3)
                    cv2.rectangle(image, sxsy, exey, (color.blue(), color.red(), color.green()), 2)
                image = cv2.resize(image, display_size)
                
                cv2.putText(image, f"{frame_index} / {end_frame} | Time: {round((time.time() - s)*1000, 2)} ms", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                save_text = "No Saving"
                cv2.putText(image, "Saving" if save_status else "No Saving", (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255) if save_status else (0, 255, 0), 2)
                cv2.imshow('Labeling Mode - ' + str(video_path.stem), image)
        cv2.destroyAllWindows()
        debug.info('End prossiing [%s] video ...'  % video_path.stem)