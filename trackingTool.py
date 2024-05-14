#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Purpose: Easy video labeling using tracking
'''
import codecs, sys, subprocess
import argparse
import cv2, os
import logging
import numpy as np
import PySimpleGUI as sg 
sg.theme('Material1')
from typing import *
from pathlib import Path
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import (QApplication, QMessageBox)

from modules.libs.create_ml_io import JSON_EXT
from modules.libs.pascal_voc_io import XML_EXT
from modules.libs.yolo_io import TXT_EXT
from modules.libs.labelFile import LabelFileFormat, LabelFile
from modules.libs import qdarkstyle
from modules.logger import Logger

debug = Logger(None, logging.INFO, logging.INFO )

argparser = argparse.ArgumentParser(description='Multitracker for labeling in the video')
argparser.add_argument('-i','--video_dir',
                        help='Path to the input video directory.')
argparser.add_argument('-c','--class_file', default= os.path.join(os.path.dirname(__file__), 'default_classes.txt'),
                        help='Path to the file containing class names. Default is "default_classes.txt".')
argparser.add_argument('-o','--save_folder', default="temp", nargs="?",
                        help='Folder to save the labeled frames. Default is "temp".')
DISPLAT_SIZE = (1280, 720)

TABLE_QSS = '''
    QTableWidget {
        color: rgb(200, 200, 200);
        background-color: rgb(39, 44, 54);
        padding: 10px;
        border-radius: 5px;
        gridline-color: rgb(54, 57, 72);
        border-bottom: 1px solid rgb(54, 57, 72);
    }

    QTableWidget::item {
        border-color: rgb(54, 57, 72);
        padding-left: 5px;
        padding-right: 5px;
        gridline-color: rgb(54, 57, 72);
    }

    QTableWidget::item:selected {
        background-color: rgb(115, 121, 145);
    }

    QScrollBar:horizontal {
        border: none;
        background: rgb(52, 59, 72);
        height: 14px;
        margin: 0px 21px 0 21px;
        border-radius: 0px;
    }

    QScrollBar:vertical {
        border: none;
        background: rgb(52, 59, 72);
        width: 14px;
        margin: 21px 0 21px 0;
        border-radius: 0px;
    }

    QHeaderView {
        qproperty-defaultAlignment: AlignCenter;
        color : rgb(255, 255, 255);
        background-color: rgb(39, 44, 54);
    }

    QHeaderView::section {
        Background-color: rgb(39, 44, 54);
        max-width: 30px;
        border: 1px solid rgb(54, 57, 72);
        border-style: none;
        border-bottom: 1px solid rgb(54, 57, 72);
        border-right: 1px solid rgb(54, 57, 72);
    }

    QTableWidget::horizontalHeader {
        background-color: rgb(81, 255, 0);
    }

    QHeaderView::section:horizontal {
        border: 1px solid rgb(32, 34, 42);
        background-color: rgb(27, 29, 35);
        padding: 3px;
        border-top-left-radius: 7px;
        border-top-right-radius: 7px;
    }

    QHeaderView::section:vertical {
        border: 1px solid rgb(54, 57, 72);
    }

    /* SCROLL BARS */

    QScrollBar:horizontal {
        border: none;
        background: rgb(52, 59, 72);
        height: 14px;
        margin: 0px 21px 0 21px;
        border-radius: 0px;
    }

    QScrollBar::handle:horizontal {
        background: rgb(85, 170, 255);
        min-width: 25px;
        border-radius: 7px;
    }

    QScrollBar::add-line:horizontal {
        border: none;
        background: rgb(55, 63, 77);
        width: 20px;
        border-top-right-radius: 7px;
        border-bottom-right-radius: 7px;
        subcontrol-position: right;
        subcontrol-origin: margin;
    }

    QScrollBar::sub-line:horizontal {
        border: none;
        background: rgb(55, 63, 77);
        width: 20px;
        border-top-left-radius: 7px;
        border-bottom-left-radius: 7px;
        subcontrol-position: left;
        subcontrol-origin: margin;
    }

    QScrollBar::up-arrow:horizontal, QScrollBar::down-arrow:horizontal {
        background: none;
    }

    QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {
        background: none;
    }

    QScrollBar:vertical {
        border: none;
        background: rgb(52, 59, 72);
        width: 14px;
        margin: 21px 0 21px 0;
        border-radius: 0px;
    }

    QScrollBar::handle:vertical {
        background: rgb(85, 170, 255);
        min-height: 25px;
        border-radius: 7px;
    }

    QScrollBar::add-line:vertical {
        border: none;
        background: rgb(55, 63, 77);
        height: 20px;
        border-bottom-left-radius: 7px;
        border-bottom-right-radius: 7px;
        subcontrol-position: bottom;
        subcontrol-origin: margin;
    }

    QScrollBar::sub-line:vertical {
        border: none;
        background: rgb(55, 63, 77);
        height: 20px;
        border-top-left-radius: 7px;
        border-top-right-radius: 7px;
        subcontrol-position: top;
        subcontrol-origin: margin;
    }

    QScrollBar::up-arrow:vertical, QScrollBar::down-arrow:vertical {
        background: none;
    }

    QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical  {
        background: none;
    } '''

BTN_QSS = '''
            QPushButton:hover {
                background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1,   stop:0 rgba(40, 166, 162, 255), stop:1 rgba(60, 186, 162, 255))
            }
            QPushButton {
                border : 2px solid rgb(29, 233, 182);
                background-color: rgb(49, 54, 59);
                color : rgb(29, 233, 182);
                border-radius: 10px;
            }

            QPushButton:disabled {
                background-color: rgb(45, 90, 83);
                color : rgb(86, 170, 156);
            }'''

class DragInWidget( QtWidgets.QListWidget):
    """ Drag files to this widget """
    def __init__(self, parent=None):
        super(DragInWidget, self).__init__(parent)
        self.setAcceptDrops(True)
        self.setDragDropMode(QtWidgets.QAbstractItemView.InternalMove)

    def dragEnterEvent(self, e):
        if e.mimeData().hasUrls():
            e.accept()
        else:
            e.ignore()

    def dropEvent(self, e):
        for url in e.mimeData().urls():
            path = url.toLocalFile()
            if Path(path).is_file() and ( path.endswith('mp4') or path.endswith('avi') or path.endswith('AVI')): 
                self.addItem(path)

class LoadingQWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.label = QtWidgets.QLabel("Video Files List Path : ",self)
        self.itemLineEdit = QtWidgets.QLineEdit(self)
        self.addBtn = QtWidgets.QPushButton("Add",self)  
        self.removeBtn = QtWidgets.QPushButton("Remove",self)
        self.pathList = DragInWidget(self)
        self.addBtn.clicked.connect(self.addClicked) 
        self.removeBtn.clicked.connect(self.removeClick)      
        self.pathList.itemSelectionChanged.connect(self.itemSelectionChange)
        self.__initialize()
        # self.hide()

    def __initialize(self):
        self.setStyleSheet('background-color: rgb(61, 68, 85);')
        btn_layout_qss = '''
            QPushButton:hover {
                background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1,   stop:0 rgba(40, 166, 162, 255), stop:1 rgba(60, 186, 162, 255))
            }
            QPushButton {
                border : 2px solid rgb(29, 233, 182);
                background-color: rgb(49, 54, 59);
                color : rgb(29, 233, 182);
                border-radius: 10px;
            }

            QPushButton:disabled {
                background-color: rgb(45, 90, 83);
                color : rgb(86, 170, 156);
            }'''

        additemHLayout = QtWidgets.QHBoxLayout()
        self.itemLineEdit.setFixedHeight(30)
        self.itemLineEdit.setAlignment( QtCore.Qt.AlignVCenter )
        self.itemLineEdit.setFont(QtGui.QFont('Lucida', 10))
        self.itemLineEdit.setStyleSheet("QLineEdit{border : 1px solid lightdark; border-radius: 10px; background-color: rgb(27,29,35); color : rgb(200, 200, 200)}")
        additemHLayout.addWidget(self.itemLineEdit)
        self.addBtn.setStyleSheet(btn_layout_qss)
        self.addBtn.setFixedSize(60, 30)
        additemHLayout.addWidget(self.addBtn)
        self.removeBtn.setEnabled(False)
        self.removeBtn.setStyleSheet(btn_layout_qss)
        self.removeBtn.setFixedSize(90, 30)
        additemHLayout.addWidget(self.removeBtn)

        listWidget_layout_qss = '''
            QListView {
                color:black;
                font: 40 12pt Bold "Microsoft YaHei";
                border: 1px solid white; /* 设置边框的大小，样式，颜色 */
                border-radius: 5px;
                background-color: lightgray;
            }
            QScrollBar:vertical {
                width: 9px;
                margin: 0px 0 0px 0;
                background-color: lightgray;
            }
            QScrollBar::handle:vertical {
                min-height: 20px;
                margin: 0 1px 0 2px;
                border-radius: 3px;
                border: none;
                background: qlineargradient(spread:reflect, 
                    x1:0, y1:0, x2:1, y2:0, 
                    stop:0 rgba(85, 170, 255, 255), 
                    stop:0.5 rgba(125, 200, 255, 255),
                    stop:1 rgba(85, 170, 255, 255));
            }
            QScrollBar:horizontal {
                height: 5px;
                margin: 0px 0 0px 0;
                background-color: lightgray;
            }
            QScrollBar::handle:horizontal {
                min-width: 20px;
                margin: 0 1px 0 2px;
                border-radius: 3px;
                border: none;
                background: qlineargradient(spread:reflect, 
                    x1:0, y1:0, x2:1, y2:0, 
                    stop:0 rgba(85, 170, 255, 255), 
                    stop:0.5 rgba(125, 200, 255, 255),
                    stop:1 rgba(85, 170, 255, 255));
            }

            QScrollBar::add-line:vertical {
                height: 0px;
                subcontrol-position: bottom;
                subcontrol-origin: margin;
            }
            
            QScrollBar::sub-line:vertical {
                height: 0px;
                subcontrol-position: top;
                subcontrol-origin: margin;
            }
            
            QScrollBar::up-arrow:vertical, QScrollBar::down-arrow:vertical {
                border: 1px solid grey;
                width: 3px;
                height: 3px;
                background: white;
            }
            
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
                background: none;
            }

        '''
        self.label.setStyleSheet("color : rgb(255, 255, 255);")
        self.label.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
        self.label.setMinimumHeight(15)
        self.pathList.setStyleSheet(listWidget_layout_qss)

        mainwindowVLayout = QtWidgets.QVBoxLayout()
        mainwindowVLayout.setContentsMargins(0, 5, 0, 0)
        mainwindowVLayout.addWidget(self.label)
        mainwindowVLayout.addLayout(additemHLayout)
        mainwindowVLayout.addWidget(self.pathList)
        self.setLayout(mainwindowVLayout)

    def itemSelectionChange(self):        
        item = self.pathList.currentItem()
        if(item == None):
            self.removeBtn.setEnabled(False)
        else:
            self.removeBtn.setEnabled(True)

    def addClicked(self) :
        query = self.itemLineEdit.text()
        if Path(query).is_file() and ( query.endswith('mp4') or query.endswith('avi') or query.endswith('AVI')): 
            self.pathList.addItem(query)
        self.itemLineEdit.setText("")

    def removeClick(self):
        rn = self.pathList.currentRow()
        self.pathList.takeItem(rn)

    def checkPath(self) :
        pathlist = []
        for index in range(self.pathList.count()):
            pathlist.append(self.pathList.item(index).text())
        return pathlist if len(pathlist) > 0 else None

class SettingDialog(QtWidgets.QDialog):
    def __init__(self, __appname__: str, class_path: str) -> None:
        super().__init__()
        self.app_name = __appname__
        self.class_path = class_path
        self.btn_trigger = False

        self.setObjectName("MainWindow")
        self.setWindowTitle(self.app_name)
        self.setWindowModality(QtCore.Qt.ApplicationModal)
        self.setStyleSheet('background-color: rgb(61, 68, 85);')
        self.desktop = QtWidgets.QApplication.desktop()
        self.resize(600, int(self.desktop.height() * 0.50))
        self.move(int(self.desktop.width() * 0.52), int(self.desktop.height() * 0.1))

        mainLayout = QtWidgets.QVBoxLayout(self)
        # ======================= Classes File ===========================
        classHLayout = QtWidgets.QHBoxLayout()
        self.openLabel = QtWidgets.QLabel("Check Label Table : ")
        self.openLabel.setStyleSheet("color : rgb(255, 255, 255);")
        self.openLabel.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
        self.openLabel.setMinimumHeight(15)
        classHLayout.addWidget(self.openLabel)
        self.openBtn = QtWidgets.QPushButton("Open")
        self.openBtn.setObjectName('QPushBtn_check')
        self.openBtn.released.connect(self.__btnMonitor)
        self.openBtn.setStyleSheet(BTN_QSS)
        self.openBtn.setFixedSize(80, 25)
        classHLayout.addWidget(self.openBtn)
        classHLayout.addStretch(1)

        # ======================= Interval Frame ===========================
        intervalHLayout = QtWidgets.QHBoxLayout()
        self.intervalLabel = QtWidgets.QLabel("Interval Frame : ")
        self.intervalLabel.setStyleSheet("color : rgb(255, 255, 255);")
        self.intervalLabel.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
        self.intervalLabel.setMinimumHeight(15)
        intervalHLayout.addWidget(self.intervalLabel)
        self.intervalBox = QtWidgets.QSpinBox()
        self.intervalBox.setStyleSheet("QSpinBox{border : 1px solid lightdark; background-color: rgb(27,29,35); color : rgb(200, 200, 200); } QSpinBox:disabled {color : rgb(150, 150, 150);}")
        self.intervalBox.setValue(10)
        self.intervalBox.setRange(1, 1000)
        self.intervalBox.setAlignment( QtCore.Qt.AlignHCenter)
        self.intervalBox.setFixedSize(60, 25)
        intervalHLayout.addWidget(self.intervalBox)
        intervalHLayout.addStretch(1)

        self.videoLoading = LoadingQWidget()
        # ======================= Keyboard Info ===========================
        keyboardVLayout = QtWidgets.QVBoxLayout(self)
        keyboardLabel = QtWidgets.QLabel("KeyBoard Control : ")
        keyboardLabel.setStyleSheet("color: rgb(255, 255, 255);")
        keyboardLabel.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
        keyboardLabel.setMinimumHeight(20)
        keyboardVLayout.addWidget(keyboardLabel)
        data = {"『Enter』" : "Create new bbox.", 
                "『Delete』" : "Delete error bbox.",
                "『Esc』" : "Quit program."}
        keyboardTableWidget = QtWidgets.QTableWidget(len(data), 2)
        keyboardTableWidget.setStyleSheet(TABLE_QSS)
        keyboardTableWidget.setFrameShape(QtWidgets.QFrame.NoFrame)
        keyboardTableWidget.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOn)
        keyboardTableWidget.setSizeAdjustPolicy(QtWidgets.QAbstractScrollArea.AdjustToContents)
        keyboardTableWidget.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        keyboardTableWidget.setAlternatingRowColors(False)
        keyboardTableWidget.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        keyboardTableWidget.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        keyboardTableWidget.setShowGrid(True)
        keyboardTableWidget.setGridStyle(QtCore.Qt.SolidLine)
        keyboardTableWidget.setSortingEnabled(False)
        keyboardTableWidget.horizontalHeader().setVisible(True)
        keyboardTableWidget.horizontalHeader().setStretchLastSection(True)
        keyboardTableWidget.verticalHeader().setVisible(False)
        keyboardTableWidget.verticalHeader().setCascadingSectionResizes(False)
        keyboardTableWidget.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        keyboardTableWidget.verticalHeader().setMinimumSectionSize(20)
        keyboardTableWidget.setHorizontalHeaderLabels(["Actions","Describe"])
        for n, item in enumerate(data.keys()):
            newitem = QtWidgets.QTableWidgetItem(item)
            newitem.setTextAlignment(QtCore.Qt.AlignCenter)
            keyboardTableWidget.setItem(n,0, newitem)
            
            newitem = QtWidgets.QTableWidgetItem(data[item])
            keyboardTableWidget.setItem(n,1, newitem)
        keyboardVLayout.addWidget(keyboardTableWidget)
        
        # ======================= Save Path ===========================
        savePathHLayout = QtWidgets.QHBoxLayout()
        self.savePathLabel = QtWidgets.QLabel("Save Path : ")
        self.savePathLabel.setStyleSheet("color : rgb(255, 255, 255);")
        self.savePathLabel.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
        self.savePathLabel.setMinimumHeight(15)
        self.savePathLineEdit = QtWidgets.QLineEdit()
        self.savePathLineEdit.setStyleSheet("QLineEdit{border : 1px solid lightdark; border-radius: 10px; background-color: rgb(27,29,35); color : rgb(200, 200, 200)}")
        self.savePathLineEdit.setPlaceholderText("Select Folder Path")
        self.savePathLineEdit.setText(str(Path(__file__).resolve().parents[0]))
        self.savePathBtn = QtWidgets.QPushButton("Browse")
        self.savePathBtn.setObjectName('QPushBtn_selectFolder')
        self.savePathBtn.released.connect(self.__btnMonitor)
        self.savePathBtn.setStyleSheet(BTN_QSS)
        self.savePathBtn.setFixedSize(80, 30)
        savePathHLayout.addWidget(self.savePathLabel)
        savePathHLayout.addWidget(self.savePathLineEdit)
        savePathHLayout.addWidget(self.savePathBtn)

        self.nextBtn = QtWidgets.QPushButton("Next") 
        self.nextBtn.setObjectName('QPushBtn_next') 
        self.nextBtn.released.connect(self.__btnMonitor)
        self.nextBtn.setStyleSheet(BTN_QSS)
        self.nextBtn.setFixedHeight(30)
        
        mainLayout.addLayout(classHLayout)
        mainLayout.addLayout(intervalHLayout)
        mainLayout.addLayout(keyboardVLayout)
        mainLayout.addWidget(self.videoLoading)
        mainLayout.addLayout(savePathHLayout)
        mainLayout.addWidget(self.nextBtn)
        
    def __btnMonitor(self) :
        sendingBtn = self.sender()
        if (sendingBtn.objectName() == "QPushBtn_check") :
            platform = sys.platform
            if platform == "win32":
                subprocess.call(['start', self.class_path], shell=True)
            elif platform == "linux":
                os.system('xdg-open %s'% self.class_path)

        if (sendingBtn.objectName() == "QPushBtn_next") :
            self.btn_trigger = True
            self.close()

        if (sendingBtn.objectName() == "QPushBtn_selectFolder") :
            folder_path = QtWidgets.QFileDialog.getExistingDirectory(self, "Select Folder")
            if folder_path:
                self.savePathLineEdit.setText(folder_path)

    def getVideoList(self):
       return  [Path(item) for item in self.videoLoading.checkPath()]

    def getOutputPath(self):
        return Path(self.savePathLineEdit.text())
    
    def getInterval(self):
        return int(self.intervalBox.value())
    
    def closeEvent(self, event):
        if self.videoLoading.checkPath():
            event.accept()
            self.close()
        else:
            if not self.btn_trigger:
                event.accept()
                sys.exit()
            else: 
                QMessageBox.warning(None, 'Warn', "video path can't be empty.")
                self.btn_trigger = False
                event.ignore()

# #############################################
class ObjectTack(object):
    def __init__(self, model: str = 'csrt'):
        self.cv2_version = ''.join(cv2.__version__.split('.'))
        self.bbox_list = []
        self.track_model = model
        self.track_init = False
        self.track_alive = False
        self.label_hist = None

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
            self.multiTracker = cv2.legacy.MultiTracker_create()
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
            self.multiTracker = cv2.MultiTracker_create()
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
            rng = np.random.default_rng()
            self.classes_color = {label:rng.integers(low=0, high=255, size=3, dtype=np.uint8).tolist() 
                                  for label in self.label_hist}
        else :
            self.classes_file = None
            self.classes_color = None

    def initBox(self, src_frame: np.ndarray) -> None:
        """
        Initialize tracking with a selected bounding box from the first frame.

        Args:
            src_frame (np.ndarray): The source frame as a NumPy array.

        Returns:
            None
        """
        height, width, _ = src_frame.shape
        if (not self.track_init) :
            self.track_init = True
            frame = cv2.resize(src_frame, DISPLAT_SIZE)
            frame_show = frame.copy()
            while(True):
                bbox = np.array(cv2.selectROI('ROI Mode', frame_show, True), dtype=float)
                if (sum(bbox[-2:]) >= 10):
                    # Initialize tracker with first frame and bounding box
                    sxsy = (int(bbox[0]), int(bbox[1]))
                    exey = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                    tracking_image = frame[sxsy[1]:exey[1], sxsy[0]:exey[0]]
                    
                    if self.label_hist != None:
                        content = [[sg.Text("Select Label", font='Any 10'), 
                                    sg.Combo(self.label_hist, size=(15, 5), font='Any 12', default_value=self._label if hasattr(self, "_label") else self.label_hist[0], 
                                    readonly=True, key = "-LABEL-")]]
                    else:
                        content = [[sg.Text('Input the name of label:', font='Any 10')],  
                                    [sg.InputText(key="-LABEL-", font='Any 10')]]
                    layout = [[sg.Image(data=cv2.imencode('.png', tracking_image)[1].tobytes())], 
                              *content,
                              [sg.Submit(), sg.Cancel()]] 
                    window = sg.Window('Select ROI', layout)
                    event, label = window.read() 
                    window.close()

                    if event in (0, 'Cancel'):
                        continue
                    elif label["-LABEL-"] != "":
                        self._label = label["-LABEL-"] 
                        bbox[[0, 2]] *= width/DISPLAT_SIZE[0]
                        bbox[[1, 3]] *= height/DISPLAT_SIZE[1]
                        if self.bbox_list != []:
                            for idx, (_, box) in enumerate(self.bbox_list):
                                thres = self._iou(box, bbox)
                                if thres > 0.4:
                                    del self.bbox_list[idx] 
                        self.bbox_list.append([label["-LABEL-"], bbox])

                        self.track_alive = True
                        cv2.rectangle(frame_show, sxsy, exey, self.classes_color[label["-LABEL-"]] if self.classes_color else (0,0,255), 2)
                else:
                    if sg.popup_ok_cancel('Is the mark completed?') == 'OK':
                        break
                    else :
                        continue

            if self.track_alive:
                self._init_model(self.track_model)
                [self.multiTracker.add(self.tracker(), src_frame, tuple(box)) for label, box in self.bbox_list]
        cv2.destroyAllWindows()  
        self.track_init = False

    def removeBox(self, src_frame: np.ndarray) -> None:
        """
        Display a GUI window to select and delete an object from the tracker.

        Args:
            src_frame (np.ndarray): The source frame as a NumPy array.

        Returns:
            None
        """
        bbox_info = [ f"{idx} | {label} | {box}" for idx, (label, box) in enumerate(self.bbox_list)]
        layout = [
            [sg.Text("Select Label: ", size=(15, 1))],
            [sg.Combo(bbox_info, size=(50, 1), default_value=bbox_info[0], readonly=True, key="-LABEL-")],
            [sg.Submit(), sg.Cancel()]
        ]
        window = sg.Window('Delete Obj', layout)
        event, label = window.read()
        window.close()

        if event == "Submit":
            id = bbox_info.index(label["-LABEL-"])
            del self.bbox_list[id]

            self._init_model(self.track_model)
            [self.multiTracker.add(self.tracker(), src_frame, tuple(box)) for label, box in self.bbox_list]
                
    def update(self, src_frame: np.ndarray) -> list:
        """
        Update the tracker with the current frame and return the updated bounding box list.

        Args:
            src_frame (np.ndarray): The current frame as a NumPy array.

        Returns:
            list: The updated bounding box list.
        """
        if (self.track_alive) :
            ret, boxes = self.multiTracker.update(src_frame)
            for idx, (label, box) in enumerate(self.bbox_list):
                box[:] = boxes[idx]
        return self.bbox_list
    
if __name__ == '__main__':
    IMAGE_TAG = "images"
    LABEL_TAG = "labels"
    args = argparser.parse_args()
    app = QtWidgets.QApplication(sys.argv)
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
        dlg = SettingDialog("File Dialog", args.class_file)
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
            reply = QMessageBox.question(None, 'Warning', 'Do you want to continue with the next source?', QMessageBox.Yes, QMessageBox.No)
            if reply == QMessageBox.Yes:
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
        end_frame = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
        ret, image = cap.retrieve()

        tracker = ObjectTack()
        tracker.loadClasses(args.class_file)
        tracker.initBox(image)
        
        for frame_index in range(start_frame, end_frame):
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            if cap.grab():
                ret, image = cap.retrieve()
                
            if ret:
                key = cv2.waitKey(33) 
                # press 'Esc' to exit
                if key == 27:
                    exit = True
                    break
                #  press 'Tab' or 'Enter' to reinitialize tracker (+ bboxs and labels)
                elif key == 9 or key==13:
                    tracker.initBox(image)
                # press 'Delete' to del bbox (linux - 255 / windows - 0)
                elif key == 255 or key== 0: 
                    tracker.removeBox(image)

                # update tracker
                tracker_list = tracker.update(image)

                # write .jpg and .xml/txt to disk
                if frame_index % interval_frame == 0:
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

                # draw tracker bboxs on image
                for idx, (label, bbox) in enumerate(tracker_list):
                    sxsy = (int(bbox[0]), int(bbox[1]))
                    exey = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                    color = tracker.classes_color[label]

                    cv2.putText(image, str(idx) + " | " + label, (sxsy[0], sxsy[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                    cv2.rectangle(image, sxsy, exey, color, 2)
                image = cv2.resize(image, DISPLAT_SIZE)
                cv2.imshow('Labeling Mode - ' + str(video_path.stem), image)
        debug.info('End prossiing [%s] video ...'  % video_path.stem)