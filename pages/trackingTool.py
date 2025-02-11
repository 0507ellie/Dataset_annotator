import cv2 
import os 
import time
import logging
import argparse
import sys
import numpy as np
import multiprocessing as mp
from typing import *
from pathlib import Path
from functools import partial
from qtpy import QtCore, QtGui, QtWidgets

from modules.logger import Logger
from modules.labeling.libs.utils import *
from modules.labeling.libs.constants import *
from modules.labeling.libs.create_ml_io import JSON_EXT
from modules.labeling.libs.labelFile import LabelFileFormat, LabelFile
from modules.labeling.libs.pascal_voc_io import XML_EXT
from modules.labeling.libs.utils import generate_color_by_text
from modules.labeling.libs.yolo_io import TXT_EXT
from modules.labeling.libs.toolBar import ToolBar
from modules.tracking.tracker import TrackerDetector
from modules.tracking.motion import MotionDetector
from modules.tracking.libs.switch import SwitchBtn
from modules.tracking.libs.style import TABLE_QSS, BTN_QSS
from modules import qdarkstyle

from pages.resources.resources  import *

APPNAME = 'TrackingTool'
debug = Logger(None, logging.INFO, logging.INFO )

IMAGE_TAG = "images"
LABEL_TAG = "labels"

class WindowMixin(object):
    def toolbar(self, title, actions=None):
        toolbar = ToolBar(title)
        toolbar.setObjectName(u'%sToolBar' % title)
        toolbar.setOrientation(QtCore.Qt.Vertical)
        toolbar.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        if actions:
            add_actions(toolbar, actions)
        return toolbar

class FilterSettings(QtWidgets.QDialog):
    def __init__(self, interval_frame: int = 10, ):
        super().__init__()
        intervalHLayout = QtWidgets.QHBoxLayout()
        self.intervalLabel = QtWidgets.QLabel("Interval Frame : ")
        self.intervalLabel.setStyleSheet("color : rgb(255, 255, 255);")
        self.intervalLabel.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
        self.intervalLabel.setMinimumHeight(15)
        intervalHLayout.addWidget(self.intervalLabel)
        self.intervalBox = QtWidgets.QSpinBox()
        self.intervalBox.setStyleSheet("QSpinBox{border : 1px solid lightdark; background-color: rgb(27,29,35); color : rgb(200, 200, 200); } QSpinBox:disabled {color : rgb(150, 150, 150);}")
        self.intervalBox.setValue(interval_frame)
        self.intervalBox.setRange(1, 1000)
        self.intervalBox.setAlignment( QtCore.Qt.AlignHCenter)
        self.intervalBox.setFixedSize(60, 25)
        intervalHLayout.addWidget(self.intervalBox)
        intervalHLayout.addStretch(1)
        
        motionFLayout = QtWidgets.QFormLayout()
        motionFLayout.setAlignment(QtCore.Qt.AlignVCenter)
        motionLabel = QtWidgets.QLabel("Motion Detector : ")
        motionLabel.setFixedHeight(30)
        motionLabel.setStyleSheet("color: rgb(220, 220, 220);")
        motionLabel.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
        self.motionBtn = SwitchBtn()
        motionFLayout.addRow(motionLabel, self.motionBtn)
        
        self.setWindowTitle("Settings")
        self.setWindowModality(QtCore.Qt.ApplicationModal)
        _layout = QtWidgets.QVBoxLayout()
        _layout.addLayout(intervalHLayout)
        _layout.addLayout(motionFLayout)
        self.setLayout(_layout)
        
    def getInterval(self):
        return int(self.intervalBox.value())
    
    def getMotionSetting(self):
        return self.motionBtn.isChecked()
    
class MainWindow(QtWidgets.QMainWindow, WindowMixin):
    def __init__(self, default_filedir=None, default_prefdef_class_file=None, 
                 default_save_dir=None, debug=None):
        super(MainWindow, self).__init__()
        self.classes_file = default_prefdef_class_file
        self.debug = Logger(None, logging.INFO, logging.INFO ) if (debug == None) else debug
        
        self.setWindowTitle(APPNAME)
        self.setWindowIcon(QtGui.QIcon(":/app/trackingTool"))
        self.video_files = []
        self.timer = None
        self.tracker = None
        self.motion = None
        self.current_video_index = -1
        
        self.label_file_format = LabelFileFormat.YOLO
        self.filterDialog = FilterSettings(10)
        self.filterDialog.hide()
        self.interval_frame = self.filterDialog.getInterval()
        self.motion_status = self.filterDialog.getMotionSetting()
        
        screen = QtWidgets.QDesktopWidget().screenGeometry()
        self.resize(int(screen.width() * 0.7), 
                        int(screen.height() * 0.8))

        # TODO: add next btn
        displayHLayout = QtWidgets.QHBoxLayout()
        self.tools = self.toolbar('Tools')
        action = partial(new_action, self)
        open_files = action("Open Files", self.open_files,
                    'Ctrl+O', 'tracking/open', "Open video files")
        open_painter = action("Open Painter", self.open_painter,
                    'Tab', 'tracking/painter', "Open painter to adjust rect", enabled=False)

        change_save_dir = action("Change Save Dir", self.change_save_dir_dialog,
                                'Ctrl+r', 'tracking/save', "Change default saved root dir")

        open_filter_setting = action("Filter Setting", self.change_filter_dialog,
                                    "Ctrl+Z", "tracking/filter", "Open filter settings")
        def get_format_meta(format: LabelFileFormat):
            """
            returns a tuple containing (title, icon_name) of the selected format
            """
            if format.value == LabelFileFormat.PASCAL_VOC.value:
                return '&PascalVOC', 'format_voc'
            elif format.value == LabelFileFormat.YOLO.value:
                return '&YOLO', 'format_yolo'
            elif format.value == LabelFileFormat.CREATE_ML.value:
                return '&CreateML', 'format_createml'

        save_format = action(get_format_meta(self.label_file_format)[0],
                                self.change_format, 'Ctrl+f',
                                get_format_meta(self.label_file_format)[1],
                                "Change save format", enabled=True)

        displayHLayout.addWidget(self.tools)
        # self.addToolBar(QtCore.Qt.LeftToolBarArea, self.tools)
        add_actions(self.tools, (open_files, open_painter, change_save_dir, open_filter_setting, save_format))
        self.actions = Struct(open_painter=open_painter, save_format=save_format)
        
          # ==================================================
        #                   Video Display
        # ==================================================
        self.videoLabel = QtWidgets.QLabel()
        self.videoLabel.setAlignment(QtCore.Qt.AlignCenter)
        # Create QScrollArea and add QLabel to it
        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(self.videoLabel)
        scroll.setWidgetResizable(True)
        self.scroll_bars = {
            QtCore.Qt.Vertical: scroll.verticalScrollBar(),
            QtCore.Qt.Horizontal: scroll.horizontalScrollBar()
        }
        self.scroll_area = scroll
        displayHLayout.addWidget(self.scroll_area)

        # ==================================================
        #                   Video Slider
        # ==================================================
        self.videoSlider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.videoSlider.setMinimum(0)
        # self.videoSlider.setMaximum(self.total_frames)
        self.videoSlider.setDisabled(True)
        self.currentTimeLabel =  QtWidgets.QLabel(f"00:00:00")
        split_label =  QtWidgets.QLabel("/")
        self.totalTimeLabel =  QtWidgets.QLabel(f"00:00:00")
        timeHLayout = QtWidgets.QHBoxLayout()
        timeHLayout.addWidget(self.videoSlider)
        timeHLayout.addWidget(self.currentTimeLabel)
        timeHLayout.addWidget(split_label)
        timeHLayout.addWidget(self.totalTimeLabel)
        
          # ==================================================
        #                   Video List
        # ==================================================
        videoListVLayout = QtWidgets.QVBoxLayout()
        videoListVLayout.setContentsMargins(0, 0, 0, 0)
        self.videoList = QtWidgets.QListWidget()
        videoListVLayout.addWidget(self.videoList)
        videoListContainer = QtWidgets.QWidget()
        videoListContainer.setLayout(videoListVLayout)
        self.file_dock = QtWidgets.QDockWidget("Video List", self)
        self.file_dock.setWidget(videoListContainer)
        self.file_dock.setMinimumHeight(80)

        # Create QWidget and set layout
        mainVLayout = QtWidgets.QVBoxLayout()
        mainVLayout.addLayout(displayHLayout)
        mainVLayout.addLayout(timeHLayout)
        self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, self.file_dock)
        
        central_widget = QtWidgets.QWidget()
        central_widget.setLayout(mainVLayout)
        self.setCentralWidget(central_widget)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_image)

        self.default_save_dir  = default_save_dir
        self.images_dir_path = None
        self.labels_dir_path = None
        if default_filedir:
            video_paths = [str(item) for item in Path(default_filedir).iterdir() if item.is_file()]
            self.load_video_files(video_paths)
            self.timer.start(int(1000 // self.fps))

    def toggle_actions(self, value=True):
        self.actions.open_painter.setEnabled(value)
        self.actions.save_format.setEnabled(value)

    def set_format(self, save_format):
        if save_format == FORMAT_PASCALVOC:
            self.actions.save_format.setIconText(FORMAT_PASCALVOC)
            self.actions.save_format.setIcon(new_icon("format_voc"))
            self.label_file_format = LabelFileFormat.PASCAL_VOC
            LabelFile.suffix = XML_EXT

        elif save_format == FORMAT_YOLO:
            self.actions.save_format.setIconText(FORMAT_YOLO)
            self.actions.save_format.setIcon(new_icon("format_yolo"))
            self.label_file_format = LabelFileFormat.YOLO
            LabelFile.suffix = TXT_EXT

        elif save_format == FORMAT_CREATEML:
            self.actions.save_format.setIconText(FORMAT_CREATEML)
            self.actions.save_format.setIcon(new_icon("format_createml"))
            self.label_file_format = LabelFileFormat.CREATE_ML
            LabelFile.suffix = JSON_EXT

    def change_format(self):
        if self.label_file_format == LabelFileFormat.PASCAL_VOC:
            self.set_format(FORMAT_YOLO)
        elif self.label_file_format == LabelFileFormat.YOLO:
            self.set_format(FORMAT_CREATEML)
        elif self.label_file_format == LabelFileFormat.CREATE_ML:
            self.set_format(FORMAT_PASCALVOC)
        else:
            raise ValueError('Unknown label file format.')

    def load_video_files(self, files: List[str]):
        self.video_files.extend(files)
        self.videoList.addItems(files)
        if self.current_video_index == -1:
            self.current_video_index = 0
            item_widget = self.videoList.item(self.current_video_index)
            item_widget.setBackground(QtGui.QColor( 'darkslategrey'))
            self.play_video(self.video_files[self.current_video_index])
   
    def open_files(self, _value=False):
        if self.timer.isActive():
            self.timer.stop()  # Stop the timer
        file_paths, _ = QtWidgets.QFileDialog.getOpenFileNames(self, "Select Files", "", "Video Files (*.mp4 *.avi *.AVI)")
        if file_paths:
            new_files = [file for file in file_paths if file.lower().endswith(('.mp4', '.avi')) and 
                                                        file not in self.video_files]
            if new_files:
                self.load_video_files(new_files)
        
        if not self.timer.isActive() and self.current_video_index != -1:
            if not self.cap.isOpened():
                self.play_next_video()
            self.timer.start(int(1000 // self.fps))

    def open_painter(self):
        if self.tracker:
            if self.timer.isActive():
                self.timer.stop() 
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, self.current_frame)
            ret, image = self.cap.retrieve()
            self.tracker.initBox(image)
            if not self.timer.isActive():
                self.timer.start(int(1000 // self.fps))
            
    def init_engine(self, image):
        self.tracker = TrackerDetector(debug=debug, cpu_workers=mp.cpu_count() - 1)
        self.tracker.loadClasses(self.classes_file)
        self.tracker.initBox(image)
        self.motion = MotionDetector()

    def init_save_folder(self, name: str, root_path: Path):
        base_dir = root_path.joinpath(name)
        base_dir.mkdir(parents=True, exist_ok=True)
        self.images_dir_path = base_dir.joinpath(IMAGE_TAG) 
        self.images_dir_path.mkdir(parents=True, exist_ok=True)
        self.labels_dir_path = base_dir.joinpath(LABEL_TAG)
        self.labels_dir_path.mkdir(parents=True, exist_ok=True)
        
    def play_video(self, file_path):
        self.videoLabel.clear()
        self.cap = cv2.VideoCapture(file_path)
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.start_frame = 0
        self.end_frame = self.total_frames - 1
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, self.start_frame)
        ret, image = self.cap.retrieve()
        self.init_engine(image)
        save_path = Path(self.default_save_dir) if self.default_save_dir else Path(file_path).parent
        self.init_save_folder(str(Path(file_path).stem), save_path)
        
        self.videoSlider.setMaximum(self.total_frames)
        self.toggle_actions(True)

    def play_next_video(self):
        self.current_video_index += 1
        if self.current_video_index < len(self.video_files):
            item_widget = self.videoList.item(self.current_video_index)
            item_widget.setBackground(QtGui.QColor( 'darkslategrey'))
            self.play_video(self.video_files[self.current_video_index])
        else:
            self.current_video_index -= 1
            self.videoLabel.clear()
            self.timer.stop()

    def update_slider(self):
        # Update progress bar
        current_position = self.cap.get(cv2.CAP_PROP_POS_FRAMES)
        self.videoSlider.setValue(int(current_position))
        # Update duration label
        current_seconds = int(current_position / self.fps)
        total_seconds = int(self.total_frames / self.fps)
        current_hours = current_seconds // 3600
        current_minutes = (current_seconds % 3600) // 60
        current_seconds = current_seconds % 60
        total_hours = total_seconds // 3600
        total_minutes = (total_seconds % 3600) // 60
        total_seconds = total_seconds % 60
        self.currentTimeLabel.setText(f"{current_hours:02d}:{current_minutes:02d}:{current_seconds:02d}")
        self.totalTimeLabel.setText(f"{total_hours:02d}:{total_minutes:02d}:{total_seconds:02d}")

    def update_image(self):
        self.current_frame = int(self.cap.get(cv2.CAP_PROP_POS_FRAMES))
        ret, frame = self.cap.read()
        if ret:
            self.update_slider()
            if self.tracker:
                tracker_list = self.tracker.update(frame)
                
            if (self.current_frame % self.interval_frame == 0) or \
                (self.motion_status and self.motion.update(frame)):
                   self.save_file(frame, tracker_list)

            # TODO: add motion
            if self.motion_status:
                self.motion.DrawMotionOnFrame(frame)
                # self.motion.DrawMotionHeatmap()

            # draw tracker bboxs on image
            for label, bbox in tracker_list:
                sxsy = (int(bbox[0]), int(bbox[1]))
                exey = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                color = generate_color_by_text(label, alpha=255)

                cv2.putText(frame, label, (sxsy[0], sxsy[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (color.blue(), color.red(), color.green()), 3)
                cv2.rectangle(frame, sxsy, exey, (color.blue(), color.red(), color.green()), 2)

            # Display video frame
            size = self.scroll_area.size()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = cv2.resize(frame, (int(size.width()*0.98), int(size.height()*0.98)))
            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            q_image = QtGui.QImage(frame.data, width, height, bytes_per_line, QtGui.QImage.Format_RGB888)
            pixmap = QtGui.QPixmap.fromImage(q_image)
            self.videoLabel.setPixmap(pixmap)
        else:
            self.play_next_video()

    def save_file(self, image: np.ndarray, label_list: list):
        image_file_name = str(self.current_frame).rjust(5, '0') + '.jpg'
        label_file_name = str(self.current_frame).rjust(5, '0')
        cv2.imwrite(str(self.images_dir_path.joinpath(image_file_name)), image)
        if label_list:
            shapes = []
            for label, bbox in label_list:
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1]))
                p3 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                p4 = (int(bbox[0]), int(bbox[1] + bbox[3]))
                shapes.append(dict(label=label, type="rectangle", points=[p1, p2, p3, p4], difficult=False))
                
            label_file = LabelFile()
            if self.label_file_format == LabelFileFormat.YOLO:
                label_file_name += TXT_EXT
                label_file.save_yolo_format(str(self.labels_dir_path.joinpath(label_file_name)),
                                            shapes, 
                                            str(self.images_dir_path.joinpath(image_file_name)),
                                            image,
                                            self.tracker.label_hist)
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
                                                    self.tracker.label_hist)

    def change_save_dir_dialog(self, _value=False):
        if self.timer.isActive():
            self.timer.stop() 

        if self.default_save_dir is not None:
            path = ustr(self.default_save_dir)
        else:
            path = '.'

        dir_path = ustr(QtWidgets.QFileDialog.getExistingDirectory(self,
                                                            '%s - Save to the directory' % APPNAME, path,  QtWidgets.QFileDialog.ShowDirsOnly
                                                            | QtWidgets.QFileDialog.DontResolveSymlinks))

        if dir_path is not None and len(dir_path) > 1:
            self.default_save_dir = dir_path
            self.init_save_folder(str(Path(self.video_files[self.current_video_index]).stem), Path(self.default_save_dir))
            
        if not self.timer.isActive() and self.current_video_index != -1:
            self.timer.start(int(1000 // self.fps))
    
    def change_filter_dialog(self, _value=False):
        if self.timer.isActive():
            self.timer.stop() 
            
        self.filterDialog.exec()
        self.interval_frame = self.filterDialog.getInterval()
        self.motion_status = self.filterDialog.getMotionSetting()
        self.filterDialog.hide()
        
        if not self.timer.isActive() and self.current_video_index != -1:
            self.timer.start(int(1000 // self.fps))
            
    def closeEvent(self, event):
        if self.timer.isActive():
            self.timer.stop()
        event.accept()
    
class MainWidget(QtWidgets.QWidget):
    def __init__(self, class_path: str, debug = None):
        super().__init__()
        self.debug = debug
        self.class_path = class_path
        self.windows = []

        # --------------------------------------------
        #                Top Part
        # --------------------------------------------
        topHLayout = QtWidgets.QHBoxLayout()
        # Show UI
        imageVLayout = QtWidgets.QVBoxLayout()
        titleLabel = QtWidgets.QLabel(APPNAME + " App")
        titleLabel.setFont(QtGui.QFont("Times", 12, QtGui.QFont.Bold))
        imageVLayout.addWidget(titleLabel)
        imagelabel = QtWidgets.QLabel()
        imagelabel.setStyleSheet("border: 2px solid rgb(29, 233, 182);")
        pixmap = QtGui.QPixmap(str(Path(__file__).resolve().parents[1] / "demo/trackingUI.png"))
        screen = QtWidgets.QDesktopWidget().screenGeometry()
        width = int(screen.width() * 0.2)
        height = int(screen.height() * 0.2)
        pixmap = pixmap.scaled(width, height)
        imagelabel.setPixmap(pixmap)
        imagelabel.setScaledContents(True)
        imagelabel.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        imageVLayout.addWidget(imagelabel)
        topHLayout.addLayout(imageVLayout)

		# painter
        painterVLayout = QtWidgets.QVBoxLayout()
        painterVLayout.setContentsMargins(5, 35, 0, 0)
        painterLabel = QtWidgets.QLabel("Painter UI : ")
        painterLabel.setStyleSheet("color: rgb(255, 255, 255);")
        painterLabel.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
        painterVLayout.addWidget(painterLabel)
        painterlabel = QtWidgets.QLabel()
        painterlabel.setStyleSheet("border: 2px solid rgb(29, 233, 182);")
        pixmap = QtGui.QPixmap(str(Path(__file__).resolve().parents[1] / "demo/painterUI.png"))
        screen = QtWidgets.QDesktopWidget().screenGeometry()
        width = int(screen.width() * 0.09)
        height = int(screen.height() * 0.08)
        pixmap = pixmap.scaled(width, height)
        painterlabel.setPixmap(pixmap)
        painterHLayout = QtWidgets.QHBoxLayout()
        painterHLayout.addStretch()
        painterHLayout.addWidget(painterlabel)
        painterHLayout.addStretch()
        painterVLayout.addLayout(painterHLayout)
        data = {"『Esc』" : "Exit Painter Mode.",
                "『W』" : "Create Box in Label Painter.",
                "『Delete』" : "Select Box and Delete in Label Painter.",
                "『Ctrl + C』" : "Select Box and Copy in Label Painter."}
        painterTableWidget = QtWidgets.QTableWidget(len(data), 2)
        painterTableWidget.setStyleSheet(TABLE_QSS)
        painterTableWidget.setFrameShape(QtWidgets.QFrame.NoFrame)
        painterTableWidget.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOn)
        painterTableWidget.setSizeAdjustPolicy(QtWidgets.QAbstractScrollArea.AdjustToContents)
        painterTableWidget.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        painterTableWidget.setAlternatingRowColors(False)
        painterTableWidget.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        painterTableWidget.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        painterTableWidget.setShowGrid(True)
        painterTableWidget.setGridStyle(QtCore.Qt.SolidLine)
        painterTableWidget.setSortingEnabled(False)
        painterTableWidget.horizontalHeader().setVisible(True)
        painterTableWidget.horizontalHeader().setStretchLastSection(True)
        painterTableWidget.verticalHeader().setVisible(False)
        painterTableWidget.verticalHeader().setCascadingSectionResizes(False)
        painterTableWidget.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        painterTableWidget.setHorizontalHeaderLabels(["Actions", "Describe"])
        for n, item in enumerate(data.keys()):
            newitem = QtWidgets.QTableWidgetItem(item)
            newitem.setTextAlignment(QtCore.Qt.AlignCenter)
            painterTableWidget.setItem(n, 0, newitem)
            
            newitem = QtWidgets.QTableWidgetItem(data[item])
            painterTableWidget.setItem(n, 1, newitem)

            painterTableWidget.setRowHeight(n, 20)
        painterTableWidget.resizeColumnsToContents()
        painterVLayout.addWidget(painterTableWidget)
        topHLayout.addLayout(painterVLayout)

        # --------------------------------------------
        #               bottom Part
        # --------------------------------------------
        bottomVLayout = QtWidgets.QVBoxLayout()

        keyboardLabel = QtWidgets.QLabel("KeyBoard Control : ")
        keyboardLabel.setStyleSheet("color: rgb(255, 255, 255);")
        keyboardLabel.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
        bottomVLayout.addWidget(keyboardLabel)
        data = {"『Ctrl+O』": [':/tracking/open', "Open video files"],
                "『Tab』" : [':/tracking/painter', "Enter Label Painter."],
                "『Ctrl+R』": [':/tracking/save', "Change default saved root dir"],
                "『Ctrl+Z』" : [":/tracking/filter", "Open filter settings"],
                "『Ctrl+F』" : [":/tracking/format", "Change save format"],}
        keyboardTableWidget = QtWidgets.QTableWidget(len(data), 3)
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
        keyboardTableWidget.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.Stretch)
        keyboardTableWidget.setHorizontalHeaderLabels(["Actions", "Icon", "Describe"])
        for n, item in enumerate(data.keys()):
            newitem = QtWidgets.QTableWidgetItem(item)
            newitem.setTextAlignment(QtCore.Qt.AlignCenter)
            keyboardTableWidget.setItem(n, 0, newitem)

            newitem = QtWidgets.QTableWidgetItem()
            newitem.setTextAlignment(QtCore.Qt.AlignHCenter)
            icon = QtGui.QIcon(data[item][0])
            pixmap_size = QtCore.QSize(50, 50) if n != 4 else QtCore.QSize(80, 30)
            pixmap = icon.pixmap(pixmap_size)
            if n == 4:
                pixmap = self.recolorPixmap(pixmap, QtGui.QColor(200, 200, 200))
            scaled_pixmap = pixmap.scaled(pixmap_size)
            newitem.setIcon(QtGui.QIcon(scaled_pixmap))
            keyboardTableWidget.setItem(n, 1, newitem)

            newitem = QtWidgets.QTableWidgetItem(data[item][1])
            keyboardTableWidget.setItem(n, 2, newitem)
        keyboardTableWidget.resizeColumnsToContents()
        bottomVLayout.addWidget(keyboardTableWidget)

        # load app
        btnHLayout = QtWidgets.QHBoxLayout()
        loadBtn = QtWidgets.QPushButton("Launch")
        loadBtn.setObjectName('QPushBtn_tracking')
        loadBtn.setFixedSize(100, 30)
        loadBtn.setStyleSheet(BTN_QSS)
        loadBtn.released.connect(self.__btnMonitor)
        btnHLayout.addStretch(1)
        btnHLayout.addWidget(loadBtn)

        _layout = QtWidgets.QVBoxLayout()
        _layout.setSpacing(10)
        _layout.addLayout(topHLayout)
        _layout.addLayout(bottomVLayout)
        _layout.addLayout(btnHLayout)
        self.setLayout(_layout)

        # if use QtWidgets.QMainWindow
        # self.centralwidget = QtWidgets.QWidget(self)
        # self.centralwidget.setObjectName("centralwidget")
        # self.centralwidget.setLayout(_layout)
        # self.setCentralWidget(self.centralwidget)

    def __btnMonitor(self) :
        sendingBtn = self.sender()
        if (sendingBtn.objectName() == "QPushBtn_tracking") :
            win = MainWindow(default_prefdef_class_file=self.class_path, debug=self.debug)
            win.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
            win.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
            win.show()
            win.destroyed.connect(partial(self.onWindowClosed, win))
            self.windows.append(win) 

    def recolorPixmap(self, pixmap, color):
        image = pixmap.toImage()
        
        # Loop through all the pixels in the image and change the color
        for x in range(image.width()):
            for y in range(image.height()):
                pixel_color = image.pixelColor(x, y)
                if pixel_color.alpha() > 0:  # Only recolor non-transparent pixels
                    new_color = QtGui.QColor(color)
                    new_color.setAlpha(pixel_color.alpha())  # Preserve original alpha
                    image.setPixelColor(x, y, new_color)
        
        return QtGui.QPixmap.fromImage(image)

    def onWindowClosed(self, win):
        if win in self.windows:
            self.windows.remove(win)

def main(argv=[]):
    """construct main app and run it"""
    """
    Standard boilerplate Qt application code.
    Do everything but app.exec_() -- so that we can test the application in one thread
    """
    app = QtWidgets.QApplication(argv)
    app.setApplicationName(APPNAME)
    app.setWindowIcon(QtGui.QIcon(":/app/trackingTool"))
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
 
    argparser = argparse.ArgumentParser(description='Multitracker for labeling in the video')
    argparser.add_argument('-i','--video_dir',
                            help='Path to the input video directory.')
    argparser.add_argument('-c','--class_file', default= os.path.join(os.getcwd(), 'default_classes.txt'),
                            help='Path to the file containing class names. Default is "default_classes.txt".')
    argparser.add_argument('-o','--save_dir', default= os.path.join(os.getcwd(),  "temp"), nargs="?",
                            help='Folder to save the labeled frames. Default is "temp".')
    args = argparser.parse_args(argv[1:])

    args.video_dir = args.video_dir and os.path.normpath(args.video_dir)
    args.class_file = args.class_file and os.path.normpath(args.class_file)
    args.save_dir = args.save_dir and os.path.normpath(args.save_dir)
    win = MainWindow(args.video_dir,
                     args.class_file,
                     args.save_dir,
                     debug=debug)
    win.show()
    return app.exec_()

# if __name__ == '__main__':
#     sys.exit(main(sys.argv))