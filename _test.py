#!/usr/bin/env python
# -*- coding: utf-8 -*-
import argparse
import os.path
import logging

import sys
import numpy as np
import cv2 as cv
import webbrowser as wb
from functools import partial
from pathlib import Path
from PyQt5 import Qt
from PyQt5 import QtCore, QtGui, QtWidgets
# Note : run pyrcc5 -o modules/resources/resources.py modules/resources.qrc

from modules.labeling.libs.constants import *
from modules.labeling.libs.create_ml_io import JSON_EXT, CreateMLReader
from modules.labeling.libs.labelDialog import LabelDialog
from modules.labeling.libs.labelFile import LabelFile, LabelFileError, LabelFileFormat
from modules.labeling.libs.pascal_voc_io import XML_EXT, PascalVocReader
from modules.labeling.libs.ustr import ustr
from modules.labeling.libs.utils import *
from modules.labeling.libs.yolo_io import TXT_EXT, YoloReader
from modules import qdarkstyle
from modules.resources.resources  import *
from modules.logger import Logger

from trackingTool import ObjectTack
LABELGTING = 'LabelTool'
debug = Logger(None, logging.INFO, logging.INFO )
DISPLAT_SIZE = (1280, 720)

class QLabelCV(QtWidgets.QLabel):
    def __init__(self):
        super(QLabelCV, self).__init__()
        self.resize(640, 360)

        self._pen = None
        self._frame = []
        self._camera = None
        self._is_camera_ok = True
        self._image_format = None
        self._send_frame_thread = None

        self._roi_top_left_x = 0
        self._roi_top_left_y = 0
        self._roi_bottom_right_x = 0
        self._roi_bottom_right_y = 0

        self._adjust_area = None
        self._show_hide_adjust_area_btn = None
        self._brightness_spin = None
        self._contrast_spin = None
        self._sharpen_spin = None
        self._blur_spin = None

        self._main()

    def _main(self):
        self._init_image_format()
        self._init_thread()
        self._init_pen_style()
        self._init_adjust_area()

    def _init_image_format(self):
        # QtGui.QImage.Format_BGR888 is added in 5.14
        # Will use QtGui.QImage.Format_RGB888 if PyQt5 is lower than 5.14
        if QtCore.PYQT_VERSION_STR < '5.14':
            self._image_format = QtGui.QImage.Format_RGB888
            print('Attention: your PyQt Version is lower than 5.14, so frames are converted \n'
                  'from BGR to RGB to work with QtGui.QImage.Format_RGB888. OpenCV Color format is now RGB not BGR!!!')
        else:
            self._image_format = QtGui.QImage.Format_BGR888

    def _init_thread(self):
        self._send_frame_thread = SendFrameThread(self)
        self._send_frame_thread.frame_signal.connect(self._get_frame_from_thread)

    def _init_pen_style(self):
        self._pen = QtGui.QPen()
        self.set_pen(2, Qt.green)

    def _init_adjust_area(self):
        self._show_hide_adjust_area_btn = QtWidgets.QPushButton('⬇', self)
        self._show_hide_adjust_area_btn.move(self.width()/2, 5)
        self._show_hide_adjust_area_btn.setStyleSheet('QPushButton{border:none}')
        self._show_hide_adjust_area_btn.clicked.connect(self._show_hide_adjust_area)

        self._adjust_area = QtWidgets.QWidget(self)
        self._adjust_area.resize(self.width(), 50)
        self._adjust_area.move(0, 0)
        self._adjust_area.hide()

        self._contrast_spin = QtWidgets.QDoubleSpinBox(self._adjust_area)
        self._brightness_spin = QtWidgets.QSpinBox(self._adjust_area)
        self._sharpen_spin = QtWidgets.QSpinBox(self._adjust_area)
        self._blur_spin = QtWidgets.QSpinBox(self._adjust_area)

        self._brightness_spin.setMaximum(255)
        self._brightness_spin.setMinimum(-255)
        self._contrast_spin.setSingleStep(0.1)
        self._contrast_spin.setValue(1)
        self._blur_spin.setSingleStep(2)
        self._blur_spin.setMinimum(1)
        self._blur_spin.setValue(1)

        _h_layout = QtWidgets.QHBoxLayout()
        _h_layout.addStretch(1)
        _h_layout.addWidget(QtWidgets.QLabel('Contrast'))
        _h_layout.addWidget(self._contrast_spin)
        _h_layout.addStretch(1)
        _h_layout.addWidget(QtWidgets.QLabel('Brightness'))
        _h_layout.addWidget(self._brightness_spin)
        _h_layout.addStretch(1)
        _h_layout.addWidget(QtWidgets.QLabel('Sharpen'))
        _h_layout.addWidget(self._sharpen_spin)
        _h_layout.addStretch(1)
        _h_layout.addWidget(QtWidgets.QLabel('Blur'))
        _h_layout.addWidget(self._blur_spin)
        _h_layout.addStretch(1)
        self._adjust_area.setLayout(_h_layout)

    def _show_hide_adjust_area(self):
        if self._show_hide_adjust_area_btn.text() == '⬇':
            self._adjust_area.show()
            self._show_hide_adjust_area_btn.setText('⬆')
            self._show_hide_adjust_area_btn.move(self.width()/2, 40)
        else:
            self._adjust_area.hide()
            self._show_hide_adjust_area_btn.setText('⬇')
            self._show_hide_adjust_area_btn.move(self.width()/2, 5)

    @property
    def camera(self):
        return self._camera

    @property
    def image_format(self):
        return self._image_format

    @property
    def sharpen_value(self):
        return self._sharpen_spin.value()

    @property
    def contrast_value(self):
        return self._contrast_spin.value()

    @property
    def brightness_value(self):
        return self._brightness_spin.value()

    @property
    def blur_value(self):
        return self._blur_spin.value()

    @property
    def roi_top_left_x(self):
        return self._roi_top_left_x

    @property
    def roi_top_left_y(self):
        return self._roi_top_left_y

    @property
    def roi_bottom_right_x(self):
        return self._roi_bottom_right_x

    @property
    def roi_bottom_right_y(self):
        return self._roi_bottom_right_y

    @roi_top_left_x.setter
    def roi_top_left_x(self, value: float):
        self.roi_top_left_x = value

    @roi_top_left_y.setter
    def roi_top_left_y(self, value: float):
        self.roi_top_left_y = value

    @roi_bottom_right_x.setter
    def roi_bottom_right_x(self, value: float):
        self.roi_bottom_right_x = value

    @roi_bottom_right_y.setter
    def roi_bottom_right_y(self, value: float):
        self.roi_bottom_right_y = value

    def get_roi_top_left_x(self):
        return self._roi_top_left_x

    def get_roi_top_left_y(self):
        return self._roi_top_left_y

    def get_roi_bottom_right_x(self):
        return self._roi_bottom_right_x

    def get_roi_bottom_right_y(self):
        return self._roi_bottom_right_y

    def set_roi_top_left_x(self, value: float):
        self._roi_top_left_x = value

    def set_roi_top_left_y(self, value: float):
        self._roi_top_left_y = value

    def set_roi_bottom_right_x(self, value: float):
        self._roi_bottom_right_x = value

    def set_roi_bottom_right_y(self, value: float):
        self._roi_bottom_right_y = value

    def get_rect(self):
        return self._roi_top_left_x, self._roi_top_left_y, self._roi_bottom_right_x, self._roi_bottom_right_y

    @property
    def frame(self):
        return self._frame

    def save_frame(self, path, with_roi_rect=False):
        """
        save frame as a local picture file
        :param path: file path
        :param with_roi_rect: if to draw roi on the saved picture
        :return: None
        """
        if not self._is_camera_ok:
            print('no frame')
            return

        self._frame = np.array(self._frame)
        if self._frame.any():
            if not with_roi_rect:
                cv.imwrite(path, self._frame)
            else:
                cv.rectangle(self._frame,
                             (self._roi_top_left_x, self._roi_top_left_y),
                             (self._roi_bottom_right_x, self._roi_bottom_right_y),
                             (self._pen.color().red(), self._pen.color().green(), self._pen.color().blue()),
                             self._pen.width())
                cv.imwrite(path, self._frame)

    def get_frame(self):
        return self._frame

    def set_camera(self, device_index: int):
        self._camera = cv.VideoCapture(device_index)
        self._check_camera()

    def _check_camera(self):
        """check if the camera is available"""
        if self._camera.isOpened():
            self._is_camera_ok = True
            self._send_frame_thread.stop()
            self._send_frame_thread.start()
        else:
            self._send_frame_thread.stop()
            self._is_camera_ok = False

    def set_pen(self, width: int, color: QtGui.QColor):
        self._pen.setWidth(width)
        self._pen.setColor(color)

    def _get_frame_from_thread(self, frame: list, width: int, height: int):
        self._frame = np.array(frame)
        _frame_bytes = self._frame.tobytes()
        _image = QtGui.QImage(_frame_bytes, width, height, self._image_format)
        _pixmap = QtGui.QPixmap.fromImage(_image)
        self.setPixmap(_pixmap)

    def resizeEvent(self, event):
        self._adjust_area.resize(self.width(), 50)
        if self._show_hide_adjust_area_btn.text() == '⬇':
            self._show_hide_adjust_area_btn.move(self.width()/2, 5)
        else:
            self._show_hide_adjust_area_btn.move(self.width()/2, 40)

    def mousePressEvent(self, event):
        if not self._is_camera_ok:
            return

        if event.buttons() == Qt.LeftButton:
            self._roi_top_left_x = event.pos().x()
            self._roi_top_left_y = event.pos().y()

    def mouseMoveEvent(self, event):
        if not self._is_camera_ok:
            return

        if event.buttons() == Qt.LeftButton:
            self._roi_bottom_right_x = event.pos().x()
            self._roi_bottom_right_y = event.pos().y()

    def paintEvent(self, event):
        super(QLabelCV, self).paintEvent(event)

        _painter = QtGui.QPainter(self)
        _painter.setPen(self._pen)

        if not self._is_camera_ok:
            _painter.drawText(10, 20, 'Device Not Available')
        else:
            _painter.drawText(self._roi_top_left_x, self._roi_top_left_y, 'ROI')
            _painter.drawRect(self._roi_top_left_x,
                              self._roi_top_left_y,
                              self._roi_bottom_right_x-self._roi_top_left_x,
                              self._roi_bottom_right_y-self._roi_top_left_y)

    def closeEvent(self, event):
        self._send_frame_thread.stop()

class SendFrameThread(QtCore.QThread):
    frame_signal = QtCore.pyqtSignal(list, int, int)

    def __init__(self, parent):
        super(SendFrameThread, self).__init__()
        self._parent = parent
        self._flag = True

    def run(self):
        self._flag = True
        self._send_video_frame()
        while self._flag:
            self._send_video_frame()
            # prevent crushing
            self.usleep(1000)

    def stop(self):
        self._flag = False

    def _send_video_frame(self):
        _ret, _frame = self._parent.camera.read()
        if self._parent.image_format == QtGui.QImage.Format_RGB888:
            _frame = cv.cvtColor(_frame, cv.COLOR_BGR2RGB)

        if not _ret:
            return

        self._frame = cv.resize(_frame, DISPLAT_SIZE)
        _frame_width, _frame_height = self._frame.shape[1], self._frame.shape[0]
        self.frame_signal.emit(list(self._frame), _frame_width, _frame_height)



class MainWindow(QtWidgets.QMainWindow):
    FIT_WINDOW, FIT_WIDTH, MANUAL_ZOOM = list(range(3))

    def __init__(self, default_filename=None, default_prefdef_class_file=None, default_save_dir=None, debug=None):
        super(MainWindow, self).__init__()
        print("Check OpenCV Version : ", cv.__version__)
        if (debug == None) :
            self.debug = Logger(None, logging.INFO, logging.INFO )
        else :
            self.debug = debug
        self.resize(640, 430)
        self.setWindowTitle(LABELGTING)
        self.classes_file = default_prefdef_class_file
        self._frame = []

        self._is_camera_ok = True
        self.image_format = None
        self._send_frame_thread = None
        self._init_thread()
        self.camera = cv.VideoCapture(str(default_filename[0]))
        self._check_camera()
        self.image = QtWidgets.QLabel()
        self.setCentralWidget(self.image)

        # self.setWindowIcon(QtGui.QIcon(':/LabelGTImg'))
        # label = QLabelCV()
        # self.setCentralWidget(label)
        # label.set_camera(str(default_filename[0]))

    def _init_thread(self):
        # QtGui.QImage.Format_BGR888 is added in 5.14
        # Will use QtGui.QImage.Format_RGB888 if PyQt5 is lower than 5.14
        if QtCore.PYQT_VERSION_STR < '5.14':
            self.image_format = QtGui.QImage.Format_RGB888
            print('Attention: your PyQt Version is lower than 5.14, so frames are converted \n'
                  'from BGR to RGB to work with QtGui.QImage.Format_RGB888. OpenCV Color format is now RGB not BGR!!!')
        else:
            self.image_format = QtGui.QImage.Format_BGR888

        self._send_frame_thread = SendFrameThread(self)
        self._send_frame_thread.frame_signal.connect(self._get_frame_from_thread)

    def _get_frame_from_thread(self, frame: list, width: int, height: int):
        self._frame = np.array(frame)
        _frame_bytes = self._frame.tobytes()
        _image = QtGui.QImage(_frame_bytes, width, height, self.image_format)
        _pixmap = QtGui.QPixmap.fromImage(_image)
        self.image.setPixmap(_pixmap)

    def _check_camera(self):
        """check if the camera is available"""
        if self.camera.isOpened():
            self._is_camera_ok = True
            self._send_frame_thread.stop()
            self._send_frame_thread.start()
        else:
            self._send_frame_thread.stop()
            self._is_camera_ok = False

def get_main_app(argv=[]):
    """
    Standard boilerplate Qt application code.
    Do everything but app.exec_() -- so that we can test the application in one thread
    """
    app = QtWidgets.QApplication(argv)
    app.setApplicationName(LABELGTING)
    # app.setWindowIcon(new_icon("app"))
    app.setStyleSheet(qdarkstyle.load_stylesheet(qt_api='pyqt5'))
    # Tzutalin 201705+: Accept extra agruments to change predefined class file
    argparser = argparse.ArgumentParser(description='Multitracker for objects in the video')
    argparser.add_argument('-i','--video_dirs',
                            help='input video dir')
    argparser.add_argument('-c','--class_file',
                            default= os.path.join(os.path.dirname(__file__), 'default_classes.txt'))
    args = argparser.parse_args(argv[1:])

    ROOT_DIRS = args.video_dirs
    if not Path(ROOT_DIRS).is_dir(): 
        print("Must be a folder path.")
        exit()
    OUTPUT_DIR = Path(ROOT_DIRS).joinpath('temp')

    video_paths = []
    for item in Path(ROOT_DIRS).iterdir():
        if item.is_file():
            video_paths.append(item)      
    print('Video Paths List: ', video_paths)

    # Usage : labelImg.py image classFile saveDir
    win = MainWindow(video_paths,
                     args.class_file,
                     OUTPUT_DIR,
                     debug=debug)
    win.show()
    return app, win

def main():
    """construct main app and run it"""
    app, _win = get_main_app(sys.argv)
    return app.exec_()

if __name__ == '__main__':
    sys.exit(main())
