import cv2 
import numpy as np
from PyQt5 import Qt
from PyQt5 import QtCore, QtGui, QtWidgets
from pathlib import Path

from tracking.libs.style import TABLE_QSS, BTN_QSS

class DeleteDialog(QtWidgets.QDialog):
    def __init__(self, frame=np.ndarray, item_list=[], *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.frame = frame
        self.item_list = item_list
        self.setWindowTitle('Delete Obj')
        # self.setStyleSheet('background-color: rgb(61, 68, 85);')

        self.imageLabel = QtWidgets.QLabel(self)
        self.imageLabel.setAlignment(QtCore.Qt.AlignCenter) 

        idHlayout = QtWidgets.QHBoxLayout()
        idLabel = QtWidgets.QLabel('Select Object ID: ', self)
        idLabel.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
        idHlayout.addWidget(idLabel )
        self.idComboBox = QtWidgets.QComboBox(self)
        [self.idComboBox.addItem(f"{idx} | {label} | {box.astype(int)}") 
                                    for idx, (label, box) in enumerate(self.item_list)]
        self.idComboBox.activated.connect(self.update)
        idHlayout.addWidget(self.idComboBox)

        btnHlayout = QtWidgets.QHBoxLayout()
        self.submitBtn = QtWidgets.QPushButton("Submit", self) 
        self.submitBtn.setObjectName('QPushBtn_submit') 
        self.submitBtn.released.connect(self.close)
        self.submitBtn.setStyleSheet(BTN_QSS)
        self.submitBtn.setFixedHeight(30)
        btnHlayout.addWidget(self.submitBtn)

        self.cancelBtn = QtWidgets.QPushButton("Cancel", self) 
        self.cancelBtn.setObjectName('QPushBtn_cancel') 
        self.cancelBtn.released.connect(self.close)
        self.cancelBtn.setStyleSheet(BTN_QSS)
        self.cancelBtn.setFixedHeight(30)
        btnHlayout.addWidget(self.cancelBtn)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.imageLabel)
        layout.addLayout(idHlayout)
        layout.addLayout(btnHlayout)

        self.update() 

    def convert_nparray_to_QPixmap(self, img):
        h, w, ch = img.shape
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        else:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_bytes = img.tobytes()
        
        qimg = QtGui.QImage(img_bytes, w, h, w * ch, QtGui.QImage.Format_RGB888) 
        qpixmap = QtGui.QPixmap.fromImage(qimg)
        return qpixmap

    def closeEvent(self, event):
        if self.sender() == self.submitBtn:
            del self.item_list[self.idComboBox.currentIndex()]
        event.accept()

    def update(self):
        if self.item_list:
            x, y, w, h = self.item_list[self.idComboBox.currentIndex()][1].astype(int)
            image = self.frame[y:y+h, x:x+w]
            pixmap = self.convert_nparray_to_QPixmap(image)
            self.imageLabel.setPixmap(pixmap)

            self.resize(image.shape[1], image.shape[0])

    def getupdateList(self):
        return self.item_list
    