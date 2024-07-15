from qtpy import QtCore, QtGui, QtWidgets

class SwitchBtn(QtWidgets.QPushButton):
    def __init__(self, on_text="ON", close_text="OFF", parent = None):
        super().__init__(parent)
        self.on_text = on_text
        self.close_text = close_text
        self.setCheckable(True)
        self.setChecked(True)
        self.setMinimumWidth(90)
        self.setMinimumHeight(25) 
        self.setMaximumHeight(30)  
        self.label = self.on_text if self.isChecked() else self.close_text
        
    def paintEvent(self, event):
        self.label = self.on_text if self.isChecked() else self.close_text
        bg_color =  QtGui.QColor("darkcyan") if self.isChecked() else QtGui.QColor("firebrick")

        radius = 12
        width = 40
        center = self.rect().center()

        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.translate(center)
        painter.setBrush(QtGui.QColor(0,0,0))

        pen = QtGui.QPen(QtCore.Qt.black)
        pen.setWidth(2)
        painter.setPen(pen)

        painter.drawRoundedRect(QtCore.QRect(-width, -radius, 2*width, 2*radius), radius, radius)
        painter.setBrush(QtGui.QBrush(bg_color))
        
        # Set font properties
        font = QtGui.QFont("Arial", 6, QtGui.QFont.Bold) 
        painter.setFont(font)
        
        sw_rect = QtCore.QRect(-radius, -radius, width + radius, 2*radius)
        if not self.isChecked():
            sw_rect.moveLeft(-width)
        painter.drawRoundedRect(sw_rect, radius, radius)
        painter.drawText(sw_rect, QtCore.Qt.AlignCenter, self.label)