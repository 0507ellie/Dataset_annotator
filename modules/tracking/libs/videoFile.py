from qtpy import QtCore, QtGui, QtWidgets
from pathlib import Path

from tracking.libs.style import TABLE_QSS, BTN_QSS, LIST_QSS

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
        self.selectBtn = QtWidgets.QPushButton(self)
        self.addBtn = QtWidgets.QPushButton("Add",self)  
        self.removeBtn = QtWidgets.QPushButton("Remove",self)
        self.pathList = DragInWidget(self)
        
        self.addBtn.clicked.connect(self.addClicked) 
        self.removeBtn.clicked.connect(self.removeClick)    
        self.selectBtn.clicked.connect(self.selectClick)      
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
        self.itemLineEdit.setAlignment( QtCore.Qt.AlignVCenter )
        self.itemLineEdit.setFont(QtGui.QFont('Lucida', 10))
        self.itemLineEdit.setStyleSheet("QLineEdit{border : 1px solid lightdark; border-radius: 10px; background-color: rgb(27,29,35); color : rgb(200, 200, 200)}")
        additemHLayout.addWidget(self.itemLineEdit)
        self.selectBtn.setIcon(QtGui.QIcon(':/open')) 
        self.selectBtn.setFocusPolicy(QtCore.Qt.NoFocus)
        self.selectBtn.setObjectName('QPushBtn_selectFolder')
        self.selectBtn.setStyleSheet(btn_layout_qss)
        self.selectBtn.setFixedSize(50, 30)
        additemHLayout.addWidget(self.selectBtn)
        self.addBtn.setStyleSheet(btn_layout_qss)
        self.addBtn.setFocusPolicy(QtCore.Qt.NoFocus)
        self.addBtn.setFixedSize(60, 30)
        additemHLayout.addWidget(self.addBtn)
        self.removeBtn.setEnabled(False)
        self.removeBtn.setFocusPolicy(QtCore.Qt.NoFocus)
        self.removeBtn.setStyleSheet(btn_layout_qss)
        self.removeBtn.setFixedSize(90, 30)
        additemHLayout.addWidget(self.removeBtn)

        self.label.setStyleSheet("color : rgb(255, 255, 255);")
        self.label.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
        self.label.setMinimumHeight(15)
        self.pathList.setStyleSheet(LIST_QSS)

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

    def selectClick(self):
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Select File")
        if file_path.endswith(('mp4', 'avi', 'AVI')):
           self.itemLineEdit.setText(file_path)
                
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
