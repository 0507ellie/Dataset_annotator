import os, sys, subprocess
from PyQt5 import Qt
from PyQt5 import QtCore, QtGui, QtWidgets
from pathlib import Path

from tracking.libs.style import TABLE_QSS, BTN_QSS

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
        self.selectBtn.setObjectName('QPushBtn_selectFolder')
        self.selectBtn.setStyleSheet(btn_layout_qss)
        self.selectBtn.setFixedSize(50, 30)
        additemHLayout.addWidget(self.selectBtn)
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

class FileDialog(QtWidgets.QDialog):
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

        mainLayout = QtWidgets.QVBoxLayout()
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
        keyboardVLayout = QtWidgets.QVBoxLayout()
        keyboardLabel = QtWidgets.QLabel("KeyBoard Control : ")
        keyboardLabel.setStyleSheet("color: rgb(255, 255, 255);")
        keyboardLabel.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
        keyboardLabel.setMinimumHeight(20)
        keyboardVLayout.addWidget(keyboardLabel)
        data = {"『Tab』|『Enter』" : "Enter Label Painter.", 
                "『Esc』" : "Quit program/Label Painter.",
                "『W』" : "Create Box in Label Painter.",
                "『Delete』" : "Select Box and Delete in Label Painter.",
                "『Ctrl + C』" : "Select Box and Copy in Label Painter."}
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
        keyboardTableWidget.resizeColumnsToContents()
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
        self.savePathLineEdit.setText(str(Path(__file__).resolve().parents[3]))
        self.savePathBtn = QtWidgets.QPushButton()
        self.savePathBtn.setIcon(QtGui.QIcon(':/open')) 
        self.savePathBtn.setObjectName('QPushBtn_selectFolder')
        self.savePathBtn.released.connect(self.__btnMonitor)
        self.savePathBtn.setStyleSheet(BTN_QSS)
        self.savePathBtn.setFixedSize(50, 30)
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
        self.setLayout(mainLayout)
        
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
        if not self.btn_trigger:
            event.accept()
            sys.exit()
        else :  
            self.btn_trigger = False
            if self.videoLoading.checkPath():
                if Path(self.savePathLineEdit.text()).exists():
                    QtWidgets.QMessageBox.warning(None, 'Warning', "The save path already exists. Please choose a different path.")
                    event.ignore()
                else:
                    event.accept()
            else:
                QtWidgets.QMessageBox.warning(None, 'Warn', "video path can't be empty.")
                event.ignore()
