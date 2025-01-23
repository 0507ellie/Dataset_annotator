
from pages.resources.resources  import *

import os, sys
import codecs
import argparse
from typing import *
from pathlib import Path
from qtpy import QtCore, QtGui, QtWidgets

from modules import qdarkstyle
from modules.labeling.libs.constants import *
from modules.labeling.libs.yolo_io import TXT_EXT, YoloReader
from modules.labeling.libs.pascal_voc_io import XML_EXT, PascalVocReader
from modules.labeling.libs.create_ml_io import JSON_EXT, CreateMLReader
from modules.labeling.libs.labelFile import LabelFileFormat, LabelFile
from modules.tracking.libs.style import TABLE_QSS, BTN_QSS, LIST_QSS


COMBO_BOX_QSS = """
	QComboBox {
		border: 1px solid gray;
		border-radius: 10px;
		padding: 1px 18px 1px 3px;
		min-width: 6em;
		font-size: 10px;
		background-color: rgb(54, 57, 72);
		color: rgb(200, 200, 200); 
	}

	QComboBox:editable {
		background: #444444; 
		color: rgb(200, 200, 200); 
	}

	QComboBox:!editable, QComboBox::drop-down:editable {
		background: rgb(54, 57, 72);
		color: rgb(200, 200, 200); 
	}

	QComboBox:!editable:on, QComboBox::drop-down:editable:on {
		background: #444444;
		color: rgb(200, 200, 200); 
	}

	QComboBox:on { /* when popup is open */
		background: #444444; 
		selection-background-color: #555555; 
		color: rgb(200, 200, 200); 
	}

	QComboBox::drop-down {
		subcontrol-origin: padding;
		subcontrol-position: top right;
		width: 15px;
		border-left-width: 1px;
		border-left-color: darkgray;
		border-left-style: solid; /* just a single line */
		border-top-right-radius: 3px; /* same radius as the QComboBox */
		border-bottom-right-radius: 3px;
	}

	QComboBox QAbstractItemView {
		border: 1px solid gray;
		selection-background-color: #555555; /* 选择项的背景颜色 */
		font-size: 16px;
		background-color: #444444; /* 深色背景 */
		color: white; /* 白色字体 */
	}
"""

class FormatConvert(object):
	IMAGE_TAG = "images"
	LABEL_TAG = "labels"
	def __init__(self, class_file) -> None:
		self.load_predefined_classes(class_file)
		self.image_path = None
		self.label_path = None
		self.shapes = []

	def change_format(self, save_format = FORMAT_YOLO):
		if save_format == FORMAT_PASCALVOC:
			LabelFile.suffix = XML_EXT
		elif save_format == FORMAT_YOLO:
			LabelFile.suffix = TXT_EXT
		elif save_format == FORMAT_CREATEML:
			LabelFile.suffix = JSON_EXT

	def load_predefined_classes(self, predef_classes_file: str) -> None:
		self.classes = []
		if os.path.exists(predef_classes_file) is True:
			with codecs.open(predef_classes_file, 'r', 'utf8') as f:
				for line in f:
					line = line.strip()
					if self.classes == []:
						self.classes = [line]
					else:
						self.classes.append(line)

	def read_annotation(self, image_path: str, label_path: str) -> None:
		xml_path = label_path if Path(label_path).suffix.lower() ==  XML_EXT else ""
		txt_path = label_path if Path(label_path).suffix.lower() ==  TXT_EXT else ""
		json_path = label_path if Path(label_path).suffix.lower() ==  JSON_EXT else ""

		reader =  QtGui.QImageReader(image_path)
		reader.setAutoTransform(True)
		image = reader.read()

		if os.path.isfile(xml_path):
			t_voc_parse_reader = PascalVocReader(xml_path) 
			self.shapes = t_voc_parse_reader.get_shapes()
			# print("PascalVocReader shape : " + str(len(self.shapes)))
		elif os.path.isfile(txt_path):
			t_yolo_parse_reader = YoloReader(txt_path, image, self.classes)
			self.shapes = t_yolo_parse_reader.get_shapes()
			# print("YoloReader shape : " + str(len(self.shapes)))
		elif os.path.isfile(json_path):
			create_ml_parse_reader = CreateMLReader(json_path, image_path)
			self.shapes = create_ml_parse_reader.get_shapes()
			# print("CreateMLReader shape : " + str(self.shapes))
		self.image_path = image_path
		self.label_path = label_path

	def convert_annotation(self, save_name: str, output_root_dir: str):
		new_image_dir = Path(output_root_dir).joinpath(FormatConvert.IMAGE_TAG)
		new_image_dir.mkdir(parents=True, exist_ok=True)
		new_label_dir = Path(output_root_dir).joinpath(FormatConvert.LABEL_TAG)
		new_label_dir.mkdir(parents=True, exist_ok=True)

		txt_path = new_label_dir.joinpath(save_name + TXT_EXT)
		xml_path = new_label_dir.joinpath(save_name + XML_EXT)
		json_path = new_label_dir.joinpath(save_name + JSON_EXT)

		reader =  QtGui.QImageReader(self.image_path)
		reader.setAutoTransform(True)
		image = reader.read()

		# save image
		image_save_path = str(new_image_dir.joinpath(save_name + Path(self.image_path).suffix.lower()))
		image.save(image_save_path)
		# save label
		label_file = LabelFile()
		convert_shapes = [ dict(label=label, points=points, difficult=difficult) for label, points, _, _, difficult in self.shapes]
		if label_file.suffix == TXT_EXT :
			label_file.save_yolo_format(txt_path, convert_shapes, image_save_path, image, self.classes)
		elif label_file.suffix == XML_EXT :
			label_file.save_pascal_voc_format(xml_path, convert_shapes, image_save_path, image) 
		elif label_file.suffix == JSON_EXT :  
			label_file.save_create_ml_format(json_path, convert_shapes, image_save_path, image, self.classes)

class DragInWidget(QtWidgets.QListWidget):
	""" Drag directories to this widget """
	itemAdded = QtCore.Signal()
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
			if Path(path).is_dir():
				self.addItem(path)
				self.itemAdded.emit()

class LoadingQWidget(QtWidgets.QWidget):
	listRequset = QtCore.Signal(list)
	def __init__(self):
		super().__init__()
		self.label = QtWidgets.QLabel("Folder Path")
		self.itemLineEdit = QtWidgets.QLineEdit(self)
		self.selectBtn = QtWidgets.QPushButton()
		self.selectBtn.setObjectName('QPushBtn_selectFolder')
		self.addBtn = QtWidgets.QPushButton("Add")  
		self.addBtn.setObjectName('QPushBtn_add')
		self.removeBtn = QtWidgets.QPushButton("Remove")
		self.removeBtn.setObjectName('QPushBtn_remove')
		self.pathList = DragInWidget(self)
		
		self.addBtn.released.connect(self.__btnMonitor) 
		self.removeBtn.released.connect(self.__btnMonitor)    
		self.selectBtn.released.connect(self.__btnMonitor)      
		self.pathList.itemSelectionChanged.connect(self.itemSelectionChange)
		self.pathList.itemAdded.connect(self.itemAddedChange)
		self.__initialize()

	def __initialize(self):
		additemHLayout = QtWidgets.QHBoxLayout()
		self.label.setStyleSheet("color : rgb(255, 255, 255);")
		self.label.setFont(QtGui.QFont('Lucida', 8, QtGui.QFont.Bold))
		self.label.setMinimumHeight(15)
		additemHLayout.addWidget(self.label)
		self.itemLineEdit.setAlignment( QtCore.Qt.AlignVCenter )
		self.itemLineEdit.setFont(QtGui.QFont('Lucida', 10))
		self.itemLineEdit.setStyleSheet("QLineEdit{border : 1px solid gray; border-radius: 10px; background-color: rgb(27,29,35); color : rgb(200, 200, 200)}")
		additemHLayout.addWidget(self.itemLineEdit)
		self.selectBtn.setIcon(QtGui.QIcon(':/open')) 
		self.selectBtn.setFocusPolicy(QtCore.Qt.NoFocus)
		self.selectBtn.setObjectName('QPushBtn_selectFolder')
		self.selectBtn.setStyleSheet(BTN_QSS)
		self.selectBtn.setFixedSize(50, 30)
		additemHLayout.addWidget(self.selectBtn)
		self.addBtn.setStyleSheet(BTN_QSS)
		self.addBtn.setFocusPolicy(QtCore.Qt.NoFocus)
		self.addBtn.setFixedSize(60, 30)
		additemHLayout.addWidget(self.addBtn)
		self.removeBtn.setEnabled(False)
		self.removeBtn.setFocusPolicy(QtCore.Qt.NoFocus)
		self.removeBtn.setStyleSheet(BTN_QSS)
		self.removeBtn.setFixedSize(90, 30)
		additemHLayout.addWidget(self.removeBtn)
		self.pathList.setStyleSheet(LIST_QSS)

		mainwindowVLayout = QtWidgets.QVBoxLayout()
		mainwindowVLayout.setContentsMargins(0, 5, 0, 0)
		mainwindowVLayout.addLayout(additemHLayout)
		mainwindowVLayout.addWidget(self.pathList)
		self.setLayout(mainwindowVLayout)

	def __btnMonitor(self) :
		sendingBtn = self.sender()
		if (sendingBtn.objectName() == "QPushBtn_selectFolder") :
			directory_path = QtWidgets.QFileDialog.getExistingDirectory(self, "Select Directory")
			if directory_path:
				self.itemLineEdit.setText(directory_path)

		if (sendingBtn.objectName() == "QPushBtn_add") :
			query = self.itemLineEdit.text()
			if Path(query).is_dir() and query != "": 
				self.pathList.addItem(query)
				self.pathList.itemAdded.emit()
			self.itemLineEdit.setText("")
		
		if (sendingBtn.objectName() == "QPushBtn_remove") :
			rn = self.pathList.currentRow()
			self.pathList.takeItem(rn)
			self.listRequset.emit(self.checkPath())

	def itemSelectionChange(self):        
		item = self.pathList.currentItem()
		if(item == None):
			self.removeBtn.setEnabled(False)
		else:
			self.removeBtn.setEnabled(True)

	def itemAddedChange(self):
		self.listRequset.emit(self.checkPath())

	def checkPath(self) -> List[Tuple[str, str]]:
		def process_folder(folder_path: str):
			image_paths, label_paths = [], []
			subfolders = [item for item in Path(folder_path).iterdir() if item.is_dir()]
			def checkimagefile(f):
				count = 0
				try:
					for item in Path(f).iterdir():
						if f != "":
							if item.suffix.lower() in ['.bmp', '.png', '.jpg']:
								count += 1
				except FileNotFoundError:
					count = -1
				return count
			
			def add_files_from_path(image_dir, label_dir=None):
				label_dir = label_dir or image_dir
				image_files = {item.stem: item for item in Path(image_dir).iterdir() 
								if item.is_file() and item.suffix.lower() in ['.bmp', '.png', '.jpg']}
				label_files = {item.stem: item for item in Path(label_dir).iterdir() 
								if item.is_file() and item.suffix.lower() in [TXT_EXT, XML_EXT, JSON_EXT]}

				for image_name, image_path in image_files.items():
					image_paths.append(str(image_path))
					label_path = label_files.get(image_name)
					if label_path:
						label_paths.append(str(label_path))
					else:
						image_paths.pop()

			if len(subfolders) > 2:
				QtWidgets.QMessageBox.warning(self, 'Warning', f'Directory [{folder_path}] contains more than two subfolders.')
				return []
			elif len(subfolders) == 2:
				# Find the image and label directories
				image_dir, label_dir = None, None
				if checkimagefile(subfolders[0]) > 0:
					image_dir = subfolders[0]
				if checkimagefile(subfolders[1]) > 0:
					image_dir = subfolders[1]
				if image_dir == subfolders[0]:
					label_dir = subfolders[1]
				else:
					label_dir = subfolders[0]

				add_files_from_path(image_dir, label_dir)
			else:
				add_files_from_path(folder_path)
			return list(zip(image_paths, label_paths))

		pathlist = []
		for index in range(self.pathList.count()):
			root_dir = self.pathList.item(index).text()
			item_paths = process_folder(root_dir)
			self.pathList.item(index).setForeground(QtGui.QBrush(QtGui.QColor(255, 0, 0) 
													if not item_paths else QtGui.QColor(0, 0, 0)))
			pathlist.extend(item_paths)
		return pathlist

class TableQWidget(QtWidgets.QTableWidget):
	ROW = 1
	COLUMN = 2

	class ReadOnlyDelegate(QtWidgets.QStyledItemDelegate):
		def createEditor(self, parent, option, index):
			return 
	
	def __init__(self, debug=None):
		super().__init__(TableQWidget.ROW, TableQWidget.COLUMN)
		self.debug = debug
		self.setHorizontalHeaderLabels(['Image Paths', 'Label Paths'])
		self.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)

		# Allow first two columns to stretch
		self.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
		self.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
		delegate = TableQWidget.ReadOnlyDelegate(self)
		self.setItemDelegateForColumn(0, delegate)
		self.setItemDelegateForColumn(1, delegate)

		self.horizontalHeader().setStretchLastSection(False)
		self.verticalHeader().setVisible(True)
		self.verticalHeader().setDefaultSectionSize(15)
		self.setDragDropMode(QtWidgets.QAbstractItemView.InternalMove)
		self.setAcceptDrops(True)
		self.setAlternatingRowColors(True)
		self.setStyleSheet(TABLE_QSS)
		self.setShowGrid(True)
		self.setGridStyle(QtCore.Qt.SolidLine)
		self.setSortingEnabled(False)
		self.verticalHeader().setVisible(False)
		# self.cellChanged.connect(self.cellEvent)

		self.itemEntered.connect(self.showFullFilename)

	def clearItems(self):
		self.setRowCount(0)

	def addItems(self, items: List[Tuple[str, str]]):
		self.clearItems()
		for item in items:
			rowCount = self.rowCount()
			self.insertRow(rowCount)
			for idx, file in enumerate(item):
				itemWidget = QtWidgets.QTableWidgetItem(file)
				itemWidget.setFlags(itemWidget.flags() ^ QtCore.Qt.ItemIsEditable)
				itemWidget.setToolTip(file)
				self.setItem(rowCount, idx, itemWidget)

	def showFullFilename(self, item):
		if item.column() in [0, 1]:
			rect = self.visualItemRect(item)
			QtWidgets.QToolTip.showText(self.mapToGlobal(rect.bottomRight()), item.text())

	def getTableData(self):
		pairlist = []
		for row in range(self.rowCount()):
			image_item = self.item(row, 0)
			label_item = self.item(row, 1)
			if image_item and label_item:
				pairlist.append((image_item.text(), label_item.text()))
		return pairlist

class progressBarWorker(QtCore.QThread):

	progressBarValue = QtCore.Signal(int) 

	def __init__(self):
		super(progressBarWorker, self).__init__()

	def run(self):
		self.progressBarValue.emit(1)

class MainWidget(QtWidgets.QWidget):

	def __init__(self, class_path: str, debug=None) -> None:
		super().__init__()
		self.debug = debug
		# Load predefined classes to the list
		self.label_hist = None
		self.load_predefined_classes(class_path)

        # --------------------------------------------
        #               Table & List
        # --------------------------------------------
		tableHLayout = QtWidgets.QVBoxLayout()
		tableLabel = QtWidgets.QLabel("Drag Data List :")
		tableLabel.setFont(QtGui.QFont("Times", 12, QtGui.QFont.Bold))
		self.dirWidget = LoadingQWidget()
		self.tableWidget = TableQWidget(self.debug)
		tableHLayout.addWidget(tableLabel)
		tableHLayout.addWidget(self.dirWidget)
		tableHLayout.addWidget(self.tableWidget)

        # --------------------------------------------
        #              Convert Settings
        # --------------------------------------------
		# change format
		formatFLayout = QtWidgets.QFormLayout()
		formatFLayout.setContentsMargins(0, 0, 60, 0)
		formatLabel = QtWidgets.QLabel("Convert to")
		formatLabel.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
		self.formatcomboBox = QtWidgets.QComboBox()
		self.formatcomboBox.setStyleSheet(COMBO_BOX_QSS)
		self.formatcomboBox.setIconSize(QtCore.QSize(20, 20))
		xmlIcon = QtGui.QIcon(':/format_voc')
		txtIcon = QtGui.QIcon(':/format_yolo')
		jsonIcon = QtGui.QIcon(':/format_createml')
		self.formatcomboBox.addItem(xmlIcon, FORMAT_PASCALVOC)
		self.formatcomboBox.addItem(txtIcon, FORMAT_YOLO)
		self.formatcomboBox.addItem(jsonIcon, FORMAT_CREATEML)
		self.formatcomboBox.setCurrentIndex(1)
		formatFLayout.addRow(formatLabel, self.formatcomboBox)

		# save path
		savepathHLayout = QtWidgets.QHBoxLayout()
		savepathHLayout.setContentsMargins(0, 10, 0, 0)
		savepathLabel = QtWidgets.QLabel("Save Path")
		savepathLabel.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
		savepathLabel.setMinimumHeight(15)
		savepathHLayout.addWidget(savepathLabel)
		self.savepathLineEdit = QtWidgets.QLineEdit()
		self.savepathLineEdit.setAlignment( QtCore.Qt.AlignVCenter )
		self.savepathLineEdit.setText(os.getcwd())
		self.savepathLineEdit.setFont(QtGui.QFont('Lucida', 10))
		self.savepathLineEdit.setStyleSheet("QLineEdit{border : 1px solid gray; border-radius: 10px; background-color: rgb(27,29,35); color : rgb(200, 200, 200)}")
		savepathHLayout.addWidget(self.savepathLineEdit)
		self.savepathBtn = QtWidgets.QPushButton()
		self.savepathBtn.setIcon(QtGui.QIcon(':/open')) 
		self.savepathBtn.setFocusPolicy(QtCore.Qt.NoFocus)
		self.savepathBtn.setObjectName('QPushBtn_selectFolder')
		self.savepathBtn.setStyleSheet(BTN_QSS)
		self.savepathBtn.setFixedSize(50, 30)
		savepathHLayout.addWidget(self.savepathBtn)

        # --------------------------------------------
        #              		Footer
        # --------------------------------------------
		footerHLayout = QtWidgets.QHBoxLayout()
		self.progressBar = QtWidgets.QProgressBar()
		self.progressBar.setProperty("value", 0) 
		self.convertBtn = QtWidgets.QPushButton("Convert")
		self.convertBtn.setFixedSize(100, 30)
		self.convertBtn.setStyleSheet(BTN_QSS)
		footerHLayout.addWidget(self.progressBar)
		footerHLayout.addWidget(self.convertBtn)

		self.dirWidget.listRequset.connect(self.tableWidget.addItems)
		self.savepathBtn.clicked.connect(self.savePathClicked) 
		self.convertBtn.clicked.connect(self.convertClicked) 

		_layout = QtWidgets.QVBoxLayout()
		_layout.setSpacing(10)
		_layout.addLayout(tableHLayout)
		_layout.addLayout(formatFLayout)
		_layout.addLayout(savepathHLayout)
		_layout.addLayout(footerHLayout)
		# if use QtWidgets.QMainWindow
		# self.centralwidget = QtWidgets.QWidget(self)
		# self.centralwidget.setObjectName("centralwidget")
		# self.centralwidget.setLayout(_layout)
		# self.setCentralWidget(self.centralwidget)
		self.setLayout(_layout)

	def _updateConvertToolBar(self, i):
		self.progressBar.setValue(i)

	def load_predefined_classes(self, predef_classes_file):
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

	def savePathClicked(self):
		folder_path = QtWidgets.QFileDialog.getExistingDirectory(self, "Select Folder")
		if folder_path:
			self.savepathLineEdit.setText(folder_path)

	def convertClicked(self):
		output_dir_path = Path(self.savepathLineEdit.text())
		if output_dir_path.exists():
			QtWidgets.QMessageBox.warning(None, 'Warning', "The save path already exists. Please create a new folder.")
			return
		else:
			output_dir_path.mkdir(parents=True, exist_ok=True)
		converter = FormatConvert(self.classes_file)
		converter.change_format(self.formatcomboBox.currentText())
		self.progressBarWork = progressBarWorker()
		self.progressBarWork.progressBarValue.connect(self._updateConvertToolBar)
		self.progressBar.setMaximum(len(self.tableWidget.getTableData()))
		for id, (image_path, label_path) in enumerate(self.tableWidget.getTableData()):
			save_name = str(id).zfill(6)
			converter.read_annotation(image_path, label_path)
			converter.convert_annotation(save_name, output_dir_path)
			self.progressBarWork.progressBarValue.emit(id+1)
		self.progressBar.setValue(0)

def main(argv=[]):
	app = QtWidgets.QApplication(sys.argv)
	app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
	win = MainWidget(os.path.join(os.getcwd(), 'default_classes.txt'))
	win.show()
	return app.exec_()
    
# if __name__ == '__main__':
# 	sys.exit(main(sys.argv))