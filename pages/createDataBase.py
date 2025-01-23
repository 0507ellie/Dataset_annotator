#!/usr/bin/env python
# -*- coding: utf-8 -*-

from pages.resources.resources import *

import sys
import cv2
import argparse
import codecs
import os.path
import logging
import time
import re 
import yaml
import numpy as np
from pathlib import Path
from functools import partial
from qtpy import QtCore, QtGui, QtWidgets
from typing import *

from modules import qdarkstyle
from modules.logger import Logger
from modules.labeling.libs.yolo_io import TXT_EXT, YoloReader
from modules.labeling.libs.pascal_voc_io import XML_EXT, PascalVocReader
from modules.labeling.libs.create_ml_io import JSON_EXT, CreateMLReader
from modules.tracking.libs.style import TABLE_QSS, BTN_QSS

COMBO_BOX_QSS = """
	QComboBox {
		width: 100px;
		padding: -5px 0px -5px -2px;
		color: rgb(200, 200, 200);
		font: 12px;
	}

	QComboBox:editable {
		background: rgb(39, 44, 54);
	}

	QComboBox:!editable, QComboBox::drop-down:editable {
		background: rgb(39, 44, 54);
	}

	QComboBox:!editable:on, QComboBox::drop-down:editable:on {
		background: rgb(39, 44, 54);
	}
	
	QComboBox::drop-down {
		width:15px;
		subcontrol-position: top right;
		background: rgb(39, 44, 54);
	}

	QComboBox QAbstractItemView {
		selection-background-color: rgb(115, 121, 145);
		background-color: rgb(39, 44, 54);
		font-size: 12px;  /* Adjust the font size to be smaller */
		padding: 2px;  /* Adjust the padding to be smaller */
	}

	QComboBox QAbstractItemView::item {
		height: 20px; /* Adjust the item height to be smaller */
		text-align: center; /* Center align text in the dropdown items */
	}

	QComboBox QAbstractItemView::item:alternate {
		background-color: rgb(44, 49, 60); /* Set the alternate background color */
	}
"""
LINE_EDIT_QSSS = """
	QLineEdit {
		border: none;
		border-bottom: 1px solid white;
		background-color: rgb(54, 57, 72);
		color: rgb(200, 200, 200);
	}
	QLineEdit:enabled {
		/* Style for enabled state */
	}
	QLineEdit:disabled {
		border-bottom: 1px solid gray;
		background-color: rgba(54, 57, 72, 0.5);
		color: rgba(200, 200, 200, 0.5);
	}
"""
  
class TableWidget(QtWidgets.QTableWidget):
	ROW = 1
	COLUMN = 5
	IMAGE_TAG = "images"
	LABEL_TAG = "labels"
	DATA_TYPE_LIST = ["train", "valid", "test"] 
	class ReadOnlyDelegate(QtWidgets.QStyledItemDelegate):
		def createEditor(self, parent, option, index):
			return 

	def __init__(self, debug=None):
		super().__init__(TableWidget.ROW, TableWidget.COLUMN)
		self.debug = debug
		self.doubleFolder = False
		self.setHorizontalHeaderLabels(['Image Path', 'Label Path', "Image Count", "Label Count", "Type"])
		self.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
		
		# Allow first two columns to stretch
		self.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
		self.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
		fixed_width = 100 
		self.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.Fixed)
		self.horizontalHeader().setSectionResizeMode(3, QtWidgets.QHeaderView.Fixed)
		self.horizontalHeader().setSectionResizeMode(4, QtWidgets.QHeaderView.Fixed)
		self.horizontalHeader().resizeSection(2, fixed_width)
		self.horizontalHeader().resizeSection(3, fixed_width)
		self.horizontalHeader().resizeSection(4, fixed_width)

		delegate = TableWidget.ReadOnlyDelegate(self)
		self.setItemDelegateForColumn(2, delegate)
		self.setItemDelegateForColumn(3, delegate)
		self.setItemDelegateForColumn(4, delegate)
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
		self.cellChanged.connect(self.cellEvent)

	def cellEvent(self, row, col):
		self.blockSignals(True)
		if self.item(row, col) != None:
			if not self._not_empty(self.item(row, col).text()):
				QtWidgets.QMessageBox.warning(self, 'Warning', f'Directory cannot be empty.')
				self.blockSignals(False)
				return 

			if col == 0:
				currentfilename = self.item(row, col).text().replace('\\', '/')
				_image_count = self._checkimagefile(currentfilename)
				self.setItem(row, 2, QtWidgets.QTableWidgetItem(str(_image_count)))
					
				if _image_count <= 0:
					self.item(row, 0).setForeground(QtGui.QBrush(QtGui.QColor(255, 0, 0)))
					self.item(row, 2).setForeground(QtGui.QBrush(QtGui.QColor(255, 0, 0)))
				else:
					self.item(row, 0).setForeground(QtGui.QBrush(QtGui.QColor(200, 200, 200)))
					self.item(row, 2).setForeground(QtGui.QBrush(QtGui.QColor(200, 200, 200)))
					
			elif col == 1:
				currentlabelname = self.item(row, col).text().replace('\\', '/')
				_label_count = self._checklabelfile(currentlabelname)
				self.setItem(row, 3, QtWidgets.QTableWidgetItem(str(_label_count)))
				if _label_count < 0:
					self.item(row, 1).setForeground(QtGui.QBrush(QtGui.QColor(255, 0, 0)))
					self.item(row, 3).setForeground(QtGui.QBrush(QtGui.QColor(255, 0, 0)))
				elif _label_count == 0:
					self.item(row, 1).setForeground(QtGui.QBrush(QtGui.QColor(255, 233, 0)))
					self.item(row, 3).setForeground(QtGui.QBrush(QtGui.QColor(255, 233, 0)))
				else:
					self.item(row, 1).setForeground(QtGui.QBrush(QtGui.QColor(200, 200, 200)))
					self.item(row, 3).setForeground(QtGui.QBrush(QtGui.QColor(200, 200, 200)))
			
			if all([self.item(row, _col) for _col in range(TableWidget.COLUMN-1)]):
				comboBox = QtWidgets.QComboBox()
				comboBox.setStyleSheet(COMBO_BOX_QSS)
				comboBox.addItems(TableWidget.DATA_TYPE_LIST)
				comboBox.setEditable(True) 
				comboBox.currentIndexChanged.connect(lambda: self.comboboxEvent(row))
				if int(self.item(row, 3).text()) <= 0:
					comboBox.setCurrentIndex(2)
				ledit = comboBox.lineEdit() 
				ledit.setReadOnly(True)
				ledit.setAlignment(QtCore.Qt.AlignCenter) 
				self.setCellWidget(row, TableWidget.COLUMN-1, comboBox)

			if self.rowCount() - 1 == row:
				self._addRow()
		self.blockSignals(False)       

	def dragEnterEvent(self, event):
		if event.mimeData().hasUrls():
			event.accept()
		else:
			event.ignore()

	def dropEvent(self, event):
		files = [u.toLocalFile() for u in event.mimeData().urls()]
		for file in files:
			currentRowCount = self.rowCount() - 1  # necessary even when there are no rows in the table
			if Path(file).is_dir():
				if self.doubleFolder:
					image_dir = Path(file).joinpath(TableWidget.IMAGE_TAG)
					label_dir = Path(file).joinpath(TableWidget.LABEL_TAG)
					
					if Path(file).name == TableWidget.IMAGE_TAG:
						self._addRow()
						label_dir = Path(file).parent / TableWidget.LABEL_TAG
						if not label_dir.is_dir():
							QtWidgets.QMessageBox.warning(self, 'Warning', f'Label directory not found: {label_dir}')
						self.setItem(currentRowCount, 0, QtWidgets.QTableWidgetItem(file))
						self.setItem(currentRowCount, 1, QtWidgets.QTableWidgetItem(str(label_dir)))
		
					elif Path(file).name == TableWidget.LABEL_TAG:
						self._addRow()
						image_dir = Path(file).parent / TableWidget.IMAGE_TAG
						if not image_dir.is_dir():
							QtWidgets.QMessageBox.warning(self, 'Warning', f'Image directory not found: {image_dir}')
						self.setItem(currentRowCount, 0, QtWidgets.QTableWidgetItem(str(image_dir)))
						self.setItem(currentRowCount, 1, QtWidgets.QTableWidgetItem(file))
					
					elif image_dir.is_dir() and label_dir.is_dir():
						self._addRow()
						self.setItem(currentRowCount, 0, QtWidgets.QTableWidgetItem(str(image_dir)))
						self.setItem(currentRowCount, 1, QtWidgets.QTableWidgetItem(str(label_dir)))
					
					else:
						if not image_dir.is_dir():
							QtWidgets.QMessageBox.warning(self, 'Warning', f'Image directory not found: {image_dir}')
						if not label_dir.is_dir():
							QtWidgets.QMessageBox.warning(self, 'Warning', f'Label directory not found: {label_dir}')
				else:
					subfolders = [item for item in Path(file).iterdir() if item.is_dir()]
					if not subfolders :
						if self._checkimagefile(Path(file)):
							self.setItem(currentRowCount, 0, QtWidgets.QTableWidgetItem(file))
							self.setItem(currentRowCount, 1, QtWidgets.QTableWidgetItem(file))
					else:
						QtWidgets.QMessageBox.warning(self, 'Warning', f'Multiple folders detected. Please use the switch button to select the corresponding Image/Label DirName.')
			else:
				QtWidgets.QMessageBox.critical(self, 'Error', 'Item must be a folder path.')

	def comboboxEvent(self, row):
		comboBox = self.cellWidget(row, TableWidget.COLUMN-1)
		if comboBox is not None:
			if int(self.item(row, 3).text()) <= 0:
				comboBox.setCurrentIndex(2)
			  
	def _addRow(self):
		rowCount = self.rowCount()
		self.insertRow(rowCount)

	def _removeRow(self):
		s_items = self.selectedItems()  # Get all currently selected items
		if s_items:
			selected_rows = []  # Find the number of selected rows
			for i in s_items:
				row = i.row()
				if row not in selected_rows:
					selected_rows.append(row)
			for r in range(len(sorted(selected_rows))):
				self.removeRow(selected_rows[r] - r) 
		if self.rowCount() == 0:
			self._addRow()

	def _checkimagefile(self, f):
		count = 0
		try:
			for item in Path(f).iterdir():
				if f != "":
					if re.search(r"\.(bmp|PNG|png|jpg)$", item.name, re.IGNORECASE):
						count += 1
		except FileNotFoundError:
			msg = 'Frame Folder does Not Exist.'
			self.debug.error(msg) if self.debug else print(msg)
			count = -1
		return count

	def _checklabelfile(self, f, format=TXT_EXT):
		count = 0
		try:
			for item in Path(f).iterdir():
				if f != "":
					if re.search(r"\%s$" % format, item.name, re.IGNORECASE):
						count += 1
		except FileNotFoundError:
			msg = f'{format} in Folder does Not Exist.'
			self.debug.warn(msg) if self.debug else print(msg)
			count = -1
		return count

	def _not_empty(self, s):
		return s and s.strip()

	def showTableData(self):
		for row in range(self.rowCount()):
			for col in range(TableWidget.COLUMN):
				if self.item(row, col) != None:
					print(row, col, self.item(row, col).text())	
				else :
					widget = self.cellWidget(row, col)
					if widget and isinstance(widget, QtWidgets.QComboBox):
						print(row, col, widget.currentText())
	
	def getTableData(self):
		dicts = {name: [] for name in TableWidget.DATA_TYPE_LIST}
		for row in range (self.rowCount()):
			if self.item(row, 2) and int(self.item(row, 2).text()) <= 0:
				continue
			comboBox = self.cellWidget(row, TableWidget.COLUMN-1)
			if comboBox is not None:
				text = comboBox.currentText()
				dicts[text].append([self.item(row, 0).text(), self.item(row, 1).text() ])
		return dicts

class progressBarWorker(QtCore.QThread):

	progressBarValue = QtCore.Signal(int) 

	def __init__(self):
		super(progressBarWorker, self).__init__()

	def run(self):
		self.progressBarValue.emit(1)

class SwitchPrivate(QtCore.QObject):
	animateSignal = QtCore.Signal(bool, name='animateSignal')

	def __init__(self, q, parent=None):
		super(SwitchPrivate, self).__init__(parent)
		self.mPointer = q
		self.mPosition = 0.0
		self.mGradient = QtGui.QLinearGradient()
		self.mGradient.setSpread(QtGui.QGradient.PadSpread)
		self.checked = False  # Track the state of the switch

		self.animation = QtCore.QPropertyAnimation(self)
		self.animation.setTargetObject(self)
		self.animation.setPropertyName(b'position')
		self.animation.setStartValue(0.0)
		self.animation.setEndValue(1.0)
		self.animation.setDuration(200)
		self.animation.setEasingCurve(QtCore.QEasingCurve.InOutExpo)

		self.animation.finished.connect(self.mPointer.update)
		self.animateSignal.connect(self.animate)

	@QtCore.Property(float)
	def position(self):
		return self.mPosition

	@position.setter
	def position(self, value):
		self.mPosition = value
		self.mPointer.update()

	def draw(self, painter):
		r = self.mPointer.rect()
		margin = int(r.height() / 10)

		if self.checked:
			# Cyan shades
			shadow = QtGui.QColor(29, 233, 182).darker(130)
			light = QtGui.QColor(175, 238, 238).darker(130)
			button = QtGui.QColor(29, 233, 182)
		else:
			# Red shades
			shadow = QtGui.QColor(255, 0, 0).darker(130)
			light = QtGui.QColor(238, 175, 175).darker(130)
			button = QtGui.QColor(255, 0, 0)

		painter.setPen(QtCore.Qt.NoPen)

		self.mGradient.setColorAt(0, shadow)
		self.mGradient.setColorAt(1, light)
		self.mGradient.setStart(0, r.height())
		self.mGradient.setFinalStop(0, 0)
		painter.setBrush(self.mGradient)
		painter.drawRoundedRect(r, r.height() / 2, r.height() / 2)

		self.mGradient.setColorAt(0, shadow.darker(140))
		self.mGradient.setColorAt(1, light.darker(160))
		self.mGradient.setStart(0, 0)
		self.mGradient.setFinalStop(0, r.height())
		painter.setBrush(self.mGradient)
		painter.drawRoundedRect(r.adjusted(margin, margin, -margin, -margin), int(r.height() / 2), int(r.height() / 2))

		self.mGradient.setColorAt(0, button.darker(130))
		self.mGradient.setColorAt(1, button)

		painter.setBrush(self.mGradient)

		x = r.height() / 2.0 + self.mPosition * (r.width() - r.height())
		painter.drawEllipse(QtCore.QPointF(x, r.height() / 2), r.height() / 2 - margin, r.height() / 2 - margin)

	def animate(self, checked):
		self.checked = checked  # Update the state of the switch
		self.animation.setDirection(QtCore.QPropertyAnimation.Forward if checked else QtCore.QPropertyAnimation.Backward)
		self.animation.start()

class SwitchButton(QtWidgets.QAbstractButton):
	def __init__(self, parent=None):
		QtWidgets.QAbstractButton.__init__(self, parent=parent)
		self.dPtr = SwitchPrivate(self)
		self.setCheckable(True)
		self.clicked.connect(self.dPtr.animate)

	def sizeHint(self):
		return QtCore.QSize(44, 22)

	def paintEvent(self, event):
		painter = QtGui.QPainter(self)
		painter.setRenderHint(QtGui.QPainter.Antialiasing)
		self.dPtr.draw(painter)

	def resizeEvent(self, event):
		self.update()
		  		
class MainWidget(QtWidgets.QMainWindow):

	def __init__(self, class_path: str, debug = None) -> None:
		super().__init__()
		self.debug = debug
		# Load predefined classes to the list
		self.label_hist = None
		self.load_predefined_classes(class_path)
		
		dataTableVLayout = QtWidgets.QVBoxLayout()
		tableLabel = QtWidgets.QLabel("Drag Data Table :")
		tableLabel.setFont(QtGui.QFont("Times", 12, QtGui.QFont.Bold))
		self.tableWidget = TableWidget(self.debug)
		tagDirHLayout = QtWidgets.QHBoxLayout()
		folderBtnFLayout = QtWidgets.QFormLayout()
		subfolderLabel = QtWidgets.QLabel("Use SubFolder :")
		subfolderLabel.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
		self.subfolderBtn = SwitchButton()
		self.subfolderBtn.setObjectName('QSwitchBtn_subfolder')
		folderBtnFLayout.addRow(subfolderLabel, self.subfolderBtn)
		tagDirHLayout.addLayout(folderBtnFLayout)
		imageDirFLayout = QtWidgets.QFormLayout()
		imageDirFLayout.setContentsMargins(0, 0, 10, 0)
		imageDirLabel = QtWidgets.QLabel("Image DirName")
		imageDirLabel.setFont(QtGui.QFont('Lucida', 8, QtGui.QFont.Bold))
		self.imageDirLineEdit = QtWidgets.QLineEdit(TableWidget.IMAGE_TAG)
		self.imageDirLineEdit.setAlignment( QtCore.Qt.AlignVCenter )
		self.imageDirLineEdit.setFont(QtGui.QFont('Lucida', 8))
		self.imageDirLineEdit.setStyleSheet(LINE_EDIT_QSSS)
		self.imageDirLineEdit.setEnabled(False)
		imageDirFLayout.addRow(imageDirLabel, self.imageDirLineEdit)
		tagDirHLayout.addLayout(imageDirFLayout)
		labelDirFLayout = QtWidgets.QFormLayout()
		labelDirFLayout.setContentsMargins(0, 0, 10, 0)
		labelDirLabel = QtWidgets.QLabel("Label DirName")
		labelDirLabel.setFont(QtGui.QFont('Lucida', 8, QtGui.QFont.Bold))
		self.labelDirLineEdit = QtWidgets.QLineEdit(TableWidget.LABEL_TAG)
		self.labelDirLineEdit.setAlignment( QtCore.Qt.AlignVCenter )
		self.labelDirLineEdit.setFont(QtGui.QFont('Lucida', 8))
		self.labelDirLineEdit.setStyleSheet(LINE_EDIT_QSSS)
		self.labelDirLineEdit.setEnabled(False)
		labelDirFLayout.addRow(labelDirLabel, self.labelDirLineEdit)
		tagDirHLayout.addLayout(labelDirFLayout)
		dataTableVLayout.addWidget(tableLabel)
		dataTableVLayout.addLayout(tagDirHLayout)
		dataTableVLayout.addWidget(self.tableWidget)

		mainSettingHLayout = QtWidgets.QHBoxLayout()
		# Label List
		labelVLayout = QtWidgets.QVBoxLayout()
		labelLabel = QtWidgets.QLabel("Label List")
		labelLabel.setFont(QtGui.QFont("Times", 12, QtGui.QFont.Bold))
		labelLabel.setAlignment(QtCore.Qt.AlignCenter)
		labelLabel.setStyleSheet("""
			background-color: rgb(54, 57, 72);
			color: white; 
			border: 2px solid rgb(34, 37, 52);
			border-radius: 10px;
			padding: 5px;
			margin: 2px;  
		""") 
		self.labelList = QtWidgets.QListWidget()
		self.labelList.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
		self.labelList.setMinimumHeight(100) 
		if self.label_hist:
			[self.labelList.addItem(label) for label in self.label_hist]
		labelVLayout.setSpacing(5)
		labelVLayout.addWidget(labelLabel)
		labelVLayout.addWidget(self.labelList)
		
		# Save Setttings
		settingVLayout = QtWidgets.QVBoxLayout()
		
		self.deleteBtn = QtWidgets.QPushButton("Remove")
		self.deleteBtn.setObjectName('QPushBtn_delete')
		self.deleteBtn.setFixedSize(100, 30)
		self.deleteBtn.setStyleSheet(BTN_QSS)
		settingVLayout.addWidget(self.deleteBtn, alignment=QtCore.Qt.AlignRight)
		
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
		settingVLayout.addLayout(savepathHLayout)

		dirnameFLayout = QtWidgets.QFormLayout()
		dirnameFLayout.setContentsMargins(0,0, 60, 0)
		dirnameLabel = QtWidgets.QLabel("Project Name")
		dirnameLabel.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
		self.dirnameLineEdit = QtWidgets.QLineEdit()
		self.dirnameLineEdit.setAlignment( QtCore.Qt.AlignVCenter )
		self.dirnameLineEdit.setFont(QtGui.QFont('Lucida', 10))
		self.dirnameLineEdit.setStyleSheet("QLineEdit{border: none; border-radius: 10px; border-bottom: 1px solid white; background-color: rgb(54, 57, 72); color: rgb(200, 200, 200)}")
		dirnameFLayout.addRow(dirnameLabel, self.dirnameLineEdit)
		settingVLayout.addLayout(dirnameFLayout)

		self.logLabel = QtWidgets.QLabel()
		self.logLabel.setFont(QtGui.QFont('Lucida', 8, QtGui.QFont.Bold))
		settingVLayout.addWidget(self.logLabel)
  
		mainSettingHLayout.addLayout(labelVLayout)
		mainSettingHLayout.addLayout(settingVLayout)
		mainSettingHLayout.setStretch(0, 1)
		mainSettingHLayout.setStretch(1, 3)

		# --------------------------------------------
		#              		Footer
		# --------------------------------------------
		footerHLayout = QtWidgets.QHBoxLayout()
		self.progressBar = QtWidgets.QProgressBar()
		self.progressBar.setProperty("value", 0) 
		self.convertBtn = QtWidgets.QPushButton("Convert")
		self.convertBtn.setObjectName('QPushBtn_convert')
		self.convertBtn.setFixedSize(100, 30)
		self.convertBtn.setStyleSheet(BTN_QSS)
		footerHLayout.addWidget(self.progressBar)
		footerHLayout.addWidget(self.convertBtn)
		
		self.imageDirLineEdit.editingFinished.connect(self.__lineEditMonitor)
		self.labelDirLineEdit.editingFinished.connect(self.__lineEditMonitor)
		self.deleteBtn.released.connect(self.__btnMonitor)
		self.savepathBtn.released.connect(self.__btnMonitor)
		self.convertBtn.released.connect(self.__btnMonitor)
		self.subfolderBtn.released.connect(self.__btnMonitor)
  
		_layout = QtWidgets.QVBoxLayout()
		_layout.setSpacing(10)
		_layout.addLayout(dataTableVLayout)
		_layout.addLayout(mainSettingHLayout)
		_layout.addLayout(footerHLayout)
		_layout.setStretch(0, 2)
		_layout.setStretch(1, 1)
		# if use QtWidgets.QMainWindow
		self.centralwidget = QtWidgets.QWidget(self)
		self.centralwidget.setObjectName("centralwidget")
		self.centralwidget.setLayout(_layout)
		self.setCentralWidget(self.centralwidget)

		screen = QtWidgets.QDesktopWidget().screenGeometry()
		width = int(screen.width() * 0.4)
		height = int(screen.height() * 0.5)
		self.resize(width, height)
		
	def __btnMonitor(self) :
		sendingBtn = self.sender()
		if (sendingBtn.objectName() == "QPushBtn_delete") :
			self.tableWidget._removeRow()

		if (sendingBtn.objectName() == "QPushBtn_selectFolder") :
			folder_path = QtWidgets.QFileDialog.getExistingDirectory(self, "Select Folder")
			if folder_path:
				self.savepathLineEdit.setText(folder_path)
		
		if (sendingBtn.objectName() == "QPushBtn_convert"):
			self.convert_package()

		if (sendingBtn.objectName() == "QSwitchBtn_subfolder"):
			self.imageDirLineEdit.setEnabled(self.subfolderBtn.isChecked())
			self.labelDirLineEdit.setEnabled(self.subfolderBtn.isChecked())
			self.tableWidget.doubleFolder = self.subfolderBtn.isChecked()
   
	def __lineEditMonitor(self):
		sender = self.sender()
		if sender == self.imageDirLineEdit:
			if self.imageDirLineEdit.text() != "":
				TableWidget.IMAGE_TAG = self.imageDirLineEdit.text() 
			else:
				self.imageDirLineEdit.setText(TableWidget.IMAGE_TAG)
		elif sender == self.labelDirLineEdit:
			if self.labelDirLineEdit.text() != "":
				TableWidget.LABEL_TAG = self.labelDirLineEdit.text()
			else:
				self.labelDirLineEdit.setText(TableWidget.LABEL_TAG)
	 
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

	# TOOD: not done
	def convert_package(self):
		print("classes :", len(self.label_hist), self.label_hist)
		save_root_path = self.savepathLineEdit.text().strip()
		if save_root_path != "" and Path(save_root_path).is_dir():
			save_project_path = Path(save_root_path).joinpath(self.dirnameLineEdit.text())
			if save_project_path.is_dir():
				print("Project path already exists.") 
			else:
				save_project_path.mkdir(parents=True, exist_ok=True)
				for folder_name in self.tableWidget.getTableData().keys():
					Path(save_root_path).joinpath(folder_name)
				self.logLabel.setStyleSheet("color: Green;")
				self.logLabel.setText(f"Created project directory at '{save_project_path}'")

			self.progressBarWork = progressBarWorker()
			self.progressBarWork.progressBarValue.connect(self._updateConvertToolBar)
			data = {
				"nc" : len(self.label_hist),
				"names" : { idx:label for idx, label in enumerate(self.label_hist)},
			}
			with open( save_root_path + '/data.yaml', 'w') as outfile:
				yaml.dump(data, outfile, default_flow_style=False, sort_keys=False)
			print(self.tableWidget.getTableData())
		else:
			self.logLabel.setStyleSheet("color: red;")
			self.logLabel.setText("Path is not a directory.")



def main(argv=[]):
	app = QtWidgets.QApplication(sys.argv)
	app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
	win = MainWidget(os.path.join(os.getcwd(), 'default_classes.txt'))
	win.show()
	return app.exec_()

# if __name__ == '__main__':
# 	sys.exit(main(sys.argv))

 