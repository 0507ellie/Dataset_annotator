try:
    from qtpy.QtGui import *
    from qtpy.QtCore import *
    from qtpy.QtWidgets import *
except ImportError:
	from PyQt4.QtGui import *
	from PyQt4.QtCore import *

import os.path, re, math
from pathlib import Path
from xml.dom.minidom import parse
from collections import OrderedDict
from dataclasses import dataclass

from libs.ustr import ustr
from libs.yolo_io import TXT_EXT, YoloReader
from libs.pascal_voc_io import XML_EXT, PascalVocReader
from libs.labelFile import LabelFileFormat, LabelFile

BTN_QSS =  '''
	QPushButton:pressed {
		background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1,   stop:0 rgba(60, 186, 162, 255), stop:1 rgba(98, 211, 162, 255))
	}
	QPushButton {
		border : 2px solid darkblue;
		color: #19232D;
		background-color: #3cbaa2;
		border-radius: 10px;
	}

	QPushButton:disabled {
		background-color: rgb(170, 170, 127)
	}'''

BAR_QSS = """
		QProgressBar {
			border: 2px solid #2196F3; /*邊框以及邊框顏色*/
			border-radius: 5px;
			background-color: #E0E0E0;
			color : black;
			text-align: center;
		}
		QProgressBar::chunk {
			background-color: #2196F3;
			width: 10px; /*區塊寬度*/
			margin: 0.5px;
		}""" 

PROGRESS_QSS = """
		QLabel{
	  		font-family: 宋体;
			font-weight:bold;
			font-size:15px;
			color: silver;}
		QProgressBar{
	  		border: 1px solid grey; /*外边框*/
			border-color:rgb(128, 128, 128); /*外边框颜色*/
			text-align: center; /*字体对齐方式*/
			color: dimgrey;
			background: rgb(255, 255, 255);}
		QProgressBar::chunk { 
  			border: none;
			background: rgb(123, 199, 187);} /*进度条颜色*/
		QPushButton{
	  		max-width:100px;  /*宽*/
			min-height:20px;  /*高*/
			color: silver;
			background-color:rgba(200,200,200,0.5);} /*设置按钮为透明*/

"""

@dataclass
class ThumbnailInfo:
	image_path: str
	annotation_path: str
	image: None
	label: str
	difficult: bool
	xmin: float = 0
	ymin: float = 0
	xmax: float = 0
	ymax: float = 0

	def __str__(self):
		return f'{os.path.basename(self.annotation_path)} | {self.center[0]}, {self.center[1]}, {self.height}, {self.width}'

	@property
	def center(self):
		return (self.xmin + self.xmax) // 2, (self.ymin + self.ymax) // 2
	
	@property
	def area(self):
		return self.width * self.height

	@property
	def area_image(self):
		crop_img = None
		if not self.image.isNull():
			self.image.convertToFormat(QImage.Format_ARGB32)
			rect = QRect( self.xmin, self.ymin , self.width, self.height )
			crop_img = self.image.copy(rect)
		return QPixmap(crop_img)
	
	@property
	def width(self):
		return self.xmax - self.xmin #+ 1
	
	@property
	def height(self):
		return self.ymax - self.ymin #+ 1

	@property
	def array(self):
		return [self.xmin, self.ymin, self.xmax, self.ymax]

	@property
	def points(self):
		return [(self.xmin, self.ymin), (self.xmax, self.ymin), (self.xmax, self.ymax), (self.xmin, self.ymax) ]
	
class Worker(QThread):

	progressBarValue = Signal(int)  # 更新进度条

	def __init__(self):
		super(Worker, self).__init__()

	def run(self):
		self.progressBarValue.emit(1)  # 发送进度条的值 信号

class ThumbnailView(QDialog):
	def __init__(self, image_path_list, label_path_list, parent = None):
		super(ThumbnailView, self).__init__(parent)
		self.setWindowTitle("Thumbnail Viewer")
		self.resize(960, 500)
		self.image_path = image_path_list
		self.label_path = label_path_list
		self.parent = parent
		self.debug = self.parent.debug

		self.Processing = True
		self.del_image_id_list = []
		self.image_id_list = []
		self.keyboard_image_id_dict = {"State" : False, "List" : []}

		self.container = QVBoxLayout()
		self.header = QHBoxLayout()
		self.displaylabel = QLabel("Display Label : ")
		self.displaylabel.setFont(QFont("Times", 8, QFont.Bold))
		self.header.addWidget(self.displaylabel)
		self.displaytext =QLineEdit()
		self.displaytext.setAlignment(Qt.AlignCenter)
		self.displaytext.setFixedSize(150, 20)
		self.displaytext.setStyleSheet(" margin: 0px; padding: 0px; border: 0.5px solid white;" )
		self.header.addWidget(self.displaytext)
		self.start_file_btn = QPushButton(self)
		self.start_file_btn.setFixedSize(100, 25)
		self.start_file_btn.setObjectName('start_pushbutton')
		self.start_file_btn.setText('Loading')
		self.start_file_btn.clicked.connect(self.start_img_viewer)
		self.start_file_btn.setStyleSheet(BTN_QSS)
		self.header.addWidget(self.start_file_btn)
		self.progressBar = QProgressBar()
		self.progressBar.setProperty("value", 0)
		self.progressBar.setStyleSheet(BAR_QSS)
		self.header.addWidget(self.progressBar)
		self.progresslabel = QLabel("( / )")
		self.progresslabel.setFont(QFont("Times", 8, QFont.Bold))
		self.header.addWidget(self.progresslabel)

		self.scroll_ares_images = QScrollArea(self)
		self.scroll_ares_images.setWidgetResizable(True)
		self.scroll_ares_images.setObjectName('scrollAreaContends')
		self.scroll_ares_images.setStyleSheet(" #scrollAreaContends{background-color: lightslategrey; border: 2px solid navy;}")
		# self.scroll_ares_images.setGeometry(20, 50, self.width, self.height)
		self.scrollAreaWidgetContents = QWidget(self)
		self.scrollAreaWidgetContents.setObjectName('scrollAreaWidgetContends')
		self.scrollAreaWidgetContents.setStyleSheet("#scrollAreaWidgetContends {background-color: lightslategrey; }")

		self.gridLayout = QGridLayout(self.scrollAreaWidgetContents)
		self.gridLayout.setSpacing(0)
		self.gridLayout.setHorizontalSpacing(0)
		self.scroll_ares_images.setWidget(self.scrollAreaWidgetContents)

		self.footer = QHBoxLayout()
		self.footer.addStretch(1)
		self.del_file_btn = QPushButton(self)
		self.del_file_btn.setFixedSize(100, 25)
		self.del_file_btn.setObjectName('del_pushbutton')
		self.del_file_btn.setText('Delete')
		self.del_file_btn.clicked.connect(self.del_file)
		self.del_file_btn.setStyleSheet(BTN_QSS)
		self.footer.addWidget(self.del_file_btn)

		self.container.addLayout(self.header)
		self.container.addWidget(self.scroll_ares_images)
		self.container.addLayout(self.footer)
		self.setLayout(self.container)

		#Default thumbnail size
		self.displayed_image_width = 60
		self.displayed_image_height = 60
		self.col = -1
		self.row = 0

	def keyPressEvent(self, event):
		if event.key() == Qt.Key_Delete:
			self.del_file()
		if event.key() == Qt.Key_Shift :
			self.keyboard_image_id_dict["State"] = True
			fwidget = QApplication.focusWidget()
			if isinstance(fwidget, QClickableImage) :
				if (fwidget.checkbox.isChecked() ) :
					self.keyboard_image_id_dict["List"].append(fwidget.objectName())
			  
	def keyReleaseEvent(self, event):
		if event.key() == Qt.Key_Shift :
			self.keyboard_image_id_dict["State"] = False
			if (len(self.keyboard_image_id_dict["List"]) >= 2) :
				init_position = -1
				end_position = -1
				for image_QWidget in self.scrollAreaWidgetContents.children():
					if (isinstance(image_QWidget, QClickableImage)):
						loop_position = -1
						if image_QWidget.objectName() == self.keyboard_image_id_dict["List"][-2] :
							loop_position = self.image_id_list.index(image_QWidget.objectName())
						
						elif image_QWidget.objectName() == self.keyboard_image_id_dict["List"][-1] :
							loop_position = self.image_id_list.index(image_QWidget.objectName())
						
						if (loop_position != -1) :
							if init_position == -1 : init_position = loop_position
							end_position = loop_position

				if (len(self.image_id_list[init_position+1 : end_position]) > 0 and end_position != -1 ):
					for image_QWidget in self.scrollAreaWidgetContents.children():
						if image_QWidget.objectName() in self.image_id_list[init_position+1 : end_position] :
							if ( not image_QWidget.checkbox.isChecked() ) :
								image_QWidget.checkbox.setChecked(True)
			self.keyboard_image_id_dict["List"] = []

	def resizeEvent(self, event):
		# size = self.centralwidget.geometry()
		for i in range(self.gridLayout.columnCount()) :
			self.gridLayout.setColumnStretch(i, 0)
		# self.scroll_ares_images.setGeometry(20, 50, size.width()- 40, size.height() - 60)

	def checkImg(self, src_frame_path):
		unicode_file_path = ustr(src_frame_path)
		unicode_file_path = os.path.abspath(unicode_file_path)
		if (os.path.exists(unicode_file_path)) :
			image_data = QImageReader(unicode_file_path).read()
			if isinstance(image_data, QImage):
				image = image_data
			else:
				image = QImage.fromData(image_data) 
			return True, image     
		else :
			self.debug.error("Frame file not exist - " + src_frame_path)  
			return False, QPixmap()

	def parseTxtFormat(self, src_frame_path, src_label_path) -> ThumbnailInfo:
		images_id_list = []
		successed , image = self.checkImg(src_frame_path)
		if not successed: return  images_id_list

		t_yolo_parse_reader = YoloReader(src_label_path, image, self.parent.classes_file)
		shapes = t_yolo_parse_reader.get_shapes()
		for idx, (label, points, line_color, fill_color, difficult) in enumerate(shapes):
			if (label != self.displaytext.text()): continue
			
			info = ThumbnailInfo(src_frame_path, src_label_path, image, label, difficult,
								 xmin = points[0][0],
								 ymin = points[0][1],
								 xmax = points[2][0],
								 ymax = points[2][1])
			if not image.isNull():
				images_id_list.append(info)
			else :
				self.debug.error("Image read error : " + src_frame_path)
		return images_id_list
  
	def parseXmlFormat(self, src_frame_path, src_label_path) -> ThumbnailInfo:
		images_id_list = []
		successed , image = self.checkImg(src_frame_path)
		if not successed: return  images_id_list

		t_voc_parse_reader = PascalVocReader(src_label_path) 
		shapes = t_voc_parse_reader.get_shapes()
		for idx, (label, points, line_color, fill_color, difficult) in enumerate(shapes):
			if (label != self.displaytext.text()): continue
			
			info = ThumbnailInfo(src_frame_path, src_label_path, image, label, difficult,
								 xmin = points[0][0],
								 ymin = points[0][1],
								 xmax = points[2][0],
								 ymax = points[2][1])
			if not image.isNull():
				images_id_list.append(info)
			else :
				self.debug.error("Image read error : " + src_frame_path)
		return images_id_list
	
	def updatebar(self, i):
		self.progressBar.setValue(i)

	def start_img_viewer(self):
		self.clearLayout()
		self.start_file_btn.setEnabled(False)
		# self.del_file_btn.setEnabled(False)
		#图像法列数
		nr_of_columns = self.get_nr_of_image_columns()
		self.max_columns = nr_of_columns
		if (self.displaytext.text() != "") :
			if (self.image_path != []) :
				num = len(self.label_path)
				if num != 0:
					self.progressBar.setMaximum(num)
					# 创建并启用子线程
					self.thread = Worker()
					self.thread.progressBarValue.connect(self.updatebar)
					for index, (frame_index, label_index) in enumerate(zip(self.image_path, self.label_path)) :
						# TODO : not success
						if self.parent.label_file_format.value == LabelFileFormat.PASCAL_VOC.value:
							for thumb in self.parseXmlFormat(frame_index, label_index):
								self.addImage(thumb)
						elif self.parent.label_file_format.value == LabelFileFormat.YOLO.value:
							for thumb in self.parseTxtFormat(frame_index, label_index):
								self.addImage(thumb)
						elif self.parent.label_file_format.value == LabelFileFormat.CREATE_ML.value:
							print("TODO: Not Done")
							pass
						else:
							pass

						if (index % 10 == 0) :
							QApplication.processEvents()
						
						self.progresslabel.setText("( {0:d} / {1:d} )".format(index+1, num) )
						self.thread.progressBarValue.emit(index+1)
						if (not self.Processing) :
							break
				else:
					QMessageBox.warning(self,'錯誤','生成图片文件为空')
			else:
				QMessageBox.warning(self,'錯誤','生成图片文件为空')
		else:
			QMessageBox.warning(self,'錯誤','Target Label Can not empty.')   
		self.start_file_btn.setEnabled(True)
	
	def del_file(self) :
		num = len(self.del_image_id_list)
		self.debug.info("Final delete count = " + str(num))
		if (num > 0) :
			progress = QProgressDialog(self)
			progress.setStyleSheet(PROGRESS_QSS)
			progress.setFixedSize(500, 100)
			progress.setWindowTitle("Please Wait")  
			progress.setLabelText("deleting ...")
			progress.setCancelButtonText("Cancel")
			progress.setMinimumDuration(5)
			progress.setWindowModality(Qt.WindowModal)
			progress.setRange(0, num) 
			i = 0
			for image_QWidget in self.scrollAreaWidgetContents.children():
				if (isinstance(image_QWidget, QClickableImage)):
					if image_QWidget.objectName() in self.del_image_id_list:
						self.gridLayout.removeWidget(image_QWidget)
						image_QWidget.setParent(None)

						temp_image_ids = [ temp_QWidget.image_id for temp_QWidget in self.scrollAreaWidgetContents.children() 
											if (isinstance(temp_QWidget, QClickableImage)) and 
												temp_QWidget.objectName() != image_QWidget.objectName() and
												temp_QWidget.image_id.annotation_path == image_QWidget.image_id.annotation_path]
						shapes = [ dict(label=image_id.label, points=image_id.points, difficult=image_id.difficult) for image_id in temp_image_ids]

						if Path(image_QWidget.image_path).exists() :
							i += 1
							self.debug.info('Delete Image Info : [ {0} ]'.format(image_QWidget.image_id))
							self.label_file = LabelFile()

							try:
								if self.parent.label_file_format.value == LabelFileFormat.PASCAL_VOC.value:
									t_voc_parse_reader = PascalVocReader(image_QWidget.image_id.annotation_path) 
									shapes += [ dict(label=label, points=points, difficult=difficult) 
												for label, points, _, _, difficult in t_voc_parse_reader.get_shapes() if label != self.displaytext.text()]

									self.label_file.save_pascal_voc_format(image_QWidget.image_id.annotation_path, 
																			shapes, 
																			image_QWidget.image_id.image_path, 
																			image_QWidget.image_id.image)
								elif self.parent.label_file_format.value == LabelFileFormat.YOLO.value:
									t_yolo_parse_reader = YoloReader(image_QWidget.image_id.annotation_path, image_QWidget.image_id.image, self.parent.classes_file)
									shapes += [ dict(label=label, points=points, difficult=difficult) 
												for label, points, _, _, difficult in t_yolo_parse_reader.get_shapes() if label != self.displaytext.text()]
									self.label_file.save_yolo_format(image_QWidget.image_id.annotation_path, 
																		shapes, 
																		image_QWidget.image_id.image_path, 
																		image_QWidget.image_id.image, 
																		self.parent.label_hist)
								elif self.parent.label_file_format.value == LabelFileFormat.CREATE_ML.value:
									# TODO: Not Done
									pass
								else:
									pass
								self.debug.info(f"File '{image_QWidget.image_id}' deleted successfully.")
							except FileNotFoundError:
								self.debug.war(f"File '{image_QWidget.image_id}' not found.")
							except PermissionError:
								self.debug.war(f"Permission denied to delete file '{image_QWidget.image_id}'.")
							except Exception as e:
								self.debug.error(f"An error occurred while deleting file '{image_QWidget.image_id}': {e}")
					else :
						self.image_id_list.append(image_QWidget.objectName())
				progress.setValue(i) 
				if (i == num) :
					progress.setValue(num)
					QMessageBox.information(self,"提示","删除成功")
					break
				if progress.wasCanceled():
					QMessageBox.warning(self,"錯誤", "刪除失敗") 
					break

			self.del_image_id_list = []

	def addImage(self, pixmap_id: ThumbnailInfo):
		# nr_of_widgets = self.gridLayout.count()
		if self.col < self.max_columns:
			self.col =self.col +1
		else:
			self.col =0
			self.row +=1

		clickable_image = QClickableImage(self.displayed_image_width, self.displayed_image_height, pixmap_id)
		clickable_image.setposition(self.row, self.col)
		clickable_image.leftClicked.connect(self.on_left_clicked)
		clickable_image.rightClicked.connect(self.on_right_clicked)
		self.gridLayout.addWidget(clickable_image, self.row, self.col)
		self.image_id_list.append(clickable_image.objectName())

	def on_left_clicked(self, image_id, status):
		if status :
			self.del_image_id_list.append(image_id)
			if (self.keyboard_image_id_dict["State"]) :
				self.keyboard_image_id_dict["List"].append(image_id)
		else :
			self.del_image_id_list.remove(image_id) 
			if (self.keyboard_image_id_dict["State"] and image_id in self.keyboard_image_id_dict["List"]) :
				self.keyboard_image_id_dict["List"].remove(image_id)
		print("Del Image List = " + str(self.del_image_id_list))

	def on_right_clicked(self, image_id):
		print('right clicked - image id = ' + image_id)

	def get_nr_of_image_columns(self):
		scroll_area_images_width = self.scroll_ares_images.width()
		if scroll_area_images_width > self.displayed_image_width:
			pic_of_columns = scroll_area_images_width // (self.displayed_image_width + 40*2)  #计算出一列几行；
		else:
			pic_of_columns = 1
		return pic_of_columns

	def setDisplayedImageSize(self,image_size):
		self.displayed_image_width =image_size

	def clearLayout(self):
		for i in reversed(range(self.gridLayout.count())):
			widgetToRemove = self.gridLayout.itemAt(i).widget()
			widgetToRemove.setParent(None)
			widgetToRemove.deleteLater()
			self.del_image_id_list = []
			self.image_id_list = []
		self.col = -1
		self.row = 0

	def closeEvent(self, event):
		self.Processing = False
		event.accept()

class QClickableImage(QWidget):
	leftClicked = Signal(str, object)
	rightClicked = Signal(object)

	def __init__(self,width =0,height =0, image_id: ThumbnailInfo = None):
		QWidget.__init__(self)
		self.setAutoFillBackground(True)
		self.setStyleSheet('background-color: lightslategrey;')
		self.layout = QVBoxLayout(self)
		self.layout.setSpacing(0)
		self.layout.setAlignment(Qt.AlignCenter)
		self.label = QLabel()
		self.label.setObjectName('label1')
		self.checkbox = QCheckBox()
		self.checkbox.stateChanged.connect(self.checkBoxChangedAction)
		self.width = width
		self.height = height
		self.image_id = image_id
		self.focus_state = False
		# self.focus_signal = False
		self.setFocusPolicy(Qt.ClickFocus)
		self.checkbox.setFocusPolicy(Qt.NoFocus)
		
		if self.width and self.height:
			self.resize(self.width, self.height)
		if self.image_id.area_image:
			pixmap = self.image_id.area_image.scaled(QSize(self.width, self.height),Qt.KeepAspectRatio,Qt.SmoothTransformation)
			self.label.setPixmap(pixmap)
			self.label.setAlignment(Qt.AlignCenter)
			self.layout.addWidget(self.label)
		if self.image_id:
			self.image_path = self.image_id.image_path
			annotation_name = str(self.image_id)
			self.setObjectName(annotation_name)
			self.checkbox.setText(annotation_name)
			self.checkbox.adjustSize()
			self.layout.addWidget(self.checkbox)
		self.setLayout(self.layout)

	def setposition(self, row, col):
		self.row, self.col = row, col

	def getposition(self):
		return self.row, self.col

	def checkBoxChangedAction(self, state) :
		if (not self.focus_state) :
			if (Qt.Checked == state):
				self.ChangeColorStyle("CheckIn")
			else:
				self.ChangeColorStyle("CheckOut")
		else :
			if (Qt.Checked == state):
				self.ChangeColorStyle("FocusIn")
			else :
				self.ChangeColorStyle("FocusOut")
		self.leftClicked.emit(self.objectName(), self.checkbox.isChecked())

	def ChangeColorStyle(self, status):
		if status == "CheckIn" :
			self.setStyleSheet('background-color: darksalmon;')
			self.setGraphicsEffect(QGraphicsOpacityEffect(opacity=.6)) 
		elif status == "CheckOut" :
			self.setStyleSheet('background-color: lightslategrey;')
			self.setGraphicsEffect(QGraphicsOpacityEffect(opacity=1))
		elif status == "FocusIn" :
			self.setStyleSheet('background-color: tomato;')
			self.setGraphicsEffect(QGraphicsOpacityEffect(opacity=.6)) 
		elif status == "FocusOut" :
			self.setStyleSheet('background-color: steelblue;')
			self.setGraphicsEffect(QGraphicsOpacityEffect(opacity=.6)) 

	def focusInEvent(self, event):
		super(QClickableImage, self).focusInEvent(event)
		self.focus_state = True

	def focusOutEvent(self, event):
		super(QClickableImage, self).focusOutEvent(event)
		self.focus_state = False
		if (self.checkbox.isChecked()):
			self.ChangeColorStyle("CheckIn")
		else:
			self.ChangeColorStyle("CheckOut")

	def mousePressEvent(self,ev):
		if ev.button() == Qt.RightButton: #鼠标右击
			self.rightClicked.emit(self.image_id)
		else:
			if self.checkbox.isChecked():
				self.checkbox.setChecked(False)
			else :
				self.checkbox.setChecked(True)

	# def keyPressEvent(self, event):
	#     if event.key() == Qt.Key_Shift :
	#         print("test :", self.image_id)
	#     if event.key() == Qt.Key_Space :
	#         print("test :", self.image_id)

	# def keyReleaseEvent(self, event):
	#     if event.key() == Qt.Key_Shift :
	#         print("test :", self.image_id)

	def imageId(self):
		return self.image_id