import codecs
import math
import argparse
import subprocess
import os.path
import logging
import platform
import shutil
import sys
import webbrowser as wb
from functools import partial
from pathlib import Path
from qtpy import QtCore, QtGui, QtWidgets

from modules.labeling.libs.canvas import Canvas
from modules.labeling.libs.colorDialog import ColorDialog
from modules.labeling.libs.combobox import ComboBox
from modules.labeling.libs.constants import *
from modules.labeling.libs.create_ml_io import JSON_EXT, CreateMLReader
from modules.labeling.libs.hashableQListWidgetItem import HashableQListWidgetItem
from modules.labeling.libs.labelDialog import LabelDialog
from modules.labeling.libs.labelFile import LabelFile, LabelFileError, LabelFileFormat
from modules.labeling.libs.pascal_voc_io import XML_EXT, PascalVocReader
from modules.labeling.libs.settings import Settings
from modules.labeling.libs.shape import DEFAULT_FILL_COLOR, DEFAULT_LINE_COLOR, Shape
from modules.labeling.libs.stringBundle import StringBundle
from modules.labeling.libs.thumbnailView import ThumbnailView
from modules.labeling.libs.toolBar import ToolBar
from modules.labeling.libs.switchBtn import SwitchBtn
from modules.labeling.libs.ustr import ustr
from modules.labeling.libs.utils import *
from modules.labeling.libs.yolo_io import TXT_EXT, YoloReader
from modules.labeling.libs.loadingWidget import LoadingExtension
from modules.labeling.libs.zoomWidget import ZoomWidget
from modules.tracking.libs.tagBar import TagBar
from modules.gdino import MODEL_DIR, InferenceThread, CreateThread
from modules import qdarkstyle
from modules.tracking.libs.style import TABLE_QSS, BTN_QSS
from modules.logger import Logger

from pages.resources.resources  import *

APPNAME = 'LabelingTool'
LANGUAGE = "en" # ex: 'zh-TW', 'zh-CN', 'ja-JP'
debug = Logger(None, logging.INFO, logging.INFO )

class WindowMixin(object):
	def menu(self, title, actions=None):
		menu = self.menuBar().addMenu(title)
		if actions:
			add_actions(menu, actions)
		return menu

	def toolbar(self, title, actions=None):
		toolbar = ToolBar(title)
		toolbar.setObjectName(u'%sToolBar' % title)
		# toolbar.setOrientation(Qt.Vertical)
		toolbar.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
		if actions:
			add_actions(toolbar, actions)
		self.addToolBar(QtCore.Qt.LeftToolBarArea, toolbar)
		return toolbar

class MainWindow(QtWidgets.QMainWindow, WindowMixin):
	FIT_WINDOW, FIT_WIDTH, MANUAL_ZOOM = list(range(3))

	def __init__(self, default_filename=None, default_prefdef_class_file=None, default_save_dir=None, 
				 frozen_path=False, debug=None):
		super(MainWindow, self).__init__()
		self.debug = Logger(None, logging.INFO, logging.INFO ) if (debug == None) else debug
		self.setWindowTitle(APPNAME)
		self.setWindowIcon(QtGui.QIcon(":/app/labelingTool"))

		self.qdarkstyle = False
		# Load setting in the main thread
		self.settings = Settings()
		self.settings.load()
		settings = self.settings

		# Load string bundle for i18n
		self.string_bundle = StringBundle.get_bundle(LANGUAGE)
		get_str = lambda str_id: self.string_bundle.get_string(str_id)
		
		# Save as Pascal voc xml
		self.default_save_dir = default_save_dir
		self.label_file_format = settings.get(SETTING_LABEL_FILE_FORMAT, LabelFileFormat.PASCAL_VOC)

		# For loading all image under a directory
		self.m_img_list = []
		self.dir_name = None
		self.last_open_dir = None
		self.cur_img_idx = 0
		self.img_count = 1

		# Whether we need to save or not.
		self.dirty = False

		self._no_selection_slot = False
		self._beginner = True
		self.screencast = "https://youtu.be/p0nR2YsCY_U"

		# Load predefined classes to the list
		label_hist = self.load_predefined_classes(default_prefdef_class_file)

		# Main widgets and related state.
		self.label_dialog = LabelDialog(parent=self, list_item=label_hist)
		self.label_dialog.edit.hide()
		self.label_dialog.button_box.hide()
		
		self.items_to_shapes = {}
		self.shapes_to_items = {}
		self.prev_label_text = ''

		dock_font = QtGui.QFont("Arial", 7)
		self.dock_features = QtWidgets.QDockWidget.DockWidgetClosable | QtWidgets.QDockWidget.DockWidgetFloatable
		# ==================================================
		#                   boxLabelText
		# ==================================================
		label_list_layout = QtWidgets.QVBoxLayout()
		label_list_layout.setContentsMargins(0, 0, 0, 0)
		label_list_layout.setSpacing(1)  # Adjust this value as needed

		# Create box size ratio
		self.use_box_size_checkbox = QtWidgets.QCheckBox("Box Size Ratio :")
		self.use_box_size_checkbox.setFont(dock_font)
		self.use_box_size_checkbox.setChecked(False)
		self.use_box_size_checkbox.stateChanged.connect(self.change_box_size_ratio)
		self.Wspbox = QtWidgets.QSpinBox()
		self.Wspbox.setMinimumHeight(10) 
		self.Wspbox.setMaximumHeight(30) 
		self.Wspbox.setValue(48)
		self.Wspbox.setRange(1, 160)
		self.Wspbox.setEnabled(False)
		self.Wspbox.valueChanged.connect(self.change_box_size_ratio)
		self.Hlabel = QtWidgets.QLabel(" x ")
		self.Hspbox = QtWidgets.QSpinBox()
		self.Hspbox.setMinimumHeight(10) 
		self.Hspbox.setMaximumHeight(30) 
		self.Hspbox.setValue(48)
		self.Hspbox.setRange(1, 160)
		self.Hspbox.setEnabled(False)
		self.Hspbox.valueChanged.connect(self.change_box_size_ratio)

		boxSizelHlayout = QtWidgets.QHBoxLayout()
		boxSizelHlayout.setContentsMargins(5, 10, 10, 5) 
		boxSizelHlayout.addWidget(self.use_box_size_checkbox)
		boxSizelHlayout.addWidget(self.Wspbox, QtCore.Qt.AlignLeft)
		boxSizelHlayout.addWidget(self.Hlabel)
		boxSizelHlayout.addWidget(self.Hspbox, QtCore.Qt.AlignLeft)
		use_box_size_container = QtWidgets.QWidget()
		use_box_size_container.setLayout(boxSizelHlayout)

		# Create a widget for edit and diffc button
		self.edit_button = QtWidgets.QToolButton()
		self.edit_button.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
		self.edit_button.setMinimumHeight(25) 
		self.edit_button.setMaximumHeight(30) 
		self.diffc_button = QtWidgets.QCheckBox(get_str('useDifficult'))
		self.diffc_button.setFont(dock_font)
		self.diffc_button.setChecked(False)
		self.diffc_button.stateChanged.connect(self.button_state)

		editLabelHlayout = QtWidgets.QHBoxLayout()
		editLabelHlayout.setContentsMargins(5, 5, 0, 10) 
		editLabelHlayout.addWidget(self.edit_button)
		editLabelHlayout.addWidget(self.diffc_button)
		use_edit_label_container = QtWidgets.QWidget()
		use_edit_label_container.setLayout(editLabelHlayout)

		# Create a widget for using default label
		self.use_default_label_checkbox = QtWidgets.QCheckBox(get_str('useDefaultLabel'))
		self.use_default_label_checkbox.setFont(dock_font)
		self.use_default_label_checkbox.setChecked(False)
		self.default_label_text_line = QtWidgets.QLineEdit()
		self.default_label_text_line.setMinimumHeight(10) 
		self.default_label_text_line.setMaximumHeight(30)  

		defaultLabelHlayout = QtWidgets.QHBoxLayout()
		defaultLabelHlayout.setContentsMargins(5, 5, 10, 5) 
		defaultLabelHlayout.addWidget(self.use_default_label_checkbox)
		defaultLabelHlayout.addWidget(self.default_label_text_line)
		use_default_label_container = QtWidgets.QWidget()
		use_default_label_container.setLayout(defaultLabelHlayout)

		# Create and add combobox for showing unique labels in group
		self.display_label = QtWidgets.QLabel("Display Label :")
		self.display_label.setFont(dock_font)
		self.combo_box = ComboBox(self) 
		self.combo_box.cb.setMinimumHeight(10) 
		self.combo_box.cb.setMaximumHeight(30) 

		displayLabelHlayout = QtWidgets.QHBoxLayout()
		displayLabelHlayout.setContentsMargins(5, 0, 0, 5) 
		displayLabelHlayout.addWidget(self.display_label)
		displayLabelHlayout.addWidget(self.combo_box, QtCore.Qt.AlignLeft)
		use_display_label_container = QtWidgets.QWidget()
		use_display_label_container.setLayout(displayLabelHlayout)

		# Create and add a widget for showing current label items
		label_menu = QtWidgets.QMenu()
		self.label_list = QtWidgets.QListWidget()
		self.label_list.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
		self.label_list.itemActivated.connect(self.label_selection_changed)
		self.label_list.itemSelectionChanged.connect(self.label_selection_changed)
		self.label_list.itemDoubleClicked.connect(self.edit_label)
		self.label_list.itemChanged.connect(self.label_item_changed)
		self.label_list.customContextMenuRequested.connect(self.pop_label_list_menu)

		# Add some of widgets to label_list_layout
		label_list_layout.addWidget(use_box_size_container)
		label_list_layout.addWidget(use_edit_label_container)
		label_list_layout.addWidget(use_default_label_container)
		label_list_layout.addWidget(use_display_label_container)
		label_list_layout.addWidget(self.label_list)
		label_list_container = QtWidgets.QWidget()
		label_list_container.setLayout(label_list_layout)

		self.label_dock = QtWidgets.QDockWidget(get_str('boxLabelText'), self)
		self.label_dock.setObjectName(get_str('labels'))
		self.label_dock.setWidget(label_list_container)
		self.label_dock.setFeatures(self.dock_features)

		# ==================================================
		#                     Model
		# ==================================================
		model_list_layout = QtWidgets.QVBoxLayout()
		model_list_layout.setContentsMargins(0, 0, 0, 0)

		# Create a widget for using auto model label
		mode_label = QtWidgets.QLabel("Mode : ")
		mode_label.setFont(dock_font)
		self.mode_btn = SwitchBtn("Online", "Offline")
		self.mode_btn.clicked.connect(self.auto_model_load)
		self.autolabel_btn = QtWidgets.QToolButton()
		self.autolabel_btn.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
		self.autolabel_btn.setMinimumHeight(10) 
		self.autolabel_btn.setMaximumHeight(30)  

		modelHLayout = QtWidgets.QHBoxLayout()
		modelHLayout.setContentsMargins(5, 10, 10, 10) 
		modelHLayout.addWidget(mode_label, alignment=QtCore.Qt.AlignLeft)
		modelHLayout.addWidget(self.mode_btn, alignment=QtCore.Qt.AlignLeft)
		modelHLayout.addStretch(1)
		modelHLayout.addWidget(self.autolabel_btn, alignment=QtCore.Qt.AlignRight)
		use_model_container = QtWidgets.QWidget()
		use_model_container.setLayout(modelHLayout)

		# Create a widget for shape selection (Polygon or Rectangle)
		shape_title_label = QtWidgets.QLabel("Shape Type: ")
		shape_title_label.setFont(dock_font)
		self.polygon_radio = QtWidgets.QRadioButton("Polygon")
		self.rectangle_radio = QtWidgets.QRadioButton("Rectangle")
		self.rectangle_radio.setChecked(True)  # Default to Rectangle
		self.polygon_radio.toggled.connect(self.auto_model_load)
		self.rectangle_radio.toggled.connect(self.auto_model_load)

		shapeHLayout = QtWidgets.QHBoxLayout()
		shapeHLayout.setContentsMargins(5, 0, 10, 0)
		shapeHLayout.addWidget(shape_title_label, alignment=QtCore.Qt.AlignLeft)
		shapeHLayout.addWidget(self.polygon_radio, alignment=QtCore.Qt.AlignLeft)
		shapeHLayout.addWidget(self.rectangle_radio, alignment=QtCore.Qt.AlignLeft)
		shapeHLayout.addStretch(1)
		use_shape_container = QtWidgets.QWidget()
		use_shape_container.setLayout(shapeHLayout)

		# Create a widget for overlap box
		overlap_title_label = QtWidgets.QLabel("Overlap Rate : ")
		overlap_title_label.setFont(dock_font)
		self.overlap_slider = QtWidgets.QSlider()
		self.overlap_slider.setObjectName('QSlider_overlap')
		self.overlap_slider.setRange(1, 100)
		self.overlap_slider.setOrientation(1)
		self.overlap_slider.setTickPosition(2) # Add tick marks below
		self.overlap_slider.setTickInterval(10) # Tick mark spacing (there will be 10 tick marks)
		self.overlap_slider.setValue(60)
		self.overlap_slider.valueChanged.connect(self.__sliderMonitor)
		self.overlap_label = QtWidgets.QLabel(str(self.overlap_slider.value()/100.0))

		overlapHLayout = QtWidgets.QHBoxLayout()
		overlapHLayout.setContentsMargins(5, 0, 10, 0) 
		overlapHLayout.addWidget(overlap_title_label)
		overlapHLayout.addWidget(self.overlap_slider)
		overlapHLayout.addWidget(self.overlap_label)
		use_overlap_container = QtWidgets.QWidget()
		use_overlap_container.setLayout(overlapHLayout)

		# Create a widget for thres
		confs_title_label = QtWidgets.QLabel("Confs Rate : ")
		confs_title_label.setFont(dock_font)
		self.confs_slider = QtWidgets.QSlider()
		self.confs_slider.setObjectName('QSlider_confs')
		self.confs_slider.setRange(30, 100)
		self.confs_slider.setOrientation(1)
		self.confs_slider.setTickPosition(2) # Add tick marks below
		self.confs_slider.setTickInterval(5) # Tick mark spacing (there will be 10 tick marks)
		self.confs_slider.setValue(30)
		self.confs_slider.valueChanged.connect(self.__sliderMonitor)
		self.confs_label = QtWidgets.QLabel(str(self.confs_slider.value()/100.0))

		confsHLayout = QtWidgets.QHBoxLayout()
		confsHLayout.setContentsMargins(5, 0, 10, 10) 
		confsHLayout.addWidget(confs_title_label)
		confsHLayout.addWidget(self.confs_slider)
		confsHLayout.addWidget(self.confs_label)
		use_confs_container = QtWidgets.QWidget()
		use_confs_container.setLayout(confsHLayout)
  
		model_list_layout.addWidget(use_model_container)
		model_list_layout.addWidget(use_shape_container)
		model_list_layout.addWidget(use_overlap_container)
		model_list_layout.addWidget(use_confs_container)
		model_list_layout.addStretch(1)
		model_list_container = QtWidgets.QWidget()
		model_list_container.setLayout(model_list_layout)

		self.model_dock = QtWidgets.QDockWidget("Auto Label", self)
		self.model_dock.setObjectName(get_str('labels'))
		self.model_dock.setWidget(model_list_container)
		self.model_dock.setFeatures(self.dock_features)
  
		# ==================================================
		#                   File List
		# ==================================================
		file_list_layout = QtWidgets.QVBoxLayout()
		file_list_layout.setContentsMargins(0, 0, 0, 0)

		# Create and add a widget for showing file items
		self.file_list_widget = QtWidgets.QListWidget()
		self.file_list_widget.itemDoubleClicked.connect(self.file_item_double_clicked)

		# Add some of widgets to file_list_layout
		file_list_layout.addWidget(self.file_list_widget)
		file_list_container = QtWidgets.QWidget()
		file_list_container.setLayout(file_list_layout)

		self.file_dock = QtWidgets.QDockWidget(get_str('fileList'), self)
		self.file_dock.setObjectName(get_str('files'))
		self.file_dock.setWidget(file_list_container)

		# ==================================================
		#                   Image Canvas
		# ==================================================
		self.zoom_widget = ZoomWidget()
		self.zoom_widget.setWhatsThis(
			u"Zoom in or out of the image. Also accessible with"
			" %s and %s from the canvas." % (format_shortcut("Ctrl+[-+]"),
											 format_shortcut("Ctrl+Wheel")))
		self.zoom_widget.setEnabled(False)
		self.zoom_widget.valueChanged.connect(self.paint_canvas)

		self.canvas = Canvas(parent=self)
		self.canvas.zoomRequest.connect(self.zoom_request)
		self.canvas.scrollRequest.connect(self.scroll_request)
		self.canvas.newShape.connect(self.new_shape)
		self.canvas.shapeMoved.connect(self.set_dirty)
		self.canvas.selectionChanged.connect(self.shape_selection_changed)
		self.canvas.drawingPolygon.connect(self.toggle_drawing_sensitive)
		self.canvas.set_drawing_free_shape(True)

		mainVLayout = QtWidgets.QVBoxLayout()
		mainVLayout.setContentsMargins(0, 2, 0, 0)
		mainVLayout.setSpacing(2)
		classHLayout = QtWidgets.QHBoxLayout()
		self.tagLabel = QtWidgets.QLabel("Label Tags : ")
		self.tagLabel.setStyleSheet("color : rgb(255, 255, 255);")
		self.tagLabel.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
		self.tagLabel.setMinimumHeight(5)
		classHLayout.addWidget(self.tagLabel)
		self.tagLineEdit = TagBar(self)
		self.tagLineEdit.load_tags(label_hist)
		self.tagLineEdit.setStyleSheet("margin: 0px; padding: 0px; background-color: rgb(27,29,35);  border-radius: 15px; color : rgb(200, 200, 200);" )
		classHLayout.addWidget(self.tagLineEdit)
		mainVLayout.addLayout(classHLayout)
  
		scroll = QtWidgets.QScrollArea()
		scroll.setWidget(self.canvas)
		scroll.setWidgetResizable(True)
		self.scroll_bars = {
			QtCore.Qt.Vertical: scroll.verticalScrollBar(),
			QtCore.Qt.Horizontal: scroll.horizontalScrollBar()
		}
		self.scroll_area = scroll
		
		mainVLayout.addWidget(scroll, 1)
		self.addDockWidget(QtCore.Qt.RightDockWidgetArea, self.label_dock)
		self.addDockWidget(QtCore.Qt.RightDockWidgetArea, self.model_dock)
		self.addDockWidget(QtCore.Qt.RightDockWidgetArea, self.file_dock)
		central_widget = QtWidgets.QWidget()
		central_widget.setLayout(mainVLayout)
		self.setCentralWidget(central_widget)
		# self.setCentralWidget(scroll)

		self.color_dialog = ColorDialog(parent=self)
		# ==================================================
		#                       Actions
		# ==================================================
		action = partial(new_action, self)
		edit = action(get_str('editLabel'), self.edit_label,
					  'Ctrl+E', 'edit', get_str('editLabelDetail'),
					  enabled=False, text_wrap=False)
		self.edit_button.setDefaultAction(edit)

		infer_model = action(get_str('autoLabel'), self.auto_label_infer,
					  'Ctrl+M', 'model', get_str('autoLabelDetail'),
					  enabled=False, text_wrap=False)
		self.autolabel_btn.setDefaultAction(infer_model)

		quit = action(get_str('quit'), self.close,
					  'Ctrl+Q', 'quit', get_str('quitApp'))

		open = action(get_str('openFile'), self.open_file,
					'Ctrl+O', 'open', get_str('openFileDetail'))

		open_dir = action(get_str('openDir'), self.open_dir_dialog,
						'Ctrl+u', 'open-dir', get_str('openDir'))

		change_save_dir = action(get_str('changeSaveDir'), self.change_save_dir_dialog,
								'Ctrl+r', 'save-dir', get_str('changeSavedAnnotationDir'))

		open_annotation = action(get_str('openAnnotation'), self.open_annotation_dialog,
								 'Ctrl+Shift+O', 'open', get_str('openAnnotationDetail'))

		copy_prev_bounding = action(get_str('copyPrevBounding'), self.copy_previous_bounding_boxes, 'Ctrl+v', 'copy', get_str('copyPrevBounding'))

		open_next_image = action(get_str('nextImg'), self.open_next_image,
								 'd', 'next', get_str('nextImgDetail'))

		open_prev_image = action(get_str('prevImg'), self.open_prev_image,
								 'a', 'prev', get_str('prevImgDetail'))

		verify = action(get_str('verifyImg'), self.verify_image,
						'space', 'verify', get_str('verifyImgDetail'))

		thumbnail = action(get_str('thumbnailImg'), self.thumbnail_image,
						'Ctrl+T', 'thumbnail', get_str('thumbnailImgDetail'))

		save = action(get_str('save'), self.save_file,
					  'Ctrl+S', 'save', get_str('saveDetail'), enabled=False)

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
							 self.change_format, 'Ctrl+',
							 get_format_meta(self.label_file_format)[1],
							 get_str('changeSaveFormat'), enabled=True)

		save_as = action(get_str('saveAs'), self.save_file_as,
						 'Ctrl+Shift+S', 'save-as', get_str('saveAsDetail'), enabled=False)

		close = action(get_str('closeCur'), self.close_file, 'Ctrl+W', 'close', get_str('closeCurDetail'))

		delete_image = action(get_str('deleteImg'), self.delete_image, 'Ctrl+Shift+D', 'close', get_str('deleteImgDetail'))

		reset_all = action(get_str('resetAll'), self.reset_all, None, 'resetall', get_str('resetAllDetail'))

		color1 = action(get_str('boxLineColor'), self.choose_color1,
						'Ctrl+L', 'color_line', get_str('boxLineColorDetail'))

		create_mode = action(get_str('crtBox'), self.set_create_mode,
							 'w', 'new', get_str('crtBoxDetail'), enabled=False)
		edit_mode = action(get_str('editBox'), self.set_edit_mode,
						   'Ctrl+J', 'edit', get_str('editBoxDetail'), enabled=False)

		create_rect = action(get_str('crtBox'), partial(self.create_shape, "rectangle"),
						'w', 'rect', get_str('crtBoxDetail'), enabled=False)
		create_poly = action(get_str('crtPoly'), partial(self.create_shape, "polygon"),
						'e', 'poly', get_str('crtPolyDetail'), enabled=False)
		delete = action(get_str('delObj'), self.delete_selected_shape,
						'Delete', 'delete', get_str('delObjDetail'), enabled=False)
		copy = action(get_str('dupObj'), self.copy_selected_shape,
					  'Ctrl+C', 'copy', get_str('dupObjDetail'),
					  enabled=False)

		advanced_mode = action(get_str('advancedMode'), self.toggle_advanced_mode,
							   'Ctrl+Shift+A', 'expert', get_str('advancedModeDetail'),
							   checkable=True)

		hide_all = action(get_str('hideAllBox'), partial(self.toggle_polygons, False),
						  'Ctrl+H', 'hide', get_str('hideAllBoxDetail'),
						  enabled=False)
		show_all = action(get_str('showAllBox'), partial(self.toggle_polygons, True),
						  'Ctrl+A', 'hide', get_str('showAllBoxDetail'),
						  enabled=False)

		show_tutorial = action(get_str('tutorialDefault'), lambda: self.show_dialog('tutorial'), 
							None, 'help', get_str('tutorialDetail'))
		show_info = action(get_str('info'), lambda: self.show_dialog('info'), 
						None, 'help', get_str('info'))
		show_shortcut = action(get_str('shortcut'), lambda: self.show_dialog('shortcut'), 
							None, 'help', get_str('shortcut'))
		show_modeldir = action(get_str('modelDir'), lambda: self.show_dialog('modeldir'), 
							None, 'help', get_str('modelDir'))
		
		zoom = QtWidgets.QWidgetAction(self)
		zoom.setDefaultWidget(self.zoom_widget)

		zoom_in = action(get_str('zoomin'), partial(self.add_zoom, 10),
						 'Ctrl++', 'zoom-in', get_str('zoominDetail'), enabled=False)
		zoom_out = action(get_str('zoomout'), partial(self.add_zoom, -10),
						  'Ctrl+-', 'zoom-out', get_str('zoomoutDetail'), enabled=False)
		zoom_org = action(get_str('originalsize'), partial(self.set_zoom, 100),
						  'Ctrl+=', 'zoom', get_str('originalsizeDetail'), enabled=False)
		fit_window = action(get_str('fitWin'), self.set_fit_window,
							'Ctrl+F', 'fit-window', get_str('fitWinDetail'),
							checkable=True, enabled=False)
		fit_width = action(get_str('fitWidth'), self.set_fit_width,
						   'Ctrl+Shift+F', 'fit-width', get_str('fitWidthDetail'),
						   checkable=True, enabled=False)
		# Group zoom controls into a list for easier toggling.
		zoom_actions = (self.zoom_widget, zoom_in, zoom_out,
						zoom_org, fit_window, fit_width)
		self.zoom_mode = self.FIT_WINDOW
		self.scalers = {
			self.FIT_WINDOW: self.scale_fit_window,
			self.FIT_WIDTH: self.scale_fit_width,
			# Set to one to scale to 100% when loading files.
			self.MANUAL_ZOOM: lambda: 1,
		}

		shape_line_color = action(get_str('shapeLineColor'), self.choose_shape_line_color,
								  icon='color_line', tip=get_str('shapeLineColorDetail'),
								  enabled=False)
		shape_fill_color = action(get_str('shapeFillColor'), self.choose_shape_fill_color,
								  icon='color', tip=get_str('shapeFillColorDetail'),
								  enabled=False)

		# Store actions for further handling.
		self.actions = Struct(save=save, save_format=save_format, saveAs=save_as, open=open, close=close, resetAll=reset_all, deleteImg=delete_image,
							  lineColor=color1, delete=delete, edit=edit, copy=copy,
							  createMode=create_mode, editMode=edit_mode, advancedMode=advanced_mode,
							  shapeLineColor=shape_line_color, shapeFillColor=shape_fill_color,
							  zoom=zoom, zoomIn=zoom_in, zoomOut=zoom_out, zoomOrg=zoom_org,
							  fitWindow=fit_window, fitWidth=fit_width,
							  zoomActions=zoom_actions,
							  create=(create_rect, create_poly), 
							  fileMenuActions=(
								  open, open_dir, save, save_as, close, reset_all, quit),
							  beginner=(), advanced=(),
							  editMenu=(edit, copy, delete, None, color1),
							  beginnerContext=(create_rect, create_poly, edit, copy, delete),
							  advancedContext=(create_mode, edit_mode, edit, copy,
											   delete, shape_line_color, shape_fill_color),
							  onLoadActive=(close, create_rect, create_poly, infer_model, create_mode, edit_mode),
							  onShapesPresent=(save_as, hide_all, show_all))

		self.menus = Struct(
			file=self.menu(get_str('menu_file')),
			edit=self.menu(get_str('menu_edit')),
			view=self.menu(get_str('menu_view')),
			help=self.menu(get_str('menu_help')),
			recentFiles=QtWidgets.QMenu(get_str('menu_openRecent')),
			labelList=label_menu)
		add_actions(label_menu, (edit, delete))
		self.menus.file.aboutToShow.connect(self.update_file_menu)

		# Auto saving : Enable auto saving if pressing next
		self.auto_saving = QtWidgets.QAction(get_str('autoSaveMode'), self)
		self.auto_saving.setCheckable(True)
		self.auto_saving.setChecked(settings.get(SETTING_AUTO_SAVE, False))
		# Sync single class mode from PR#106
		# self.single_class_mode = QtWidgets.QAction(get_str('singleClsMode'), self)
		# self.single_class_mode.setShortcut("Ctrl+Shift+S")
		# self.single_class_mode.setCheckable(True)
		# self.single_class_mode.setChecked(settings.get(SETTING_SINGLE_CLASS, False))
		# self.lastLabel = None
		# Add option to enable/disable labels being displayed at the top of bounding boxes
		self.display_label_option = QtWidgets.QAction(get_str('displayLabel'), self)
		self.display_label_option.setShortcut("Ctrl+Shift+P")
		self.display_label_option.setCheckable(True)
		self.display_label_option.setChecked(settings.get(SETTING_PAINT_LABEL, False))
		self.display_label_option.triggered.connect(self.toggle_paint_labels_option)
		# Draw squares/rectangles
		self.draw_rectangles_option = QtWidgets.QAction(get_str('drawRectangles'), self)
		self.draw_rectangles_option.setCheckable(True)
		self.draw_rectangles_option.setShortcut('Ctrl+Shift+R')
		self.draw_rectangles_option.setChecked(settings.get(SETTING_DRAW_SQUARE, False))
		self.draw_rectangles_option.triggered.connect(self.toggle_draw_rectangles)
		# Show/Hide Label Panel
		labels = self.label_dock.toggleViewAction()
		labels.setText(get_str('showHide'))
		labels.setShortcut('Ctrl+Shift+L')
		labels.setCheckable(True)

		if (frozen_path) :
			add_actions(self.menus.file,
						(open_annotation, copy_prev_bounding, self.menus.recentFiles, save, save_format, save_as, close, reset_all, delete_image, quit))
		else :
			add_actions(self.menus.file,
						(open, open_dir, open_annotation, change_save_dir, copy_prev_bounding, self.menus.recentFiles, save, save_format, save_as, close, reset_all, delete_image, quit))
		add_actions(self.menus.help, (show_tutorial, show_info, show_shortcut, show_modeldir))
		add_actions(self.menus.view, (
			self.auto_saving,
			# self.single_class_mode,
			self.display_label_option,
			self.draw_rectangles_option,
			labels, advanced_mode, None,
			hide_all, show_all, None,
			zoom_in, zoom_out, zoom_org, None,
			fit_window, fit_width))

		# Custom context menu for the canvas widget:
		add_actions(self.canvas.menus[0], self.actions.beginnerContext)
		add_actions(self.canvas.menus[1], (
			action('&Copy here', self.copy_shape),
			action('&Move here', self.move_shape)))

		self.tools = self.toolbar('Tools')
		if (frozen_path) :
			self.actions.beginner = (
				open_next_image, open_prev_image, verify, thumbnail, save, save_format, None, create_rect, create_poly, copy, delete, None,
				zoom, zoom_in, zoom_out, fit_window, fit_width)

			self.actions.advanced = (
				open_next_image, open_prev_image, save, save_format, None,
				create_mode, edit_mode, None,
				hide_all, show_all)
		else :
			self.actions.beginner = (
				open, open_dir, change_save_dir, open_next_image, open_prev_image, verify, thumbnail, save, save_format, None, create_rect, create_poly, copy, delete, None,
				zoom, zoom_in, zoom_out, fit_window, fit_width)

			self.actions.advanced = (
				open, open_dir, change_save_dir, open_next_image, open_prev_image, save, save_format, None,
				create_mode, edit_mode, None,
				hide_all, show_all)

		self.statusBar().showMessage('%s started.' % APPNAME)
		self.statusBar().setStyleSheet("QStatusBar{padding-left:8px;background:rgba(86,104,118,255);color:black;font-weight:bold;}")
		self.statusBar().show()

		# Application state.
		self.image = QtGui.QImage()
		self.file_path = ustr(default_filename)
		self.last_open_dir = None
		self.recent_files = []
		self.max_recent = 7
		self.line_color = None
		self.fill_color = None
		self.zoom_level = 100
		self.fit_window = False

		# Fix the compatible issue for qt4 and qt5. Convert the QStringList to python list
		if settings.get(SETTING_RECENT_FILES):
			if have_qstring():
				recent_file_qstring_list = settings.get(SETTING_RECENT_FILES)
				self.recent_files = [ustr(i) for i in recent_file_qstring_list]
			else:
				self.recent_files = recent_file_qstring_list = settings.get(SETTING_RECENT_FILES)

		size = settings.get(SETTING_WIN_SIZE, QtCore.QSize(600, 500))
		position = QtCore.QPoint(0, 0)
		saved_position = settings.get(SETTING_WIN_POSE, position)
		# Fix the multiple monitors issue
		for i in range(QtWidgets.QApplication.desktop().screenCount()):
			if QtWidgets.QApplication.desktop().availableGeometry(i).contains(saved_position):
				position = saved_position
				break
		self.resize(size)
		self.move(position)
		save_dir = ustr(settings.get(SETTING_SAVE_DIR, None))
		self.last_open_dir = ustr(settings.get(SETTING_LAST_OPEN_DIR, None))
		if self.default_save_dir is None and save_dir is not None and os.path.exists(save_dir):
			self.default_save_dir = save_dir
			self.statusBar().showMessage('%s started. Annotation will be saved to %s' %
										 (APPNAME, self.default_save_dir))
			self.statusBar().show()
		self.restoreState(settings.get(SETTING_WIN_STATE, QtCore.QByteArray()))
		Shape.line_color = self.line_color = QtGui.QColor(settings.get(SETTING_LINE_COLOR, DEFAULT_LINE_COLOR))
		Shape.fill_color = self.fill_color = QtGui.QColor(settings.get(SETTING_FILL_COLOR, DEFAULT_FILL_COLOR))
		self.canvas.set_drawing_color(self.line_color)
		# Add chris
		Shape.difficult = False

		def xbool(x):
			if isinstance(x, QtCore.QVariant):
				return x.toBool()
			return bool(x)

		if xbool(settings.get(SETTING_ADVANCE_MODE, False)):
			self.actions.advancedMode.setChecked(True)
			self.toggle_advanced_mode()

		# Populate the File menu dynamically.
		self.update_file_menu()

		# Since loading the file may take some time, make sure it runs in the background.
		if self.file_path and os.path.isdir(self.file_path):
			self.queue_event(partial(self.import_dir_images, self.file_path or ""))
		elif self.file_path:
			self.queue_event(partial(self.load_file, self.file_path or ""))

		self.populate_mode_actions()

		# Display cursor coordinates at the right of status bar
		self.label_coordinates = QtWidgets.QLabel('')
		self.statusBar().addPermanentWidget(self.label_coordinates)

		# Open Dir if default file
		if self.file_path and os.path.isdir(self.file_path):
			self.open_dir_dialog(dir_path=self.file_path, silent=True)

	def set_qdarkstyle(self):
		self.qdarkstyle = True
		self.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())

	def setDebugLevel(self, level) :
		self.debug.changelevel(level)
		self.debug.debug("Change [ %s ] debug level." % APPNAME )

	def __sliderMonitor(self):
		sendingBtn = self.sender()
		if (sendingBtn.objectName() == "QSlider_overlap") :
			self.overlap_label.setText(str(self.overlap_slider.value()/100.0))

		if (sendingBtn.objectName() =="QSlider_confs") :
			self.confs_label.setText(str(self.confs_slider.value()/100.0))

	# Support Functions #
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
		self.set_dirty()

	def no_shapes(self):
		return not self.items_to_shapes

	def toggle_advanced_mode(self, value=True):
		self._beginner = not value
		self.canvas.set_editing(True)
		self.populate_mode_actions()
		self.edit_button.setVisible(not value)
		if value:
			self.actions.createMode.setEnabled(True)
			self.actions.editMode.setEnabled(False)
			self.label_dock.setFeatures(self.label_dock.features() | self.dock_features)
		else:
			self.label_dock.setFeatures(self.label_dock.features() ^ self.dock_features)

	def populate_mode_actions(self):
		if self.beginner():
			tool, menu = self.actions.beginner, self.actions.beginnerContext
		else:
			tool, menu = self.actions.advanced, self.actions.advancedContext
		self.tools.clear()
		add_actions(self.tools, tool)
		self.canvas.menus[0].clear()
		add_actions(self.canvas.menus[0], menu)
		self.menus.edit.clear()
		actions = (*self.actions.create,) if self.beginner()\
			else (self.actions.createMode, self.actions.editMode)
		add_actions(self.menus.edit, actions + self.actions.editMenu)

	def set_beginner(self):
		self.tools.clear()
		add_actions(self.tools, self.actions.beginner)

	def set_advanced(self):
		self.tools.clear()
		add_actions(self.tools, self.actions.advanced)

	def set_dirty(self):
		self.dirty = True
		self.actions.save.setEnabled(True)

	def set_clean(self):
		self.dirty = False
		self.actions.save.setEnabled(False)
		for create in self.actions.create:
			create.setEnabled(True)
	
	def toggle_actions(self, value=True):
		"""Enable/Disable widgets which depend on an opened image."""
		for z in self.actions.zoomActions:
			z.setEnabled(value)
		for action in self.actions.onLoadActive:
			action.setEnabled(value)

	def queue_event(self, function):
		QtCore.QTimer.singleShot(0, function)

	def status(self, message, delay=5000):
		self.statusBar().showMessage(message, delay)

	def reset_state(self):
		self.items_to_shapes.clear()
		self.shapes_to_items.clear()
		self.label_list.clear()
		self.file_path = None
		self.image_data = None
		self.label_file = None
		self.canvas.reset_state()
		self.label_coordinates.clear()
		self.combo_box.current_item()
		self.combo_box.cb.clear()

	def current_item(self):
		items = self.label_list.selectedItems()
		if items:
			return items[0]
		return None

	def add_recent_file(self, file_path):
		if file_path in self.recent_files:
			self.recent_files.remove(file_path)
		elif len(self.recent_files) >= self.max_recent:
			self.recent_files.pop()
		self.recent_files.insert(0, file_path)

	def beginner(self):
		return self._beginner

	def advanced(self):
		return not self.beginner()

	def show_dialog(self, name):

		def show_tutorial_dialog(browser='default', link=self.screencast):
			if browser.lower() == 'default':
				wb.open(link, new=2)
			elif browser.lower() == 'chrome' and platform.system() == 'Windows':
				if shutil.which(browser.lower()):  # 'chrome' not in wb._browsers in windows
					wb.register('chrome', None, wb.BackgroundBrowser('chrome'))
				else:
					chrome_path="D:\\Program Files (x86)\\Google\\Chrome\\Application\\chrome.exe"
					if os.path.isfile(chrome_path):
						wb.register('chrome', None, wb.BackgroundBrowser(chrome_path))
				try:
					wb.get('chrome').open(link, new=2)
				except:
					wb.open(link, new=2)
			elif browser.lower() in wb._browsers:
				wb.get(browser.lower()).open(link, new=2)

		if name == 'tutorial':
			show_tutorial_dialog(browser='default')
		elif name == 'info':
			from modules.labeling.libs.__init__ import __version__
			msg = u'Name:{0} \nApp Version:{1} \n{2} '.format(APPNAME, __version__, sys.version_info)
			QtWidgets.QMessageBox.information(self, u'Information', msg)
		elif name == 'shortcut':
			show_tutorial_dialog(browser='default', link='http://gitlab.mirle.com.tw/9b0/9bf/annotator')
		elif name == "modeldir":
			mdoel_dir = MODEL_DIR
			if os.name == 'nt':  # Windows
				subprocess.call(['start', mdoel_dir], shell=True)
			elif os.name == 'posix':
				if sys.platform == 'darwin':  # macOS
					subprocess.call(['open', mdoel_dir])
				else:  # Linux
					subprocess.call(['xdg-open', mdoel_dir])
			else:
				print("Unsupported OS")
		else:
			self.status(f"Unknown action: {name}")

	def create_shape(self, shape_type):
		assert self.beginner()
		self.canvas._create_mode = shape_type
		self.canvas.set_editing(False)
		for create in self.actions.create:
			create.setEnabled(False)

	def toggle_drawing_sensitive(self, drawing=True):
		"""In the middle of drawing, toggling between modes should be disabled."""
		self.actions.editMode.setEnabled(not drawing)
		if not drawing and self.beginner():
			# Cancel creation.
			self.debug.war('Cancel creation rect.')
			self.canvas.set_editing(True)
			self.canvas.restore_cursor()
			for create in self.actions.create:
				create.setEnabled(True)

	def toggle_draw_mode(self, edit=True):
		self.canvas.set_editing(edit)
		for create in self.actions.create:
			create.setEnabled(edit)
		self.actions.editMode.setEnabled(not edit)

	def set_create_mode(self):
		assert self.advanced()
		self.toggle_draw_mode(False)

	def set_edit_mode(self):
		assert self.advanced()
		self.toggle_draw_mode(True)
		self.label_selection_changed()

	def update_file_menu(self):
		curr_file_path = self.file_path

		def exists(filename):
			return os.path.exists(filename)
		menu = self.menus.recentFiles
		menu.clear()
		files = [f for f in self.recent_files if f !=
				 curr_file_path and exists(f)]
		for i, f in enumerate(files):
			icon = new_icon('labels')
			action = QtWidgets.QAction(
				icon, '&%d %s' % (i + 1, QtCore.QFileInfo(f).fileName()), self)
			action.triggered.connect(partial(self.load_recent, f))
			menu.addAction(action)

	def pop_label_list_menu(self, point):
		self.menus.labelList.exec_(self.label_list.mapToGlobal(point))

	def edit_label(self):
		if not self.canvas.editing():
			return
		item = self.current_item()
		if not item:
			return
		text = self.label_dialog.pop_up(item.text())
		if text is not None:
			item.setText(text)
			item.setBackground(generate_color_by_text(text))
			self.set_dirty()
			self.update_combo_box()

	def auto_model_load(self):
		shape_type = "polygon" if self.polygon_radio.isChecked() else "rectangle"
		if hasattr(self, "_load_thread") and self._load_thread.isRunning():
			self._load_thread.quit()
			self._load_thread.wait()

		self._load_thread = CreateThread(self.mode_btn.label, shape_type, self)
		self._load_thread.message.connect(self.status)
		self._load_thread.createLoaded.connect(self._load_thread._loading)
		self._load_thread.createFinished.connect(InferenceThread._loading)
		self._load_thread.start()

	def auto_label_infer(self):
		if self.file_path != None:
			prompts = dict(image=self.file_path, prompt='.'.join(self.tagLineEdit.tags))
			self._infer_thread = InferenceThread(self.mode_btn.label, prompts, self)
			self._infer_thread.message.connect(self.status)
			self._infer_thread.inferenceFinished.connect(self.auto_label_result)
			self._infer_thread.start()

	def auto_label_result(self, data):

		def format_shape(s):
			return [s.label, s.shape_type, [(p.x(), p.y()) for p in s.points], s.line_color.getRgb(), s.fill_color.getRgb(), s.difficult]

		def similarity(box1, box2) -> float:
			# Calculate the upper left and lower right coordinates of the intersection
			x1 = max(box1[0][0], box2[0][0])
			y1 = max(box1[1][1], box2[1][1])
			x2 = min(box1[2][0], box2[2][0])
			y2 = min(box1[2][1], box2[2][1])
			w1, h1 = (box1[2][0] - box1[0][0]), (box1[2][1] - box1[0][1])
			w2, h2 = (box2[2][0] - box2[0][0]), (box2[2][1] - box2[0][1])
			# Calculate intersection area
			intersection_area = max(0, x2 - x1 + 1) * max(0, y2 - y1 + 1)
			
			# Calculate the area of two boxes
			box1_area = (w1 + 1) * (h1 + 1)
			box2_area = (w2 + 1) * (h2 + 1)
			
			return intersection_area / float(box1_area + box2_area - intersection_area)
		if "error" not in data.keys():
			shapes = [format_shape(shape) for shape in self.canvas.shapes]

			for _score, _shape in zip(data['scores'], data['shapes']):
				if _score < float(self.confs_label.text()):
					continue
				new_shape = format_shape(_shape)

				# Determine whether the new shape is similar to an existing shape
				similar_shapes = [shape for shape in shapes if similarity(shape[2], new_shape[2]) > float(self.overlap_label.text())]
				if not similar_shapes:
					shapes.append(new_shape)
			self.label_list.clear()
			self.load_labels(shapes)
			self.set_clean()
			self.set_dirty()
		else:
			self.error_message(u'Error Auto-Label', data['error'])

	# Tzutalin 20160906 : Add file list and dock to move faster
	def file_item_double_clicked(self, item=None):
		self.cur_img_idx = self.m_img_list.index(ustr(item.text()))
		filename = self.m_img_list[self.cur_img_idx]
		if filename:
			self.load_file(filename)

	# Add chris
	def button_state(self, item=None):
		""" Function to handle difficult examples
		Update on each object """
		if not self.canvas.editing():
			return

		item = self.current_item()
		if not item:  # If not selected Item, take the first one
			item = self.label_list.item(self.label_list.count() - 1)

		difficult = self.diffc_button.isChecked()

		try:
			shape = self.items_to_shapes[item]
		except:
			pass
		# Checked and Update
		try:
			if difficult != shape.difficult:
				shape.difficult = difficult
				self.set_dirty()
			else:  # User probably changed item visibility
				self.canvas.set_shape_visible(shape, item.checkState() == QtCore.Qt.Checked)
		except:
			pass

	# React to canvas signals.
	def shape_selection_changed(self, selected=False):
		if self._no_selection_slot:
			self._no_selection_slot = False
		else:
			shape = self.canvas.selected_shape
			if shape:
				self.shapes_to_items[shape].setSelected(True)
			else:
				self.label_list.clearSelection()
		self.actions.delete.setEnabled(selected)
		self.actions.copy.setEnabled(selected)
		self.actions.edit.setEnabled(selected)
		self.actions.shapeLineColor.setEnabled(selected)
		self.actions.shapeFillColor.setEnabled(selected)

	def add_label(self, shape):
		if shape is None:
			return
		shape.paint_label = self.display_label_option.isChecked()
		item = HashableQListWidgetItem(shape.label)
		item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
		item.setCheckState(QtCore.Qt.Checked)
		item.setBackground(generate_color_by_text(shape.label))
		self.items_to_shapes[item] = shape
		self.shapes_to_items[shape] = item
		self.label_list.addItem(item)
		for action in self.actions.onShapesPresent:
			action.setEnabled(True)
		if (self.combo_box.cb.findText(shape.label) < 0) :
			self.update_combo_box()

	def remove_label(self, shape):
		if shape is None:
			# print('rm empty label')
			return
		item = self.shapes_to_items[shape]
		self.label_list.takeItem(self.label_list.row(item))
		del self.shapes_to_items[shape]
		del self.items_to_shapes[item]
		self.update_combo_box()

	def load_labels(self, shapes):
		s = []
		for label, type, points, line_color, fill_color, difficult in shapes:
			shape = Shape(label=label, shape_type=type)
			for x, y in points:

				# Ensure the labels are within the bounds of the image. If not, fix them.
				x, y, snapped = self.canvas.snap_point_to_canvas(x, y)
				if snapped:
					self.set_dirty()

				shape.add_point(QtCore.QPointF(x, y))
			shape.difficult = difficult
			shape.close()
			s.append(shape)

			if line_color:
				shape.line_color = QtGui.QColor(*line_color)
			else:
				shape.line_color = generate_color_by_text(label, alpha=150)
			
			if fill_color:
				shape.fill_color = QtGui.QColor(*fill_color)
			else:
				shape.fill_color = generate_color_by_text(label)
			self.add_label(shape)
		self.update_combo_box()
		self.canvas.load_shapes(s)

	def update_combo_box(self):
		# Get the unique labels and add them to the Combobox.
		items_text_list = [str(self.label_list.item(i).text()) for i in range(self.label_list.count())]
		unique_text_list = list(set(items_text_list))
		# Add a null row for showing all the labels
		unique_text_list.sort()
		unique_text_list.insert(0, "All")
		self.combo_box.update_items(unique_text_list)

	def save_labels(self, annotation_file_path):
		annotation_file_path = ustr(annotation_file_path)
		if self.label_file is None:
			self.label_file = LabelFile()
			self.label_file.verified = self.canvas.verified

		def format_shape(s):
			return dict(label=s.label,
						line_color=s.line_color.getRgb(),
						fill_color=s.fill_color.getRgb(),
						points=[(p.x(), p.y()) for p in s.points],
						type=s.shape_type,
						# add chris
						difficult=s.difficult)

		shapes = [format_shape(shape) for shape in self.canvas.shapes]
		# Can add different annotation formats here
		try:
			if self.label_file_format == LabelFileFormat.PASCAL_VOC:
				if annotation_file_path[-4:].lower() != ".xml":
					annotation_file_path += XML_EXT
				self.label_file.save_pascal_voc_format(annotation_file_path, shapes, self.file_path, self.image_data,
													   self.line_color.getRgb(), self.fill_color.getRgb())
			elif self.label_file_format == LabelFileFormat.YOLO:
				if annotation_file_path[-4:].lower() != ".txt":
					annotation_file_path += TXT_EXT
				self.label_file.save_yolo_format(annotation_file_path, shapes, self.file_path, self.image_data, self.tagLineEdit.tags,
												 self.line_color.getRgb(), self.fill_color.getRgb())
			elif self.label_file_format == LabelFileFormat.CREATE_ML:
				if annotation_file_path[-5:].lower() != ".json":
					annotation_file_path += JSON_EXT
				self.label_file.save_create_ml_format(annotation_file_path, shapes, self.file_path, self.image_data,
													  self.tagLineEdit.tags, self.line_color.getRgb(), self.fill_color.getRgb())
			else:
				self.label_file.save(annotation_file_path, shapes, self.file_path, self.image_data,
									 self.line_color.getRgb(), self.fill_color.getRgb())
			self.debug.info('Image:{0} -> Annotation:{1}'.format(self.file_path, annotation_file_path))
			self.update_annotation_status(annotation_file_path)
			return True
		except LabelFileError as e:
			self.error_message(u'Error saving label data', u'<b>%s</b>' % e)
			return False

	def copy_selected_shape(self):
		self.add_label(self.canvas.copy_selected_shape())
		# fix copy and delete
		self.shape_selection_changed(True)
		self.set_dirty()
		
	def combo_selection_changed(self, index):
		text = self.combo_box.cb.itemText(index)
		items = self.label_list.findItems(text, QtCore.Qt.MatchFixedString | QtCore.Qt.MatchCaseSensitive)
		if(items):
			self.combo_box.current_item()
			self.label_list.setCurrentItem(items[0])

		for i in range(self.label_list.count()):
			if text == "All":
				self.label_list.item(i).setCheckState(2)
			elif text != self.label_list.item(i).text():
				self.label_list.item(i).setCheckState(0)
			else:
				self.label_list.item(i).setCheckState(2)

	def label_selection_changed(self):
		item = self.current_item()
		if item and self.canvas.editing():
			self._no_selection_slot = True
			if (self.label_list.currentItem() != None) :
				if (self.label_list.currentItem().text() == self.combo_box.status or self.combo_box.status == "All") :
					self.canvas.select_shape(self.items_to_shapes[item])
			shape = self.items_to_shapes[item]
			# Add Chris
			self.diffc_button.setChecked(shape.difficult)

	def label_item_changed(self, item):
		shape = self.items_to_shapes[item]
		label = item.text()
		if label != shape.label:
			shape.label = item.text()
			shape.line_color = generate_color_by_text(shape.label)
			self.set_dirty()
		else:  # User probably changed item visibility
			self.canvas.set_shape_visible(shape, item.checkState() == QtCore.Qt.Checked)

	# Callback functions:
	def new_shape(self):
		"""Pop-up and give focus to the label editor.

		position MUST be in global coordinates.
		"""
		if not self.use_default_label_checkbox.isChecked() or not self.default_label_text_line.text():
			if len(self.tagLineEdit.tags) > 0:
				self.label_dialog = LabelDialog(
					parent=self, list_item=self.tagLineEdit.tags)
				self.label_dialog.edit.hide()
				self.label_dialog.button_box.hide()
		
			# Sync single class mode from PR#106
			# if self.single_class_mode.isChecked() and self.lastLabel:
			#     text = self.lastLabel
			# else:
			text = self.label_dialog.pop_up(text=self.prev_label_text)
			self.lastLabel = text
		else:
			text = self.default_label_text_line.text()

		# Add Chris
		self.diffc_button.setChecked(False)
		if text is not None:
			self.prev_label_text = text
			generate_color = generate_color_by_text(text)
			shape = self.canvas.set_last_label(text, generate_color, generate_color)
			self.add_label(shape)
			if self.beginner():  # Switch to edit mode.
				self.canvas.set_editing(True)
				for create in self.actions.create:
					create.setEnabled(True)
			else:
				self.actions.editMode.setEnabled(True)
			self.set_dirty()
		else:
			# self.canvas.undoLastLine()
			self.canvas.reset_all_lines()

	def scroll_request(self, delta, orientation):
		units = -delta * 0.1  # natural scroll
		scroll_bar = self.scroll_bars[orientation]
		value = scroll_bar.value() + scroll_bar.singleStep() * units
		self.set_scroll(orientation, value)

	def set_scroll(self, orientation, value):
		self.scroll_bars[orientation].setValue(round(value))

	def set_zoom(self, value):
		self.actions.fitWidth.setChecked(False)
		self.actions.fitWindow.setChecked(False)
		self.zoom_mode = self.MANUAL_ZOOM
		self.zoom_widget.setValue(value)

	def add_zoom(self, increment=1.1):
		zoom_value = self.zoom_widget.value() * increment
		if increment > 1:
			zoom_value = math.ceil(zoom_value)
		else:
			zoom_value = math.floor(zoom_value)
		self.set_zoom(zoom_value)

	def zoom_request(self, delta, pos):
		canvas_width_old = self.canvas.width()
		units = 1.1
		if delta < 0:
			units = 0.9
		self.add_zoom(units)

		canvas_width_new = self.canvas.width()
		if canvas_width_old != canvas_width_new:
			canvas_scale_factor = canvas_width_new / canvas_width_old

			x_shift = round(pos.x() * canvas_scale_factor - pos.x())
			y_shift = round(pos.y() * canvas_scale_factor - pos.y())

			self.set_scroll(
				QtCore.Qt.Horizontal,
				self.scroll_bars[QtCore.Qt.Horizontal].value() + x_shift,
			)
			self.set_scroll(
				QtCore.Qt.Vertical,
				self.scroll_bars[QtCore.Qt.Vertical].value() + y_shift,
			)

	def set_fit_window(self, value=True):
		if value:
			self.actions.fitWidth.setChecked(False)
		self.zoom_mode = self.FIT_WINDOW if value else self.MANUAL_ZOOM
		self.adjust_scale()

	def set_fit_width(self, value=True):
		if value:
			self.actions.fitWindow.setChecked(False)
		self.zoom_mode = self.FIT_WIDTH if value else self.MANUAL_ZOOM
		self.adjust_scale()

	def toggle_polygons(self, value):
		for item, shape in self.items_to_shapes.items():
			item.setCheckState(QtCore.Qt.Checked if value else QtCore.Qt.Unchecked)

	def load_file(self, file_path=None):
		"""Load the specified file, or the last opened file if None."""
		self.reset_state()
		self.canvas.setEnabled(False)
		if file_path is None:
			file_path = self.settings.get(SETTING_FILENAME)

		# Make sure that filePath is a regular python string, rather than QString
		file_path = ustr(file_path)

		# Fix bug: An  index error after select a directory when open a new file.
		unicode_file_path = ustr(file_path)
		unicode_file_path = os.path.abspath(unicode_file_path)
		# Tzutalin 20160906 : Add file list and dock to move faster
		# Highlight the file item
		if unicode_file_path and self.file_list_widget.count() > 0:
			if unicode_file_path in self.m_img_list:
				index = self.m_img_list.index(unicode_file_path)
				file_widget_item = self.file_list_widget.item(index)
				file_widget_item.setSelected(True)
			else:
				self.file_list_widget.clear()
				self.m_img_list.clear()

		if unicode_file_path and os.path.exists(unicode_file_path):
			if LabelFile.is_label_file(unicode_file_path):
				try:
					self.label_file = LabelFile(unicode_file_path)
				except LabelFileError as e:
					self.error_message(u'Error opening file',
									   (u"<p><b>%s</b></p>"
										u"<p>Make sure <i>%s</i> is a valid label file.")
									   % (e, unicode_file_path))
					self.status("Error reading %s" % unicode_file_path)
					return False
				self.image_data = self.label_file.image_data
				self.line_color = QtGui.QColor(*self.label_file.lineColor)
				self.fill_color = QtGui.QColor(*self.label_file.fillColor)
				self.canvas.verified = self.label_file.verified
			else:
				# Load image:
				# read data first and store for saving into label file.
				def read(filename, default=None):
					try:
						reader =  QtGui.QImageReader(filename)
						reader.setAutoTransform(True)
						return reader.read()
					except:
						return default
				self.image_data = read(unicode_file_path, None)
				self.label_file = None
				self.canvas.verified = False

			if isinstance(self.image_data, QtGui.QImage):
				image = self.image_data
			else:
				image = QtGui.QImage.fromData(self.image_data)
			if image.isNull():
				self.error_message(u'Error opening file',
								   u"<p>Make sure <i>%s</i> is a valid image file." % unicode_file_path)
				self.status("Error reading %s" % unicode_file_path)
				return False
			self.status("Loaded %s" % os.path.basename(unicode_file_path))
			self.image = image
			self.file_path = unicode_file_path
			self.canvas.load_pixmap(QtGui.QPixmap.fromImage(image))
			if self.label_file:
				self.load_labels(self.label_file.shapes)
			self.set_clean()
			self.canvas.setEnabled(True)
			self.adjust_scale(initial=True)
			self.paint_canvas()
			self.add_recent_file(self.file_path)
			self.toggle_actions(True)
			self.show_bounding_box_from_annotation_file(file_path)

			counter = self.counter_str()
			self.setWindowTitle(APPNAME + ' ' + file_path + ' ' + counter)
		
			# Default : select last item if there is at least one item
			if self.label_list.count():
				items = self.label_list.findItems(self.combo_box.status, QtCore.Qt.MatchFixedString | QtCore.Qt.MatchCaseSensitive)
				if(items):
					self.label_list.setCurrentItem(items[0])
				else :
					self.label_list.setCurrentItem(self.label_list.item(self.label_list.count() - 1))
					self.label_list.item(self.label_list.count() - 1).setSelected(True)
			self.canvas.setFocus(True)
			return True
		return False

	def counter_str(self):
		"""
		Converts image counter to string representation.
		"""
		return '[{} / {}]'.format(self.cur_img_idx + 1, self.img_count)

	def show_bounding_box_from_annotation_file(self, file_path):
		if self.default_save_dir is not None:
			basename = os.path.basename(os.path.splitext(file_path)[0])
			xml_path = os.path.join(self.default_save_dir, basename + XML_EXT)
			txt_path = os.path.join(self.default_save_dir, basename + TXT_EXT)
			json_path = os.path.join(self.default_save_dir, basename + JSON_EXT)
			"""Annotation file priority:
			PascalXML > YOLO
			"""
			if os.path.isfile(xml_path):
				self.load_pascal_xml_by_filename(xml_path)
			elif os.path.isfile(txt_path):
				self.load_yolo_txt_by_filename(txt_path)
			elif os.path.isfile(json_path):
				self.load_create_ml_json_by_filename(json_path, file_path)

		else:
			xml_path = os.path.splitext(file_path)[0] + XML_EXT
			txt_path = os.path.splitext(file_path)[0] + TXT_EXT
			if os.path.isfile(xml_path):
				self.load_pascal_xml_by_filename(xml_path)
			elif os.path.isfile(txt_path):
				self.load_yolo_txt_by_filename(txt_path)

	def resizeEvent(self, event):
		if self.canvas and not self.image.isNull()\
		   and self.zoom_mode != self.MANUAL_ZOOM:
			self.adjust_scale()
		super(MainWindow, self).resizeEvent(event)

	def paint_canvas(self):
		if self.image.isNull() :
			self.debug.war("Cannot paint null image")
		else :
			assert not self.image.isNull(), "cannot paint null image"
			self.canvas.scale = 0.01 * self.zoom_widget.value()
			self.canvas.label_font_size = int(0.015 * max(self.image.width(), self.image.height()))
			self.canvas.adjustSize()
			self.canvas.update()

	def adjust_scale(self, initial=False):
		value = self.scalers[self.FIT_WINDOW if initial else self.zoom_mode]()
		self.zoom_widget.setValue(int(100 * value))

	def scale_fit_window(self):
		"""Figure out the size of the pixmap in order to fit the main widget."""
		e = 2.0  # So that no scrollbars are generated.
		w1 = self.centralWidget().width() - e
		h1 = self.centralWidget().height() - e
		a1 = w1 / h1
		# Calculate a new scale value based on the pixmap's aspect ratio.
		w2 = self.canvas.pixmap.width() - 0.0
		h2 = self.canvas.pixmap.height() - 0.0
		a2 = w2 / h2
		return w1 / w2 if a2 >= a1 else h1 / h2

	def scale_fit_width(self):
		# The epsilon does not seem to work too well here.
		w = self.centralWidget().width() - 2.0
		return w / self.canvas.pixmap.width()

	def closeEvent(self, event):
		if not self.may_continue():
			event.ignore()
		settings = self.settings
		# If it loads images from dir, don't load it at the beginning
		if self.dir_name is None:
			settings[SETTING_FILENAME] = self.file_path if self.file_path else ''
		else:
			settings[SETTING_FILENAME] = ''

		settings[SETTING_WIN_SIZE] = self.size()
		settings[SETTING_WIN_POSE] = self.pos()
		settings[SETTING_WIN_STATE] = self.saveState()
		settings[SETTING_LINE_COLOR] = self.line_color
		settings[SETTING_FILL_COLOR] = self.fill_color
		settings[SETTING_RECENT_FILES] = self.recent_files
		settings[SETTING_ADVANCE_MODE] = not self._beginner
		if self.default_save_dir and os.path.exists(self.default_save_dir):
			settings[SETTING_SAVE_DIR] = ustr(self.default_save_dir)
		else:
			settings[SETTING_SAVE_DIR] = ''

		if self.last_open_dir and os.path.exists(self.last_open_dir):
			settings[SETTING_LAST_OPEN_DIR] = self.last_open_dir
		else:
			settings[SETTING_LAST_OPEN_DIR] = ''

		settings[SETTING_AUTO_SAVE] = self.auto_saving.isChecked()
		# settings[SETTING_SINGLE_CLASS] = self.single_class_mode.isChecked()
		settings[SETTING_PAINT_LABEL] = self.display_label_option.isChecked()
		settings[SETTING_DRAW_SQUARE] = self.draw_rectangles_option.isChecked()
		settings[SETTING_LABEL_FILE_FORMAT] = self.label_file_format
		settings.save()
		
	def load_recent(self, filename):
		if self.may_continue():
			self.load_file(filename)

	def scan_all_images(self, folder_path):
		extensions = ['.%s' % fmt.data().decode("ascii").lower() for fmt in QtGui.QImageReader.supportedImageFormats()]
		images = []

		for root, dirs, files in os.walk(folder_path):
			for file in files:
				if file.lower().endswith(tuple(extensions)):
					relative_path = os.path.join(root, file)
					path = ustr(os.path.abspath(relative_path))
					images.append(path)
		natural_sort(images, key=lambda x: x.lower())
		return images

	def check_annotation_file_exist(self) :
		labels = []
		if self.default_save_dir and os.path.exists(self.default_save_dir):
			for file in Path(self.default_save_dir).iterdir():
				if file.suffix.lower() in (XML_EXT, TXT_EXT, JSON_EXT):
					labels.append(str(file.stem))

		self.loading = LoadingExtension(self)
		self.loading.startLoading()
		for index in range(self.file_list_widget.count()):
			step = max(int(self.file_list_widget.count()//10), 1)
			if index%step == 0:
				QtWidgets.QApplication.processEvents()
			item_widget = self.file_list_widget.item(index)
			self.loading.setProgress(int((index/(self.file_list_widget.count()-1))*100))
			saved_file_name = os.path.basename(item_widget.text())
			if os.path.splitext(saved_file_name)[0]  not in labels:
				item_widget.setBackground(QtGui.QColor('brown'))
			else :
				item_widget.setBackground(QtGui.QColor( 'darkslategrey'))   
		self.loading.loadingFinished()

	def change_save_dir_dialog(self, _value=False):
		if self.default_save_dir is not None:
			path = ustr(self.default_save_dir)
		else:
			path = '.'

		dir_path = ustr(QtWidgets.QFileDialog.getExistingDirectory(self,
														 '%s - Save annotations to the directory' % APPNAME, path,  QtWidgets.QFileDialog.ShowDirsOnly
														 | QtWidgets.QFileDialog.DontResolveSymlinks))
		if dir_path is not None and len(dir_path) > 1:
			self.default_save_dir = dir_path
			self.check_annotation_file_exist()

		self.statusBar().showMessage('%s . Annotation will be saved to %s' %
									 ('Change saved folder', self.default_save_dir))
		self.statusBar().show()

	def update_annotation_status(self, annotation_file_path) :
		if self.file_path is None:
			return
		
		backbround_color = 'brown' if not os.path.isfile(annotation_file_path) else 'darkslategrey' # '#19232D'
		for item in self.file_list_widget.findItems(self.file_path, QtCore.Qt.MatchExactly) :
			self.file_list_widget.item(self.file_list_widget.row(item)).setBackground(QtGui.QColor(backbround_color))

	def open_annotation_dialog(self, _value=False):
		if self.file_path is None:
			self.statusBar().showMessage('Please select image first')
			self.statusBar().show()
			return

		path = os.path.dirname(ustr(self.file_path))\
			if self.file_path else '.'
		if self.label_file_format == LabelFileFormat.PASCAL_VOC:
			filters = "Open Annotation XML file (%s)" % ' '.join(['*.xml'])
			filename = ustr(QtWidgets.QFileDialog.getOpenFileName(self, '%s - Choose a xml file' % APPNAME, path, filters))
			if filename:
				if isinstance(filename, (tuple, list)):
					filename = filename[0]
			self.load_pascal_xml_by_filename(filename)

	def open_dir_dialog(self, _value=False, dir_path=None, silent=False):
		if not self.may_continue():
			return

		default_open_dir_path = dir_path if dir_path else '.'
		if self.last_open_dir and os.path.exists(self.last_open_dir):
			default_open_dir_path = self.last_open_dir
		else:
			default_open_dir_path = os.path.dirname(self.file_path) if self.file_path else '.'
		if silent != True:
			target_dir_path = ustr(QtWidgets.QFileDialog.getExistingDirectory(self,
																	'%s - Open Directory' % APPNAME, default_open_dir_path,
																	QtWidgets.QFileDialog.ShowDirsOnly | QtWidgets.QFileDialog.DontResolveSymlinks))
		else:
			target_dir_path = ustr(default_open_dir_path)
		self.last_open_dir = target_dir_path
		self.import_dir_images(target_dir_path)
		if len(target_dir_path) > 1:
			subfolders = [item for item in Path(target_dir_path).iterdir() if item.is_dir()]
			if len(subfolders) == 2:
				for folder in subfolders:
					if not self.scan_all_images(str(target_dir_path / folder)):
						self.default_save_dir = str(target_dir_path / folder)
			else:
				if len(subfolders) > 2:
					self.statusBar().showMessage(f"Directory [{target_dir_path}] contains more than two subfolders can't find labels folder.")
					self.statusBar().show()
				self.default_save_dir = target_dir_path
			self.check_annotation_file_exist()

	def import_dir_images(self, dir_path):
		if not self.may_continue() or not dir_path:
			return

		self.last_open_dir = dir_path
		self.dir_name = dir_path
		self.file_path = None
		self.file_list_widget.clear()

		self.m_img_list = self.scan_all_images(dir_path) 
		self.img_count = len(self.m_img_list)
		if self.img_count == 0:
			self.statusBar().showMessage(f"Directory [{dir_path}] can't find images.")
			self.statusBar().show()
		self.open_next_image()
		for imgPath in self.m_img_list:
			item = QtWidgets.QListWidgetItem(imgPath)
			self.file_list_widget.addItem(item)

	def verify_image(self, _value=False):
		# Proceeding next image without dialog if having any label
		if self.file_path is not None:
			try:
				self.label_file.toggle_verify()
			except AttributeError:
				# If the labelling file does not exist yet, create if and
				# re-save it with the verified attribute.
				self.save_file()
				if self.label_file is not None:
					self.label_file.toggle_verify()
				else:
					return

			self.canvas.verified = self.label_file.verified
			self.paint_canvas()
			self.save_file()

	def thumbnail_image(self, _value=False):
		save_label_list = []
		read_image_list = []
		self.loading = LoadingExtension(self)
		self.loading.startLoading()

		for index in range(self.file_list_widget.count()):
			step = max(int(self.file_list_widget.count()//10), 1)
			if index%step == 0:
				QtWidgets.QApplication.processEvents()
			self.loading.setProgress(int((index/(self.file_list_widget.count()-1))*100))
			image_path = self.file_list_widget.item(index).text()
			basename = os.path.splitext(os.path.basename(image_path))[0]
			label_path = os.path.join(self.default_save_dir, basename + LabelFile.suffix)
			if os.path.isfile(label_path):
				save_label_list.append(label_path)
				read_image_list.append(image_path)
		self.loading.loadingFinished()

		natural_sort(save_label_list, key=lambda x: x.lower())
		thumbnailview = ThumbnailView(read_image_list, save_label_list, parent=self) 
		if self.qdarkstyle : 
			thumbnailview.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
		thumbnailview.exec()
		self.load_file(self.file_path)
		self.check_annotation_file_exist()

	def open_prev_image(self, _value=False):
		# Proceeding prev image without dialog if having any label
		if self.auto_saving.isChecked():
			if self.default_save_dir is not None:
				if self.dirty is True:
					self.save_file()
			else:
				self.change_save_dir_dialog()
				return

		if not self.may_continue():
			return

		if self.img_count <= 0:
			return

		if self.file_path is None:
			return

		if self.cur_img_idx - 1 >= 0:
			self.cur_img_idx -= 1
			filename = self.m_img_list[self.cur_img_idx]
			if filename:
				self.load_file(filename)

	def open_next_image(self, _value=False):
		# Proceeding prev image without dialog if having any label
		if self.auto_saving.isChecked():
			if self.default_save_dir is not None:
				if self.dirty is True:
					self.save_file()
			else:
				self.change_save_dir_dialog()
				return

		if not self.may_continue():
			return

		if self.img_count <= 0:
			return

		filename = None
		if self.file_path is None and len(self.m_img_list):
			filename = self.m_img_list[0]
			self.cur_img_idx = 0
		else:
			if self.cur_img_idx + 1 < self.img_count:
				self.cur_img_idx += 1
				filename = self.m_img_list[self.cur_img_idx]

		if filename:
			self.load_file(filename)

	def open_file(self, _value=False):
		if not self.may_continue():
			return
		path = os.path.dirname(ustr(self.file_path)) if self.file_path else '.'
		formats = ['*.%s' % fmt.data().decode("ascii").lower() for fmt in QtGui.QImageReader.supportedImageFormats()]
		filters = "Image & Label files (%s)" % ' '.join(formats + ['*%s' % LabelFile.suffix])
		filename = QtWidgets.QFileDialog.getOpenFileName(self, '%s - Choose Image or Label file' % APPNAME, path, filters)
		if filename:
			if isinstance(filename, (tuple, list)):
				filename = filename[0]
			self.cur_img_idx = 0
			self.img_count = 1
			self.load_file(filename)
   
   
			self.m_img_list.append(filename)
			item = QtWidgets.QListWidgetItem(filename)
			self.file_list_widget.addItem(item)
   

			self.m_img_list.append(filename)
			item = QtWidgets.QListWidgetItem(filename)
			self.file_list_widget.addItem(item)
   
	def save_file(self, _value=False):
		if self.default_save_dir is not None and len(ustr(self.default_save_dir)):
			if self.file_path:
				image_file_name = os.path.basename(self.file_path)
				saved_file_name = os.path.splitext(image_file_name)[0]
				saved_path = os.path.join(ustr(self.default_save_dir), saved_file_name)
				self._save_file(saved_path)
		else:
			image_file_dir = os.path.dirname(self.file_path)
			image_file_name = os.path.basename(self.file_path)
			saved_file_name = os.path.splitext(image_file_name)[0]
			saved_path = os.path.join(image_file_dir, saved_file_name)
			self._save_file(saved_path if self.label_file
							else self.save_file_dialog(remove_ext=False))

	def save_file_as(self, _value=False):
		assert not self.image.isNull(), "cannot save empty image"
		self._save_file(self.save_file_dialog())

	def save_file_dialog(self, remove_ext=True):
		caption = '%s - Choose File' % APPNAME
		filters = 'File (*%s)' % LabelFile.suffix
		open_dialog_path = self.current_path()
		dlg = QtWidgets.QFileDialog(self, caption, open_dialog_path, filters)
		dlg.setDefaultSuffix(LabelFile.suffix[1:])
		dlg.setAcceptMode(QtWidgets.QFileDialog.AcceptSave)
		filename_without_extension = os.path.splitext(self.file_path)[0]
		dlg.selectFile(filename_without_extension)
		dlg.setOption(QtWidgets.QFileDialog.DontUseNativeDialog, False)
		if dlg.exec_():
			full_file_path = ustr(dlg.selectedFiles()[0])
			if remove_ext:
				return os.path.splitext(full_file_path)[0]  # Return file path without the extension.
			else:
				return full_file_path
		return ''

	def _save_file(self, annotation_file_path):
		if annotation_file_path and self.save_labels(annotation_file_path):
			self.set_clean()
			self.statusBar().showMessage('Saved to  %s' % annotation_file_path)
			self.statusBar().show()

	def close_file(self, _value=False):
		if not self.may_continue():
			return
		self.reset_state()
		self.set_clean()
		self.toggle_actions(False)
		self.canvas.setEnabled(False)
		self.actions.saveAs.setEnabled(False)

	def delete_image(self):
		delete_path = self.file_path
		if delete_path is not None:
			self.open_next_image()
			self.cur_img_idx -= 1
			self.img_count -= 1
			if os.path.exists(delete_path):
				os.remove(delete_path)
			self.import_dir_images(self.last_open_dir)

	def reset_all(self):
		self.settings.reset()
		self.close()
		process = QtCore.QProcess()
		process.startDetached(os.path.abspath(__file__))

	def may_continue(self):
		if not self.dirty:
			return True
		else:
			discard_changes = self.discard_changes_dialog()
			if discard_changes == QtWidgets.QMessageBox.No:
				return True
			elif discard_changes == QtWidgets.QMessageBox.Yes:
				self.save_file()
				return True
			else:
				return False

	def discard_changes_dialog(self):
		yes, no, cancel = QtWidgets.QMessageBox.Yes, QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.Cancel
		msg = u'You have unsaved changes, would you like to save them and proceed?\nClick "No" to undo all changes.'
		return QtWidgets.QMessageBox.warning(self, u'Attention', msg, yes | no | cancel)

	def error_message(self, title, message):
		return QtWidgets.QMessageBox.critical(self, title,
									'<p><b>%s</b></p>%s' % (title, message))

	def current_path(self):
		return os.path.dirname(self.file_path) if self.file_path else '.'

	def choose_color1(self):
		color = self.color_dialog.getColor(self.line_color, u'Choose line color',
										   default=DEFAULT_LINE_COLOR)
		if color:
			self.line_color = color
			Shape.line_color = color
			self.canvas.set_drawing_color(color)
			self.canvas.update()
			self.set_dirty()

	def delete_selected_shape(self):
		self.remove_label(self.canvas.delete_selected())
		self.set_dirty()
		if self.no_shapes():
			for action in self.actions.onShapesPresent:
				action.setEnabled(False)

	def choose_shape_line_color(self):
		color = self.color_dialog.getColor(self.line_color, u'Choose Line Color',
										   default=DEFAULT_LINE_COLOR)
		if color:
			self.canvas.selected_shape.line_color = color
			self.canvas.update()
			self.set_dirty()

	def choose_shape_fill_color(self):
		color = self.color_dialog.getColor(self.fill_color, u'Choose Fill Color',
										   default=DEFAULT_FILL_COLOR)
		if color:
			self.canvas.selected_shape.fill_color = color
			self.canvas.update()
			self.set_dirty()

	def copy_shape(self):
		self.canvas.end_move(copy=True)
		self.add_label(self.canvas.selected_shape)
		self.set_dirty()

	def move_shape(self):
		self.canvas.end_move(copy=False)
		self.set_dirty()

	def load_predefined_classes(self, predef_classes_file):
		label_hist = None
		if os.path.exists(predef_classes_file) is True:
			with codecs.open(predef_classes_file, 'r', 'utf8') as f:
				for line in f:
					line = line.strip()
					if label_hist is None:
						label_hist = [line]
					else:
						label_hist.append(line)
			self.classes_file = predef_classes_file
		else :
			self.classes_file = None
		return label_hist

	def load_pascal_xml_by_filename(self, xml_path):
		if self.file_path is None:
			return
		if os.path.isfile(xml_path) is False:
			return

		self.set_format(FORMAT_PASCALVOC)
		self.update_annotation_status(xml_path)
		
		t_voc_parse_reader = PascalVocReader(xml_path) 
		shapes = t_voc_parse_reader.get_shapes()
		self.debug.debug("PascalVocReader shape : " + str(shapes))
		self.load_labels(shapes) 
		self.canvas.verified = t_voc_parse_reader.verified

	def load_yolo_txt_by_filename(self, txt_path):
		if self.file_path is None:
			return
		if os.path.isfile(txt_path) is False:
			return

		self.set_format(FORMAT_YOLO)
		self.update_annotation_status(txt_path)
		
		t_yolo_parse_reader = YoloReader(txt_path, self.image, self.classes_file)
		shapes = t_yolo_parse_reader.get_shapes()
		self.debug.debug("YoloReader shape : " + str(shapes))
		self.load_labels(shapes)
		self.canvas.verified = t_yolo_parse_reader.verified

	def load_create_ml_json_by_filename(self, json_path, file_path):
		if self.file_path is None:
			return
		if os.path.isfile(json_path) is False:
			return

		self.set_format(FORMAT_CREATEML)
		self.update_annotation_status(json_path)
		
		create_ml_parse_reader = CreateMLReader(json_path, file_path)
		shapes = create_ml_parse_reader.get_shapes()
		self.debug.debug("CreateMLReader shape : " + str(shapes))
		self.load_labels(shapes)
		self.canvas.verified = create_ml_parse_reader.verified

	def copy_previous_bounding_boxes(self):
		current_index = self.m_img_list.index(self.file_path)
		if current_index - 1 >= 0:
			prev_file_path = self.m_img_list[current_index - 1]
			self.show_bounding_box_from_annotation_file(prev_file_path)
			self.save_file()

	def toggle_paint_labels_option(self):
		for shape in self.canvas.shapes:
			shape.paint_label = self.display_label_option.isChecked()

	def toggle_draw_rectangles(self):
		self.use_box_size_checkbox.setChecked(self.draw_rectangles_option.isChecked())
		self.change_box_size_ratio()

	def change_box_size_ratio(self):
		if self.draw_rectangles_option.isChecked() != self.use_box_size_checkbox.isChecked():
			self.draw_rectangles_option.setChecked(self.use_box_size_checkbox.isChecked())

		if (self.use_box_size_checkbox.isChecked() ) :
			self.Wspbox.setEnabled(True)
			self.Hspbox.setEnabled(True)
			self.canvas.set_drawing_free_shape(False)
			self.canvas.set_drawing_shape_Width_and_Height((self.Wspbox.value(), self.Hspbox.value()) )
			self.label_selection_changed()
		else :
			self.Wspbox.setEnabled(False)
			self.Hspbox.setEnabled(False)
			self.canvas.set_drawing_free_shape(True)

class MainWidget(QtWidgets.QWidget):
	def __init__(self, class_path: str, debug = None):
		super().__init__()
		self.debug = debug
		self.class_path = class_path

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
		pixmap = QtGui.QPixmap(str(Path(__file__).resolve().parents[1] / "demo/labelingUI.png")) 
		screen = QtWidgets.QDesktopWidget().screenGeometry()
		width = int(screen.width() * 0.2)
		height = int(screen.height() * 0.2)
		pixmap = pixmap.scaled(width, height)
		imagelabel.setPixmap(pixmap)
		imagelabel.setScaledContents(True)
		imagelabel.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
		imageVLayout.addWidget(imagelabel)
		topHLayout.addLayout(imageVLayout)

		# format
		formatVLayout = QtWidgets.QVBoxLayout()
		formatVLayout.setContentsMargins(5, 40, 0, 0)
		formatLabel = QtWidgets.QLabel("Save Label Format : ")
		formatLabel.setStyleSheet("color: rgb(255, 255, 255);")
		formatLabel.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
		formatVLayout.addWidget(formatLabel)
		data = {':/format_voc' : FORMAT_PASCALVOC + " ➡️ " + XML_EXT, 
				':/format_yolo' : FORMAT_YOLO + " ➡️ " + TXT_EXT,
				':/format_createml' : FORMAT_CREATEML + " ➡️ " + JSON_EXT}
		formatTableWidget = QtWidgets.QTableWidget(len(data), 2)
		formatTableWidget.setStyleSheet(TABLE_QSS)
		formatTableWidget.setFrameShape(QtWidgets.QFrame.NoFrame)
		formatTableWidget.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOn)
		formatTableWidget.setSizeAdjustPolicy(QtWidgets.QAbstractScrollArea.AdjustToContents)
		formatTableWidget.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
		formatTableWidget.setAlternatingRowColors(False)
		formatTableWidget.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
		formatTableWidget.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
		formatTableWidget.setShowGrid(True)
		formatTableWidget.setGridStyle(QtCore.Qt.SolidLine)
		formatTableWidget.setSortingEnabled(False)
		formatTableWidget.horizontalHeader().setVisible(True)
		formatTableWidget.horizontalHeader().setStretchLastSection(True)
		formatTableWidget.verticalHeader().setVisible(False)
		formatTableWidget.verticalHeader().setCascadingSectionResizes(False)
		formatTableWidget.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
		formatTableWidget.setHorizontalHeaderLabels(["Icon", "Describe"])
		for n, item in enumerate(data.keys()):
			newitem = QtWidgets.QTableWidgetItem()
			icon = QtGui.QIcon(item)
			pixmap = icon.pixmap(QtCore.QSize(50, 50))
			newitem.setIcon(QtGui.QIcon(pixmap))
			newitem.setTextAlignment(QtCore.Qt.AlignCenter)
			formatTableWidget.setItem(n, 0, newitem)
			
			newitem = QtWidgets.QTableWidgetItem(data[item])
			formatTableWidget.setItem(n, 1, newitem)

			formatTableWidget.setRowHeight(n, 25)
		formatTableWidget.resizeColumnsToContents()
		formatVLayout.addWidget(formatTableWidget)
		saveLabel = QtWidgets.QLabel("✟ Two recommended methods for saving labels ✟\n1) Save image and label files in an item folder.\n2) Create image and label subfolders within an item folder.")
		saveLabel.setStyleSheet("color: rgb(160, 160, 160);")
		font = QtGui.QFont('Times', 10, QtGui.QFont.Bold)
		font.setItalic(True)
		saveLabel.setFont(font)
		formatVLayout.addWidget(saveLabel)
		topHLayout.addLayout(formatVLayout)

		# --------------------------------------------
		#               bottom Part
		# --------------------------------------------
		bottomVLayout = QtWidgets.QVBoxLayout()

		keyboardLabel = QtWidgets.QLabel("KeyBoard Control : ")
		keyboardLabel.setStyleSheet("color: rgb(255, 255, 255);")
		keyboardLabel.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
		bottomVLayout.addWidget(keyboardLabel)
		data1 = { '『Ctrl+U』': [":/open-dir", "Open images/label Dir"], 
				  '『Ctrl+R』': [":/save-dir", "Change default saved label dir"], 
		   		  '『D』' : [':/next', "Open the next Image"],  
				  '『A』' : [':/prev', "Open the previous Image"], 
				  '『W』' : [':/rect', "Draw a new Rectangle"], 
				  '『E』' : [':/poly', "Draw a new Polygon"], 
				  '『Delete』': [':/delete', "Remove the object area"], 
				  '『Ctrl+C』' : [':/copy', "Create a duplicate of the selected object"], 
				  '『Ctrl+S』' : [':/save', "Save the labels to a file"], 
				  '『Ctrl+E』' : [':/edit',"Modify the label of the selected object"], 
				  '『Ctrl+Q』' : [':/quit',"Quit application"], } 
		keyboardTableWidget = QtWidgets.QTableWidget(len(data1), 3) 
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
		for n, item in enumerate(data1.keys()):
			newitem = QtWidgets.QTableWidgetItem(item) 
			newitem.setTextAlignment(QtCore.Qt.AlignCenter) 
			keyboardTableWidget.setItem(n, 0, newitem) 
			
			newitem = QtWidgets.QTableWidgetItem() 
			newitem.setTextAlignment(QtCore.Qt.AlignCenter) 
			icon = QtGui.QIcon(data1[item][0]) 
			pixmap_size = QtCore.QSize(50, 50) 
			pixmap = icon.pixmap(pixmap_size) 
			scaled_pixmap = pixmap.scaled(pixmap_size) 
			newitem.setIcon(QtGui.QIcon(scaled_pixmap)) 
			keyboardTableWidget.setItem(n, 1, newitem) 
				 
			newitem = QtWidgets.QTableWidgetItem(data1[item][1]) 
			keyboardTableWidget.setItem(n, 2, newitem) 
		keyboardTableWidget.resizeColumnsToContents()
		bottomVLayout.addWidget(keyboardTableWidget)

		# load app
		btnHLayout = QtWidgets.QHBoxLayout()
		loadBtn = QtWidgets.QPushButton("Launch")
		loadBtn.setObjectName('QPushBtn_labeling')
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
		if (sendingBtn.objectName() == "QPushBtn_labeling") :
			self.win = MainWindow(default_prefdef_class_file = self.class_path,
								  debug=self.debug)
			self.win.set_qdarkstyle()
			self.win.show()

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

def main(argv=[]):
	"""construct main app and run it"""
	"""
	Standard boilerplate Qt application code.
	Do everything but app.exec_() -- so that we can test the application in one thread
	"""
	app = QtWidgets.QApplication(argv)
	app.setApplicationName(APPNAME)
	app.setWindowIcon(new_icon("app"))
	app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
	# Tzutalin 201705+: Accept extra agruments to change predefined class file
	argparser = argparse.ArgumentParser()
	argparser.add_argument("-i", "--image_dir", nargs="?")
	argparser.add_argument("-c", "--class_file",
						   default= os.path.join(os.getcwd(), 'default_classes.txt'),
						   nargs="?")
	argparser.add_argument("-o", "--save_dir", nargs="?")
	args = argparser.parse_args(argv[1:])

	args.image_dir = args.image_dir and os.path.normpath(args.image_dir)
	args.class_file = args.class_file and os.path.normpath(args.class_file)
	args.save_dir = args.save_dir and os.path.normpath(args.save_dir)

	# Usage : labelImg.py image classFile saveDir
	win = MainWindow(args.image_dir,
					 args.class_file,
					 args.save_dir,
					 debug=debug)
	win.show()
	return app.exec_()

# if __name__ == '__main__':
# 	sys.exit(main(sys.argv))
