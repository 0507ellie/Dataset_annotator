import argparse
from enum import Enum
from qtpy import QtCore, QtGui, QtWidgets
from pages.resources.resources  import *

__version_info__ = ('1', '2', '1', '2505')
__version__ = '.'.join(__version_info__)

class AnnotatorType(Enum):
	Labeling = 0
	Tracking = 1
	Convert  = 2
	Create   = 3

	@classmethod
	def list(cls):
		return list(map(lambda c: c.value, cls))

	@classmethod
	def argparse_type(cls, value):
		try:
			# 將輸入字串轉換為枚舉值
			return cls[value]
		except KeyError:
			raise argparse.ArgumentTypeError(
				f"Invalid task type: {value}. Valid options are: {[t.name for t in cls]}"
			)
   
class MainWindow(object):
	def setupUi(self, MainWindow):
		MainWindow.setWindowTitle("Annotator-" + __version__)
		MainWindow.resize(950, 600)
		MainWindow.setWindowIcon(QtGui.QIcon(":/app/logo"))
		self.centralwidget = QtWidgets.QWidget(MainWindow)
		self.centralwidget.setObjectName("centralwidget")
		
		self.gridLayout = QtWidgets.QGridLayout(self.centralwidget)
		self.gridLayout.setContentsMargins(0, 0, 0, 0)
		self.gridLayout.setSpacing(0)
		self.gridLayout.setObjectName("gridLayout")
		
		self.initIcon()
		self.initIconMenuUI()
		self.initFullMenuUI()

		# ==================================================
		#                 Page Part
		# ==================================================  
		self.widget_3 = QtWidgets.QWidget(self.centralwidget)
		self.widget_3.setObjectName("widget_3")
		
		self.pageVLayout = QtWidgets.QVBoxLayout(self.widget_3)
		self.pageVLayout.setContentsMargins(0, 0, 0, 0)
		self.pageVLayout.setSpacing(0)
		self.titleWidget = QtWidgets.QWidget(self.widget_3)
		self.titleWidget.setMinimumSize(QtCore.QSize(0, 40))
		self.titleWidget.setStyleSheet("background-color: #262c32;")
		self.titleHLayout = QtWidgets.QHBoxLayout(self.titleWidget)
		self.titleHLayout.setContentsMargins(0, 0, 15, 0)
		self.titleHLayout.setSpacing(0)
		menuBtn = QtWidgets.QPushButton(self.titleWidget)
		menuBtn.setText("")
		menuBtn.setIcon(self.menuIcon)
		menuBtn.setIconSize(QtCore.QSize(25, 25))
		menuBtn.setCheckable(True)
		menuBtn.setObjectName("menu_btn")
		self.titleHLayout.addWidget(menuBtn)
		self.pageVLayout.addWidget(self.titleWidget)
		
		spacerItem = QtWidgets.QSpacerItem(236, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
		self.titleHLayout.addItem(spacerItem)

		# --------------------------------------------
		#                  Page Widget 
		# --------------------------------------------
		self.stackedWidget = QtWidgets.QStackedWidget(self.widget_3)
		self.stackedWidget.setObjectName("stackedWidget")
		self.pageVLayout.addWidget(self.stackedWidget)
		self.gridLayout.addWidget(self.widget_3, 0, 2, 1, 1)
		
		self.stackedWidget.setCurrentIndex(6)
		menuBtn.toggled['bool'].connect(self.icon_menu_widget.setVisible) # type: ignore
		menuBtn.toggled['bool'].connect(self.full_menu_widget.setHidden) # type: ignore
		self.labelingIconBtn.toggled['bool'].connect(self.labelingFullBtn.setChecked) # type: ignore
		self.tracklingIconBtn.toggled['bool'].connect(self.tracklingFullBtn.setChecked) # type: ignore
		self.formatIconBtn.toggled['bool'].connect(self.formatFullBtn.setChecked) # type: ignore
		self.databaseIconBtn.toggled['bool'].connect(self.databaseFullBtn.setChecked) # type: ignore
		self.labelingFullBtn.toggled['bool'].connect(self.labelingIconBtn.setChecked) # type: ignore
		self.tracklingFullBtn.toggled['bool'].connect(self.tracklingIconBtn.setChecked) # type: ignore
		self.formatFullBtn.toggled['bool'].connect(self.formatIconBtn.setChecked) # type: ignore
		self.databaseFullBtn.toggled['bool'].connect(self.databaseIconBtn.setChecked) # type: ignore
		self.exitFullBtn.clicked.connect(MainWindow.close) # type: ignore
		self.exitIconBtn.clicked.connect(MainWindow.close) # type: ignore
		MainWindow.setCentralWidget(self.centralwidget)
		QtCore.QMetaObject.connectSlotsByName(MainWindow)

	def initIcon(self):
		self.logoIcon = QtGui.QPixmap(":/app/logo")
		self.labelingIcon = QtGui.QIcon()
		self.labelingIcon.addPixmap(QtGui.QPixmap(":/app/labelingTool"), QtGui.QIcon.Normal, QtGui.QIcon.On)
		self.trackingIcon = QtGui.QIcon()
		self.trackingIcon.addPixmap(QtGui.QPixmap(":/app/trackingTool"), QtGui.QIcon.Normal, QtGui.QIcon.On)
		self.formatIcon = QtGui.QIcon()
		self.formatIcon.addPixmap(QtGui.QPixmap(":/annotator/convert-label"), QtGui.QIcon.Normal, QtGui.QIcon.On)
		self.databaseIcon = QtGui.QIcon()
		self.databaseIcon.addPixmap(QtGui.QPixmap(":/annotator/database"), QtGui.QIcon.Normal, QtGui.QIcon.On)
		self.exitIcon = QtGui.QIcon()
		self.exitIcon.addPixmap(QtGui.QPixmap(":/annotator/close"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
		self.menuIcon = QtGui.QIcon()
		self.menuIcon.addPixmap(QtGui.QPixmap(":/annotator/menu"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
		
	def initIconMenuUI(self) :
		icon_size = QtCore.QSize(25, 25)
		# ==================================================
		#                 Icon SideBar Item
		# ==================================================
		self.icon_menu_widget = QtWidgets.QWidget(self.centralwidget)
		self.icon_menu_widget.setObjectName("icon_menu_widget")
		menuVLayout = QtWidgets.QVBoxLayout(self.icon_menu_widget)
		menuVLayout.setContentsMargins(0, 0, 0, 0)
		menuVLayout.setSpacing(0)
		logoHLayout = QtWidgets.QHBoxLayout()
		
		# Icon_Menu LOGO
		logoLabel = QtWidgets.QLabel(self.icon_menu_widget)
		logoLabel.setMinimumSize(QtCore.QSize(70, 70))
		logoLabel.setMaximumSize(QtCore.QSize(70, 70))
		logoLabel.setText("")
		logoLabel.setPixmap(self.logoIcon)
		logoLabel.setScaledContents(True)
		logoLabel.setObjectName("icon_logo_label")
		logoHLayout.addWidget(logoLabel)
		menuVLayout.addLayout(logoHLayout)
		
		# --------------------------------------------
		#                Icon_Menu Item
		# --------------------------------------------
		btnVLayout = QtWidgets.QVBoxLayout()
		btnVLayout.setSpacing(0)

		# LabelingTool
		self.labelingIconBtn = QtWidgets.QPushButton(self.icon_menu_widget)
		self.labelingIconBtn.setText("")
		self.labelingIconBtn.setIcon(self.labelingIcon)
		self.labelingIconBtn.setIconSize(icon_size)
		self.labelingIconBtn.setCheckable(True)
		self.labelingIconBtn.setAutoExclusive(True)
		self.labelingIconBtn.setObjectName("labelingBtn")
		btnVLayout.addWidget(self.labelingIconBtn)
		
		# TrackingTool
		self.tracklingIconBtn = QtWidgets.QPushButton(self.icon_menu_widget)
		self.tracklingIconBtn.setText("")
		self.tracklingIconBtn.setIcon(self.trackingIcon)
		self.tracklingIconBtn.setIconSize(icon_size)
		self.tracklingIconBtn.setCheckable(True)
		self.tracklingIconBtn.setAutoExclusive(True)
		self.tracklingIconBtn.setObjectName("tracklingBtn")
		btnVLayout.addWidget(self.tracklingIconBtn)
		
		# ConvertFormat
		self.formatIconBtn = QtWidgets.QPushButton(self.icon_menu_widget)
		self.formatIconBtn.setText("")
		self.formatIconBtn.setIcon(self.formatIcon)
		self.formatIconBtn.setIconSize(icon_size)
		self.formatIconBtn.setCheckable(True)
		self.formatIconBtn.setAutoExclusive(True)
		self.formatIconBtn.setObjectName("formatBtn")
		btnVLayout.addWidget(self.formatIconBtn)
		
		# CreateDataBase
		self.databaseIconBtn = QtWidgets.QPushButton(self.icon_menu_widget)
		self.databaseIconBtn.setText("")
		self.databaseIconBtn.setIcon(self.databaseIcon)
		self.databaseIconBtn.setIconSize(icon_size)
		self.databaseIconBtn.setCheckable(True)
		self.databaseIconBtn.setAutoExclusive(True)
		self.databaseIconBtn.setObjectName("databaseBtn")
		btnVLayout.addWidget(self.databaseIconBtn)
		menuVLayout.addLayout(btnVLayout)
		
		spacerItem = QtWidgets.QSpacerItem(20, 375, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
		menuVLayout.addItem(spacerItem)
		
		# Exit
		self.exitIconBtn = QtWidgets.QPushButton(self.icon_menu_widget)
		self.exitIconBtn.setText("")
		self.exitIconBtn.setIcon(self.exitIcon)
		self.exitIconBtn.setIconSize(icon_size)
		self.exitIconBtn.setObjectName("exit_btn_1")
		menuVLayout.addWidget(self.exitIconBtn)
		
		self.gridLayout.addWidget(self.icon_menu_widget, 0, 0, 1, 1)

	def initFullMenuUI(self):
		# ==================================================
		#                 Full SideBar Item
		# ==================================================  
		self.full_menu_widget = QtWidgets.QWidget(self.centralwidget)
		self.full_menu_widget.setObjectName("full_menu_widget")
		menuVLayout = QtWidgets.QVBoxLayout(self.full_menu_widget)
		menuVLayout.setObjectName("verticalLayout_4")
		logoHLayout = QtWidgets.QHBoxLayout()
		logoHLayout.setSpacing(0)
		
		# Full_Menu LOGO
		logoIconLabel = QtWidgets.QLabel(self.full_menu_widget)
		logoIconLabel.setMinimumSize(QtCore.QSize(60, 60))
		logoIconLabel.setMaximumSize(QtCore.QSize(60, 60))
		logoIconLabel.setText("")
		logoIconLabel.setPixmap(self.logoIcon)
		logoIconLabel.setScaledContents(True)
		logoIconLabel.setObjectName("full_logo_icon_label")
		logoHLayout.addWidget(logoIconLabel)
		logoLabel = QtWidgets.QLabel("Annotator", self.full_menu_widget)
		logoLabel.setFont(QtGui.QFont("Times", 15, QtGui.QFont.Bold))
		logoLabel.setObjectName("full_logo_label")

		logoHLayout.addWidget(logoLabel)
		menuVLayout.addLayout(logoHLayout)
		
		# --------------------------------------------
		#                Full_Menu Item
		# --------------------------------------------
		icon_size = QtCore.QSize(20, 20)
		font_size = QtGui.QFont("Times", 10, QtGui.QFont.Bold)
		btnVLayout = QtWidgets.QVBoxLayout()
		btnVLayout.setSpacing(0)
		
		# LabelingTool
		self.labelingFullBtn = QtWidgets.QPushButton("Manual Labeling", self.full_menu_widget)
		self.labelingFullBtn.setIcon(self.labelingIcon)
		self.labelingFullBtn.setIconSize(icon_size)
		self.labelingFullBtn.setFont(font_size)
		self.labelingFullBtn.setCheckable(True)
		self.labelingFullBtn.setAutoExclusive(True)
		self.labelingFullBtn.setObjectName("labelingBtn")
		btnVLayout.addWidget(self.labelingFullBtn)
		
		# TrackingTool
		self.tracklingFullBtn = QtWidgets.QPushButton("Tracker Labeling", self.full_menu_widget)
		self.tracklingFullBtn.setIcon(self.trackingIcon)
		self.tracklingFullBtn.setIconSize(icon_size)
		self.tracklingFullBtn.setFont(font_size)
		self.tracklingFullBtn.setCheckable(True)
		self.tracklingFullBtn.setAutoExclusive(True)
		self.tracklingFullBtn.setObjectName("tracklingBtn")
		btnVLayout.addWidget(self.tracklingFullBtn)
		
		# ConvertFormat
		self.formatFullBtn = QtWidgets.QPushButton("Convert Label Format", self.full_menu_widget)
		self.formatFullBtn.setIcon(self.formatIcon)
		self.formatFullBtn.setIconSize(icon_size)
		self.formatFullBtn.setFont(font_size)
		self.formatFullBtn.setCheckable(True)
		self.formatFullBtn.setAutoExclusive(True)
		self.formatFullBtn.setObjectName("formatBtn")
		btnVLayout.addWidget(self.formatFullBtn)
		
		# CreateDataBase
		self.databaseFullBtn = QtWidgets.QPushButton("Create DataBase", self.full_menu_widget)
		self.databaseFullBtn.setIcon(self.databaseIcon)
		self.databaseFullBtn.setIconSize(icon_size)
		self.databaseFullBtn.setFont(font_size)
		self.databaseFullBtn.setCheckable(True)
		self.databaseFullBtn.setAutoExclusive(True)
		self.databaseFullBtn.setObjectName("databaseBtn")
		btnVLayout.addWidget(self.databaseFullBtn)
		menuVLayout.addLayout(btnVLayout)
		
		spacerItem = QtWidgets.QSpacerItem(20, 373, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
		menuVLayout.addItem(spacerItem)
		
		# Exit
		self.exitFullBtn = QtWidgets.QPushButton("Exit", self.full_menu_widget)
		self.exitFullBtn.setIcon(self.exitIcon)
		self.exitFullBtn.setIconSize(icon_size)
		self.exitFullBtn.setFont(font_size)
		menuVLayout.addWidget(self.exitFullBtn)
		self.gridLayout.addWidget(self.full_menu_widget, 0, 1, 1, 1)
