
#! /usr/bin/env python
#coding=gbk
import logging
import ctypes
from qtpy import QtCore, QtGui, QtWidgets


FOREGROUND_WHITE = "\x1b[38;20m"
FOREGROUND_BLUE = "\x1b[34;20m" # text color contains blue.
FOREGROUND_GREEN = "\x1b[32;20m" # text color contains green.
FOREGROUND_RED  = "\x1b[31;20m" # text color contains red.
FOREGROUND_BOLD_RED  = "\x1b[31;1m" 
FOREGROUND_YELLOW = "\x1b[33;20m"
 
class LogFormatter(logging.Formatter):
    reset = "\x1b[0m"
    format = "[%(asctime)s] [%(levelname)s] %(message)s "

    FORMATS = {
        logging.DEBUG: FOREGROUND_WHITE + format + reset,
        logging.INFO: FOREGROUND_BLUE + format + reset,
        logging.WARNING: FOREGROUND_YELLOW + format + reset,
        logging.ERROR: FOREGROUND_RED + format + reset,
        logging.CRITICAL: FOREGROUND_BOLD_RED + format + reset
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)
 
class Logger:
    def __init__(self, path, clevel = logging.DEBUG, Flevel = logging.DEBUG):
        self.clevel = clevel
        self.logger = logging.getLogger(path)
        self.logger.setLevel(clevel)
        fmt = logging.Formatter('[%(asctime)s] [%(levelname)s] %(message)s', '%Y-%m-%d %H:%M:%S')
        
        # 先判断logger是否存在文件句柄，若不存在则进行创建，否则会出现日志重复打印的问题
        if not self.logger.handlers:
            #設置CMD日誌
            self.sh = logging.StreamHandler()
            self.sh.setFormatter(LogFormatter())
            # self.sh.setFormatter(fmt)
            self.logger.addHandler(self.sh)
            #設置文件日誌
            if (path != None) :
                fh = logging.FileHandler(path)
                fh.setFormatter(fmt)
                fh.setLevel(Flevel)
                self.logger.addHandler(fh)

    def changelevel(self, clevel) :
        self.clevel = clevel
        self.logger.setLevel(clevel)

    def debug(self,message):
        self.logger.debug(message)

    def info(self,message,color=FOREGROUND_BLUE):
        # set_color(color)
        self.logger.info(message)
        # set_color(FOREGROUND_WHITE)

    def war(self,message,color=FOREGROUND_YELLOW):
        # set_color(color)
        self.logger.warning(message)
        # set_color(FOREGROUND_WHITE)

    def error(self,message,color=FOREGROUND_RED):
        # set_color(color)
        self.logger.error(message)
        # set_color(FOREGROUND_WHITE)

    def cri(self,message):
        self.logger.critical(message)

class DialogFormatter(logging.Formatter):
    FORMATS = {
        logging.ERROR:   ("[%(asctime)s] [%(levelname)s] %(message)s", '%Y-%m-%d %H:%M:%S', "red"),
        logging.DEBUG:   ("[%(asctime)s] [%(levelname)s] %(message)s", '%Y-%m-%d %H:%M:%S', "gray"),
        logging.INFO:    ("[%(asctime)s] [%(levelname)s] %(message)s", '%Y-%m-%d %H:%M:%S', 'blue'),
        logging.WARNING: ('[%(asctime)s] [%(levelname)s] %(message)s', '%Y-%m-%d %H:%M:%S', 'gold')
    }

    def format( self, record ):
        last_fmt = self._style._fmt
        opt = DialogFormatter.FORMATS.get(record.levelno)
        if opt:
            fmt, format, color = opt
            self.datefmt = format
            self._style._fmt = "<font color=\"{}\">{}</font>".format(QtGui.QColor(color).name(),fmt)
        res = logging.Formatter.format( self, record )
        self._style._fmt = last_fmt
        return res

class QPlainTextEditLogger(logging.Handler):
    def __init__(self, parent=None):
        super().__init__()
        self.widget = QtWidgets.QPlainTextEdit(parent)
        self.widget.setFont(QtGui.QFont("Times", 10, QtGui.QFont.Bold))
        self.widget.setStyleSheet('background-color: darkgray; border: 2px solid black;')
        self.widget.setReadOnly(True)  
        
    def emit(self, record):
        msg = self.format(record)
        self.widget.appendHtml(msg) 
        # move scrollbar
        scrollbar = self.widget.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

class LoggerDialog(QtWidgets.QDialog, QtWidgets.QPlainTextEdit):
    def __init__(self, debug=None, clevel = logging.INFO, parent=None):
        super().__init__(parent)
        self.Processing = True
        self.debug = debug

        #获取显示器分辨率
        desktop = QtWidgets.QApplication.desktop()
        screenRect = desktop.screenGeometry()
        screenheight = screenRect.height()
        screenwidth = screenRect.width()
        
        windowheight = int(screenheight * 0.38)
        windowwidth = int(screenwidth * 0.35)
        self.resize(windowwidth, windowheight)

        self.setStyleSheet('background-color: rgb(61, 68, 85);')
        self.setWindowTitle("Log")
        # You can control the logging level
        if (self.debug != None) :
            clevel = self.debug.clevel
        logging.getLogger().setLevel(clevel)
        self.logTextBox = QPlainTextEditLogger(self)
        # You can format what is printed to text box
        self.logTextBox.setFormatter(DialogFormatter())
        logging.getLogger().addHandler(self.logTextBox)

        horizontal_layout = QtWidgets.QHBoxLayout()
        horizontal_layout.addStretch(1)
        clevel_label = QtWidgets.QLabel("Level : ")
        clevel_label.setStyleSheet("color: rgb(220, 220, 220);")
        clevel_label.setFont(QtGui.QFont('Lucida', 12, QtGui.QFont.Bold))
        horizontal_layout.addWidget(clevel_label)

        self.combox = QtWidgets.QComboBox()
        self.combox.setStyleSheet("QComboBox { background: lightgray; } QComboBox QAbstractItemView { border: 1px solid grey; background: lightgray; selection-background-color: blue; }")
        self.combox.setFocusPolicy(QtCore.Qt.NoFocus)
        self.combox.setFixedHeight(25)
        self.combox.setEditable(True)
        self.combox.setFont(QtGui.QFont('Lucida', 10))
        level_list = ['DEBUG', 'INFO','WARNING','ERROR']
        self.combox.addItems(level_list)
        current_level_index =  level_list.index(logging._levelToName.get(clevel))
        self.combox.setCurrentIndex(current_level_index )
        self.combox.currentIndexChanged.connect(self._onCurrentIndexChanged)
        ledit = self.combox.lineEdit()
        ledit.setAlignment(QtCore.Qt.AlignCenter)
        ledit.setReadOnly(True)
        horizontal_layout.addWidget(self.combox)

        layout = QtWidgets.QVBoxLayout()
        # Add the new logging box widget to the layout
        layout.addWidget(self.logTextBox.widget)
        layout.addLayout(horizontal_layout)
        # layout.addWidget(self._button)
        self.setLayout(layout)

    def _onCurrentIndexChanged(self, i) :
        clevel = logging._nameToLevel.get(self.combox.currentText())
        logging.getLogger().setLevel(clevel)

    def closeEvent(self, event):
        if (self.Processing) :
            if self.isVisible():
                self.hide()
            else :
                self.show()
        else :
            pass
        event.accept()

    def release(self):
        self.Processing = False
        self.close()

if __name__ =='__main__':
    import sys
    # app = QtWidgets.QApplication(sys.argv)
    # dlg = LoggerDialog()
    # logging.info("test")
    # dlg.show()
    # dlg.raise_()
    # sys.exit(app.exec_())

    debug = Logger(None,logging.DEBUG,logging.DEBUG)
    debug.debug('一個debug信息')
    debug.info('一個info信息')
    debug.war('一個warning信息')
    debug.error('一個error信息')
    debug.cri('一個致命critical信息')