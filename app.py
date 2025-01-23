import sys
import logging
from pathlib import Path
from qtpy import QtCore, QtGui, QtWidgets

from pages import labelingTool, trackingTool, convertFormat, createDataBase
from pages.guideSideBar import MainWindow, AnnotatorType
from modules import qdarkstyle
from modules.tracking.libs.style import TABLE_QSS, BTN_QSS
from modules.logger import Logger

'''
# Bundles Python Application (For linux)
$ pyinstaller --paths=./Annotator/modules/:modules  --add-data=./modules/style.qss:modules --add-data=./modules/gdino/*.json:modules/gdino --add-data=./demo/*:demo  -F -w app.py --icon=./pages/resources/icons/Logo.ico
$ ./app

# Bundles Python Application (For windows)
> pyinstaller --paths=./Annotator/modules/;modules  --add-data=./modules/style.qss;modules --add-data=./modules/gdino/*.json;modules/gdino --add-data=./demo/*:demo -F -w app.py --icon=./pages/resources/icons/Logo.ico
'''

debug = Logger(None, logging.INFO, logging.INFO )

class GuideWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(GuideWindow, self).__init__()
        self.ui = MainWindow()
        self.ui.setupUi(self)
        self.initBtnPageUI()
        
        self.ui.icon_menu_widget.hide()
        self.ui.stackedWidget.setCurrentIndex(0)
        self.ui.labelingFullBtn.setChecked(True)

    def initBtnPageUI(self):
        if not Path('default_classes.txt').is_file():
            with open('default_classes.txt', 'w') as f:
                pass

        # LabelingTool
        self.labelingPage = QtWidgets.QWidget()
        self.labelingGridLayout = QtWidgets.QGridLayout(self.labelingPage)
        labelingApp = labelingTool.MainWidget('default_classes.txt', debug)
        self.labelingGridLayout.addWidget(labelingApp, 0, 0, 1, 1)
        self.ui.stackedWidget.addWidget(self.labelingPage)
        
        # TrackingTool
        self.trackingPage = QtWidgets.QWidget()
        self.trackingGridLayout = QtWidgets.QGridLayout(self.trackingPage)
        trackingApp = trackingTool.MainWidget('default_classes.txt', debug)
        self.trackingGridLayout.addWidget(trackingApp, 0, 0, 1, 1)
        self.ui.stackedWidget.addWidget(self.trackingPage)
        
        # ConvertFormat 
        self.formatPage = QtWidgets.QWidget()
        self.formatGridLayout = QtWidgets.QGridLayout(self.formatPage)
        formatApp = convertFormat.MainWidget('default_classes.txt', debug)
        self.formatGridLayout.addWidget(formatApp, 0, 0, 1, 1)
        self.ui.stackedWidget.addWidget(self.formatPage)
        
        # CreateDataBase (TODO: not done)
        self.databasePage = QtWidgets.QWidget()
        self.databaseGridLayout = QtWidgets.QGridLayout(self.databasePage)
        databaseApp = createDataBase.MainWidget('default_classes.txt', debug)
        self.databaseGridLayout.addWidget(databaseApp, 0, 0, 1, 1)
        self.ui.stackedWidget.addWidget(self.databasePage)
    
    ## Change QPushButton Checkable status when stackedWidget index changed
    def on_stackedWidget_currentChanged(self, index):
        btn_list = self.ui.icon_menu_widget.findChildren(QtWidgets.QPushButton) \
                    + self.ui.full_menu_widget.findChildren(QtWidgets.QPushButton)
        
        for btn in btn_list:
            if index in [5, 6]:
                btn.setAutoExclusive(False)
                btn.setChecked(False)
            else:
                btn.setAutoExclusive(True)
            
    ## functions for changing menu page
    def on_labelingBtn_toggled(self):
        self.ui.stackedWidget.setCurrentIndex(0)

    def on_tracklingBtn_toggled(self):
        self.ui.stackedWidget.setCurrentIndex(1)

    def on_formatBtn_toggled(self):
        self.ui.stackedWidget.setCurrentIndex(2)

    def on_databaseBtn_toggled(self):
        self.ui.stackedWidget.setCurrentIndex(3)


def main(argv=[]):
    app = QtWidgets.QApplication(sys.argv)

    # Load the qdarkstyle stylesheet
    dark_stylesheet = qdarkstyle.load_stylesheet_pyqt5()
    # Load the custom stylesheet
    style_file = QtCore.QFile(str(Path(__file__).resolve().parent / "modules/style.qss"))
    style_file.open(QtCore.QFile.ReadOnly | QtCore.QFile.Text)
    custom_stylesheet = QtCore.QTextStream(style_file).readAll()
    style_file.close()
    combined_stylesheet = dark_stylesheet + "\n" + custom_stylesheet
    app.setStyleSheet(combined_stylesheet)

    # Create and show the main window
    window = GuideWindow()
    window.show()
    return app.exec()

if __name__ == "__main__":
    import argparse
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        "--task", '-t', 
        default=None, 
        type=AnnotatorType.argparse_type, 
        choices=list(AnnotatorType), 
        help='Choose a task type. Valid options: ' + ', '.join([t.name for t in AnnotatorType])
    )
    args, unknown = argparser.parse_known_args() # Parse known parameters and ignore unknown parameters

    task: AnnotatorType = args.task
    if task:
        debug.info(f'Selected task: {task.name}')
        argv = [sys.argv[0]] + unknown if unknown else [sys.argv[0]]
        debug.info(f"Final argv: {argv}")
        
        if task == AnnotatorType.Labeling:
            labelingTool.main(argv)
        elif task == AnnotatorType.Tracking:
            trackingTool.main(argv)
        elif task == AnnotatorType.Convert:
            convertFormat.main(argv)
        elif task == AnnotatorType.Create:
            createDataBase.main(argv)
        else:
            debug.error(f"Unsupported task type: {task.name}")
    else:
        debug.info('No task provided. Running default guide.')
        sys.exit(main(sys.argv))





