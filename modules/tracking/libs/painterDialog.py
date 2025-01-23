#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import math
import cv2
import argparse
import codecs
import os.path
import logging
import numpy as np
from functools import partial
from qtpy import QtCore, QtGui, QtWidgets
from typing import *

from ... import qdarkstyle
from ...logger import Logger
from ...labeling.libs.canvas import Canvas
from ...labeling.libs.colorDialog import ColorDialog
from ...labeling.libs.constants import *
from ...labeling.libs.create_ml_io import JSON_EXT, CreateMLReader
from ...labeling.libs.hashableQListWidgetItem import HashableQListWidgetItem
from ...labeling.libs.labelDialog import LabelDialog
from ...labeling.libs.labelFile import LabelFile, LabelFileError, LabelFileFormat
from ...labeling.libs.pascal_voc_io import XML_EXT, PascalVocReader
from ...labeling.libs.settings import Settings
from ...labeling.libs.shape import DEFAULT_FILL_COLOR, DEFAULT_LINE_COLOR, Shape
from ...labeling.libs.stringBundle import StringBundle
from ...labeling.libs.toolBar import ToolBar
from ...labeling.libs.ustr import ustr
from ...labeling.libs.utils import *
from ...labeling.libs.yolo_io import TXT_EXT, YoloReader
from ...labeling.libs.zoomWidget import ZoomWidget
from ...tracking.libs.tagBar import TagBar
from pages.resources.resources import *

TITLE = 'LabelPainter'
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
        # toolbar.setOrientation(QtCore.Qt.Vertical)
        toolbar.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        if actions:
            add_actions(toolbar, actions)
        return toolbar

class PainterDialog(QtWidgets.QDialog, WindowMixin):
    FIT_WINDOW, FIT_WIDTH, MANUAL_ZOOM = list(range(3))

    def __init__(self, default_filename: Optional[Union[str, np.ndarray]] = None, 
                       default_prefdef_class_file: Optional[str] = None, 
                       debug=None):
        super(PainterDialog, self).__init__()
        if (debug == None) :
            self.debug = Logger(None, logging.INFO, logging.INFO )
        else :
            self.debug = debug
        self.setWindowTitle(TITLE)
        self.setWindowIcon(QtGui.QIcon(":/app/trackingTool"))
        # Load setting in the main thread
        self.settings = Settings()
        self.settings.load()
        settings = self.settings

        # Load string bundle for i18n
        self.string_bundle = StringBundle.get_bundle()
        get_str = lambda str_id: self.string_bundle.get_string(str_id)

        # For loading all image under a directory
        self.dir_name = None
        self.cur_img_idx = 0
        self.img_count = 1

        # Whether we need to save or not.
        self.dirty = False
        
        self._beginner = True

        # Load predefined classes to the list
        label_hist = self.load_predefined_classes(default_prefdef_class_file)

        # Main widgets and related state.
        self.label_dialog = LabelDialog(parent=self, list_item=label_hist)

        self.items_to_shapes = {}
        self.shapes_to_items = {}
        self.prev_label_text = ''

        self.zoom_widget = ZoomWidget()
        self.zoom_widget.valueChanged.connect(self.paint_canvas)
        self.zoom_widget.setWhatsThis(
            u"Zoom in or out of the image. Also accessible with"
            " %s and %s from the canvas." % (format_shortcut("Ctrl+[-+]"),
                                             format_shortcut("Ctrl+Wheel")))
        self.zoom_widget.setEnabled(False)
        self.color_dialog = ColorDialog(parent=self)
        
        self.canvas = Canvas(parent=self)
        self.canvas.zoomRequest.connect(self.zoom_request)
        self.canvas.set_drawing_free_shape(True)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(self.canvas)
        scroll.setWidgetResizable(True)
        self.scroll_bars = {
            QtCore.Qt.Vertical: scroll.verticalScrollBar(),
            QtCore.Qt.Horizontal: scroll.horizontalScrollBar()
        }
        self.scroll_area = scroll
        self.canvas.scrollRequest.connect(self.scroll_request)
        self.canvas.newShape.connect(self.new_shape)
        self.canvas.selectionChanged.connect(self.shape_selection_changed)
        self.canvas.drawingPolygon.connect(self.toggle_drawing_sensitive)

        self.status_label = QtWidgets.QLabel()
        self.status_label.setStyleSheet("padding-left:8px;background:rgba(86,104,118,255);color:black;font-weight:bold;")
        self.status('%s started.' % TITLE)
        # self.setCentralWidget(scroll)
        
        action = partial(new_action, self)
        quit = action(get_str('quit'), self.close,
                      'Esc', 'quit', get_str('quitApp'), text_wrap=False)
        create_mode = action(get_str('crtBox'), self.set_create_mode,
                             'w', 'new', get_str('crtBoxDetail'), enabled=False, text_wrap=False)
        edit_mode = action(get_str('editBox'), self.set_edit_mode,
                           'Ctrl+J', 'edit', get_str('editBoxDetail'), enabled=False, text_wrap=False)

        create = action(get_str('crtBox'), self.create_shape,
                        'w', 'new', get_str('crtBoxDetail'), enabled=False, text_wrap=False)
        delete = action(get_str('delObj'), self.delete_selected_shape,
                        'Delete', 'delete', get_str('delObjDetail'), enabled=False, text_wrap=False)
        copy = action(get_str('dupObj'), self.copy_selected_shape,
                      'Ctrl+C', 'copy', get_str('dupObjDetail'),
                      enabled=False, text_wrap=False)
        color1 = action(get_str('boxLineColor'), self.choose_color1,
                        'Ctrl+L', 'color_line', get_str('boxLineColorDetail'))
        edit = action(get_str('editLabel'), self.edit_label,
                      'Ctrl+E', 'edit', get_str('editLabelDetail'),
                      enabled=False, text_wrap=False)
        shape_line_color = action(get_str('shapeLineColor'), self.choose_shape_line_color,
                                  icon='color_line', tip=get_str('shapeLineColorDetail'),
                                  enabled=False, text_wrap=False)
        shape_fill_color = action(get_str('shapeFillColor'), self.choose_shape_fill_color,
                                  icon='color', tip=get_str('shapeFillColorDetail'),
                                  enabled=False, text_wrap=False)
        
        zoom = QtWidgets.QWidgetAction(self)
        zoom.setDefaultWidget(self.zoom_widget)
        zoom_in = action(get_str('zoomin'), partial(self.add_zoom, 10),
                         'Ctrl++', 'zoom-in', get_str('zoominDetail'), enabled=False, text_wrap=False)
        zoom_out = action(get_str('zoomout'), partial(self.add_zoom, -10),
                          'Ctrl+-', 'zoom-out', get_str('zoomoutDetail'), enabled=False, text_wrap=False)
        zoom_org = action(get_str('originalsize'), partial(self.set_zoom, 100),
                          'Ctrl+=', 'zoom', get_str('originalsizeDetail'), enabled=False, text_wrap=False)
        fit_window = action(get_str('fitWin'), self.set_fit_window,
                            'Ctrl+F', 'fit-window', get_str('fitWinDetail'),
                            checkable=True, enabled=False, text_wrap=False)
        fit_width = action(get_str('fitWidth'), self.set_fit_width,
                           'Ctrl+Shift+F', 'fit-width', get_str('fitWidthDetail'),
                           checkable=True, enabled=False, text_wrap=False)
        # Group zoom controls into a list for easier toggling.
        zoom_actions = (self.zoom_widget, zoom_in, zoom_out,
                        zoom_org, fit_window, fit_width)
        self.zoom_mode = self.MANUAL_ZOOM
        self.scalers = {
            self.FIT_WINDOW: self.scale_fit_window,
            self.FIT_WIDTH: self.scale_fit_width,
            # Set to one to scale to 100% when loading files.
            self.MANUAL_ZOOM: lambda: 1,
        }

        # Label list context menu.
        label_menu = QtWidgets.QMenu()
        add_actions(label_menu, (edit, delete))
        
        # Store actions for further handling.
        self.actions = Struct(open=open, lineColor=color1, 
                              create=create, delete=delete, edit=edit, copy=copy,
                              createMode=create_mode, editMode=edit_mode, 
                              shapeLineColor=shape_line_color, shapeFillColor=shape_fill_color,
                              zoom=zoom, zoomIn=zoom_in, zoomOut=zoom_out, zoomOrg=zoom_org,
                              fitWindow=fit_window, fitWidth=fit_width,
                              zoomActions=zoom_actions,
                              beginner=(),
                              beginnerContext=(create, edit, copy, delete),)
        self.actions.beginner = (None, create, copy, delete, None, zoom, zoom_in, zoom_out, fit_window, fit_width, None, quit)
        
        # Custom context menu for the canvas widget:
        add_actions(self.canvas.menus[0], self.actions.beginnerContext)
        add_actions(self.canvas.menus[1], (
            action('&Copy here', self.copy_shape),
            action('&Move here', self.move_shape)))
        
        self.tools = self.toolbar('Tools')
        self.tools.clear()
        add_actions(self.tools, self.actions.beginner)
        
        classHLayout = QtWidgets.QHBoxLayout()
        self.tagLabel = QtWidgets.QLabel("Label Tags : ")
        self.tagLabel.setStyleSheet("color : rgb(255, 255, 255);")
        self.tagLabel.setFont(QtGui.QFont('Lucida', 10, QtGui.QFont.Bold))
        self.tagLabel.setMinimumHeight(5)
        classHLayout.addWidget(self.tagLabel)
        self.tagLineEdit = TagBar(self)
        self.tagLineEdit.load_tags(label_hist)
        self.tagLineEdit.setStyleSheet(" margin: 0px; padding: 0px; background-color: rgb(27,29,35);  border-radius: 15px; color : rgb(200, 200, 200);" )
        classHLayout.addWidget(self.tagLineEdit)
  
        self.canvas.menus[0].clear()
        add_actions(self.canvas.menus[0], self.actions.beginnerContext)

        # Application state.
        self.image = QtGui.QImage()
        self.file_path = ustr(default_filename)
        self.line_color = None
        self.fill_color = None
        self.zoom_level = 100
        self.fit_window = False
        # Add Chris
        self.difficult = False

        position = QtCore.QPoint(0, 0)
        saved_position = settings.get(SETTING_WIN_POSE, position)
        # Fix the multiple monitors issue
        for i in range(QtWidgets.QApplication.desktop().screenCount()):
            if QtWidgets.QApplication.desktop().availableGeometry(i).contains(saved_position):
                position = saved_position
                break

        screen = QtWidgets.QDesktopWidget().screenGeometry()
        width = int(screen.width() * 0.8)
        height = int(screen.height() * 0.8)
        self.resize(width, height)
        self.move(position)
        self.adjust_scale()

        # self.restoreState(settings.get(SETTING_WIN_STATE, QtCore.QByteArray()))
        Shape.difficult = self.difficult
        Shape.line_color = self.line_color = QtGui.QColor(settings.get(SETTING_LINE_COLOR, DEFAULT_LINE_COLOR))
        Shape.fill_color = self.fill_color = QtGui.QColor(settings.get(SETTING_FILL_COLOR, DEFAULT_FILL_COLOR))
        self.canvas.set_drawing_color(self.line_color)

        # Display cursor coordinates at the right of status bar
        self.label_coordinates = QtWidgets.QLabel('')
        # self.tools.addPermanentWidget(self.label_coordinates)
        # Since loading the file may take some time, make sure it runs in the background.
        # if self.file_path:
        #     self.queue_event(partial(self.load_file, self.file_path or ""))
        if isinstance(self.file_path, str):
            self.load_file(self.file_path)
        elif isinstance(self.file_path, np.ndarray):
            self.load_image(self.file_path)
            
        layout = QtWidgets.QVBoxLayout()
        footerHlayout = QtWidgets.QHBoxLayout()
        footerHlayout.addWidget(self.status_label)
        centralVlayout = QtWidgets.QVBoxLayout()
        centralVlayout.addWidget(self.tools)
        centralVlayout.addLayout(classHLayout)
        centralVlayout.addWidget(scroll, 1)
        layout.addLayout(centralVlayout)
        layout.addLayout(footerHlayout)
        self.setLayout(layout)
        
    def set_qdarkstyle(self):
        self.setStyleSheet(qdarkstyle.load_stylesheet(qt_api='pyqt5'))

    def setDebugLevel(self, level) :
        self.debug.changelevel(level)
        self.debug.debug("Change [ %s ] debug level." % TITLE )

    # Support Functions #
    def set_clean(self):
        self.actions.create.setEnabled(True)

    def toggle_actions(self, value=True):
        """Enable/Disable widgets which depend on an opened image."""
        for z in self.actions.zoomActions:
            z.setEnabled(value)

    # def queue_event(self, function):
    #     QtCore.QTimer.singleShot(0, function)

    def status(self, message):
        self.status_label.setText(message)

    def reset_state(self):
        self.items_to_shapes.clear()
        self.shapes_to_items.clear()
        self.file_path = None
        self.image_data = None
        self.label_file = None
        self.canvas.reset_state()
        self.label_coordinates.clear()

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
            self.update_combo_box()

    def beginner(self):
        return self._beginner

    def advanced(self):
        return not self.beginner()

    def create_shape(self):
        assert self.beginner()
        self.canvas.set_editing(False)
        self.actions.create.setEnabled(False)

    def toggle_drawing_sensitive(self, drawing=True):
        """In the middle of drawing, toggling between modes should be disabled."""
        self.actions.editMode.setEnabled(not drawing)
        if not drawing and self.beginner():
            # Cancel creation.
            self.debug.war('Cancel creation rect.')
            self.canvas.set_editing(True)
            self.canvas.restore_cursor()
            self.actions.create.setEnabled(True)

    def toggle_draw_mode(self, edit=True):
        self.canvas.set_editing(edit)
        self.actions.createMode.setEnabled(edit)
        self.actions.editMode.setEnabled(not edit)

    def set_create_mode(self):
        assert self.advanced()
        self.toggle_draw_mode(False)

    def set_edit_mode(self):
        assert self.advanced()
        self.toggle_draw_mode(True)

    # React to canvas signals.
    def shape_selection_changed(self, selected=False):
        shape = self.canvas.selected_shape
        if shape:
            self.shapes_to_items[shape].setSelected(True)
            points = [[point.x(), point.y()] for point in shape.points]
            self.status('Select - Width: %d, Height: %d '% (int(points[2][0] - points[0][0]), 
                                                            int(points[2][1] - points[0][1])) )
        self.actions.delete.setEnabled(selected)
        self.actions.copy.setEnabled(selected)
        self.actions.edit.setEnabled(selected)
        self.actions.shapeLineColor.setEnabled(selected)
        self.actions.shapeFillColor.setEnabled(selected)

    def copy_selected_shape(self):
        self.add_label(self.canvas.copy_selected_shape())
        # fix copy and delete
        self.shape_selection_changed(True)
        
    def add_label(self, shape):
        if shape is None:
            return
        shape.paint_label = shape.label
        item = HashableQListWidgetItem(shape.label)
        item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
        item.setCheckState(QtCore.Qt.Checked)
        item.setBackground(generate_color_by_text(shape.label))
        self.items_to_shapes[item] = shape
        self.shapes_to_items[shape] = item
        points = [[point.x(), point.y()] for point in shape.points]
        self.status('Add - Width: %d, Height: %d '% (int(points[2][0] - points[0][0]), 
                                                     int(points[2][1] - points[0][1])) )

    def remove_label(self, shape):
        if shape is None:
            # print('rm empty label')
            return
        item = self.shapes_to_items[shape]
        del self.shapes_to_items[shape]
        del self.items_to_shapes[item]

    def load_labels(self, shapes):
        s = []
        for label, type, points, line_color, fill_color, difficult in shapes:
            shape = Shape(label=label, shape_type=type)
            for x, y in points:

                # Ensure the labels are within the bounds of the image. If not, fix them.
                x, y, snapped = self.canvas.snap_point_to_canvas(x, y)
                shape.add_point(QtCore.QPointF(x, y))
            shape.difficult = difficult
            shape.label_font_size = self.canvas.label_font_size
            shape.close()
            s.append(shape)

            if line_color:
                shape.line_color = QtGui.QtGui.QColor(*line_color)
            else:
                shape.line_color = generate_color_by_text(label, alpha=255)
            
            if fill_color:
                shape.fill_color = QtGui.QtGui.QColor(*fill_color)
            else:
                shape.fill_color = generate_color_by_text(label)
            self.add_label(shape)
        self.canvas.load_shapes(s)
        
    def get_labels(self):
        def format_shape(s):
            return dict(label=s.label,
                        line_color=s.line_color.getRgb(),
                        fill_color=s.fill_color.getRgb(),
                        points=[(p.x(), p.y()) for p in s.points],
                        # add chris
                        difficult=s.difficult)

        shapes = []
        for shape in self.canvas.shapes:
            shape = format_shape(shape)
            xmin, ymin, xmax, ymax = LabelFile.convert_points_to_bnd_box(shape["points"])
            if not self.image.isNull():
                xmax, ymax = min(xmax, self.image.width()-1), min(ymax, self.image.height()-1)
            shapes.append([shape["label"], np.array([xmin, ymin, xmax-xmin, ymax-ymin]) ])
        return shapes

    # Callback functions:
    def new_shape(self):
        """Pop-up and give focus to the label editor.

        position MUST be in global coordinates.
        """
        if len(self.tagLineEdit.tags) > 0:
            self.label_dialog = LabelDialog(
                parent=self, list_item=self.tagLineEdit.tags)
            self.label_dialog.edit.hide()
            self.label_dialog.button_box.hide()
            
            text = self.label_dialog.pop_up(text=self.prev_label_text)
            self.lastLabel = text
        else:
            QtWidgets.QMessageBox.critical(self, "Error", f"Label tags is empty, can't create new shape")
            self.canvas.shapes.pop()
            return
   
        # Add Chris
        if text is not None:
            self.prev_label_text = text
            line_color = generate_color_by_text(text, alpha=255)
            fill_color = generate_color_by_text(text)
            shape = self.canvas.set_last_label(text, line_color, fill_color)
            self.add_label(shape)
            if self.beginner():  # Switch to edit mode.
                self.canvas.set_editing(True)
                self.actions.create.setEnabled(True)
            else:
                self.actions.editMode.setEnabled(True)
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

    def counter_str(self):
        """
        Converts image counter to string representation.
        """
        return '[{} / {}]'.format(self.cur_img_idx + 1, self.img_count)

    def resizeEvent(self, event):
        if self.canvas and not self.image.isNull()\
           and self.zoom_mode != self.MANUAL_ZOOM:
            self.adjust_scale()
        super(PainterDialog, self).resizeEvent(event)

    def paint_canvas(self):
        if self.image.isNull() :
            self.debug.war("Cannot paint null image")
        else :
            assert not self.image.isNull(), "cannot paint null image"
            self.canvas.scale = 0.01 * self.zoom_widget.value()
            self.canvas.label_font_size = int(0.02 * min(self.image.width(), self.image.height()))
            self.canvas.adjustSize()
            self.canvas.update()

    def adjust_scale(self, initial=False):
        value = self.scalers[self.FIT_WINDOW if initial else self.zoom_mode]()
        self.zoom_widget.setValue(int(100 * value))

    def scale_fit_window(self):
        """Figure out the size of the pixmap in order to fit the main widget."""
        e = 50.0  # So that no scrollbars are generated.
        w1 = self.width() - e
        h1 = self.height() - e
        a1 = w1 / h1
        # Calculate a new scale value based on the pixmap's aspect ratio.
        w2 = self.canvas.pixmap.width() - 0.0
        h2 = self.canvas.pixmap.height() - 0.0
        a2 = w2 / h2
        return w1 / w2 if a2 >= a1 else h1 / h2

    def scale_fit_width(self):
        # The epsilon does not seem to work too well here.
        w = self.width() - 2.0
        return w / self.canvas.pixmap.width()

    def closeEvent(self, event):
        # self.dirty = True
        if not self.may_continue():
            event.ignore()

    def load_image(self, img):
        h, w, ch = img.shape
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        else:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_bytes = img.tobytes()
        
        qimg = QtGui.QImage(img_bytes, w, h, w * ch, QtGui.QImage.Format_RGB888) 
        image = QtGui.QPixmap.fromImage(qimg)
        if image.isNull():
            self.status("Error reading image!")
            return False
        self.image = image
        self.file_path = None
        self.canvas.load_pixmap(image)
        self.set_clean()
        self.canvas.setEnabled(True)
        self.adjust_scale(initial=True)
        self.paint_canvas()
        self.toggle_actions(True)
        self.canvas.setFocus(True)
        return True
    
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
            self.toggle_actions(True)
            counter = self.counter_str()
            self.setWindowTitle(TITLE + ' ' + file_path + ' ' + counter)
            self.canvas.setFocus(True)
            return True
        return False
    
    def open_file(self, _value=False):
        if not self.may_continue():
            return
        path = os.path.dirname(ustr(self.file_path)) if self.file_path else '.'
        formats = ['*.%s' % fmt.data().decode("ascii").lower() for fmt in QtGui.QImageReader.supportedImageFormats()]
        filters = "Image & Label files (%s)" % ' '.join(formats + ['*%s' % LabelFile.suffix])
        filename = QtWidgets.QFileDialog.getOpenFileName(self, '%s - Choose Image or Label file' % TITLE, path, filters)
        if filename:
            if isinstance(filename, (tuple, list)):
                filename = filename[0]
            self.cur_img_idx = 0
            self.img_count = 1
            self.load_file(filename)

    def close_file(self, _value=False):
        if not self.may_continue():
            return
        self.reset_state()
        self.set_clean()
        self.toggle_actions(False)
        self.canvas.setEnabled(False)
        self.actions.saveAs.setEnabled(False)

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
            if discard_changes == QtWidgets.QMessageBox.Yes:
                # self.save_file()
                return True
            else:
                return False

    def discard_changes_dialog(self):
        yes, cancel = QtWidgets.QMessageBox.Yes, QtWidgets.QMessageBox.Cancel
        msg = u"Are you sure you want to finish the modifications?"
        return QtWidgets.QMessageBox.warning(self, u'Attention', msg, yes | cancel)

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

    def delete_selected_shape(self):
        self.remove_label(self.canvas.delete_selected())

    def choose_shape_line_color(self):
        color = self.color_dialog.getColor(self.line_color, u'Choose Line Color',
                                           default=DEFAULT_LINE_COLOR)
        if color:
            self.canvas.selected_shape.line_color = color
            self.canvas.update()

    def choose_shape_fill_color(self):
        color = self.color_dialog.getColor(self.fill_color, u'Choose Fill Color',
                                           default=DEFAULT_FILL_COLOR)
        if color:
            self.canvas.selected_shape.fill_color = color
            self.canvas.update()

    def copy_shape(self):
        self.canvas.end_move(copy=True)
        self.add_label(self.canvas.selected_shape)

    def move_shape(self):
        self.canvas.end_move(copy=False)
    
    def load_predefined_classes(self, predef_classes_file):
        label_hist = []
        if os.path.exists(predef_classes_file) is True:
            with codecs.open(predef_classes_file, 'r', 'utf8') as f:
                for line in f:
                    line = line.strip()
                    if label_hist is []:
                        label_hist = [line]
                    else:
                        label_hist.append(line)
            self.classes_file = predef_classes_file
        else :
            self.classes_file = None
        return label_hist

    def load_bbox_by_list(self, bbox_list):
        shapes = []
        for label, box in bbox_list:
            '''
            box format: x, y, width, height
            '''
            x_min = int(box[0])
            x_max = int(box[0] + box[2])
            y_min = int(box[1])
            y_max = int(box[1] + box[3])
            points = [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]
            shapes.append((label, "rectangle", points, None, None, False))
        self.debug.debug("List shape : " + str(shapes))
        self.load_labels(shapes)

def read(filename, default=None):
    try:
        reader =  QtGui.QImageReader(filename)
        reader.setAutoTransform(True)
        return reader.read()
    except:
        return default

if __name__ == '__main__':
    """
    Standard boilerplate Qt application code.
    Do everything but app.exec_() -- so that we can test the application in one thread
    """
    argparser = argparse.ArgumentParser()
    argparser.add_argument("-i", "--image_file", default="./demo/trackingSelectUI.png", nargs="?")
    argparser.add_argument("-c", "--class_file",
                           default= os.path.join(os.path.dirname(__file__), 'default_classes.txt'),
                           nargs="?")
    args = argparser.parse_args(sys.argv[1:])

    args.image_file = args.image_file and os.path.normpath(args.image_file)
    args.class_file = args.class_file and os.path.normpath(args.class_file)
    # image = cv2.imread(args.image_file)

    app = QtWidgets.QApplication(sys.argv)
    win = PainterDialog(args.image_file,
                     args.class_file,
                     debug=debug)
    win.set_qdarkstyle()
    # win.load_bbox_by_list([ ["people", [59., 75., 170., 180.]] ])
    win.exec()
    print(win.get_labels())

