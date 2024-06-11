import os
import codecs
from qtpy import QtCore, QtGui, QtWidgets
from functools import partial

from resources.resources  import *

class DraggableTag(QtWidgets.QFrame):
    def __init__(self, text, parent=None):
        super(DraggableTag, self).__init__(parent)
        self.parent = parent
        self.original_style = 'border: 1px solid rgb(29, 233, 182); border-radius: 8px; background-color: rgb(34, 43, 39);'
        self.dragging_style = 'border: 1px solid rgb(29, 233, 182); border-radius: 8px; background-color: rgb(54, 65, 60);'
        
        self.setStyleSheet(self.original_style)
        self.setContentsMargins(2, 2, 2, 2)
        self.setFixedHeight(28)
        self.setSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Preferred)
        
        self.tagHlayout = QtWidgets.QHBoxLayout()
        self.tagHlayout.setContentsMargins(4, 4, 4, 4)
        self.tagHlayout.setSpacing(10)
        
        self.label = QtWidgets.QLabel(text)
        self.label.setStyleSheet('border:0px')
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.tagHlayout.addWidget(self.label)
        
        self.button = QtWidgets.QPushButton()
        self.button.setIcon(QtGui.QIcon(':/delete'))
        self.button.setFixedSize(20, 15)
        self.button.setStyleSheet('border:0px; font-weight:bold')
        self.button.setSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Maximum)
        self.button.clicked.connect(partial(parent.delete_tag, text))
        self.tagHlayout.addWidget(self.button)
        
        self.setLayout(self.tagHlayout)
        
        self.setAcceptDrops(True)
        self.setMouseTracking(True)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.drag_start_position = event.pos()
            self.setStyleSheet(self.dragging_style)

    def mouseMoveEvent(self, event):
        if not (event.buttons() & QtCore.Qt.LeftButton):
            return
        if (event.pos() - self.drag_start_position).manhattanLength() < QtWidgets.QApplication.startDragDistance():
            return
        drag = QtGui.QDrag(self)
        mime_data = QtCore.QMimeData()
        drag.setMimeData(mime_data)
        drag.exec_(QtCore.Qt.MoveAction)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.setStyleSheet(self.original_style)
        
    def dragEnterEvent(self, event):
        event.accept()

    def dropEvent(self, event):
        event.setDropAction(QtCore.Qt.MoveAction)
        event.accept()
        source = event.source()
        target_layout = self.parentWidget().tagsHlayout
        target_index = target_layout.indexOf(self)
        target_layout.insertWidget(target_index, source)
        source.setStyleSheet(self.original_style)
        
        # Update self.tags order
        widget_list = []
        for i in range(target_layout.count()):
            widget = target_layout.itemAt(i).widget()
            if isinstance(widget, DraggableTag):
                widget_list.append(widget.label.text())
        self.parent.tags = widget_list
        self.parent.refresh()
    
class TagBar(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(TagBar, self).__init__(parent)
        self.parent = parent
        self.tags = []
        self.tagsHlayout = QtWidgets.QHBoxLayout()
        self.tagsHlayout.setAlignment(QtCore.Qt.AlignLeft)
        self.tagsHlayout.setSpacing(4)
        
        self.line_edit = QtWidgets.QLineEdit()
        self.line_edit.setFixedSize(100, 28)
        self.line_edit.setStyleSheet('border-radius: 8px;')
        self.line_edit.setSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Maximum)
        self.line_edit.returnPressed.connect(self.create_tags)
        self.line_edit.installEventFilter(self)
        self.setSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        self.setContentsMargins(2, 2, 2, 2)
        self.tagsHlayout.setContentsMargins(2, 2, 2, 2)
        self.refresh()
        self.setLayout(self.tagsHlayout)

    def eventFilter(self, source, event):
        if event.type() == QtCore.QEvent.FocusOut and source is self.line_edit:
            if self.line_edit.text().strip():
                self.create_tags()
        return super(TagBar, self).eventFilter(source, event)
    
    def load_tags(self, tags):
        if tags is not None:
            if isinstance(tags, list):
                self.tags = [tag.strip() for tag in tags if tag.strip()]
            else:
                self.tags.append(tags.strip())
            self.refresh()

    def delete_tag(self, tag_name):
        self.tags.remove(tag_name)
        self.refresh()

    def create_tags(self):
        new_tags = [line for line in self.line_edit.text().split(', ') 
                    if line.strip()]
        self.line_edit.setText('')
        temp_tags = [ tag for tag in new_tags if tag not in self.tags]
        self.tags.extend(temp_tags)
        self.refresh()
        
    def refresh(self):
        for i in reversed(range(self.tagsHlayout.count())):
            self.tagsHlayout.itemAt(i).widget().setParent(None)

        for tag in self.tags:
            self.add_tag_to_bar(tag)
        self.tagsHlayout.addWidget(self.line_edit)
        self.line_edit.setFocus()
        
        if isinstance(self.parent.classes_file, str):
            if os.path.exists(self.parent.classes_file) is True:
                with codecs.open(self.parent.classes_file, 'w', 'utf8') as f:
                    for tag in self.tags:
                        f.write(tag+'\n')
            
    def add_tag_to_bar(self, text):
        tag = DraggableTag(text, self)
        self.tagsHlayout.addWidget(tag)
