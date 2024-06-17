from qtpy import QtCore, QtGui, QtWidgets

from .shape import Shape
from .utils import distance
import math 

CURSOR_DEFAULT = QtCore.Qt.ArrowCursor
CURSOR_POINT = QtCore.Qt.PointingHandCursor
CURSOR_DRAW = QtCore.Qt.CrossCursor
CURSOR_MOVE = QtCore.Qt.ClosedHandCursor
CURSOR_GRAB = QtCore.Qt.OpenHandCursor

# class Canvas(QGLWidget):

class Canvas(QtWidgets.QWidget):
    zoomRequest = QtCore.Signal(int)
    scrollRequest = QtCore.Signal(int, int)
    newShape = QtCore.Signal()
    selectionChanged = QtCore.Signal(bool)
    shapeMoved = QtCore.Signal()
    drawingPolygon = QtCore.Signal(bool)

    CREATE, EDIT = list(range(2))

    epsilon = 11.0

    def __init__(self, *args, **kwargs):
        super(Canvas, self).__init__(*args, **kwargs)
        # Initialise local state.
        self.debug = self.parent().debug
        self.mode = self.EDIT
        self.shapes = []
        self.current = None
        self.selected_shape = None  # save the selected shape here
        self.selected_shape_copy = None
        self.drawing_line_color = QtGui.QColor(0, 0, 255)
        self.drawing_rect_color = QtGui.QColor(0, 0, 255)
        self.line = Shape(line_color=self.drawing_line_color)
        self.prev_point = QtCore.QPointF()
        self.offsets = QtCore.QPointF(), QtCore.QPointF()
        self.scale = 1.0
        self.label_font_size = 8
        self.pixmap = QtGui.QPixmap()
        self.visible = {}
        self._hide_background = False
        self.hide_background = False
        self.h_shape = None
        self.h_vertex = None
        self._painter = QtGui.QPainter()
        self._cursor = CURSOR_DEFAULT
        # Menus:
        self.menus = (QtWidgets.QMenu(), QtWidgets.QMenu())
        # Set widget options.
        self.setMouseTracking(True)
        self.setFocusPolicy(QtCore.Qt.WheelFocus)
        self.verified = False
        self.draw_free_size = True
        self.Width_Height = (None, None)
        self.WH_Ratio = 1

        # initialisation for panning
        self.pan_initial_pos = QtCore.QPoint()

    def set_drawing_color(self, QColor):
        self.drawing_line_color = QColor
        self.drawing_rect_color = QColor

    def enterEvent(self, ev):
        self.override_cursor(self._cursor)

    def leaveEvent(self, ev):
        self.restore_cursor()

    def focusOutEvent(self, ev):
        self.restore_cursor()

    def isVisible(self, shape):
        return self.visible.get(shape, True)

    def drawing(self):
        return self.mode == self.CREATE

    def editing(self):
        return self.mode == self.EDIT

    def set_editing(self, value=True):
        self.mode = self.EDIT if value else self.CREATE
        if not value:  # Create
            self.un_highlight()
            self.de_select_shape()
        self.prev_point = QtCore.QPointF()
        self.repaint()

    def un_highlight(self):
        if self.h_shape:
            self.h_shape.highlight_clear()
        self.h_vertex = self.h_shape = None

    def selected_vertex(self):
        return self.h_vertex is not None

    def mouseMoveEvent(self, ev):
        """Update line with last point and current coordinates."""
        pos = self.transform_pos(ev.pos())

        # Update coordinates in status bar if image is opened
        window = self.parent().window()
        if window.file_path is not None:
            self.parent().window().label_coordinates.setText(
                'X: %d; Y: %d' % (pos.x(), pos.y()))

        # Polygon drawing.
        if self.drawing():
            self.override_cursor(CURSOR_DRAW)
            if self.current:
                # Display annotation width and height while drawing
                current_width = abs(self.current[0].x() - pos.x())
                current_height = abs(self.current[0].y() - pos.y())
                self.parent().window().label_coordinates.setText(
                        'Width: %d, Height: %d / X: %d; Y: %d' % (current_width, current_height, pos.x(), pos.y()))

                color = self.drawing_line_color
                if self.out_of_pixmap(pos):
                    # Don't allow the user to draw outside the pixmap.
                    # Clip the coordinates to 0 or max,
                    # if they are outside the range [0, max]
                    size = self.pixmap.size()
                    clipped_x = min(max(0, pos.x()), size.width())
                    clipped_y = min(max(0, pos.y()), size.height())
                    pos = QtCore.QPointF(clipped_x, clipped_y)
                elif len(self.current) > 1 and self.close_enough(pos, self.current[0]):
                    # Attract line to starting point and colorise to alert the
                    # user:
                    pos = self.current[0]
                    color = self.current.line_color
                    self.override_cursor(CURSOR_POINT)
                    self.current.highlight_vertex(0, Shape.NEAR_VERTEX)

                if self.draw_free_size:
                    self.line[1] = pos
                else:
                    init_pos = self.current[0]
                    min_x = init_pos.x()
                    min_y = init_pos.y()
                    direction_x = -1 if pos.x() - min_x < 0 else 1
                    direction_y = -1 if pos.y() - min_y < 0 else 1

                    if (self.WH_Ratio >= 1.0) :
                        min_size = min(abs(pos.x() - min_x), abs(pos.y() - min_y))
                        self.line[1] = QtCore.QPointF(int(min_x + direction_x * min_size), int(min_y + direction_y * min_size * self.WH_Ratio))
                    else :
                        max_size = max(abs(pos.x() - min_x), abs(pos.y() - min_y))
                        self.line[1] = QtCore.QPointF(int(min_x + direction_x * max_size), int(min_y + direction_y * max_size * self.WH_Ratio))

                self.line.line_color = color
                self.prev_point = QtCore.QPointF()
                self.current.highlight_clear()
            else:
                self.prev_point = pos
            self.repaint()
            return

        # Polygon copy moving.
        if QtCore.Qt.RightButton & ev.buttons():
            if self.selected_shape_copy and self.prev_point:
                self.override_cursor(CURSOR_MOVE)
                self.bounded_move_shape(self.selected_shape_copy, pos)
                self.repaint()
            elif self.selected_shape:
                self.selected_shape_copy = self.selected_shape.copy()
                self.repaint()
            return

        # Polygon/Vertex moving.
        if QtCore.Qt.LeftButton & ev.buttons():
            if self.selected_vertex():
                self.bounded_move_vertex(pos)
                self.shapeMoved.emit()
                self.repaint()

                # Display annotation width and height while [moving vertex]
                point1 = self.h_shape[1]
                point3 = self.h_shape[3]
                current_width = abs(point1.x() - point3.x())
                current_height = abs(point1.y() - point3.y())
                self.parent().window().label_coordinates.setText(
                        'Width: %d, Height: %d / X: %d; Y: %d' % (current_width, current_height, pos.x(), pos.y()))
            elif self.selected_shape and self.prev_point:
                self.override_cursor(CURSOR_MOVE)
                self.bounded_move_shape(self.selected_shape, pos)
                self.shapeMoved.emit()
                self.repaint()

                # Display annotation width and height while [moving shape]
                point1 = self.selected_shape[1]
                point3 = self.selected_shape[3]
                current_width = abs(point1.x() - point3.x())
                current_height = abs(point1.y() - point3.y())
                self.parent().window().label_coordinates.setText(
                        'Width: %d, Height: %d / X: %d; Y: %d' % (current_width, current_height, pos.x(), pos.y()))
            else:
                # pan
                delta_x = pos.x() - self.pan_initial_pos.x()
                delta_y = pos.y() - self.pan_initial_pos.y()
                self.scrollRequest.emit(delta_x, QtCore.Qt.Horizontal)
                self.scrollRequest.emit(delta_y, QtCore.Qt.Vertical)
                self.update()
            return

        # Just hovering over the canvas, 2 possibilities:
        # - Highlight shapes
        # - Highlight vertex
        # Update shape/vertex fill and tooltip value accordingly.
        self.setToolTip("Image")
        for shape in reversed([s for s in self.shapes if self.isVisible(s)]):
            # Look for a nearby vertex to highlight. If that fails,
            # check if we happen to be inside a shape.
            index = shape.nearest_vertex(pos, self.epsilon)
            if index is not None:
                if self.selected_vertex():
                    self.h_shape.highlight_clear()
                self.h_vertex, self.h_shape = index, shape
                shape.highlight_vertex(index, shape.MOVE_VERTEX)
                self.override_cursor(CURSOR_POINT)
                self.setToolTip("Click & drag to move point")
                self.setStatusTip(self.toolTip())
                self.update()
                break
            elif shape.contains_point(pos):
                if self.selected_vertex():
                    self.h_shape.highlight_clear()
                self.h_vertex, self.h_shape = None, shape
                self.setToolTip(
                    "Click & drag to move shape '%s'" % shape.label)
                self.setStatusTip(self.toolTip())
                self.override_cursor(CURSOR_GRAB)
                self.update()

                # Display annotation width and height while hovering inside
                point1 = self.h_shape[1]
                point3 = self.h_shape[3]
                current_width = abs(point1.x() - point3.x())
                current_height = abs(point1.y() - point3.y())
                self.parent().window().label_coordinates.setText(
                        'Width: %d, Height: %d / X: %d; Y: %d' % (current_width, current_height, pos.x(), pos.y()))
                break
        else:  # Nothing found, clear highlights, reset state.
            if self.h_shape:
                self.h_shape.highlight_clear()
                self.update()
            self.h_vertex, self.h_shape = None, None
            self.override_cursor(CURSOR_DEFAULT)

    def mousePressEvent(self, ev):
        pos = self.transform_pos(ev.pos())
        if ev.button() == QtCore.Qt.LeftButton:
            if self.drawing():
                self.handle_drawing(pos)
            else:
                selection = self.select_shape_point(pos)
                self.prev_point = pos

                if selection is None:
                    # pan
                    QtWidgets.QApplication.setOverrideCursor( QtGui.QCursor(QtCore.Qt.OpenHandCursor))
                    self.pan_initial_pos = pos

        elif ev.button() == QtCore.Qt.RightButton and self.editing():
            self.select_shape_point(pos)
            self.prev_point = pos
        self.update()

    def mouseReleaseEvent(self, ev):
        if ev.button() == QtCore.Qt.RightButton:
            menu = self.menus[bool(self.selected_shape_copy)]
            self.restore_cursor()
            if not menu.exec_(self.mapToGlobal(ev.pos()))\
               and self.selected_shape_copy:
                # Cancel the move by deleting the shadow copy.
                self.selected_shape_copy = None
                self.repaint()
        elif ev.button() == QtCore.Qt.LeftButton and self.selected_shape:
            if self.selected_vertex():
                self.override_cursor(CURSOR_POINT)
            else:
                self.override_cursor(CURSOR_GRAB)
        elif ev.button() == QtCore.Qt.LeftButton:
            pos = self.transform_pos(ev.pos())
            if self.drawing():
                self.handle_drawing(pos)
            else:
                # pan
                QtWidgets.QApplication.restoreOverrideCursor()

    def end_move(self, copy=False):
        assert self.selected_shape and self.selected_shape_copy
        shape = self.selected_shape_copy
        # del shape.fill_color
        # del shape.line_color
        if copy:
            self.shapes.append(shape)
            self.selected_shape.selected = False
            self.selected_shape = shape
            self.repaint()
        else:
            self.selected_shape.points = [p for p in shape.points]
        self.selected_shape_copy = None

    def hide_background_shapes(self, value):
        self.hide_background = value
        if self.selected_shape:
            # Only hide other shapes if there is a current selection.
            # Otherwise the user will not be able to select a shape.
            self.set_hiding(True)
            self.repaint()

    def handle_drawing(self, pos):
        if self.current and self.current.reach_max_points() is False:
            init_pos = self.current[0]
            min_x = init_pos.x()
            min_y = init_pos.y()
            target_pos = self.line[1]
            max_x = target_pos.x()
            max_y = target_pos.y()
            self.current.add_point(QtCore.QPointF(max_x, min_y))
            self.current.add_point(target_pos)
            self.current.add_point(QtCore.QPointF(min_x, max_y))
            self.finalise()
        elif not self.out_of_pixmap(pos):
            self.current = Shape()
            self.current.add_point(pos)
            self.line.points = [pos, pos]
            self.set_hiding()
            self.drawingPolygon.emit(True)
            self.update()

    def set_hiding(self, enable=True):
        self._hide_background = self.hide_background if enable else False

    def can_close_shape(self):
        return self.drawing() and self.current and len(self.current) > 2

    def mouseDoubleClickEvent(self, ev):
        # We need at least 4 points here, since the mousePress handler
        # adds an extra one before this handler is called.
        if self.can_close_shape() and len(self.current) > 3:
            self.current.pop_point()
            self.finalise()

    def select_shape(self, shape):
        self.de_select_shape()
        shape.selected = True
        self.selected_shape = shape
        if (self.draw_free_size) :
            pass
        else :
            point1 = self.selected_shape[1] # left-top
            point3 = self.selected_shape[3] # right-bottom
            direction_x = -1 if point1.x() - point3.x() < 0 else 1
            direction_y = -1 if point1.y() - point3.y() < 0 else 1
            w = abs(point1.x() - point3.x())
            h = abs(point1.y() - point3.y())
            center_x = point3.x() + direction_x*float(w/2)
            center_y = point3.y() + direction_y*float(h/2)
            size = min(abs(point1.x() - point3.x()), abs(point1.y() - point3.y()))
            if (self.WH_Ratio >= 1.0) :
                ratio_w = size
                ratio_h = size * self.WH_Ratio
            else :
                ratio_w = size / self.WH_Ratio
                ratio_h = size
            # ratio_w = size
            # ratio_h = size * self.WH_Ratio
            self.selected_shape[0] = QtCore.QPointF(round(center_x - direction_x * float(ratio_w/2)), round(center_y + direction_y * float(ratio_h/2)))
            self.selected_shape[1] = QtCore.QPointF(round(center_x + direction_x * float(ratio_w/2)), round(center_y + direction_y * float(ratio_h/2)))
            self.selected_shape[2] = QtCore.QPointF(round(center_x + direction_x * float(ratio_w/2)), round(center_y - direction_y * float(ratio_h/2)))
            self.selected_shape[3] = QtCore.QPointF(round(center_x - direction_x * float(ratio_w/2)), round(center_y - direction_y * float(ratio_h/2)))
            if (point1 != self.selected_shape[1] and point3 != self.selected_shape[3]) :
                self.override_cursor(CURSOR_MOVE)
                self.shapeMoved.emit()
        self.set_hiding()
        self.selectionChanged.emit(True)
        self.update()
        
    def select_shape_point(self, point):
        """Select the first shape created which contains this point."""
        self.de_select_shape()
        if self.selected_vertex():  # A vertex is marked for selection.
            index, shape = self.h_vertex, self.h_shape
            shape.highlight_vertex(index, shape.MOVE_VERTEX)
            self.select_shape(shape)
            return self.h_vertex
        for shape in reversed(self.shapes):
            if self.isVisible(shape) and shape.contains_point(point):
                self.select_shape(shape)
                self.calculate_offsets(shape, point)
                return self.selected_shape
        return None

    def calculate_offsets(self, shape, point):
        rect = shape.bounding_rect()
        x1 = rect.x() - point.x()
        y1 = rect.y() - point.y()
        x2 = (rect.x() + rect.width()) - point.x()
        y2 = (rect.y() + rect.height()) - point.y()
        self.offsets = QtCore.QPointF(x1, y1), QtCore.QPointF(x2, y2)

    def snap_point_to_canvas(self, x, y):
        """
        Moves a point x,y to within the boundaries of the canvas.
        :return: (x,y,snapped) where snapped is True if x or y were changed, False if not.
        """
        if x < 0 or x > self.pixmap.width() or y < 0 or y > self.pixmap.height():
            x = max(x, 0)
            y = max(y, 0)
            x = min(x, self.pixmap.width())
            y = min(y, self.pixmap.height())
            return x, y, True

        return x, y, False

    def bounded_move_vertex(self, pos):
        index, shape = self.h_vertex, self.h_shape
        point = shape[index]
        if self.out_of_pixmap(pos):
            size = self.pixmap.size()
            clipped_x = min(max(0, pos.x()), size.width())
            clipped_y = min(max(0, pos.y()), size.height())
            pos = QtCore.QPointF(clipped_x, clipped_y)

        if self.draw_free_size:
            shift_pos = pos - point
        else:
            opposite_point_index = (index + 2) % 4
            opposite_point = shape[opposite_point_index]

            direction_x = -1 if pos.x() - opposite_point.x() < 0 else 1
            direction_y = -1 if pos.y() - opposite_point.y() < 0 else 1
            if (self.WH_Ratio >= 1.0) :
                min_size = min(abs(pos.x() - opposite_point.x()), abs(pos.y() - opposite_point.y()))
                shift_pos = QtCore.QPointF(int(opposite_point.x() + direction_x * min_size - point.x()),
                                    int(opposite_point.y() + direction_y * min_size * self.WH_Ratio - point.y()))
            else :
                max_size = max(abs(pos.x() - opposite_point.x()), abs(pos.y() - opposite_point.y()))
                shift_pos = QtCore.QPointF(int(opposite_point.x() + direction_x * max_size - point.x()),
                                    int(opposite_point.y() + direction_y * max_size * self.WH_Ratio - point.y()))


        shape.move_vertex_by(index, shift_pos)

        left_index = (index + 1) % 4
        right_index = (index + 3) % 4
        left_shift = None
        right_shift = None
        if index % 2 == 0:
            right_shift = QtCore.QPointF(shift_pos.x(), 0)
            left_shift = QtCore.QPointF(0, shift_pos.y())
        else:
            left_shift = QtCore.QPointF(shift_pos.x(), 0)
            right_shift = QtCore.QPointF(0, shift_pos.y())
        shape.move_vertex_by(right_index, right_shift)
        shape.move_vertex_by(left_index, left_shift)

    def bounded_move_shape(self, shape, pos):
        if self.out_of_pixmap(pos):
            return False  # No need to move
        o1 = pos + self.offsets[0]
        if self.out_of_pixmap(o1):
            pos -= QtCore.QPointF(min(0, o1.x()), min(0, o1.y()))
        o2 = pos + self.offsets[1]
        if self.out_of_pixmap(o2):
            pos += QtCore.QPointF(min(0, self.pixmap.width() - o2.x()),
                           min(0, self.pixmap.height() - o2.y()))
        # The next line tracks the new position of the cursor
        # relative to the shape, but also results in making it
        # a bit "shaky" when nearing the border and allows it to
        # go outside of the shape's area for some reason. XXX
        # self.calculateOffsets(self.selectedShape, pos)
        dp = pos - self.prev_point
        if dp:
            shape.move_by(dp)
            self.prev_point = pos
            return True
        return False

    def de_select_shape(self):
        if self.selected_shape:
            self.selected_shape.selected = False
            self.selected_shape = None
            self.set_hiding(False)
            self.selectionChanged.emit(False)
            self.update()

    def delete_selected(self):
        if self.selected_shape:
            shape = self.selected_shape
            self.shapes.remove(self.selected_shape)
            self.selected_shape = None
            self.update()
            return shape

    def copy_selected_shape(self):
        if self.selected_shape:
            shape = self.selected_shape.copy()
            self.de_select_shape()
            self.shapes.append(shape)
            shape.selected = True
            self.selected_shape = shape
            self.bounded_shift_shape(shape)
            return shape

    def bounded_shift_shape(self, shape):
        # Try to move in one direction, and if it fails in another.
        # Give up if both fail.
        point = shape[0]
        offset = QtCore.QPointF(2.0, 2.0)
        self.calculate_offsets(shape, point)
        self.prev_point = point
        if not self.bounded_move_shape(shape, point - offset):
            self.bounded_move_shape(shape, point + offset)

    def paintEvent(self, event):
        if not self.pixmap:
            return super(Canvas, self).paintEvent(event)

        p = self._painter
        p.begin(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.setRenderHint(QtGui.QPainter.HighQualityAntialiasing)
        p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform)

        p.scale(self.scale, self.scale)
        p.translate(self.offset_to_center())

        p.drawPixmap(0, 0, self.pixmap)
        Shape.scale = self.scale
        Shape.label_font_size = self.label_font_size
        for shape in self.shapes:
            if (shape.selected or not self._hide_background) and self.isVisible(shape):
                shape.fill = shape.selected or shape == self.h_shape
                shape.paint(p)
                if (shape.selected) :
                    point1 = shape[1]
                    point3 = shape[3]
                    x = min(point1.x(), point3.x())
                    y = max(point1.y(), point3.y())
                    current_width = abs(point1.x() - point3.x())
                    current_height = abs(point1.y() - point3.y())
                    p.setFont(QtGui.QFont("黑体", 6))
                    p.drawText(x + 5 , y + 10, "W: {}, H: {}".format(abs(int(current_width)), abs(int(current_height))) )
        if self.current:
            self.current.paint(p)
            self.line.paint(p)
        if self.selected_shape_copy:
            self.selected_shape_copy.paint(p)

        # Paint rect
        if self.current is not None and len(self.line) == 2:
            left_top = self.line[0]
            right_bottom = self.line[1]
            x = min(right_bottom.x(), left_top.x())
            y = max(right_bottom.y(), left_top.y())
            rect_width = right_bottom.x() - left_top.x()
            rect_height = right_bottom.y() - left_top.y()
            p.setPen(self.drawing_rect_color)
            brush = QtGui.QBrush(QtCore.Qt.BDiagPattern)
            p.setBrush(brush)
            # p.drawRect(left_top.x(), left_top.y(), rect_width, rect_height)
            p.drawRect(int(left_top.x()), int(left_top.y()), int(rect_width), int(rect_height))
            p.setFont(QtGui.QFont("黑体", 6))
            p.drawText(x + 5 , y + 10, "W: {}, H: {}".format(abs(int(rect_width)), abs(int(rect_height))) )

        if self.drawing() and not self.prev_point.isNull() and not self.out_of_pixmap(self.prev_point):
            p.setPen(QtGui.QColor(0, 0, 0))
            # p.drawLine(self.prev_point.x(), 0, self.prev_point.x(), self.pixmap.height())
            p.drawLine( int(self.prev_point.x()), 0, int(self.prev_point.x()), int(self.pixmap.height()))
            # p.drawLine(0, self.prev_point.y(), self.pixmap.width(), self.prev_point.y())
            p.drawLine( 0, int(self.prev_point.y()), int(self.pixmap.width()), int(self.prev_point.y()))

        self.setAutoFillBackground(True)
        if self.verified:
            pal = self.palette()
            pal.setColor(self.backgroundRole(), QtGui.QColor(184, 239, 38, 128))
            self.setPalette(pal)
        else:
            pal = self.palette()
            pal.setColor(self.backgroundRole(), QtGui.QColor(232, 232, 232, 255))
            self.setPalette(pal)

        p.end()

    def transform_pos(self, point):
        """Convert from widget-logical coordinates to painter-logical coordinates."""
        return point / self.scale - self.offset_to_center()

    def offset_to_center(self):
        s = self.scale
        area = super(Canvas, self).size()
        w, h = self.pixmap.width() * s, self.pixmap.height() * s
        aw, ah = area.width(), area.height()
        x = (aw - w) / (2 * s) if aw > w else 0
        y = (ah - h) / (2 * s) if ah > h else 0
        return QtCore.QPointF(x, y)

    def out_of_pixmap(self, p):
        w, h = self.pixmap.width(), self.pixmap.height()
        return not (0 <= p.x() <= w and 0 <= p.y() <= h)

    def finalise(self):
        assert self.current
        if self.current.points[0] == self.current.points[-1]:
            self.current = None
            self.drawingPolygon.emit(False)
            self.update()
            return

        self.current.close()
        self.shapes.append(self.current)
        self.current = None
        self.set_hiding(False)
        self.newShape.emit()
        self.update()

    def close_enough(self, p1, p2):
        # d = distance(p1 - p2)
        # m = (p1-p2).manhattanLength()
        # print "d %.2f, m %d, %.2f" % (d, m, d - m)
        return distance(p1 - p2) < self.epsilon

    # These two, along with a call to adjustSize are required for the
    # scroll area.
    def sizeHint(self):
        return self.minimumSizeHint()

    def minimumSizeHint(self):
        if self.pixmap:
            return self.scale * self.pixmap.size()
        return super(Canvas, self).minimumSizeHint()

    def wheelEvent(self, ev):
        QtCore.Qt_version = 4 if hasattr(ev, "delta") else 5
        if QtCore.Qt_version == 4:
            if ev.orientation() == QtCore.Qt.Vertical:
                v_delta = ev.delta()
                h_delta = 0
            else:
                h_delta = ev.delta()
                v_delta = 0
        else:
            delta = ev.angleDelta()
            h_delta = delta.x()
            v_delta = delta.y()

        mods = ev.modifiers()
        if QtCore.Qt.ControlModifier == int(mods) and v_delta:
            self.zoomRequest.emit(v_delta)
        else:
            v_delta and self.scrollRequest.emit(v_delta, QtCore.Qt.Vertical)
            h_delta and self.scrollRequest.emit(h_delta, QtCore.Qt.Horizontal)
        ev.accept()

    def keyReleaseEvent(self, ev):
        key = ev.key()
        # if key == QtCore.Qt.Key_Control:
        #     self.set_drawing_free_shape(False)


    def keyPressEvent(self, ev):
        key = ev.key()
        if key == QtCore.Qt.Key_Escape and self.current:
            print('ESC press')
            self.current = None
            self.drawingPolygon.emit(False)
            self.update()
        elif key == QtCore.Qt.Key_Return and self.can_close_shape():
            self.finalise()
        elif key == QtCore.Qt.Key_Left and self.selected_shape:
            self.move_one_pixel('Left')
        elif key == QtCore.Qt.Key_Right and self.selected_shape:
            self.move_one_pixel('Right')
        elif key == QtCore.Qt.Key_Up and self.selected_shape:
            self.move_one_pixel('Up')
        elif key == QtCore.Qt.Key_Down and self.selected_shape:
            self.move_one_pixel('Down')
        # elif key == QtCore.Qt.Key_Control:
        #     # Draw rectangle if Ctrl is pressed
        #     self.set_drawing_free_shape(True)

    def move_one_pixel(self, direction):
        # print(self.selectedShape.points)
        if direction == 'Left' and not self.move_out_of_bound(QtCore.QPointF(-1.0, 0)):
            self.debug.debug("Move rect 'Left' one pixel")
            self.selected_shape.points[0] += QtCore.QPointF(-1.0, 0)
            self.selected_shape.points[1] += QtCore.QPointF(-1.0, 0)
            self.selected_shape.points[2] += QtCore.QPointF(-1.0, 0)
            self.selected_shape.points[3] += QtCore.QPointF(-1.0, 0)
        elif direction == 'Right' and not self.move_out_of_bound(QtCore.QPointF(1.0, 0)):
            self.debug.debug("Move rect 'Right' one pixel")
            self.selected_shape.points[0] += QtCore.QPointF(1.0, 0)
            self.selected_shape.points[1] += QtCore.QPointF(1.0, 0)
            self.selected_shape.points[2] += QtCore.QPointF(1.0, 0)
            self.selected_shape.points[3] += QtCore.QPointF(1.0, 0)
        elif direction == 'Up' and not self.move_out_of_bound(QtCore.QPointF(0, -1.0)):
            self.debug.debug("Move rect 'Up' one pixel")
            self.selected_shape.points[0] += QtCore.QPointF(0, -1.0)
            self.selected_shape.points[1] += QtCore.QPointF(0, -1.0)
            self.selected_shape.points[2] += QtCore.QPointF(0, -1.0)
            self.selected_shape.points[3] += QtCore.QPointF(0, -1.0)
        elif direction == 'Down' and not self.move_out_of_bound(QtCore.QPointF(0, 1.0)):
            self.debug.debug("Move rect 'Down' one pixel")
            self.selected_shape.points[0] += QtCore.QPointF(0, 1.0)
            self.selected_shape.points[1] += QtCore.QPointF(0, 1.0)
            self.selected_shape.points[2] += QtCore.QPointF(0, 1.0)
            self.selected_shape.points[3] += QtCore.QPointF(0, 1.0)
        self.shapeMoved.emit()
        self.repaint()

    def move_out_of_bound(self, step):
        points = [p1 + p2 for p1, p2 in zip(self.selected_shape.points, [step] * 4)]
        return True in map(self.out_of_pixmap, points)

    def set_last_label(self, text, line_color=None, fill_color=None):
        assert text
        self.shapes[-1].label = text
        if line_color:
            self.shapes[-1].line_color = line_color

        if fill_color:
            self.shapes[-1].fill_color = fill_color
        return self.shapes[-1]

    def undo_last_line(self):
        assert self.shapes
        self.current = self.shapes.pop()
        self.current.set_open()
        self.line.points = [self.current[-1], self.current[0]]
        self.drawingPolygon.emit(True)

    def reset_all_lines(self):
        assert self.shapes
        self.current = self.shapes.pop()
        self.current.set_open()
        self.line.points = [self.current[-1], self.current[0]]
        self.drawingPolygon.emit(True)
        self.current = None
        self.drawingPolygon.emit(False)
        self.update()

    def load_pixmap(self, pixmap):
        self.pixmap = pixmap
        self.shapes = []
        self.repaint()

    def load_shapes(self, shapes):
        self.shapes = list(shapes)
        self.current = None
        self.repaint()

    def set_shape_visible(self, shape, value):
        self.visible[shape] = value
        self.repaint()

    def current_cursor(self):
        cursor = QtWidgets.QApplication.overrideCursor()
        if cursor is not None:
            cursor = cursor.shape()
        return cursor

    def override_cursor(self, cursor):
        self._cursor = cursor
        if self.current_cursor() is None:
            QtWidgets.QApplication.setOverrideCursor(cursor)
        else:
            QtWidgets.QApplication.changeOverrideCursor(cursor)

    def restore_cursor(self):
        QtWidgets.QApplication.restoreOverrideCursor()

    def reset_state(self):
        self.restore_cursor()
        self.pixmap = None
        self.update()

    def set_drawing_free_shape(self, status):
        self.draw_free_size = status

    def set_drawing_shape_Width_and_Height(self, Width_Height):
        self.WH_Ratio = float(Width_Height[1] / Width_Height[0])