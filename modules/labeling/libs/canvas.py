from qtpy import QtCore, QtGui, QtWidgets
from qtpy.QtGui import QWheelEvent

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
    zoomRequest = QtCore.Signal(int, QtCore.QPoint)
    scrollRequest = QtCore.Signal(int, int)
    # [Feature] support for automatically switching to editing mode
    # when the cursor moves over an object
    newShape = QtCore.Signal()
    selectionChanged = QtCore.Signal(bool)
    shapeMoved = QtCore.Signal()
    drawingPolygon = QtCore.Signal(bool)

    CREATE, EDIT = list(range(2))

    # polygon, rectangle, rotation, line, or point
    _create_mode = "rectangle"

    _fill_drawing = False

    def __init__(self, *args, **kwargs):
        self.double_click = kwargs.pop("double_click", "close")
        if self.double_click not in [None, "close"]:
            raise ValueError(
                f"Unexpected value for double_click event: {self.double_click}"
            )
            
        super(Canvas, self).__init__(*args, **kwargs)
        # Initialise local state.
        self.epsilon = kwargs.pop("epsilon", 10.0)
        self.debug = self.parent().debug
        self.mode = self.EDIT
        self.shapes = []
        self.current = None
        self.selected_shape: Shape = None  # save the selected shape here
        self.selected_shape_copy = None
        self.drawing_line_color = QtGui.QColor(0, 0, 255)
        self.drawing_rect_color = QtGui.QColor(0, 0, 255)
        
        # self.line represents:
        #   - create_mode == 'polygon': edge from last point to current
        #   - create_mode == 'rectangle': diagonal line of the rectangle
        #   - create_mode == 'line': the line
        #   - create_mode == 'point': the point
        self.line = Shape(line_color=self.drawing_line_color)
        self.prev_point = QtCore.QPoint()
        self.prev_move_point = QtCore.QPoint()
        self.offsets = QtCore.QPointF(), QtCore.QPointF()
        self.scale = 1.0
        self.label_font_size = 8
        self.pixmap = QtGui.QPixmap()
        self.visible = {}
        self._hide_background = False
        self.hide_background = False
        
        self.h_shape: Shape = None
        self.prev_h_shape: Shape = None
        self.h_vertex = None
        self.prev_h_vertex = None
        self.h_edge = None
        self.prev_h_edge = None
        self.moving_shape = False
        
        self.allowed_oop_shape_types = ["rotation"]
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

    def fill_drawing(self):
        """Get option to fill shapes by color"""
        return self._fill_drawing

    def set_fill_drawing(self, value):
        """Set shape filling option"""
        self._fill_drawing = value

    @property
    def create_mode(self):
        """Create mode for canvas - Modes: polygon, rectangle, rotation, circle,..."""
        return self._create_mode

    @create_mode.setter
    def create_mode(self, value):
        """Set create mode for canvas"""
        if value not in [
            "polygon",
            "rectangle",
            # "rotation",
            # "circle",
            # "line",
            # "point",
            # "linestrip",
        ]:
            raise ValueError(f"Unsupported create_mode: {value}")
        self._create_mode = value
         
    def set_drawing_color(self, QColor):
        self.drawing_line_color = QColor
        self.drawing_rect_color = QColor

    def enterEvent(self, _):
        """Mouse enter event"""
        self.override_cursor(self._cursor)

    def leaveEvent(self, _):
        """Mouse leave event"""
        self.un_highlight()
        self.restore_cursor()

    def focusOutEvent(self, _):
        """Window out of focus event"""
        self.restore_cursor()

    def isVisible(self, shape):
        """Check if a shape is visible"""
        return self.visible.get(shape, True)

    def drawing(self):
        """Check if user is drawing (mode==CREATE)"""
        return self.mode == self.CREATE

    def editing(self):
        """Check if user is editing (mode==EDIT)"""
        return self.mode == self.EDIT

    def set_editing(self, value=True):
        self.mode = self.EDIT if value else self.CREATE
        if not value:  # Create
            self.un_highlight()
            self.de_select_shape()
        self.prev_point = QtCore.QPointF()
        self.repaint()

    def un_highlight(self):
        """Unhighlight shape/vertex/edge"""
        if self.h_shape:
            self.h_shape.highlight_clear()
            self.update()
        self.prev_h_shape = self.h_shape
        self.prev_h_vertex = self.h_vertex
        self.prev_h_edge = self.h_edge
        self.h_shape = self.h_vertex = self.h_edge = None

    def selected_vertex(self):
        """Check if selected a vertex"""
        return self.h_vertex is not None

    def selected_edge(self):
        """Check if selected an edge"""
        return self.h_edge is not None
    
    def mouseMoveEvent(self, ev):
        """Update line with last point and current coordinates."""
        try:
            pos = self.transform_pos(ev.localPos())
        except AttributeError:
            return


        # Update coordinates in status bar if image is opened
        window = self.parent().window()
        if window.file_path is not None:
            self.parent().window().label_coordinates.setText(
                'X: %d; Y: %d' % (pos.x(), pos.y()))

        self.prev_move_point = pos
        self.repaint()
        self.restore_cursor()

        # Polygon drawing.
        if self.drawing():
            color = self.drawing_line_color
            self.line.line_color = color
            self.line.shape_type = self.create_mode

            self.override_cursor(CURSOR_DRAW)
            if not self.current:
                return

            # Display annotation width and height while drawing
            if self.create_mode == "rectangle":
                shape_width = int(abs(self.current[0].x() - pos.x()))
                shape_height = int(abs(self.current[0].y() - pos.y()))
                self.parent().window().label_coordinates.setText(
                        'Width: %d, Height: %d / X: %d; Y: %d' % (shape_width, shape_height, pos.x(), pos.y()))

            color = QtGui.QColor(0, 0, 255)
            if (
                self.out_of_pixmap(pos)
                and self.create_mode not in self.allowed_oop_shape_types
            ):
                # Don't allow the user to draw outside the pixmap, except for rotation.
                # Project the point to the pixmap's edges.
                pos = self.intersection_point(self.current[-1], pos)
            elif (
                len(self.current) > 1
                and self.create_mode == "polygon"
                and self.close_enough(pos, self.current[0])
            ):
                # Attract line to starting point and
                # colorise to alert the user.
                pos = self.current[0]
                self.override_cursor(CURSOR_POINT)
                self.current.highlight_vertex(0, Shape.NEAR_VERTEX)
            elif (
                self.create_mode == "rotation"
                and len(self.current) > 0
                and self.close_enough(pos, self.current[0])
            ):
                pos = self.current[0]
                color = self.current.line_color
                self.override_cursor(CURSOR_POINT)
                self.current.highlight_vertex(0, Shape.NEAR_VERTEX)
            if self.create_mode in ["polygon", "linestrip"]:
                self.line[0] = self.current[-1]
                self.line[1] = pos
            elif self.create_mode == "rectangle":
                self.line.points = [self.current[0], pos]
                self.line.close()
            elif self.create_mode == "rotation":
                self.line[1] = pos
                self.line.line_color = color
            elif self.create_mode == "circle":
                self.line.points = [self.current[0], pos]
                self.line.shape_type = "circle"
            elif self.create_mode == "line":
                self.line.points = [self.current[0], pos]
                self.line.close()
            elif self.create_mode == "point":
                self.line.points = [self.current[0]]
                self.line.close()
            self.repaint()
            self.current.highlight_clear()
            return

        # Polygon copy moving.
        if QtCore.Qt.RightButton & ev.buttons():
            if self.selected_shape_copy and self.prev_point:
                self.override_cursor(CURSOR_MOVE)
                self.bounded_move_shapes(self.selected_shape_copy, pos)
                self.repaint()
            elif self.selected_shape:
                self.selected_shape_copy = self.selected_shape.copy()
                self.repaint()
            return

        # Polygon/Vertex moving.
        if QtCore.Qt.LeftButton & ev.buttons():
            if self.selected_vertex():
                try:
                    self.bounded_move_vertex(pos)
                    self.moving_shape = True
                    self.repaint()
                except IndexError:
                    return
                
                if self.h_shape.shape_type == "rectangle":
                    p1 = self.h_shape[0]
                    p2 = self.h_shape[2]
                    shape_width = int(abs(p2.x() - p1.x()))
                    shape_height = int(abs(p2.y() - p1.y()))
                    self.parent().window().label_coordinates.setText(
                            'Width: %d, Height: %d / X: %d; Y: %d' % (shape_width, shape_height, pos.x(), pos.y()))
            elif self.selected_shape and self.prev_point:
                self.override_cursor(CURSOR_MOVE)
                self.bounded_move_shapes(self.selected_shape, pos)
                self.moving_shape = True
                self.repaint()

                if self.selected_shape.shape_type == "rectangle":
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
        for _shape in reversed([s for s in self.shapes if self.isVisible(s)]):
            shape : Shape = _shape
            # Look for a nearby vertex to highlight. If that fails,
            # check if we happen to be inside a shape.
            index = shape.nearest_vertex(pos, self.epsilon / self.scale)
            index_edge = shape.nearest_edge(pos, self.epsilon / self.scale)
            if index is not None:
                if self.selected_vertex():
                    self.h_shape.highlight_clear()
                self.prev_h_vertex = self.h_vertex = index
                self.prev_h_shape = self.h_shape = shape
                self.prev_h_edge = self.h_edge
                self.h_edge = None
                shape.highlight_vertex(index, shape.MOVE_VERTEX)
                self.override_cursor(CURSOR_POINT)
                self.setToolTip(
                    ("Click & drag to move point of shape '%s'")
                    % shape.label
                )
                self.setStatusTip(self.toolTip())
                self.update()
                break
            if index_edge is not None and shape.can_add_point():
                if self.selected_vertex():
                    self.h_shape.highlight_clear()
                self.prev_h_vertex = self.h_vertex
                self.h_vertex = None
                self.prev_h_shape = self.h_shape = shape
                self.prev_h_edge = self.h_edge = index_edge
                self.override_cursor(CURSOR_POINT)
                self.setToolTip(
                    ("Click to create point of shape '%s'")
                    % shape.label
                )
                self.setStatusTip(self.toolTip())
                self.update()
                break

            if len(shape.points) > 1 and shape.contains_point(pos):
                if self.selected_vertex():
                    self.h_shape.highlight_clear()
                self.prev_h_vertex = self.h_vertex
                self.h_vertex = None
                self.prev_h_shape = self.h_shape = shape
                self.prev_h_edge = self.h_edge
                self.h_edge = None
                if shape.shape_type == "rectangle":
                    tooltip_text = "Click & drag to move shape '{label}'".format(
                        label=shape.label)
                    self.setToolTip(self.tr(tooltip_text))
                else:
                    self.setToolTip(
                        self.tr("Click & drag to move shape '%s'")
                        % shape.label
                    )
                self.setStatusTip(self.toolTip())
                self.override_cursor(CURSOR_GRAB)
                self.update()
                
            elif shape.contains_point(pos):
                if self.selected_vertex():
                    self.h_shape.highlight_clear()
                self.h_vertex, self.h_shape = None, shape
                self.setToolTip(
                    "Click & drag to move shape '%s'" % shape.label)
                self.setStatusTip(self.toolTip())
                self.override_cursor(CURSOR_GRAB)
                self.update()

                if shape.shape_type == "rectangle":
                    p1 = self.h_shape[0]
                    p2 = self.h_shape[2]
                    shape_width = int(abs(p2.x() - p1.x()))
                    shape_height = int(abs(p2.y() - p1.y()))
                    self.parent().window().label_coordinates.setText(
                            'Width: %d, Height: %d / X: %d; Y: %d' % (shape_width, shape_height, pos.x(), pos.y()))
        else:  # Nothing found, clear highlights, reset state.
            self.un_highlight()
            self.override_cursor(CURSOR_DEFAULT)

    def add_point_to_edge(self):
        """Add a point to current shape"""
        shape = self.prev_h_shape
        index = self.prev_h_edge
        point = self.prev_move_point
        if shape is None or index is None or point is None:
            return
        shape.insert_point(index, point)
        shape.highlight_vertex(index, shape.MOVE_VERTEX)
        self.h_shape = shape
        self.h_vertex = index
        self.h_edge = None
        self.moving_shape = True

    def remove_selected_point(self):
        """Remove a point from current shape"""
        shape = self.prev_h_shape
        index = self.prev_h_vertex
        if shape is None or index is None:
            return
        shape.remove_point(index)
        shape.highlight_clear()
        self.h_shape = shape
        self.prev_h_vertex = None
        self.moving_shape = True  # Save changes

    # QT Overload
    def mousePressEvent(self, ev):  # noqa: C901
        """Mouse press event"""
        pos = self.transform_pos(ev.localPos())

        if ev.button() == QtCore.Qt.LeftButton:
            if self.drawing():
                if self.current:
                    # Add point to existing shape.
                    if self.create_mode == "polygon":
                        self.current.add_point(self.line[1])
                        self.line[0] = self.current[-1]
                        
                        if self.current.is_closed():
                            self.finalise()
                    elif self.create_mode in ["circle", "line"]:
                        assert len(self.current.points) == 1
                        self.current.points = self.line.points
                        self.finalise()
                    elif self.create_mode == "rectangle":
                        if self.current.reach_max_points() is False:
                            init_pos = self.current[0]
                            min_x = init_pos.x()
                            min_y = init_pos.y()
                            target_pos = self.line[1]
                            max_x = target_pos.x()
                            max_y = target_pos.y()
                            self.current.add_point(
                                QtCore.QPointF(max_x, min_y)
                            )
                            self.current.add_point(target_pos)
                            self.current.add_point(
                                QtCore.QPointF(min_x, max_y)
                            )
                            self.finalise()
                    elif self.create_mode == "rotation":
                        initPos = self.current[0]
                        minX = initPos.x()
                        minY = initPos.y()
                        targetPos = self.line[1]
                        maxX = targetPos.x()
                        maxY = targetPos.y()
                        self.current.add_point(QtCore.QPointF(maxX, minY))
                        self.current.add_point(targetPos)
                        self.current.add_point(QtCore.QPointF(minX, maxY))
                        self.current.add_point(initPos)
                        self.line[0] = self.current[-1]
                        if self.current.is_closed():
                            self.finalise()
                    elif self.create_mode == "linestrip":
                        self.current.add_point(self.line[1])
                        self.line[0] = self.current[-1]
                        if int(ev.modifiers()) == QtCore.Qt.ControlModifier:
                            self.finalise()

                elif not self.out_of_pixmap(pos):
                    # Create new shape.
                    self.current = Shape(shape_type=self.create_mode)
                    self.current.add_point(pos)
                    if self.create_mode == "point":
                        self.finalise()
                    else:
                        if self.create_mode == "circle":
                            self.current.shape_type = "circle"
                        self.line.points = [pos, pos]
                        self.set_hiding()
                        # self.drawing_polygon.emit(True)
                        self.update()
                elif (
                    self.out_of_pixmap(pos)
                    and self.create_mode in self.allowed_oop_shape_types
                ):
                    # Create new shape.
                    self.current = Shape(shape_type=self.create_mode)
                    self.current.add_point(pos)
                    self.line.points = [pos, pos]
                    self.set_hiding()
                    self.drawingPolygon.emit(True)
                    self.update()
            elif self.editing():
                if self.selected_edge():
                    self.add_point_to_edge()
                elif (
                    self.selected_vertex()
                    and int(ev.modifiers()) == QtCore.Qt.ShiftModifier
                    and self.h_shape.shape_type
                    not in ["rectangle", "rotation", "line"]
                ):
                    # Delete point if: left-click + SHIFT on a point
                    self.remove_selected_point()

                self.select_shape_point(pos)
                self.prev_point = pos
                self.repaint()
        elif ev.button() == QtCore.Qt.RightButton and self.editing():
            if not self.selected_shape or (
                self.h_shape is not None
                and self.h_shape not in self.selected_shape
            ):
                self.select_shape_point(pos)
                self.repaint()
            self.prev_point = pos

    def mouseReleaseEvent(self, ev):
        """Mouse release event"""
        if ev.button() == QtCore.Qt.RightButton:
            menu = self.menus[bool(self.selected_shape_copy)]
            self.restore_cursor()
            if not menu.exec_(self.mapToGlobal(ev.pos()))\
               and self.selected_shape_copy:
                # Cancel the move by deleting the shadow copy.
                self.selected_shape_copy = None
                self.repaint()
        # elif ev.button() == QtCore.Qt.LeftButton and self.selected_shape:
        #     if self.selected_vertex():
        #         self.override_cursor(CURSOR_POINT)
        #     else:
        #         self.override_cursor(CURSOR_GRAB)
        elif ev.button() == QtCore.Qt.LeftButton:
            if self.editing():
                if (
                    self.h_shape is not None
                    and not self.moving_shape
                ):
                    self.selectionChanged.emit(True)
        
        if self.moving_shape:
            self.shapeMoved.emit() 
            self.moving_shape = False
            

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
        """Check if a shape can be closed (number of points > 2)"""
        return self.drawing() and self.current and len(self.current) > 2

    # QT Overload
    def mouseDoubleClickEvent(self, _):
        """Mouse double click event"""
        # We need at least 4 points here, since the mousePress handler
        # adds an extra one before this handler is called.
        if (
            self.double_click == "close"
            and self.can_close_shape()
            and len(self.current) > 3
        ):
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
                self.calculate_offsets(point)
                return self.selected_shape
        return None

    def calculate_offsets(self, point):
        """Calculate offsets of a point to pixmap borders"""
        left = self.pixmap.width() - 1
        right = 0
        top = self.pixmap.height() - 1
        bottom = 0
        s = self.selected_shape
        rect = s.bounding_rect()
        if rect.left() < left:
            left = rect.left()
        if rect.right() > right:
            right = rect.right()
        if rect.top() < top:
            top = rect.top()
        if rect.bottom() > bottom:
            bottom = rect.bottom()

        x1 = left - point.x()
        y1 = top - point.y()
        x2 = right - point.x()
        y2 = bottom - point.y()
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

    def get_adjoint_points(self, theta, p3, p1, index):
        a1 = math.tan(theta)
        if a1 == 0:
            if index % 2 == 0:
                p2 = QtCore.QPointF(p3.x(), p1.y())
                p4 = QtCore.QPointF(p1.x(), p3.y())
            else:
                p4 = QtCore.QPointF(p3.x(), p1.y())
                p2 = QtCore.QPointF(p1.x(), p3.y())
        else:
            a3 = a1
            a2 = -1 / a1
            a4 = -1 / a1
            b1 = p1.y() - a1 * p1.x()
            b2 = p1.y() - a2 * p1.x()
            b3 = p3.y() - a1 * p3.x()
            b4 = p3.y() - a2 * p3.x()

            if index % 2 == 0:
                p2 = self.get_cross_point(a1, b1, a4, b4)
                p4 = self.get_cross_point(a2, b2, a3, b3)
            else:
                p4 = self.get_cross_point(a1, b1, a4, b4)
                p2 = self.get_cross_point(a2, b2, a3, b3)

        return p2, p3, p4

    @staticmethod
    def get_cross_point(a1, b1, a2, b2):
        x = (b2 - b1) / (a1 - a2)
        y = (a1 * b2 - a2 * b1) / (a1 - a2)
        return QtCore.QPointF(x, y)

    def bounded_move_vertex(self, pos):
        """Move a vertex. Adjust position to be bounded by pixmap border"""
        index, shape = self.h_vertex, self.h_shape
        point = shape[index]
        if (
            self.out_of_pixmap(pos)
            and shape.shape_type not in self.allowed_oop_shape_types
        ):
            pos = self.intersection_point(point, pos)

        # if shape.shape_type == "rotation":
        #     sindex = (index + 2) % 4
        #     # Get the other 3 points after transformed
        #     p2, p3, p4 = self.get_adjoint_points(
        #         shape.direction, shape[sindex], pos, index
        #     )
        #     # if (
        #     #     self.out_off_pixmap(p2)
        #     #     or self.out_off_pixmap(p3)
        #     #     or self.out_off_pixmap(p4)
        #     # ):
        #     #     # No need to move if one pixal out of map
        #     #     return
        #     # Move 4 pixal one by one
        #     shape.move_vertex_by(index, pos - point)
        #     lindex = (index + 1) % 4
        #     rindex = (index + 3) % 4
        #     shape[lindex] = p2
        #     shape[rindex] = p4
        #     shape.close()
        if shape.shape_type == "rectangle":
            shift_pos = pos - point
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
        else:
            shape.move_vertex_by(index, pos - point)

    def bounded_move_shapes(self, shapes, pos):
        """Move shapes. Adjust position to be bounded by pixmap border"""
        if not isinstance(shapes, list):
            shapes = [shapes]
            
        shape_types = []
        for shape in shapes:
            if shape.shape_type in self.allowed_oop_shape_types:
                shape_types.append(shape.shape_type)

        if self.out_of_pixmap(pos) and len(shape_types) == 0:
            return False  # No need to move
        if len(shape_types) > 0 and len(shapes) != len(shape_types):
            return False

        if len(shape_types) == 0:
            o1 = pos + self.offsets[0]
            if self.out_of_pixmap(o1):
                pos -= QtCore.QPoint(min(0, int(o1.x())), min(0, int(o1.y())))
            o2 = pos + self.offsets[1]
            if self.out_of_pixmap(o2):
                pos += QtCore.QPoint(
                    min(0, int(self.pixmap.width() - o2.x())),
                    min(0, int(self.pixmap.height() - o2.y())),
                )
        # XXX: The next line tracks the new position of the cursor
        # relative to the shape, but also results in making it
        # a bit "shaky" when nearing the border and allows it to
        # go outside of the shape's area for some reason.
        # self.calculateOffsets(self.selectedShapes, pos)
        dp = pos - self.prev_point
        if dp:
            for shape in shapes:
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
            # self.end_move(copy=True) # TODO: 
            return shape

    def bounded_shift_shape(self, shape):
        # Try to move in one direction, and if it fails in another.
        # Give up if both fail.
        point = shape[0]
        offset = QtCore.QPointF(2.0, 2.0)
        self.calculate_offsets(point)
        self.prev_point = point
        if not self.bounded_move_shapes(shape, point - offset):
            self.bounded_move_shapes(shape, point + offset)

    def paintEvent(self, event):
        """Paint event for canvas"""
        if (
            self.pixmap is None
            or self.pixmap.width() == 0
            or self.pixmap.height() == 0
        ):
            super().paintEvent(event)
            return
        
        p = self._painter
        p.begin(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform)
        p.setRenderHint(QtGui.QPainter.HighQualityAntialiasing)

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
                    if shape.shape_type == "rectangle":
                        point1 = shape[1]
                        point3 = shape[-1]
                        x = min(point1.x(), point3.x())
                        y = max(point1.y(), point3.y())
                        current_width = abs(point1.x() - point3.x())
                        current_height = abs(point1.y() - point3.y())
                        p.setFont(QtGui.QFont("黑体", 6))
                        p.drawText(int(x + 5) , int(y + 10), "W: {}, H: {}".format(abs(int(current_width)), abs(int(current_height))) )
                    if shape.shape_type == "polygon":
                        if len(shape) > 3:
                            point1 = shape[1]
                            point3 = shape[len(shape) // 2]
                            x = min(point1.x(), point3.x())
                            y = max(point1.y(), point3.y())

                            p.setFont(QtGui.QFont("黑体", 6))
                            p.drawText(int(x + 5) , int(y + 10),"Nums: {}".format(abs(len(shape))) )
                
        if self.current:
            self.current.paint(p)
            self.line.paint(p)
        if self.selected_shape_copy:
            self.selected_shape_copy.paint(p)

        
        if (
            self.fill_drawing()
            and self.create_mode == "polygon"
            and self.current is not None
            and len(self.current.points) >= 2
        ):
            drawing_shape = self.current.copy()
            drawing_shape.add_point(self.line[1])
            drawing_shape.fill = True
            drawing_shape.paint(p)
            
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
            if self.create_mode == "rectangle":
                p.drawRect(int(left_top.x()), int(left_top.y()), int(rect_width), int(rect_height))
                p.setFont(QtGui.QFont("黑体", 6))
                p.drawText(int(x + 5) , int(y + 10), "W: {}, H: {}".format(abs(int(rect_width)), abs(int(rect_height))) )

        if self.drawing() and self.create_mode == "rectangle" and not self.prev_point.isNull() and not self.out_of_pixmap(self.prev_point):
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

    def inverse_transform_pos(self, point):
        """Convert from painter-logical coordinates back to widget-logical coordinates."""
        return (point + self.offset_to_center()) * self.scale

    def offset_to_center(self):
        """Calculate offset to the center"""
        if self.pixmap is None:
            return QtCore.QPointF()
        s = self.scale
        area = super().size()
        w, h = self.pixmap.width() * s, self.pixmap.height() * s
        area_width, area_height = area.width(), area.height()
        x = (area_width - w) / (2 * s) if area_width > w else 0
        y = (area_height - h) / (2 * s) if area_height > h else 0
        return QtCore.QPointF(x, y)

    def out_of_pixmap(self, p):
        """Check if a position is out of pixmap"""
        if self.pixmap is None:
            return True
        w, h = self.pixmap.width(), self.pixmap.height()
        return not (0 <= p.x() <= w - 1 and 0 <= p.y() <= h - 1)

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
        """Check if 2 points are close enough (by an threshold epsilon)"""
        # d = distance(p1 - p2)
        # m = (p1-p2).manhattanLength()
        # print "d %.2f, m %d, %.2f" % (d, m, d - m)
        # divide by scale to allow more precision when zoomed in
        return distance(p1 - p2) < (self.epsilon / self.scale)

    def intersection_point(self, p1, p2):
        """Cycle through each image edge in clockwise fashion,
        and find the one intersecting the current line segment.
        """
        size = self.pixmap.size()
        points = [
            (0, 0),
            (size.width() - 1, 0),
            (size.width() - 1, size.height() - 1),
            (0, size.height() - 1),
        ]
        # x1, y1 should be in the pixmap, x2, y2 should be out of the pixmap
        x1 = min(max(p1.x(), 0), size.width() - 1)
        y1 = min(max(p1.y(), 0), size.height() - 1)
        x2, y2 = p2.x(), p2.y()
        _, i, (x, y) = min(self.intersecting_edges((x1, y1), (x2, y2), points))
        x3, y3 = points[i]
        x4, y4 = points[(i + 1) % 4]
        x1, y1 = int(x1), int(y1)
        x2, y2 = int(x2), int(y2)
        x3, y3 = int(x3), int(y3)
        x4, y4 = int(x4), int(y4)
        if (x, y) == (x1, y1):
            # Handle cases where previous point is on one of the edges.
            if x3 == x4:
                return QtCore.QPoint(x3, min(max(0, y2), max(y3, y4)))
            # y3 == y4
            return QtCore.QPoint(min(max(0, x2), max(x3, x4)), y3)
        return QtCore.QPoint(int(x), int(y))
    
    def intersecting_edges(self, point1, point2, points):
        """Find intersecting edges.

        For each edge formed by `points', yield the intersection
        with the line segment `(x1,y1) - (x2,y2)`, if it exists.
        Also return the distance of `(x2,y2)' to the middle of the
        edge along with its index, so that the one closest can be chosen.
        """
        (x1, y1) = point1
        (x2, y2) = point2
        for i in range(4):
            x3, y3 = points[i]
            x4, y4 = points[(i + 1) % 4]
            denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
            nua = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)
            nub = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)
            if denom == 0:
                # This covers two cases:
                #   nua == nub == 0: Coincident
                #   otherwise: Parallel
                continue
            ua, ub = nua / denom, nub / denom
            if 0 <= ua <= 1 and 0 <= ub <= 1:
                x = x1 + ua * (x2 - x1)
                y = y1 + ua * (y2 - y1)
                m = QtCore.QPointF((x3 + x4) / 2, (y3 + y4) / 2)
                d = distance(m - QtCore.QPointF(x2, y2))
                yield d, i, (x, y)
    
    # These two, along with a call to adjustSize are required for the
    # scroll area.
    # QT Overload
    def sizeHint(self):
        """Get size hint"""
        return self.minimumSizeHint()

    # QT Overload
    def minimumSizeHint(self):
        """Get minimum size hint"""
        if self.pixmap:
            return self.scale * self.pixmap.size()
        return super().minimumSizeHint()

    # QT Overload
    def wheelEvent(self, ev: QWheelEvent):
        """Mouse wheel event"""
        mods = ev.modifiers()
        delta = ev.angleDelta()
        if QtCore.Qt.ControlModifier == int(mods):
            # with Ctrl/Command key
            # zoom
            self.zoomRequest.emit(delta.y(), ev.pos())
        else:
            # scroll
            self.scrollRequest.emit(delta.x(), QtCore.Qt.Horizontal)
            self.scrollRequest.emit(delta.y(), QtCore.Qt.Vertical)
        ev.accept()

    def keyReleaseEvent(self, ev):
        key = ev.key()
        # if key == QtCore.Qt.Key_Control:
        #     self.set_drawing_free_shape(False)


    def keyPressEvent(self, ev):
        key = ev.key()
        if key == QtCore.Qt.Key_Escape and self.current:
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
        """Undo last line"""
        assert self.shapes
        self.current = self.shapes.pop()
        self.current.set_open()
        if self.create_mode in ["polygon", "linestrip"]:
            self.line.points = [self.current[-1], self.current[0]]
        elif self.create_mode in ["rectangle", "line", "circle", "rotation"]:
            self.current.points = self.current.points[0:1]
        elif self.create_mode == "point":
            self.current = None
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

    def load_pixmap(self, pixmap, clear_shapes=True):
        """Load pixmap"""
        self.pixmap = pixmap
        if clear_shapes:
            self.shapes = []
        self.update()

    def load_shapes(self, shapes, replace=True):
        """Load shapes"""
        if replace:
            self.shapes = list(shapes)
        else:
            self.shapes.extend(shapes)
        self.current = None
        self.h_shape = None
        self.h_vertex = None
        self.h_edge = None
        self.update()

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