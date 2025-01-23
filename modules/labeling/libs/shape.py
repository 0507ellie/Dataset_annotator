#!/usr/bin/python
# -*- coding: utf-8 -*-


try:
    from qtpy.QtGui import *
    from qtpy.QtCore import *
except ImportError:
    from PyQt4.QtGui import *
    from PyQt4.QtCore import *

from .utils import distance, distance_to_line
import sys
import copy
import math

DEFAULT_LINE_COLOR = QColor(0, 255, 0, 128)  # bf hovering
DEFAULT_FILL_COLOR = QColor(100, 100, 100, 100)  # hovering
DEFAULT_SELECT_LINE_COLOR = QColor(255, 255, 255)  # selected
DEFAULT_SELECT_FILL_COLOR = QColor(0, 255, 0, 155)  # selected
DEFAULT_VERTEX_FILL_COLOR = QColor(0, 255, 0, 255)  # hovering
DEFAULT_HVERTEX_FILL_COLOR = QColor(255, 255, 255, 255)  # hovering

class Shape(object):
    P_SQUARE, P_ROUND = range(2)

    MOVE_VERTEX, NEAR_VERTEX = range(2)

    # The following class variables influence the drawing of all shape objects.
    line_color = DEFAULT_LINE_COLOR
    fill_color = DEFAULT_FILL_COLOR
    select_line_color = DEFAULT_SELECT_LINE_COLOR
    select_fill_color = DEFAULT_SELECT_FILL_COLOR
    vertex_fill_color = DEFAULT_VERTEX_FILL_COLOR
    hvertex_fill_color = DEFAULT_HVERTEX_FILL_COLOR
    point_type = P_ROUND
    point_size = 4
    scale = 1.5
    line_width = 2.0
    label_font_size = 8
    
    def __init__(self, 
                 label=None, 
                 line_color=None, 
                 difficult=False, 
                 paint_label=False,
                 shape_type="rectangle"): # default: None
        self.label = label
        self.points = []
        self.fill = False
        self.selected = False
        self.difficult = difficult
        self.paint_label = paint_label

        self._highlight_index = None
        self._highlight_mode = self.NEAR_VERTEX
        self._highlight_settings = {
            self.NEAR_VERTEX: (4, self.P_ROUND),
            self.MOVE_VERTEX: (1.5, self.P_SQUARE),
        }
        self._vertex_fill_color = None
        self._closed = False

        if line_color is not None:
            # Override the class line_color attribute
            # with an object attribute. Currently this
            # is used for drawing the pending line a different color.
            self.line_color = line_color
        self.shape_type = shape_type

    @property
    def shape_type(self):
        """Get shape type (polygon, rectangle, rotation, point, line, ...)"""
        return self._shape_type

    @shape_type.setter
    def shape_type(self, value):
        """Set shape type"""
        if value is None:
            value = "polygon"
        if value not in self.get_supported_shape():
            raise ValueError(f"Unexpected shape_type: {value}")
        self._shape_type = value

    @staticmethod
    def get_supported_shape():
        return [
            "polygon",
            "rectangle",
            # "rotation",
            # "point",
            # "line",
            # "circle",
            # "linestrip",
        ]
           
    def close(self):
        """Close the shape"""
        if self.shape_type == "rotation" and len(self.points) == 4:
            cx = (self.points[0].x() + self.points[2].x()) / 2
            cy = (self.points[0].y() + self.points[2].y()) / 2
            self.center = QPointF(cx, cy)
        self._closed = True

    def reach_max_points(self):
        if len(self.points) >= 4:
            return True
        return False

    def add_point(self, point):
        """Add a point"""
        if self.shape_type == "rectangle":
            if not self.reach_max_points():
                self.points.append(point)
        else:
            if self.points and point == self.points[0]:
                self.close()
            else:
                self.points.append(point)

    def can_add_point(self):
        """Check if shape supports more points"""
        return self.shape_type in ["polygon", "linestrip"]

    def pop_point(self):
        """Remove and return the last point of the shape"""
        if self.points:
            return self.points.pop()
        return None

    def insert_point(self, i, point):
        """Insert a point to a specific index"""
        self.points.insert(i, point)

    def remove_point(self, i):
        """Remove point from a specific index"""
        self.points.pop(i)

    def is_closed(self):
        """Check if the shape is closed"""
        return self._closed

    def set_open(self):
        """Set shape to open - (_close=False)"""
        self._closed = False

    def get_rect_from_line(self, pt1, pt2):
        """Get rectangle from diagonal line"""
        x1, y1 = pt1.x(), pt1.y()
        x2, y2 = pt2.x(), pt2.y()
        return QRectF(x1, y1, x2 - x1, y2 - y1)

    def paint(self, painter: QPainter):
        if self.points:
            color = (
                self.select_line_color if self.selected else self.line_color
            )
            pen = QPen(color)
            # Try using integer sizes for smoother drawing(?)
            pen.setWidth(max(1, int(round(self.line_width / self.scale))))
            painter.setPen(pen)

            line_path = QPainterPath()
            vrtx_path = QPainterPath()

            if self.shape_type == "rectangle":
                assert len(self.points) in [1, 2, 4]
                if len(self.points) == 2:
                    rectangle = self.get_rect_from_line(*self.points)
                    line_path.addRect(rectangle)
                if len(self.points) == 4:
                    line_path.moveTo(self.points[0])
                    for i, p in enumerate(self.points):
                        line_path.lineTo(p)
                        if self.selected:
                            self.draw_vertex(vrtx_path, i)
                    if self.is_closed() or self.label is not None:
                        line_path.lineTo(self.points[0])
            elif self.shape_type == "point":
                assert len(self.points) == 1
                self.draw_vertex(vrtx_path, 0)
            else:
                line_path.moveTo(self.points[0])
                # Uncommenting the following line will draw 2 paths
                # for the 1st vertex, and make it non-filled, which
                # may be desirable.
                self.draw_vertex(vrtx_path, 0)

                for i, p in enumerate(self.points):
                    line_path.lineTo(p)
                    if self.selected:
                        self.draw_vertex(vrtx_path, i)
                if self.is_closed():
                    line_path.lineTo(self.points[0])
 
                
            painter.drawPath(line_path)
            painter.drawPath(vrtx_path)
            if self._vertex_fill_color is not None:
                painter.fillPath(vrtx_path, self._vertex_fill_color)

            if self.fill:
                color = (
                    self.select_fill_color
                    if self.selected
                    else self.fill_color
                )
                painter.fillPath(line_path, color)
            else:
                color = self.fill_color.lighter(self.fill_color.alpha() >> 1)
                painter.fillPath(line_path, color)
                  
            # Draw text at the top-left
            if self.paint_label:
                min_x = sys.maxsize
                min_y = sys.maxsize
                min_y_label = int(1.25 * self.label_font_size)
                for point in self.points:
                    min_x = min(min_x, point.x())
                    min_y = min(min_y, point.y() - self.label_font_size//2)
                if min_x != sys.maxsize and min_y != sys.maxsize:
                    font = QFont()
                    font.setPointSize(self.label_font_size)
                    font.setBold(True)
                    painter.setFont(font)
                    if self.label is None:
                        self.label = ""
                    if min_y < min_y_label:
                        min_y += min_y_label
                    painter.drawText(min_x, min_y, self.label)
                
    def draw_vertex(self, path, i):
        """Draw a vertex"""
        d = self.point_size / self.scale
        shape = self.point_type
        point = self.points[i]
        if i == self._highlight_index:
            size, shape = self._highlight_settings[self._highlight_mode]
            d *= size
        if self._highlight_index is not None:
            self._vertex_fill_color = self.hvertex_fill_color
        else:
            self._vertex_fill_color = self.vertex_fill_color
        if shape == self.P_SQUARE:
            path.addRect(point.x() - d / 2, point.y() - d / 2, d, d)
        elif shape == self.P_ROUND:
            path.addEllipse(point, d / 2.0, d / 2.0)
        else:
            assert False, "unsupported vertex shape"

    def nearest_vertex(self, point, epsilon):
        """Find the index of the nearest vertex to a point
        Only consider if the distance is smaller than epsilon
        """
        min_distance = float("inf")
        min_i = None
        for i, p in enumerate(self.points):
            dist = distance(p - point)
            if dist <= epsilon and dist < min_distance:
                min_distance = dist
                min_i = i
        return min_i

    def nearest_edge(self, point, epsilon):
        """Get nearest edge index"""
        min_distance = float("inf")
        post_i = None
        for i in range(len(self.points)):
            line = [self.points[i - 1], self.points[i]]
            dist = distance_to_line(point, line)
            if dist <= epsilon and dist < min_distance:
                min_distance = dist
                post_i = i
        return post_i

    def contains_point(self, point):
        """Check if shape contains a point"""
        return self.make_path().contains(point)

    def make_path(self):
        """Create a path from shape"""
        if self.shape_type == "rectangle":
            path = QPainterPath(self.points[0])
            for p in self.points[1:]:
                path.lineTo(p)
        # elif self.shape_type == "circle":
        #     path = QtGui.QPainterPath()
        #     if len(self.points) == 2:
        #         rectangle = self.get_circle_rect_from_line(self.points)
        #         path.addEllipse(rectangle)
        else:
            path = QPainterPath(self.points[0])
            for p in self.points[1:]:
                path.lineTo(p)
        return path

    def bounding_rect(self):
        """Return bounding rectangle of the shape"""
        return self.make_path().boundingRect()

    def move_by(self, offset):
        """Move all points by an offset"""
        self.points = [p + offset for p in self.points]
        
    def move_vertex_by(self, i, offset):
        """Move a specific vertex by an offset"""
        self.points[i] = self.points[i] + offset

    def highlight_vertex(self, i, action):
        """Highlight a vertex appropriately based on the current action

        Args:
            i (int): The vertex index
            action (int): The action
            (see Shape.NEAR_VERTEX and Shape.MOVE_VERTEX)
        """
        self._highlight_index = i
        self._highlight_mode = action

    def highlight_clear(self):
        """Clear the highlighted point"""
        self._highlight_index = None
        
    def copy(self):
        """Copy shape"""
        return copy.deepcopy(self)

    def __len__(self):
        return len(self.points)

    def __getitem__(self, key):
        return self.points[key]

    def __setitem__(self, key, value):
        self.points[key] = value
