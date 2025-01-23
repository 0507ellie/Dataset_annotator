#!/usr/bin/env python
# -*- coding: utf8 -*-
import numpy as np
import os
from xml.etree import ElementTree
from xml.etree.ElementTree import Element, SubElement
from lxml import etree
import codecs
from .constants import DEFAULT_ENCODING

TXT_EXT = '.txt'
ENCODE_METHOD = DEFAULT_ENCODING

class YOLOWriter:

    def __init__(self, folder_name, filename, img_size, database_src='Unknown', local_img_path=None):
        self.folder_name = folder_name
        self.filename = filename
        self.database_src = database_src
        self.img_size = img_size
        self.rectangle_list = []
        self.polygon_list = []
        self.local_img_path = local_img_path
        self.verified = False

    def add_bnd_box(self, x_min, y_min, x_max, y_max, name, difficult):
        bnd_box = {'xmin': x_min, 'ymin': y_min, 'xmax': x_max, 'ymax': y_max}
        bnd_box['name'] = name
        bnd_box['difficult'] = difficult
        self.rectangle_list.append(bnd_box)

    def add_seg_points(self, points, name, difficult):
        seg_box = {"points" : points}
        seg_box['name'] = name
        seg_box['difficult'] = difficult
        self.polygon_list.append(seg_box)

    def bnd_box_to_yolo_line(self, box, class_list=[]):
        x_min = box['xmin']
        x_max = box['xmax']
        y_min = box['ymin']
        y_max = box['ymax']

        x_center = float((x_min + x_max)) / 2 / self.img_size[1]
        y_center = float((y_min + y_max)) / 2 / self.img_size[0]

        width = float((x_max - x_min)) / self.img_size[1]
        height = float((y_max - y_min)) / self.img_size[0]

        # PR387
        box_name = box['name']
        if box_name not in class_list:
            class_list.append(box_name)

        class_index = class_list.index(box_name)

        return f"{class_index} {x_center} {y_center} {width} {height}\n"

    def poly_points_to_yolo_line(self, poly, class_list=[]):
        points = poly["points"]
        image_size = np.array([[self.img_size[1], self.img_size[0]]])
        norm_points = points / image_size

        # PR387
        poly_name = poly['name']
        if poly_name not in class_list:
            class_list.append(poly_name)

        class_index = class_list.index(poly_name)
        return  (f"{class_index} "
                    + " ".join(
                        [
                            " ".join([str(cell[0]), str(cell[1])])
                            for cell in norm_points.tolist()
                        ]
                    )
                + "\n")
        
    def save(self, class_list=[], target_file=None):

        out_file = None  # Update yolo .txt
        out_class_file = None   # Update class list .txt

        if target_file is None:
            out_file = open(
            self.filename + TXT_EXT, 'w', encoding=ENCODE_METHOD)
            classes_file = os.path.join(os.path.dirname(os.path.abspath(self.filename)), "classes.txt")
            out_class_file = open(classes_file, 'w')

        else:
            out_file = codecs.open(target_file, 'w', encoding=ENCODE_METHOD)
            classes_file = os.path.join(os.path.dirname(os.path.abspath(target_file)), "classes.txt")
            out_class_file = open(classes_file, 'w')


        for rect in self.rectangle_list:
            line = self.bnd_box_to_yolo_line(rect, class_list)
            out_file.write(line)

        for poly in self.polygon_list:
            line = self.poly_points_to_yolo_line(poly, class_list)
            out_file.write(line)

        # print (classList)
        # print (out_class_file)
        for c in class_list:
            out_class_file.write(c+'\n')

        out_class_file.close()
        out_file.close()



class YoloReader:

    def __init__(self, file_path, image, class_list_path=None):
        # shapes type:
        # [labbel, [(x1,y1), (x2,y2), (x3,y3), (x4,y4)], color, color, difficult]
        self.shapes = []
        self.file_path = file_path

        if class_list_path is None:
            dir_path = os.path.dirname(os.path.realpath(self.file_path))
            self.class_list_path = os.path.join(dir_path, "classes.txt")
        else:
            self.class_list_path = class_list_path

        if (isinstance(self.class_list_path, str)):
            classes_file = open(self.class_list_path, 'r')
            self.classes = classes_file.read().strip('\n').split('\n')
        else :
            self.classes = self.class_list_path

        img_size = [image.height(), image.width(),
                    1 if image.isGrayscale() else 3]

        self.img_size = img_size

        self.verified = False
        # try:
        self.parse_yolo_format()
        # except:
        #     pass

    def get_shapes(self):
        return self.shapes

    def parse_yolo_format(self):
        lines = open(self.file_path, 'r')
        for line in lines:
            line = line.strip().split(" ")
            class_index = int(line[0])
            try:
                label = self.classes[int(class_index)]
            except:
                label = "unknown"
                
            if len(line) == 5:
                shape_type = "rectangle"
                cx = float(line[1])
                cy = float(line[2])
                nw = float(line[3])
                nh = float(line[4])
                xmin = int((cx - nw / 2) * self.img_size[1])
                ymin = int((cy - nh / 2) * self.img_size[0])
                xmax = int((cx + nw / 2) * self.img_size[1])
                ymax = int((cy + nh / 2) * self.img_size[0])
                points = [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]
            else:
                shape_type = "polygon"
                points, masks = [], line[1:]
                image_size = np.array([self.img_size[1], self.img_size[0]], np.float64)
                for x, y in zip(masks[0::2], masks[1::2]):
                    point = [np.float64(x), np.float64(y)]
                    point = np.array(point, np.float64) * image_size
                    points.append(point.tolist())
                    
            difficult = False
            self.shapes.append((label, shape_type, points, None, None, difficult))
