import os
import codecs
import argparse
from pathlib import Path
from typing import List
from PyQt5 import Qt
from PyQt5 import QtCore, QtGui, QtWidgets

from modules.labeling.libs.ustr import ustr
from modules.labeling.libs.yolo_io import TXT_EXT, YoloReader
from modules.labeling.libs.pascal_voc_io import XML_EXT, PascalVocReader
from modules.labeling.libs.create_ml_io import JSON_EXT, CreateMLReader
from modules.labeling.libs.labelFile import LabelFileFormat, LabelFile

# TODO: 
argparser = argparse.ArgumentParser(description='Convert LabelFileFormat')
argparser.add_argument('-i','--root_dir',
                        help='The path to the root directory contains an images/labels folder or \
                              subdirectory contains images/labels folder.')
argparser.add_argument('-c','--class_file', default= os.path.join(os.path.dirname(__file__), 'default_classes.txt'),
                        help='Path to the file containing class names. Default is "default_classes.txt".')
argparser.add_argument('-o','--save_folder', default="yolo", nargs="?",
                        help='Folder to save the labeled frames. Default is "yolo".')

IMAGE_TAG = "images"
LABEL_TAG = "labels"

class FormatConvert(object):
    def __init__(self, class_file) -> None:
        self.label_file_format = None
        self.load_predefined_classes(class_file)
        self.shapes = []

    @staticmethod
    def getImagesInDir(folder_dir: str) -> List[str]:
        images_list = []
        for root, dirs, files in os.walk(folder_dir):
            for file in files:
                if file.lower().endswith(tuple(["jpg", "png"])):
                    relative_path = os.path.join(root, file)
                    path = ustr(os.path.abspath(relative_path))
                    images_list.append(path)
        image_count = len(images_list)
        print(f"Loading image from [{folder_dir}] ...")
        if image_count:
            print(f"Images counts = {str(image_count)} success!")
        else :
            print(f"Images counts = {str(image_count)}, dir is empty!")
        return images_list

    def load_predefined_classes(self, predef_classes_file: str) -> None:
        self.classes = []
        if os.path.exists(predef_classes_file) is True:
            with codecs.open(predef_classes_file, 'r', 'utf8') as f:
                for line in f:
                    line = line.strip()
                    if self.classes == []:
                        self.classes = [line]
                    else:
                        self.classes.append(line)

    def read_annotation(self, input_label_dir, image_path):
        if not Path(input_label_dir).is_dir():
            raise f"[{input_label_dir}] not a dir."
        image_name = os.path.splitext(os.path.basename(image_path))[0]
        xml_path = os.path.join(input_label_dir, image_name + XML_EXT)
        txt_path = os.path.join(input_label_dir, image_name + TXT_EXT)
        json_path = os.path.join(input_label_dir, image_name + JSON_EXT)

        reader =  QtGui.QImageReader(image_path)
        reader.setAutoTransform(True)
        image = reader.read()

        if os.path.isfile(xml_path):
            self.label_file_format = LabelFileFormat.PASCAL_VOC
            t_voc_parse_reader = PascalVocReader(xml_path) 
            self.shapes = t_voc_parse_reader.get_shapes()
            print("PascalVocReader shape : " + str(len(self.shapes)))
        elif os.path.isfile(txt_path):
            self.label_file_format = LabelFileFormat.YOLO
            t_yolo_parse_reader = YoloReader(txt_path, image, self.classes)
            self.shapes = t_yolo_parse_reader.get_shapes()
            print("YoloReader shape : " + str(len(self.shapes)))
        elif os.path.isfile(json_path):
            print("Not supper yet!")
            # create_ml_parse_reader = CreateMLReader(json_path, file_path)
            # self.shapes = create_ml_parse_reader.get_shapes()
            # print("CreateMLReader shape : " + str(self.shapes))
            pass

    def convert_annotation(self, output_label_dir, image_path):
        if not Path(output_dir_path).is_dir():
            output_dir_path.mkdir(parents=True, exist_ok=True)
        image_name = os.path.splitext(os.path.basename(image_path))[0]
        txt_path = os.path.join(output_label_dir, image_name + TXT_EXT)
        xml_path = os.path.join(output_label_dir, image_name + XML_EXT)
        json_path = os.path.join(output_label_dir, image_name + JSON_EXT)

        reader =  QtGui.QImageReader(image_path)
        reader.setAutoTransform(True)
        image = reader.read()

        label_file = LabelFile()
        convert_shapes = [ dict(label=label, points=points, difficult=difficult) for label, points, _, _, difficult in self.shapes]
        if self.label_file_format != LabelFileFormat.YOLO:
            label_file.save_yolo_format(txt_path, convert_shapes, image_path, image, classes)
        elif self.label_file_format != LabelFileFormat.PASCAL_VOC:
            label_file.save_pascal_voc_format(xml_path, convert_shapes, image_path, image)    

if __name__ == '__main__':
    args = argparser.parse_args()
    root_dir = args.root_dir
    class_file = args.class_file
    out_folder = args.save_folder
    
    print(f"Root Dir: {Path(root_dir)}")
    single_folder = False
    sub_folders = [entry.name for entry in Path(root_dir).iterdir() if entry.is_dir()]
    if IMAGE_TAG in sub_folders and LABEL_TAG in sub_folders:
        single_folder = True
        sub_folders = [root_dir]
    for folder in sub_folders:
        if single_folder:
            item_dir = Path(sub_folders[0])
        else:
            item_dir = Path(root_dir).joinpath(folder).resolve()

        print(f"Starting Processing [{item_dir}] Path... ")
        images_dir_path = item_dir.joinpath(IMAGE_TAG) 
        labels_dir_path = item_dir.joinpath(LABEL_TAG)
        output_dir_path = item_dir.joinpath(out_folder)

        # loading images list
        image_paths = FormatConvert.getImagesInDir(str(images_dir_path))
            
        # loading labels list
        print(f"Source label dir: {labels_dir_path}")
        print(f"Save label dir: {output_dir_path}")
        converter = FormatConvert(class_file)
        list_file = open(str(item_dir) + '.txt', 'w')
        for image_path in image_paths:
            list_file.write(image_path + '\n')
            converter.read_annotation(str(labels_dir_path), image_path)
            converter.convert_annotation(str(output_dir_path), image_path)
        list_file.close()
        print(f"Finished Processing [{folder}] Folder.")