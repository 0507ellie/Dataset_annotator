#!/usr/bin/env python
# -*- coding: utf8 -*-
import json
import os
from pathlib import Path
from datetime import datetime
from .constants import DEFAULT_ENCODING

JSON_EXT = '.json'
ENCODE_METHOD = DEFAULT_ENCODING


class CocoWriter:
    def __init__(self, folder_name, filename, img_size, shapes, output_file, database_src='Unknown', local_img_path=None):
        self.folder_name = folder_name
        self.filename = filename
        self.database_src = database_src
        self.img_size = img_size  # [height, width, channels]
        self.shapes = shapes
        self.output_file = output_file
        self.local_img_path = local_img_path
        self.verified = False

    def write(self):
        # Load existing COCO data or create new structure
        if os.path.isfile(self.output_file):
            with open(self.output_file, "r", encoding=ENCODE_METHOD) as file:
                try:
                    coco_data = json.load(file)
                    # Ensure all required fields exist
                    if "categories" not in coco_data:
                        coco_data["categories"] = []
                    if "annotations" not in coco_data:
                        coco_data["annotations"] = []
                    if "images" not in coco_data:
                        coco_data["images"] = []
                except (json.JSONDecodeError, ValueError):
                    print(f"Warning: Invalid JSON in {self.output_file}, creating new structure")
                    coco_data = self._create_empty_coco_structure()
        else:
            coco_data = self._create_empty_coco_structure()

        # Get or create image entry
        image_id = self._get_or_create_image_entry(coco_data)
        
        # Remove existing annotations for this image (in case of re-processing)
        coco_data["annotations"] = [ann for ann in coco_data["annotations"] 
                                   if ann["image_id"] != image_id]
        
        # Add new annotations
        for shape in self.shapes:
            annotation = self._create_annotation_from_shape(shape, image_id, coco_data)
            if annotation:
                coco_data["annotations"].append(annotation)

        # Write back to file
        with open(self.output_file, "w", encoding=ENCODE_METHOD) as file:
            json.dump(coco_data, file, indent=2)

    def _create_empty_coco_structure(self):
        return {
            "info": {
                "description": "Dataset converted using format converter",
                "url": "",
                "version": "1.0",
                "year": datetime.now().year,
                "contributor": "",
                "date_created": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            },
            "licenses": [
                {
                    "id": 1,
                    "name": "Unknown",
                    "url": ""
                }
            ],
            "images": [],
            "annotations": [],
            "categories": []
        }

    def _get_or_create_image_entry(self, coco_data):
        # Check if image already exists
        for img in coco_data["images"]:
            if img["file_name"] == self.filename:
                return img["id"]
        
        # Create new image entry
        new_image_id = max([img["id"] for img in coco_data["images"]], default=0) + 1
        image_entry = {
            "id": new_image_id,
            "width": self.img_size[1],
            "height": self.img_size[0],
            "file_name": self.filename,
            "license": 1,
            "date_captured": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        coco_data["images"].append(image_entry)
        return new_image_id

    def _get_or_create_category_id(self, label, coco_data):
        # Check if category already exists
        for cat in coco_data["categories"]:
            if cat["name"] == label:
                return cat["id"]
        
        # Create new category
        new_category_id = max([cat["id"] for cat in coco_data["categories"]], default=0) 
        category_entry = {
            "id": new_category_id,
            "name": label,
            "supercategory": ""
        }
        coco_data["categories"].append(category_entry)
        print(f"Created new category: {label} with ID {new_category_id}")  # Debug print
        return new_category_id

    def _create_annotation_from_shape(self, shape, image_id, coco_data):
        try:
            points = shape["points"]
            label = shape["label"]
            
            # Debug print
            print(f"Processing shape with label: {label}")
            
            # Get category ID
            category_id = self._get_or_create_category_id(label, coco_data)
            
            # Create new annotation ID
            new_annotation_id = max([ann["id"] for ann in coco_data["annotations"]], default=0) 
            
            # Determine shape type
            shape_type = shape.get("type", "rectangle")
            
            if shape_type == "rectangle" or len(points) == 4:
                # Convert rectangle points to COCO bbox format
                x_coords = [p[0] for p in points]
                y_coords = [p[1] for p in points]
                
                x_min = min(x_coords)
                y_min = min(y_coords)
                x_max = max(x_coords)
                y_max = max(y_coords)
                
                width = x_max - x_min
                height = y_max - y_min
                area = width * height
                
                annotation = {
                    "id": new_annotation_id,
                    "image_id": image_id,
                    "category_id": category_id,
                    "segmentation": [],
                    "area": float(area),
                    "bbox": [float(x_min), float(y_min), float(width), float(height)],
                    "iscrowd": 0
                }
            else:
                # Handle polygon/segmentation
                segmentation = []
                for point in points:
                    segmentation.extend([float(point[0]), float(point[1])])
                
                # Calculate area and bbox from polygon
                x_coords = [p[0] for p in points]
                y_coords = [p[1] for p in points]
                
                x_min = min(x_coords)
                y_min = min(y_coords)
                x_max = max(x_coords)
                y_max = max(y_coords)
                
                width = x_max - x_min
                height = y_max - y_min
                
                # Simple area calculation (could be improved with proper polygon area formula)
                area = width * height
                
                annotation = {
                    "id": new_annotation_id,
                    "image_id": image_id,
                    "category_id": category_id,
                    "segmentation": [segmentation],
                    "area": float(area),
                    "bbox": [float(x_min), float(y_min), float(width), float(height)],
                    "iscrowd": 0
                }
            
            return annotation
            
        except (KeyError, ValueError, TypeError) as e:
            print(f"Error creating annotation from shape: {e}")
            print(f"Shape data: {shape}")
            return None


class CocoReader:
    def __init__(self, json_path, file_path):
        self.json_path = json_path
        self.shapes = []
        self.verified = False
        self.filename = os.path.basename(file_path)
        self.categories = {}
        try:
            self.parse_json()
        except (ValueError, KeyError, FileNotFoundError) as e:
            print(f"COCO JSON parsing failed: {e}")

    def parse_json(self):
        with open(self.json_path, "r", encoding=ENCODE_METHOD) as file:
            coco_data = json.load(file)

        # Build category mapping
        categories = coco_data.get("categories", [])
        if not categories:
            print("Warning: No categories found in COCO JSON")
            
        for category in categories:
            self.categories[category["id"]] = category["name"]
        
        print(f"Loaded {len(self.categories)} categories: {list(self.categories.values())}")

        # Find the image entry
        image_id = None
        images = coco_data.get("images", [])
        
        for image in images:
            if image["file_name"] == self.filename:
                image_id = image["id"]
                break

        if image_id is None:
            print(f"Warning: Image {self.filename} not found in COCO JSON")
            return

        # Clear existing shapes
        self.shapes = []

        # Process annotations for this image
        annotations = coco_data.get("annotations", [])
        matched_annotations = [ann for ann in annotations if ann["image_id"] == image_id]
        
        print(f"Found {len(matched_annotations)} annotations for image {self.filename}")
        
        for annotation in matched_annotations:
            self._add_shape_from_annotation(annotation)

        self.verified = True

    def _add_shape_from_annotation(self, annotation):
        try:
            category_id = annotation["category_id"]
            label = self.categories.get(category_id, f"category_{category_id}")
            
            # Check if it's a segmentation or bbox
            if annotation.get("segmentation") and annotation["segmentation"]:
                # Handle segmentation
                segmentation = annotation["segmentation"][0]  # Take first segmentation
                points = []
                for i in range(0, len(segmentation), 2):
                    if i + 1 < len(segmentation):
                        points.append((segmentation[i], segmentation[i + 1]))
                
                shape_type = "polygon"
            else:
                # Handle bbox - convert to rectangle points
                bbox = annotation["bbox"]
                x, y, width, height = bbox
                
                points = [
                    (x, y),                    # top-left
                    (x + width, y),            # top-right
                    (x + width, y + height),   # bottom-right
                    (x, y + height)            # bottom-left
                ]
                
                shape_type = "rectangle"

            # Add shape in the format expected by the application
            # Format: (label, type, points, line_color, fill_color, difficult)
            self.shapes.append((label, shape_type, points, None, None, False))
            
        except (KeyError, ValueError, IndexError) as e:
            print(f"Error processing annotation: {e}")
            print(f"Annotation data: {annotation}")

    def get_shapes(self):
        return self.shapes