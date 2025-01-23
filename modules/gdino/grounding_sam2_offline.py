import cv2, os, sys
import numpy as np
import threading
import onnxruntime as ort
from collections import OrderedDict
from typing import *
from pathlib import Path
from tokenizers import Tokenizer
from qtpy.QtCore import QCoreApplication, QPointF

from .opencv import qt_img_to_rgb_cv_img
from ..labeling.libs.shape import Shape

class Args:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

class OnnxBaseModel:
    def __init__(self, model_path: str, device_type: str = "gpu"):
        self.sess_opts = ort.SessionOptions()
        if "OMP_NUM_THREADS" in os.environ:
            self.sess_opts.inter_op_num_threads = int(
                os.environ["OMP_NUM_THREADS"]
            )

        self.providers = ["CPUExecutionProvider"]
        if device_type.lower() == "gpu":
            self.providers = ["CUDAExecutionProvider"]

        self.ort_session = ort.InferenceSession(
            model_path,
            providers=self.providers,
            sess_options=self.sess_opts,
        )

    def get_ort_inference(
        self, blob, inputs=None, extract=True, squeeze=False
    ):
        if inputs is None:
            inputs = self.get_input_name()
            outs = self.ort_session.run(None, {inputs: blob})
        else:
            outs = self.ort_session.run(None, inputs)
        if extract:
            outs = outs[0]
        if squeeze:
            outs = outs.squeeze(axis=0)
        return outs

    def get_input_name(self):
        return self.ort_session.get_inputs()[0].name

    def get_input_shape(self):
        return self.ort_session.get_inputs()[0].shape

    def get_output_name(self):
        return [out.name for out in self.ort_session.get_outputs()]

class LRUCache:
    """Thread-safe LRU cache implementation."""

    def __init__(self, maxsize=10):
        self.maxsize = maxsize
        self.lock = threading.Lock()
        self._cache = OrderedDict()

    def get(self, key):
        """Get value from cache. Returns None if key is not present."""
        with self.lock:
            if key not in self._cache:
                return None
            self._cache.move_to_end(key)
            return self._cache[key]

    def put(self, key, value):
        """Put value into cache. If cache is full, oldest item is evicted."""
        with self.lock:
            self._cache[key] = value
            self._cache.move_to_end(key)
            if len(self._cache) > self.maxsize:
                self._cache.popitem(last=False)

    def find(self, key):
        """Returns True if key is in cache, False otherwise."""
        with self.lock:
            return key in self._cache
        
class SegmentAnything2ONNX:
    """Segmentation model using Segment Anything 2 (SAM2)"""

    def __init__(self, encoder_model_path, decoder_model_path, device) -> None:
        self.encoder = SAM2ImageEncoder(encoder_model_path, device)
        self.decoder = SAM2ImageDecoder(
            decoder_model_path, device, self.encoder.input_shape[2:]
        )

    def encode(self, cv_image: np.ndarray) -> List[np.ndarray]:
        original_size = cv_image.shape[:2]
        high_res_feats_0, high_res_feats_1, image_embed = self.encoder(
            cv_image
        )
        return {
            "high_res_feats_0": high_res_feats_0,
            "high_res_feats_1": high_res_feats_1,
            "image_embedding": image_embed,
            "original_size": original_size,
        }

    def predict_masks(self, embedding, prompt) -> List[np.ndarray]:
        points = []
        labels = []
        for mark in prompt:
            if mark["type"] == "point":
                points.append(mark["data"])
                labels.append(mark["label"])
            elif mark["type"] == "rectangle":
                points.append([mark["data"][0], mark["data"][1]])  # top left
                points.append(
                    [mark["data"][2], mark["data"][3]]
                )  # bottom right
                labels.append(2)
                labels.append(3)
        points, labels = np.array(points), np.array(labels)

        image_embedding = embedding["image_embedding"]
        high_res_feats_0 = embedding["high_res_feats_0"]
        high_res_feats_1 = embedding["high_res_feats_1"]
        original_size = embedding["original_size"]
        self.decoder.set_image_size(original_size)
        masks, _ = self.decoder(
            image_embedding,
            high_res_feats_0,
            high_res_feats_1,
            points,
            labels,
        )

        return masks

    def transform_masks(self, masks, original_size, transform_matrix):
        """Transform the masks back to the original image size."""
        output_masks = []
        for batch in range(masks.shape[0]):
            batch_masks = []
            for mask_id in range(masks.shape[1]):
                mask = masks[batch, mask_id]
                mask = cv2.warpAffine(
                    mask,
                    transform_matrix[:2],
                    (original_size[1], original_size[0]),
                    flags=cv2.INTER_LINEAR,
                )
                batch_masks.append(mask)
            output_masks.append(batch_masks)
        return np.array(output_masks)

class SAM2ImageEncoder:
    def __init__(self, path: str, device: str) -> None:
        # Initialize model
        providers = ["CPUExecutionProvider"]
        if device.lower() == "gpu":
            providers = ["CUDAExecutionProvider"]
        sess_options = ort.SessionOptions()
        sess_options.log_severity_level = 3
        self.session = ort.InferenceSession(
            path, providers=providers, sess_options=sess_options
        )

        # Get model info
        self.get_input_details()
        self.get_output_details()

    def __call__(
        self, image: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self.encode_image(image)

    def encode_image(
        self, image: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        input_tensor = self.prepare_input(image)

        outputs = self.forward_encoder(input_tensor)

        return self.process_output(outputs)

    def prepare_input(self, image: np.ndarray) -> np.ndarray:
        self.img_height, self.img_width = image.shape[:2]

        input_img = cv2.resize(image, (self.input_width, self.input_height))

        mean = np.array([0.485, 0.456, 0.406])
        std = np.array([0.229, 0.224, 0.225])
        input_img = (input_img / 255.0 - mean) / std
        input_img = input_img.transpose(2, 0, 1)
        input_tensor = input_img[np.newaxis, :, :, :].astype(np.float32)

        return input_tensor

    def forward_encoder(self, input_tensor: np.ndarray) -> List[np.ndarray]:
        outputs = self.session.run(
            self.output_names, {self.input_names[0]: input_tensor}
        )

        return outputs

    def process_output(
        self, outputs: List[np.ndarray]
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return outputs[0], outputs[1], outputs[2]

    def get_input_details(self) -> None:
        model_inputs = self.session.get_inputs()
        self.input_names = [
            model_inputs[i].name for i in range(len(model_inputs))
        ]

        self.input_shape = model_inputs[0].shape
        self.input_height = self.input_shape[2]
        self.input_width = self.input_shape[3]

    def get_output_details(self) -> None:
        model_outputs = self.session.get_outputs()
        self.output_names = [
            model_outputs[i].name for i in range(len(model_outputs))
        ]

class SAM2ImageDecoder:
    def __init__(
        self,
        path: str,
        device: str,
        encoder_input_size: Tuple[int, int],
        orig_im_size: Tuple[int, int] = None,
        mask_threshold: float = 0.0,
    ) -> None:
        # Initialize model
        providers = ["CPUExecutionProvider"]
        if device.lower() == "gpu":
            providers = ["CUDAExecutionProvider"]
        sess_options = ort.SessionOptions()
        sess_options.log_severity_level = 3
        self.session = ort.InferenceSession(
            path, providers=providers, sess_options=sess_options
        )

        self.orig_im_size = (
            orig_im_size if orig_im_size is not None else encoder_input_size
        )
        self.encoder_input_size = encoder_input_size
        self.mask_threshold = mask_threshold
        self.scale_factor = 4

        # Get model info
        self.get_input_details()
        self.get_output_details()

    def __call__(
        self,
        image_embed: np.ndarray,
        high_res_feats_0: np.ndarray,
        high_res_feats_1: np.ndarray,
        point_coords: Union[List[np.ndarray], np.ndarray],
        point_labels: Union[List[np.ndarray], np.ndarray],
    ) -> Tuple[List[np.ndarray], np.ndarray]:
        return self.predict(
            image_embed,
            high_res_feats_0,
            high_res_feats_1,
            point_coords,
            point_labels,
        )

    def predict(
        self,
        image_embed: np.ndarray,
        high_res_feats_0: np.ndarray,
        high_res_feats_1: np.ndarray,
        point_coords: Union[List[np.ndarray], np.ndarray],
        point_labels: Union[List[np.ndarray], np.ndarray],
    ) -> Tuple[List[np.ndarray], np.ndarray]:
        inputs = self.prepare_inputs(
            image_embed,
            high_res_feats_0,
            high_res_feats_1,
            point_coords,
            point_labels,
        )

        outputs = self.forward_decoder(inputs)

        return self.process_output(outputs)

    def prepare_inputs(
        self,
        image_embed: np.ndarray,
        high_res_feats_0: np.ndarray,
        high_res_feats_1: np.ndarray,
        point_coords: Union[List[np.ndarray], np.ndarray],
        point_labels: Union[List[np.ndarray], np.ndarray],
    ):
        input_point_coords, input_point_labels = self.prepare_points(
            point_coords, point_labels
        )

        num_labels = input_point_labels.shape[0]
        mask_input = np.zeros(
            (
                num_labels,
                1,
                self.encoder_input_size[0] // self.scale_factor,
                self.encoder_input_size[1] // self.scale_factor,
            ),
            dtype=np.float32,
        )
        has_mask_input = np.array([0], dtype=np.float32)

        return (
            image_embed,
            high_res_feats_0,
            high_res_feats_1,
            input_point_coords,
            input_point_labels,
            mask_input,
            has_mask_input,
        )

    def prepare_points(
        self,
        point_coords: Union[List[np.ndarray], np.ndarray],
        point_labels: Union[List[np.ndarray], np.ndarray],
    ) -> Tuple[np.ndarray, np.ndarray]:
        if isinstance(point_coords, np.ndarray):
            input_point_coords = point_coords[np.newaxis, ...]
            input_point_labels = point_labels[np.newaxis, ...]
        else:
            max_num_points = max([coords.shape[0] for coords in point_coords])
            # We need to make sure that all inputs have the same number of points
            # Add invalid points to pad the input (0, 0) with -1 value for labels
            input_point_coords = np.zeros(
                (len(point_coords), max_num_points, 2), dtype=np.float32
            )
            input_point_labels = (
                np.ones((len(point_coords), max_num_points), dtype=np.float32)
                * -1
            )

            for i, (coords, labels) in enumerate(
                zip(point_coords, point_labels)
            ):
                input_point_coords[i, : coords.shape[0], :] = coords
                input_point_labels[i, : labels.shape[0]] = labels

        input_point_coords[..., 0] = (
            input_point_coords[..., 0]
            / self.orig_im_size[1]
            * self.encoder_input_size[1]
        )  # Normalize x
        input_point_coords[..., 1] = (
            input_point_coords[..., 1]
            / self.orig_im_size[0]
            * self.encoder_input_size[0]
        )  # Normalize y

        return input_point_coords.astype(
            np.float32
        ), input_point_labels.astype(np.float32)

    def forward_decoder(self, inputs) -> List[np.ndarray]:
        outputs = self.session.run(
            self.output_names,
            {
                self.input_names[i]: inputs[i]
                for i in range(len(self.input_names))
            },
        )
        return outputs

    def process_output(
        self, outputs: List[np.ndarray]
    ) -> Tuple[List[Union[np.ndarray, Any]], np.ndarray]:
        scores = outputs[1].squeeze()
        masks = outputs[0][0]

        # Select the best masks based on the scores
        best_mask = masks[np.argmax(scores)]
        best_mask = cv2.resize(
            best_mask, (self.orig_im_size[1], self.orig_im_size[0])
        )
        return (
            np.array([[best_mask]]),
            scores,
        )

    def set_image_size(self, orig_im_size: Tuple[int, int]) -> None:
        self.orig_im_size = orig_im_size

    def get_input_details(self) -> None:
        model_inputs = self.session.get_inputs()
        self.input_names = [
            model_inputs[i].name for i in range(len(model_inputs))
        ]

    def get_output_details(self) -> None:
        model_outputs = self.session.get_outputs()
        self.output_names = [
            model_outputs[i].name for i in range(len(model_outputs))
        ]
        
class GroundingDINOSAM2Local:
    """Open-Set object detection model using Grounding_DINO"""
    default_output_mode = "polygon"
    def __init__(self, model_config=None, output_mode=None) -> None:
        self.output_mode = output_mode if output_mode else GroundingDINOSAM2Local.default_output_mode
        self.net = None
        self.model = None
        self.config = model_config

        if not self.config : return 
        model_abs_path = self.config["model_path"]
        encoder_model_abs_path = self.config["encoder_model_path"]
        decoder_model_abs_path = self.config["decoder_model_path"]
        if not model_abs_path or not os.path.isfile(model_abs_path):
            QCoreApplication.translate(
                "Model",
                f"Could not download or initialize {model_abs_path} model.",
            )
        else:
            # ----------- Grounding-DINO ---------- #
            self.net = OnnxBaseModel(
                model_abs_path, device_type=self.config["device"]
            )
            self.model_configs = self.get_configs(self.config["model_type"])
            self.net.max_text_len = self.model_configs.max_text_len
            self.net.tokenizer = self.get_tokenizer()
            self.box_threshold = self.config["box_threshold"]
            self.text_threshold = self.config["text_threshold"]
            self.target_size = (
                self.config["input_width"],
                self.config["input_height"],
            )

        if not encoder_model_abs_path or not os.path.isfile(encoder_model_abs_path):
            QCoreApplication.translate(
                "Model",
                f"Could not download or initialize {encoder_model_abs_path} model.",
            )
        elif not decoder_model_abs_path or not os.path.isfile(decoder_model_abs_path):
            QCoreApplication.translate(
                "Model",
                f"Could not download or initialize {decoder_model_abs_path} model.",
            )
        else:
            # ----------- Segment-Anything-2 ---------- #
            # Load models
            self.model = SegmentAnything2ONNX(
                encoder_model_abs_path,
                decoder_model_abs_path,
                device=self.config["device"]
            )

        # Mark for auto labeling
        # points, rectangles
        self.marks = []

        # Cache for image embedding
        self.cache_size = 10
        self.preloaded_size = self.cache_size - 3
        self.image_embedding_cache = LRUCache(self.cache_size)
        self.current_image_embedding_cache = {}
        
    def preprocess(self, image, text_prompt):
        # Resize the image
        image = cv2.resize(
            image, self.target_size, interpolation=cv2.INTER_LINEAR
        )

        image = image.astype(np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406])
        std = np.array([0.229, 0.224, 0.225])
        image = (image - mean) / std
        image = np.transpose(image, (2, 0, 1))
        image = np.expand_dims(image, 0).astype(np.float32)

        # encoder texts
        captions = self.get_caption(str(text_prompt))
        tokenized_raw_results = self.net.tokenizer.encode(captions)
        tokenized = {
            "input_ids": np.array([tokenized_raw_results.ids], dtype=np.int64),
            "token_type_ids": np.array(
                [tokenized_raw_results.type_ids], dtype=np.int64
            ),
            "attention_mask": np.array([tokenized_raw_results.attention_mask]),
        }
        specical_tokens = [101, 102, 1012, 1029]
        (
            text_self_attention_masks,
            position_ids,
            _,
        ) = self.generate_masks_with_special_tokens_and_transfer_map(
            tokenized, specical_tokens
        )
        if text_self_attention_masks.shape[1] > self.net.max_text_len:
            text_self_attention_masks = text_self_attention_masks[
                :, : self.net.max_text_len, : self.net.max_text_len
            ]

            position_ids = position_ids[:, : self.net.max_text_len]
            tokenized["input_ids"] = tokenized["input_ids"][
                :, : self.net.max_text_len
            ]
            tokenized["attention_mask"] = tokenized["attention_mask"][
                :, : self.net.max_text_len
            ]
            tokenized["token_type_ids"] = tokenized["token_type_ids"][
                :, : self.net.max_text_len
            ]
        inputs = {}
        inputs["img"] = image
        inputs["input_ids"] = np.array(tokenized["input_ids"], dtype=np.int64)
        inputs["attention_mask"] = np.array(
            tokenized["attention_mask"], dtype=bool
        )
        inputs["token_type_ids"] = np.array(
            tokenized["token_type_ids"], dtype=np.int64
        )
        inputs["position_ids"] = np.array(position_ids, dtype=np.int64)
        inputs["text_token_mask"] = np.array(
            text_self_attention_masks, dtype=bool
        )

        return image, inputs, captions

    def postprocess(
        self, outputs, caption, with_logits=True, token_spans=None
    ):
        logits, boxes = outputs
        prediction_logits_ = np.squeeze(
            logits, 0
        )  # [0]  # prediction_logits.shape = (nq, 256)
        logits_filt = self.sig(prediction_logits_)
        boxes_filt = np.squeeze(
            boxes, 0
        )  # [0]  # prediction_boxes.shape = (nq, 4)
        # filter output
        if token_spans is None:
            filt_mask = logits_filt.max(axis=1) > self.box_threshold
            logits_filt = logits_filt[filt_mask]  # num_filt, 256
            boxes_filt = boxes_filt[filt_mask]  # num_filt, 4

            # get phrase
            tokenlizer = self.net.tokenizer
            tokenized_raw_results = tokenlizer.encode(caption)
            tokenized = {
                "input_ids": np.array(
                    tokenized_raw_results.ids, dtype=np.int64
                ),
                "token_type_ids": np.array(
                    tokenized_raw_results.type_ids, dtype=np.int64
                ),
                "attention_mask": np.array(
                    tokenized_raw_results.attention_mask
                ),
            }
            # build pred
            pred_phrases = []
            for logit in logits_filt:
                posmap = logit > self.text_threshold
                pred_phrase = self.get_phrases_from_posmap(
                    posmap, tokenized, tokenlizer
                )
                if with_logits:
                    pred_phrases.append([pred_phrase, logit.max()])
                else:
                    pred_phrases.append([pred_phrase, 1.0])
        else:
            # TODO: Using token_spans.
            raise NotImplementedError
        return boxes_filt, pred_phrases

    def post_process(self, masks, label=None):
        """
        Post process masks
        """
        # Find contours
        masks[masks > 0.0] = 255
        masks[masks <= 0.0] = 0
        masks = masks.astype(np.uint8)
        contours, _ = cv2.findContours(
            masks, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )

        # Refine contours
        approx_contours = []
        for contour in contours:
            # Approximate contour
            epsilon = 0.001 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            approx_contours.append(approx)

        # Remove too big contours ( >90% of image size)
        if len(approx_contours) > 1:
            image_size = masks.shape[0] * masks.shape[1]
            areas = [cv2.contourArea(contour) for contour in approx_contours]
            filtered_approx_contours = [
                contour
                for contour, area in zip(approx_contours, areas)
                if area < image_size * 0.9
            ]

        # Remove small contours (area < 20% of average area)
        if len(approx_contours) > 1:
            areas = [cv2.contourArea(contour) for contour in approx_contours]
            avg_area = np.mean(areas)

            filtered_approx_contours = [
                contour
                for contour, area in zip(approx_contours, areas)
                if area > avg_area * 0.2
            ]
            approx_contours = filtered_approx_contours

        # Contours to shapes
        shapes = []
        if self.output_mode == "polygon":
            for approx in approx_contours:
                # Scale points
                points = approx.reshape(-1, 2)
                points[:, 0] = points[:, 0]
                points[:, 1] = points[:, 1]
                points = points.tolist()
                if len(points) < 3:
                    continue
                points.append(points[0])

                # Create shape
                shape = Shape(shape_type="polygon")
                for point in points:
                    point[0] = int(point[0])
                    point[1] = int(point[1])
                    shape.add_point(QPointF(point[0], point[1]))
                # shape.fill_color = "#000000"
                # shape.line_color = "#000000"
                # shape.line_width = 1
                shape.label = "AUTOLABEL_OBJECT" if label is None else label
                shape.selected = False
                shapes.append(shape)
        elif self.output_mode in ["rectangle", "rotation"]:
            x_min = 100000000
            y_min = 100000000
            x_max = 0
            y_max = 0
            for approx in approx_contours:
                # Scale points
                points = approx.reshape(-1, 2)
                points[:, 0] = points[:, 0]
                points[:, 1] = points[:, 1]
                points = points.tolist()
                if len(points) < 3:
                    continue

                # Get min/max
                for point in points:
                    x_min = min(x_min, point[0])
                    y_min = min(y_min, point[1])
                    x_max = max(x_max, point[0])
                    y_max = max(y_max, point[1])

            # Create shape
            shape = Shape()
            shape.add_point(QPointF(x_min, y_min))
            shape.add_point(QPointF(x_max, y_min))
            shape.add_point(QPointF(x_max, y_max))
            shape.add_point(QPointF(x_min, y_max))
            shape.shape_type = (
                "rectangle" if self.output_mode == "rectangle" else "rotation"
            )
            # shape.fill_color = "#000000"
            # shape.line_color = "#000000"
            # shape.line_width = 1
            shape.label = "AUTOLABEL_OBJECT" if label is None else label
            shape.selected = False
            shapes.append(shape)

        return shapes if label is None else shapes[0]

    def predict_shapes(self, image, image_path=None, text_prompt=None):
        """
        Predict shapes from image
        """
        if image is None:
            return []

        try:
            cv_image = qt_img_to_rgb_cv_img(image, image_path)
        except Exception as e:  # noqa
            print("Could not inference model")
            print(e)
            return []

        # try:
        cached_data = self.image_embedding_cache.get(image_path)
        if cached_data is not None:
            image_embedding = cached_data
        else:
            image_embedding = self.model.encode(cv_image)
            self.image_embedding_cache.put(
                image_path,
                image_embedding,
            )

        if text_prompt:
            blob, inputs, caption = self.preprocess(cv_image, text_prompt)
            outputs = self.net.get_ort_inference(
                blob, inputs=inputs, extract=False
            )

            boxes_filt, pred_phrases = self.postprocess(outputs, caption)
            img_h, img_w, _ = cv_image.shape
            boxes = self.rescale_boxes(boxes_filt, img_h, img_w)
            shapes = []
            scores = []
            for box, label_info in zip(boxes, pred_phrases):
                label, score = label_info
                marks = [
                    {
                        "data": box,
                        "label": 1,
                        "type": "rectangle",
                    }
                ]
                masks = self.model.predict_masks(image_embedding, marks)
                if len(masks.shape) == 4:
                    masks = masks[0][0]
                else:
                    masks = masks[0]
                shape = self.post_process(masks, label=label)
                shapes.append(shape)
                scores.append(score)

            result = {"scores": scores, "shapes": shapes}
        # TODO: 
        # else:
        #     masks = self.model.predict_masks(image_embedding, self.marks)
        #     if len(masks.shape) == 4:
        #         masks = masks[0][0]
        #     else:
        #         masks = masks[0]
        #     shapes = self.post_process(masks)
        #     result = AutoLabelingResult(shapes, replace=False)
        return result
        # except Exception as e:  # noqa
        #     print("Could not inference model")
        #     return {"scores":[], "shapes": []}

    def inference(self, prompts):
        return self.predict_shapes([], image_path=prompts['image'], text_prompt=prompts['prompt'])
    
    @staticmethod
    def sig(x):
        return 1 / (1 + np.exp(-x))

    @staticmethod
    def rescale_boxes(boxes, img_h, img_w):
        converted_boxes = []
        for box in boxes:
            # from 0..1 to 0..W, 0..H
            converted_box = box * np.array([img_w, img_h, img_w, img_h])
            # from xywh to xyxy
            converted_box[:2] -= converted_box[2:] / 2
            converted_box[2:] += converted_box[:2]
            converted_boxes.append(converted_box)
        return np.array(converted_boxes, dtype=int)

    @staticmethod
    def get_configs(model_type):
        if model_type == "groundingdino_swinb_cogcoor":
            configs = Args(
                batch_size=1,
                modelname="groundingdino",
                backbone="swin_B_384_22k",
                position_embedding="sine",
                pe_temperatureH=20,
                pe_temperatureW=20,
                return_interm_indices=[1, 2, 3],
                backbone_freeze_keywords=None,
                enc_layers=6,
                dec_layers=6,
                pre_norm=False,
                dim_feedforward=2048,
                hidden_dim=256,
                dropout=0.0,
                nheads=8,
                num_queries=900,
                query_dim=4,
                num_patterns=0,
                num_feature_levels=4,
                enc_n_points=4,
                dec_n_points=4,
                two_stage_type="standard",
                two_stage_bbox_embed_share=False,
                two_stage_class_embed_share=False,
                transformer_activation="relu",
                dec_pred_bbox_embed_share=True,
                dn_box_noise_scale=1.0,
                dn_label_noise_ratio=0.5,
                dn_label_coef=1.0,
                dn_bbox_coef=1.0,
                embed_init_tgt=True,
                dn_labelbook_size=2000,
                max_text_len=256,
                text_encoder_type="bert-base-uncased",
                use_text_enhancer=True,
                use_fusion_layer=True,
                use_checkpoint=True,
                use_transformer_ckpt=True,
                use_text_cross_attention=True,
                text_dropout=0.0,
                fusion_dropout=0.0,
                fusion_droppath=0.1,
                sub_sentence_present=True,
            )
        elif model_type == "groundingdino_swint_ogc":
            configs = Args(
                batch_size=1,
                modelname="groundingdino",
                backbone="swin_T_224_1k",
                position_embedding="sine",
                pe_temperatureH=20,
                pe_temperatureW=20,
                return_interm_indices=[1, 2, 3],
                backbone_freeze_keywords=None,
                enc_layers=6,
                dec_layers=6,
                pre_norm=False,
                dim_feedforward=2048,
                hidden_dim=256,
                dropout=0.0,
                nheads=8,
                num_queries=900,
                query_dim=4,
                num_patterns=0,
                num_feature_levels=4,
                enc_n_points=4,
                dec_n_points=4,
                two_stage_type="standard",
                two_stage_bbox_embed_share=False,
                two_stage_class_embed_share=False,
                transformer_activation="relu",
                dec_pred_bbox_embed_share=True,
                dn_box_noise_scale=1.0,
                dn_label_noise_ratio=0.5,
                dn_label_coef=1.0,
                dn_bbox_coef=1.0,
                embed_init_tgt=True,
                dn_labelbook_size=2000,
                max_text_len=256,
                text_encoder_type="bert-base-uncased",
                use_text_enhancer=True,
                use_fusion_layer=True,
                use_checkpoint=True,
                use_transformer_ckpt=True,
                use_text_cross_attention=True,
                text_dropout=0.0,
                fusion_dropout=0.0,
                fusion_droppath=0.1,
                sub_sentence_present=True,
            )
        else:
            raise ValueError(
                QCoreApplication.translate(
                    "Model", "Invalid model_type in GroundingDINO model."
                )
            )
        return configs

    @staticmethod
    def get_caption(text_prompt):
        caption = text_prompt.lower()
        caption = caption.strip()
        if not caption.endswith("."):
            caption = caption + "."
        captions = caption
        return captions

    @staticmethod
    def get_tokenizer():
        current_dir = Path(__file__).resolve().parents[2] / "modules" / "gdino"
        config_json_file = f"{str(current_dir)}/bert_base_uncased_tokenizer.json"
        tokenizer = Tokenizer.from_file(config_json_file)
        return tokenizer

    @staticmethod
    def get_phrases_from_posmap(
        posmap: np.ndarray,
        tokenized: Dict,
        tokenizer,
        left_idx: int = 0,
        right_idx: int = 255,
    ):
        assert isinstance(posmap, np.ndarray), "posmap must be numpy.ndarray"
        if posmap.ndim == 1:
            posmap[0 : left_idx + 1] = False
            posmap[right_idx:] = False
            non_zero_idx = np.where(posmap)[0]
            token_ids = [tokenized["input_ids"][i] for i in non_zero_idx]
            return tokenizer.decode(token_ids)
        else:
            raise NotImplementedError("posmap must be 1-dim")

    @staticmethod
    def generate_masks_with_special_tokens_and_transfer_map(
        tokenized, special_tokens_list
    ):
        input_ids = tokenized["input_ids"]
        bs, num_token = input_ids.shape
        # special_tokens_mask: bs, num_token. 1 for special tokens. 0 for normal tokens
        special_tokens_mask = np.zeros((bs, num_token), dtype=bool)
        for special_token in special_tokens_list:
            special_tokens_mask |= input_ids == special_token

        # idxs: each row is a list of indices of special tokens
        idxs = np.argwhere(special_tokens_mask)

        # generate attention mask and positional ids
        attention_mask = np.eye(num_token, dtype=bool).reshape(
            1, num_token, num_token
        )
        attention_mask = np.tile(attention_mask, (bs, 1, 1))
        position_ids = np.zeros((bs, num_token), dtype=int)
        cate_to_token_mask_list = [[] for _ in range(bs)]
        previous_col = 0
        for i in range(idxs.shape[0]):
            row, col = idxs[i]
            if (col == 0) or (col == num_token - 1):
                attention_mask[row, col, col] = True
                position_ids[row, col] = 0
            else:
                attention_mask[
                    row, previous_col + 1 : col + 1, previous_col + 1 : col + 1
                ] = True
                position_ids[row, previous_col + 1 : col + 1] = np.arange(
                    0, col - previous_col
                )
                c2t_maski = np.zeros((num_token), dtype=bool)
                c2t_maski[previous_col + 1 : col] = True
                cate_to_token_mask_list[row].append(c2t_maski)
            previous_col = col

        cate_to_token_mask_list = [
            np.stack(cate_to_token_mask_listi, axis=0)
            for cate_to_token_mask_listi in cate_to_token_mask_list
        ]

        return attention_mask, position_ids, cate_to_token_mask_list

    def unload(self):
        del self.net
