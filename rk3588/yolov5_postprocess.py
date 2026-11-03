from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


COCO_CLASSES = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
)

ANCHORS = np.asarray(
    (
        (10, 13), (16, 30), (33, 23),
        (30, 61), (62, 45), (59, 119),
        (116, 90), (156, 198), (373, 326),
    ),
    dtype=np.float32,
)
ANCHOR_MASKS = ((0, 1, 2), (3, 4, 5), (6, 7, 8))


@dataclass(frozen=True)
class DetectionBox:
    class_id: int
    label: str
    score: float
    left: float
    top: float
    right: float
    bottom: float

    @property
    def x_center(self) -> float:
        return (self.left + self.right) / 2.0

    @property
    def y_center(self) -> float:
        return (self.top + self.bottom) / 2.0


def _to_hwa_features(output: np.ndarray, class_count: int) -> np.ndarray:
    """Normalize one RKNN YOLO output to [grid_h, grid_w, 3, 5+C]."""
    array = np.asarray(output)
    features = class_count + 5

    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]

    if array.ndim == 3:
        if array.shape[0] == 3 * features:
            array = array.reshape(3, features, array.shape[1], array.shape[2])
            return np.transpose(array, (2, 3, 0, 1))
        if array.shape[-1] == 3 * features:
            return array.reshape(array.shape[0], array.shape[1], 3, features)

    if array.ndim == 4:
        if array.shape[:2] == (3, features):
            return np.transpose(array, (2, 3, 0, 1))
        if array.shape[-2:] == (3, features):
            return array

    raise ValueError(f"unsupported YOLO output shape: {array.shape}")


def _decode_scale(
    features: np.ndarray,
    anchors: np.ndarray,
    input_size: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    grid_h, grid_w = features.shape[:2]
    grid_y, grid_x = np.meshgrid(
        np.arange(grid_h, dtype=np.float32),
        np.arange(grid_w, dtype=np.float32),
        indexing="ij",
    )
    grid = np.stack((grid_x, grid_y), axis=-1)[:, :, None, :]

    box_xy = (features[..., :2] * 2.0 - 0.5) + grid
    box_xy *= np.asarray((input_size / grid_w, input_size / grid_h))
    box_wh = np.square(features[..., 2:4] * 2.0) * anchors

    boxes = np.concatenate((box_xy, box_wh), axis=-1).reshape(-1, 4)
    objectness = features[..., 4].reshape(-1)
    class_probabilities = features[..., 5:].reshape(-1, features.shape[-1] - 5)
    return boxes, objectness, class_probabilities


def xywh_to_xyxy(boxes: np.ndarray) -> np.ndarray:
    converted = np.array(boxes, dtype=np.float32, copy=True)
    converted[:, 0] = boxes[:, 0] - boxes[:, 2] / 2.0
    converted[:, 1] = boxes[:, 1] - boxes[:, 3] / 2.0
    converted[:, 2] = boxes[:, 0] + boxes[:, 2] / 2.0
    converted[:, 3] = boxes[:, 1] + boxes[:, 3] / 2.0
    return converted


def nms_indices(
    boxes: np.ndarray,
    scores: np.ndarray,
    threshold: float,
) -> np.ndarray:
    if len(boxes) == 0:
        return np.empty(0, dtype=np.int64)

    x1, y1, x2, y2 = boxes.T
    widths = np.maximum(0.0, x2 - x1)
    heights = np.maximum(0.0, y2 - y1)
    areas = widths * heights
    order = scores.argsort()[::-1]
    keep: list[int] = []

    while order.size:
        current = int(order[0])
        keep.append(current)
        if order.size == 1:
            break

        remaining = order[1:]
        xx1 = np.maximum(x1[current], x1[remaining])
        yy1 = np.maximum(y1[current], y1[remaining])
        xx2 = np.minimum(x2[current], x2[remaining])
        yy2 = np.minimum(y2[current], y2[remaining])
        intersection = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        union = areas[current] + areas[remaining] - intersection
        iou = np.divide(
            intersection,
            union,
            out=np.zeros_like(intersection),
            where=union > 0,
        )
        order = remaining[iou <= threshold]

    return np.asarray(keep, dtype=np.int64)


def decode_yolov5_outputs(
    outputs: Sequence[np.ndarray],
    object_threshold: float = 0.25,
    nms_threshold: float = 0.45,
    input_size: int = 640,
    class_names: Sequence[str] = COCO_CLASSES,
) -> list[DetectionBox]:
    if len(outputs) != 3:
        raise ValueError(f"YOLOv5 expects 3 output tensors, received {len(outputs)}")
    if not 0.0 < object_threshold < 1.0:
        raise ValueError("object_threshold must be in (0, 1)")
    if not 0.0 < nms_threshold < 1.0:
        raise ValueError("nms_threshold must be in (0, 1)")

    all_boxes: list[np.ndarray] = []
    all_classes: list[np.ndarray] = []
    all_scores: list[np.ndarray] = []

    for output, mask in zip(outputs, ANCHOR_MASKS):
        features = _to_hwa_features(output, len(class_names))
        boxes, objectness, probabilities = _decode_scale(
            features,
            ANCHORS[list(mask)],
            input_size,
        )
        class_ids = np.argmax(probabilities, axis=1)
        class_scores = probabilities[np.arange(len(probabilities)), class_ids]
        scores = objectness * class_scores
        selected = (objectness >= object_threshold) & (class_scores >= object_threshold)
        if np.any(selected):
            all_boxes.append(xywh_to_xyxy(boxes[selected]))
            all_classes.append(class_ids[selected])
            all_scores.append(scores[selected])

    if not all_boxes:
        return []

    boxes = np.concatenate(all_boxes)
    class_ids = np.concatenate(all_classes)
    scores = np.concatenate(all_scores)
    detections: list[DetectionBox] = []

    for class_id in np.unique(class_ids):
        indices = np.flatnonzero(class_ids == class_id)
        kept = nms_indices(boxes[indices], scores[indices], nms_threshold)
        for index in indices[kept]:
            left, top, right, bottom = boxes[index]
            detections.append(
                DetectionBox(
                    class_id=int(class_id),
                    label=class_names[int(class_id)],
                    score=float(scores[index]),
                    left=float(left),
                    top=float(top),
                    right=float(right),
                    bottom=float(bottom),
                )
            )

    detections.sort(key=lambda item: item.score, reverse=True)
    return detections


def restore_letterbox_coordinates(
    detection: DetectionBox,
    original_width: int,
    original_height: int,
    input_size: int = 640,
) -> DetectionBox:
    ratio = min(input_size / original_width, input_size / original_height)
    resized_width = round(original_width * ratio)
    resized_height = round(original_height * ratio)
    pad_x = (input_size - resized_width) / 2.0
    pad_y = (input_size - resized_height) / 2.0

    def x(value: float) -> float:
        return float(np.clip((value - pad_x) / ratio, 0, original_width - 1))

    def y(value: float) -> float:
        return float(np.clip((value - pad_y) / ratio, 0, original_height - 1))

    return DetectionBox(
        class_id=detection.class_id,
        label=detection.label,
        score=detection.score,
        left=x(detection.left),
        top=y(detection.top),
        right=x(detection.right),
        bottom=y(detection.bottom),
    )
