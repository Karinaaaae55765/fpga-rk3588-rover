from __future__ import annotations

from pathlib import Path

from yolov5_postprocess import (
    DetectionBox,
    decode_yolov5_outputs,
    restore_letterbox_coordinates,
)


class RKNNYoloV5Detector:
    """YOLOv5 image detector for RK3588 using RKNNLite 1.5.2."""

    def __init__(
        self,
        model_path: Path,
        object_threshold: float = 0.25,
        nms_threshold: float = 0.45,
        input_size: int = 640,
    ) -> None:
        import cv2
        import numpy as np
        from rknnlite.api import RKNNLite

        self._cv2 = cv2
        self._np = np
        self._rknn = RKNNLite(verbose=False)
        self.object_threshold = object_threshold
        self.nms_threshold = nms_threshold
        self.input_size = input_size

        result = self._rknn.load_rknn(str(model_path))
        if result != 0:
            raise RuntimeError(f"load_rknn failed: {result}")
        result = self._rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
        if result != 0:
            self._rknn.release()
            raise RuntimeError(f"init_runtime failed: {result}")

    def _letterbox(self, image):
        height, width = image.shape[:2]
        ratio = min(self.input_size / width, self.input_size / height)
        new_width = round(width * ratio)
        new_height = round(height * ratio)
        resized = self._cv2.resize(
            image,
            (new_width, new_height),
            interpolation=self._cv2.INTER_LINEAR,
        )
        pad_x = self.input_size - new_width
        pad_y = self.input_size - new_height
        left = pad_x // 2
        right = pad_x - left
        top = pad_y // 2
        bottom = pad_y - top
        return self._cv2.copyMakeBorder(
            resized,
            top,
            bottom,
            left,
            right,
            self._cv2.BORDER_CONSTANT,
            value=(0, 0, 0),
        )

    def detect(self, image_path: Path) -> tuple[object, list[DetectionBox]]:
        original = self._cv2.imread(str(image_path))
        if original is None:
            raise ValueError(f"cannot read image: {image_path}")
        return original, self.detect_frame(original)

    def detect_frame(self, original) -> list[DetectionBox]:
        """Detect an in-memory uint8 BGR frame without image-file I/O."""
        if (
            original is None
            or original.ndim != 3
            or original.shape[2] != 3
            or original.shape[0] == 0
            or original.shape[1] == 0
            or original.dtype != self._np.uint8
        ):
            raise ValueError("frame must be a non-empty uint8 BGR image")
        height, width = original.shape[:2]
        model_input = self._letterbox(original)
        model_input = self._cv2.cvtColor(model_input, self._cv2.COLOR_BGR2RGB)

        outputs = self._rknn.inference(inputs=[model_input])
        if not outputs:
            raise RuntimeError("inference returned no outputs")

        decoded = decode_yolov5_outputs(
            outputs,
            object_threshold=self.object_threshold,
            nms_threshold=self.nms_threshold,
            input_size=self.input_size,
        )
        restored = [
            restore_letterbox_coordinates(item, width, height, self.input_size)
            for item in decoded
        ]
        return restored

    def annotate(self, image, detections: list[DetectionBox]):
        annotated = image.copy()
        for item in detections:
            left, top, right, bottom = map(
                int,
                (item.left, item.top, item.right, item.bottom),
            )
            self._cv2.rectangle(
                annotated,
                (left, top),
                (right, bottom),
                (0, 255, 0),
                2,
            )
            self._cv2.putText(
                annotated,
                f"{item.label} {item.score:.2f}",
                (left, max(20, top - 6)),
                self._cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
        return annotated

    def save_annotated(self, output_path: Path, image, detections) -> None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not self._cv2.imwrite(
            str(output_path),
            self.annotate(image, detections),
        ):
            raise RuntimeError(f"cannot write annotated image: {output_path}")

    def close(self) -> None:
        self._rknn.release()
