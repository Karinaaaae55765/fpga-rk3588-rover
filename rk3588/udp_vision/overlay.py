"""Yellow boxes in original-image coordinates; never reuse an old detection."""
import math
import numpy as np

YELLOW_BGR = (0, 255, 255)


def box_pixels(item, width, height):
    values = (item.left, item.top, item.right, item.bottom, item.score)
    if not all(math.isfinite(v) for v in values) or not 0 <= item.score <= 1:
        return None
    if item.right <= item.left or item.bottom <= item.top:
        return None
    left = max(0, math.floor(item.left))
    top = max(0, math.floor(item.top))
    right = min(width - 1, math.ceil(item.right))
    bottom = min(height - 1, math.ceil(item.bottom))
    if left >= right or top >= bottom:
        return None
    return left, top, right, bottom


def annotate(image, detections, min_score=0.4, labels=None, cv2_module=None):
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("expected uint8 BGR image")
    if not math.isfinite(min_score) or not 0 < min_score <= 1:
        raise ValueError("min_score must be in (0,1]")
    if cv2_module is None:
        import cv2 as cv2_module
    out = image.copy()
    height, width = out.shape[:2]
    accepted = []
    for item in detections:
        coordinates = box_pixels(item, width, height)
        if coordinates is None or item.score < min_score or (labels and item.label not in labels):
            continue
        left, top, right, bottom = coordinates
        # Direct array drawing keeps clipping/color explicit and independently testable.
        thickness = min(3, right - left + 1, bottom - top + 1)
        out[top:top + thickness, left:right + 1] = YELLOW_BGR
        out[bottom - thickness + 1:bottom + 1, left:right + 1] = YELLOW_BGR
        out[top:bottom + 1, left:left + thickness] = YELLOW_BGR
        out[top:bottom + 1, right - thickness + 1:right + 1] = YELLOW_BGR
        text = "%s %.0f%%" % (item.label, item.score * 100)
        # A black outline makes yellow labels readable on bright camera backgrounds.
        position = (min(left, max(0, width - 180)), min(height - 4, max(20, top - 7)))
        for color, stroke in (((0, 0, 0), 4), (YELLOW_BGR, 1)):
            cv2_module.putText(out, text, position, cv2_module.FONT_HERSHEY_SIMPLEX,
                               0.6, color, stroke, cv2_module.LINE_AA)
        accepted.append(item)
    return out, accepted

