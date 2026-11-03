"""Serializable visual observations, never motor commands or metric ranges."""
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class Observation:
    index: int
    label: str
    score: float
    box: tuple
    colors: dict
    horizontal_offset: float

    def record(self):
        return asdict(self)


def observe(image, detections, classifier, min_score=0.5):
    import math
    height, width = image.shape[:2]
    result = []
    for item in detections:
        values = (item.score, item.left, item.top, item.right, item.bottom)
        if not all(math.isfinite(v) for v in values) or item.score < min_score:
            continue
        left, top = max(0, int(item.left)), max(0, int(item.top))
        right, bottom = min(width, int(item.right)), min(height, int(item.bottom))
        if right <= left or bottom <= top:
            continue
        box = (left, top, right, bottom)
        result.append(Observation(len(result), item.label, float(item.score), box,
                                  classifier.regions(image, box, item.label),
                                  ((left + right) / width) - 1.0))
    return result
