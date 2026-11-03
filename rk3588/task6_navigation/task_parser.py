"""Deterministic task parameters; no LLM, ambiguous free text is rejected."""
import math
from dataclasses import dataclass

LABELS = {'水瓶':'bottle','瓶子':'bottle','水杯':'cup','杯子':'cup','人':'person',
          '人物':'person','bottle':'bottle','cup':'cup','person':'person'}
COLORS = {'红色':'red','蓝色':'blue','绿色':'green','黄色':'yellow','黑色':'black',
          '白色':'white','灰色':'gray','橙色':'orange','青色':'cyan','紫色':'purple'}
COLORS.update({v:v for v in list(COLORS.values())})

@dataclass(frozen=True)
class Task:
    mode: str
    label: str
    color: str | None
    distance_m: float

def parse(mode, label, color=None, distance=None):
    mode = {'寻找':'search','跟随':'follow','接近':'approach','可见跟随':'follow-visible'}.get(mode,mode)
    label = LABELS.get(label)
    if mode not in ('search','follow','approach','follow-visible') or label is None:
        raise ValueError('supported: search, follow, approach, follow-visible')
    if mode in ('follow','follow-visible') and label != 'person':
        raise ValueError('follow mode requires person')
    if mode=='approach' and label not in ('cup','bottle'):
        raise ValueError('approach mode requires a floor-standing cup or bottle')
    if color is not None and color not in COLORS:
        raise ValueError('unknown color')
    distance = (0.8 if mode in ('follow','follow-visible') else 0.5) if distance is None else float(distance)
    if not math.isfinite(distance) or not .45 <= distance <= 1.5:
        raise ValueError('target distance must be 0.45..1.5m from axle midpoint')
    return Task(mode,label,COLORS.get(color),distance)
