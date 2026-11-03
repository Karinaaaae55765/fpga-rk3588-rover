"""One task-five computation shared by the screen, web and motion controller."""
import copy
from collections import deque
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time


class RegistrationHighlight:
    """Display-only continuous association; loss is latched until a new registration."""
    def __init__(self, min_iou=.25, max_gap=.8):
        self.min_iou, self.max_gap = min_iou, max_gap
        self.history = deque(maxlen=32)
        self.target_id = self.previous = self.previous_at = self.epoch = None

    @staticmethod
    def overlap(a, b):
        area = max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
        union = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-area
        return area/union if union>0 else 0.

    def match(self, previous, items, threshold):
        matches = [item for item in items if item.label==previous.label
                   and self.overlap(previous.box,item.box)>=threshold]
        if len(matches)!=1:
            return None
        candidate = matches[0]
        # A changed reliable color is evidence against continuation, not a new identity.
        for region,color in previous.colors.items():
            other = candidate.colors.get(region,{})
            if color.get('reliable') and other.get('reliable') and color['name']!=other['name']:
                return None
        return candidate

    def invalidate(self):
        self.history.clear()
        self.previous = self.previous_at = None
        # Retain target_id: the same saved record may not reacquire after loss.

    def update(self, record, items, frame, now):
        from types import SimpleNamespace
        if self.epoch is not None and frame.epoch!=self.epoch:
            self.invalidate()
        self.epoch = frame.epoch
        if not 0<=now-frame.received_at<self.max_gap:
            self.invalidate()
            return None
        key = (frame.epoch,frame.frame_id)
        if self.history and (self.history[-1][0]==key or frame.received_at<=self.history[-1][1]):
            self.invalidate()
            return None
        self.history.append((key,frame.received_at,items))
        if not record:
            self.target_id = self.previous = self.previous_at = None
            return None
        target_id = record['target_id']
        if target_id!=self.target_id:
            self.target_id = target_id
            self.previous = self.previous_at = None
            anchor_key = (record['epoch'],record['frame_id'])
            history = list(self.history)
            anchor = next((i for i,row in enumerate(history) if row[0]==anchor_key),None)
            if anchor is None or now-history[anchor][1]>=self.max_gap:
                return None
            box = record['box']
            seed = SimpleNamespace(label=box['label'],box=tuple(box[k] for k in ('left','top','right','bottom')),colors={})
            for i,(_key,at,observations) in enumerate(history[anchor:]):
                if self.previous_at is not None and not 0<at-self.previous_at<self.max_gap:
                    self.previous = None
                    return None
                seed = self.match(seed,observations,.8 if i==0 else self.min_iou)
                if seed is None:
                    self.previous = None
                    return None
                self.previous,self.previous_at = seed,at
        elif self.previous is not None:
            if not 0<frame.received_at-self.previous_at<self.max_gap:
                self.previous = None
            else:
                self.previous = self.match(self.previous,items,self.min_iou)
                self.previous_at = frame.received_at
        return self.previous.index if self.previous is not None else None


def load_module(name, path):
    path = Path(path).resolve()
    if name in sys.modules:
        module = sys.modules[name]
        if Path(module.__file__).resolve() != path:
            raise RuntimeError('module collision: ' + name)
        return module
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(name, None)
        raise
    return module


class Task5Bridge:
    def __init__(self, root):
        root = Path(root)
        self.cfg = json.loads((root/'config.json').read_text(encoding='utf-8'))
        classifier = load_module('color_classifier', root/'color_classifier.py')
        self.observations = load_module('_ui_observations', root/'observations.py')
        self.selector = load_module('target_selector', root/'target_selector.py').TargetSelector
        self.tracker_type = load_module('target_tracker', root/'target_tracker.py').TargetTracker
        self.overlay = load_module('_ui_task5_overlay', root/'overlay.py')
        self.classifier = classifier.ColorClassifier(self.cfg['color'])
        self.lock = threading.Lock()
        self.generation = 0
        self.latest = None
        self.epoch = None
        self.registration_highlight = RegistrationHighlight(self.cfg['min_iou'],self.cfg['max_track_gap'])
        self.select(None, None)

    def select(self, label, color):
        with self.lock:
            self.tracker = self.tracker_type(self.selector(label, color),
                self.cfg['confirm_frames'], self.cfg['min_iou'], self.cfg['max_track_gap'])
            self.generation += 1
            self.latest = None
            return self.generation

    def process(self, image, boxes, frame, registration_record=None):
        with self.lock:
            generation = self.generation
        items = self.observations.observe(image, boxes, self.classifier, self.cfg['min_score'])
        now = time.monotonic()
        with self.lock:
            if generation != self.generation:
                return None
            if frame.epoch != self.epoch:
                self.tracker.reset('stream_restart')
                self.epoch = frame.epoch
            state = self.tracker.update(items, frame.received_at, now)
            registered_index = self.registration_highlight.update(registration_record,items,frame,now)
            self.latest = dict(event='observations', generation=generation,
                frame_id=frame.frame_id, stream_epoch=frame.epoch,
                observed_at_monotonic=frame.received_at,
                objects=[item.record() for item in items], tracking=state,
                registration_highlight=dict(observation_index=registered_index,
                    target_id=registration_record['target_id'] if registration_record else None,
                    visible=registered_index is not None, identity_verified=False))
        annotated = self.overlay.annotate(image, items, state)
        cv2 = self.overlay.cv2
        # Overlay association is intentionally separate from the motion tracker's state.
        if registered_index is not None:
            item = next(item for item in items if item.index==registered_index)
            x1,y1,x2,y2 = item.box
            cv2.rectangle(annotated,(x1,y1),(x2,y2),(0,255,0),3)
            position = (max(0,min(x1,annotated.shape[1]-180)),max(80,min(y1-8,annotated.shape[0]-8)))
            for color,stroke in (((0,0,0),3),((0,255,0),1)):
                cv2.putText(annotated,'REGISTERED',position,cv2.FONT_HERSHEY_SIMPLEX,.55,color,stroke)
        cv2.rectangle(annotated,(0,0),(annotated.shape[1],62),(0,0,0),-1)
        cv2.putText(annotated,'TASK5 | CATEGORY + COLOR | '+state['status'],
                    (12,24),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),2)
        cv2.putText(annotated,'Color = approximate ROI; motion status in web console',
                    (12,48),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
        return annotated

    def snapshot(self, now, max_age, generation=None):
        with self.lock:
            row = self.latest
            if (row is None or (generation is not None and row['generation'] != generation)
                    or not 0 <= now-row['observed_at_monotonic'] < max_age):
                return None
            return copy.deepcopy(row)

    def invalidate(self):
        with self.lock:
            self.latest = None
            self.tracker.reset('stream_unavailable')
            self.registration_highlight.invalidate()
