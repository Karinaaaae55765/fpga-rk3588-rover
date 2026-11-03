"""Register one presented object; no recognition of instance identity or motion.

Only a fresh, stable detection fully inside the guide can be confirmed. Confirm
stores the *unannotated crop from that detection's frame*, never a later frame.
The ready token identifies a continuous candidate episode. UI callers must echo
it; the physical button uses the currently ready token. Re-arming is explicit.
"""
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid

import cv2
import numpy as np
from yolov5_postprocess import COCO_CLASSES


def intersection_over_union(a, b):
    overlap = max(0, min(a.right, b.right)-max(a.left, b.left)) * max(0, min(a.bottom, b.bottom)-max(a.top, b.top))
    union = ((a.right-a.left)*(a.bottom-a.top) + (b.right-b.left)*(b.bottom-b.top) - overlap)
    return overlap/union if union > 0 else 0.0


def load_config(path):
    config = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(config.get('enabled'), bool):
        raise ValueError('enabled must be boolean')
    roi = config['roi']
    if (len(roi) != 4 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in roi)
            or not 0 <= roi[0] < roi[2] <= 1 or not 0 <= roi[1] < roi[3] <= 1):
        raise ValueError('invalid normalized registration ROI')
    for name in ('confirm_frames', 'min_side_pixels', 'control_port'):
        if type(config[name]) is not int or config[name] < 1:
            raise ValueError('invalid ' + name)
    if config['control_port'] > 65535:
        raise ValueError('invalid control_port')
    for name in ('min_score', 'min_iou', 'max_frame_age', 'sharpness_threshold'):
        if not isinstance(config[name], (int, float)) or not math.isfinite(config[name]) or config[name] <= 0:
            raise ValueError('invalid ' + name)
    if config['min_score'] > 1 or config['min_iou'] > 1:
        raise ValueError('threshold must be <= 1')
    button = config['button']
    if type(button['enabled']) is not bool or type(button['verified']) is not bool:
        raise ValueError('button flags must be boolean')
    if button['enabled'] and (not button['verified'] or not button['device_name']
                              or type(button['key_code']) is not int):
        raise ValueError('physical button mapping has not been verified')
    if config['vlm']['enabled'] is not False:
        raise ValueError('live VLM transport is not configured; leave vlm.enabled false')
    return config


class TargetStore:
    """Each transaction creates a new directory, without replacing older samples.

    metadata.json is the commit record. It is written only after the JPEG has
    been flushed. Partial directories after a failed write are retained and
    never reported as successful. A new process starts unarmed for old samples:
    files are archival evidence, not authorization to run a task.
    """
    def __init__(self, root):
        self.root = Path(root)

    def save(self, crop, metadata):
        ok, encoded = cv2.imencode('.jpg', crop, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if not ok:
            raise OSError('JPEG encoding failed')
        jpeg = encoded.tobytes()
        record_id = uuid.uuid4().hex
        directory = self.root / record_id
        directory.mkdir(parents=True, exist_ok=False)
        record = dict(metadata, schema_version=1, target_id=record_id,
                      created_utc=datetime.now(timezone.utc).isoformat(),
                      image_file='object.jpg', image_sha256=hashlib.sha256(jpeg).hexdigest(),
                      image_width=int(crop.shape[1]), image_height=int(crop.shape[0]),
                      instance_identity_verified=False, hardware_write=False)
        for name, data in [('object.jpg', jpeg),
                           ('metadata.json', json.dumps(record, ensure_ascii=False, indent=2).encode('utf-8'))]:
            with (directory / name).open('xb') as stream:
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
        if os.name == 'posix':
            for path in (directory, self.root):
                fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(fd)
                finally:
                    os.close(fd)
        return record


class Registration:
    def __init__(self, config, store, clock=time.monotonic, epoch_getter=None):
        self.config, self.store, self.clock = config, store, clock
        self.epoch_getter = epoch_getter
        self.lock = threading.RLock()
        self.session_id = uuid.uuid4().hex
        self.state, self.reason = 'waiting', 'show_one_object'
        self.token = None
        self.count = 0
        self.previous = None
        self.sample = None
        self.last_key = None
        self.last_received = None
        self.registered = None
        self.completed_token = None
        self.button_status = 'unbound' if not config['button']['enabled'] else 'starting'

    def _reset(self, reason):
        self.state, self.reason = 'waiting', reason
        self.token = self.sample = self.previous = None
        self.count = 0

    def _expire(self):
        if self.state in ('registered', 'saving', 'error') or self.sample is None:
            return
        now = self.clock()
        age = now - self.sample['received_at']
        if not 0 <= age < self.config['max_frame_age']:
            self._reset('frame_expired')
        elif self.epoch_getter and self.epoch_getter() != self.sample['epoch']:
            self._reset('stream_restarted')

    def invalidate(self, reason):
        with self.lock:
            if self.state not in ('registered', 'saving', 'error'):
                self._reset(reason)

    def arm(self, session_id):
        with self.lock:
            if session_id != self.session_id:
                return dict(ok=False, reason='session_changed')
            if self.state == 'saving':
                return dict(ok=False, reason='saving')
            self._reset('show_one_object')
            self.registered = self.completed_token = None
            # Do not let the previously processed frame satisfy confirmation again.
            return dict(ok=True, status=self.status())

    def observe(self, image, detections, frame):
        with self.lock:
            self._expire()
            if self.state in ('registered', 'saving', 'error'):
                return self.status()
            key = (frame.epoch, frame.frame_id)
            now = self.clock()
            if not 0 <= now - frame.received_at < self.config['max_frame_age']:
                self._reset('frame_expired')
                return self.status()
            if self.epoch_getter and self.epoch_getter() != frame.epoch:
                self._reset('stream_restarted')
                return self.status()
            if self.last_key:
                if key == self.last_key or (frame.epoch == self.last_key[0] and
                        (frame.received_at <= self.last_received or
                         not 0 < ((frame.frame_id - self.last_key[1]) & 0xffffffff) < 0x80000000)):
                    return self.status()
                if frame.epoch != self.last_key[0]:
                    self._reset('stream_restarted')
            self.last_key, self.last_received = key, frame.received_at
            height, width = image.shape[:2]
            x1, y1, x2, y2 = self.config['roi']
            guide = (int(width*x1), int(height*y1), int(width*x2), int(height*y2))
            candidates = []
            for box in detections:
                values = (box.left, box.top, box.right, box.bottom, box.score)
                if (box.label not in COCO_CLASSES or not all(math.isfinite(v) for v in values)
                        or not self.config['min_score'] <= box.score <= 1
                        or box.left >= box.right or box.top >= box.bottom):
                    continue
                cx, cy = (box.left+box.right)/2, (box.top+box.bottom)/2
                if guide[0] <= cx <= guide[2] and guide[1] <= cy <= guide[3]:
                    candidates.append(box)
            if len(candidates) != 1:
                self._reset('multiple_objects' if candidates else 'show_one_object')
                return self.status()
            box = candidates[0]
            if not (guide[0] < box.left < box.right < guide[2]
                    and guide[1] < box.top < box.bottom < guide[3]):
                self._reset('object_not_fully_inside')
                return self.status()
            left, top = math.floor(box.left), math.floor(box.top)
            right, bottom = math.ceil(box.right), math.ceil(box.bottom)
            if min(right-left, bottom-top) < self.config['min_side_pixels']:
                self._reset('object_too_small')
                return self.status()
            crop = image[top:bottom, left:right].copy()
            sharpness = float(cv2.Laplacian(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var())
            if sharpness < self.config['sharpness_threshold']:
                self._reset('image_not_clear')
                return self.status()
            same = self.previous and box.label == self.previous.label and intersection_over_union(box, self.previous) >= self.config['min_iou']
            if not same:
                self.count, self.token = 0, None
            self.count += 1
            self.previous = box
            self.sample = dict(crop=crop, box=asdict(box), frame_id=frame.frame_id,
                               epoch=frame.epoch, received_at=frame.received_at,
                               sharpness=sharpness, image_size=[width, height])
            if self.count >= self.config['confirm_frames']:
                self.state, self.reason = 'ready', 'press_confirm'
                if self.token is None:
                    self.token = uuid.uuid4().hex
            else:
                self.state, self.reason = 'stabilizing', 'hold_object_still'
            return self.status()

    def status(self):
        with self.lock:
            self._expire()
            return dict(session_id=self.session_id, state=self.state, reason=self.reason,
                        candidate_token=self.token if self.state == 'ready' else None,
                        stable_frames=min(self.count, self.config['confirm_frames']),
                        required_frames=self.config['confirm_frames'],
                        frame_id=self.sample['frame_id'] if self.sample else None,
                        stream_epoch=self.sample['epoch'] if self.sample else None,
                        candidate=self.sample['box'] if self.sample else None,
                        target_id=self.registered['target_id'] if self.registered else None,
                        button=self.button_status, vlm='disabled', hardware_write=False)

    def confirm(self, token, source='control', event_at=None):
        if source not in ('control', 'button'):
            return dict(ok=False, reason='invalid_source')
        with self.lock:
            self._expire()
            if event_at is not None:
                if (not math.isfinite(event_at) or not 0 <= self.clock()-event_at < self.config['max_frame_age']
                        or (self.sample and self.sample['received_at'] > event_at)):
                    return dict(ok=False, reason='button_event_outdated')
            if token and token == self.completed_token and self.registered:
                return dict(ok=True, reason='already_registered', target_id=self.registered['target_id'])
            if self.state != 'ready' or not token or token != self.token:
                return dict(ok=False, reason=self.reason if self.state != 'ready' else 'candidate_changed')
            sample = self.sample
            metadata = {k: v for k, v in sample.items() if k not in ('crop', 'received_at')}
            metadata.update(confirmation_source=source, confirmed_candidate_token=token,
                            local_frame_age_ms=round((self.clock()-sample['received_at'])*1000, 2),
                            detection_box_semantics='model_estimate_not_verified_object_boundary')
            self.state, self.reason = 'saving', 'saving_sample'
        try:
            record = self.store.save(sample['crop'], metadata)
        except Exception as exc:
            with self.lock:
                self._reset('save_failed')
                self.state = 'error'
            return dict(ok=False, reason='save_failed', error=type(exc).__name__)
        with self.lock:
            self.registered, self.completed_token = record, token
            self.state, self.reason = 'registered', 'target_saved_no_motion'
            self.token = self.sample = self.previous = None
            return dict(ok=True, reason='registered', target_id=record['target_id'])

    def confirm_button(self, event_at=None):
        with self.lock:
            token = self.token if self.state == 'ready' else self.completed_token
        # confirm checks the event time against its snapshot under the same lock.
        return self.confirm(token, 'button', event_at)

    def annotate(self, image):
        status = self.status()
        out = image.copy()
        height, width = out.shape[:2]
        x1, y1, x2, y2 = self.config['roi']
        cv2.rectangle(out, (int(width*x1), int(height*y1)), (int(width*x2), int(height*y2)), (0, 255, 255), 2)
        # ASCII remains readable on the current OpenCV font without installing fonts.
        lines = ['REGISTER: %s | %s' % (status['state'].upper(), status['reason']),
                 'CONFIRM saves this object. Vehicle remains stopped.']
        if status['target_id']:
            lines[1] = 'Saved target: ' + status['target_id'][:12] + ' | NO MOTOR CONTROL'
        for index, line in enumerate(lines):
            y = 27 + index*27
            cv2.rectangle(out, (0, y-22), (min(width-1, 1000), y+5), (0, 0, 0), -1)
            cv2.putText(out, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 1, cv2.LINE_AA)
        return out
