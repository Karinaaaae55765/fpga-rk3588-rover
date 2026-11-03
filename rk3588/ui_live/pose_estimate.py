"""Explicitly initialized, non-absolute wheel/IMU odometry for the printed map."""
from dataclasses import asdict
import math

WIDTH, HEIGHT = 1.9, 1.267


class PoseEstimate:
    def __init__(self, cfg, localizer_type, clock):
        self.cfg, self.localizer_type, self.clock = cfg, localizer_type, clock
        self.localizer = None
        self.reason = 'initial_pose_required'
        self.received = None
        self.trace = []

    def reset(self, x, y, heading_deg):
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in (x,y,heading_deg)):
            raise ValueError('invalid_initial_pose')
        if not 0 <= x <= WIDTH or not 0 <= y <= HEIGHT or not -180 <= heading_deg <= 180:
            raise ValueError('initial_pose_outside_printed_map')
        self.localizer = self.localizer_type(self.cfg, [x,y,math.radians(heading_deg)])
        self.reason = 'waiting_telemetry'
        self.received = None
        self.trace = [[x,y]]

    def update(self, sample):
        if self.localizer is None:
            return
        try:
            pose = self.localizer.update(sample)
        except (ValueError, KeyError, TypeError) as exc:
            self.localizer = None
            self.reason = str(exc)
            self.received = None
            return
        self.received = sample['received_at']
        self.reason = 'estimated_uncalibrated'
        if math.dist(self.trace[-1], [pose.x,pose.y]) >= .005:
            self.trace.append([pose.x,pose.y])
            self.trace = self.trace[-2000:]

    def snapshot(self):
        fresh = self.received is not None and 0 <= self.clock()-self.received < .2
        return dict(available=bool(self.localizer and fresh), estimated=True,
                    reason=self.reason if fresh or self.received is None else 'telemetry_stale',
                    pose=asdict(self.localizer.pose) if self.localizer and fresh else None,
                    trace=list(self.trace), map_width_m=WIDTH, map_height_m=HEIGHT,
                    note='Wheel/IMU estimate; manual origin; drift; configured scales require verification.')
