"""Conservative single-target association. Track IDs are NOT person identities.

Loss breaks the association. A new target requires fresh confirmation and a new
ID. Multiple matching candidates invalidate the lock instead of choosing one.
"""
import math


def iou(a, b):
    area = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - area
    return area / union if union > 0 else 0.0


class TargetTracker:
    def __init__(self, selector, confirm_frames=3, min_iou=.25, max_gap=.8):
        if (not isinstance(confirm_frames, int) or confirm_frames < 2
                or not 0 < min_iou <= 1 or not math.isfinite(max_gap) or max_gap <= 0):
            raise ValueError('invalid tracker parameters')
        self.selector = selector
        self.required, self.min_iou, self.max_gap = confirm_frames, min_iou, max_gap
        self.next_id = 1
        self.reset('idle')

    def reset(self, reason):
        self.previous = None
        self.previous_at = None
        self.count = 0
        self.track_id = None
        self.state = dict(status=reason, track_id=None, observation_index=None,
                          confirmed=False, identity_verified=False, hardware_write=False)
        return self.state.copy()

    def expire(self, now):
        if self.previous_at is not None and now - self.previous_at >= self.max_gap:
            return self.reset('lost_timeout')
        return None

    def update(self, observations, observed_at, now):
        if (not math.isfinite(now) or not math.isfinite(observed_at)
                or not 0 <= now - observed_at < self.max_gap):
            return self.reset('stale')
        if self.previous_at is not None and observed_at <= self.previous_at:
            return self.reset('duplicate_or_reordered_frame')
        status, item = self.selector.choose(observations)
        if item is None:
            return self.reset(status)
        associated = (self.previous is not None
                      and observed_at - self.previous_at < self.max_gap
                      and self.previous.label == item.label
                      and iou(self.previous.box, item.box) >= self.min_iou)
        if not associated:
            self.reset('reconfirming')
        self.count += 1
        self.previous, self.previous_at = item, observed_at
        confirmed = self.count >= self.required
        if confirmed and self.track_id is None:
            self.track_id, self.next_id = self.next_id, self.next_id + 1
        self.state = dict(status='locked' if confirmed else 'confirming',
                          track_id=self.track_id, observation_index=item.index,
                          confirm_count=self.count, confirmed=confirmed,
                          identity_verified=False, hardware_write=False)
        return self.state.copy()
