"""Read a verified Linux evdev button without grabbing/changing system input.

Default config is disabled. Never infer a physical key from /dev/input/eventN:
event numbers change across boots. Device name and code need on-board testing.
"""
import math
import os
from pathlib import Path
import select
import struct
import threading
import time

INPUT_EVENT = struct.Struct('@llHHi')
FORBIDDEN_KEYS = {116, 142, 143, 152, 205}  # power, sleep, wakeup, screen lock, suspend


class ButtonDecoder:
    def __init__(self, key_code, debounce_seconds=.25, clock=time.monotonic, max_event_age=None):
        if type(key_code) is not int or not 1 <= key_code <= 767 or key_code in FORBIDDEN_KEYS:
            raise ValueError('not an allowed confirmation key')
        if not math.isfinite(debounce_seconds) or debounce_seconds <= 0:
            raise ValueError('invalid debounce_seconds')
        self.key_code, self.debounce, self.clock = key_code, debounce_seconds, clock
        self.down = False
        self.last_press = -math.inf
        self.pending = b''
        self.desynced = False
        self.max_event_age = max_event_age
        self.accepted_times = []

    def feed(self, data):
        self.pending += data
        presses = 0
        self.accepted_times = []
        while len(self.pending) >= INPUT_EVENT.size:
            raw, self.pending = self.pending[:INPUT_EVENT.size], self.pending[INPUT_EVENT.size:]
            sec, usec, kind, code, value = INPUT_EVENT.unpack(raw)
            if kind == 0 and code == 3:  # SYN_DROPPED: ignore presses until a release.
                self.down = True
                self.desynced = True
                continue
            if self.desynced:
                if kind == 0 and code == 0:  # SYN_REPORT ends the discarded event batch.
                    self.desynced = False
                continue
            if kind != 1 or code != self.key_code:
                continue
            if value == 0:
                self.down = False
            elif value == 1 and not self.down:
                self.down = True
                now = self.clock()
                event_at = sec + usec/1000000 if self.max_event_age is not None else now
                if self.max_event_age is not None and not 0 <= now-event_at < self.max_event_age:
                    continue
                if now - self.last_press >= self.debounce:
                    self.last_press = now
                    presses += 1
                    self.accepted_times.append(event_at)
            # value=2 is keyboard auto-repeat, never a new confirmation.
        return presses


class BoardButton:
    def __init__(self, config, callback):
        if not config.get('enabled') or not config.get('verified'):
            raise ValueError('button mapping must be explicitly verified before enabling')
        self.decoder = ButtonDecoder(config['key_code'], config['debounce_seconds'], max_event_age=.25)
        name = config['device_name']
        if not isinstance(name, str) or not name.strip() or 'pwrkey' in name.lower():
            raise ValueError('invalid or reserved input device')
        matches = []
        for event in Path('/sys/class/input').glob('event*'):
            try:
                if (event / 'device/name').read_text().strip() == name:
                    matches.append(Path('/dev/input') / event.name)
            except OSError:
                continue
        if len(matches) != 1:
            raise ValueError('expected exactly one input device named ' + name)
        self.callback, self.error = callback, None
        self.stop = threading.Event()
        self.fd = os.open(matches[0], os.O_RDONLY | os.O_NONBLOCK)
        try:
            import fcntl
            # Linux input.h: _IOW('E', 0xa0, int). Changes this reader's clock only.
            clock_ioctl = (1 << 30) | (struct.calcsize('i') << 16) | (ord('E') << 8) | 0xa0
            fcntl.ioctl(self.fd, clock_ioctl, struct.pack('i', time.CLOCK_MONOTONIC))
            self.thread = threading.Thread(target=self._run, name='registration-button', daemon=True)
            self.thread.start()
        except BaseException:
            os.close(self.fd)
            raise

    def _run(self):
        try:
            while not self.stop.is_set():
                readable, _, _ = select.select([self.fd], [], [], .1)
                if not readable:
                    continue
                data = os.read(self.fd, INPUT_EVENT.size*64)
                if not data:
                    raise OSError('input device disconnected')
                # Never enqueue presses to confirm some future candidate.
                self.decoder.feed(data)
                for event_at in self.decoder.accepted_times:
                    self.callback(event_at)
        except Exception as exc:
            self.error = str(exc)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        os.close(self.fd)
