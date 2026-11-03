"""FUDP input; reuse the receiver shipped with the approved Ethernet image."""
import importlib.util
import ipaddress
import math
from pathlib import Path
import socket
import threading
import time
from dataclasses import dataclass

import numpy as np

WIDTH, HEIGHT = 1280, 720


def load_receiver(path):
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError("FUDP receiver missing: %s" % path)
    spec = importlib.util.spec_from_file_location("udp_vision_fudp_receiver", str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if (module.WIDTH, module.HEIGHT, module.CHUNK) != (WIDTH, HEIGHT, 1280):
        raise ValueError("receiver must match the 1280x720 FUDP v1 bitstream")
    return module


def rgb565_to_bgr(raw, width=WIDTH, height=HEIGHT):
    if len(raw) != width * height * 2:
        raise ValueError("incomplete RGB565 frame")
    words = np.frombuffer(raw, dtype=">u2").reshape(height, width)
    r = (words >> 11) & 31
    g = (words >> 5) & 63
    b = words & 31
    image = np.empty((height, width, 3), dtype=np.uint8)
    image[:, :, 0] = (b << 3) | (b >> 2)
    image[:, :, 1] = (g << 2) | (g >> 4)
    image[:, :, 2] = (r << 3) | (r >> 2)
    return image


@dataclass(frozen=True)
class Frame:
    frame_id: int
    epoch: int
    received_at: float  # Local reassembly completion, NOT camera capture time.
    pixels: bytes


def is_fresh(frame, now, max_age, epoch=None):
    return (math.isfinite(now) and math.isfinite(frame.received_at)
            and 0 <= now - frame.received_at < max_age
            and (epoch is None or frame.epoch == epoch))


class LatestFrame:
    """One pending frame, atomically replaced; no accumulating inference queue."""
    def __init__(self):
        self.condition = threading.Condition()
        self.value = None
        self.replaced = 0
        self.closed = False

    def put(self, value):
        with self.condition:
            if self.closed:
                return
            if self.value is not None:
                self.replaced += 1
            self.value = value
            self.condition.notify()

    def get(self, timeout=0.1):
        with self.condition:
            self.condition.wait_for(lambda: self.value is not None or self.closed, timeout)
            value, self.value = self.value, None
            return value

    def close(self):
        with self.condition:
            self.closed = True
            self.value = None
            self.condition.notify_all()


class UdpCapture:
    def __init__(self, receiver, bind="0.0.0.0", port=6102,
                 source="192.168.50.2", source_port=5000):
        ipaddress.IPv4Address(bind)
        ipaddress.IPv4Address(source)
        if not (0 <= port <= 65535 and 1 <= source_port <= 65535):
            raise ValueError("invalid UDP port")
        self.assembler = receiver.Assembler()
        self.source = (source, source_port)
        self.latest = LatestFrame()
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.error = None
        self.foreign_packets = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
            # No SO_REUSEADDR: a second preview must not compete for these frames.
            self.sock.bind((bind, port))
            self.sock.settimeout(0.1)
            self.address = self.sock.getsockname()
            self.socket_buffer = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
        except BaseException:
            self.sock.close()
            raise

    @property
    def epoch(self):
        with self.lock:
            return self.assembler.stats["input_restarts"]

    def stats(self):
        with self.lock:
            return dict(self.assembler.stats, foreign_packets=self.foreign_packets,
                        pending_replaced=self.latest.replaced)

    def _run(self):
        try:
            while not self.stop.is_set():
                try:
                    packet, source = self.sock.recvfrom(2048)
                except socket.timeout:
                    with self.lock:
                        self.assembler.expire(time.monotonic())
                    continue
                now = time.monotonic()
                with self.lock:
                    if source != self.source:
                        self.foreign_packets += 1
                        continue
                    raw = self.assembler.feed(packet, now)
                    if raw is not None:
                        frame = Frame(self.assembler.output_id,
                                      self.assembler.stats["input_restarts"], now, raw)
                    else:
                        frame = None
                if frame is not None:
                    self.latest.put(frame)
        except Exception as exc:
            if not self.stop.is_set():
                self.error = str(exc)
        finally:
            self.stop.set()

    def close(self):
        self.stop.set()
        self.latest.close()
        self.thread.join(timeout=1)
        self.sock.close()

