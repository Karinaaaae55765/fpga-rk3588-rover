"""Read-only V3 UART telemetry. Never writes command bytes to the serial port."""
import binascii
import os
import select
import struct
import threading
import time


class Parser:
    HEADER = b"\xa5\x5a\x01\x10\x15"

    def __init__(self):
        self.buffer = bytearray()
        self.good = self.bad = self.duplicates = 0
        self.last_sequence = self.last_uptime = None

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 5:
            if self.buffer[:5] != self.HEADER:
                del self.buffer[0]
                continue
            if len(self.buffer) < 28:
                break
            raw = bytes(self.buffer[:28])
            if binascii.crc_hqx(raw[2:26], 0xffff) != int.from_bytes(raw[26:], "little"):
                self.bad += 1
                del self.buffer[0]
                continue
            del self.buffer[:28]
            seq, flags, distance, strength, roll, pitch, yaw, tof_errors, imu_errors, uptime = struct.unpack(
                "<HBHHhhhHHI", raw[5:26]
            )
            if flags & 0xc0:
                self.bad += 1
                continue
            if self.last_sequence is not None:
                forward = (seq - self.last_sequence) & 0xffff
                # A reset of FPGA uptime may start a new sequence epoch.
                reset = uptime < self.last_uptime and self.last_uptime - uptime < 0x80000000
                if not reset and (forward == 0 or forward >= 0x8000):
                    self.duplicates += 1
                    continue
            self.last_sequence, self.last_uptime = seq, uptime
            self.good += 1
            tof_valid = flags & 0x05 == 0x05 and strength >= 100 and distance > 0
            imu_valid = flags & 0x0a == 0x0a
            frames.append(dict(sequence=seq, uptime_ms=uptime, flags=flags,
                               distance_mm=distance if tof_valid else None, strength=strength,
                               roll=roll / 100 if imu_valid else None,
                               pitch=pitch / 100 if imu_valid else None,
                               yaw=yaw / 100 if imu_valid else None,
                               tof_errors=tof_errors, imu_errors=imu_errors))
        return frames


class Telemetry:
    def __init__(self, device, clock=time.monotonic, voice=None):
        self.device, self.clock = device, clock
        self.lock = threading.Lock()
        self.parser = Parser()
        self.latest = self.received = None
        self.error = None
        self.stop = threading.Event()
        self.thread = None
        self.voice = voice
        self.route_parser = self.route_observer = None
        self.route_received = None
        if voice:
            from status_voice.telemetry import RouteParser
            from status_voice import RouteObserver
            self.route_parser, self.route_observer = RouteParser(), RouteObserver(voice)

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def snapshot(self):
        with self.lock:
            age = self.clock() - self.received if self.received is not None else None
            fresh = self.error is None and age is not None and 0 <= age < .5
            return dict(connected=fresh, age_ms=round(age * 1000) if age is not None else None,
                        values=dict(self.latest) if fresh else None, error=self.error,
                        frames_ok=self.parser.good, crc_errors=self.parser.bad,
                        ignored_sequence=self.parser.duplicates, hardware_write=False)

    def _run(self):
        fd = saved = None
        exclusive = False
        try:
            import fcntl
            import termios
            fd = os.open(self.device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(fd, termios.TIOCEXCL)
            exclusive = True
            saved = termios.tcgetattr(fd)
            attrs = termios.tcgetattr(fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[4] = attrs[5] = termios.B115200
            attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            while not self.stop.is_set():
                if not select.select([fd], [], [], .1)[0]:
                    if self.route_observer and self.route_received is not None and self.clock()-self.route_received>=.5:
                        self.route_observer.lost()
                    continue
                data = os.read(fd, 4096)
                if not data:
                    raise OSError("serial device disconnected")
                self.accept(data)
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
            if self.voice:self.voice.failed('serial_error')
        finally:
            if fd is not None:
                try:
                    if saved is not None:
                        termios.tcsetattr(fd, termios.TCSANOW, saved)
                    if exclusive:
                        fcntl.ioctl(fd, termios.TIOCNXCL)
                except OSError:
                    pass
                os.close(fd)

    def accept(self, data):
        previous_uptime = self.latest.get('uptime_ms') if self.latest else None
        with self.lock:
            frames = self.parser.feed(data)
            if frames:
                self.latest, self.received = frames[-1], self.clock()
        if self.route_observer:
            now = self.clock()
            if frames and previous_uptime is not None and frames[-1]['uptime_ms'] < previous_uptime:
                self.route_observer.reset()
                self.route_parser.buffer.clear()
            for row in self.route_parser.feed(data):
                self.route_observer.observe(row, now)
                self.route_received = now
            if self.route_received is not None and now-self.route_received>=.5:
                self.route_observer.lost()

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=2)
