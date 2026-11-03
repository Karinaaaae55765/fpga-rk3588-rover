"""Independent Wayland display with expiry even while NPU inference is blocked."""
import os
import select
import shutil
import subprocess
import threading
import time


def check_display():
    for executable in ("gst-launch-1.0", "gst-inspect-1.0"):
        if not shutil.which(executable):
            raise RuntimeError("missing executable: " + executable)
    for element in ("fdsrc", "rawvideoparse", "videoconvert", "queue", "waylandsink"):
        if subprocess.call(["gst-inspect-1.0", element], stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL):
            raise RuntimeError("missing GStreamer element: " + element)


class DisplayState:
    """Thread-safe pending image; expire before sending an image to GStreamer."""
    def __init__(self, size):
        self.lock = threading.Lock()
        self.pending = None
        self.black = bytes(size)
        self.deadline = None
        self.showing_black = False

    def submit(self, pixels, expires_at):
        if len(pixels) != len(self.black):
            raise ValueError("wrong display frame size")
        with self.lock:
            self.pending = (pixels, expires_at)

    def next_frame(self, now):
        with self.lock:
            if self.pending is not None:
                pixels, expires = self.pending
                self.pending = None
                if now < expires:
                    self.deadline = expires
                    self.showing_black = False
                    return pixels
            if not self.showing_black and (self.deadline is None or now >= self.deadline):
                self.showing_black = True
                return self.black
            return None


class WaylandDisplay:
    def __init__(self, width=1280, height=720):
        check_display()
        if width * 3 % 4:
            raise ValueError("BGR row width must be aligned to 4 bytes")
        self.state = DisplayState(width * height * 3)
        self.stop = threading.Event()
        self.error = None
        self.child = None
        command = ["gst-launch-1.0", "-q", "fdsrc", "fd=0", "!", "rawvideoparse",
                   "format=bgr", "width=" + str(width), "height=" + str(height),
                   "framerate=15/1", "!", "queue", "max-size-buffers=1",
                   "max-size-bytes=0", "max-size-time=0", "leaky=downstream",
                   "!", "videoconvert", "!", "waylandsink", "fullscreen=true", "sync=false"]
        try:
            self.child = subprocess.Popen(command, stdin=subprocess.PIPE)
            os.set_blocking(self.child.stdin.fileno(), False)
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
        except BaseException:
            if self.child is not None:
                self.child.terminate()
                self.child.wait(timeout=2)
                self.child.stdin.close()
            raise

    def submit(self, image, expires_at):
        self.state.submit(image.tobytes(order="C"), expires_at)

    def _run(self):
        try:
            while not self.stop.is_set():
                if self.child.poll() is not None:
                    raise RuntimeError("GStreamer exited: %s" % self.child.returncode)
                pixels = self.state.next_frame(time.monotonic())
                if pixels is None:
                    self.stop.wait(0.02)
                    continue
                view = memoryview(pixels)
                deadline = time.monotonic() + 1.0
                # Once a raw frame starts, finish it; otherwise byte framing is corrupted.
                while view and not self.stop.is_set():
                    if self.child.poll() is not None:
                        raise RuntimeError("GStreamer exited during frame write")
                    if time.monotonic() >= deadline:
                        raise RuntimeError("display pipe stalled for 1 second")
                    fd = self.child.stdin.fileno()
                    if select.select([], [fd], [], 0.05)[1]:
                        try:
                            written = os.write(fd, view)
                            if written == 0:
                                raise RuntimeError("zero-length display pipe write")
                            view = view[written:]
                        except BlockingIOError:
                            pass
        except Exception as exc:
            self.error = str(exc)
            self.stop.set()
        finally:
            if self.error and self.child.poll() is None:
                self.child.terminate()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.5)
        if self.child.poll() is None:
            self.child.terminate()
        try:
            self.child.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.child.kill()
            self.child.wait()
        self.child.stdin.close()

