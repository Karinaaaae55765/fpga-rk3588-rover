"""Live vision bridge; reuses installed modules without editing their sources."""
from dataclasses import asdict
import importlib
import json
from pathlib import Path
import sys
import threading
import time


def dependencies(vision_root, receiver, model):
    root = Path(vision_root).resolve()
    for file in (root / "capture.py", root / "registration.py",
                 root / "registration_config.json", root / "overlay.py",
                 root.parent / "detector_yolov5_rknn.py", Path(receiver), Path(model)):
        if not file.is_file():
            raise FileNotFoundError(str(file))
    sys.path.insert(0, str(root.parent))
    sys.path.insert(0, str(root))
    modules = {name: importlib.import_module(name) for name in
               ("cv2", "numpy", "capture", "overlay", "registration", "detector_yolov5_rknn")}
    importlib.import_module("rknnlite.api")
    modules["capture"].load_receiver(receiver)
    config = json.loads((root / "registration_config.json").read_text(encoding="utf-8"))
    if config.get("vlm", {}).get("enabled"):
        raise ValueError("configured VLM is unsupported")
    # Run the installed loader to validate thresholds and source configuration.
    config = modules["registration"].load_config(root / "registration_config.json")
    return modules, config


class LivePipeline:
    def __init__(self, args, telemetry=None):
        self.args = args
        self.telemetry = telemetry
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = None
        self.capture = self.registration = None
        self.jpeg = self.received = self.result = None
        self.npu = "starting"
        self.error = None
        self.processed = 0
        self.started = time.monotonic()
        self.epoch = None
        self.task5 = None

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def fresh(self, now):
        return (not self.error and self.received is not None and
                0 <= now - self.received < self.args.max_age and
                self.capture is not None and not self.capture.error and
                self.epoch == self.capture.epoch)

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            fresh = self.fresh(now)
            vision = dict(self.result) if fresh and self.result else None
            status = dict(connected=fresh, npu=self.npu, error=self.error,
                          frame=vision, processed=self.processed,
                          average_processing_fps=round(self.processed / max(.001, now-self.started), 2),
                          local_frame_age_ms=round((now-self.received)*1000) if self.received is not None else None,
                          expires_in_ms=max(0, int((self.args.max_age-(now-self.received))*1000)) if fresh else 0)
        if self.capture:
            status["capture"] = self.capture.stats()
            if self.capture.error:
                status["error"] = str(self.capture.error)
        registration = self.registration.status() if self.registration else None
        # Never offer a live registration token while pipeline output is stale.
        if registration and not fresh:
            registration = dict(registration, candidate_token=None)
        return dict(vision=status, registration=registration, hardware_write=False,
                    navigation=dict(available=False, reason="no_verified_localization_or_motor_control"),
                    tracking=dict(available=False, reason="not_connected_to_live_tracker"))

    def perception_snapshot(self, now, max_age, generation=None):
        with self.lock:
            fresh = self.fresh(now)
        if not fresh or self.task5 is None:
            return None
        return self.task5.snapshot(now, max_age, generation)

    def image(self):
        with self.lock:
            if not self.fresh(time.monotonic()) or not self.jpeg:
                return None
            return self.jpeg, self.args.max_age-(time.monotonic()-self.received)

    def action(self, operation, body):
        reg = self.registration
        if reg is None:
            return dict(ok=False, reason="registration_unavailable")
        if operation == "confirm":
            with self.lock:
                if not self.fresh(time.monotonic()):
                    return dict(ok=False, reason="frame_expired")
            return reg.confirm(body["candidate_token"])
        if operation == "arm":
            return reg.arm(body["session_id"])
        return dict(ok=False, reason="unsupported_action")

    def _run(self):
        detector = capture = display = button = None
        try:
            m, config = dependencies(self.args.vision_root, self.args.receiver, self.args.model)
            receiver = m["capture"].load_receiver(self.args.receiver)
            # Exclusive socket acquisition precedes NPU initialization. Existing service must stop.
            capture = m["capture"].UdpCapture(receiver, "0.0.0.0", 6102, "192.168.50.2", 5000)
            self.capture = capture
            from task5_bridge import Task5Bridge
            self.task5 = Task5Bridge(self.args.task5_root)
            detector = m["detector_yolov5_rknn"].RKNNYoloV5Detector(Path(self.args.model), object_threshold=.4)
            self.registration = m["registration"].Registration(
                config, m["registration"].TargetStore(Path(__file__).parent / "targets"),
                epoch_getter=lambda: capture.epoch)
            if not self.args.no_display:
                from display import WaylandDisplay
                display = WaylandDisplay()
            if config["button"]["enabled"]:
                from board_button import BoardButton
                try:
                    button = BoardButton(config["button"], self.registration.confirm_button)
                    self.registration.button_status = "active"
                except (OSError, ValueError) as exc:
                    self.registration.button_status = "unavailable"
                    print("BUTTON UNAVAILABLE:", exc, flush=True)
            with self.lock:
                self.npu = "initialized"
            cv2 = m["cv2"]
            while not self.stop.is_set():
                if capture.error:
                    raise RuntimeError(str(capture.error))
                if display and display.error:
                    raise RuntimeError(str(display.error))
                if button and button.error:
                    self.registration.button_status = "unavailable"
                frame = capture.latest.get(timeout=.1)
                if frame is None:
                    continue
                if not m["capture"].is_fresh(frame, time.monotonic(), self.args.max_age, capture.epoch):
                    self.registration.invalidate("frame_expired")
                    continue
                image = m["capture"].rgb565_to_bgr(frame.pixels)
                started = time.monotonic()
                boxes = detector.detect_frame(image)
                processing_ms = (time.monotonic()-started)*1000
                if not m["capture"].is_fresh(frame, time.monotonic(), self.args.max_age, capture.epoch):
                    self.registration.invalidate("frame_expired")
                    continue
                self.registration.observe(image, boxes, frame)
                # Both web confirmation and the physical button update this same record.
                # Only copy association metadata, never the saved crop or confirmation token.
                with self.registration.lock:
                    saved = self.registration.registered if self.registration.state == 'registered' else None
                    registration_record = ({key:saved[key] for key in ('target_id','epoch','frame_id','box')}
                                           if saved else None)
                annotated = self.task5.process(image, boxes, frame, registration_record)
                if annotated is None:
                    continue
                row = self.task5.snapshot(time.monotonic(), self.args.max_age)
                if row is None:
                    continue
                # Keep the registration region without the instructional banner.
                height, width = annotated.shape[:2]
                x1, y1, x2, y2 = config["roi"]
                cv2.rectangle(annotated, (int(width*x1), int(height*y1)),
                              (int(width*x2), int(height*y2)), (0, 255, 255), 2)
                from sensor_overlay import annotate_sensors
                annotated = annotate_sensors(annotated,
                    self.telemetry.snapshot() if self.telemetry else None, cv2)
                if display:
                    display.submit(annotated, frame.received_at+self.args.max_age)
                annotated = cv2.resize(annotated, (960, 540))
                ok, encoded = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if not ok:
                    raise RuntimeError("JPEG encode failed")
                if not m["capture"].is_fresh(frame, time.monotonic(), self.args.max_age, capture.epoch):
                    continue
                with self.lock:
                    self.jpeg, self.received, self.epoch = encoded.tobytes(), frame.received_at, frame.epoch
                    self.processed += 1
                    self.result = dict(frame_id=frame.frame_id, epoch=frame.epoch,
                                       processing_ms=round(processing_ms, 1),
                                       boxes=row["objects"], tracking=row["tracking"],
                                       registration_highlight=row["registration_highlight"])
        except Exception as exc:
            with self.lock:
                self.error, self.npu = str(exc), "error"
                self.jpeg = None
            print("VISION ERROR:", exc, flush=True)
        finally:
            if self.task5:
                self.task5.invalidate()
            if button:
                button.close()
            if display:
                display.close()
            if capture:
                capture.close()
            if detector:
                detector.close()

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=3)
