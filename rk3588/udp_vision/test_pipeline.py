"""Run with -B. --report writes only output/tests.log and verification.json."""
import argparse
import base64
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import types
import unittest
from unittest.mock import patch

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
from capture import Frame, LatestFrame, UdpCapture, is_fresh, load_receiver, rgb565_to_bgr
from display import DisplayState
from overlay import YELLOW_BGR, annotate, box_pixels
from yolov5_postprocess import DetectionBox, restore_letterbox_coordinates

RECEIVER_PATH = HERE.parents[1] / "camera_udp_sensor_uart/linux/receiver.py"
receiver = load_receiver(RECEIVER_PATH)
REAL_INFERENCE = {}
REAL_PREVIEW = None


def packets(raw, fid=1, width=4, height=2, chunk=8):
    count = len(raw) // chunk
    result = []
    for index in range(count + 1):
        final = index == count
        payload = b"" if final else raw[index * chunk:(index + 1) * chunk]
        result.append(receiver.HEADER.pack(b"FUDP", 1, 1, int(final), fid, index,
                      count, width, height, len(payload), 32, len(raw), 0) + payload)
    return result


def box(left=10, top=20, right=80, bottom=70, score=0.9, label="bottle"):
    return DetectionBox(39, label, score, left, top, right, bottom)


class TextRecorder:
    FONT_HERSHEY_SIMPLEX = 0
    LINE_AA = 16

    def __init__(self):
        self.calls = []

    def putText(self, image, text, position, font, scale, color, thickness, line_type):
        self.calls.append((text, position, color))


class PipelineTests(unittest.TestCase):
    def test_rgb565_network_byte_order_and_channels(self):
        raw = bytes.fromhex("f800 07e0 001f ffff 0000 8410")
        actual = rgb565_to_bgr(raw, 6, 1)
        np.testing.assert_array_equal(actual[0], [[0, 0, 255], [0, 255, 0],
                                     [255, 0, 0], [255, 255, 255], [0, 0, 0], [132, 130, 132]])
        self.assertTrue(actual.flags.c_contiguous)
        with self.assertRaises(ValueError):
            rgb565_to_bgr(raw[:-1], 6, 1)

    def test_yellow_pixels_label_and_source_unchanged(self):
        image = np.zeros((100, 200, 3), np.uint8)
        text = TextRecorder()
        output, selected = annotate(image, [box()], cv2_module=text)
        self.assertEqual(len(selected), 1)
        np.testing.assert_array_equal(output[20, 10], YELLOW_BGR)
        np.testing.assert_array_equal(output[70, 80], YELLOW_BGR)
        self.assertFalse(image.any())
        self.assertFalse(output[45, 45].any())
        self.assertEqual(text.calls[-1][0], "bottle 90%")
        self.assertEqual(text.calls[-1][2], YELLOW_BGR)

    def test_box_clipping_and_bad_coordinates(self):
        self.assertEqual(box_pixels(box(-20, -10, 240, 110), 200, 100), (0, 0, 199, 99))
        for item in [box(right=float("nan")), box(left=100, right=50),
                     box(left=201, right=250), box(score=float("inf")), box(score=1.1)]:
            self.assertIsNone(box_pixels(item, 200, 100))

    def test_class_confidence_filter_and_empty_result(self):
        image = np.zeros((100, 200, 3), np.uint8)
        data = [box(score=.39), box(label="person"), box()]
        out, selected = annotate(image, data, labels=["bottle"], cv2_module=TextRecorder())
        self.assertEqual(selected, [data[2]])
        blank, selected = annotate(image, [], cv2_module=TextRecorder())
        self.assertEqual(selected, [])
        np.testing.assert_array_equal(blank, image)

    def test_letterbox_720p_mapping(self):
        # 1280x720 -> 640x360, with 140 pixels of vertical padding at each side.
        restored = restore_letterbox_coordinates(box(64, 176, 320, 320), 1280, 720, 640)
        np.testing.assert_allclose([restored.left, restored.top, restored.right, restored.bottom],
                                   [128, 72, 640, 360])

    def test_latest_slot_replaces_backlog(self):
        slot = LatestFrame()
        slot.put(Frame(1, 0, 10, b"a"))
        slot.put(Frame(2, 0, 11, b"b"))
        self.assertEqual(slot.get(0).frame_id, 2)
        self.assertEqual(slot.replaced, 1)
        self.assertIsNone(slot.get(0))
        slot.close()
        slot.put(Frame(3, 0, 12, b"c"))
        self.assertIsNone(slot.get(0))

    def test_freshness_expiry_and_sender_restart(self):
        frame = Frame(12, 3, 10, b"image")
        self.assertTrue(is_fresh(frame, 10.9, 1, 3))
        for now, epoch in [(11, 3), (9.9, 3), (10.1, 4), (float("nan"), 3)]:
            self.assertFalse(is_fresh(frame, now, 1, epoch))

    def test_display_blanks_without_inference_progress(self):
        state = DisplayState(6)
        self.assertEqual(state.next_frame(10), bytes(6))
        state.submit(b"abcdef", 11)
        self.assertEqual(state.next_frame(10.5), b"abcdef")
        self.assertIsNone(state.next_frame(10.9))
        # No new submit occurs: the display must still clear the previous result.
        self.assertEqual(state.next_frame(11), bytes(6))
        self.assertIsNone(state.next_frame(11.1))
        state.submit(b"oldold", 10)
        self.assertIsNone(state.next_frame(12))
        state.submit(b"newnew", 14)
        self.assertEqual(state.next_frame(13), b"newnew")

    def test_reassembly_requires_commit_and_all_packets(self):
        raw = bytes(range(16))
        a = receiver.Assembler(4, 2, 8)
        data = packets(raw)
        self.assertIsNone(a.feed(data[1], 1))
        self.assertIsNone(a.feed(data[1], 1.01))
        self.assertIsNone(a.feed(data[0], 1.02))
        self.assertEqual(a.feed(data[2], 1.03), raw)
        self.assertIsNone(a.feed(data[2], 1.04))

    def test_missing_packet_expiry_and_conflicting_duplicate(self):
        raw = bytes(range(16))
        a = receiver.Assembler(4, 2, 8)
        data = packets(raw)
        a.feed(data[0], 1)
        self.assertIsNone(a.feed(data[-1], 1.1))
        a.expire(1.4)
        self.assertEqual(a.stats["missing_packets"], 1)
        a.feed(packets(raw, fid=2)[0], 1.5)
        conflict = packets(b"x" * 16, fid=2)[0]
        self.assertIsNone(a.feed(conflict, 1.51))
        self.assertEqual(a.stats["conflicting_frames"], 1)

    def test_full_720p_frame_and_commit_first(self):
        raw = bytes.fromhex("f80007e0001fffff") * (1280 * 720 // 4)
        data = packets(raw, width=1280, height=720, chunk=1280)
        a = receiver.Assembler()
        self.assertIsNone(a.feed(data[-1], 1))
        result = None
        for p in reversed(data[:-1]):
            value = a.feed(p, 1.1)
            if value is not None:
                result = value
        self.assertEqual(result, raw)
        self.assertEqual(a.stats["frames_ok"], 1)

    def test_frame_id_wrap_and_restart(self):
        a = receiver.Assembler(4, 2, 8)
        raw = bytes(16)
        for fid, now in [(0xffffffff, 1), (0, 1.1), (5, 1.2), (0, 2)]:
            value = None
            for packet in packets(raw, fid):
                value = a.feed(packet, now)
            self.assertEqual(value, raw)
        self.assertEqual(a.stats["input_restarts"], 1)

    def test_real_udp_socket_filters_source_and_shuts_down(self):
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        foreign = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sender.bind(("127.0.0.1", 0))
        fake_module = types.SimpleNamespace(Assembler=lambda: receiver.Assembler(4, 2, 8))
        capture = UdpCapture(fake_module, "127.0.0.1", 0, "127.0.0.1", sender.getsockname()[1])
        try:
            raw = bytes(range(16))
            foreign.sendto(packets(raw)[0], capture.address)
            for packet in packets(raw):
                sender.sendto(packet, capture.address)
            frame = capture.latest.get(2)
            self.assertIsNotNone(frame, "error=%r stats=%r" % (capture.error, capture.stats()))
            self.assertEqual(frame.pixels, raw)
            deadline = time.monotonic() + 1
            while capture.stats()["foreign_packets"] == 0 and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertEqual(capture.stats()["foreign_packets"], 1)
        finally:
            capture.close()
            sender.close()
            foreign.close()
        self.assertFalse(capture.thread.is_alive())
        self.assertIsNone(capture.error)

    @unittest.skipUnless(os.name == "posix", "display pipes use Linux select semantics")
    def test_display_pipe_frame_integrity_and_independent_expiry(self):
        import display as display_module
        original_popen = subprocess.Popen
        children = []
        # An in-memory pipe consumer substitutes only GStreamer, not the write loop.
        code = ("import sys; "
                "frames=[sys.stdin.buffer.read(12) for _ in range(3)]; "
                "print([frame.hex() for frame in frames], flush=True); "
                "sys.stdin.buffer.read()")
        def consumer(*args, **kwargs):
            child = original_popen([sys.executable, "-B", "-u", "-c", code],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE)
            children.append(child)
            return child
        with patch.object(display_module, "check_display"), \
             patch.object(display_module.subprocess, "Popen", consumer):
            display = display_module.WaylandDisplay(4, 1)
            try:
                time.sleep(.08)  # Initial black frame.
                pixels = np.full((1, 4, 3), (0, 255, 255), dtype=np.uint8)
                display.submit(pixels, time.monotonic() + .15)
                import select
                ready, _, _ = select.select([children[0].stdout], [], [], 2)
                self.assertTrue(ready, "display did not clear the expired frame")
                import ast
                frames = ast.literal_eval(children[0].stdout.readline().decode())
                self.assertEqual(frames, [bytes(12).hex(), pixels.tobytes().hex(), bytes(12).hex()])
                self.assertIsNone(display.error)
            finally:
                display.close()
                children[0].stdout.close()

    def test_main_frame_association_and_stale_result_rejection(self):
        import main
        import capture as capture_module
        import detector_yolov5_rknn as detector_module
        import overlay as overlay_module
        class FakeCapture:
            epoch = 0
            error = None
            socket_buffer = 1
            def __init__(self, *args):
                self.latest = LatestFrame()
                pixels = bytes.fromhex("f800") * (1280 * 720)
                self.latest.put(Frame(19, 0, time.monotonic(), pixels))
            def stats(self):
                return {"frames_ok": 1}
            def close(self):
                self.latest.close()
        class FakeDetector:
            delay = 0
            def __init__(self, *args, **kwargs):
                pass
            def detect_frame(self, image):
                np.testing.assert_array_equal(image[0, 0], [0, 0, 255])
                time.sleep(self.delay)
                return [box()]
            def close(self):
                pass
        real_annotate = annotate
        def local_annotate(*args):
            return real_annotate(*args, cv2_module=TextRecorder())
        args = main.arguments(["--no-display", "--seconds", "0.05"])
        with patch.object(capture_module, "UdpCapture", FakeCapture), \
             patch.object(detector_module, "RKNNYoloV5Detector", FakeDetector), \
             patch.object(overlay_module, "annotate", local_annotate):
            output = io.StringIO()
            with patch("sys.stdout", output):
                self.assertEqual(main.run(args), 0)
            rows = [json.loads(line) for line in output.getvalue().splitlines()]
            self.assertEqual([r["frame_id"] for r in rows if r["event"] == "detections"], [19])
            FakeDetector.delay = .03
            args.max_frame_age = .01
            output = io.StringIO()
            with patch("sys.stdout", output):
                self.assertEqual(main.run(args), 2)
            rows = [json.loads(line) for line in output.getvalue().splitlines()]
            self.assertFalse(any(r["event"] == "detections" for r in rows))
            self.assertEqual(rows[-1]["stale_results"], 1)

    def test_real_opencv_text_and_onnx_on_udp_roundtrip(self):
        global REAL_PREVIEW
        if not all(importlib.util.find_spec(m) for m in ("cv2", "onnxruntime")):
            self.skipTest("OpenCV/ONNX Runtime not installed in this interpreter")
        import cv2
        from ai_video_demo import ONNXVideoDetector
        model = HERE.parent / "models/yolov5s_relu.onnx"
        image = cv2.imread(str(HERE.parent / "models/bus.jpg"))
        self.assertIsNotNone(image)
        scale = min(1280 / image.shape[1], 720 / image.shape[0])
        resized = cv2.resize(image, (round(image.shape[1] * scale), round(image.shape[0] * scale)))
        image = np.zeros((720, 1280, 3), np.uint8)
        y, x = (720 - resized.shape[0]) // 2, (1280 - resized.shape[1]) // 2
        image[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
        channels = image.astype(np.uint16)
        words = ((channels[:, :, 2] >> 3) << 11) | ((channels[:, :, 1] >> 2) << 5) | (channels[:, :, 0] >> 3)
        raw = words.astype(">u2").tobytes()
        a = receiver.Assembler()
        frame = None
        for packet in packets(raw, width=1280, height=720, chunk=1280):
            frame = a.feed(packet, 1)
        decoded = rgb565_to_bgr(frame)
        detector = ONNXVideoDetector(model, .4)
        try:
            detections = detector.detect_frame(decoded)
            annotated, selected = annotate(decoded, detections)
            self.assertGreater(len(selected), 0)
            self.assertGreater(np.all(annotated == YELLOW_BGR, axis=2).sum(), 200)
            ok, encoded = cv2.imencode(".jpg", cv2.resize(annotated, (640, 360)),
                                       [cv2.IMWRITE_JPEG_QUALITY, 85])
            self.assertTrue(ok)
            REAL_PREVIEW = base64.b64encode(encoded).decode("ascii")
            REAL_INFERENCE.update(backend="ONNX CPU", image="models/bus.jpg",
                                  udp_frames=1, detections=[{"label": d.label, "score": d.score} for d in selected],
                                  yellow_pixels=int(np.all(annotated == YELLOW_BGR, axis=2).sum()),
                                  board_tested=False)
        finally:
            detector.close()


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_tests():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", action="store_true")
    parser.add_argument("--preview", action="store_true", help="emit JPEG in memory; no image file is written")
    args = parser.parse_args()
    dependencies = [RECEIVER_PATH, HERE.parent / "detector_yolov5_rknn.py",
                    HERE.parent / "yolov5_postprocess.py", HERE.parent / "ai_video_demo.py",
                    HERE.parent / "models/yolov5s_relu_rk3588.rknn"]
    before = {str(p): digest(p) for p in dependencies}
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(PipelineTests)
    # Reuse the existing numerical postprocessing regression suite without editing it.
    spec = importlib.util.spec_from_file_location("existing_yolo_tests", HERE.parent / "tests/test_yolov5_postprocess.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(module))
    log = io.StringIO()
    result = unittest.TextTestRunner(stream=log, verbosity=2).run(suite)
    after = {str(p): digest(p) for p in dependencies}
    unchanged = before == after
    print(log.getvalue())
    report = dict(passed=result.wasSuccessful() and unchanged,
                  platform=sys.platform, python=sys.version,
                  tests_run=result.testsRun, skipped=[reason for _, reason in result.skipped],
                  failures=len(result.failures), errors=len(result.errors),
                  existing_dependencies_unchanged=unchanged, dependency_sha256=after,
                  new_source_sha256={p.name: digest(p) for p in HERE.iterdir() if p.is_file()},
                  real_inference=REAL_INFERENCE or None,
                  board_npu_tested=False, wayland_display_tested=False,
                  limits=["Local reassembly age is not camera-to-display latency",
                          "No FPGA programming, board deployment, or live camera acceptance performed",
                          "TextRecorder tests label calls; actual glyph rendering only runs with OpenCV"])
    if args.report:
        out = HERE / "output"
        if any((out / name).exists() for name in ("tests.log", "verification.json")):
            raise FileExistsError("report already exists; refusing to overwrite")
        out.mkdir(exist_ok=True)
        (out / "tests.log").write_text(log.getvalue(), encoding="utf-8")
        (out / "verification.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=True))
    if args.preview and REAL_PREVIEW:
        print("PREVIEW_JPEG_BASE64=" + REAL_PREVIEW)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(run_tests())

