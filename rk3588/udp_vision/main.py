#!/usr/bin/env python3
"""Live FUDP recognition for camera_udp_sensor_20261019_091257.fs; no motor I/O."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receiver", type=Path, default=HERE.parents[1] / "camera_udp_sensor_uart/linux/receiver.py")
    parser.add_argument("--model", type=Path, default=HERE.parent / "models/yolov5s_relu_rk3588.rknn")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=6102)
    parser.add_argument("--source", default="192.168.50.2")
    parser.add_argument("--source-port", type=int, default=5000)
    parser.add_argument("--min-score", type=float, default=0.4)
    parser.add_argument("--label", action="append", help="COCO label; repeat to select several (default: all)")
    parser.add_argument("--max-frame-age", type=float, default=1.0,
                        help="max seconds since local reassembly, not camera capture")
    parser.add_argument("--seconds", type=float, default=0, help="0 runs until Ctrl+C")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--check", action="store_true", help="load model/check display; do not open UDP")
    parser.add_argument("--registration-config", type=Path, default=HERE / "registration_config.json")
    parser.add_argument("--no-registration", action="store_true", help="run the original detection-only mode")
    args = parser.parse_args(argv)
    if not math.isfinite(args.min_score) or not 0 < args.min_score <= 1:
        parser.error("--min-score must be in (0,1]")
    if not math.isfinite(args.max_frame_age) or args.max_frame_age <= 0:
        parser.error("--max-frame-age must be positive")
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error("--seconds must be nonnegative")
    if not 1 <= args.port <= 65535 or not 1 <= args.source_port <= 65535:
        parser.error("UDP ports must be in 1..65535")
    return args


def emit(event, **fields):
    print(json.dumps(dict(event=event, **fields), ensure_ascii=True), flush=True)


def run(args):
    from capture import load_receiver, UdpCapture, rgb565_to_bgr, is_fresh
    from display import WaylandDisplay, check_display
    from overlay import annotate
    from detector_yolov5_rknn import RKNNYoloV5Detector
    from yolov5_postprocess import COCO_CLASSES

    registration_config = None
    if not args.no_registration:
        from registration import load_config
        registration_config = load_config(args.registration_config)
        if not registration_config['enabled']:
            registration_config = None

    if not args.model.is_file():
        raise FileNotFoundError("RKNN model missing: %s" % args.model)
    if args.label and set(args.label) - set(COCO_CLASSES):
        raise ValueError("unknown labels: " + ", ".join(sorted(set(args.label) - set(COCO_CLASSES))))
    receiver = load_receiver(args.receiver)
    detector = capture = display = None
    registration = control = button = None
    processed = stale = 0
    started = time.monotonic()
    last_report = started
    try:
        if not args.no_display:
            check_display()
        detector = RKNNYoloV5Detector(args.model, object_threshold=args.min_score)
        if args.check:
            emit("check_ok", model=str(args.model), display_checked=not args.no_display,
                 npu_initialized=True, live_capture_tested=False,
                 registration_config_checked=registration_config is not None)
            return 0
        if not args.no_display:
            display = WaylandDisplay()
        capture = UdpCapture(receiver, args.bind, args.port, args.source, args.source_port)
        if registration_config is not None:
            from registration import Registration, TargetStore
            from registration_control import ControlServer
            registration = Registration(registration_config, TargetStore(HERE / 'targets'),
                                        epoch_getter=lambda: capture.epoch)
            control = ControlServer(registration, registration_config['control_port'])
            if registration_config['button']['enabled']:
                from board_button import BoardButton
                try:
                    def confirm_button(event_at):
                        emit('registration_confirm', source='button', **registration.confirm_button(event_at))
                    button = BoardButton(registration_config['button'], confirm_button)
                    registration.button_status = 'active'
                except (OSError, ValueError) as exc:
                    registration.button_status = 'unavailable'
                    emit('registration_button_unavailable', error=str(exc))
            emit('registration_control', bind=control.address[0], port=control.address[1],
                 button=registration.button_status, vlm='disabled', hardware_write=False)
        started = last_report = time.monotonic()
        emit("listening", bind=args.bind, port=args.port, source=args.source,
             source_port=args.source_port, socket_buffer=capture.socket_buffer,
             labels=args.label or "all", max_frame_age=args.max_frame_age,
             motor_commands_enabled=False)
        while not args.seconds or time.monotonic() - started < args.seconds:
            if capture.error:
                raise RuntimeError("capture failed: " + capture.error)
            if display and display.error:
                raise RuntimeError("display failed: " + display.error)
            if button and button.error and registration.button_status != 'unavailable':
                registration.button_status = 'unavailable'
                emit('registration_button_unavailable', error=button.error)
            frame = capture.latest.get(timeout=0.1)
            if frame is not None:
                if not is_fresh(frame, time.monotonic(), args.max_frame_age, capture.epoch):
                    stale += 1
                    if registration:
                        registration.invalidate('frame_expired')
                else:
                    image = rgb565_to_bgr(frame.pixels)
                    infer_start = time.monotonic()
                    detections = detector.detect_frame(image)
                    infer_ms = (time.monotonic() - infer_start) * 1000
                    annotated, selected = annotate(image, detections, args.min_score, args.label)
                    if is_fresh(frame, time.monotonic(), args.max_frame_age, capture.epoch):
                        if registration:
                            registration.observe(image, detections, frame)
                            annotated = registration.annotate(annotated)
                        if display:
                            display.submit(annotated, frame.received_at + args.max_frame_age)
                        processed += 1
                        emit("detections", frame_id=frame.frame_id, stream_epoch=frame.epoch,
                             processing_ms=round(infer_ms, 2),
                             local_frame_age_ms=round((time.monotonic()-frame.received_at)*1000, 2),
                             boxes=[asdict(box) for box in selected])
                    else:
                        stale += 1
                        if registration:
                            registration.invalidate('frame_expired')
            now = time.monotonic()
            if now - last_report >= 1:
                emit("stats", **capture.stats(), processed_frames=processed,
                     stale_results=stale, average_processing_fps=round(processed / (now-started), 2))
                if registration:
                    emit('registration_status', **registration.status())
                last_report = now
        return 0 if processed else 2
    except KeyboardInterrupt:
        return 0
    finally:
        if button:
            button.close()
        if control:
            control.close()
        # Close display before model release, which may be slow in a faulty runtime.
        if capture:
            capture.close()
        if display:
            display.close()
        if detector:
            detector.close()
        if capture:
            emit("summary", **capture.stats(), processed_frames=processed, stale_results=stale)


def main():
    args = arguments()
    try:
        return run(args)
    except Exception as exc:
        print("ERROR: %s" % exc, file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())

