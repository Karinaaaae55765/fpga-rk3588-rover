#!/usr/bin/env python3
"""Task 5: live category/color observations and visual locks. No motor I/O."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent


def emit(event, **fields):
    print(json.dumps(dict(event=event, hardware_write=False, **fields), ensure_ascii=True), flush=True)


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=HERE / 'config.json')
    parser.add_argument('--vision-root', type=Path)
    parser.add_argument('--receiver', type=Path)
    parser.add_argument('--model', type=Path)
    parser.add_argument('--label', help='target COCO class; omitted = observe all, no lock')
    parser.add_argument('--color', help='red/orange/yellow/green/cyan/blue/purple/black/gray/white')
    parser.add_argument('--region', default='auto', choices=['auto', 'body', 'upper', 'lower'])
    parser.add_argument('--seconds', type=float, default=0)
    parser.add_argument('--no-display', action='store_true')
    parser.add_argument('--check', action='store_true', help='check files/NPU/display, no UDP capture')
    parser.add_argument('--self-test', action='store_true', help='synthetic tests only; no NPU/network')
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error('seconds must be finite and nonnegative')
    return args


def load_config(path):
    cfg = json.loads(path.read_text(encoding='utf-8'))
    for key in ('min_score', 'max_frame_age', 'max_track_gap', 'min_iou'):
        if not isinstance(cfg[key], (float, int)) or not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError('invalid config: ' + key)
    if cfg['min_score'] > 1 or cfg['min_iou'] > 1:
        raise ValueError('score and IoU must be <= 1')
    for key in ('port', 'source_port'):
        if not isinstance(cfg[key], int) or not 1 <= cfg[key] <= 65535:
            raise ValueError('invalid port')
    return cfg


def run(args):
    if args.self_test:
        return subprocess.call([sys.executable, '-B', str(HERE/'test_perception.py')])
    cfg = load_config(args.config)
    root = args.vision_root or Path(cfg['vision_root'])
    receiver_path = args.receiver or Path(cfg['receiver'])
    model_path = args.model or Path(cfg['model'])
    dependencies = [root/'detector_yolov5_rknn.py', root/'yolov5_postprocess.py',
                    root/'udp_vision/capture.py', root/'udp_vision/display.py', receiver_path, model_path]
    missing = [str(path) for path in dependencies if not path.is_file()]
    if missing:
        raise FileNotFoundError('Existing board dependency missing (no automatic installation): ' + ', '.join(missing))
    # Keep this directory's overlay/tracker ahead of the reused modules.
    sys.path[1:1] = [str(root), str(root/'udp_vision')]
    from capture import UdpCapture, load_receiver, rgb565_to_bgr, is_fresh
    from display import WaylandDisplay, check_display
    from detector_yolov5_rknn import RKNNYoloV5Detector
    from yolov5_postprocess import COCO_CLASSES
    from color_classifier import ColorClassifier
    from observations import observe
    from target_selector import TargetSelector
    from target_tracker import TargetTracker
    from overlay import annotate
    if args.label and args.label not in COCO_CLASSES:
        raise ValueError('unknown COCO class: ' + args.label)
    classifier = ColorClassifier(cfg['color'])
    tracker = TargetTracker(TargetSelector(args.label, args.color, args.region),
                            cfg['confirm_frames'], cfg['min_iou'], cfg['max_track_gap'])
    receiver = load_receiver(receiver_path)
    detector = capture = display = None
    processed = stale = 0
    epoch = None
    try:
        if not args.no_display:
            check_display()
        detector = RKNNYoloV5Detector(model_path, object_threshold=cfg['min_score'])
        if args.check:
            emit('check_ok', npu_initialized=True, camera_tested=False,
                 real_color_accuracy_tested=False)
            return 0
        # Claim the UDP port before opening a window; old service is never killed.
        capture = UdpCapture(receiver, port=cfg['port'], source=cfg['source'],
                             source_port=cfg['source_port'])
        if not args.no_display:
            display = WaylandDisplay()
        started = last_report = time.monotonic()
        emit('listening', source=cfg['source'], port=cfg['port'],
             target=dict(label=args.label, color=args.color, region=args.region),
             vlm_enabled=False, identity_verified=False)
        while not args.seconds or time.monotonic()-started < args.seconds:
            if capture.error:
                raise RuntimeError('capture: ' + capture.error)
            if display and display.error:
                raise RuntimeError('display: ' + display.error)
            frame = capture.latest.get(timeout=.1)
            now = time.monotonic()
            expired = tracker.expire(now)
            if expired:
                emit('tracking', state=expired)
            if frame is not None:
                if frame.epoch != epoch:
                    tracker.reset('stream_restart')
                    epoch = frame.epoch
                if not is_fresh(frame, now, cfg['max_frame_age'], capture.epoch):
                    stale += 1
                    tracker.reset('stale')
                    continue
                image = rgb565_to_bgr(frame.pixels)
                begin = time.monotonic()
                detections = detector.detect_frame(image)
                items = observe(image, detections, classifier, cfg['min_score'])
                now = time.monotonic()
                if not is_fresh(frame, now, cfg['max_frame_age'], capture.epoch):
                    stale += 1
                    emit('tracking', state=tracker.reset('stale_or_restarted'))
                    continue
                state = tracker.update(items, frame.received_at, now)
                if display:
                    display.submit(annotate(image, items, state),
                                   frame.received_at + min(cfg['max_frame_age'], cfg['max_track_gap']))
                processed += 1
                emit('observations', frame_id=frame.frame_id, stream_epoch=frame.epoch,
                     observed_at_monotonic=frame.received_at,
                     time_basis='local_reassembly_not_exposure',
                     processing_ms=round((now-begin)*1000, 2),
                     objects=[item.record() for item in items], tracking=state,
                     target_distance_m=None)
            if now-last_report >= 1:
                emit('stats', **capture.stats(), processed=processed, stale=stale,
                     processing_fps=round(processed/(now-started), 2))
                last_report = now
        return 0 if processed else 2
    finally:
        # Independent display thread clears expired images even during slow inference.
        if capture:
            capture.close()
        if display:
            display.close()
        if detector:
            detector.close()
        emit('finished', processed=processed, stale=stale)


def main():
    try:
        return run(arguments())
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print('ERROR: %s' % exc, file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
