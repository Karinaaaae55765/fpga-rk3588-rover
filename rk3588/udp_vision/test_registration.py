"""Offline tests: no board, no motor command, no model service or uploaded image.

Run with python3 -B test_registration.py. Disk tests use disposable system temp
directories; production targets and the old verification reports are untouched.
"""
import copy
import hashlib
import http.client
import importlib.util
import io
import json
import os
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]
from board_button import ButtonDecoder, BoardButton, INPUT_EVENT
from capture import Frame
from registration import Registration, TargetStore, load_config
from registration_control import ControlServer, request
from vlm_bridge import VLMBridge
from yolov5_postprocess import DetectionBox


class Clock:
    def __init__(self):
        self.now = 100.0
    def __call__(self):
        return self.now


class MemoryStore:
    def __init__(self):
        self.calls = []
    def save(self, crop, metadata):
        self.calls.append((crop.copy(), copy.deepcopy(metadata)))
        return dict(metadata, target_id='sample-%d' % len(self.calls))


def box(left=50, top=30, right=100, bottom=80, label='bottle', score=.9):
    return DetectionBox(39 if label == 'bottle' else 41, label, score, left, top, right, bottom)


def event(value, code=139, kind=1):
    return INPUT_EVENT.pack(0, 0, kind, code, value)


class RegistrationTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.config = load_config(HERE / 'registration_config.json')
        self.config.update(roi=[.1, .1, .9, .9], min_side_pixels=16)
        self.store = MemoryStore()
        self.epoch = 0
        self.reg = Registration(self.config, self.store, self.clock, lambda: self.epoch)
        pattern = (np.indices((120, 160)).sum(axis=0) % 2 * 255).astype(np.uint8)
        self.image = np.repeat(pattern[:, :, None], 3, axis=2)
        self.fid = 0

    def observe(self, boxes=None, image=None, epoch=None):
        self.fid += 1
        self.clock.now += .05
        frame = Frame(self.fid, self.epoch if epoch is None else epoch, self.clock.now, b'')
        result = self.reg.observe(self.image if image is None else image,
                                  [box()] if boxes is None else boxes, frame)
        return result, frame

    def ready(self):
        result = None
        for _ in range(3):
            result, _ = self.observe()
        self.assertEqual(result['state'], 'ready')
        return result['candidate_token']

    def test_three_distinct_frames_required(self):
        for expected in (1, 2):
            result, frame = self.observe()
            self.assertEqual(result['state'], 'stabilizing')
            self.assertEqual(result['stable_frames'], expected)
            repeated = self.reg.observe(self.image, [box()], frame)
            self.assertEqual(repeated['stable_frames'], expected)
        self.assertEqual(self.observe()[0]['state'], 'ready')

    def test_confirm_saves_exact_unannotated_crop_and_metadata(self):
        token = self.ready()
        result = self.reg.confirm(token)
        self.assertTrue(result['ok'])
        crop, metadata = self.store.calls[0]
        np.testing.assert_array_equal(crop, self.image[30:80, 50:100])
        self.assertEqual(metadata['frame_id'], 3)
        self.assertEqual(metadata['epoch'], 0)
        self.assertEqual(metadata['confirmation_source'], 'control')
        self.assertEqual(self.reg.status()['state'], 'registered')
        self.assertFalse(self.reg.status()['hardware_write'])

    def test_duplicate_confirmation_is_idempotent_and_new_frames_do_not_rearm(self):
        token = self.ready()
        first = self.reg.confirm(token)
        second = self.reg.confirm(token)
        self.assertEqual(first['target_id'], second['target_id'])
        for _ in range(4):
            self.observe()
        self.reg.confirm_button()
        self.assertEqual(len(self.store.calls), 1)

    def test_concurrent_button_and_control_only_save_once(self):
        token = self.ready()
        entered, release = threading.Event(), threading.Event()
        original_save = self.store.save
        def slow_save(*args):
            entered.set()
            release.wait(2)
            return original_save(*args)
        self.store.save = slow_save
        results = []
        first = threading.Thread(target=lambda: results.append(self.reg.confirm(token)))
        first.start()
        try:
            self.assertTrue(entered.wait(1))
            self.assertFalse(self.reg.confirm_button()['ok'])
            self.assertFalse(self.reg.arm(self.reg.session_id)['ok'])
        finally:
            release.set()
            first.join(2)
        self.assertTrue(results[0]['ok'])
        self.assertEqual(len(self.store.calls), 1)

    def test_expiry_invalidates_confirm_even_without_new_frames(self):
        token = self.ready()
        self.clock.now += self.config['max_frame_age'] + .01
        self.assertEqual(self.reg.status()['reason'], 'frame_expired')
        self.assertFalse(self.reg.confirm(token)['ok'])
        self.assertEqual(self.store.calls, [])

    def test_live_epoch_change_invalidates_ready_token_without_observe(self):
        token = self.ready()
        self.epoch += 1
        self.assertFalse(self.reg.confirm(token)['ok'])
        self.assertEqual(self.reg.status()['reason'], 'stream_restarted')
        self.assertEqual(self.store.calls, [])

    def test_ready_identity_changes_require_new_token_and_confirmation(self):
        token = self.ready()
        result, _ = self.observe([box(label='cup')])
        self.assertEqual(result['stable_frames'], 1)
        self.assertFalse(self.reg.confirm(token)['ok'])
        self.observe([box(label='cup')])
        result, _ = self.observe([box(label='cup')])
        self.assertNotEqual(token, result['candidate_token'])

    def test_multiple_objects_reject_confirmation(self):
        token = self.ready()
        result, _ = self.observe([box(), box(left=103, right=135)])
        self.assertEqual(result['reason'], 'multiple_objects')
        self.assertFalse(self.reg.confirm(token)['ok'])

    def test_disappearance_invalidates_old_token_when_target_returns(self):
        token = self.ready()
        self.observe([])
        replacement = self.ready()
        self.assertNotEqual(token, replacement)
        self.assertFalse(self.reg.confirm(token)['ok'])

    def test_partial_small_and_blurred_candidates_are_rejected(self):
        cases = [([box(left=5)], self.image, 'object_not_fully_inside'),
                 ([box(right=58)], self.image, 'object_too_small'),
                 ([box()], np.zeros_like(self.image), 'image_not_clear')]
        for boxes, image, reason in cases:
            with self.subTest(reason=reason):
                result, _ = self.observe(boxes, image)
                self.assertEqual(result['reason'], reason)
                self.assertIsNone(result['candidate_token'])

    def test_invalid_detection_and_low_confidence_are_rejected(self):
        for bad in (box(score=.1), box(score=float('nan')), box(left=float('inf')),
                    box(left=100, right=50), box(label='unsupported'), box(score=1.1)):
            result, _ = self.observe([bad])
            self.assertEqual(result['state'], 'waiting')
            self.assertFalse(self.reg.confirm_button()['ok'])

    def test_frame_gap_and_out_of_order_frames_do_not_accumulate_stability(self):
        self.observe()
        self.clock.now += 1
        result, _ = self.observe()
        self.assertEqual(result['stable_frames'], 1)
        self.clock.now += .01
        frame = Frame(self.fid-1, 0, self.clock.now, b'')
        result = self.reg.observe(self.image, [box()], frame)
        self.assertEqual(result['stable_frames'], 1)

    def test_stream_restart_requires_three_new_frames(self):
        self.ready()
        self.epoch += 1
        self.assertEqual(self.observe()[0]['stable_frames'], 1)
        self.assertEqual(self.observe()[0]['stable_frames'], 2)
        self.assertEqual(self.observe()[0]['state'], 'ready')

    def test_arm_is_explicit_and_stale_session_cannot_reset_registration(self):
        token = self.ready()
        self.reg.confirm(token)
        self.assertFalse(self.reg.arm('previous-process')['ok'])
        self.assertEqual(self.reg.status()['state'], 'registered')
        self.assertTrue(self.reg.arm(self.reg.session_id)['ok'])
        self.assertFalse(self.reg.confirm(token)['ok'])
        self.assertNotEqual(self.ready(), token)

    def test_save_failure_is_not_success_and_retry_needs_new_confirmation(self):
        token = self.ready()
        self.store.save = Mock(side_effect=OSError('disk full'))
        result = self.reg.confirm(token)
        self.assertFalse(result['ok'])
        self.assertEqual(result['reason'], 'save_failed')
        self.assertIsNone(self.reg.status()['target_id'])
        self.assertIsNone(self.reg.status()['candidate_token'])
        self.assertFalse(self.reg.confirm(token)['ok'])
        for _ in range(4):
            self.observe()
        self.assertEqual(self.reg.status()['reason'], 'save_failed')
        self.assertEqual(self.reg.status()['state'], 'error')
        self.assertTrue(self.reg.arm(self.reg.session_id)['ok'])
        self.assertEqual(self.observe()[0]['state'], 'stabilizing')

    def test_old_button_event_cannot_confirm_a_newer_frame(self):
        self.ready()
        pressed_at = self.clock.now
        self.observe()
        result = self.reg.confirm_button(pressed_at)
        self.assertFalse(result['ok'])
        self.assertEqual(result['reason'], 'button_event_outdated')
        self.assertEqual(self.store.calls, [])
        self.assertTrue(self.reg.confirm_button(self.clock.now)['ok'])

    def test_real_jpeg_store_has_matching_hash_metadata_and_no_overwrite(self):
        with tempfile.TemporaryDirectory(prefix='registration-test-') as directory:
            self.reg.store = TargetStore(Path(directory) / 'targets')
            first = self.reg.confirm(self.ready())
            self.assertTrue(first['ok'])
            saved = Path(directory) / 'targets' / first['target_id']
            metadata = json.loads((saved / 'metadata.json').read_text())
            jpeg = (saved / 'object.jpg').read_bytes()
            self.assertEqual(hashlib.sha256(jpeg).hexdigest(), metadata['image_sha256'])
            self.assertEqual(cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), 1).shape, (50, 50, 3))
            self.assertFalse(metadata['hardware_write'])
            self.assertFalse(metadata['instance_identity_verified'])
            self.reg.arm(self.reg.session_id)
            second = self.reg.confirm(self.ready())
            self.assertNotEqual(first['target_id'], second['target_id'])
            self.assertEqual((saved / 'object.jpg').read_bytes(), jpeg)
            files = sorted(p.name for p in (Path(directory) / 'targets').glob('*/*'))
            self.assertEqual(files, ['metadata.json', 'metadata.json', 'object.jpg', 'object.jpg'])

    def test_disk_flush_failure_leaves_no_committed_metadata(self):
        with tempfile.TemporaryDirectory(prefix='registration-failure-') as directory:
            self.reg.store = TargetStore(directory)
            token = self.ready()
            with patch('registration.os.fsync', side_effect=OSError('I/O failure')):
                result = self.reg.confirm(token)
            self.assertFalse(result['ok'])
            self.assertEqual(list(Path(directory).glob('*/metadata.json')), [])

    def test_overlay_is_separate_from_saved_pixels(self):
        self.ready()
        original = self.image.copy()
        drawn = self.reg.annotate(self.image)
        np.testing.assert_array_equal(original, self.image)
        self.assertGreater(np.count_nonzero(drawn != self.image), 0)
        self.assertGreater(np.all(drawn == [0, 255, 255], axis=2).sum(), 0)
        self.reg.confirm_button()
        np.testing.assert_array_equal(self.store.calls[0][0], original[30:80, 50:100])

    def test_api_uses_same_confirm_logic_and_rejects_cross_origin(self):
        token = self.ready()
        server = ControlServer(self.reg, port=0)
        port = server.address[1]
        try:
            self.assertEqual(server.address[0], '127.0.0.1')
            self.assertEqual(request(port, 'status')['status']['candidate_token'], token)
            self.assertFalse(request(port, 'confirm', {'candidate_token': 'old-token'})['ok'])
            result = request(port, 'confirm', {'candidate_token': token})
            self.assertTrue(result['ok'])
            self.assertTrue(request(port, 'confirm', {'candidate_token': token})['ok'])
            self.assertEqual(len(self.store.calls), 1)
            connection = http.client.HTTPConnection('127.0.0.1', port, timeout=2)
            connection.request('POST', '/arm', body=json.dumps({'session_id': self.reg.session_id}),
                               headers={'Content-Type': 'application/json', 'Origin': 'http://untrusted.example'})
            response = connection.getresponse()
            self.assertEqual(response.status, 403)
            response.read(); connection.close()
            self.assertEqual(self.reg.status()['state'], 'registered')
            self.assertTrue(request(port, 'arm', {'session_id': self.reg.session_id})['ok'])
        finally:
            server.close()
        self.assertFalse(server.thread.is_alive())

    def test_api_rejects_unknown_operations_without_writing(self):
        self.ready()
        server = ControlServer(self.reg, port=0)
        try:
            for endpoint, body in [('motor', {'speed': 100}), ('confirm', {'candidate_token': []}),
                                   ('arm', {'session_id': 'old'}), ('confirm', {'candidate_token': 'x', 'extra': 1})]:
                self.assertFalse(request(server.address[1], endpoint, body)['ok'])
            self.assertEqual(self.store.calls, [])
        finally:
            server.close()


class ButtonAndVLMTests(unittest.TestCase):
    def test_long_press_repeat_bounce_and_next_real_press(self):
        clock = Clock()
        decoder = ButtonDecoder(139, .25, clock)
        self.assertEqual(decoder.feed(event(1)), 1)
        self.assertEqual(decoder.feed(event(2)*10 + event(1)), 0)
        self.assertEqual(decoder.feed(event(0)+event(1)), 0)
        clock.now += .3
        self.assertEqual(decoder.feed(event(2)), 0)
        self.assertEqual(decoder.feed(event(0)+event(1)), 1)

    def test_partial_reads_other_keys_and_syn_dropped(self):
        clock = Clock()
        decoder = ButtonDecoder(139, .25, clock)
        raw = event(1)
        self.assertEqual(decoder.feed(raw[:5]), 0)
        self.assertEqual(decoder.feed(raw[5:]), 1)
        clock.now += 1
        self.assertEqual(decoder.feed(event(0, code=3, kind=0)+event(1)), 0)
        self.assertEqual(decoder.feed(event(0)+event(1)), 0)
        self.assertEqual(decoder.feed(event(0, code=0, kind=0)+event(0)+event(1)), 1)
        self.assertEqual(decoder.feed(event(1, code=116)), 0)

    def test_monotonic_event_age_rejects_backlog_and_future_events(self):
        clock = Clock()
        decoder = ButtonDecoder(139, .25, clock, max_event_age=.25)
        self.assertEqual(decoder.feed(INPUT_EVENT.pack(98, 0, 1, 139, 1)), 0)
        self.assertEqual(decoder.feed(event(0)+INPUT_EVENT.pack(101, 0, 1, 139, 1)), 0)
        self.assertEqual(decoder.feed(event(0)+INPUT_EVENT.pack(100, 0, 1, 139, 1)), 1)
        self.assertEqual(decoder.accepted_times, [100.0])

    def test_reserved_and_unverified_buttons_are_never_opened(self):
        for code in (116, 142, 143, 152, 205, -1, True):
            with self.assertRaises(ValueError):
                ButtonDecoder(code)
        config = load_config(HERE / 'registration_config.json')['button']
        with patch('board_button.os.open') as opened:
            with self.assertRaises(ValueError):
                BoardButton(config, lambda: None)
            opened.assert_not_called()

    def test_vlm_disabled_never_calls_transport(self):
        transport = Mock()
        self.assertEqual(VLMBridge(transport=transport).describe('id', b'jpeg')['status'], 'disabled')
        transport.assert_not_called()
        self.assertEqual(VLMBridge(enabled=True).describe('id', b'jpeg')['status'], 'not_configured')

    def test_vlm_mock_is_labelled_and_not_identity_evidence(self):
        transport = Mock(return_value=dict(description='A red cup', attributes={'color': 'red'}))
        result = VLMBridge(True, transport).describe('sample-id', b'jpeg')
        self.assertEqual(result['status'], 'ok')
        self.assertTrue(result['simulated'])
        self.assertFalse(result['instance_identity_verified'])
        self.assertFalse(result['hardware_write'])
        self.assertEqual(transport.call_args.kwargs['timeout_seconds'], 5)

    def test_vlm_timeout_failure_and_unexpected_action_are_rejected(self):
        for transport, status in [(Mock(side_effect=TimeoutError), 'timeout'),
                                  (Mock(side_effect=RuntimeError), 'failed'),
                                  (Mock(return_value={'motor_speed': 100}), 'invalid_response')]:
            self.assertEqual(VLMBridge(True, transport).describe('id', b'jpeg')['status'], status)


class MainIntegrationTests(unittest.TestCase):
    def test_main_registration_confirms_without_npu_board_or_production_files(self):
        import main
        import capture
        import detector_yolov5_rknn
        import registration
        import registration_control
        store = MemoryStore()
        servers, results = [], []
        actual_server = registration_control.ControlServer
        def server_factory(reg, port):
            server = actual_server(reg, 0)
            servers.append(server)
            return server
        pattern = (np.indices((720, 1280)).sum(axis=0) % 2 * 65535).astype('>u2').tobytes()
        class Capture:
            epoch, error, socket_buffer = 0, None, 1
            def __init__(self, *args):
                self.index, self.latest = 0, self
            def get(self, timeout):
                self.index += 1
                if self.index <= 3:
                    return Frame(self.index, 0, time.monotonic(), pattern)
                if self.index == 4:
                    port = servers[0].address[1]
                    status = request(port, 'status')['status']
                    results.append(request(port, 'confirm', {'candidate_token': status['candidate_token']}))
                time.sleep(.02)
                return None
            def stats(self):
                return {'frames_ok': 3}
            def close(self):
                pass
        class Detector:
            def __init__(self, *args, **kwargs):
                pass
            def detect_frame(self, image):
                return [box(400, 220, 700, 500)]
            def close(self):
                pass
        args = main.arguments(['--no-display', '--seconds', '.4'])
        with patch.object(capture, 'UdpCapture', Capture), \
             patch.object(detector_yolov5_rknn, 'RKNNYoloV5Detector', Detector), \
             patch.object(registration, 'TargetStore', return_value=store), \
             patch.object(registration_control, 'ControlServer', server_factory), \
             patch('sys.stdout', io.StringIO()):
            self.assertEqual(main.run(args), 0)
        self.assertTrue(results and results[0]['ok'])
        self.assertEqual(len(store.calls), 1)
        self.assertEqual(store.calls[0][1]['frame_id'], 3)
        self.assertFalse(servers[0].thread.is_alive())


if __name__ == '__main__':
    unittest.main(verbosity=2)
