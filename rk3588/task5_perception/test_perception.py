"""Synthetic checks; no NPU, camera, motor I/O or files written."""
import json
from pathlib import Path
import unittest
from dataclasses import replace
from types import SimpleNamespace
import cv2
import numpy as np
from color_classifier import ColorClassifier
from observations import Observation, observe
from target_selector import TargetSelector
from target_tracker import TargetTracker
from overlay import annotate
from main import arguments, load_config

CFG = load_config(Path(__file__).with_name('config.json'))


def item(index=0, box=(10, 10, 110, 210), color='red', label='bottle'):
    return Observation(index, label, .9, box,
                       {'body': dict(name=color, reliable=True, fraction=1.0)}, 0.)


class Tests(unittest.TestCase):
    def setUp(self):
        self.color = ColorClassifier(CFG['color'])

    def test_primary_neutral_colors(self):
        for name, bgr in [('red', (0,0,255)), ('green',(0,255,0)), ('blue',(255,0,0)),
                          ('yellow',(0,255,255)), ('cyan',(255,255,0)), ('purple',(255,0,255)),
                          ('orange',(0,128,255)), ('black',(0,0,0)), ('gray',(110,110,110)),
                          ('white',(255,255,255))]:
            with self.subTest(name=name):
                result = self.color.classify(np.full((30,30,3), bgr, dtype=np.uint8))
                self.assertEqual(result['name'], name)
                self.assertTrue(result['reliable'])

    def test_red_hue_wrap(self):
        for h in (0, 179):
            bgr = cv2.cvtColor(np.full((30,30,3),(h,255,200),dtype=np.uint8), cv2.COLOR_HSV2BGR)
            self.assertEqual(self.color.classify(bgr)['name'], 'red')

    def test_mixed_rejected(self):
        image = np.full((30,30,3), (0,0,255), dtype=np.uint8)
        image[:,15:] = (255,0,0)
        result = self.color.classify(image)
        self.assertEqual(result['name'], 'mixed')
        self.assertFalse(result['reliable'])

    def test_small_crop_unknown(self):
        self.assertEqual(self.color.classify(np.zeros((3,3,3),np.uint8))['name'], 'unknown')

    def test_color_validation(self):
        with self.assertRaises(ValueError):
            ColorClassifier(dict(CFG['color'], min_fraction=.4))

    def test_person_regions(self):
        image = np.full((200,100,3),(255,0,0),dtype=np.uint8)
        image[110:] = (0,0,255)
        result = self.color.regions(image,(0,0,100,200),'person')
        self.assertEqual(result['upper']['name'],'blue')
        self.assertEqual(result['lower']['name'],'red')

    def test_roi_avoids_outer_border(self):
        image = np.full((100,100,3),(0,255,0),dtype=np.uint8)
        image[20:85,20:80] = (0,0,255)
        self.assertEqual(self.color.regions(image,(0,0,100,100),'bottle')['body']['name'],'red')

    def test_selection(self):
        selector = TargetSelector('bottle','red')
        self.assertEqual(selector.choose([item(), item(1,color='blue')])[0], 'candidate')
        self.assertEqual(selector.choose([item(), item(1)])[0], 'ambiguous')
        self.assertEqual(selector.choose([item(color='blue')])[0], 'not_found')
        self.assertEqual(TargetSelector().choose([item()])[0], 'observe_only')

    def test_invalid_selection(self):
        for args in [(None,'red','auto'), ('person','red','body'), ('cup','red','upper')]:
            with self.assertRaises(ValueError):
                TargetSelector(*args)

    def test_unknown_color_no_lock(self):
        candidate = replace(item(), colors={'body':dict(name='red',reliable=False)})
        self.assertEqual(TargetSelector('bottle','red').choose([candidate])[0], 'not_found')

    def tracker(self):
        return TargetTracker(TargetSelector('bottle','red'))

    def lock(self, tracker):
        for t in (1.,1.1,1.2):
            state = tracker.update([item()],t,t+.01)
        self.assertTrue(state['confirmed'])
        return state

    def test_multiframe_confirmation(self):
        tracker = self.tracker()
        for t in (1.,1.1):
            self.assertFalse(tracker.update([item()],t,t+.01)['confirmed'])
        self.assertTrue(tracker.update([item()],1.2,1.21)['confirmed'])

    def test_ambiguous_breaks_lock(self):
        tracker = self.tracker()
        self.lock(tracker)
        state = tracker.update([item(),item(1)],1.3,1.31)
        self.assertEqual(state['status'],'ambiguous')
        self.assertIsNone(state['track_id'])

    def test_loss_new_id(self):
        tracker = self.tracker()
        previous = self.lock(tracker)['track_id']
        tracker.update([],1.3,1.31)
        for t in (1.4,1.5,1.6):
            state = tracker.update([item()],t,t+.01)
        self.assertNotEqual(previous,state['track_id'])

    def test_jump_reconfirms(self):
        tracker = self.tracker()
        self.lock(tracker)
        state = tracker.update([item(box=(400,10,500,210))],1.3,1.31)
        self.assertFalse(state['confirmed'])

    def test_stale_duplicate_future(self):
        for observed, now in [(1.2,1.21),(1.3,3.),(4.,3.)]:
            tracker = self.tracker()
            self.lock(tracker)
            self.assertFalse(tracker.update([item()],observed,now)['confirmed'])

    def test_timeout_without_frames(self):
        tracker = self.tracker()
        self.lock(tracker)
        self.assertEqual(tracker.expire(2.1)['status'],'lost_timeout')
        self.assertIsNone(tracker.expire(2.2))

    def test_observation_clipping_and_nan(self):
        image = np.full((100,100,3),(0,0,255),dtype=np.uint8)
        base = dict(label='bottle', score=.9, left=-10,top=-10,right=110,bottom=110)
        boxes = [SimpleNamespace(**base), SimpleNamespace(**dict(base,score=float('nan'))),
                 SimpleNamespace(**dict(base,right=-20))]
        observations = observe(image,boxes,self.color)
        self.assertEqual(len(observations),1)
        self.assertEqual(observations[0].box,(0,0,100,100))
        self.assertEqual(observations[0].colors['body']['name'],'red')
        json.dumps(observations[0].record(),allow_nan=False)

    def test_overlay_no_input_mutation(self):
        image = np.zeros((720,1280,3),dtype=np.uint8)
        result = annotate(image,[item()],dict(status='locked',confirmed=True,observation_index=0,track_id=1))
        self.assertEqual(result.shape,image.shape)
        self.assertGreater(np.count_nonzero(result),0)
        self.assertEqual(np.count_nonzero(image),0)

    def test_cli_and_python310(self):
        import ast
        args = arguments(['--label','person','--color','blue','--region','upper','--seconds','30'])
        self.assertEqual(args.seconds,30)
        for path in Path(__file__).parent.glob('*.py'):
            ast.parse(path.read_text(encoding='utf-8'), feature_version=(3,10))

    def test_live_entry_with_injected_frames(self):
        # Exercise the actual entry/decoder/observation path, with capture and
        # NPU replaced explicitly. No socket or hardware is opened.
        import contextlib
        import io
        import sys
        import time
        from unittest.mock import patch
        from main import run
        root = Path(__file__).resolve().parents[1]
        if not (root/'udp_vision/capture.py').is_file():
            self.skipTest('desktop-only integration: sibling source tree not present')
        old_path = sys.path[:]
        sys.path.extend([str(root), str(root/'udp_vision')])
        import capture
        import detector_yolov5_rknn
        raw = bytes((0xF8, 0x00)) * (1280*720)
        boxes = [SimpleNamespace(label='bottle',score=.9,left=100,top=100,right=300,bottom=600)]
        instances = []

        class FakeCapture:
            def __init__(self, *args, **kwargs):
                self.latest = self
                self.error, self.epoch, self.number, self.closed = None, 0, 0, False
                instances.append(self)
            def get(self, timeout):
                time.sleep(.001)
                self.number += 1
                return capture.Frame(self.number,0,time.monotonic(),raw)
            def stats(self):
                return {}
            def close(self):
                self.closed = True

        class FakeDetector:
            def __init__(self,*args,**kwargs):
                self.closed = False
                instances.append(self)
            def detect_frame(self,image):
                return boxes
            def close(self):
                self.closed = True

        args = arguments(['--vision-root',str(root),'--receiver',
                          str(root.parents[0]/'camera_udp_sensor_uart/linux/receiver.py'),
                          '--model',str(root/'models/yolov5s_relu_rk3588.rknn'),
                          '--no-display','--seconds','.15','--label','bottle','--color','red'])
        output = io.StringIO()
        try:
            with patch.object(capture,'UdpCapture',FakeCapture), \
                    patch.object(capture,'load_receiver',return_value=object()), \
                    patch.object(detector_yolov5_rknn,'RKNNYoloV5Detector',FakeDetector), \
                    contextlib.redirect_stdout(output):
                self.assertEqual(run(args),0)
        finally:
            sys.path[:] = old_path
        rows = [json.loads(line) for line in output.getvalue().splitlines()]
        results = [row for row in rows if row['event']=='observations']
        self.assertTrue(results)
        self.assertEqual(results[0]['objects'][0]['colors']['body']['name'],'red')
        self.assertTrue(all(row['hardware_write'] is False for row in rows))
        self.assertTrue(all(obj.closed for obj in instances))


if __name__ == '__main__':
    unittest.main(verbosity=2)
