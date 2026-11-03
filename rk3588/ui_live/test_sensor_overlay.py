import unittest
from sensor_overlay import panel_lines, annotate_sensors
from pipeline import LivePipeline
from types import SimpleNamespace

class OverlayTests(unittest.TestCase):
    def test_units_sign_and_invalid(self):
        lines = panel_lines(dict(connected=True, values=dict(roll=-12.34, pitch=2.5, yaw=None, distance_mm=678)))
        text = '\n'.join(lines)
        self.assertIn('-12.34 deg', text)
        self.assertIn('+2.50 deg', text)
        self.assertIn('678 mm', text)
        self.assertIn('IMU: INVALID', text)
    def test_stale_never_renders_old_values(self):
        text = '\n'.join(panel_lines(dict(connected=False, values=dict(roll=99, distance_mm=999))))
        self.assertNotIn('999', text)
        self.assertIn('STALE', text)
    def test_shared_provider_no_new_serial(self):
        snapshot = dict(connected=True, values=dict(roll=1, pitch=2, yaw=3, distance_mm=500))
        telemetry = SimpleNamespace(snapshot=lambda: snapshot)
        pipeline = LivePipeline(SimpleNamespace(), telemetry=telemetry)
        self.assertIs(pipeline.telemetry, telemetry)
    def test_draw_and_recovery(self):
        calls=[]
        cv=SimpleNamespace(FONT_HERSHEY_SIMPLEX=0, LINE_AA=0,
                           rectangle=lambda *a: None, putText=lambda *a: calls.append(a[1]))
        image=SimpleNamespace(shape=(720,1280,3))
        annotate_sensors(image, None, cv)
        self.assertIn('STALE', calls[0])
        calls.clear()
        annotate_sensors(image, dict(connected=True, values=dict(roll=0,pitch=0,yaw=0,distance_mm=10)), cv)
        self.assertIn('LIVE', calls[0])
        self.assertIn('ToF: 10 mm', calls[2])

if __name__=='__main__': unittest.main()
