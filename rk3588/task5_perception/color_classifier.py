"""HSV baseline on approximate interior crops, NOT instance segmentation.

Fractions describe pixel support, not calibrated probabilities. No color is
inferred for transparent surfaces, identity, or portions outside the crop.
"""
import cv2
import numpy as np

COLORS = ('red', 'orange', 'yellow', 'green', 'cyan', 'blue', 'purple',
          'black', 'gray', 'white')


class ColorClassifier:
    def __init__(self, config):
        self.cfg = config
        if (not isinstance(config['min_pixels'], int) or config['min_pixels'] < 1
                or not 0.5 <= config['min_fraction'] <= 1
                or not 0 <= config['min_margin'] <= 1
                or not 0 <= config['black_v'] < config['white_v'] <= 255
                or not 0 < config['neutral_s'] < 256):
            raise ValueError('invalid color parameters')

    def classify(self, crop):
        unknown = dict(name='unknown', fraction=0.0, runner_up=0.0,
                       reliable=False, method='interior_hsv')
        if crop.size == 0 or crop.shape[0] * crop.shape[1] < self.cfg['min_pixels']:
            return unknown
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        h, s, v = (hsv[:, :, i] for i in range(3))
        black = v < self.cfg['black_v']
        neutral = (s < self.cfg['neutral_s']) & ~black
        chromatic = ~black & ~neutral
        masks = {
            'red': ((h < 10) | (h >= 170)) & chromatic,
            'orange': (h >= 10) & (h < 22) & chromatic,
            'yellow': (h >= 22) & (h < 35) & chromatic,
            'green': (h >= 35) & (h < 85) & chromatic,
            'cyan': (h >= 85) & (h < 100) & chromatic,
            'blue': (h >= 100) & (h < 130) & chromatic,
            'purple': (h >= 130) & (h < 170) & chromatic,
            'black': black,
            'white': neutral & (v >= self.cfg['white_v']),
            'gray': neutral & (v < self.cfg['white_v']),
        }
        ranking = sorted(((float(np.mean(mask)), name) for name, mask in masks.items()),
                         reverse=True)
        (fraction, name), (second, _) = ranking[:2]
        reliable = (fraction >= self.cfg['min_fraction'] and
                    fraction - second >= self.cfg['min_margin'])
        return dict(name=name if reliable else 'mixed', fraction=round(fraction, 3),
                    runner_up=round(second, 3), reliable=reliable, method='interior_hsv')

    def regions(self, image, box, label):
        x1, y1, x2, y2 = box
        width, height = x2 - x1, y2 - y1
        # Upright-person approximation. Bending, occlusion and loose boxes can
        # include skin/background; this baseline cannot certify garment masks.
        regions = ({'upper': (.25, .22, .75, .52), 'lower': (.25, .57, .75, .88)}
                   if label == 'person' else {'body': (.20, .20, .80, .85)})
        result = {}
        for name, (a, b, c, d) in regions.items():
            crop = image[y1 + int(b * height):y1 + int(d * height),
                         x1 + int(a * width):x1 + int(c * width)]
            result[name] = self.classify(crop)
        return result
