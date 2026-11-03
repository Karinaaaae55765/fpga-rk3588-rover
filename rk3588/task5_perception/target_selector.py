"""Select by explicit label/color; equally matching objects remain ambiguous."""
from color_classifier import COLORS


class TargetSelector:
    def __init__(self, label=None, color=None, region='auto'):
        if color and not label:
            raise ValueError('--color requires --label')
        if color and color not in COLORS:
            raise ValueError('unsupported color')
        if region not in ('auto', 'body', 'upper', 'lower'):
            raise ValueError('invalid region')
        if label == 'person' and region == 'body':
            raise ValueError('person uses upper/lower, not body')
        if label and label != 'person' and region in ('upper', 'lower'):
            raise ValueError('upper/lower only supported for person')
        self.label, self.color, self.region = label, color, region

    def matches(self, item):
        if item.label != self.label:
            return False
        if not self.color:
            return True
        region = self.region
        if region == 'auto':
            region = 'upper' if item.label == 'person' else 'body'
        value = item.colors.get(region, {})
        return value.get('reliable', False) and value.get('name') == self.color

    def choose(self, observations):
        if not self.label:
            return 'observe_only', None
        candidates = [item for item in observations if self.matches(item)]
        if len(candidates) > 1:
            return 'ambiguous', None
        return ('candidate', candidates[0]) if candidates else ('not_found', None)
