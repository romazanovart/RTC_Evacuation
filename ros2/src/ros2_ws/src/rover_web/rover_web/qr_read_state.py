"""Keep a genuine decode event separate from per-frame search candidates."""
import copy
import math


class QRReadState:
    def __init__(self):
        self.clear()

    def clear(self):
        self.decoded = None
        self.received = 0.
        self.captured = 0.

    def update(self, candidate, now, capture_age=0.):
        if not isinstance(candidate,dict) or candidate.get('candidate',True) or not candidate.get('text'):
            return
        try:
            capture_age=float(capture_age)
        except (ValueError,TypeError):
            return
        if not math.isfinite(capture_age) or not 0 <= capture_age <= 2.:
            return
        self.decoded=copy.deepcopy(candidate)
        self.received=now
        self.captured=now-capture_age

    def snapshot(self, now, include_image=True):
        decoded=copy.deepcopy(self.decoded)
        if decoded is not None and not include_image:
            decoded.pop('imageData',None)
        return {'decoded':decoded,
                'decoded_age_sec':max(0.,now-self.captured) if self.decoded is not None else None,
                'decoded_sequence':self.received}
