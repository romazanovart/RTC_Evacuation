"""Bounded, expiring laptop inference results. No ROS or model dependency."""
import base64
import math
import secrets
import threading
import time
from collections import OrderedDict

LABELS = {0: 'Бензовоз', 1: 'Дерево', 2: 'ДТП', 3: 'QR-код'}
OBJECTS = {0: 'tanker', 1: 'tree', 2: 'accident'}
INPUT_MAX_AGE = 1.0
RESULT_MAX_AGE = 3.0
DISPLAY_MAX_AGE = 1.0


class NeuralOverlay:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.pending = OrderedDict()
        self.latest = None

    def issue(self, jpeg, captured, frame_id):
        now = self.clock()
        if not jpeg or not 0 <= now - captured < INPUT_MAX_AGE:
            raise RuntimeError('Нет свежего кадра для нейросети')
        token = secrets.token_urlsafe(24)
        with self.lock:
            self.pending = OrderedDict((key, value) for key, value in self.pending.items()
                                       if now - value[0] < RESULT_MAX_AGE)
            self.pending[token] = (captured, str(frame_id))
            while len(self.pending) > 8:
                self.pending.popitem(last=False)
        return {'ok': True, 'token': token, 'frame_id': str(frame_id),
                'image_b64': base64.b64encode(jpeg).decode('ascii')}

    def accept(self, payload):
        with self.lock:
            source = self.pending.pop(str(payload.get('token', '')), None)
        if source is None:
            return False
        captured, frame_id = source
        now = self.clock()
        if not 0 <= now - captured < RESULT_MAX_AGE:
            return False
        inference_ms = float(payload.get('inference_ms', 0))
        if not math.isfinite(inference_ms) or not 0 <= inference_ms < RESULT_MAX_AGE * 1000:
            raise ValueError('Invalid inference duration')
        encoded = payload.get('image_b64', '')
        if not isinstance(encoded, str) or len(encoded) > 550_000:
            raise ValueError('Neural frame is too large')
        jpeg = base64.b64decode(encoded, validate=True)
        if not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
            raise ValueError('Expected JPEG frame')
        rows = payload.get('detections', [])
        if not isinstance(rows, list) or len(rows) > 50:
            raise ValueError('Invalid detections')
        detections = []
        for row in rows:
            cls = int(row['class_id'])
            confidence = float(row['confidence'])
            box = [float(v) for v in row['box']]
            if (cls not in LABELS or not math.isfinite(confidence) or not 0 <= confidence <= 1
                    or len(box) != 4 or any(not math.isfinite(v) or not 0 <= v <= 1 for v in box)
                    or box[0] >= box[2] or box[1] >= box[3]):
                raise ValueError('Invalid detection geometry')
            detections.append({'class_id': cls, 'label': LABELS[cls],
                               'confidence': confidence, 'box': box})
        with self.lock:
            # A slow/out-of-order worker must not replace a newer result.
            accepted = self.clock()
            if (accepted - captured >= RESULT_MAX_AGE
                    or (self.latest and captured <= self.latest['captured'])):
                return False
            self.latest = dict(captured=captured, accepted=accepted, frame_id=frame_id, jpeg=jpeg,
                               inference_ms=inference_ms, detections=detections)
        return True

    def frame(self):
        with self.lock:
            if self.latest and 0 <= self.clock() - self.latest['accepted'] < DISPLAY_MAX_AGE:
                return self.latest['jpeg']
        return None

    def status(self):
        with self.lock:
            latest = self.latest
            age = self.clock() - latest['captured'] if latest else None
            active = (latest is not None
                      and 0 <= self.clock() - latest['accepted'] < DISPLAY_MAX_AGE)
            return {'active': active, 'engine': 'laptop', 'model': 'RTK YOLOv8n',
                    'classes': list(LABELS.values()), 'age_sec': age,
                    'inference_ms': latest['inference_ms'] if active else None,
                    'detections': list(latest['detections']) if active else []}
