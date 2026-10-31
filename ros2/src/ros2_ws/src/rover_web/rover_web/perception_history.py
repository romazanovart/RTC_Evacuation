"""Three most recent recognition snapshots, persisted off the ROS callback."""
import base64
import hashlib
import json
import logging
import os
from pathlib import Path
import threading
import time
import uuid


class PerceptionHistory:
    def __init__(self, path, clock=time.monotonic):
        self.path = Path(path)
        self.clock = clock
        self.lock = threading.Lock()
        self.changed = threading.Event()
        self.stopping = threading.Event()
        self.items = []
        self.seen = {}
        self.tracks = []
        self.revision = uuid.uuid4().hex
        self.last_error = None
        try:
            if self.path.exists() and self.path.stat().st_size < 8_000_000:
                rows = json.loads(self.path.read_text()).get('items', [])
                self.items = [row for row in rows if isinstance(row, dict)
                              and row.get('kind') in ('object', 'qr')
                              and isinstance(row.get('id'), str)
                              and isinstance(row.get('image_b64'), str)][:3]
        except (OSError, ValueError) as exc:
            self.last_error = str(exc)
        self.writer = threading.Thread(target=self._write_loop, name='recognition-snapshots', daemon=True)
        self.writer.start()

    def _record(self, key, kind, label, text, confidence, image_b64):
        now = self.clock()
        with self.lock:
            previous = self.seen.get(key)
            self.seen = {key: value for key, value in self.seen.items() if now - value < 8.}
            self.seen[key] = now
            # A continuously visible object is one sighting, not a new screenshot per frame.
            if previous is not None and now - previous < 8.:
                return None
            item = {'id': uuid.uuid4().hex, 'kind': kind, 'label': label,
                    'text': text, 'confidence': confidence, 'timestamp': time.time(),
                    'image_b64': image_b64}
            self.items.insert(0, item)
            self.items = self.items[:3]
            self.revision = uuid.uuid4().hex
            self.changed.set()
            return {key: value for key, value in item.items() if key != 'image_b64'}

    def objects(self, detections, image_b64):
        now = self.clock()
        sightings = []
        with self.lock:
            self.tracks = [track for track in self.tracks if now - track['seen'] < 8.]
            matched = set()
            for row in detections:
                box = row['box']
                candidates = []
                for track in self.tracks:
                    if track['id'] in matched or track['class_id'] != row['class_id']:
                        continue
                    old = track['box']
                    intersection = max(0, min(box[2], old[2]) - max(box[0], old[0])) * max(
                        0, min(box[3], old[3]) - max(box[1], old[1]))
                    union = ((box[2] - box[0]) * (box[3] - box[1])
                             + (old[2] - old[0]) * (old[3] - old[1]) - intersection)
                    overlap = intersection / union if union else 0
                    if overlap > .1:
                        candidates.append((overlap, track))
                if candidates:
                    track = max(candidates, key=lambda item: item[0])[1]
                    track.update(box=box, seen=now)
                else:
                    track = {'id': uuid.uuid4().hex, 'class_id': row['class_id'],
                             'box': box, 'seen': now}
                    self.tracks.append(track)
                matched.add(track['id'])
                sightings.append((track['id'], row))
        created = []
        for track_id, row in sightings:
            item = self._record(f'object:{track_id}', 'object', row['label'], '',
                                row['confidence'], image_b64)
            if item is not None:
                created.append(item)
        return created

    def qr(self, candidate):
        if not candidate or candidate.get('candidate') or not candidate.get('text'):
            return
        image = candidate.get('imageData', '')
        if not isinstance(image, str) or not image.startswith('data:image/jpeg;base64,'):
            return
        encoded = image.split(',', 1)[1]
        if len(encoded) > 2_000_000:
            return
        text = str(candidate['text'])
        key = 'qr:' + hashlib.sha256(text.encode()).hexdigest()
        self._record(key, 'qr', 'QR-код', text, None, encoded)

    def metadata(self):
        with self.lock:
            return {'ok': True, 'revision': self.revision, 'save_error': self.last_error,
                    'items': [{**{key: value for key, value in row.items() if key != 'image_b64'},
                               'image_url': f"/api/perception/image?id={row['id']}"}
                              for row in self.items]}

    def image(self, image_id):
        with self.lock:
            encoded = next((row['image_b64'] for row in self.items if row['id'] == image_id), None)
        if encoded is None:
            raise ValueError('Снимок уже заменён более новым распознаванием')
        return base64.b64decode(encoded, validate=True)

    def _write_loop(self):
        while True:
            self.changed.wait()
            self.changed.clear()
            with self.lock:
                snapshot = {'items': list(self.items)}
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                temporary = self.path.with_suffix('.tmp')
                with temporary.open('w', encoding='utf-8') as output:
                    json.dump(snapshot, output, ensure_ascii=False)
                    output.flush()
                    os.fsync(output.fileno())
                temporary.replace(self.path)
                directory = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(directory)
                finally:
                    os.close(directory)
                with self.lock:
                    self.last_error = None
            except OSError as exc:
                logging.warning('Could not save recognition snapshots: %s', exc)
                with self.lock:
                    self.last_error = str(exc)
            if self.stopping.is_set() and not self.changed.is_set():
                return

    def close(self):
        self.stopping.set()
        self.changed.set()
        self.writer.join(timeout=2.)
