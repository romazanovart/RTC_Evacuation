#!/usr/bin/env python3
import base64
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'rover_web'))
from rover_web.neural_overlay import NeuralOverlay
from rover_web.perception_history import PerceptionHistory

JPEG = b'\xff\xd8test\xff\xd9'
ENCODED = base64.b64encode(JPEG).decode()
DETECTION = {'class_id': 1, 'confidence': .91, 'box': [.1, .2, .8, .9], 'label': 'Дерево'}


class OverlayTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.cache = NeuralOverlay(lambda: self.now)

    def result(self, captured=None, **changes):
        request = self.cache.issue(JPEG, self.now if captured is None else captured, 'frame')
        return {'token': request['token'], 'image_b64': ENCODED, 'inference_ms': 40.,
                'detections': [DETECTION], **changes}

    def test_fresh_then_expired(self):
        self.assertTrue(self.cache.accept(self.result()))
        self.assertEqual(self.cache.frame(), JPEG)
        self.assertEqual(self.cache.status()['detections'][0]['label'], 'Дерево')
        self.now += 1.01
        self.assertIsNone(self.cache.frame())
        self.assertEqual(self.cache.status()['detections'], [])

    def test_slow_and_unknown_rejected(self):
        result = self.result()
        self.now += 3.01
        self.assertFalse(self.cache.accept(result))
        self.assertFalse(self.cache.accept({**result, 'token': 'unknown'}))
        self.assertIsNone(self.cache.frame())

    def test_age_counted_from_capture_not_upload(self):
        result = self.result(captured=9.2)
        self.assertTrue(self.cache.accept(result))
        self.assertEqual(self.cache.frame(), JPEG)
        self.now += 1.01
        self.assertIsNone(self.cache.frame())

    def test_out_of_order_and_replay(self):
        older = self.result()
        self.now += .1
        newer = self.result()
        self.assertTrue(self.cache.accept(newer))
        self.assertFalse(self.cache.accept(older))
        self.assertFalse(self.cache.accept(newer))

    def test_bounded_requests(self):
        for _ in range(30):
            self.result()
        self.assertEqual(len(self.cache.pending), 8)

    def test_bad_geometry_and_nan(self):
        for row in [{**DETECTION, 'confidence': float('nan')},
                    {**DETECTION, 'box': [.8, .2, .1, .9]},
                    {**DETECTION, 'class_id': 7}]:
            with self.assertRaises(ValueError):
                self.cache.accept(self.result(detections=[row]))

    def test_no_detections_is_valid_live_result(self):
        self.assertTrue(self.cache.accept(self.result(detections=[])))
        self.assertTrue(self.cache.status()['active'])
        self.assertEqual(self.cache.status()['detections'], [])

    def test_qr_neural_class_is_accepted(self):
        qr = {**DETECTION, 'class_id': 3}
        self.assertTrue(self.cache.accept(self.result(detections=[qr])))
        self.assertEqual(self.cache.status()['detections'][0]['label'], 'QR-код')

    def test_stale_input_not_issued(self):
        with self.assertRaises(RuntimeError):
            self.cache.issue(JPEG, 8., 'old')


class HistoryTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / 'history.json'
        self.now = 10.
        self.history = PerceptionHistory(self.path, lambda: self.now)

    def tearDown(self):
        self.history.close()
        self.directory.cleanup()

    def qr(self, text, candidate=False):
        self.history.qr({'text': text, 'candidate': candidate,
                         'imageData': 'data:image/jpeg;base64,' + ENCODED})

    def test_same_object_not_three_screenshots(self):
        for _ in range(40):
            self.history.objects([DETECTION], ENCODED)
            self.now += .3
        self.assertEqual(len(self.history.metadata()['items']), 1)

    def test_objects_returns_only_new_sightings(self):
        first = self.history.objects([DETECTION], ENCODED)
        repeated = self.history.objects([DETECTION], ENCODED)
        self.assertEqual(len(first), 1)
        self.assertEqual(first[0]['label'], 'Дерево')
        self.assertNotIn('image_b64', first[0])
        self.assertEqual(repeated, [])

    def test_reappearance_new_sighting(self):
        self.history.objects([DETECTION], ENCODED)
        self.now += 9.
        self.history.objects([DETECTION], ENCODED)
        self.assertEqual(len(self.history.metadata()['items']), 2)

    def test_two_objects_of_same_class_are_distinct(self):
        left = {**DETECTION, 'box': [.05, .1, .3, .5]}
        right = {**DETECTION, 'box': [.6, .1, .9, .5]}
        for _ in range(5):
            self.history.objects([left, right], ENCODED)
            self.now += .25
        self.assertEqual(len(self.history.metadata()['items']), 2)

    def test_only_decoded_qr_saved(self):
        self.qr('not decoded', candidate=True)
        self.history.qr({'text': ''})
        self.assertEqual(self.history.metadata()['items'], [])
        self.qr('payload')
        self.qr('payload')
        self.assertEqual(len(self.history.metadata()['items']), 1)

    def test_last_three_and_restart(self):
        self.history.objects([DETECTION], ENCODED)
        self.qr('first')
        self.qr('second')
        self.qr('third')
        self.history.close()
        reloaded = PerceptionHistory(self.path)
        try:
            rows = reloaded.metadata()['items']
            self.assertEqual([row['text'] for row in rows], ['third', 'second', 'first'])
            self.assertNotIn('image_b64', rows[0])
            self.assertEqual(reloaded.image(rows[0]['id']), JPEG)
            self.assertEqual(len(json.loads(self.path.read_text())['items']), 3)
        finally:
            reloaded.close()


if __name__ == '__main__':
    unittest.main()
