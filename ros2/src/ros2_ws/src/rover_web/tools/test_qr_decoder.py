"""QR pixel/coordinate regressions. No ROS, camera or movement required."""
import time
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from rover_web.qr_worker import Decoder, QRWorker

PAYLOAD = ('FROB-field-test;state=serious;conscious=yes;breathing=difficult;'
           'pulse=118;action=urgent;' + ''.join(str(i % 10) for i in range(180)))


def sample(size=110, blur=0., contrast=1.):
    code = cv2.QRCodeEncoder_create().encode(PAYLOAD)
    code = cv2.resize(code, (size,size), interpolation=cv2.INTER_AREA)
    if blur:
        code = cv2.GaussianBlur(code, (0,0), blur)
    code = np.clip(120+(code.astype(float)-128)*contrast+
                   np.linspace(-30,30,size)[None,:],0,255).astype(np.uint8)
    image = np.full((720,1280,3),160,np.uint8)
    image[313:313+size,853:853+size] = cv2.cvtColor(code,cv2.COLOR_GRAY2BGR)
    points = np.array([[853,313],[853+size,313],[853+size,313+size],[853,313+size]],np.float32)
    return image, points


class DecoderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(1)

    def test_dense_small_low_contrast(self):
        # Native full-frame decoder missed these 110px/81-module samples.
        for contrast in (1., .35):
            with self.subTest(contrast=contrast):
                frame, points = sample(110,.45,contrast)
                decoder = Decoder()
                result = decoder.decode(frame,roi_hint=points)
                self.assertEqual(result[0][0],PAYLOAD)
                self.assertTrue(decoder.last_stats['method'].startswith('candidate-'))
                self.assertTrue(np.all(result[0][1] >= points.min(0)-2))
                self.assertTrue(np.all(result[0][1] <= points.max(0)+2))

    def test_quality_sweep(self):
        passed = 0
        for size in (110,130,150,170,190):
            for blur in (0.,.45,.7):
                for contrast in (1.,.35):
                    frame, points = sample(size,blur,contrast)
                    result = Decoder().decode(frame,roi_hint=points)
                    passed += any(text == PAYLOAD for text,_ in result)
                    self.assertTrue(all(text == PAYLOAD for text,_ in result))
        self.assertGreaterEqual(passed,26)

    def test_bad_hint_still_uses_full_frame(self):
        frame, points = sample(190)
        for hint in (None, [[float('nan'),0]]*4, [1,2], points-500):
            self.assertTrue(any(text==PAYLOAD for text,_ in Decoder().decode(frame,roi_hint=hint)))

    def test_blank_never_reuses_decoded_text(self):
        frame, points = sample(190)
        decoder = Decoder()
        self.assertTrue(decoder.decode(frame,roi_hint=points))
        blank = np.full_like(frame,180)
        for _ in range(4):
            self.assertEqual(decoder.decode(blank,roi_hint=points),[])

    def test_expired_budget_does_no_decode(self):
        frame, points = sample()
        decoder = Decoder()
        self.assertEqual(decoder.decode(frame,budget=0,roi_hint=points),[])
        self.assertEqual(decoder.last_stats['attempts'],0)

    def test_worker_roi_transport(self):
        frame, points = sample()
        worker = QRWorker(timeout=3.)
        try:
            self.assertTrue(worker.submit(frame,roi_hint=points))
            deadline = time.monotonic()+6
            result = None
            while time.monotonic()<deadline and result is None:
                time.sleep(.03)
                result=worker.poll()
            self.assertIsNotNone(result)
            self.assertIsNone(result[1])
            self.assertTrue(any(text==PAYLOAD for text,_ in result[0]))
        finally:
            worker.close()

    def test_perspective_points_map_back_to_camera(self):
        code=cv2.QRCodeEncoder_create().encode('FROB-perspective-test')
        code=cv2.resize(code,(250,250),interpolation=cv2.INTER_NEAREST)
        src=np.array([[0,0],[249,0],[249,249],[0,249]],np.float32)
        dst=np.array([[700,220],[950,260],[930,520],[730,490]],np.float32)
        matrix=cv2.getPerspectiveTransform(src,dst)
        gray=cv2.warpPerspective(code,matrix,(1280,720),borderValue=255)
        decoder=Decoder()
        read=decoder.zx.read_barcodes
        calls=0
        def force_perspective(*args,**kwargs):
            nonlocal calls
            calls+=1
            return [] if calls<=4 else read(*args,**kwargs)
        with patch.object(decoder.zx,'read_barcodes',side_effect=force_perspective):
            result=decoder.decode(cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR),roi_hint=dst)
        self.assertEqual(decoder.last_stats['method'],'perspective')
        self.assertEqual(result[0][0],'FROB-perspective-test')
        for point in result[0][1]:
            self.assertGreaterEqual(cv2.pointPolygonTest(dst,tuple(map(float,point)),True),-3.)


if __name__=='__main__':
    unittest.main()
