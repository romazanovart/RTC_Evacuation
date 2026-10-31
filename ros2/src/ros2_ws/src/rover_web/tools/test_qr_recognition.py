import unittest
from unittest.mock import patch
from pathlib import Path
import cv2
import numpy as np
from rover_web.qr_geometry import locate_qr
from rover_web.qr_read_state import QRReadState
from rover_web.stand_detector import StandTracker
from rover_web.stand_motion import command


class RecognitionTests(unittest.TestCase):
    def test_cut_stand_finds_whole_code(self):
        image=cv2.imread(str(Path(__file__).parent/'fixtures/close-qr-cut-stand.jpg'))
        tracker=StandTracker()
        self.assertIsNone(tracker.detect(image))
        self.assertIsNone(tracker.detect(image))
        points=tracker.detect(image)
        self.assertEqual(tracker.source,'qr_geometry')
        self.assertIsNotNone(points)
        self.assertGreater(np.ptp(points[:,0]),180)
        self.assertGreater(np.ptp(points[:,1]),180)
        self.assertLess(np.linalg.norm(points.mean(0)-[595,508]),12)
        self.assertIsNone(tracker.detect(np.full_like(image,180)))

    def test_standalone_rotated_code(self):
        code=cv2.QRCodeEncoder_create().encode('QR-without-stand')
        code=cv2.resize(code,(160,160),interpolation=cv2.INTER_NEAREST)
        for rotation in range(4):
            image=np.full((480,640,3),230,np.uint8)
            image[50:210,400:560]=cv2.cvtColor(np.rot90(code,rotation).copy(),cv2.COLOR_GRAY2BGR)
            points=locate_qr(image)
            self.assertIsNotNone(points)
            self.assertLess(np.linalg.norm(points.mean(0)-[480,130]),10)

    def test_finder_shapes_without_data_are_not_qr(self):
        image=np.full((480,640,3),220,np.uint8)
        for x,y in ((80,80),(220,80),(80,220)):
            for inset,color in ((0,0),(7,255),(14,0)):
                cv2.rectangle(image,(x+inset,y+inset),(x+49-inset,y+49-inset),(color,)*3,-1)
        self.assertIsNone(locate_qr(image))

    def test_two_codes_do_not_select_arbitrary_target(self):
        code=cv2.QRCodeEncoder_create().encode('duplicate')
        code=cv2.resize(code,(150,150),interpolation=cv2.INTER_NEAREST)
        image=np.full((480,640,3),220,np.uint8)
        for x in (30,440):
            image[100:250,x:x+150]=cv2.cvtColor(code,cv2.COLOR_GRAY2BGR)
        self.assertIsNone(locate_qr(image))
        with patch('rover_web.stand_detector.find_stand') as fallback:
            self.assertIsNone(StandTracker().detect(image))
            fallback.assert_not_called()

    def test_geometry_supported_by_motion_policy(self):
        v,w=command({'source':'qr_geometry','offset_normalized':[.4,0],'side_ratio':.1},.1)
        self.assertGreater(v,0)
        self.assertLess(w,0)

    def test_decode_is_not_erased_by_search_candidates(self):
        state=QRReadState()
        candidate={'text':'actual payload','candidate':False,'imageData':'test image'}
        state.update(candidate,10.,.25)
        state.update({'text':'','candidate':True},10.1)
        state.update(None,10.2)
        candidate['text']='mutated'
        snap=state.snapshot(10.3)
        self.assertEqual(snap['decoded']['text'],'actual payload')
        self.assertAlmostEqual(snap['decoded_age_sec'],.55)
        self.assertEqual(snap['decoded_sequence'],10.)
        # Search candidates do not refresh the time of old decodes.
        self.assertGreater(state.snapshot(20.)['decoded_age_sec'],10.)
        state.clear()
        self.assertIsNone(state.snapshot(21.)['decoded'])

    def test_stale_and_invalid_decode_rejected(self):
        for age in (3.,-1.,float('nan'),None):
            state=QRReadState();state.update({'candidate':False,'text':'stale'},5.,age)
            self.assertIsNone(state.snapshot(5.)['decoded'])


if __name__=='__main__':unittest.main()
