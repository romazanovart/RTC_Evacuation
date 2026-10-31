import unittest
from pathlib import Path
from unittest.mock import Mock
import math
import cv2
import numpy as np
from rover_web.stand_detector import StandTracker, find_stand
from rover_web.stand_motion import command, SmoothCommand
from rover_web.evacuation import Evacuation


class Tests(unittest.TestCase):
    def test_smooth_commands(self):
        smooth = SmoothCommand()
        linear, angular = smooth.step(.2, .35, .1)
        self.assertAlmostEqual(linear, .02)
        self.assertAlmostEqual(angular, .05)
        for _ in range(30):
            linear, angular = smooth.step(.2, .35, .1)
        self.assertAlmostEqual(linear, .2)
        self.assertAlmostEqual(angular, .35)
        self.assertAlmostEqual(smooth.step(.2, -.35, .1)[1], .3)

    def test_restore_heading_wrap_and_stop(self):
        controller = Evacuation.__new__(Evacuation)
        controller.gateway = Mock()
        controller.cancel = Mock()
        controller.cancel.wait.return_value = False
        controller.event = Mock()
        target = math.radians(179)
        controller._fresh_pose = Mock(side_effect=[{'yaw': math.radians(-170)}] + [{'yaw': target}] * 4)
        controller._restore_qr_heading(target)
        calls = controller.gateway.set_drive_command.call_args_list
        self.assertLess(calls[0].args[2], 0)
        self.assertTrue(all(call.args[0] == 0 for call in calls))
        self.assertEqual(calls[-1].args, (0, 0, 0))

    def test_restore_heading_stale_odometry_stops(self):
        controller = Evacuation.__new__(Evacuation)
        controller.gateway = Mock()
        controller.cancel = Mock()
        controller.cancel.wait.return_value = False
        controller.event = Mock()
        controller._fresh_pose = Mock(side_effect=RuntimeError('stale'))
        with self.assertRaises(RuntimeError):
            controller._restore_qr_heading(0)
        controller.gateway.set_drive_command.assert_called_with(0, 0, 0)

    def test_left_stand_long_legs(self):
        image = cv2.imread(str(Path(__file__).parent / 'fixtures/left-stand-long-legs.png'))[74:774,18:1262]
        points = find_stand(image)
        self.assertIsNotNone(points)
        self.assertLess(points[:,0].mean(), 180)
        self.assertTrue(330 < points[:,1].min() < 355)
        # Keep legs/base/road but remove the printed QR: not a target.
        blank = image.copy()
        blank[330:450,40:180] = 150
        self.assertIsNone(find_stand(blank))

    def test_field_target_and_loss(self):
        image = cv2.imread('/tmp/qr-approach-current.jpg')
        tracker = StandTracker()
        self.assertIsNone(tracker.detect(image))
        self.assertIsNone(tracker.detect(image))
        self.assertIsNotNone(tracker.detect(image))
        self.assertIsNone(tracker.detect(np.zeros_like(image)))
        # White road markings without the placard must not become a target.
        blank = image.copy()
        blank[320:535,490:650] = 125
        self.assertIsNone(find_stand(blank))

    def test_policy(self):
        c = dict(source='white_stand', offset_normalized=[0.,0.], side_ratio=.09)
        self.assertEqual(command(c,.1),(.18,0.))
        self.assertEqual(command(c,1.),(0.,0.))
        self.assertEqual(command(None,0.),(0.,0.))
        c['offset_normalized']=[.5,0.]
        self.assertAlmostEqual(command(c,.1)[0],.117)
        self.assertEqual(command(c,.1)[1],-.325)
        c['offset_normalized']=[-.7,0.]
        self.assertAlmostEqual(command(c,.1)[0],.099)
        self.assertEqual(command(c,.1)[1],.35)
        c['offset_normalized']=[.25,0.]
        self.assertAlmostEqual(command(c,.1)[0],.1485)
        self.assertEqual(command(c,.1)[1],-.1625)
        c['offset_normalized']=[0.,0.]
        c['side_ratio']=.2
        self.assertEqual(command(c,.1),(.04,0.))


if __name__ == '__main__':
    unittest.main()
