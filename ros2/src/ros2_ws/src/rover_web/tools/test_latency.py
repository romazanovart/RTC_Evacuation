import ast
import math
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import cv2
import numpy as np
from rover_web.camera_preview import encode_preview
from rover_web.drive_sequence import DriveSequence, DriveLease, legacy_browser_control


def gateway_method(name):
    """Exercise the actual method without requiring ROS installed on the laptop."""
    path = Path(__file__).parents[1] / 'rover_web/web_gateway_node.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RoverWebGateway')
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace = {'Any': object, 'math': math}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[name]


class LatencyTests(unittest.TestCase):
    def test_only_pre_fallback_browser_requires_reload(self):
        old = {'client_id':'old-ui','sequence':1}
        new = {**old, 'http_ticket':'signed'}
        self.assertTrue(legacy_browser_control(old, 'Mozilla/5.0'))
        self.assertFalse(legacy_browser_control(new, 'Mozilla/5.0'))
        self.assertFalse(legacy_browser_control(old, 'Python-urllib/3.12'))

    def test_http_ticket_expiry_and_tampering(self):
        now = [1_000_000_000]
        gate = DriveLease(clock=lambda:now[0])
        ticket = gate.issue()
        self.assertTrue(gate.valid(ticket))
        for invalid in [None, {}, True, 'garbage', '0.abc', ticket+'x', ticket.replace('1', '2'), '1.'+'я'*64]:
            self.assertFalse(gate.valid(invalid))
        self.assertFalse(DriveLease(clock=lambda:now[0]).valid(ticket))
        now[0] += 250_000_001
        self.assertFalse(gate.valid(ticket))
        now[0] = 0
        self.assertFalse(gate.valid(ticket))

    def test_gateway_expired_http_does_not_refresh_watchdog(self):
        now = [1_000_000_000]
        gateway = SimpleNamespace(_lock=threading.RLock(), _drive_sequence=DriveSequence(),
            _drive_lease=DriveLease(clock=lambda:now[0]), drive_command_timeout_sec=.25,
            set_drive_command=Mock(return_value={'ok': True}))
        method = gateway_method('set_web_drive_command')
        ticket = gateway._drive_lease.issue()
        command = {'client_id':'tab','sequence':1,'http_ticket':ticket,'linear_x':.2}
        self.assertEqual(method(gateway, command), {'ok':True})
        now[0] += 300_000_000
        command['sequence'] = 2
        self.assertEqual(method(gateway, command), {'ok':True,'ignored':'expired'})
        gateway.set_drive_command.assert_called_once_with(.2,0.,0.,timeout_sec=.55)
        method(gateway, command, stop=True)
        self.assertEqual(gateway.set_drive_command.call_count, 2)

    def test_sequence_stop_barrier(self):
        gate = DriveSequence()
        self.assertTrue(gate.accept('tab', 1))
        self.assertTrue(gate.accept('tab', 3))  # STOP overtakes old motion.
        self.assertFalse(gate.accept('tab', 2))
        self.assertFalse(gate.accept('tab', 3))
        self.assertTrue(gate.accept('tab', 4))
        self.assertTrue(gate.accept('new-tab', 1))
        self.assertTrue(gate.accept(None, None))

    def test_invalid_sequences_and_bounded_sessions(self):
        gate = DriveSequence(limit=1)
        for args in [('a', True), ('a', 1.5), ('a', -1), ('a', 2**53), ('', 2), ('a', None)]:
            with self.assertRaises(ValueError): gate.accept(*args)
        gate.accept('a', 1)
        with self.assertRaises(ValueError): gate.accept('b', 1)
        self.assertFalse(gate.accept('a', 0))

    def test_gateway_stop_rejects_late_command(self):
        gateway = SimpleNamespace(_lock=threading.RLock(), _drive_sequence=DriveSequence(),
                                  drive_command_timeout_sec=.25,
                                  set_drive_command=Mock(return_value={'ok': True}))
        method = gateway_method('set_web_drive_command')
        method(gateway, {'client_id': 'tab', 'sequence': 3}, stop=True)
        gateway.set_drive_command.assert_called_once_with(0., 0., 0., timeout_sec=None)
        self.assertEqual(method(gateway, {'client_id': 'tab', 'sequence': 2, 'linear_x': .2}),
                         {'ok': True, 'ignored': 'superseded'})
        gateway.set_drive_command.assert_called_once()
        for value in (float('nan'), float('inf')):
            with self.assertRaises(ValueError): method(gateway, {'linear_x': value})

    def test_preview_is_smaller_without_mutating_full_frame(self):
        frame = cv2.imread(str(Path(__file__).parent/'fixtures/close-qr-cut-stand.jpg'))
        before = frame.copy()
        data = encode_preview(frame)
        decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape[1], 640)
        self.assertAlmostEqual(decoded.shape[0]/640, frame.shape[0]/frame.shape[1], places=2)
        ok, full = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
        self.assertTrue(ok)
        self.assertLess(len(data), full.size*.45)
        np.testing.assert_array_equal(frame, before)

    def test_small_preview_not_upscaled(self):
        frame = np.full((100, 200, 3), 120, np.uint8)
        decoded = cv2.imdecode(np.frombuffer(encode_preview(frame), np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape, frame.shape)


if __name__ == '__main__': unittest.main()
