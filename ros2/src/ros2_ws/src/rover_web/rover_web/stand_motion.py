"""Conservative field-test policy; returns body forward/yaw velocities."""
import math


class SmoothCommand:
    """Rate-limit normal commands; emergency stops must bypass this filter."""
    def __init__(self):
        self.linear = self.angular = 0.

    def step(self, linear, angular, dt):
        dt = max(0., min(.15, dt))
        self.linear += max(-.4*dt, min(.2*dt, linear-self.linear))
        self.angular += max(-.5*dt, min(.5*dt, angular-self.angular))
        return self.linear, self.angular


def command(candidate, age):
    if not candidate or age is None or age > .6 or candidate.get('source') not in ('white_stand', 'qr_geometry'):
        return 0., 0.
    offset = float(candidate['offset_normalized'][0])
    ratio = float(candidate.get('side_ratio', 0))
    if not math.isfinite(offset) or not .015 < ratio < .28:
        return 0., 0.
    # Keep a small forward component while steering. Previously a large image
    # offset reduced forward motion to almost zero, so the rover mostly turned
    # in place instead of closing the distance to the QR stand.
    linear = .18 if ratio < .10 else (.07 if ratio < .18 else .04)
    linear *= max(.55, 1. - .7 * abs(offset))
    # Image x grows right; ROS positive yaw turns left. Steer while advancing.
    angular = 0. if abs(offset) < .06 else max(-.35, min(.35, -.65 * offset))
    return linear, angular
