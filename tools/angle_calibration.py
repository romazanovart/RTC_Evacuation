#!/usr/bin/env python3
"""Interactive, read-only reference-angle recorder. Never commands motors.

Keep still for the baseline, then use: mark 90, mark 180, mark 90, mark 0,
status, finish, abort. Positive reference angles are counterclockwise.
Manual lifting/sliding is valid for gyro scale only, not wheel calibration.
"""
import argparse
import json
import math
import os
from pathlib import Path
import select
import socket
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def stamp_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9


def yaw(msg):
    q=msg.pose.pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def fit_segments(marks, field):
    pairs=[(b[field]-a[field], b['reference_deg']-a['reference_deg'])
           for a,b in zip(marks,marks[1:])]
    pairs=[(observed,reference) for observed,reference in pairs if abs(reference)>=30]
    if not pairs or sum(observed**2 for observed,_ in pairs)<1:
        return None
    gain=sum(observed*reference for observed,reference in pairs)/sum(observed**2 for observed,_ in pairs)
    return dict(gain=gain, segments=[dict(observed_deg=o,reference_deg=r,
        corrected_error_deg=gain*o-r) for o,r in pairs])


class Recorder(Node):
    def __init__(self, path, reference_start=0.):
        super().__init__('reference_angle_calibration')
        path.parent.mkdir(parents=True,exist_ok=True)
        self.log=path.open('x',encoding='utf-8',buffering=1)
        self.started=time.monotonic()
        self.reference_start=reference_start
        self.wheel_time=0.
        self.last_motion=self.started
        self.wheel_raw=None
        self.wheel_total=0.
        self.wheel_origin=0.
        self.filtered=None
        self.filtered_total=0.
        self.filtered_origin=0.
        self.baseline=[]
        self.bias=None
        self.previous=None
        self.last_imu_time=0.
        self.gyro_angle=0.
        self.bad_gaps=0
        self.marks=[]
        self.done=False
        self.last_event=None
        self.create_subscription(Odometry,'/odom',self.on_wheel,10)
        self.create_subscription(Odometry,'/odometry/filtered',self.on_filtered,10)
        self.create_subscription(Imu,'/imu/mpu6050',self.on_imu,qos_profile_sensor_data)
        self.create_subscription(Twist,'/frob/motor_cmd_vel',self.on_command,10)
        self.emit('started',path=str(path),reference_start=reference_start,
                  boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                  message='Keep stationary for 5 seconds; recorder does not control motors.')

    def emit(self,event,**values):
        record={'event':event,'elapsed_s':time.monotonic()-self.started,**values}
        self.log.write(json.dumps(record,ensure_ascii=False)+'\n')
        # Reference marks must survive a power interruption, not just SSH loss.
        if event in ('baseline_ready','mark','report'):
            self.log.flush()
            os.fsync(self.log.fileno())
        self.last_event=record
        print(json.dumps(record,ensure_ascii=False),flush=True)

    def on_command(self,msg):
        if abs(msg.linear.x)>.001 or abs(msg.angular.z)>.001:
            self.last_motion=time.monotonic()

    def on_wheel(self,msg):
        self.wheel_time=time.monotonic()
        angle=yaw(msg)
        if self.wheel_raw is not None:
            self.wheel_total+=math.atan2(math.sin(angle-self.wheel_raw),math.cos(angle-self.wheel_raw))
        self.wheel_raw=angle
        if abs(msg.twist.twist.linear.x)>.001 or abs(msg.twist.twist.angular.z)>.005:
            self.last_motion=self.wheel_time

    def on_filtered(self,msg):
        angle=yaw(msg)
        if self.filtered is not None:
            self.filtered_total+=math.atan2(math.sin(angle-self.filtered),math.cos(angle-self.filtered))
        self.filtered=angle

    def on_imu(self,msg):
        now=time.monotonic()
        stamp=stamp_seconds(msg)
        rate=float(msg.angular_velocity.z)
        if not math.isfinite(rate):
            self.bad_gaps+=1
            return
        if self.previous is not None and stamp<=self.previous[0]:
            if stamp < self.previous[0]-.15:
                self.bad_gaps+=1
                self.baseline=[]
                self.previous=(stamp,rate)
                self.emit('clock_went_backwards')
            return
        self.last_imu_time=now
        if self.bias is None:
            if self.previous is not None and stamp-self.previous[0]>.15:
                self.baseline=[]
            stationary=(now-self.wheel_time<.3 and now-self.last_motion>1. and abs(rate)<.04)
            if not stationary:
                self.baseline=[]
            else:
                self.baseline.append((stamp,rate))
            if len(self.baseline)>=300 and self.baseline[-1][0]-self.baseline[0][0]>=5.:
                values=[entry[1] for entry in self.baseline]
                noise=statistics.pstdev(values)
                if noise>.004:
                    self.baseline=[]
                    self.emit('baseline_rejected',noise_radps=noise)
                elif self.filtered is not None:
                    self.bias=statistics.fmean(values)
                    self.wheel_origin=self.wheel_total
                    self.filtered_origin=self.filtered_total
                    self.emit('baseline_ready',bias_radps=self.bias,noise_radps=noise,
                              samples=len(values),reference_start=self.reference_start,
                              message='Reference start recorded. Continuous recording active.')
                    self.mark(self.reference_start,baseline=True)
            self.previous=(stamp,rate)
            return
        previous_stamp,previous_rate=self.previous
        dt=stamp-previous_stamp
        if dt>.15:
            self.bad_gaps+=1
            self.emit('sample_gap',seconds=dt)
            self.previous=(stamp,rate)
            return  # Never invent rotation across missing samples/clock steps.
        # Integrate raw gyro against the FIXED measured baseline. Online
        # stationary bias adaptation must not absorb an operator's slow turn.
        self.gyro_angle+=((rate+previous_rate)*.5-self.bias)*dt
        self.previous=(stamp,rate)
        self.log.write(json.dumps(dict(event='sample',stamp=stamp,raw_gyro_radps=rate,
            gyro_deg=math.degrees(self.gyro_angle),wheel_deg=math.degrees(self.wheel_total-self.wheel_origin),
            fused_deg=math.degrees(self.filtered_total-self.filtered_origin)))+'\n')

    def mark(self,reference,baseline=False):
        if self.bias is None:
            raise ValueError('Baseline is not ready')
        if not math.isfinite(reference) or abs(reference)>720:
            raise ValueError('Invalid reference angle')
        now=time.monotonic()
        if not baseline and (now-self.wheel_time>.3 or now-self.last_imu_time>.3 or now-self.last_motion<.6 or
                abs(self.previous[1]-self.bias)>.015):
            raise ValueError('Stop and hold the reference angle before marking')
        mark=dict(reference_deg=reference,gyro_deg=math.degrees(self.gyro_angle),
                  wheel_deg=math.degrees(self.wheel_total-self.wheel_origin),
                  fused_deg=math.degrees(self.filtered_total-self.filtered_origin),
                  elapsed_s=now-self.started,bad_gaps=self.bad_gaps)
        self.marks.append(mark)
        self.emit('mark',**mark)

    def report(self):
        self.emit('report',bias_radps=self.bias,marks=self.marks,bad_gaps=self.bad_gaps,
                  gyro_fit=fit_segments(self.marks,'gyro_deg'),
                  wheel_fit=fit_segments(self.marks,'wheel_deg'),
                  enough_reference_points=len(self.marks)>=5,parameters_applied=False,
                  note='No parameters changed. Wheel fit only valid for powered turns without manual repositioning. Repeat to validate.')

    def command(self,line):
        parts=line.strip().split()
        if not parts: return
        if parts[0]=='mark' and len(parts)==2:
            self.mark(float(parts[1]))
        elif parts[0]=='status':
            self.emit('status',bias_radps=self.bias,gyro_deg=math.degrees(self.gyro_angle),
                      wheel_deg=math.degrees(self.wheel_total-self.wheel_origin),bad_gaps=self.bad_gaps,marks=self.marks)
        elif parts[0]=='finish':
            self.report()
            self.done=True
        elif parts[0]=='abort':
            self.emit('aborted')
            self.done=True
        else:
            raise ValueError('Commands: mark ANGLE, status, finish, abort')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path)
    parser.add_argument('--reference-start',type=float,default=0.)
    parser.add_argument('--socket',type=Path)
    parser.add_argument('--send')
    args=parser.parse_args()
    if args.send is not None:
        if args.socket is None: parser.error('--send requires --socket')
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
            client.settimeout(3.)
            client.connect(str(args.socket))
            client.sendall((args.send+'\n').encode())
            print(client.recv(65536).decode(),flush=True)
        return
    if args.output is None: parser.error('--output required when recording')
    if not math.isfinite(args.reference_start) or abs(args.reference_start)>720:
        parser.error('Invalid reference start')
    server=None
    if args.socket:
        server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        server.bind(str(args.socket))  # Never replace another live recording.
        os.chmod(args.socket,0o600)
        server.listen(2)
        server.setblocking(False)
    rclpy.init()
    recorder=Recorder(args.output,args.reference_start)
    try:
        while rclpy.ok() and not recorder.done:
            if time.monotonic()-recorder.started>1800:
                recorder.emit('recording_timeout')
                recorder.report()
                break
            rclpy.spin_once(recorder,timeout_sec=.02)
            if server and select.select([server],[],[],0)[0]:
                client,_=server.accept()
                with client:
                    client.settimeout(.1)
                    try:
                        line=client.recv(1024).decode().strip()
                        recorder.command(line)
                        client.sendall((json.dumps(recorder.last_event,ensure_ascii=False)+'\n').encode())
                    except (OSError,ValueError) as error:
                        recorder.emit('command_error',message=str(error))
            elif not server and select.select([sys.stdin],[],[],0)[0]:
                line=sys.stdin.readline()
                if not line:
                    recorder.emit('stdin_closed')
                    break
                try: recorder.command(line)
                except ValueError as error: recorder.emit('command_error',message=str(error))
    except KeyboardInterrupt:
        recorder.report()
    finally:
        recorder.log.close()
        recorder.destroy_node()
        rclpy.shutdown()
        if server:
            server.close()
            args.socket.unlink(missing_ok=True)


if __name__=='__main__': main()
