"""Synthetic comparison of actual EKF nodes. Use an isolated ROS_DOMAIN_ID.

Publishes only /turn_test sensor topics, never velocity commands. Encoder
scale is deliberately 7.5% high; the gyro reports the reference rotation.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--rate',type=float,default=.45)
    parser.add_argument('--rejection-test',action='store_true')
    options=parser.parse_args()
    rclpy.init()
    node=Node('turn_fusion_regression')
    wheel_pub=node.create_publisher(Odometry, '/turn_test/wheel', 10)
    imu_pub=node.create_publisher(Imu, '/turn_test/imu', 10)
    samples={'before':[], 'after':[]}
    subscriptions=[]
    for name in samples:
        subscriptions.append(node.create_subscription(Odometry, '/turn_test/'+name,
            lambda m, key=name:samples[key].append(math.atan2(2*m.pose.pose.orientation.w*m.pose.pose.orientation.z,
                1-2*m.pose.pose.orientation.z**2)), 10))
    config=Path(__file__).resolve().parents[1]/'frob_odometry/config/ekf.yaml'
    processes=[]
    try:
        for name in samples:
            args=['/opt/ros/jazzy/lib/robot_localization/ekf_node','--ros-args',
                '--params-file',str(config),'-p','publish_tf:=false',
                '-r','/odom:=/turn_test/wheel','-r','/imu/corrected:=/turn_test/imu',
                '-r','/odometry/filtered:=/turn_test/'+name]
            if options.rejection_test:
                threshold=5.0 if name=='before' else 1.e6
                args += ['-p',f'odom0_twist_rejection_threshold:={threshold}',
                         '-p',f'imu0_twist_rejection_threshold:={threshold}']
            elif name=='before':
                args += ['-p','odom0_config:=[true,true,false,false,false,true,true,true,false,false,false,true,false,false,false]']
            processes.append(subprocess.Popen(args,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL))
        start=time.monotonic()
        iteration=0
        actual_angle=math.pi/2
        duration=actual_angle/abs(options.rate)
        integrated_reference=0.
        previous_rate=0.
        previous_stamp=None
        while time.monotonic()-start < 4.+duration+4.:
            elapsed=time.monotonic()-start-4.
            theta=options.rate*min(duration,max(0.,elapsed))
            rate=options.rate if 0<=elapsed<duration else 0.
            stamp=node.get_clock().now().to_msg()
            stamp_s=stamp.sec+stamp.nanosec*1e-9
            if previous_stamp is not None:
                integrated_reference+=(previous_rate+rate)*.5*(stamp_s-previous_stamp)
            previous_rate,previous_stamp=rate,stamp_s
            imu=Imu()
            imu.header.stamp=stamp
            imu.header.frame_id='base_footprint'
            imu.orientation_covariance[0]=-1.
            imu.angular_velocity.z=rate
            imu.angular_velocity_covariance[8]=.0004
            imu_pub.publish(imu)
            if iteration%5==0:
                wheel=Odometry()
                wheel.header.stamp=stamp
                wheel.header.frame_id='odom'
                wheel.child_frame_id='base_footprint'
                wheel.pose.pose.orientation.z=math.sin(theta*1.075/2)
                wheel.pose.pose.orientation.w=math.cos(theta*1.075/2)
                wheel.twist.twist.angular.z=rate*1.075
                for i, value in enumerate((.001,.001,1e6,1e6,1e6,.003)):
                    wheel.pose.covariance[i*7]=value
                for i, value in enumerate((.0004,.001,1e6,1e6,1e6,.003)):
                    wheel.twist.covariance[i*7]=value
                wheel_pub.publish(wheel)
            for _ in range(6): rclpy.spin_once(node,timeout_sec=0.)
            time.sleep(.02)
            iteration+=1
        result={name:{'samples':len(values),'final_deg':math.degrees(values[-1]),
                      'error_deg':math.degrees(math.atan2(math.sin(values[-1]-integrated_reference),
                                                         math.cos(values[-1]-integrated_reference)))}
                for name,values in samples.items() if values}
        print(json.dumps(result,indent=2))
        assert len(result)==2 and all(v['samples']>100 for v in result.values())
        assert abs(result['after']['error_deg'])<1.5
        assert abs(result['before']['error_deg'])>abs(result['after']['error_deg'])+3.
    finally:
        for process in processes:
            process.terminate()
            try: process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__': main()
