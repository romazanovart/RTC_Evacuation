from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'frob_sign_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Frob Team',
    maintainer_email='pi@raspberrypi.local',
    description='YOLO traffic sign detector with ROS and MJPEG output.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_driver = frob_sign_detection.camera_node:main',
            'sign_detector = frob_sign_detection.sign_detector:main',
        ],
    },
)
