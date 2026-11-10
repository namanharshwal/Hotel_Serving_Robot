import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'nammy_camera'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Naman Harshwal',
    maintainer_email='namanharshwal@gmail.com',
    description='Logitech C270 HD Webcam Driver and Vision Streamer for Nammy Hotel Serving Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'c270_camera_node = nammy_camera.c270_camera_node:main',
            'camera_viewer = nammy_camera.camera_viewer:main',
        ],
    },
)
