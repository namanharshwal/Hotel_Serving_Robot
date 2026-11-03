from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nammy_lidar'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Naman Harshwal',
    maintainer_email='namanharshwal@gmail.com',
    description='YDLIDAR G2 360-Degree 2D Laser Scanner Package',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_sim_fallback = nammy_lidar.lidar_sim_fallback:main',
        ],
    },
)
