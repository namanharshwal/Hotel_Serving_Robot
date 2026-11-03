from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nammy_sonar'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Naman Harshwal',
    maintainer_email='namanharshwal@gmail.com',
    description='4-Channel US-100 Ultrasonic Array & Nav2 Costmap Fusion Package',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_fusion_node = nammy_sonar.sonar_fusion_node:main',
        ],
    },
)
