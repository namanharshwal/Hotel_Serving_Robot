from setuptools import setup

package_name = 'nammy_power'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Naman Harshwal',
    maintainer_email='namanharshwal@gmail.com',
    description='24V LiFePO4 Power and Battery Subsystem Monitor for Nammy Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_monitor_node = nammy_power.battery_monitor_node:main',
        ],
    },
)
