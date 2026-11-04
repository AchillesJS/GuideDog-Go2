from glob import glob

from setuptools import find_packages, setup

package_name = 'guidedog_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='achilles',
    maintainer_email='achilles@todo.todo',
    description='GuideDog RGB ingestion, simulation bringup and sensor diagnostics',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rgb_node = guidedog_perception.rgb_node:main',
            'sensor_monitor = guidedog_perception.sensor_monitor:main',
            'lidar_stop = guidedog_perception.lidar_stop:main',
            'velocity_watchdog = guidedog_perception.velocity_watchdog:main',
            'guarded_teleop = guidedog_perception.guarded_teleop:main',
        ],
    },
)
