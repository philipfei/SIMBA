from glob import glob
from setuptools import find_packages, setup

package_name = 'simba_bringup'
setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='philip',
    maintainer_email='philip@example.com',
    description='Create 3, RPLIDAR, launch, RViz, teleoperation, and velocity ownership.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'keyboard_teleop = simba_bringup.keyboard_teleop:main',
        'tf_relay = simba_bringup.tf_relay:main',
        'velocity_gate = simba_bringup.velocity_gate:main',
        'exploration_coordinator = simba_bringup.exploration_coordinator:main',
    ]},
)
