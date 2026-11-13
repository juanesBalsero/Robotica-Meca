import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'data'), glob('data/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_nombre',
    maintainer_email='tu@correo.com',
    description='CC, puente UDP con la ESP32 y publicador de trayectoria',
    license='MIT',
    entry_points={
        'console_scripts': [
            'udp_bridge = robot_control.udp_bridge:main',
            'kinematic_control = robot_control.kinematic_control:main',
            'trajectory_publisher = robot_control.trajectory_publisher:main',
        ],
    },
)
