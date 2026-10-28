import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Incluir archivos de launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Incluir archivos de mundos (.sdf)
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usuario',
    maintainer_email='est.juan.ebalsero@unimilitar.edu.co',
    description='Paquete integrador para multi-robot en Gazebo Harmonic',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)