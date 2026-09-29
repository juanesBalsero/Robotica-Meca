import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'antropomorfico_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Carpetas de recursos (launch, urdf, config, worlds)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juanes',
    maintainer_email='est.juan.ebalsero@unimilitar.edu.co',
    description='Robot antopomorfico en linea de produccion',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'ik_solver = antropomorfico_sim.ik_solver:main',
        'trayectoria = antropomorfico_sim.trayectoria:main',
        'trapezoidal_node = antropomorfico_sim.trapezoidal_trajectory_node:main',
        ],
    },
)
