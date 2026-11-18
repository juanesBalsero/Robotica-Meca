import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'brain'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='equipo',
    maintainer_email='todo@todo.todo',
    description='Nodo coordinador (brain) de la planta de ensamble de 5 robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'brain_node = brain.brain_node:main',
            'robots_falsos = brain.robots_falsos:main',
        ],
    },
)
