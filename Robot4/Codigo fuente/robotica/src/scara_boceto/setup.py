import os
from glob import glob
from setuptools import setup

package_name = 'scara_boceto'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),
    (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
    (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
],
entry_points={
    'console_scripts': [
        'gui_sliders = scara_boceto.gui_sliders:main',
        'fk_publisher = scara_boceto.fk_publisher:main',
    ],
    },
)