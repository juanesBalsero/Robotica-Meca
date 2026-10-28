from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
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
        ],
    },
)
