
from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'deteccion_obstaculos_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grupo7',
    maintainer_email='grupo7@example.com',
    description='Nodo de deteccion de obstaculos del robot guia.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deteccion_obstaculos_node = deteccion_obstaculos_pkg.deteccion_obstaculos_node:main',
        ],
    },
)
