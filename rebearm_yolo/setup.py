import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rebearm_yolo'

setup(
    name=package_name,
    version='0.9.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kalana',
    maintainer_email='kalanaratnayake95@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'yolo_ros = rebearm_yolo.node:main',
                'ncnn_ros = rebearm_yolo.ncnn_node:main',
        ],
    },
)
