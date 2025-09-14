import os
from glob import glob
from setuptools import find_packages
from setuptools import setup

package_name = 'rebearm_description'

setup(
    name=package_name,
    version='0.9.1',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'xacro'), glob('xacro/*.xacro')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'visualization_msgs',
        'geometry_msgs',
        'sensor_msgs',
        'tf2_ros',
        'tf2_geometry_msgs',
    ],
    
    zip_safe=True,
    author='ChangWhan Lee',
    author_email='zeta0707@gmail.com',
    maintainer='ChangWhan Lee',
    maintainer_email='zeta0707@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Robot arm description nodes'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'place_cube = rebearm_description.place_cube:main',
        ],
    },
)
