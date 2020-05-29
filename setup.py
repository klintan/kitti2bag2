#!/usr/bin/env python

from setuptools import setup

setup(
    name='kitti2bag2',
    version='1.0',
    description='Convert KITTI dataset to ROS2 bag file the easy way!',
    author='Tomas Krejci',
    author_email='tomas@krej.ci',
    url='https://github.com/klintan/kitti2bag2/',
    download_url='https://github.com/klintan/kitti2bag2/archive/1.0.zip',
    keywords=['dataset', 'ros2', 'rosbag2', 'kitti'],
    entry_points={
        'console_scripts': ['kitti2bag2=kitti2bag2.kitti2bag2:main'],
    },
    install_requires=['pykitti',
                      'progressbar2',
                      'transforms3d'
                      ]

)
