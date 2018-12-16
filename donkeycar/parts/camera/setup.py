#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ros_donkey_camera'],
    package_dir={'': 'src'},
    scripts=['bin/ros_donkey_camera_node']
)

setup(**d)
