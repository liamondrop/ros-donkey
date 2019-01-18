#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['donkey_bag_recorder'],
    package_dir={'': 'src'},
    scripts=['bin/donkey_bag_recorder_node']
)

setup(**d)
