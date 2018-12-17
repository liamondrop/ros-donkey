#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['donkey_joy'],
    package_dir={'': 'src'},
    scripts=['bin/donkey_joy_node']
)

setup(**d)
