#!/usr/bin/env python

# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_bt'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_bt']
)

setup(**d)
