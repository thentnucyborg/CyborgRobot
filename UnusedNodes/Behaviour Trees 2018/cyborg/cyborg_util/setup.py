#!/usr/bin/env python

# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
        packages=['cyborg_types', 'cyborg_util'],
        package_dir={'': 'src'},
        )

setup(**d)
