#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'cyborg_bt_nodes',
        'cyborg_bt_nodes.actions',
        'cyborg_bt_nodes.decorators',
    ],
    package_dir={'': 'src'},
)

setup(**d)
