#!/usr/bin/env python

from distutils.core import setup

setup(
    name='behavior3py',
    version='0.1.0',
    description='Behavior3 Python library',
    url='https://github.com/behavior3/behavior3py',
    packages=[
        'b3',
        'b3.actions',
        'b3.composites',
        'b3.core',
        'b3.decorators',
    ],
)
