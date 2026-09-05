#!/usr/bin/env python3

from setuptools import setup


setup(
    name='rqt_rotors',
    version='2.2.3',
    packages=['rqt_rotors'],
    package_dir={'': 'src'},
    scripts=['scripts/hil_plugin'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pavel Vechersky',
    maintainer_email='pavelv@student.ethz.ch',
    description='The rqt_rotors package',
    license='ASL 2.0',
)
