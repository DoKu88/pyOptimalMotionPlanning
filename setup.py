#!/usr/bin/env python
import os
from setuptools import setup, find_packages


def read(fname):
    with open(os.path.join(os.path.dirname(__file__), fname)) as f:
        return f.read()


setup(
    name='pyOptimalMotionPlanning',
    packages=[package for package in find_packages()
              if package.startswith('pyOptimalMotionPlanning')],
    include_package_data=True,
    install_requires=read('requirements.txt').strip().splitlines(),
    version='0.0.1',
    description='optimal motion planning implemented in Python',
    author='unknown',
    author_email='unknown',
    url='https://github.com/krishauser/pyOptimalMotionPlanning',
)
