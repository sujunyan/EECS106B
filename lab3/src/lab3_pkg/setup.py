"""
Setup of Lab 2 python codebase
Author: Chris Correa
"""
from setuptools import setup

setup(name='lab3',
      version='0.1.0',
      description='EE 106B Turtlebot Lab code',
      author='Chris Correa',
      author_email='chris.correa@berkeley.edu',
      package_dir = {'': 'src'},
      packages=['lab3'],
      install_requires=[],
      test_suite='test'
     )