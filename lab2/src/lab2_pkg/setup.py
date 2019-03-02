"""
Setup of Lab 2 python codebase
Author: Chris Correa
"""
from setuptools import setup

requirements = [
    # Our Repos
    'autolab-core',
    
    # Contrib repos
    'trimesh[easy]',

    # External repos
    'numpy',
]

setup(name='toppling',
      version='0.1.0',
      description='EE 106B Grasping Lab project code',
      author='Chris Correa',
      author_email='chris.correa@berkeley.edu',
      package_dir = {'': 'src'},
      packages=['lab2'],
      install_requires=requirements,
      test_suite='test'
     )