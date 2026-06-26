from setuptools import find_packages
from setuptools import setup

setup(
    name='openarm_bringup',
    version='1.0.0',
    packages=find_packages(
        include=('openarm_bringup', 'openarm_bringup.*')),
)
