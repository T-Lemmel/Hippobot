from setuptools import find_packages
from setuptools import setup

setup(
    name='move',
    version='1.0.0',
    packages=find_packages(
        include=('move', 'move.*')),
)
