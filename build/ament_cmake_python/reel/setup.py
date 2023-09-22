from setuptools import find_packages
from setuptools import setup

setup(
    name='reel',
    version='0.0.0',
    packages=find_packages(
        include=('reel', 'reel.*')),
)
