from setuptools import find_packages
from setuptools import setup

setup(
    name='mbf_msgs',
    version='1.0.5',
    packages=find_packages(
        include=('mbf_msgs', 'mbf_msgs.*')),
)
