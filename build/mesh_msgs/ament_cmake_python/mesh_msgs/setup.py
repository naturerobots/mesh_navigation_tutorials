from setuptools import find_packages
from setuptools import setup

setup(
    name='mesh_msgs',
    version='2.1.1',
    packages=find_packages(
        include=('mesh_msgs', 'mesh_msgs.*')),
)
