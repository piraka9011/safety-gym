#!/usr/bin/env python

import codecs
import os
import re
from setuptools import find_packages, setup
import sys

assert sys.version_info.major == 3 and sys.version_info.minor >= 6, \
    "Safety Gym is designed to work with Python 3.6 and greater. " \
    + "Please install it before proceeding."


def find_version(*file_paths: str) -> str:
    with codecs.open(os.path.join(*file_paths), "r") as fp:
        version_file = fp.read()
    version_match = re.search(r"^__version__ = ['\"]([^'\"]*)['\"]", version_file, re.M)
    if version_match:
        return version_match.group(1)
    raise RuntimeError("Unable to find version string.")


setup(
    name='safety_gym',
    version=find_version("safety_gym", "__init__.py"),
    packages=find_packages(include=['safety_gym']),
    install_requires=[
        'gym>=0.15.3',
        'hydra-core==1.0.5',
        'joblib~=0.14.0',
        'mujoco_py==2.0.2.7',
        'numpy==1.20.0',
        'xmltodict~=0.12.0',
    ],
)
