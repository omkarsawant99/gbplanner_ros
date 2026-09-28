# SPDX-License-Identifier: BSD-3-Clause
# Invoked by catkin_python_setup(); do not run directly.
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(**generate_distutils_setup(packages=["autoassess_bridge"], package_dir={"": "src"}))
