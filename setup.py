
#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_joint_trajectory_plot'],
    package_dir={'': 'src'},
    scripts=['script/joint_trajectory_generator.py', 'script/rqt_joint_trajectory_plot']
)

setup(**d)
