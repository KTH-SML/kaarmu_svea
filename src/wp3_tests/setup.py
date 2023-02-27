## ! DO NOT INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    scripts=[],
    packages=['wp3_tests'],
    package_dir={'': 'src'},
    requires=['rospy'],
)

setup(**setup_args)
