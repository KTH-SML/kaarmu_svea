## ! DO NOT INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    scripts=[
        'scripts/aruco_detect.py',
        'scripts/object_detect.py',
        'scripts/object_pose.py',
        'scripts/object_viz.py',
    ],
    packages=[
        'sort',
        'tool',
    ],
    package_dir={'': 'src'},
)

setup(**setup_args)
