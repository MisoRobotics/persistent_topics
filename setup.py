## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['persistent_topics'],
    package_dir={'': 'src'},
    scripts=['nodes/single_channel_persistent_topics_node', 'nodes/multi_channel_persistent_topics_node'])

setup(**setup_args)
