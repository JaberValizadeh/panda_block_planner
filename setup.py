## Installs src/panda_block_planner as an importable module.
## Driven by catkin_python_setup() in CMakeLists.txt - do not run directly.

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(
    packages=["panda_block_planner"],
    package_dir={"": "src"},
))
